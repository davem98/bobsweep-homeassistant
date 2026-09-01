"""DataUpdateCoordinator for the bObsweep (Tuya local protocol) integration.

Wraps a local `tinytuya.Device` connection. tinytuya is a blocking library —
every call that touches the socket runs via `hass.async_add_executor_job`.

The coordinator also owns the room-awareness state for the entry: the persisted
zone set (`ZoneStore`), the position source (`PositionSource` — today always the
null one, see `position.py`) and the `RoomTracker` that turns fixes into a room.
They live here rather than on an entity because the capture services, the
`current_room` sensor and the vacuum segment API all need the same instance, and
because room tracking must keep running whether or not any of those exist.
"""

from __future__ import annotations

import logging
from datetime import timedelta
from typing import Any

import tinytuya
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator, UpdateFailed

from .const import (
    CONF_AI_OBJECT_POSITION,
    CONF_DEVICE_ID,
    CONF_HOST,
    CONF_LOCAL_KEY,
    CONF_MODEL_FAMILY,
    CONF_PROTOCOL_VERSION,
    DEFAULT_AI_OBJECT_POSITION,
    DEFAULT_MODEL_FAMILY,
    DEFAULT_PROTOCOL_VERSION,
    DOMAIN,
    FamilySpec,
    resolve_family,
)
from .position import PositionSource, create_position_source
from .rooms import RoomTracker
from .transport import AiObjectTracker
from .zones import ZoneStore

_LOGGER = logging.getLogger(__name__)

UPDATE_INTERVAL_SECONDS = 15


class BobsweepCoordinator(DataUpdateCoordinator[dict[str, Any]]):
    """Coordinates polling of a bObsweep robot over the local Tuya protocol."""

    def __init__(self, hass: HomeAssistant, entry: ConfigEntry) -> None:
        """Set up the coordinator and the underlying tinytuya device."""
        super().__init__(
            hass,
            _LOGGER,
            name=DOMAIN,
            update_interval=timedelta(seconds=UPDATE_INTERVAL_SECONDS),
        )

        self.entry = entry

        data = entry.data
        self.device_id: str = data[CONF_DEVICE_ID]
        self.host: str = data[CONF_HOST]
        self.model_family: str = data.get(CONF_MODEL_FAMILY, DEFAULT_MODEL_FAMILY)
        # The DP table every platform reads its ids and vocabularies from.
        # Unknown/missing values resolve to SLAM so old entries keep working.
        self.spec: FamilySpec = resolve_family(self.model_family)

        local_key: str = data[CONF_LOCAL_KEY]
        protocol_version: str = data.get(
            CONF_PROTOCOL_VERSION, DEFAULT_PROTOCOL_VERSION
        )

        self.device = tinytuya.Device(
            self.device_id,
            self.host,
            local_key,
            version=float(protocol_version),
        )
        self.device.set_socketPersistent(True)

        # --- room awareness -------------------------------------------------
        # Zones are persisted per config entry in `.storage/bobsweep_zones.*`,
        # NOT in entry options: options changes reload the entry, which would
        # tear down an in-flight zone capture. `async_setup()` loads them.
        self.zone_store = ZoneStore(hass, entry.entry_id)
        # Accumulates the robot's AI obstacle reports off DP 105. Owned here
        # rather than by either consumer because both the obstacle sensor and
        # the optional position source need the *same* instance, and because it
        # has to be fed from the poll loop: the merged DP snapshot keeps only
        # the newest DP 105 value, which is often some other kind of frame.
        self.ai_objects: AiObjectTracker | None = (
            AiObjectTracker() if self.spec.dp_transportation is not None else None
        )
        # Opt-in, default off -- a coarse position source feeding room
        # classification can name rooms confidently and wrongly. See position.py.
        self.ai_object_position: bool = bool(
            entry.options.get(CONF_AI_OBJECT_POSITION, DEFAULT_AI_OBJECT_POSITION)
        )
        # The one swap point for "where is the robot". Returns a null source
        # unless the user has opted into the approximate AI-object source.
        self.position_source: PositionSource = create_position_source(
            self.spec,
            self.async_set_dp,
            ai_object_tracker=self.ai_objects,
            ai_object_position=self.ai_object_position,
        )
        self.rooms = RoomTracker(
            hass,
            zone_store=self.zone_store,
            position_source=self.position_source,
            status_dp=self.spec.dp_status,
            entry_id=entry.entry_id,
            device_id=self.device_id,
        )

        # Over a persistent socket the robot pushes PARTIAL updates (often just
        # {"6": <battery>}). Accumulate every datapoint ever seen so a partial
        # push doesn't blank out the rest of the state.
        self._dps: dict[str, Any] = {}

    async def async_setup(self) -> None:
        """Load persisted state that must exist before the first refresh."""
        await self.zone_store.async_load()

    async def _async_update_data(self) -> dict[str, Any]:
        """Poll the device for its current datapoint values."""
        try:
            result = await self.hass.async_add_executor_job(self.device.status)
        except Exception as err:  # noqa: BLE001 - tinytuya raises bare Exception
            raise UpdateFailed(f"Error communicating with bObsweep: {err}") from err

        if not isinstance(result, dict):
            raise UpdateFailed(f"Unexpected response from bObsweep: {result!r}")

        if "Error" in result:
            raise UpdateFailed(
                f"bObsweep reported an error: {result.get('Error')} "
                f"({result.get('Err')})"
            )

        dps = result.get("dps")
        if isinstance(dps, dict) and dps:
            self._dps.update(dps)
        elif not self._dps:
            # first poll returned no usable datapoints
            raise UpdateFailed(f"No 'dps' in bObsweep response: {result!r}")

        # Feed the obstacle tracker *before* room tracking runs: the position
        # source reads new sightings out of the tracker, so the order matters.
        # Only values from this poll are offered -- re-ingesting the merged
        # snapshot every poll would re-present the same frame indefinitely.
        if self.ai_objects is not None and isinstance(dps, dict):
            value = dps.get(self.spec.dp_transportation)
            if value is not None:
                self.ai_objects.ingest(value)

        snapshot = dict(self._dps)
        # Room tracking is strictly downstream of polling and never raises, so a
        # position source that misbehaves can't stop the robot's normal state
        # from updating.
        await self.rooms.async_update(snapshot)
        return snapshot

    async def async_set_dp(self, dp: str, value: Any) -> None:
        """Set a single datapoint on the device and refresh state."""
        try:
            await self.hass.async_add_executor_job(
                lambda: self.device.set_value(dp, value, nowait=False)
            )
        except Exception as err:  # noqa: BLE001 - tinytuya raises bare Exception
            raise UpdateFailed(f"Error sending command to bObsweep: {err}") from err

        await self.async_request_refresh()

    async def async_set_dps(self, dps: dict[str, Any]) -> None:
        """Set multiple datapoints on the device and refresh state."""
        try:
            if hasattr(self.device, "set_multiple_values"):
                await self.hass.async_add_executor_job(
                    lambda: self.device.set_multiple_values(dps, nowait=False)
                )
            else:
                for dp, value in dps.items():
                    await self.hass.async_add_executor_job(
                        lambda dp=dp, value=value: self.device.set_value(
                            dp, value, nowait=False
                        )
                    )
        except Exception as err:  # noqa: BLE001 - tinytuya raises bare Exception
            raise UpdateFailed(f"Error sending commands to bObsweep: {err}") from err

        await self.async_request_refresh()
