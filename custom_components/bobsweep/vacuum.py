"""Vacuum platform for the bObsweep (local Tuya) integration.

The entity is built against the config entry's `FamilySpec` (see `const.py`)
rather than a fixed DP block, because the three bObsweep DP families disagree on
datapoint ids, on the enum strings for the same concept, and on which
capabilities exist at all. Two rules follow from that:

* a `VacuumEntityFeature` is only advertised when the family can actually
  perform it (no LOCATE for Vision/Random, which have no COMMAND_SEEK_ROBOT), and
* a custom service that needs an absent datapoint raises
  `ServiceValidationError` instead of writing to a guessed id.
"""

from __future__ import annotations

import logging
from typing import Any

import voluptuous as vol
from homeassistant.components.vacuum import (
    StateVacuumEntity,
    VacuumActivity,
    VacuumEntityFeature,
)
from homeassistant.core import HomeAssistant, callback
from homeassistant.exceptions import HomeAssistantError, ServiceValidationError
from homeassistant.helpers import config_validation as cv, entity_platform
from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from . import BobsweepConfigEntry
from .const import DEFAULT_NAME, DOMAIN, FAMILIES
from .coordinator import BobsweepCoordinator
from .faults import decode_faults
from .zone_services import async_register_zone_services
from .zones import ZoneError, resolve_segment_boxes, segment_specs

_LOGGER = logging.getLogger(__name__)

# --- vacuum segment API, feature-detected ------------------------------------
# Home Assistant 2026.3 introduced a first-class vacuum segment/area system:
# `VacuumEntityFeature.CLEAN_AREA`, a `Segment(id, name, group)` dataclass,
# `async_get_segments()` / `async_clean_segments()`, a user-owned segment -> HA
# area mapping in entity-registry options, and a `HassVacuumCleanArea` voice
# intent for free.
#
# `hacs.json` declares a minimum HA of 2024.8.0 and this does not raise it. Both
# halves are probed at import time and the whole feature is simply absent on an
# older core -- no hard import, no version string comparison (which would be
# wrong the moment a feature is backported), and nothing else in the integration
# changes behaviour. On an HA that has it, taught zones become segments; on one
# that does not, everything else works exactly as before.
try:  # HA >= 2026.3
    from homeassistant.components.vacuum import Segment  # type: ignore[attr-defined]
except ImportError:  # pragma: no cover - depends on the running HA version
    Segment = None  # type: ignore[assignment]

SEGMENTS_SUPPORTED = Segment is not None and hasattr(
    VacuumEntityFeature, "CLEAN_AREA"
)

# --- custom service names ----------------------------------------------------
SERVICE_SET_MODE = "set_mode"
SERVICE_EMPTY_DUSTBIN = "empty_dustbin"
SERVICE_SET_DP = "set_dp"


def coerce_dp_value(value: Any) -> Any:
    """Turn a service-call value into the type the datapoint expects.

    Tuya datapoints are typed on the wire: a bool DP given the *string* "True"
    is silently ignored by the robot. The previous schema (`vol.Any(cv.string,
    ...)`) stringified every value first, which is how `set_dp 102 true` came
    to do nothing. Keep native bools and ints; map the textual forms a YAML or
    UI call produces onto them; leave everything else a string.
    """
    if isinstance(value, bool) or (isinstance(value, int) and not isinstance(value, bool)):
        return value
    if isinstance(value, float):
        return int(value) if value.is_integer() else value
    text = str(value).strip()
    lowered = text.lower()
    if lowered in ("true", "on", "yes"):
        return True
    if lowered in ("false", "off", "no"):
        return False
    if lowered.lstrip("-").isdigit():
        return int(lowered)
    return text

# Entity services are registered once per platform, so the `set_mode` schema has
# to accept any family's work-mode vocabulary. The entity then rejects a value
# its own family doesn't define — that check is the family-specific one.
_ALL_WORK_MODE_VALUES = sorted(
    {value for spec in FAMILIES.values() for value in spec.work_mode.values()}
)


class BobsweepVacuum(CoordinatorEntity[BobsweepCoordinator], StateVacuumEntity):
    """A bObsweep robot vacuum controlled over the local Tuya protocol."""

    _attr_has_entity_name = True
    _attr_name = None  # single primary entity -> use the device name

    def __init__(self, coordinator: BobsweepCoordinator) -> None:
        """Initialize the vacuum entity for the configured model family."""
        super().__init__(coordinator)
        self._spec = coordinator.spec
        self._attr_unique_id = coordinator.device_id
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, coordinator.device_id)},
            manufacturer="bObsweep",
            model=coordinator.model_family,
            name=DEFAULT_NAME,
        )

        # Every family can start/stop/pause, go home, report state and take a
        # raw pass-through command; the rest is capability-gated on the DP map.
        features = (
            VacuumEntityFeature.START
            | VacuumEntityFeature.PAUSE
            | VacuumEntityFeature.STOP
            | VacuumEntityFeature.RETURN_HOME
            | VacuumEntityFeature.STATE
            | VacuumEntityFeature.SEND_COMMAND
        )
        if self._spec.dp_fan is not None:
            features |= VacuumEntityFeature.FAN_SPEED
        if self._spec.dp_locate is not None:
            features |= VacuumEntityFeature.LOCATE
        if self._spec.mode_spot is not None:
            features |= VacuumEntityFeature.CLEAN_SPOT
        if SEGMENTS_SUPPORTED:
            # Advertised on capability, not on whether any zone has been taught
            # yet: `supported_features` is fixed at entity construction, and a
            # zone taught five minutes from now must not require a reload to
            # become a segment. With no zones, `async_get_segments()` simply
            # returns an empty list.
            features |= VacuumEntityFeature.CLEAN_AREA
        self._attr_supported_features = features

        self._attr_fan_speed_list = list(self._spec.fan_speeds)
        # The full fan vocabulary, including any value (SLAM's "closed") that is
        # not user-selectable but can still be reported by the device.
        self._fan_enum_values = set(self._spec.fan_speed.values())

    # --- helpers -------------------------------------------------------------
    def _dp(self, dp: str | None) -> Any:
        """Return the current value of a datapoint, or None if unavailable."""
        if dp is None:
            return None
        data = self.coordinator.data or {}
        return data.get(dp)

    def _has_error(self) -> bool:
        """Return True if any error DP reports a real (non-note) fault.

        Decoding by name (see `faults.py`) keeps the app's two informational
        notes and Random's `Normal` bit from putting the vacuum into ERROR; a
        non-empty value that can't be decoded still does, as before.
        """
        if decode_faults(self._spec, self.coordinator.data).has_fault:
            return True
        # Random signals faults through its status enum ('in_trouble') as well.
        return self._dp(self._spec.dp_status) in self._spec.status_error

    def _mode_value(self, name: str) -> str:
        """Return the family's Tuya enum string for a work-mode NAME key."""
        return self._spec.work_mode[name]

    # --- state ---------------------------------------------------------------
    @property
    def activity(self) -> VacuumActivity:
        """Map the raw status DP enum onto the HA vacuum activity."""
        if self._has_error():
            return VacuumActivity.ERROR

        status = self._dp(self._spec.dp_status)
        if status in self._spec.status_cleaning:
            return VacuumActivity.CLEANING
        if status in self._spec.status_returning:
            return VacuumActivity.RETURNING
        if status in self._spec.status_docked:
            return VacuumActivity.DOCKED
        if status in self._spec.status_paused:
            return VacuumActivity.PAUSED
        return VacuumActivity.IDLE

    @property
    def fan_speed(self) -> str | None:
        """Return the current fan speed as an HA-facing value."""
        value = self._dp(self._spec.dp_fan)
        if value is None:
            return None
        # The fan DP already carries the Tuya enum string (e.g. "strong");
        # surface it directly when it is one of the known enum values.
        if value in self._fan_enum_values:
            return value
        return None

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        """Expose maintenance counters and diagnostic status fields.

        Only attributes the family actually has a datapoint for are included, so
        a Vision robot doesn't advertise a filter-life reading it never sends.
        """
        spec = self._spec
        candidates: tuple[tuple[str, str | None], ...] = (
            ("clean_area", spec.dp_clean_area),
            ("clean_time", spec.dp_clean_time),
            ("side_brush_life", spec.dp_side_brush_life),
            ("roll_brush_life", spec.dp_roll_brush_life),
            ("filter_life", spec.dp_filter_life),
            ("mop_status", spec.dp_status_mop),
            ("vacuum_status", spec.dp_status_vacuum),
            ("status", spec.dp_status),
            ("error", spec.dp_error),
            # family-specific extras (all None on SLAM)
            ("water_control", spec.dp_water_control),
            ("dustbin_watertank_status", spec.dp_status_dustbin_watertank),
            ("muted", spec.dp_mute),
            ("cliff_sensor", spec.dp_cliff_sensor),
        )
        attrs = {name: self._dp(dp) for name, dp in candidates if dp is not None}
        # Additive: the raw `error` value above is unchanged, these name it.
        report = decode_faults(spec, self.coordinator.data)
        attrs["fault"] = report.state
        attrs["faults"] = list(report.faults)
        attrs["notes"] = list(report.notes)
        return attrs

    # --- standard vacuum commands -------------------------------------------
    async def async_start(self) -> None:
        """Start (or resume) the family's normal full-coverage clean."""
        # Ensure the enable switch is on, then request the family's auto mode
        # (SLAM/Vision: "smart"; Random has no smart mode, its run is "random").
        mode = self._mode_value(self._spec.mode_auto)
        if self._spec.dp_power is None:
            await self.coordinator.async_set_dp(self._spec.dp_mode, mode)
            return
        await self.coordinator.async_set_dps(
            {self._spec.dp_power: True, self._spec.dp_mode: mode}
        )

    async def async_pause(self) -> None:
        """Pause the current job.

        SLAM has a dedicated pause datapoint (COMMAND_PAUSE, the app's
        `pauseRobot()`); families without one fall back to clearing
        COMMAND_ENABLE, which ends the run rather than suspending it.
        """
        if self._spec.dp_pause is not None:
            await self.coordinator.async_set_dp(self._spec.dp_pause, True)
            return
        if self._spec.dp_power is None:
            raise ServiceValidationError(
                f"The {self._spec.key} bObsweep family has no enable datapoint to pause with"
            )
        await self.coordinator.async_set_dp(self._spec.dp_power, False)

    async def async_stop(self, **kwargs: Any) -> None:
        """Stop the vacuum (end the cleaning run, or cancel a return to dock).

        Mirrors the app's `sendStop()`: while the robot is returning to the
        dock the stop is START_STOP_DOCKING=false; while it is cleaning the
        stop is ENABLE=false. Vision and Random instead have an explicit
        standby work mode.
        """
        if self._spec.mode_stop is not None:
            await self.coordinator.async_set_dp(
                self._spec.dp_mode, self._mode_value(self._spec.mode_stop)
            )
            return
        if (
            self._spec.dp_docking is not None
            and self._dp(self._spec.dp_status) in self._spec.status_returning
        ):
            await self.coordinator.async_set_dp(self._spec.dp_docking, False)
            return
        await self.coordinator.async_set_dp(self._spec.dp_power, False)

    async def async_return_to_base(self, **kwargs: Any) -> None:
        """Send the vacuum back to its dock.

        On SLAM this is START_STOP_DOCKING=true, exactly what the app's
        `startDocking()` writes; it works mid-clean. Writing the `chargego`
        work mode alone does not redirect a running job (verified on hardware
        2026-09-05: the robot kept cleaning), so that is only the fallback for
        families without the docking datapoint.
        """
        if self._spec.dp_docking is not None:
            await self.coordinator.async_set_dp(self._spec.dp_docking, True)
            return
        await self.coordinator.async_set_dp(
            self._spec.dp_mode, self._mode_value(self._spec.mode_charge)
        )

    async def async_set_fan_speed(self, fan_speed: str, **kwargs: Any) -> None:
        """Set the cleaning (fan) power using this family's vocabulary."""
        if fan_speed in self._fan_enum_values:
            enum_value = fan_speed
        elif fan_speed in self._spec.fan_speed:
            # Accept a NAME key (e.g. "STRONG") as well as the enum value.
            enum_value = self._spec.fan_speed[fan_speed]
        else:
            _LOGGER.warning(
                "Unknown fan speed %r for bObsweep family %s",
                fan_speed,
                self._spec.key,
            )
            return
        await self.coordinator.async_set_dp(self._spec.dp_fan, enum_value)

    async def async_locate(self, **kwargs: Any) -> None:
        """Locate the vacuum by playing a sound."""
        # LOCATE is not advertised without the DP, so this is belt-and-braces.
        if self._spec.dp_locate is None:
            raise ServiceValidationError(
                f"The {self._spec.key} bObsweep family has no locate datapoint"
            )
        await self.coordinator.async_set_dp(self._spec.dp_locate, True)

    async def async_clean_spot(self, **kwargs: Any) -> None:
        """Perform a spot/partial clean."""
        if self._spec.mode_spot is None:
            raise ServiceValidationError(
                f"The {self._spec.key} bObsweep family has no spot-clean mode"
            )
        await self.coordinator.async_set_dp(
            self._spec.dp_mode, self._mode_value(self._spec.mode_spot)
        )

    async def async_send_command(
        self,
        command: str,
        params: dict[str, Any] | None = None,
        **kwargs: Any,
    ) -> None:
        """Pass-through command interface for advanced/debug control."""
        params = params or {}
        if command == "set_dp":
            dp = params.get("dp")
            if dp is None or "value" not in params:
                _LOGGER.warning(
                    "send_command 'set_dp' requires 'dp' and 'value' params: %s",
                    params,
                )
                return
            await self.coordinator.async_set_dp(str(dp), params["value"])
        elif command == "set_mode":
            mode = params.get("mode")
            if mode is None:
                _LOGGER.warning("send_command 'set_mode' requires a 'mode' param")
                return
            await self.coordinator.async_set_dp(self._spec.dp_mode, mode)
        else:
            _LOGGER.warning("Unsupported bObsweep send_command: %r", command)

    # --- segments (HA 2026.3+) ----------------------------------------------
    async def async_get_segments(self) -> list[Any]:
        """Expose every taught zone as a cleanable segment.

        Segment id is the zone's stable uuid — the user maps segments onto HA
        areas in the entity registry and that mapping is keyed by id, so it has
        to survive a rename. `group` is the map id, which is how a multi-floor
        home is namespaced; it is None until something is verified to report one.
        """
        if not SEGMENTS_SUPPORTED:
            return []
        return [
            Segment(id=spec["id"], name=spec["name"], group=spec["group"])
            for spec in segment_specs(self.coordinator.rooms.zones)
        ]

    async def async_clean_segments(self, segment_ids: list[str], **kwargs: Any) -> None:
        """Clean the named segments. **Not implemented — no verified send path.**

        The resolution half is real and runs first, so a bad segment id fails on
        the id rather than on the transport, and so this code path is exercised
        the moment sending becomes possible: ids -> zones -> bounding boxes in
        raw map-cell units (rectangles, because the robot's zone command takes
        rectangles, not polygons).

        The send half is deliberately absent. The intended mechanism is a DP 105
        (`COMMAND_TRANSPORTATION`) `eDesignated:16` write carrying the corner
        pairs as signed int16 in the 0xAA frame format
        (`[AA][len][cmd][data...][chk]`, base64 on the wire), followed by a
        work-mode write of `zone`. Frame format and checksum are confirmed
        against ten captured frames, but that is confirmation of *decoding*
        frames the robot sent — no write to DP 105 has ever been probed. Sending
        a speculative frame to a robot that is mapping a real house is not a
        thing to guess at: a malformed transportation payload is the same channel
        that carries map save/delete and room split/merge commands.
        """
        if not SEGMENTS_SUPPORTED:
            raise HomeAssistantError(
                "This Home Assistant version has no vacuum segment support."
            )

        zone_set = self.coordinator.rooms.zones
        try:
            resolved = resolve_segment_boxes(zone_set, segment_ids)
        except ZoneError as err:
            known = ", ".join(sorted(zone_set.names)) or "(no zones taught)"
            raise ServiceValidationError(
                f"{err}. Known bObsweep zones: {known}"
            ) from err

        names = ", ".join(zone.name for zone, _ in resolved)
        boxes = [
            [int(round(v)) for v in box] for _zone, box in resolved
        ]
        _LOGGER.debug("bObsweep segment clean resolved to %s: %s", names, boxes)

        raise HomeAssistantError(
            f"bObsweep cannot yet start a segment clean ({names}). The zones "
            f"resolved correctly to map-cell rectangles {boxes}, but writing "
            "them to the robot needs DP 105 (COMMAND_TRANSPORTATION, "
            "eDesignated:16) plus the 'zone' work mode, and that write has "
            "never been probed on real hardware. Sending a speculative frame "
            "on the same datapoint that carries map delete and room merge is "
            "not safe, so nothing was sent."
        )

    # --- custom entity services ---------------------------------------------
    async def async_set_mode(self, mode: str) -> None:
        """Custom service: write a raw work-mode value to the work-mode DP."""
        if mode not in self._spec.work_mode.values():
            raise ServiceValidationError(
                f"{mode!r} is not a work mode of the {self._spec.key} bObsweep "
                f"family; valid values are: "
                f"{', '.join(sorted(self._spec.work_mode.values()))}"
            )
        await self.coordinator.async_set_dp(self._spec.dp_mode, mode)

    async def async_empty_dustbin(self) -> None:
        """Custom service: trigger the auto-empty dock to empty the dustbin."""
        if self._spec.dp_dustbin_empty_switch is None:
            raise ServiceValidationError(
                f"The {self._spec.key} bObsweep family has no self-emptying dock"
            )
        # Some models treat the self-empty DP as a persistent enable and others
        # as a momentary trigger; True works for both to kick off a cycle.
        await self.coordinator.async_set_dp(
            self._spec.dp_dustbin_empty_switch, True
        )

    async def async_set_dp_service(self, dp: str, value: Any) -> None:
        """Custom service: raw DP passthrough for debugging."""
        await self.coordinator.async_set_dp(str(dp), value)

    @callback
    def _handle_coordinator_update(self) -> None:
        """Handle updated data from the coordinator."""
        super()._handle_coordinator_update()


async def async_setup_entry(
    hass: HomeAssistant,
    entry: BobsweepConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up the bObsweep vacuum from a config entry."""
    coordinator = entry.runtime_data
    async_add_entities([BobsweepVacuum(coordinator)])

    # Register custom entity services on this platform.
    platform = entity_platform.async_get_current_platform()

    platform.async_register_entity_service(
        SERVICE_SET_MODE,
        {vol.Required("mode"): vol.In(_ALL_WORK_MODE_VALUES)},
        "async_set_mode",
    )
    platform.async_register_entity_service(
        SERVICE_EMPTY_DUSTBIN,
        {},
        "async_empty_dustbin",
    )
    platform.async_register_entity_service(
        SERVICE_SET_DP,
        {
            vol.Required("dp"): cv.string,
            vol.Required("value"): coerce_dp_value,
        },
        "async_set_dp_service",
    )

    # Zone capture / management services (bobsweep.capture_point, ...).
    async_register_zone_services(platform)
