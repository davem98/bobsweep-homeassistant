"""Switch platform for the bObsweep (local Tuya) integration.

One entity today: the robot's on-board object-detection camera (DP 128,
`COMMAND_CAMERA`). It is a plain boolean, verified three ways in the vendor app
-- the read path (`isObjectDetection`), the write path
(`publishDps({128: <bool>})`) and a UI toggle that drives both -- and the
reference unit reports `True`.

**Why this platform exists at all.** DP 128 is the only user-facing control over
the camera on the entire device, and turning a camera off is exactly the kind of
thing someone should be able to do from their own dashboard without opening the
vendor's cloud app. bObsweep's own copy states the images are processed on the
robot and never uploaded; that is their claim about their firmware, not
something this integration can verify, which is another reason the user should
have the switch.

SLAM only. Vision and Random have no DP 128 (`dp_camera` is `None` for them), so
the platform creates nothing rather than publishing a switch that writes into a
datapoint the robot does not implement.

**Settings switches, added 2026-09-05.** `mute`, `cliff_sensor`,
`auto_empty` and `quick_clean_use_global_vacuum` follow the same
presence-gating rule as `select.py` (read that module's docstring for the
rationale): an entity is created only when the family declares the datapoint
AND the datapoint is actually present in `coordinator.data` at platform setup.
`cliff_sensor` is the one oddity -- DP 111 (`COMMAND_CLIFF_SENSOR`) is a
*string* `'on'`/`'off'` on the wire, not a Tuya bool, so it gets its own
read/write conversion rather than the plain-bool one the others share.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Callable

from homeassistant.components.switch import SwitchEntity, SwitchEntityDescription
from homeassistant.const import EntityCategory
from homeassistant.core import HomeAssistant
from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from . import BobsweepConfigEntry
from .const import DEFAULT_NAME, DOMAIN, FamilySpec
from .coordinator import BobsweepCoordinator

_LOGGER = logging.getLogger(__name__)

CAMERA_DESCRIPTION = SwitchEntityDescription(
    key="camera_object_detection",
    translation_key="camera_object_detection",
    name="Camera obstacle detection",
    icon="mdi:cctv",
    entity_category=EntityCategory.CONFIG,
)


class BobsweepCameraSwitch(CoordinatorEntity[BobsweepCoordinator], SwitchEntity):
    """Turns the robot's obstacle-detection camera on and off (DP 128).

    Named for what it actually controls. "Camera" alone would be ambiguous --
    there is no video feed here and this integration exposes no camera entity --
    while "obstacle detection" alone would hide that a camera is involved, which
    is the part a privacy-minded user cares about. Off means the camera is not
    running and the AI-object obstacle reports (see `transport.py`) stop with it.
    """

    _attr_has_entity_name = True

    def __init__(self, coordinator: BobsweepCoordinator, dp: str) -> None:
        """Bind the switch to the already-resolved camera datapoint."""
        super().__init__(coordinator)
        self.entity_description = CAMERA_DESCRIPTION
        self._dp = dp
        self._attr_unique_id = f"{coordinator.device_id}_{CAMERA_DESCRIPTION.key}"
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, coordinator.device_id)},
            manufacturer="bObsweep",
            model=coordinator.model_family,
            name=DEFAULT_NAME,
        )

    @property
    def is_on(self) -> bool | None:
        """True/False from DP 128, or None before the first value arrives.

        The datapoint is a real boolean on the wire, but tinytuya has been seen
        surfacing Tuya booleans as the strings "True"/"False" on some protocol
        versions, so string forms are accepted rather than silently reading as
        truthy-non-empty (which would make "False" mean on).
        """
        data = self.coordinator.data
        if not data or self._dp not in data:
            return None
        value = data[self._dp]
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in ("true", "on", "1"):
                return True
            if lowered in ("false", "off", "0"):
                return False
            return None
        if isinstance(value, (int, float)):
            return bool(value)
        return None

    async def async_turn_on(self, **kwargs: Any) -> None:
        """Turn the obstacle-detection camera on."""
        await self.coordinator.async_set_dp(self._dp, True)

    async def async_turn_off(self, **kwargs: Any) -> None:
        """Turn the obstacle-detection camera off."""
        await self.coordinator.async_set_dp(self._dp, False)


@dataclass(frozen=True, kw_only=True)
class BobsweepDpSwitchEntityDescription(SwitchEntityDescription):
    """Describes a plain settings switch bound to one FamilySpec DP attribute."""

    dp_attr: str
    # Converts the raw coordinator value to True/False/None (unknown).
    decode: Callable[[Any], bool | None]
    # Converts True/False to the exact wire value `async_set_dp` should send.
    encode: Callable[[bool], Any]


def _decode_bool(value: Any) -> bool | None:
    """Same tolerant bool decode as the camera switch's `is_on`."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ("true", "on", "1"):
            return True
        if lowered in ("false", "off", "0"):
            return False
        return None
    if isinstance(value, (int, float)):
        return bool(value)
    return None


def _encode_bool(value: bool) -> bool:
    return value


def _decode_onoff_string(value: Any) -> bool | None:
    """DP 111 (COMMAND_CLIFF_SENSOR) is the *string* 'on'/'off', not a bool."""
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered == "on":
            return True
        if lowered == "off":
            return False
    return None


def _encode_onoff_string(value: bool) -> str:
    return "on" if value else "off"


SETTINGS_SWITCH_DESCRIPTIONS: tuple[BobsweepDpSwitchEntityDescription, ...] = (
    BobsweepDpSwitchEntityDescription(
        key="mute",
        translation_key="mute",
        name="Mute",
        icon="mdi:volume-off",
        entity_category=EntityCategory.CONFIG,
        dp_attr="dp_mute_switch",
        decode=_decode_bool,
        encode=_encode_bool,
    ),
    # SLAM only: Vision's cliff sensor (DP 218) stays the existing raw
    # `dp_cliff_sensor` sensor field, untouched by this addition.
    BobsweepDpSwitchEntityDescription(
        key="cliff_sensor",
        translation_key="cliff_sensor",
        name="Cliff sensor",
        icon="mdi:signal-distance-variant",
        entity_category=EntityCategory.CONFIG,
        dp_attr="dp_cliff_sensor_switch",
        decode=_decode_onoff_string,
        encode=_encode_onoff_string,
    ),
    BobsweepDpSwitchEntityDescription(
        key="auto_empty",
        translation_key="auto_empty",
        name="Auto empty",
        icon="mdi:delete-empty",
        entity_category=EntityCategory.CONFIG,
        dp_attr="dp_dustbin_empty_switch",
        decode=_decode_bool,
        encode=_encode_bool,
    ),
    BobsweepDpSwitchEntityDescription(
        key="quick_clean_use_global_vacuum",
        translation_key="quick_clean_use_global_vacuum",
        name="Quick clean uses global vacuum settings",
        icon="mdi:vacuum-cleaner",
        entity_category=EntityCategory.CONFIG,
        dp_attr="dp_quick_clean_use_global_vacuum",
        decode=_decode_bool,
        encode=_encode_bool,
    ),
)


class BobsweepDpSwitch(CoordinatorEntity[BobsweepCoordinator], SwitchEntity):
    """A plain settings switch bound to one datapoint via its description's
    decode/encode pair (see `BobsweepDpSwitchEntityDescription`)."""

    _attr_has_entity_name = True

    entity_description: BobsweepDpSwitchEntityDescription

    def __init__(
        self,
        coordinator: BobsweepCoordinator,
        description: BobsweepDpSwitchEntityDescription,
        dp: str,
    ) -> None:
        """Bind the switch to its resolved datapoint."""
        super().__init__(coordinator)
        self.entity_description = description
        self._dp = dp
        self._attr_unique_id = f"{coordinator.device_id}_{description.key}"
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, coordinator.device_id)},
            manufacturer="bObsweep",
            model=coordinator.model_family,
            name=DEFAULT_NAME,
        )

    @property
    def is_on(self) -> bool | None:
        """Decoded state from the bound datapoint, or None before it arrives."""
        data = self.coordinator.data
        if not data or self._dp not in data:
            return None
        return self.entity_description.decode(data[self._dp])

    async def async_turn_on(self, **kwargs: Any) -> None:
        """Turn the setting on."""
        await self.coordinator.async_set_dp(self._dp, self.entity_description.encode(True))

    async def async_turn_off(self, **kwargs: Any) -> None:
        """Turn the setting off."""
        await self.coordinator.async_set_dp(self._dp, self.entity_description.encode(False))


def _settings_switch_dp(
    spec: FamilySpec, data: dict[str, Any] | None, description: BobsweepDpSwitchEntityDescription
) -> str | None:
    """Presence-gated DP lookup: see the module docstring's gating rule."""
    dp = getattr(spec, description.dp_attr)
    if dp is None:
        return None
    if not data or dp not in data:
        return None
    return dp


async def async_setup_entry(
    hass: HomeAssistant,
    entry: BobsweepConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up the bObsweep switches for a config entry."""
    coordinator = entry.runtime_data
    spec = coordinator.spec
    data = coordinator.data

    entities: list[SwitchEntity] = []

    dp = spec.dp_camera
    if dp is None:
        _LOGGER.debug(
            "Skipping bObsweep camera switch: family %s has no camera datapoint",
            spec.key,
        )
    else:
        entities.append(BobsweepCameraSwitch(coordinator, dp))

    for description in SETTINGS_SWITCH_DESCRIPTIONS:
        dp = _settings_switch_dp(spec, data, description)
        if dp is None:
            _LOGGER.debug(
                "Skipping bObsweep switch %s: unsupported by family %s or absent from data",
                description.key,
                spec.key,
            )
            continue
        entities.append(BobsweepDpSwitch(coordinator, description, dp))

    async_add_entities(entities)
