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
"""

from __future__ import annotations

import logging
from typing import Any

from homeassistant.components.switch import SwitchEntity, SwitchEntityDescription
from homeassistant.const import EntityCategory
from homeassistant.core import HomeAssistant
from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from . import BobsweepConfigEntry
from .const import DEFAULT_NAME, DOMAIN
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


async def async_setup_entry(
    hass: HomeAssistant,
    entry: BobsweepConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up the bObsweep switches for a config entry."""
    coordinator = entry.runtime_data
    dp = coordinator.spec.dp_camera
    if dp is None:
        _LOGGER.debug(
            "Skipping bObsweep camera switch: family %s has no camera datapoint",
            coordinator.spec.key,
        )
        return
    async_add_entities([BobsweepCameraSwitch(coordinator, dp)])
