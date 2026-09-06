"""Number platform for the bObsweep (local Tuya) integration.

One entity: COMMAND_VOLUME (DP 108, SLAM only), a 0-100 int slider for the
robot's voice/alert volume. Same presence-gating rule as `select.py` (read that
module's docstring first): created only when the family has `dp_volume` AND the
datapoint is actually present in `coordinator.data` at setup.
"""

from __future__ import annotations

import logging
from typing import Any

from homeassistant.components.number import (
    NumberEntity,
    NumberEntityDescription,
    NumberMode,
)
from homeassistant.const import EntityCategory
from homeassistant.core import HomeAssistant
from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from . import BobsweepConfigEntry
from .const import DEFAULT_NAME, DOMAIN
from .coordinator import BobsweepCoordinator

_LOGGER = logging.getLogger(__name__)

VOLUME_DESCRIPTION = NumberEntityDescription(
    key="volume",
    translation_key="volume",
    name="Volume",
    icon="mdi:volume-high",
    entity_category=EntityCategory.CONFIG,
    native_min_value=0,
    native_max_value=100,
    native_step=1,
    mode=NumberMode.SLIDER,
)


class BobsweepVolumeNumber(CoordinatorEntity[BobsweepCoordinator], NumberEntity):
    """The robot's voice/alert volume (DP 108, 0-100 int)."""

    _attr_has_entity_name = True

    def __init__(self, coordinator: BobsweepCoordinator, dp: str) -> None:
        """Bind the number entity to the resolved volume datapoint."""
        super().__init__(coordinator)
        self.entity_description = VOLUME_DESCRIPTION
        self._dp = dp
        self._attr_unique_id = f"{coordinator.device_id}_{VOLUME_DESCRIPTION.key}"
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, coordinator.device_id)},
            manufacturer="bObsweep",
            model=coordinator.model_family,
            name=DEFAULT_NAME,
        )

    @property
    def native_value(self) -> float | None:
        """Current volume 0-100, or None before the first value arrives."""
        data = self.coordinator.data
        if not data or self._dp not in data:
            return None
        value = data[self._dp]
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    async def async_set_native_value(self, value: float) -> None:
        """Write the chosen volume as an int, the wire type DP 108 expects."""
        await self.coordinator.async_set_dp(self._dp, int(value))


async def async_setup_entry(
    hass: HomeAssistant,
    entry: BobsweepConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up the bObsweep number entities for a config entry."""
    coordinator = entry.runtime_data
    spec = coordinator.spec
    dp = spec.dp_volume
    if dp is None:
        _LOGGER.debug(
            "Skipping bObsweep volume number: family %s has no volume datapoint",
            spec.key,
        )
        return
    data = coordinator.data
    if not data or dp not in data:
        _LOGGER.debug(
            "Skipping bObsweep volume number: DP %s absent from coordinator data",
            dp,
        )
        return
    async_add_entities([BobsweepVolumeNumber(coordinator, dp)])
