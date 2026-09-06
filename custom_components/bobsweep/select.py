"""Select platform for the bObsweep (local Tuya) integration: enum settings.

**Presence-gating rule (applies to every entity this module and the switch/number
platforms create for the settings DPs added 2026-09-05).** The vendor's DP-MAPS
tables describe what a product *family* can support, not what a specific unit
actually emits. A live raw-status read of the reference SLAM unit on 2026-09-05
had DP 20/107/108/111/112/115/122/124/134 present but DP 119, 130, 132, 133 and
135 absent, even though the family tables list them all. An entity is therefore
created only when BOTH of these hold:

1. The active `FamilySpec` has the datapoint (its `dp_*` field is not `None`).
2. The datapoint key is actually present in `coordinator.data` at platform
   setup (`_present()` below).

Failing either check means the platform creates nothing for that DP, rather
than publishing a control that can never be read back and would silently do
nothing when written.

Five selects, all `EntityCategory.CONFIG`, all SLAM-only except `water_level`
(SLAM + Random, different option lists) and all following the DP 128 camera
switch's structure (`CoordinatorEntity`, `_attr_has_entity_name`, per-family
device_info, `coordinator.async_set_dp` for writes).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any

from homeassistant.components.select import SelectEntity, SelectEntityDescription
from homeassistant.const import EntityCategory
from homeassistant.core import HomeAssistant
from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from . import BobsweepConfigEntry
from .const import DEFAULT_NAME, DOMAIN, FamilySpec
from .coordinator import BobsweepCoordinator

_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True, kw_only=True)
class BobsweepSelectEntityDescription(SelectEntityDescription):
    """Describes one enum-setting select: which DP field and which vocabulary."""

    dp_attr: str      # FamilySpec attribute name holding the datapoint id
    vocab_attr: str   # FamilySpec attribute name holding the NAME -> wire enum map


SELECT_DESCRIPTIONS: tuple[BobsweepSelectEntityDescription, ...] = (
    BobsweepSelectEntityDescription(
        key="water_level",
        translation_key="water_level",
        name="Water level",
        icon="mdi:water-percent",
        entity_category=EntityCategory.CONFIG,
        dp_attr="dp_water_level",
        vocab_attr="water_level",
    ),
    BobsweepSelectEntityDescription(
        key="floor_type_detection",
        translation_key="floor_type_detection",
        name="Floor type detection",
        icon="mdi:floor-plan",
        entity_category=EntityCategory.CONFIG,
        dp_attr="dp_floor_type_detection",
        vocab_attr="floor_type_detection",
    ),
    BobsweepSelectEntityDescription(
        key="self_empty_power",
        translation_key="self_empty_power",
        name="Self-empty power",
        icon="mdi:fan",
        entity_category=EntityCategory.CONFIG,
        dp_attr="dp_self_empty_power",
        vocab_attr="self_empty_power",
    ),
    BobsweepSelectEntityDescription(
        key="mop_maintenance_strategy",
        translation_key="mop_maintenance_strategy",
        name="Mop maintenance strategy",
        icon="mdi:mop",
        entity_category=EntityCategory.CONFIG,
        dp_attr="dp_mop_maintenance_strategy",
        vocab_attr="mop_maintenance_strategy",
    ),
    BobsweepSelectEntityDescription(
        key="extending_arms",
        translation_key="extending_arms",
        name="Extending arms",
        icon="mdi:arrow-expand-horizontal",
        entity_category=EntityCategory.CONFIG,
        dp_attr="dp_extending_arms",
        vocab_attr="extending_arms",
    ),
    BobsweepSelectEntityDescription(
        key="mop_dry_duration",
        translation_key="mop_dry_duration",
        name="Mop dry duration",
        icon="mdi:hair-dryer",
        entity_category=EntityCategory.CONFIG,
        dp_attr="dp_mop_dry_duration",
        vocab_attr="mop_dry_duration",
    ),
    BobsweepSelectEntityDescription(
        key="mop_wash_temperature",
        translation_key="mop_wash_temperature",
        name="Mop wash temperature",
        icon="mdi:thermometer",
        entity_category=EntityCategory.CONFIG,
        dp_attr="dp_mop_wash_temperature",
        vocab_attr="mop_wash_temperature",
    ),
    BobsweepSelectEntityDescription(
        key="dock_task_self_empty",
        translation_key="dock_task_self_empty",
        name="Dock task on self-empty",
        icon="mdi:home-import-outline",
        entity_category=EntityCategory.CONFIG,
        dp_attr="dp_dock_task_self_empty",
        vocab_attr="dock_task_self_empty",
    ),
)


def supported(spec: FamilySpec, data: dict[str, Any] | None, description: BobsweepSelectEntityDescription) -> str | None:
    """Return the datapoint id if this description should get an entity, else None.

    Enforces the two-part presence-gating rule from the module docstring: the
    family must declare the datapoint AND that datapoint must have actually
    shown up in a merged coordinator snapshot.
    """
    dp = getattr(spec, description.dp_attr)
    vocab = getattr(spec, description.vocab_attr)
    if dp is None or not vocab:
        return None
    if not data or dp not in data:
        return None
    return dp


class BobsweepSelect(CoordinatorEntity[BobsweepCoordinator], SelectEntity):
    """A settable enum datapoint, exposed as an HA select."""

    _attr_has_entity_name = True

    entity_description: BobsweepSelectEntityDescription

    def __init__(
        self,
        coordinator: BobsweepCoordinator,
        description: BobsweepSelectEntityDescription,
        dp: str,
    ) -> None:
        """Bind the select to its resolved datapoint and enum vocabulary."""
        super().__init__(coordinator)
        self.entity_description = description
        self._dp = dp
        self._vocab: dict[str, str] = dict(getattr(coordinator.spec, description.vocab_attr))
        self._reverse: dict[str, str] = {v: k for k, v in self._vocab.items()}
        self._attr_unique_id = f"{coordinator.device_id}_{description.key}"
        self._attr_options = list(self._vocab.values())
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, coordinator.device_id)},
            manufacturer="bObsweep",
            model=coordinator.model_family,
            name=DEFAULT_NAME,
        )

    @property
    def current_option(self) -> str | None:
        """The wire-value enum currently reported, or None if unknown."""
        data = self.coordinator.data
        if not data or self._dp not in data:
            return None
        value = data[self._dp]
        if value in self._reverse:
            return value
        return None

    async def async_select_option(self, option: str) -> None:
        """Write the chosen wire-value enum straight to the datapoint."""
        await self.coordinator.async_set_dp(self._dp, option)


async def async_setup_entry(
    hass: HomeAssistant,
    entry: BobsweepConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up the bObsweep selects for a config entry."""
    coordinator = entry.runtime_data
    spec = coordinator.spec
    data = coordinator.data

    entities: list[BobsweepSelect] = []
    for description in SELECT_DESCRIPTIONS:
        dp = supported(spec, data, description)
        if dp is None:
            _LOGGER.debug(
                "Skipping bObsweep select %s: unsupported by family %s or absent from data",
                description.key,
                spec.key,
            )
            continue
        entities.append(BobsweepSelect(coordinator, description, dp))

    async_add_entities(entities)
