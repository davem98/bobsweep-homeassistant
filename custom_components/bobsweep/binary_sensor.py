"""Binary sensor platform for the bObsweep (local Tuya) integration.

These are derived states, not raw datapoints: each description carries a
predicate over the coordinator's merged DP dict. The predicate takes the active
`FamilySpec` as well, because the DP ids *and* the status enum strings differ per
family (SLAM reports "standby" where Vision reports "idle", and so on).

A description also declares what it needs to exist. `required_dps` lists spec
attributes of which at least one must be non-None, and `required_status` lists
status-vocabulary keys the family must define. If the family can't satisfy them
the entity is not created at all — a Vision robot has no self-empty dock, so it
gets no permanently-off "Self-emptying" sensor.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Callable

from homeassistant.components.binary_sensor import (
    BinarySensorDeviceClass,
    BinarySensorEntity,
    BinarySensorEntityDescription,
)
from homeassistant.core import HomeAssistant
from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from . import BobsweepConfigEntry
from .const import DEFAULT_NAME, DOMAIN, FamilySpec
from .coordinator import BobsweepCoordinator
from .faults import decode_faults

_LOGGER = logging.getLogger(__name__)

@dataclass(frozen=True, kw_only=True)
class BobsweepBinarySensorEntityDescription(BinarySensorEntityDescription):
    """Describes a bObsweep binary sensor with a coordinator-data predicate."""

    is_on_fn: Callable[[FamilySpec, dict[str, Any]], bool]
    extra_attrs_fn: Callable[[FamilySpec, dict[str, Any]], dict[str, Any]] | None = None
    # At least one of these FamilySpec DP attributes must be non-None.
    required_dps: tuple[str, ...] = ()
    # All of these keys must exist in the family's status vocabulary.
    required_status: tuple[str, ...] = ()


def _self_emptying(spec: FamilySpec, data: dict[str, Any]) -> bool:
    return data.get(spec.dp_status) == spec.status["SELF_EMPTY_INPROGRESS"]


def _charging(spec: FamilySpec, data: dict[str, Any]) -> bool:
    return data.get(spec.dp_status) == spec.status["CHARGING"]


def _docked(spec: FamilySpec, data: dict[str, Any]) -> bool:
    return data.get(spec.dp_status) in spec.status_docked


def _has_error(spec: FamilySpec, data: dict[str, Any]) -> bool:
    """True when the robot reports a real fault on any error datapoint.

    The bits are decoded by name (see `faults.py`) so the two informational
    notes the app itself skips (`NoteMopWaterLow`, `NoteMoppingIsOff`) and
    Random's `Normal` bit 0 don't make a healthy robot look broken. Anything
    non-empty that can't be decoded still counts as a problem, which is the
    behaviour this sensor has always had.
    """
    if decode_faults(spec, data).has_fault:
        return True
    # Random is the only family with an explicit fault *status* value; for the
    # others status_error is empty and this is a no-op.
    return data.get(spec.dp_status) in spec.status_error


def _error_attrs(spec: FamilySpec, data: dict[str, Any]) -> dict[str, Any]:
    """Raw error datapoints plus their decoded fault names.

    The `error` / `error2` keys keep their original raw values so anything
    already reading them keeps working; everything else is additive.
    """
    attrs: dict[str, Any] = {}
    if spec.dp_error is not None:
        attrs["error"] = data.get(spec.dp_error)
    if spec.dp_error2 is not None:
        attrs["error2"] = data.get(spec.dp_error2)
    if spec.dp_error3 is not None:
        attrs["error3"] = data.get(spec.dp_error3)

    report = decode_faults(spec, data)
    attrs["faults"] = list(report.faults)
    attrs["fault"] = report.state
    attrs["notes"] = list(report.notes)
    if report.undecodable:
        # A non-empty error DP that could not be parsed: say so rather than reporting
        # an empty fault list next to an "on" problem sensor.
        attrs["fault_decode_error"] = True
    return attrs


# Enum DPs whose value is a word ("closed") rather than a bool: treat these
# tokens as "off" so e.g. the mop status "closed" reads as not mopping.
_OFF_TOKENS = ("closed", "close", "off", "0", "false", "none", "")


def _off_value(value: Any) -> bool:
    """True if the value is off *or absent*.

    Absence and "off" are deliberately conflated here, which is only safe where
    a missing datapoint genuinely means "not doing that". Use `_explicitly_off`
    wherever absence should mean "no evidence" instead -- conflating the two is
    what left the `vacuuming` sensor permanently off (see `_vacuuming`).
    """
    return value is None or _explicitly_off(value)


def _explicitly_off(value: Any) -> bool:
    """True only if the datapoint is present *and* reads as off."""
    return value is not None and str(value).strip().lower() in _OFF_TOKENS


def _cleaning(spec: FamilySpec, data: dict[str, Any]) -> bool:
    return data.get(spec.dp_status) in spec.status_cleaning


def _mopping(spec: FamilySpec, data: dict[str, Any]) -> bool:
    # Actively cleaning with the mop/water engaged. SLAM reports this on its mop
    # attachment DP (118); Random has no attachment DP for it and is judged by
    # its water-control setting (DP 20) instead.
    dp = spec.dp_status_mop or spec.dp_water_control
    if dp is None:
        return False
    if not _cleaning(spec, data):
        return False
    value = data.get(dp)
    # Absence is treated as "not mopping" *by choice*, not by accident. Unlike
    # DP 14, SLAM's mop-attachment DP 118 is returned by every `status()` poll
    # (observed reading `closed` while docked), so a merged snapshot without it
    # means the attachment genuinely is not reporting -- and "no mop attached"
    # is the right answer for a vacuum-only run. If a firmware ever stops
    # polling 118, this silently reads off: that is the same trap `_vacuuming`
    # fell into, so it is called out rather than left implicit.
    return value is not None and not _explicitly_off(value)


def _vacuuming(spec: FamilySpec, data: dict[str, Any]) -> bool:
    """Is the robot running a job with suction engaged?

    **Re-derived, 2026-09-01.** This sensor was originally modelled on DP 119
    (`COMMAND_STATUS_VACUUM`), the app's vacuum-attachment status. That DP does
    not exist on this firmware: it is absent from a live `status()` dump, and it
    never appeared once in a 107-record capture of a real cleaning job. The
    entity was therefore permanently off. It is deliberately *re-derived* rather
    than deleted -- it already exists in a live Home Assistant install, so
    removing it would leave an orphan in the entity registry; the `unique_id`
    (`<device_id>_vacuuming`), the key, the name and the on/off meaning are all
    unchanged, only the definition behind them.

    It is now built from datapoints the robot demonstrably does send:

    * **DP 5** (status) must be one of the family's cleaning statuses. In the
      reference capture DP 5 read `smart` for the whole job.
    * **DP 2** (enable), when the family has it, must not be explicitly off. A
      robot with the enable flag cleared is not running, whatever DP 5 says.
    * **DP 14** (fan speed) must not be the family's explicit off value
      (`closed`, SLAM only) -- that is a vacuum-off job such as a mop-only or
      quick-map run.

    The important detail is the last one's `is not None` guard. DP 14 is
    *pushed* only when it changes, so it can be absent from a merged snapshot
    even though a `status()` poll reports it. The previous implementation read a
    missing DP 14 as "off", which reintroduced exactly the always-off bug this
    is fixing. Absent now means "no evidence suction is off", and the status
    decides.
    """
    if not _cleaning(spec, data):
        return False

    if spec.dp_power is not None:
        power = data.get(spec.dp_power)
        if _explicitly_off(power):
            return False

    if spec.dp_fan is not None:
        fan = data.get(spec.dp_fan)
        if _explicitly_off(fan):
            return False

    return True


def _muted(spec: FamilySpec, data: dict[str, Any]) -> bool:
    # Vision's ROBOT_MUTE is a genuine bool ({ENABLE: true, DISABLE: false}).
    return bool(data.get(spec.dp_mute))


BINARY_SENSOR_DESCRIPTIONS: tuple[BobsweepBinarySensorEntityDescription, ...] = (
    BobsweepBinarySensorEntityDescription(
        key="self_emptying",
        translation_key="self_emptying",
        name="Self-emptying",
        device_class=BinarySensorDeviceClass.RUNNING,
        is_on_fn=_self_emptying,
        # SLAM only: no other family has a self-empty dock or its status value.
        required_status=("SELF_EMPTY_INPROGRESS",),
    ),
    BobsweepBinarySensorEntityDescription(
        key="charging",
        translation_key="charging",
        name="Charging",
        device_class=BinarySensorDeviceClass.BATTERY_CHARGING,
        is_on_fn=_charging,
        required_status=("CHARGING",),
    ),
    BobsweepBinarySensorEntityDescription(
        key="docked",
        translation_key="docked",
        name="Docked",
        icon="mdi:home-import-outline",
        is_on_fn=_docked,
    ),
    BobsweepBinarySensorEntityDescription(
        key="problem",
        translation_key="problem",
        name="Problem",
        device_class=BinarySensorDeviceClass.PROBLEM,
        is_on_fn=_has_error,
        extra_attrs_fn=_error_attrs,
        required_dps=("dp_error", "dp_error2", "dp_error3"),
    ),
    BobsweepBinarySensorEntityDescription(
        key="mopping",
        translation_key="mopping",
        name="Mopping",
        icon="mdi:mop",
        is_on_fn=_mopping,
        # Vision has neither a mop attachment DP nor water control -> skipped.
        required_dps=("dp_status_mop", "dp_water_control"),
    ),
    BobsweepBinarySensorEntityDescription(
        key="vacuuming",
        translation_key="vacuuming",
        name="Vacuuming",
        icon="mdi:robot-vacuum",
        is_on_fn=_vacuuming,
        required_dps=("dp_fan",),
    ),
    # Vision only (DP 215).
    BobsweepBinarySensorEntityDescription(
        key="muted",
        translation_key="muted",
        name="Muted",
        icon="mdi:volume-off",
        is_on_fn=_muted,
        required_dps=("dp_mute",),
    ),
)


class BobsweepBinarySensor(CoordinatorEntity[BobsweepCoordinator], BinarySensorEntity):
    """A derived on/off state for a bObsweep robot vacuum."""

    _attr_has_entity_name = True

    entity_description: BobsweepBinarySensorEntityDescription

    def __init__(
        self,
        coordinator: BobsweepCoordinator,
        description: BobsweepBinarySensorEntityDescription,
    ) -> None:
        """Initialize the binary sensor entity."""
        super().__init__(coordinator)
        self.entity_description = description
        self._attr_unique_id = f"{coordinator.device_id}_{description.key}"
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, coordinator.device_id)},
            manufacturer="bObsweep",
            model=coordinator.model_family,
            name=DEFAULT_NAME,
        )

    @property
    def is_on(self) -> bool | None:
        """Return True/False from the description's predicate, None if no data."""
        data = self.coordinator.data
        if not data:
            return None
        return self.entity_description.is_on_fn(self.coordinator.spec, data)

    @property
    def extra_state_attributes(self) -> dict[str, Any] | None:
        """Return description-specific extra attributes, if any."""
        if self.entity_description.extra_attrs_fn is None:
            return None
        data = self.coordinator.data or {}
        return self.entity_description.extra_attrs_fn(self.coordinator.spec, data)


def _supported(spec: FamilySpec, description: BobsweepBinarySensorEntityDescription) -> bool:
    """Return True if the family has everything this description needs."""
    if description.required_dps and not any(
        getattr(spec, attr) is not None for attr in description.required_dps
    ):
        return False
    return all(key in spec.status for key in description.required_status)


async def async_setup_entry(
    hass: HomeAssistant,
    entry: BobsweepConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up bObsweep binary sensors from a config entry."""
    coordinator = entry.runtime_data
    spec = coordinator.spec

    entities: list[BobsweepBinarySensor] = []
    for description in BINARY_SENSOR_DESCRIPTIONS:
        if not _supported(spec, description):
            _LOGGER.debug(
                "Skipping bObsweep binary sensor %s: unsupported by family %s",
                description.key,
                spec.key,
            )
            continue
        entities.append(BobsweepBinarySensor(coordinator, description))

    async_add_entities(entities)
