"""Sensor platform for the bObsweep (local Tuya) integration.

Each sensor is one Tuya datapoint surfaced raw. Because the three DP families
put the same reading on different ids — and some families simply do not report a
value at all — descriptions name the *FamilySpec attribute* holding the id
(`dp_attr`) rather than a literal id. Setup resolves it against the configured
family and skips any description whose DP is `None`, so a Vision robot never
gets a "Filter life" entity that would sit unknown forever.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any

from homeassistant.components.sensor import (
    SensorDeviceClass,
    SensorEntity,
    SensorEntityDescription,
    SensorStateClass,
)
from homeassistant.const import PERCENTAGE, EntityCategory
from homeassistant.core import HomeAssistant
from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from . import BobsweepConfigEntry
from .const import DEFAULT_NAME, DOMAIN
from .coordinator import BobsweepCoordinator
from .faults import FAULT_CHANNELS, decode_faults, fault_state_options

_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True, kw_only=True)
class BobsweepSensorEntityDescription(SensorEntityDescription):
    """Describes a bObsweep sensor entity backed by a single Tuya datapoint.

    `dp_attr` is the name of the `FamilySpec` field carrying the datapoint id for
    the configured model family. If that field is `None` the family has no such
    datapoint and the entity is not created.
    """

    dp_attr: str


SENSOR_DESCRIPTIONS: tuple[BobsweepSensorEntityDescription, ...] = (
    BobsweepSensorEntityDescription(
        key="side_brush_life",
        translation_key="side_brush_life",
        name="Side brush life",
        dp_attr="dp_side_brush_life",
        native_unit_of_measurement=PERCENTAGE,
        icon="mdi:broom",
        entity_category=EntityCategory.DIAGNOSTIC,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    BobsweepSensorEntityDescription(
        key="roll_brush_life",
        translation_key="roll_brush_life",
        name="Rolling brush life",
        dp_attr="dp_roll_brush_life",
        native_unit_of_measurement=PERCENTAGE,
        icon="mdi:broom",
        entity_category=EntityCategory.DIAGNOSTIC,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    BobsweepSensorEntityDescription(
        key="filter_life",
        translation_key="filter_life",
        name="Filter life",
        dp_attr="dp_filter_life",
        native_unit_of_measurement=PERCENTAGE,
        icon="mdi:air-filter",
        entity_category=EntityCategory.DIAGNOSTIC,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    BobsweepSensorEntityDescription(
        key="battery",
        translation_key="battery",
        name="Battery",
        dp_attr="dp_battery",
        native_unit_of_measurement=PERCENTAGE,
        device_class=SensorDeviceClass.BATTERY,
        entity_category=EntityCategory.DIAGNOSTIC,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    BobsweepSensorEntityDescription(
        key="clean_area",
        translation_key="clean_area",
        name="Last clean area",
        dp_attr="dp_clean_area",
        native_unit_of_measurement="m²",
        icon="mdi:set-square",
        state_class=SensorStateClass.MEASUREMENT,
    ),
    BobsweepSensorEntityDescription(
        key="clean_time",
        translation_key="clean_time",
        name="Last clean time",
        dp_attr="dp_clean_time",
        native_unit_of_measurement="min",
        device_class=SensorDeviceClass.DURATION,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    BobsweepSensorEntityDescription(
        key="status",
        translation_key="status",
        name="Status",
        dp_attr="dp_status",
        icon="mdi:robot-vacuum",
        entity_category=EntityCategory.DIAGNOSTIC,
    ),
    # --- family-specific extras ---------------------------------------------
    # NB: water control (DP 20) used to be a read-only sensor here. It is now a
    # writable `select` on both families that have it, so a duplicate read-only
    # copy would only be a second name for the same value.
    #
    # Random only (DP 103): the combined dustbin / water-tank attachment status
    # that replaces SLAM's separate mop (118) and vacuum (119) DPs.
    BobsweepSensorEntityDescription(
        key="dustbin_watertank",
        translation_key="dustbin_watertank",
        name="Dustbin / water tank",
        dp_attr="dp_status_dustbin_watertank",
        icon="mdi:delete-variant",
        entity_category=EntityCategory.DIAGNOSTIC,
    ),
    # Vision only (DP 218). The app bundle documents no enum for it, so the raw
    # value is surfaced as-is rather than guessed at as a boolean.
    BobsweepSensorEntityDescription(
        key="cliff_sensor",
        translation_key="cliff_sensor",
        name="Cliff sensor",
        dp_attr="dp_cliff_sensor",
        icon="mdi:stairs",
        entity_category=EntityCategory.DIAGNOSTIC,
    ),
)


class BobsweepSensor(CoordinatorEntity[BobsweepCoordinator], SensorEntity):
    """A single-datapoint sensor on a bObsweep robot vacuum."""

    _attr_has_entity_name = True

    entity_description: BobsweepSensorEntityDescription

    def __init__(
        self,
        coordinator: BobsweepCoordinator,
        description: BobsweepSensorEntityDescription,
        dp: str,
    ) -> None:
        """Initialize the sensor entity for an already-resolved datapoint id."""
        super().__init__(coordinator)
        self.entity_description = description
        self._dp_id = dp
        self._attr_unique_id = f"{coordinator.device_id}_{description.key}"
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, coordinator.device_id)},
            manufacturer="bObsweep",
            model=coordinator.model_family,
            name=DEFAULT_NAME,
        )

    def _dp(self, dp: str) -> Any:
        """Return the current value of a datapoint, or None if unavailable."""
        data = self.coordinator.data or {}
        return data.get(dp)

    @property
    def native_value(self) -> Any:
        """Return the raw datapoint value, or None when it is unavailable."""
        return self._dp(self._dp_id)


class BobsweepFaultSensor(CoordinatorEntity[BobsweepCoordinator], SensorEntity):
    """The robot's current fault, by name.

    Unlike every other sensor here this one is not a raw datapoint: it decodes
    the family's error datapoints (DP 18 / 113 / 131) against the app's own
    bit-index tables and reports the highest-priority active fault. `none` means
    the robot is reporting no fault at all — a real state, so an automation can
    trigger on the transition away from it. Because several bits can be set at
    once, the full set is on the `faults` attribute.
    """

    _attr_has_entity_name = True

    def __init__(self, coordinator: BobsweepCoordinator) -> None:
        """Initialize the fault sensor for the coordinator's model family."""
        super().__init__(coordinator)
        self.entity_description = SensorEntityDescription(
            key="fault",
            translation_key="fault",
            name="Fault",
            device_class=SensorDeviceClass.ENUM,
            icon="mdi:alert-circle-outline",
            options=fault_state_options(coordinator.spec),
        )
        self._attr_unique_id = f"{coordinator.device_id}_fault"
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, coordinator.device_id)},
            manufacturer="bObsweep",
            model=coordinator.model_family,
            name=DEFAULT_NAME,
        )

    def _report(self):
        """Decode the current fault state; never raises."""
        return decode_faults(self.coordinator.spec, self.coordinator.data)

    @property
    def native_value(self) -> str | None:
        """Return the top active fault slug, `none`, or None when unknown."""
        if self.coordinator.data is None:
            return None
        report = self._report()
        if not report.faults and report.undecodable:
            # Something is wrong but it cannot be named — don't claim `none`.
            return None
        return report.state

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        """All active faults and notes, plus the raw datapoints behind them."""
        spec = self.coordinator.spec
        report = self._report()
        attrs: dict[str, Any] = {
            "faults": list(report.faults),
            "fault_count": len(report.faults),
            "notes": list(report.notes),
        }
        for dp_attr, key in (
            ("dp_error", "error"),
            ("dp_error2", "error2"),
            ("dp_error3", "error3"),
        ):
            if getattr(spec, dp_attr) is not None:
                attrs[key] = report.raw.get(dp_attr)
        if report.undecodable:
            attrs["fault_decode_error"] = True
        return attrs


class BobsweepObstaclesSensor(CoordinatorEntity[BobsweepCoordinator], SensorEntity):
    """How many obstacles the robot's camera currently reports (DP 105, 0x37).

    Not a raw datapoint. The robot pushes `eAiObjectToAPP` frames on the
    transportation channel during a job; `transport.py` decodes them and
    `AiObjectTracker` (owned by the coordinator) keeps the latest list. The
    state is the length of that list and the `objects` attribute carries it as
    `{x, y, class}` entries in raw map cells -- the same coordinate frame as the
    DP 105 no-go zones, so an obstacle and a zone are directly comparable.

    Two properties of the underlying data, both the robot's behaviour rather
    than this integration's:

    * The list **accumulates over a job** and is ordered **newest first**. It is
      not a snapshot of what is in front of the robot right now, so the count
      only ever grows during a run.
    * The robot **refines** an existing object's coordinates between frames, by
      tens of map cells. The state deliberately reports the robot's own count
      verbatim; `distinct_seen` on the attributes is this module's tolerance-based
      dedupe of everything seen this session, and the two can legitimately differ.

    `None` (unknown) until a valid 0x37 frame has been decoded, which is honest:
    "the robot has not reported anything" is not the same as "zero obstacles".
    Count 0 is a real value the robot does send.

    SLAM only -- the other two families have no transportation datapoint.
    """

    _attr_has_entity_name = True

    def __init__(self, coordinator: BobsweepCoordinator) -> None:
        """Initialize the obstacle-count sensor."""
        super().__init__(coordinator)
        self.entity_description = SensorEntityDescription(
            key="detected_obstacles",
            translation_key="detected_obstacles",
            name="Detected obstacles",
            icon="mdi:eye-outline",
            state_class=SensorStateClass.MEASUREMENT,
        )
        self._attr_unique_id = f"{coordinator.device_id}_detected_obstacles"
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, coordinator.device_id)},
            manufacturer="bObsweep",
            model=coordinator.model_family,
            name=DEFAULT_NAME,
        )

    @property
    def native_value(self) -> int | None:
        """The robot's current obstacle count, or None if it has not said."""
        tracker = self.coordinator.ai_objects
        return None if tracker is None else tracker.count

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        """The obstacle list plus decode diagnostics."""
        tracker = self.coordinator.ai_objects
        if tracker is None:
            return {}
        return tracker.as_attributes()


class BobsweepSelectedRoomsSensor(CoordinatorEntity[BobsweepCoordinator], SensorEntity):
    """Which rooms the last room-targeted clean was told to do (DP 105, 0x22).

    Not a raw datapoint, and not pollable. `eCleanSelectRoomsToApp` is the
    robot's **ack** of an `eCleanSelectRooms` command from the app; it is not
    part of the `eAll` report set, so the only way to know a room selection is
    to be listening when it goes past and then remember it.
    `RoomSelectionTracker` (owned by the coordinator) does the remembering.

    Three states, all meaningfully different:

    * ``"1,3"`` -- a comma-joined list of the selected room ids;
    * ``"none"`` -- the robot acked a selection with no rooms in it;
    * ``None`` / unknown -- no ack has been seen this session, or the one that
      was seen had a payload layout this integration could not read. The
      ``layout`` and ``raw`` attributes tell those two apart.

    The ids are the robot's own room numbers from its map partition, not the
    zone names taught through this integration; the two are separate concepts
    and nothing here tries to reconcile them.

    SLAM only -- the other two families have no transportation datapoint.
    """

    _attr_has_entity_name = True

    def __init__(self, coordinator: BobsweepCoordinator) -> None:
        """Initialize the selected-rooms sensor."""
        super().__init__(coordinator)
        self.entity_description = SensorEntityDescription(
            key="selected_rooms",
            translation_key="selected_rooms",
            name="Selected rooms",
            icon="mdi:floor-plan",
            entity_category=EntityCategory.DIAGNOSTIC,
        )
        self._attr_unique_id = f"{coordinator.device_id}_selected_rooms"
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, coordinator.device_id)},
            manufacturer="bObsweep",
            model=coordinator.model_family,
            name=DEFAULT_NAME,
        )

    @property
    def native_value(self) -> str | None:
        """The selected room ids, `none`, or None when nothing is known."""
        tracker = getattr(self.coordinator, "room_selection", None)
        if tracker is None or tracker.room_ids is None:
            return None
        if not tracker.room_ids:
            return "none"
        return ",".join(str(room_id) for room_id in tracker.room_ids)

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        """The selection, its decoded layout, the raw payload, and room names."""
        tracker = getattr(self.coordinator, "room_selection", None)
        if tracker is None:
            return {}
        attrs = tracker.as_attributes()
        # Names for the ids, where any are known. The robot never answers "what
        # is room 3 called" directly; the map is assembled from single-room
        # schedule names plus anything named by hand. See coordinator.room_names.
        names = self._room_names()
        attrs["room_names"] = names
        if tracker.room_ids:
            attrs["selected_room_names"] = [
                names.get(room_id, f"Room {room_id}") for room_id in tracker.room_ids
            ]
        return attrs

    def _room_names(self) -> dict[int, str]:
        """Effective id -> name map, tolerating a coordinator without one."""
        getter = getattr(self.coordinator, "room_names", None)
        if not callable(getter):
            return {}
        try:
            return dict(getter())
        except Exception:  # noqa: BLE001 - an attribute must never raise
            _LOGGER.debug("bObsweep: room_names() failed", exc_info=True)
            return {}


class BobsweepPathTrailSensor(CoordinatorEntity[BobsweepCoordinator], SensorEntity):
    """How many points the robot's current path trail contains (DP 104).

    The state is the **robot's own** `count` for the current path, not how many
    points this integration happens to hold. Those differ, legitimately and
    often: the trail is received passively -- the robot emits new points while a
    map session is running and we never ask for any -- so a session that starts
    listening mid-job sees only the batches that were still on the wire.
    `points_held` on the attributes is our number; the state is the robot's.

    `None` (unknown) until a `cmd:102` batch has been decoded. The count resets
    when the `pathid` changes, which is once per job.

    SLAM only -- the other two families have no path datapoint.
    """

    _attr_has_entity_name = True

    def __init__(self, coordinator: BobsweepCoordinator) -> None:
        """Initialize the path-trail sensor."""
        super().__init__(coordinator)
        self.entity_description = SensorEntityDescription(
            key="path_trail",
            translation_key="path_trail",
            name="Path trail",
            icon="mdi:map-marker-path",
            entity_category=EntityCategory.DIAGNOSTIC,
            state_class=SensorStateClass.MEASUREMENT,
        )
        self._attr_unique_id = f"{coordinator.device_id}_path_trail"
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, coordinator.device_id)},
            manufacturer="bObsweep",
            model=coordinator.model_family,
            name=DEFAULT_NAME,
        )

    @property
    def native_value(self) -> int | None:
        """The robot's point count for the current path, or None if unknown."""
        tracker = getattr(self.coordinator, "trail", None)
        return None if tracker is None else tracker.count

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        """Path id, how much of the trail we hold, and decode diagnostics."""
        tracker = getattr(self.coordinator, "trail", None)
        return {} if tracker is None else tracker.as_attributes()


class BobsweepCurrentRoomSensor(CoordinatorEntity[BobsweepCoordinator], SensorEntity):
    """Which taught zone the robot is currently in.

    Three genuinely different states, and automations need to tell them apart:

    * a **zone name** — the robot is confirmed to be in that room;
    * ``unmapped`` — the robot's position is known and it is in no taught zone
      (a hallway, or a room nobody has taught yet);
    * ``None`` / unknown — the robot's position is not known at all: no verified
      position source (today's normal case, see `position.py`), no fix this
      tick, or the zone set has gone stale and can no longer be trusted.

    Collapsing the last two would be the easy mistake. "In an unmapped part of
    the house" and "position unavailable, do not act on this" call for opposite
    automation behaviour.

    **Not a `SensorDeviceClass.ENUM`.** An enum sensor must declare its full
    option set up front, and the option set here is the zone names, which change
    every time the user teaches or deletes a zone. Redeclaring options means
    reloading the entity (and HA logs an error for any state outside the
    declared set), so a mid-capture rename would churn the entity registry. A
    plain string state costs the translated state names an enum would give and
    buys a sensor whose value can change the moment a zone does. The trade is
    deliberate; `dreame-vacuum` and `mqtt_vacuum_camera`, the two shipping
    integrations with this entity, both make the same call — which is also why
    the entity is named `current_room` rather than anything more inventive.

    The entity is created even when no position source exists. It is the surface
    for `last_known_room` and the `stale` flag, both of which the user needs
    while zones are being taught, and it goes live with no reconfiguration the
    moment a position source is verified.
    """

    _attr_has_entity_name = True

    def __init__(self, coordinator: BobsweepCoordinator) -> None:
        """Initialize the current-room sensor."""
        super().__init__(coordinator)
        self.entity_description = SensorEntityDescription(
            key="current_room",
            translation_key="current_room",
            name="Current room",
            icon="mdi:floor-plan",
        )
        self._attr_unique_id = f"{coordinator.device_id}_current_room"
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, coordinator.device_id)},
            manufacturer="bObsweep",
            model=coordinator.model_family,
            name=DEFAULT_NAME,
        )

    @property
    def native_value(self) -> str | None:
        """Return the confirmed room name, `unmapped`, or None for unknown."""
        return self.coordinator.rooms.state

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        """Position, last known room, zone provenance and staleness."""
        return self.coordinator.rooms.attributes()


class BobsweepRobotInfoSensor(CoordinatorEntity[BobsweepCoordinator], SensorEntity):
    """Base for the sensors fed by the robot's read-only DP 105 getters.

    These are not datapoints and they are not polled. The coordinator asks the
    robot once at startup (and on `bobsweep.refresh_robot_info`) using the exact
    frames the vendor app sends from its own named getters; the replies arrive
    asynchronously and `RobotInfoTracker` keeps the decoded result. So every one
    of these reads `unknown` until a reply has landed, which is a real state --
    the robot may simply not have answered.
    """

    _attr_has_entity_name = True

    def __init__(self, coordinator: BobsweepCoordinator, key: str, name: str, icon: str) -> None:
        """Wire up one robot-info sensor."""
        super().__init__(coordinator)
        self.entity_description = SensorEntityDescription(
            key=key,
            translation_key=key,
            name=name,
            icon=icon,
            entity_category=EntityCategory.DIAGNOSTIC,
        )
        self._attr_unique_id = f"{coordinator.device_id}_{key}"
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, coordinator.device_id)},
            manufacturer="bObsweep",
            model=coordinator.model_family,
            name=DEFAULT_NAME,
        )

    @property
    def _info(self) -> Any:
        """The tracker, or None on a coordinator that predates it."""
        return getattr(self.coordinator, "robot_info", None)


class BobsweepSavedMapsSensor(BobsweepRobotInfoSensor):
    """How many floor maps the robot has saved, with their ids and names.

    The names are the only free text this robot hands over locally, and they are
    the user's own from the app ("first floor v5"). Multi-map units keep several;
    a single-map unit reports one.
    """

    def __init__(self, coordinator: BobsweepCoordinator) -> None:
        """Initialize the saved-maps sensor."""
        super().__init__(coordinator, "saved_maps", "Saved maps", "mdi:map-outline")
        self._attr_state_class = SensorStateClass.MEASUREMENT

    @property
    def native_value(self) -> int | None:
        """How many saved maps the robot reported, or None if it has not."""
        info = self._info
        if info is None or info.saved_maps is None:
            return None
        return len(info.saved_maps)

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        """The maps as `{map_id, name}` entries."""
        info = self._info
        if info is None or info.saved_maps is None:
            return {}
        return {"maps": [saved_map.as_dict() for saved_map in info.saved_maps]}


class BobsweepSchedulesSensor(BobsweepRobotInfoSensor):
    """The robot's stored cleaning schedules, and the room names inside them.

    Worth more than it looks. Each entry carries the room ids it targets *and*
    the name the user gave it, so a single-room schedule is the robot telling
    you what that room is called -- the only local source of room names there
    is. `coordinator.room_names()` derives that map; the `room_names` attribute
    here shows the result.

    The state counts the schedules the robot holds, enabled or not; `enabled` on
    each entry says which are live.
    """

    def __init__(self, coordinator: BobsweepCoordinator) -> None:
        """Initialize the schedules sensor."""
        super().__init__(coordinator, "schedules", "Schedules", "mdi:calendar-clock")
        self._attr_state_class = SensorStateClass.MEASUREMENT

    @property
    def native_value(self) -> int | None:
        """How many schedules the robot reported, or None if it has not."""
        info = self._info
        if info is None or info.schedules is None:
            return None
        return len(info.schedules)

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        """The schedule list plus the room names derived from it."""
        info = self._info
        if info is None or info.schedules is None:
            return {}
        attrs: dict[str, Any] = {
            "schedules": [entry.as_dict() for entry in info.schedules],
            "enabled_count": sum(1 for entry in info.schedules if entry.enabled),
        }
        getter = getattr(self.coordinator, "room_names", None)
        if callable(getter):
            try:
                attrs["room_names"] = dict(getter())
            except Exception:  # noqa: BLE001 - an attribute must never raise
                _LOGGER.debug("bObsweep: room_names() failed", exc_info=True)
        return attrs


class BobsweepMopClothSensor(BobsweepRobotInfoSensor):
    """How dirty the robot reckons its mop cloth is, as a percentage."""

    def __init__(self, coordinator: BobsweepCoordinator) -> None:
        """Initialize the mop-cloth sensor."""
        super().__init__(
            coordinator, "mop_cloth_dirt", "Mop cloth dirt", "mdi:water-percent"
        )
        self._attr_native_unit_of_measurement = PERCENTAGE
        self._attr_state_class = SensorStateClass.MEASUREMENT

    @property
    def native_value(self) -> int | None:
        """The dirt percentage, or None if the robot has not reported one."""
        info = self._info
        return None if info is None else info.mop_cloth_percent


async def async_setup_entry(
    hass: HomeAssistant,
    entry: BobsweepConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up bObsweep sensors from a config entry."""
    coordinator = entry.runtime_data
    spec = coordinator.spec

    entities: list[SensorEntity] = []
    for description in SENSOR_DESCRIPTIONS:
        dp = getattr(spec, description.dp_attr)
        if dp is None:
            # This family has no such datapoint — don't publish a dead entity.
            _LOGGER.debug(
                "Skipping bObsweep sensor %s: family %s has no %s",
                description.key,
                spec.key,
                description.dp_attr,
            )
            continue
        entities.append(BobsweepSensor(coordinator, description, dp))

    # The named-fault sensor exists whenever the family has an error datapoint
    # *and* a transcribed bit table to name its bits with.
    if any(
        getattr(spec, dp_attr) is not None and getattr(spec, table_attr)
        for dp_attr, table_attr, _kind in FAULT_CHANNELS
    ):
        entities.append(BobsweepFaultSensor(coordinator))
    else:
        _LOGGER.debug(
            "Skipping bObsweep fault sensor: family %s has no fault table", spec.key
        )

    # The AI obstacle detector rides on the transportation datapoint, so the
    # sensor exists exactly when the family has one (SLAM today).
    if spec.dp_transportation is not None:
        entities.append(BobsweepObstaclesSensor(coordinator))
    else:
        _LOGGER.debug(
            "Skipping bObsweep obstacle sensor: family %s has no transportation "
            "datapoint",
            spec.key,
        )

    # Room selections ride on the same transportation datapoint, but the sensor
    # also needs the coordinator to actually own a tracker: a coordinator from
    # an older install would otherwise take the whole platform setup down.
    if spec.dp_transportation is not None and (
        getattr(coordinator, "room_selection", None) is not None
    ):
        entities.append(BobsweepSelectedRoomsSensor(coordinator))
    else:
        _LOGGER.debug(
            "Skipping bObsweep selected-rooms sensor: family %s has no "
            "transportation datapoint or no room-selection tracker",
            spec.key,
        )

    # The path trail is DP 104, and likewise only if the tracker exists.
    if spec.dp_path_data is not None and getattr(coordinator, "trail", None) is not None:
        entities.append(BobsweepPathTrailSensor(coordinator))
    else:
        _LOGGER.debug(
            "Skipping bObsweep path-trail sensor: family %s has no path "
            "datapoint or no trail tracker",
            spec.key,
        )

    # The robot-info sensors ride on the same transportation datapoint and the
    # coordinator's tracker. They are created before any reply has arrived --
    # the getters go out ~10 s after setup -- so they start out unknown by
    # design rather than being withheld until the robot answers.
    if spec.dp_transportation is not None and (
        getattr(coordinator, "robot_info", None) is not None
    ):
        entities.append(BobsweepSavedMapsSensor(coordinator))
        entities.append(BobsweepSchedulesSensor(coordinator))
        entities.append(BobsweepMopClothSensor(coordinator))
    else:
        _LOGGER.debug(
            "Skipping bObsweep robot-info sensors: family %s has no "
            "transportation datapoint or no robot-info tracker",
            spec.key,
        )

    # Room awareness is additive and family-independent: the sensor reports
    # `unknown` until a position source is verified (see position.py).
    entities.append(BobsweepCurrentRoomSensor(coordinator))
    if not coordinator.position_source.available:
        _LOGGER.debug(
            "bObsweep current_room sensor will stay unknown: %s",
            coordinator.position_source.unavailable_reason,
        )

    async_add_entities(entities)
