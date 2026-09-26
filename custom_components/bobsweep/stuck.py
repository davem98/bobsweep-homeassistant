"""Stuck detection with a best-effort location, for bObsweep robots.

**What the robot actually reports when it gets stuck** (measured on the
validation unit, 2026-09-02): the error bitmask DP 18 went `4096 -> 6144`
(bit 11 `TroubleBobStuck` joined bit 12 `TroubleChargingStation`), the status
DP went to `standby` -- *not* an error status -- and the enable DP dropped to
`False`. No DP 105 frame arrived, and no datapoint anywhere carried the robot's
position or room at that moment. The only positional evidence was the DP 104
path trail, whose last point had landed 0.37 s before the fault -- and the
trail flows **only while the vendor app's map screen is open**.

So a stuck alert can always say *that* the robot is stuck and *which* fault
says so, but *where* is best-effort. This module is built around being honest
about that: every location field on a `StuckEvent` carries its provenance
(which source, exact or approximate, how old), and the human sentence built by
`describe()` says "position unknown" in so many words when that is the truth.

The module is HA-free. `StuckMonitor` is a pure state machine fed once per
ingest by the coordinator; `StuckAlerter` at the bottom is the thin glue that
turns its transitions into a bus event and a persistent notification, and it
imports Home Assistant lazily so this file stays importable (and testable) with
no HA installed -- the same split `rooms.py` and `zones.py` use.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable, Iterable, Mapping, Sequence

_LOGGER = logging.getLogger(__name__)

#: Fault slugs (see `faults.slugify_trouble`) that mean the robot cannot move
#: on its own and needs a human. Each entry is a judgement about what the
#: vendor's trouble key describes, not a measurement -- with one exception:
#:
#: * ``bob_stuck`` -- SLAM bit 11 / Vision bit 14 `TroubleBobStuck`. The robot's
#:   own verdict that it is stuck. **This is the one slug that has been seen on
#:   hardware** (the 2026-09-02 event above); everything below is inferred from
#:   the key's name and the app's guidance text for it.
#: * ``wheel``, ``wheel_left``, ``wheel_right`` -- a drive wheel is jammed or
#:   suspended (lifted off the floor / hanging over an edge). Either way the
#:   robot halts and will not drive until it is picked up and put down.
#: * ``wheel_sensor`` -- Random bit 13: the wheel's drop sensor, i.e. the
#:   wheel-suspended condition on a family that reports it separately.
#: * ``bumper``, ``bumper_left``, ``bumper_right``, ``lidar_bumper`` -- a bumper
#:   held pressed, which is what a robot wedged under furniture reports. The
#:   lidar bumper (SLAM bit 21) is the turret's own bumper: the robot has driven
#:   its lidar under something and cannot back out.
#: * ``edge_sensors``, ``edge_sensor_front``, ``edge_sensor_left``,
#:   ``edge_sensor_right`` -- the cliff sensors. Reported when the robot has
#:   stopped at a drop it cannot get away from, or is dangling with a sensor
#:   over the edge. (Also reported for a dirty sensor, which is a false
#:   positive this module accepts: it still stops the robot.)
#: * ``congestion`` -- SLAM DP 113 bit 8 `TroubleCongestion`: the robot reports
#:   itself boxed in / trapped, unable to find a way out.
#:
#: Deliberately **not** included: ``localization``, ``positioning`` and
#: ``navigation`` (the robot has lost its bearings but can still drive, and
#: usually recovers), ``charging_station`` (about the dock, and set alongside
#: `bob_stuck` in the measured event without meaning anything of its own),
#: battery and consumable faults, and every mop / dock / water fault.
STUCK_FAULTS: frozenset[str] = frozenset(
    {
        "bob_stuck",
        "wheel",
        "wheel_left",
        "wheel_right",
        "wheel_sensor",
        "bumper",
        "bumper_left",
        "bumper_right",
        "lidar_bumper",
        "edge_sensors",
        "edge_sensor_front",
        "edge_sensor_left",
        "edge_sensor_right",
        "congestion",
    }
)

#: Oldest position fix a stuck event will still report as the location. Older
#: than this and the robot has almost certainly driven somewhere else since;
#: the event then says "position unknown" and keeps the stale age on
#: `stale_fix_age` so a reader can see there *was* a fix, just not a usable
#: one. Ten minutes is a guess sized to a typical single-room clean; it is
#: not measured.
MAX_POSITION_AGE_SECONDS = 600.0

#: Faults that do **not** end a job, so they never open a ``"fault"`` event on
#: their own: the robot keeps driving through them (and usually clears them).
#: ``charging_station`` is set alongside `bob_stuck` in the measured 2026-09-02
#: event, 16 s *before* it, during the return trip -- alerting on it would
#: pre-empt the real stuck alert with the wrong fault. The three bearings
#: faults are excluded for the reason given under `STUCK_FAULTS`. A fault
#: that is not here and not a stuck slug is assumed to stop the robot, which
#: is the safe default: a missed alert is the failure this exists to fix.
NON_STOPPING_FAULTS: frozenset[str] = frozenset(
    {"charging_station", "localization", "positioning", "navigation"}
)


def _utcnow() -> datetime:
    """Timezone-aware UTC now; a function so tests can patch it."""
    return datetime.now(timezone.utc)


def stuck_faults_in(faults: Iterable[str]) -> tuple[str, ...]:
    """The stuck slugs present in an active-fault list, in report order."""
    return tuple(slug for slug in faults if slug in STUCK_FAULTS)


def stopping_faults_in(faults: Iterable[str]) -> tuple[str, ...]:
    """The slugs that end a job: everything but `NON_STOPPING_FAULTS`."""
    return tuple(slug for slug in faults if slug not in NON_STOPPING_FAULTS)


# --- the event ---------------------------------------------------------------
@dataclass(frozen=True)
class PositionFix:
    """A position offered to the monitor, with the provenance it must carry.

    `age_seconds` is how old the fix was *at the moment of the fault*, which is
    the number a human wants ("0.4 s before the fault") and the number the
    max-age cut-off is applied to. None means the source does not track age;
    such a fix is accepted and reported with `age_seconds: null`.
    """

    x: float
    y: float
    #: Which source produced it, e.g. `dp_path_trail` or `ai_object_sighting`.
    source: str
    approximate: bool = False
    age_seconds: float | None = None


@dataclass(frozen=True)
class StuckEvent:
    """One transition into a stranded state, frozen at the moment it happened.

    Two kinds, on one channel because a person wants one alert either way:

    * ``"stuck"`` -- a slug from `STUCK_FAULTS` appeared: the robot says it
      cannot move.
    * ``"fault"`` -- any other real fault appeared while a job was open and the
      robot was off the dock. It can move, but it will not: the job is over and
      it is sitting wherever it was. Measured 2026-09-26: a `side_brush`
      (jammed side brush) nine minutes into a scheduled clean left the robot
      asleep on the floor for hours with nothing reporting it, because a brush
      jam is not "stuck". Faults while docked (a full bin, a dry tank) are not
      this: the robot is home.
    """

    #: The slug that triggered the event (the first stuck slug, or the first
    #: fault for a ``"fault"`` kind).
    primary: str
    #: Every active fault slug at the time, stuck or not, in report order.
    faults: tuple[str, ...]
    #: The status DP value at the time (`standby` in the measured event).
    status: str | None
    #: ``"stuck"`` or ``"fault"``; see the class docstring.
    kind: str = "stuck"
    #: The rooms the current job was told to clean, if known.
    #: Each is `{"id": int, "name": str}`; the name falls back to `Room <id>`.
    rooms: tuple[Mapping[str, Any], ...] = ()
    #: How the rooms are known: ``"ack"`` (the robot's 0x22), ``"schedule"``
    #: (the job started on one of the robot's stored schedules), or None.
    rooms_source: str | None = None
    #: The matched schedule's name and `HH:MM`, when `rooms_source` is
    #: ``"schedule"``.
    schedule: str | None = None
    schedule_time: str | None = None
    #: Last known position, or None when there is no usable fix.
    position: PositionFix | None = None
    #: Age of a fix that existed but was too old to report, else None.
    stale_fix_age: float | None = None
    #: The taught zone containing `position`, if any.
    room: str | None = None
    #: The closest reported obstacle to `position`, or None.
    nearest_obstacle: Mapping[str, Any] | None = None
    #: When the transition was detected (UTC, tz-aware).
    at: datetime = field(default_factory=_utcnow)

    @property
    def message(self) -> str:
        """The human sentence for this event."""
        return describe(self)

    def as_dict(self) -> dict[str, Any]:
        """Attribute/event-friendly form. Plain JSON-safe data only."""
        position = None
        if self.position is not None:
            position = {
                "x": self.position.x,
                "y": self.position.y,
                "source": self.position.source,
                "approximate": self.position.approximate,
                "age_seconds": (
                    None
                    if self.position.age_seconds is None
                    else round(self.position.age_seconds, 1)
                ),
            }
        return {
            "kind": self.kind,
            "primary": self.primary,
            "faults": list(self.faults),
            "status": self.status,
            "rooms": [dict(room) for room in self.rooms],
            "rooms_source": self.rooms_source,
            "schedule": self.schedule,
            "schedule_time": self.schedule_time,
            "position": position,
            "stale_fix_age": (
                None if self.stale_fix_age is None else round(self.stale_fix_age, 1)
            ),
            "room": self.room,
            "nearest_obstacle": (
                None if self.nearest_obstacle is None else dict(self.nearest_obstacle)
            ),
            "at": self.at.isoformat(timespec="seconds"),
            "message": self.message,
        }


# --- the sentence ------------------------------------------------------------
def _join_names(names: Sequence[str]) -> str:
    """`Studio` / `Studio and Pantry` / `Studio, Pantry and Nook`."""
    names = [str(name) for name in names]
    if not names:
        return ""
    if len(names) == 1:
        return names[0]
    return ", ".join(names[:-1]) + " and " + names[-1]


def _age_text(seconds: float) -> str:
    """`0.4 s`, `12 s`, `3.5 min`, `1.2 h` -- short enough for a notification."""
    if seconds < 10:
        return f"{seconds:.1f} s"
    if seconds < 120:
        return f"{seconds:.0f} s"
    if seconds < 7200:
        return f"{seconds / 60:.1f} min"
    return f"{seconds / 3600:.1f} h"


def describe(event: StuckEvent) -> str:
    """Build the one-line human description of a stuck event.

    Pure: the same event always gives the same sentence. Every locational claim
    in it is qualified -- exact vs approximate, how old -- and when nothing
    locational is known the sentence says so rather than trailing off.

    Example: ``Stuck (bob_stuck) while cleaning Studio and Pantry; in Studio;
    last known position (1922, 1233), 0.4 s before the fault (exact); nearest
    obstacle: shoes, 140 cells away``. A ``"fault"`` kind opens with
    ``Stopped by a side_brush fault while cleaning Loft (the 10:30 "weekday"
    schedule)`` instead.
    """
    parts: list[str] = []

    if event.kind == "fault":
        head = f"Stopped by a {event.primary} fault"
    else:
        head = f"Stuck ({event.primary})"
    if event.rooms:
        head += " while cleaning " + _join_names(
            [str(room.get("name", f"Room {room.get('id')}")) for room in event.rooms]
        )
    if event.rooms_source == "schedule":
        label = f'the {event.schedule_time} "{event.schedule}" schedule' if event.schedule else f"the {event.schedule_time} schedule"
        head += (" (" if event.rooms else " during ") + label + (")" if event.rooms else "")
    parts.append(head)

    if event.room:
        parts.append(f"in {event.room}")

    position = event.position
    if position is not None:
        text = f"last known position ({position.x:.0f}, {position.y:.0f})"
        if position.age_seconds is not None:
            text += f", {_age_text(position.age_seconds)} before the fault"
        text += " (approximate)" if position.approximate else " (exact)"
        parts.append(text)
        if event.nearest_obstacle is not None:
            obstacle = event.nearest_obstacle
            parts.append(
                f"nearest obstacle: {obstacle.get('class')}, "
                f"{float(obstacle.get('distance', 0.0)):.0f} cells away"
            )
    elif event.stale_fix_age is not None:
        parts.append(
            "position unknown (last fix was "
            f"{_age_text(event.stale_fix_age)} old, too old to trust)"
        )
    else:
        parts.append("position unknown (no map session was open)")

    return "; ".join(parts)


# --- the monitor -------------------------------------------------------------
def nearest_obstacle(
    objects: Iterable[Any], x: float, y: float
) -> dict[str, Any] | None:
    """The closest obstacle to `(x, y)`, as `{class, x, y, distance}`.

    `objects` is anything with `x`, `y` and `class_name` (`transport.AiObject`).
    Distance is Euclidean, in map cells -- the same frame as the position and
    the zones, so it is directly comparable to `DEFAULT_DEDUP_TOLERANCE`.
    """
    best: dict[str, Any] | None = None
    for obj in objects:
        try:
            distance = math.hypot(float(obj.x) - x, float(obj.y) - y)
        except (AttributeError, TypeError, ValueError):
            continue
        if best is None or distance < best["distance"]:
            best = {
                "class": getattr(obj, "class_name", "unknown"),
                "x": obj.x,
                "y": obj.y,
                "distance": round(distance, 1),
            }
    return best


class StuckMonitor:
    """Detects the transition into (and out of) a stranded state.

    Fed once per ingest with the decoded `FaultReport` and whatever context is
    to hand. It looks only at *edges*. Two edges open an event (see
    `StuckEvent` for why both matter): a stuck slug appearing where none was
    active; or, with `job_open` and the status off the dock, any real fault
    appearing where none was active. The faults all dropping, or the status
    landing in a docked state, clears it. A fault that stays set for an hour
    produces exactly one event, which is what a notification wants.

    `active` is the event for the current state or None; `last` is the most
    recent event whether or not it has cleared, so a sensor can keep showing
    "the last time" after the robot is back on the dock.
    """

    def __init__(
        self,
        *,
        docked_statuses: Iterable[str] = (),
        max_position_age: float = MAX_POSITION_AGE_SECONDS,
    ) -> None:
        """Set the family's docked statuses and the fix age cut-off."""
        self.docked_statuses = frozenset(docked_statuses)
        self.max_position_age = max_position_age
        self.active: StuckEvent | None = None
        self.last: StuckEvent | None = None
        self.events_seen = 0

    def update(
        self,
        faults: Iterable[str],
        *,
        status: Any = None,
        job_open: bool = False,
        room_ids: Sequence[int] | None = None,
        room_names: Mapping[int, str] | None = None,
        rooms_source: str | None = None,
        schedule: str | None = None,
        schedule_time: str | None = None,
        position: PositionFix | None = None,
        resolve_zone: Callable[[float, float], str | None] | None = None,
        objects: Iterable[Any] | None = None,
        now: datetime | None = None,
    ) -> str | None:
        """Feed one observation. Returns `"stuck"`, `"fault"`, `"cleared"`, or None.

        `faults` is `FaultReport.faults` (active slugs, notes excluded).
        `job_open` says a cleaning job has begun and not been followed by a
        return to the dock (the coordinator's job tracker). `position` is the
        best fix the caller has, *with its age*; the cut-off is applied here.
        `resolve_zone(x, y)` names the taught zone at a point, or None.
        `objects` is the current obstacle list, newest first.
        """
        faults = tuple(faults)
        active_stuck = stuck_faults_in(faults)
        stopping = stopping_faults_in(faults)
        status_text = None if status is None else str(status)
        docked = status_text in self.docked_statuses

        # A stuck slug arriving while a lesser "fault" event is active upgrades
        # it: the stuck alert is the precise one and must not be masked by a
        # brush jam reported a moment earlier. It counts as a new event.
        upgrade = (
            self.active is not None and self.active.kind == "fault" and bool(active_stuck)
        )

        if self.active is None or upgrade:
            if active_stuck:
                kind, primary = "stuck", active_stuck[0]
            elif stopping and job_open and not docked:
                kind, primary = "fault", stopping[0]
            else:
                return None
            self.active = self._build_event(
                kind=kind,
                primary=primary,
                faults=faults,
                status=status_text,
                room_ids=room_ids,
                room_names=room_names or {},
                rooms_source=rooms_source,
                schedule=schedule,
                schedule_time=schedule_time,
                position=position,
                resolve_zone=resolve_zone,
                objects=objects,
                now=now,
            )
            self.last = self.active
            self.events_seen += 1
            return kind

        # Already alerting: clear on the faults dropping, or on the robot
        # turning up on the dock (someone carried it back and the bit lags).
        # A "stuck" event clears as soon as the stuck slug itself is gone, even
        # if a lesser fault remains -- that is the old behaviour, kept.
        gone = not active_stuck if self.active.kind == "stuck" else not stopping
        if gone or docked:
            self.active = None
            return "cleared"
        return None

    def _build_event(
        self,
        *,
        kind: str,
        primary: str,
        faults: tuple[str, ...],
        status: str | None,
        room_ids: Sequence[int] | None,
        room_names: Mapping[int, str],
        rooms_source: str | None,
        schedule: str | None,
        schedule_time: str | None,
        position: PositionFix | None,
        resolve_zone: Callable[[float, float], str | None] | None,
        objects: Iterable[Any] | None,
        now: datetime | None,
    ) -> StuckEvent:
        """Freeze everything known about the moment into an event."""
        rooms = tuple(
            {"id": int(room_id), "name": room_names.get(int(room_id), f"Room {int(room_id)}")}
            for room_id in (room_ids or [])
        )

        usable: PositionFix | None = None
        stale_age: float | None = None
        if position is not None:
            if position.age_seconds is None or position.age_seconds <= self.max_position_age:
                usable = position
            else:
                stale_age = position.age_seconds

        room: str | None = None
        nearest: dict[str, Any] | None = None
        if usable is not None:
            if resolve_zone is not None:
                try:
                    room = resolve_zone(usable.x, usable.y)
                except Exception:  # noqa: BLE001 - never break ingest over a lookup
                    _LOGGER.debug("bObsweep: zone lookup failed", exc_info=True)
            if objects:
                nearest = nearest_obstacle(objects, usable.x, usable.y)

        return StuckEvent(
            kind=kind,
            primary=primary,
            faults=faults,
            status=status,
            rooms=rooms,
            rooms_source=rooms_source if rooms or rooms_source == "schedule" else None,
            schedule=schedule,
            schedule_time=schedule_time,
            position=usable,
            stale_fix_age=stale_age,
            room=room,
            nearest_obstacle=nearest,
            at=now or _utcnow(),
        )

    def as_attributes(self) -> dict[str, Any]:
        """The active-or-last event as attributes, plus the count."""
        event = self.active or self.last
        attrs: dict[str, Any] = {"active": self.active is not None, "events_seen": self.events_seen}
        if event is not None:
            attrs.update(event.as_dict())
        return attrs


# --- Home Assistant glue -----------------------------------------------------
EVENT_STUCK = "bobsweep_stuck"
EVENT_STUCK_CLEARED = "bobsweep_stuck_cleared"
NOTIFICATION_TITLE = "bObsweep is stuck"
NOTIFICATION_TITLE_FAULT = "bObsweep stopped: {fault}"


def _job_rooms(coordinator: Any) -> Any | None:
    """The current job's rooms (`insights.JobRooms`), if known for *this* job.

    The 0x22 ack is remembered until the next one, so a selection from last
    Tuesday's room clean must not be pinned on today's whole-house run; and a
    job the robot started on its own schedule has no ack at all, so the
    schedule table is the fallback. `insights.rooms_for_job` applies both
    rules; the job tracker's start time is already in local time.
    """
    jobs = getattr(coordinator, "jobs", None)
    robot_info = getattr(coordinator, "robot_info", None)
    try:
        from .insights import rooms_for_job  # noqa: PLC0415

        return rooms_for_job(
            getattr(coordinator, "room_selection", None),
            jobs,
            getattr(robot_info, "schedules", None),
        )
    except Exception:  # noqa: BLE001
        _LOGGER.debug("bObsweep: job room lookup failed", exc_info=True)
        return None


def position_fix_from(coordinator: Any) -> PositionFix | None:
    """Best available fix from the coordinator's position machinery.

    In order: the room tracker's last accepted fix (already filtered for
    freshness by its source), then the raw path-trail tracker's last point --
    which outlives the tracker's 90 s freshness window, and is exactly the
    evidence the measured stuck event had. Each comes with its own age.
    """
    rooms = getattr(coordinator, "rooms", None)
    position = getattr(rooms, "position", None)
    if position is not None:
        return PositionFix(
            x=float(position.x),
            y=float(position.y),
            source=str(getattr(position, "source", "unknown")),
            approximate=bool(getattr(position, "approximate", False)),
            age_seconds=getattr(position, "age", None),
        )
    trail = getattr(coordinator, "trail", None)
    point = getattr(trail, "last_point", None)
    if point is not None:
        return PositionFix(
            x=float(point[0]),
            y=float(point[1]),
            source="dp_path_trail",
            approximate=False,
            age_seconds=getattr(trail, "age", None),
        )
    return None


class StuckAlerter:
    """Feeds the monitor from a coordinator and reacts to its transitions.

    Owned by the coordinator, called once per ingest *after* the per-DP
    consumers (it needs the merged snapshot and the trackers' latest state).
    On `stuck` it fires `bobsweep_stuck` and creates a persistent notification;
    on `cleared` it fires `bobsweep_stuck_cleared` and dismisses it. Nothing in
    here may raise into the ingest path, so each HA call is wrapped separately
    -- a missing `persistent_notification` component costs the notification,
    not the event and never the ingest.
    """

    def __init__(self, hass: Any, *, entry_id: str, device_id: str, monitor: StuckMonitor) -> None:
        """Bind to one entry."""
        self.hass = hass
        self.entry_id = entry_id
        self.device_id = device_id
        self.monitor = monitor
        self.notification_id = f"bobsweep_stuck_{entry_id}"

    def observe(self, coordinator: Any, snapshot: Mapping[str, Any]) -> str | None:
        """One ingest's worth of observation. Never raises."""
        try:
            return self._observe(coordinator, snapshot)
        except Exception:  # noqa: BLE001 - never break ingest
            _LOGGER.debug("bObsweep: stuck monitor failed", exc_info=True)
            return None

    def _observe(self, coordinator: Any, snapshot: Mapping[str, Any]) -> str | None:
        from .faults import decode_faults  # noqa: PLC0415 - sibling, kept lazy for symmetry

        spec = coordinator.spec
        report = decode_faults(spec, dict(snapshot))
        status = snapshot.get(spec.dp_status) if spec.dp_status else None

        room_names: Mapping[int, str] = {}
        getter = getattr(coordinator, "room_names", None)
        if callable(getter):
            room_names = getter()

        rooms = getattr(coordinator, "rooms", None)
        zones = getattr(rooms, "zones", None)
        resolve_zone = None
        if zones is not None:

            def resolve_zone(x: float, y: float, _zones: Any = zones) -> str | None:
                zone = _zones.resolve(x, y)
                return None if zone is None else zone.name

        ai_objects = getattr(coordinator, "ai_objects", None)
        objects = getattr(ai_objects, "objects", None) or []

        jobs = getattr(coordinator, "jobs", None)
        found = _job_rooms(coordinator)

        transition = self.monitor.update(
            report.faults,
            status=status,
            job_open=getattr(jobs, "job", None) is not None,
            room_ids=None if found is None else list(found.room_ids),
            room_names=room_names,
            rooms_source=None if found is None else found.source,
            schedule=None if found is None else found.schedule,
            schedule_time=None if found is None else found.schedule_time,
            position=position_fix_from(coordinator),
            resolve_zone=resolve_zone,
            objects=objects,
        )
        if transition in ("stuck", "fault") and self.monitor.active is not None:
            self._announce(self.monitor.active)
        elif transition == "cleared":
            self._clear()
        return transition

    def _event_data(self, event: StuckEvent) -> dict[str, Any]:
        data = event.as_dict()
        data["device_id"] = self.device_id
        data["entry_id"] = self.entry_id
        return data

    def _announce(self, event: StuckEvent) -> None:
        """Fire the event and raise the notification. Each step guarded."""
        try:
            self.hass.bus.async_fire(EVENT_STUCK, self._event_data(event))
        except Exception:  # noqa: BLE001
            _LOGGER.debug("bObsweep: failed to fire stuck event", exc_info=True)
        try:
            from homeassistant.components import persistent_notification  # noqa: PLC0415

            persistent_notification.async_create(
                self.hass,
                event.message,
                title=(
                    NOTIFICATION_TITLE_FAULT.format(fault=event.primary)
                    if event.kind == "fault"
                    else NOTIFICATION_TITLE
                ),
                notification_id=self.notification_id,
            )
        except Exception:  # noqa: BLE001 - the component may be absent
            _LOGGER.debug("bObsweep: could not create stuck notification", exc_info=True)

    def _clear(self) -> None:
        """Fire the cleared event and dismiss the notification."""
        event = self.monitor.last
        data: dict[str, Any] = (
            self._event_data(event) if event is not None
            else {"device_id": self.device_id, "entry_id": self.entry_id}
        )
        try:
            self.hass.bus.async_fire(EVENT_STUCK_CLEARED, data)
        except Exception:  # noqa: BLE001
            _LOGGER.debug("bObsweep: failed to fire stuck-cleared event", exc_info=True)
        try:
            from homeassistant.components import persistent_notification  # noqa: PLC0415

            persistent_notification.async_dismiss(self.hass, self.notification_id)
        except Exception:  # noqa: BLE001
            _LOGGER.debug("bObsweep: could not dismiss stuck notification", exc_info=True)
