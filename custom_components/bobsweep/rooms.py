"""Room classification, hysteresis and frame-drift (staleness) detection.

Two layers, split so the interesting half needs no Home Assistant at all:

* `RoomClassifier` and `DockDriftMonitor` are pure state machines. They take
  samples in and hand decisions back; they own no I/O, no clock they cannot be
  handed, and no HA objects, so they can be exercised offline.
* `RoomTracker` is the glue: it pulls a fix from the `PositionSource`, feeds the
  state machines, fires the `bobsweep_room_changed` event, raises and clears the
  staleness repair issue, and holds the capture buffers the services write into.

**Hysteresis is not a nicety.** A robot crossing a doorway produces positions on
both sides of the boundary within one poll interval, and SLAM jitter moves a
stationary robot by a few cells. Without hysteresis `sensor.<name>_current_room`
would flip several times per doorway transit, and every flip is a state write
plus a recorder row plus any automation that triggers on room changes. So:

* entering a *different* named zone needs N consecutive agreeing samples;
* demoting to `unmapped` needs M consecutive out-of-zone samples, with M > N so
  the robot can cross an unzoned hallway or doorway without the room flipping;
* `last_known_room` is never cleared by `unmapped` or by an unknown position.

That last rule is the one that matters operationally. A robot that has jammed
under the sofa stops producing new positions -- exactly when you most want to
know which room it is in. "Which room did it get stuck in" is a question about
the last room that was *confirmed*, so `last_known_room` is sticky until some
other room is confirmed.

**Staleness.** The dock does not move. Its observed map cell is therefore a
fingerprint of the coordinate frame, and if it shifts, every stored polygon is
measuring the wrong floor. When that happens the sensor goes `unknown` and a
repair issue is raised -- it does *not* keep reporting a room name. Confidently
reporting the wrong room into an automation ("vacuum finished the nursery") is
far worse than reporting nothing.
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Iterable, Mapping

from .geometry import Point, density_footprint, distance, median_point
from .position import PositionSource, RobotPosition
from .zones import SOURCE_TAUGHT, Zone, ZoneSet, new_zone

_LOGGER = logging.getLogger(__name__)

#: Sensor state for "the robot's position is known, and it is in no taught zone".
#: Distinct from `None`/unknown, which means the position is not known at all.
#: Automations need to tell those apart -- one says "somewhere unmapped", the
#: other says "no position source / frame is untrustworthy".
ROOM_UNMAPPED = "unmapped"

#: Consecutive agreeing samples before a *different* named zone is confirmed.
DEFAULT_ENTER_SAMPLES = 3
#: Consecutive out-of-zone samples before demoting to `unmapped`. Larger than
#: the enter threshold on purpose: crossing an unzoned doorway must not flip the
#: room, but entering a genuinely different room should be quick.
DEFAULT_EXIT_SAMPLES = 5

#: Statuses during which the robot's own frame is being rebuilt or re-solved.
#: Classification is suppressed entirely (state frozen, counters reset) rather
#: than fed garbage, because positions reported mid-relocalisation can land
#: anywhere on the map.
SUPPRESS_STATUSES = frozenset(
    {"relocalizing", "map_operation", "map_housekeeping", "quick_map"}
)

#: Statuses that mean "physically on the dock", so a fix taken now is a
#: measurement of where the dock is in the current frame.
DOCK_STATUSES = frozenset({"charging", "charge_done"})

#: How many dock observations to keep, and how many are needed before their
#: median is trusted. GUESS: 9/3 -- enough to shrug off a single bad fix,
#: short enough to notice a real frame shift within a few charge cycles.
DOCK_SAMPLE_WINDOW = 9
DOCK_MIN_SAMPLES = 3

#: Dock movement, in map cells, beyond which the zone set is declared stale.
#: GUESS: 20 cells. The verified DP 105 no-go rectangles span hundreds of cells
#: per side, so 20 is small against a room and large against SLAM jitter -- but
#: it has not been validated against a real drift event.
DEFAULT_DOCK_DRIFT_CELLS = 20.0

#: Event fired on every confirmed room transition.
EVENT_ROOM_CHANGED = "bobsweep_room_changed"

#: Repair-issue id prefix for a stale zone set.
ISSUE_ZONES_STALE = "zones_stale"


def _utcnow() -> datetime:
    """Timezone-aware now, kept local so the pure layer needs no HA."""
    return datetime.now(timezone.utc)


@dataclass(frozen=True)
class RoomTransition:
    """A confirmed change of room, as fired on the event bus."""

    from_room: str | None
    to_room: str | None
    x: float | None = None
    y: float | None = None

    def as_event_data(self) -> dict[str, Any]:
        """The event payload: `{from, to, x, y}`."""
        return {
            "from": self.from_room,
            "to": self.to_room,
            "x": self.x,
            "y": self.y,
        }


class RoomClassifier:
    """Turns a stream of zone hits into a stable room state.

    Feed it one `update()` per coordinator poll. It returns a `RoomTransition`
    only when the state actually changed -- callers use that to fire the event
    and nothing else.
    """

    def __init__(
        self,
        *,
        enter_samples: int = DEFAULT_ENTER_SAMPLES,
        exit_samples: int = DEFAULT_EXIT_SAMPLES,
    ) -> None:
        """Configure the two hysteresis thresholds."""
        self.enter_samples = max(1, enter_samples)
        self.exit_samples = max(1, exit_samples)

        #: Confirmed state: a zone name, `ROOM_UNMAPPED`, or None for unknown.
        self.room: str | None = None
        #: Sticky: the last zone name that was confirmed. Never cleared by
        #: `unmapped` or by an unknown position -- see the module docstring.
        self.last_known_room: str | None = None
        #: When `room` last changed.
        self.since: datetime | None = None

        self._candidate: str | None = None
        self._candidate_count = 0
        self._out_count = 0

    # --- internals -----------------------------------------------------------
    def _reset_counters(self) -> None:
        """Drop any partially-accumulated evidence."""
        self._candidate = None
        self._candidate_count = 0
        self._out_count = 0

    def _set(
        self,
        room: str | None,
        *,
        x: float | None,
        y: float | None,
        now: datetime | None,
    ) -> RoomTransition | None:
        """Commit a new confirmed state, returning the transition if it changed."""
        if room == self.room:
            return None
        transition = RoomTransition(from_room=self.room, to_room=room, x=x, y=y)
        self.room = room
        self.since = now or _utcnow()
        if room is not None and room != ROOM_UNMAPPED:
            self.last_known_room = room
        self._reset_counters()
        return transition

    # --- the state machine ---------------------------------------------------
    def update(
        self,
        zone_name: str | None,
        *,
        position_available: bool = True,
        suppressed: bool = False,
        stale: bool = False,
        x: float | None = None,
        y: float | None = None,
        now: datetime | None = None,
    ) -> RoomTransition | None:
        """Feed one sample in; return a transition if the state changed.

        `zone_name` is the resolved zone for this sample, or None when the
        position is known but falls in no zone. `position_available` False means
        there was no fix at all this tick -- a different condition, and the
        reason the two are separate arguments rather than one nullable one.
        """
        if stale:
            # The frame itself is untrustworthy: report nothing rather than a
            # room name that can no longer be trusted.
            return self._set(None, x=x, y=y, now=now)

        if suppressed:
            # Hold the current state and forget partial evidence; samples taken
            # during relocalisation are not evidence of anything.
            self._reset_counters()
            return None

        if not position_available:
            return self._set(None, x=None, y=None, now=now)

        if zone_name is not None:
            self._out_count = 0
            if zone_name == self.room:
                # Already there: nothing pending, nothing to confirm.
                self._candidate = None
                self._candidate_count = 0
                return None
            if zone_name != self._candidate:
                self._candidate = zone_name
                self._candidate_count = 1
            else:
                self._candidate_count += 1
            if self._candidate_count >= self.enter_samples:
                return self._set(zone_name, x=x, y=y, now=now)
            return None

        # Position known, in no zone.
        self._candidate = None
        self._candidate_count = 0
        if self.room == ROOM_UNMAPPED:
            return None
        self._out_count += 1
        if self._out_count >= self.exit_samples:
            return self._set(ROOM_UNMAPPED, x=x, y=y, now=now)
        return None

    def as_dict(self) -> dict[str, Any]:
        """Diagnostic snapshot of the machine's internal counters."""
        return {
            "room": self.room,
            "last_known_room": self.last_known_room,
            "candidate": self._candidate,
            "candidate_count": self._candidate_count,
            "out_count": self._out_count,
            "since": self.since.isoformat() if self.since else None,
        }


class DockDriftMonitor:
    """Detects coordinate-frame drift by watching where the dock appears.

    The dock is bolted to the floor; if the cell it reports as "here while
    charging" moves, the map origin has moved. Uses a median over a short
    window so a single bad relocalisation fix cannot raise a false alarm, and
    requires a minimum sample count before it will answer at all.
    """

    def __init__(
        self,
        *,
        window: int = DOCK_SAMPLE_WINDOW,
        min_samples: int = DOCK_MIN_SAMPLES,
        threshold: float = DEFAULT_DOCK_DRIFT_CELLS,
    ) -> None:
        """Configure the sample window and the drift threshold in map cells."""
        self._samples: deque[Point] = deque(maxlen=max(1, window))
        self.min_samples = max(1, min_samples)
        self.threshold = threshold

    def observe(self, point: Point) -> None:
        """Record one dock-position observation (caller checks the status DP)."""
        self._samples.append((point[0], point[1]))

    @property
    def sample_count(self) -> int:
        """How many observations are currently in the window."""
        return len(self._samples)

    @property
    def observed(self) -> Point | None:
        """Median of the window, or None until there are enough samples."""
        if len(self._samples) < self.min_samples:
            return None
        return median_point(self._samples)

    def drift(self, reference: Point | None) -> float | None:
        """Distance in map cells between the stored reference and what is observed."""
        observed = self.observed
        if observed is None or reference is None:
            return None
        return distance(observed, reference)

    def is_stale(self, reference: Point | None) -> bool:
        """True when the dock has moved far enough to distrust every zone.

        Returns False when there is no reference or not enough samples: "cannot
        tell" is not "it is broken", and raising a repair issue on
        insufficient evidence would train the user to ignore it.
        """
        drift = self.drift(reference)
        return drift is not None and drift > self.threshold

    def reset(self) -> None:
        """Forget the window (after the reference is re-established)."""
        self._samples.clear()


@dataclass
class CapturePoint:
    """One manually captured fix, from `bobsweep.capture_point`."""

    label: str
    x: float
    y: float
    at: str

    def as_dict(self) -> dict[str, Any]:
        """Service-response representation."""
        return {"label": self.label, "x": self.x, "y": self.y, "at": self.at}


@dataclass
class ActiveCapture:
    """An in-flight `start_zone_capture` session."""

    name: str
    started: datetime
    points: list[Point] = field(default_factory=list)
    # Samples where the position source returned nothing, for the report.
    misses: int = 0


def derive_zone(
    name: str, points: Iterable[Point], **kwargs: Any
) -> tuple[Zone | None, Any]:
    """Derive a zone footprint from captured points via density binning.

    Returns `(zone, footprint)`; `zone` is None when the capture was too sparse
    or too diffuse to keep any bin. See `geometry.density_footprint` for why
    this is binning and not a convex hull.
    """
    footprint = density_footprint(list(points), **kwargs)
    if footprint is None:
        return None, None
    return new_zone(name, footprint.polygon, source=SOURCE_TAUGHT), footprint


class RoomTracker:
    """Home Assistant glue around the classifier, drift monitor and captures.

    One per config entry, owned by the coordinator and updated once per poll.
    Everything it does is safe when `position_source.available` is False, which
    is the shipping configuration today: it takes no fixes, confirms no rooms,
    raises no repair issues, and reports `None`.
    """

    def __init__(
        self,
        hass: Any,
        *,
        zone_store: Any,
        position_source: PositionSource,
        status_dp: str | None,
        entry_id: str,
        device_id: str,
    ) -> None:
        """Wire the tracker to its config entry's store and position source."""
        self.hass = hass
        self.zone_store = zone_store
        self.position_source = position_source
        self.status_dp = status_dp
        self.entry_id = entry_id
        self.device_id = device_id

        self.classifier = RoomClassifier()
        self.dock_monitor = DockDriftMonitor()

        self.position: RobotPosition | None = None
        self.stale = False
        self.scratch: list[CapturePoint] = []
        self.capture: ActiveCapture | None = None

        self._issue_active = False

    # --- convenience ---------------------------------------------------------
    @property
    def zones(self) -> ZoneSet:
        """The live zone set."""
        return self.zone_store.data

    @property
    def available(self) -> bool:
        """True when a position source exists that could ever produce a fix."""
        return self.position_source.available

    @property
    def unavailable_reason(self) -> str:
        """Why capture services will refuse, when they refuse."""
        return self.position_source.unavailable_reason

    async def async_get_position(
        self, dps: Mapping[str, Any] | None = None
    ) -> RobotPosition | None:
        """One-shot fix, used by `capture_point`. None is an ordinary answer."""
        try:
            return await self.position_source.async_get_position(dps or {})
        except Exception:  # noqa: BLE001 - never break a poll over position
            _LOGGER.debug("bObsweep: position source raised", exc_info=True)
            return None

    # --- the per-poll update -------------------------------------------------
    async def async_update(self, dps: Mapping[str, Any]) -> None:
        """Take a fix, classify it, and maintain staleness. Never raises."""
        try:
            await self._async_update(dps)
        except Exception:  # noqa: BLE001 - room awareness must not break polling
            _LOGGER.exception("bObsweep: room tracking update failed")

    async def _async_update(self, dps: Mapping[str, Any]) -> None:
        """Inner update; see `async_update`."""
        status = dps.get(self.status_dp) if self.status_dp else None
        suppressed = status in SUPPRESS_STATUSES

        position = None
        if self.position_source.available:
            position = await self.async_get_position(dps)
        self.position = position

        # Dock observations only mean anything while the robot is on the dock.
        if position is not None and status in DOCK_STATUSES:
            self.dock_monitor.observe(position.point)
            await self._async_check_staleness()

        if self.capture is not None and not suppressed:
            if position is None:
                self.capture.misses += 1
            else:
                self.capture.points.append(position.point)

        zone: Zone | None = None
        if position is not None and not self.stale:
            zone = self.zones.resolve(position.x, position.y)

        transition = self.classifier.update(
            zone.name if zone else None,
            position_available=position is not None,
            suppressed=suppressed,
            stale=self.stale,
            x=position.x if position else None,
            y=position.y if position else None,
        )
        self._current_zone = zone
        if transition is not None:
            self._fire(transition)

    def _fire(self, transition: RoomTransition) -> None:
        """Fire `bobsweep_room_changed` for a confirmed transition.

        Fired for every confirmed change, including changes *to* `unmapped` and
        to unknown (`to: null`) -- an automation that wants "left the kitchen"
        needs those as much as it needs the arrivals.
        """
        data = transition.as_event_data()
        data["device_id"] = self.device_id
        data["entry_id"] = self.entry_id
        try:
            self.hass.bus.async_fire(EVENT_ROOM_CHANGED, data)
        except Exception:  # noqa: BLE001
            _LOGGER.debug("bObsweep: failed to fire room-changed event", exc_info=True)

    # --- staleness -----------------------------------------------------------
    async def _async_check_staleness(self) -> None:
        """Compare the observed dock cell with the stored reference."""
        reference = self.zones.dock_ref
        if reference is None:
            # No reference yet: adopt the current observation once the median
            # has settled, but only if there is something to protect. With no
            # zones there is no frame worth pinning.
            observed = self.dock_monitor.observed
            if observed is not None and self.zones.zones:
                self.zones.dock_ref = observed
                await self.zone_store.async_save()
            return

        stale = self.dock_monitor.is_stale(reference)
        if stale == self.stale:
            return
        self.stale = stale
        if stale:
            drift = self.dock_monitor.drift(reference)
            _LOGGER.warning(
                "bObsweep: dock has moved %.1f map cells from its reference "
                "position; the stored zones no longer describe this map and "
                "room reporting has been suspended",
                drift or 0.0,
            )
            self._async_create_issue(drift or 0.0)
        else:
            self._async_delete_issue()

    def _async_create_issue(self, drift: float) -> None:
        """Raise the repair issue for a stale zone set."""
        try:
            from homeassistant.helpers import issue_registry as ir  # noqa: PLC0415

            ir.async_create_issue(
                self.hass,
                "bobsweep",
                f"{ISSUE_ZONES_STALE}_{self.entry_id}",
                is_fixable=False,
                severity=ir.IssueSeverity.WARNING,
                translation_key=ISSUE_ZONES_STALE,
                translation_placeholders={
                    "drift": f"{drift:.0f}",
                    "threshold": f"{self.dock_monitor.threshold:.0f}",
                },
            )
            self._issue_active = True
        except Exception:  # noqa: BLE001 - a repair is advisory, never fatal
            _LOGGER.debug("bObsweep: could not raise stale-zones issue", exc_info=True)

    def _async_delete_issue(self) -> None:
        """Clear the repair issue once the dock is back where it belongs."""
        if not self._issue_active:
            return
        try:
            from homeassistant.helpers import issue_registry as ir  # noqa: PLC0415

            ir.async_delete_issue(
                self.hass, "bobsweep", f"{ISSUE_ZONES_STALE}_{self.entry_id}"
            )
        except Exception:  # noqa: BLE001
            _LOGGER.debug("bObsweep: could not clear stale-zones issue", exc_info=True)
        self._issue_active = False

    def clear_stale(self) -> None:
        """Drop the stale flag and re-pin the dock reference to what is observed now.

        Called after the user re-teaches or re-imports zones: the new set is by
        definition measured in the current frame.
        """
        self.stale = False
        self.dock_monitor.reset()
        self._async_delete_issue()

    # --- state surface for the sensor ---------------------------------------
    _current_zone: Zone | None = None

    @property
    def current_zone(self) -> Zone | None:
        """The zone the most recent *sample* fell in (pre-hysteresis)."""
        return self._current_zone

    @property
    def state(self) -> str | None:
        """Confirmed room name, `unmapped`, or None for unknown."""
        return self.classifier.room

    def attributes(self) -> dict[str, Any]:
        """Attribute payload for `sensor.<name>_current_room`."""
        zone = self._current_zone
        position = self.position
        return {
            "x": position.x if position else None,
            "y": position.y if position else None,
            "last_known_room": self.classifier.last_known_room,
            "zone_source": zone.source if zone else None,
            "room_id": zone.room_id if zone else None,
            "map_id": self.zones.map_id,
            "stale": self.stale,
            "since": (
                self.classifier.since.isoformat() if self.classifier.since else None
            ),
            "position_source": self.position_source.key,
            # Which source actually produced *this* fix. With a composite source
            # `position_source` is only "composite", so the useful provenance --
            # exact trail point vs. approximate obstacle sighting -- lives here.
            "position_fix_source": position.source if position is not None else None,
            "zone_count": len(self.zones.zones),
            # Provenance of the fix behind this state. Both are None for the
            # null source; an approximate, event-driven source (see
            # `AiObjectSightingPositionSource`) fills them in so nobody reads a
            # room name without seeing how good and how old the evidence is.
            # `position_age` deliberately outlives `position`, which is None on
            # every poll that produced no new fix.
            "position_approximate": (
                position.approximate if position is not None else None
            ),
            "position_age": getattr(self.position_source, "last_fix_age", None),
        }
