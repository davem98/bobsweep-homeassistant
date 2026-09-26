"""Obstacle insights: a persisted history of what the robot's camera found.

The robot's `eAiObjectToAPP` (0x37) frames tell us what it saw and where, in
raw map cells (see `transport.AiObjectTracker`). That tracker deliberately
forgets everything at the end of a session; this module is the long memory.
Every genuinely-new sighting becomes an `ObstacleRecord` stamped with the
wall-clock time, the taught zone it fell in (if any), the rooms the current
job was told to clean (from the 0x22 ack for *this* job, or -- since the
robot's own scheduled cleans send no ack -- from the stored schedule whose
weekday and time the job started on; see `rooms_for_job`), and a
job id. The records are kept in a bounded, persisted history and a handful of
pure functions turn them into the views a dashboard wants: counts by class,
this job's tally, per-job summaries, and *hotspots* -- places where things
keep turning up, clean after clean.

**Job boundaries are inferred, not reported.** The robot has no "job started"
datapoint. A job begins here when the status DP enters one of the family's
cleaning statuses from a *parked* one: docked/charging, or the family's idle
statuses (`standby`, `sleep`, `idle`, `clean_finish`). Every other status is
treated as *within* a job -- `paused`, a mid-job `mop_wash`, `relocalizing`,
`dustbin_emptying`, the return trip -- so a mop-wash break does not split one
clean into three. A cleaning status seen with no prior status at all (startup
mid-job) also opens a job, honestly stamped with startup time rather than the
unknown real start. The rule lives in `JobTracker.JOB_START_FROM_STATUSES`.

Layering, as in the rest of the integration: everything above the
`ObstacleHistoryStore` line is HA-free and exercised by
`research/verify_insights.py`; the store and the `ObstacleInsightsRecorder`
glue import Home Assistant lazily.

Storage key: `bobsweep_obstacles.<entry_id>`. On-disk schema (version 1)::

    {"version": 1, "records": [{"at": "<iso>", "class": "shoes", "x": 1, "y": 2,
                                "room": "Studio" | null,
                                "job_rooms": ["Studio", "Pantry"] | null,
                                "job": "<iso>" | null}, ...]}
"""

from __future__ import annotations

import logging
import math
import time
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from typing import Any, Callable, Iterable, Mapping, Sequence

_LOGGER = logging.getLogger(__name__)

STORAGE_VERSION = 1
STORAGE_KEY_TEMPLATE = "bobsweep_obstacles.{entry_id}"

#: How many records the history keeps. Oldest are dropped first. A busy home
#: produces perhaps ten new sightings a clean, so this is months of history in
#: a file that stays well under 100 kB.
MAX_RECORDS = 1000

#: Debounce for persisting after a new sighting. Sightings arrive in bursts at
#: the start of a job; one write per burst is plenty.
SAVE_DELAY_SECONDS = 10.0

#: Cluster radius for `hotspots()`, in map cells. Twice the tracker's
#: same-object tolerance (80): a hotspot is "the same patch of floor", not "the
#: same object", and the robot's own coordinate refinement between jobs is
#: larger than within one. A guess sized from one capture, like the tolerance.
DEFAULT_HOTSPOT_RADIUS = 150.0

#: A 0x22 room-selection ack arrives when the app sends the room-clean command,
#: which is *before* the status flips to a cleaning value. A selection this
#: much older than the job start is still taken as belonging to the job.
SELECTION_LEAD_SECONDS = 120.0


class InsightsError(ValueError):
    """Stored obstacle data could not be accepted."""


def _utcnow() -> datetime:
    """Timezone-aware UTC now; a function so tests can inject their own."""
    return datetime.now(timezone.utc)


def _iso(when: datetime) -> str:
    return when.isoformat(timespec="seconds")


# --- records -----------------------------------------------------------------
@dataclass(frozen=True)
class ObstacleRecord:
    """One genuinely-new obstacle sighting, with everything known at the time."""

    #: UTC ISO-8601 when it was noticed (wall clock -- this is history).
    at: str
    #: Vendor class name (`wire`, `shoes`, ..., `unknown`, `class_<n>`).
    class_name: str
    x: int
    y: int
    #: The taught zone containing (x, y) at the time, or None.
    room: str | None = None
    #: Names of the rooms the job was told to clean, if a selection is known.
    job_rooms: tuple[str, ...] | None = None
    #: The job id (ISO time the job began), or None if no job was open.
    job: str | None = None

    def to_dict(self) -> dict[str, Any]:
        """Plain JSON-safe form."""
        return {
            "at": self.at,
            "class": self.class_name,
            "x": self.x,
            "y": self.y,
            "room": self.room,
            "job_rooms": None if self.job_rooms is None else list(self.job_rooms),
            "job": self.job,
        }

    @classmethod
    def from_dict(cls, raw: Any) -> ObstacleRecord:
        """Parse one stored record, raising `InsightsError` if unusable."""
        if not isinstance(raw, dict):
            raise InsightsError(f"record must be a mapping, got {type(raw).__name__}")
        try:
            at = str(raw["at"])
            class_name = str(raw["class"])
            x = int(raw["x"])
            y = int(raw["y"])
        except (KeyError, TypeError, ValueError) as err:
            raise InsightsError(f"record is missing or mistyped a field: {err}") from err
        room = raw.get("room")
        if room is not None and not isinstance(room, str):
            raise InsightsError("record 'room' must be a string or null")
        job_rooms_raw = raw.get("job_rooms")
        job_rooms: tuple[str, ...] | None
        if job_rooms_raw is None:
            job_rooms = None
        elif isinstance(job_rooms_raw, list):
            job_rooms = tuple(str(name) for name in job_rooms_raw)
        else:
            raise InsightsError("record 'job_rooms' must be a list or null")
        job = raw.get("job")
        if job is not None and not isinstance(job, str):
            raise InsightsError("record 'job' must be a string or null")
        return cls(
            at=at, class_name=class_name, x=x, y=y,
            room=room, job_rooms=job_rooms, job=job,
        )


@dataclass
class ObstacleHistory:
    """A bounded, oldest-first list of records. Pure data."""

    records: list[ObstacleRecord] = field(default_factory=list)
    max_records: int = MAX_RECORDS

    def append(self, record: ObstacleRecord) -> None:
        """Add the newest record, dropping the oldest past the bound."""
        self.records.append(record)
        overflow = len(self.records) - self.max_records
        if overflow > 0:
            del self.records[:overflow]

    def clear(self) -> None:
        """Forget everything."""
        self.records.clear()

    @property
    def last(self) -> ObstacleRecord | None:
        """The newest record, or None."""
        return self.records[-1] if self.records else None

    def to_dict(self) -> dict[str, Any]:
        """Plain JSON-safe snapshot, materialised now."""
        return {
            "version": STORAGE_VERSION,
            "records": [record.to_dict() for record in self.records],
        }

    @classmethod
    def from_dict(cls, raw: Any, *, max_records: int = MAX_RECORDS) -> ObstacleHistory:
        """Parse a stored document. `None` (no file yet) is an empty history."""
        if raw is None:
            return cls(max_records=max_records)
        if not isinstance(raw, dict):
            raise InsightsError(f"obstacle data must be a mapping, got {type(raw).__name__}")
        version = raw.get("version", STORAGE_VERSION)
        if isinstance(version, int) and version > STORAGE_VERSION:
            raise InsightsError(
                f"obstacle data is version {version}, this integration understands "
                f"up to version {STORAGE_VERSION}"
            )
        records_raw = raw.get("records", [])
        if not isinstance(records_raw, list):
            raise InsightsError("'records' must be a list")
        history = cls(max_records=max_records)
        for item in records_raw:
            history.append(ObstacleRecord.from_dict(item))
        return history


# --- jobs --------------------------------------------------------------------
class JobTracker:
    """Infers cleaning-job boundaries from the status datapoint.

    See the module docstring for the rule. `job` is the current job id (the
    ISO time it began) while a job is open and None otherwise; `started_mono`
    is the matching `time.monotonic()` so other trackers' monotonic stamps can
    be compared against it.
    """

    #: Statuses that count as "parked": a cleaning status seen right after one
    #: of these starts a new job. Family docked statuses are added on top.
    JOB_START_FROM_STATUSES: frozenset[str] = frozenset(
        {"standby", "sleep", "idle", "clean_finish"}
    )

    def __init__(
        self,
        *,
        cleaning_statuses: Iterable[str],
        docked_statuses: Iterable[str] = (),
    ) -> None:
        """Take the family's status vocabulary."""
        self.cleaning = frozenset(cleaning_statuses)
        self.start_from = self.JOB_START_FROM_STATUSES | frozenset(docked_statuses)
        self.job: str | None = None
        self.started_mono: float | None = None
        #: The previous job's `started_mono`, so an ack that belonged to it can
        #: be told from one that belongs to the job that followed.
        self.previous_started_mono: float | None = None
        #: When the current job began, as the tz-aware datetime the caller
        #: supplied (Home Assistant passes local time, which is what a
        #: schedule match needs). None when no job is open.
        self.started_at: datetime | None = None
        self.last_status: str | None = None
        self.jobs_seen = 0

    def observe(
        self, status: Any, *, now: datetime | None = None, mono: float | None = None
    ) -> str | None:
        """Feed one status value. Returns a new job id when one begins."""
        text = None if status is None else str(status)
        previous = self.last_status
        self.last_status = text
        if text is None or text not in self.cleaning:
            return None
        if previous is not None and previous in self.cleaning:
            return None
        if previous is not None and previous not in self.start_from:
            # paused / mop_wash / relocalizing / returning -> still the same job
            # -- unless no job is open at all, in which case we are seeing the
            # tail of something we missed and a job is better than none.
            if self.job is not None:
                return None
        started = now or _utcnow()
        self.started_at = started
        # The id stays UTC whatever zone the caller's clock is in, so records
        # written under different HA time zones still group by job.
        self.job = _iso(started.astimezone(timezone.utc) if started.tzinfo else started)
        self.previous_started_mono = self.started_mono
        self.started_mono = time.monotonic() if mono is None else mono
        self.jobs_seen += 1
        return self.job


def selection_for_job(selection: Any, jobs: JobTracker | None) -> list[int] | None:
    """Room ids from a `RoomSelectionTracker`, if the ack belongs to this job.

    The ack is remembered until the next one, so without this rule a room
    selection from a previous day would be attached to every later job. Two
    tests: the ack must not predate this job's start by more than the lead
    window (the ack precedes the status flip), and it must postdate the
    *previous* job's start -- otherwise a job that begins within the lead
    window of the last one (a schedule firing right after a short room clean)
    inherits that clean's rooms. With no job tracker the selection is taken
    at face value.
    """
    room_ids = getattr(selection, "room_ids", None)
    if not room_ids:
        return None
    if jobs is None:
        return list(room_ids)
    observed = getattr(selection, "observed_at", None)
    started = jobs.started_mono
    if jobs.job is None or observed is None or started is None:
        return None
    if observed < started - SELECTION_LEAD_SECONDS:
        return None
    previous = jobs.previous_started_mono
    if previous is not None and observed <= previous:
        return None
    return list(room_ids)


#: How far a job's start may sit from a schedule's HH:MM and still be that
#: schedule. Measured 2026-09-26: five scheduled jobs each began 20-25 s
#: *before* their nominal minute (the robot rounds its clock its own way), so
#: the window is symmetric and generous; two schedules within six minutes of
#: each other would be the only ambiguity, and the nearer one wins.
SCHEDULE_MATCH_SECONDS = 180.0


@dataclass(frozen=True)
class JobRooms:
    """Which rooms the current job is cleaning, and how that is known.

    `source` is ``"ack"`` when the robot acknowledged a room-clean command for
    this job (0x22, the only direct evidence), or ``"schedule"`` when the job
    began at the time and weekday of one of the robot's own stored schedules.
    **Scheduled cleans do not emit the ack** (measured 2026-09-26: five in a
    row, none acked), so without the schedule route every job the robot starts
    on its own would have no rooms at all.
    """

    room_ids: tuple[int, ...]
    source: str
    schedule: str | None = None
    #: `HH:MM` of the matched schedule, for the sentence.
    schedule_time: str | None = None


def match_schedule(
    schedules: Iterable[Any] | None,
    started_local: datetime,
    *,
    tolerance: float = SCHEDULE_MATCH_SECONDS,
) -> Any | None:
    """The enabled schedule whose weekday and time best fit a job start.

    `schedules` are `transport.ScheduleEntry` (or anything with `enabled`,
    `days_mask`, `hour`, `minute`); `started_local` must be wall-clock time in
    the robot's own zone -- the schedule store holds local HH:MM and a
    Monday-first weekday mask, so comparing against UTC would be off by the
    zone. A job that started within `tolerance` of a matching schedule's minute
    is that schedule; the closest wins if several fit. None when nothing fits,
    which is the right answer for a job started by hand.
    """
    if not schedules:
        return None
    weekday_bit = 1 << started_local.weekday()  # Monday = bit 0, as on the wire
    best: Any | None = None
    best_gap = tolerance
    for entry in schedules:
        try:
            if not entry.enabled or not (int(entry.days_mask) & weekday_bit):
                continue
            nominal = started_local.replace(
                hour=int(entry.hour), minute=int(entry.minute), second=0, microsecond=0
            )
        except (AttributeError, TypeError, ValueError):
            continue
        gap = abs((started_local - nominal).total_seconds())
        if gap <= best_gap:
            best, best_gap = entry, gap
    return best


def rooms_for_job(
    selection: Any,
    jobs: JobTracker | None,
    schedules: Iterable[Any] | None = None,
    *,
    local_tz: Any = None,
) -> JobRooms | None:
    """Rooms for the open job: the 0x22 ack if there is one, else a schedule.

    `local_tz` converts the job's start to the robot's wall clock for the
    schedule match; with None the start is used as given (callers that already
    pass local time to `JobTracker.observe` need nothing more).
    """
    ids = selection_for_job(selection, jobs)
    if ids:
        return JobRooms(room_ids=tuple(int(i) for i in ids), source="ack")
    if jobs is None or jobs.job is None or jobs.started_at is None:
        return None
    started = jobs.started_at
    if local_tz is not None and started.tzinfo is not None:
        started = started.astimezone(local_tz)
    entry = match_schedule(schedules, started)
    if entry is None:
        return None
    return JobRooms(
        room_ids=tuple(int(i) for i in getattr(entry, "room_ids", ()) or ()),
        source="schedule",
        schedule=str(getattr(entry, "name", "") or "") or None,
        schedule_time=f"{int(entry.hour):02d}:{int(entry.minute):02d}",
    )


# --- derived views (pure) ----------------------------------------------------
def by_class(records: Iterable[ObstacleRecord]) -> dict[str, int]:
    """Count records per class, most common first."""
    counts = Counter(record.class_name for record in records)
    return dict(sorted(counts.items(), key=lambda item: (-item[1], item[0])))


def current_job(records: Iterable[ObstacleRecord], job: str | None) -> list[ObstacleRecord]:
    """The records belonging to `job` (none if `job` is None)."""
    if job is None:
        return []
    return [record for record in records if record.job == job]


def recent(
    records: Iterable[ObstacleRecord], days: float = 30, *, now: datetime | None = None
) -> list[ObstacleRecord]:
    """Records from the last `days` days. Unparseable timestamps are excluded."""
    cutoff = (now or _utcnow()) - timedelta(days=days)
    kept: list[ObstacleRecord] = []
    for record in records:
        try:
            when = datetime.fromisoformat(record.at)
        except ValueError:
            continue
        if when.tzinfo is None:
            when = when.replace(tzinfo=timezone.utc)
        if when >= cutoff:
            kept.append(record)
    return kept


def job_summaries(records: Iterable[ObstacleRecord]) -> list[dict[str, Any]]:
    """Per-job `{job, count, by_class, rooms}`, newest job first.

    Records with no job are grouped under `job: null` at the end.
    """
    groups: dict[str | None, list[ObstacleRecord]] = {}
    for record in records:
        groups.setdefault(record.job, []).append(record)
    summaries: list[dict[str, Any]] = []
    for job in sorted((j for j in groups if j is not None), reverse=True):
        members = groups[job]
        rooms: tuple[str, ...] | None = next(
            (m.job_rooms for m in members if m.job_rooms is not None), None
        )
        summaries.append(
            {
                "job": job,
                "count": len(members),
                "by_class": by_class(members),
                "rooms": None if rooms is None else list(rooms),
            }
        )
    if None in groups:
        members = groups[None]
        summaries.append(
            {"job": None, "count": len(members), "by_class": by_class(members), "rooms": None}
        )
    return summaries


def hotspots(
    records: Iterable[ObstacleRecord],
    radius: float = DEFAULT_HOTSPOT_RADIUS,
    *,
    min_count: int = 2,
) -> list[dict[str, Any]]:
    """Places where obstacles keep turning up, most repeated first.

    Greedy single-pass clustering: each record joins the first existing cluster
    whose running centre is within `radius` cells, otherwise starts one. Order
    of input therefore matters slightly at the margins, which is acceptable for
    a "where do things pile up" view and keeps it O(n * clusters). Each cluster
    reports `{x, y, count, jobs, class, room, classes}`: the centroid, how many
    sightings, how many distinct jobs contributed, the dominant class and room.
    Only clusters with at least `min_count` sightings are hotspots.
    """
    clusters: list[dict[str, Any]] = []
    for record in records:
        for cluster in clusters:
            if math.hypot(record.x - cluster["_sx"] / cluster["count"],
                          record.y - cluster["_sy"] / cluster["count"]) <= radius:
                cluster["_sx"] += record.x
                cluster["_sy"] += record.y
                cluster["count"] += 1
                cluster["_classes"][record.class_name] += 1
                if record.room is not None:
                    cluster["_rooms"][record.room] += 1
                if record.job is not None:
                    cluster["_jobs"].add(record.job)
                break
        else:
            clusters.append(
                {
                    "_sx": record.x,
                    "_sy": record.y,
                    "count": 1,
                    "_classes": Counter({record.class_name: 1}),
                    "_rooms": Counter({record.room: 1} if record.room is not None else {}),
                    "_jobs": {record.job} if record.job is not None else set(),
                }
            )

    result: list[dict[str, Any]] = []
    for cluster in clusters:
        if cluster["count"] < min_count:
            continue
        result.append(
            {
                "x": round(cluster["_sx"] / cluster["count"]),
                "y": round(cluster["_sy"] / cluster["count"]),
                "count": cluster["count"],
                "jobs": len(cluster["_jobs"]),
                "class": cluster["_classes"].most_common(1)[0][0],
                "room": (
                    cluster["_rooms"].most_common(1)[0][0] if cluster["_rooms"] else None
                ),
                "classes": dict(cluster["_classes"].most_common()),
            }
        )
    result.sort(key=lambda h: (-h["count"], -h["jobs"], h["x"], h["y"]))
    return result


def build_record(
    obj: Any,
    *,
    resolve_zone: Callable[[float, float], str | None] | None = None,
    job_rooms: Sequence[str] | None = None,
    job: str | None = None,
    now: datetime | None = None,
) -> ObstacleRecord:
    """Turn a `transport.AiObject` (anything with x, y, class_name) into a record."""
    x = int(obj.x)
    y = int(obj.y)
    room: str | None = None
    if resolve_zone is not None:
        try:
            room = resolve_zone(float(x), float(y))
        except Exception:  # noqa: BLE001 - a lookup must not lose the record
            _LOGGER.debug("bObsweep: zone lookup failed for obstacle", exc_info=True)
    return ObstacleRecord(
        at=_iso(now or _utcnow()),
        class_name=str(getattr(obj, "class_name", "unknown")),
        x=x,
        y=y,
        room=room,
        job_rooms=None if job_rooms is None else tuple(str(name) for name in job_rooms),
        job=job,
    )


def insights_attributes(
    history: ObstacleHistory,
    job: str | None,
    *,
    now: datetime | None = None,
    top_hotspots: int = 10,
) -> dict[str, Any]:
    """The `obstacle_insights` sensor's attribute payload, all pure."""
    records = history.records
    summaries = job_summaries(records)
    return {
        "job": job,
        "by_class_job": by_class(current_job(records, job)),
        "by_class_30d": by_class(recent(records, 30, now=now)),
        "hotspots": hotspots(records)[:top_hotspots],
        "jobs_recorded": sum(1 for s in summaries if s["job"] is not None),
        "last_job": next((s for s in summaries if s["job"] is not None), None),
        "total_recorded": len(records),
    }


# --- Home Assistant glue -----------------------------------------------------
EVENT_OBSTACLE_DETECTED = "bobsweep_obstacle_detected"


class ObstacleHistoryStore:
    """`homeassistant.helpers.storage.Store` wrapper around an `ObstacleHistory`.

    Mirrors `zones.ZoneStore`: thin, HA-aware only in the constructor, and
    every save hands the store an already-materialised plain dict built on the
    event loop. Saves are debounced with `async_delay_save`; the callable it is
    given returns the snapshot taken at scheduling time, so nothing it runs
    touches the live history.
    """

    def __init__(self, hass: Any, entry_id: str) -> None:
        """Create (but do not yet load) the store for one config entry."""
        from homeassistant.helpers.storage import Store  # noqa: PLC0415

        self._store: Any = Store(
            hass, STORAGE_VERSION, STORAGE_KEY_TEMPLATE.format(entry_id=entry_id)
        )
        self.data = ObstacleHistory()
        self._snapshot: dict[str, Any] | None = None

    async def async_load(self) -> ObstacleHistory:
        """Load the history, degrading to an empty one on unusable data."""
        raw = await self._store.async_load()
        try:
            self.data = ObstacleHistory.from_dict(raw)
        except InsightsError as err:
            _LOGGER.error(
                "bObsweep: stored obstacle history is unusable (%s); continuing "
                "with none. The file has been left in place",
                err,
            )
            self.data = ObstacleHistory()
        return self.data

    def schedule_save(self) -> None:
        """Persist soon (debounced). Safe to call on every new record."""
        self._snapshot = self.data.to_dict()
        try:
            self._store.async_delay_save(self._take_snapshot, SAVE_DELAY_SECONDS)
        except Exception:  # noqa: BLE001 - never break ingest over a save
            _LOGGER.debug("bObsweep: could not schedule obstacle save", exc_info=True)

    def _take_snapshot(self) -> dict[str, Any]:
        """The data callable for `async_delay_save`: plain data, nothing live."""
        return self._snapshot if self._snapshot is not None else self.data.to_dict()

    async def async_save(self) -> None:
        """Persist immediately."""
        await self._store.async_save(self.data.to_dict())

    async def async_clear(self) -> None:
        """Forget every record and persist that."""
        self.data.clear()
        self._snapshot = None
        await self.async_save()

    async def async_remove(self) -> None:
        """Delete the backing file (used when the config entry is removed)."""
        await self._store.async_remove()


class ObstacleInsightsRecorder:
    """Watches the coordinator's obstacle tracker and records each new sighting.

    **It does not consume sightings.** `AiObjectTracker.consume_new_sighting()`
    is a one-shot hand-off owned by the opt-in `AiObjectSightingPositionSource`;
    taking it here would silently switch that source off. Instead the tracker
    counts sightings (`sightings_seen`) and this recorder remembers how many it
    has already seen, reading `last_sighting` when the count moves. Ingest
    handles one DP 105 value per message, so the count moves by at most one
    per call and nothing is skipped.

    Called once per ingest after the per-DP consumers, like `StuckAlerter`.
    Never raises.
    """

    def __init__(
        self, hass: Any, *, entry_id: str, device_id: str, store: ObstacleHistoryStore
    ) -> None:
        """Bind to one entry."""
        self.hass = hass
        self.entry_id = entry_id
        self.device_id = device_id
        self.store = store
        self._seen = 0

    @property
    def history(self) -> ObstacleHistory:
        """The live history."""
        return self.store.data

    def observe(self, coordinator: Any, snapshot: Mapping[str, Any]) -> ObstacleRecord | None:
        """One ingest's worth of observation. Never raises."""
        try:
            return self._observe(coordinator, snapshot)
        except Exception:  # noqa: BLE001 - never break ingest
            _LOGGER.debug("bObsweep: obstacle insights failed", exc_info=True)
            return None

    def _observe(self, coordinator: Any, snapshot: Mapping[str, Any]) -> ObstacleRecord | None:
        tracker = getattr(coordinator, "ai_objects", None)
        if tracker is None:
            return None
        seen = getattr(tracker, "sightings_seen", 0)
        if seen <= self._seen:
            # Also handles the tracker being reset (count went backwards).
            self._seen = min(self._seen, seen)
            return None
        self._seen = seen
        sighting = tracker.last_sighting
        if sighting is None:
            return None

        jobs = getattr(coordinator, "jobs", None)
        job = getattr(jobs, "job", None)

        job_rooms: list[str] | None = None
        robot_info = getattr(coordinator, "robot_info", None)
        found = rooms_for_job(
            getattr(coordinator, "room_selection", None),
            jobs,
            getattr(robot_info, "schedules", None),
        )
        if found is not None and found.room_ids:
            names: Mapping[int, str] = {}
            getter = getattr(coordinator, "room_names", None)
            if callable(getter):
                names = getter()
            job_rooms = [names.get(room_id, f"Room {room_id}") for room_id in found.room_ids]

        rooms = getattr(coordinator, "rooms", None)
        zones = getattr(rooms, "zones", None)
        resolve_zone = None
        if zones is not None:

            def resolve_zone(x: float, y: float, _zones: Any = zones) -> str | None:
                zone = _zones.resolve(x, y)
                return None if zone is None else zone.name

        record = build_record(
            sighting.obj, resolve_zone=resolve_zone, job_rooms=job_rooms, job=job
        )
        self.history.append(record)
        self.store.schedule_save()

        data = record.to_dict()
        data["device_id"] = self.device_id
        data["entry_id"] = self.entry_id
        try:
            self.hass.bus.async_fire(EVENT_OBSTACLE_DETECTED, data)
        except Exception:  # noqa: BLE001
            _LOGGER.debug("bObsweep: failed to fire obstacle event", exc_info=True)
        return record
