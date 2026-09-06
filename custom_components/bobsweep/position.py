"""Robot position acquisition -- the one place that knows where the robot is.

**Read this before touching anything else in the room-awareness stack.**

**DP 104 is a real position datapoint with a working decoder, and this module
reads it passively.** The robot emits `cmd:102` replies -- `{"cmd":102,"data":
{"pathid":829,"type":2,"count":649,"curnums":2,"startno":647,"point":[[x,y],
...]}}` -- as ordinary DP 104 updates whenever a map session is running, which
in practice means whenever the vendor app's map screen is open. Points are
unscaled map cells in the same frame as DP 105 geometry; the feed is
incremental (`startno` + `curnums`, running `count`) and the `pathid` changes
per job. **Eliciting the trail ourselves is still unproven** -- writes of DP 106
and of `startno` requests, in every order, with the robot docked and the app
closed, drew no reply on 2026-09-05, consistent with the robot only ever
emitting *new* points. So `PathTrailTracker` and `PathTrailPositionSource`
below **never write DP 104**: they listen, accumulate, and go quiet when the
feed does. A live-clean elicitation test is still outstanding.

So position lives behind this interface and nowhere else. Zones, the room
classifier, the capture services, staleness detection and the segment API are
all built against `PositionSource`, and all of them remain correct when the
answer is permanently `None` (`NullPositionSource`) -- which is still what a
family without DP 104 gets.

Two real sources exist. `PathTrailPositionSource` is **exact** but only fresh
while the trail is flowing. `AiObjectSightingPositionSource` infers a rough
position from where the robot spotted an obstacle; it is **opt-in, off by
default**, and its docstring is emphatic about what it is not. When both are
configured, `CompositePositionSource` asks them in that order. See
`create_position_source()`.

Nothing outside this module should mention DP 104, `startno`, the 0.1 scale, or
any other property of a particular source.

**Units contract.** `async_get_position()` returns coordinates in *raw map
cells* -- the same frame DP 105 geometry uses (signed int16, origin-relative,
y-axis inverted). A source that receives coordinates in some other unit is
responsible for converting on the way out; every consumer may assume map cells.
This module deliberately imports nothing from Home Assistant, so the whole
position path can be exercised offline.
"""

from __future__ import annotations

import base64
import binascii
import json
import logging
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Sequence

from .const import FamilySpec
from .transport import AiObjectTracker

_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class RobotPosition:
    """One position fix, in raw map-cell units.

    `map_id` is the robot's map identity when the source can determine it (multi-floor
    homes save several maps). None means "unknown", not "map zero" -- a zone set
    recorded against a known map must not be matched against an unknown one by
    accident, so the comparison is always explicit at the call site.
    """

    x: float
    y: float
    map_id: int | None = None
    # Which source produced this fix, for diagnostics/attributes.
    source: str = "unknown"
    # True when the source is honest that this is a rough indication of where
    # the robot is rather than a localisation reading. Consumers must not treat
    # an approximate fix as interchangeable with an exact one; today the only
    # consumer that cares surfaces it as an attribute so a human can judge.
    approximate: bool = False
    # `time.monotonic()` when the fix was *observed*, so age is computable and
    # immune to the wall clock being stepped. None = the source does not track
    # observation time.
    observed_at: float | None = None

    @property
    def point(self) -> tuple[float, float]:
        """The fix as a plain `(x, y)` tuple for the geometry helpers."""
        return (self.x, self.y)

    @property
    def age(self) -> float | None:
        """Seconds since this fix was observed, or None if that is unknown."""
        if self.observed_at is None:
            return None
        return max(0.0, time.monotonic() - self.observed_at)


class PositionSource(ABC):
    """A thing that can (or explicitly cannot) report where the robot is.

    Two questions, kept separate on purpose:

    * `available` -- could this source *ever* produce a fix on this device? It
      is a static capability answer, used to fail capture services fast and with
      a useful message rather than silently recording nothing.
    * `async_get_position()` -- can it produce one *right now*? `None` is an
      ordinary, expected answer (robot asleep, DP not answering, map not loaded)
      and must never be treated as an error by callers.
    """

    #: Short stable identifier, surfaced on the `current_room` sensor.
    key: str = "none"

    @property
    def available(self) -> bool:
        """True when this source can, in principle, produce a position."""
        return False

    @property
    def unavailable_reason(self) -> str:
        """Human-readable explanation for `available == False`."""
        return "no position source is configured"

    @abstractmethod
    async def async_get_position(
        self, dps: Mapping[str, Any]
    ) -> RobotPosition | None:
        """Return the robot's current position, or None if not knowable now.

        `dps` is the coordinator's accumulated datapoint dict, so a source that
        can read position straight out of normal polling does not need to do any
        I/O of its own.
        """


class NullPositionSource(PositionSource):
    """The source that always answers "unknown" -- today's default.

    This is not a stub to be deleted: it is the correct implementation for every
    device whose position datapoint has not been verified, and keeping it as a
    first-class source is what forces every layer above to handle `None`
    properly instead of handling it only in tests.
    """

    key = "none"

    def __init__(self, reason: str | None = None) -> None:
        """Store why no position is available, for the error messages."""
        self._reason = reason or (
            "no position source is wired up: this robot family has no path-trail "
            "datapoint (DP 104), and the optional AI-object sighting source is off"
        )

    @property
    def unavailable_reason(self) -> str:
        """Explain why nothing can be captured or classified."""
        return self._reason

    async def async_get_position(
        self, dps: Mapping[str, Any]
    ) -> RobotPosition | None:
        """Always None -- by design."""
        return None


class StaticPositionSource(PositionSource):
    """Replays a fixed list of positions, one per call. Offline use only.

    Exists so the room classifier, capture services and staleness detection can
    be driven end-to-end with no robot and no Home Assistant (see
    an offline test harness). It is never returned by
    `create_position_source()`.
    """

    key = "static"

    def __init__(
        self,
        points: Iterable[tuple[float, float] | None],
        *,
        map_id: int | None = None,
        repeat_last: bool = True,
    ) -> None:
        """Queue up the fixes to replay."""
        self._points = list(points)
        self._index = 0
        self._map_id = map_id
        self._repeat_last = repeat_last

    @property
    def available(self) -> bool:
        """A scripted source is always "capable"."""
        return True

    async def async_get_position(
        self, dps: Mapping[str, Any]
    ) -> RobotPosition | None:
        """Return the next scripted fix (None entries mean "no fix")."""
        if self._index >= len(self._points):
            if not self._repeat_last or not self._points:
                return None
            point = self._points[-1]
        else:
            point = self._points[self._index]
            self._index += 1
        if point is None:
            return None
        return RobotPosition(
            x=point[0], y=point[1], map_id=self._map_id, source=self.key
        )


# --- DP 104: the path trail (real, decoded, read passively) -------------------
# The datapoint carries a genuine position trail and the decoder below is
# verified against live captures. Nothing here writes it: see the module
# docstring for why elicitation is still unproven.

#: Retired. It used to gate `PathDataPositionSource`, which no longer exists --
#: the trail source is built from observed values instead, so nothing gates.
#: The name is kept, still False, because it now means exactly one narrower
#: thing: **we have never made the robot answer a request of ours.** Passive
#: reception is verified; on-demand elicitation is not.
PATH_DATA_VERIFIED = False

#: The evidence, in one line, so it travels with the error messages.
PATH_DATA_EVIDENCE = (
    "DP 104 carries a real position trail (cmd:102 with data.point) and is read "
    "passively; it flows while a map session is running (in practice, the vendor "
    "app's map screen). Requests of ours have never been answered, so nothing "
    "here writes the datapoint"
)

#: How long a trail point stays usable as "where the robot is", in seconds.
#:
#: 90 s. Derived, not guessed: in a passive capture of a real job the robot
#: emitted 30 `cmd:102` batches whose inter-batch gaps were 0.1-11.9 s for 26 of
#: the 29 intervals (median 6.1 s). The four outliers -- 20.2, 36.4, 38.4, 41.9
#: and one 107.8 s -- all fall in the stretch where the robot was stuck and
#: therefore *not moving*, so a slightly stale fix there was still correct. 90 s
#: is six coordinator polls (15 s each) and comfortably clears every gap seen
#: while the robot was actually driving, while being far shorter than a job, so
#: a trail that stops flowing mid-clean stops producing fixes rather than
#: pinning the robot to the last place it was seen.
TRAIL_STALE_SECONDS = 90.0

#: Trail coordinates are used **as-is**, in the same map-cell frame as the
#: DP 105 no-go rectangles and AI-object sightings.
#:
#: This was 0.1 -- a guess taken from the app's *display* transform. Real trail
#: points settle it: an observed batch runs [[872,560] ... [594,411]], while the
#: no-go rectangles span roughly x -1076..2468 and sightings reach (536,948).
#: Scaling by 0.1 would squeeze the entire path into a ~90x60 box in the corner
#: of a map thousands of cells wide, which is not where the robot was. The 0.1
#: belongs to rendering, exactly as already documented for AI-object coordinates.
PATH_SCALE = 1.0


def encode_path_request(startno: int = 0) -> str:
    """Build the DP 104 request payload, base64-encoded.

    **Nothing in the integration calls this, and nothing should.** It is a pure
    encoder kept for out-of-tree probes: the robot has never answered a
    request of ours, and the trail source is deliberately passive so that a
    position read can never write to the robot's map channel.

    `startno` is the index into the path trail to resume from; the robot replies
    with `curnums` points beginning there and reports the running `count`.

    **Base64, not hex.** The vendor app emits hex because the Tuya SDK
    hex-decodes a raw datapoint before transmitting; tinytuya base64-decodes it.
    Both put identical bytes on the wire. Sending hex from a tinytuya client
    produces garbage that the robot silently discards -- which is precisely how
    DP 104 was mistakenly written off as empty.
    """
    body = json.dumps({"cmd": 104, "data": {"startno": startno}}, separators=(",", ":"))
    return base64.b64encode(body.encode("utf-8")).decode("ascii")


def _b64_to_text(text: str) -> str | None:
    """base64 -> JSON text, or None if it is not base64-wrapped JSON."""
    try:
        decoded = base64.b64decode(text + "=" * (-len(text) % 4), validate=True)
    except (binascii.Error, ValueError):
        return None
    try:
        out = decoded.decode("utf-8")
    except UnicodeDecodeError:
        return None
    return out if out.lstrip().startswith(("{", "[")) else None


def _hex_to_text(text: str) -> str | None:
    """hex -> JSON text, or None. Only the vendor app writes this form."""
    if not text or len(text) % 2 or not all(c in "0123456789abcdefABCDEF" for c in text):
        return None
    try:
        out = bytes.fromhex(text).decode("utf-8")
    except (ValueError, UnicodeDecodeError):
        return None
    return out if out.lstrip().startswith(("{", "[")) else None


def decode_path_points(raw: Any) -> list[tuple[float, float]]:
    """Decode a DP 104 readback into map-cell points. Returns [] if it cannot.

    The real shape is `data.point` -- singular -- inside a `cmd:102` object;
    the other keys are tolerated aliases kept because they cost nothing. A
    request (`cmd:104`) has no point list and therefore correctly yields `[]`,
    which is what stops our own echoes being counted as data.

    This is the low-level helper. `PathTrailTracker` is what the integration
    uses, because a single batch is only a fragment of the trail.
    """
    if raw is None:
        return []

    payload: Any = raw
    if isinstance(payload, (bytes, bytearray)):
        payload = payload.decode("utf-8", "ignore")
    if isinstance(payload, str):
        text = payload.strip()
        # Robot-originated values arrive base64-wrapped; try that first, then
        # hex, and fall through to treating the string as bare JSON.
        for decoder in (_b64_to_text, _hex_to_text):
            decoded = decoder(text)
            if decoded is not None:
                text = decoded
                break
        try:
            payload = json.loads(text)
        except (ValueError, TypeError):
            return []

    candidates: Sequence[Any] | None = None
    if isinstance(payload, list):
        candidates = payload
    elif isinstance(payload, dict):
        data = payload.get("data")
        for holder in (data, payload):
            if isinstance(holder, dict):
                # "point" (singular) is what the robot actually sends; the
                # others are kept as tolerated aliases.
                for key in ("point", "path", "points", "pathData", "data"):
                    value = holder.get(key)
                    if isinstance(value, list):
                        candidates = value
                        break
            elif isinstance(holder, list):
                candidates = holder
            if candidates is not None:
                break

    if not candidates:
        return []

    points: list[tuple[float, float]] = []
    for item in candidates:
        if (
            isinstance(item, (list, tuple))
            and len(item) >= 2
            and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in item[:2])
        ):
            points.append((item[0] * PATH_SCALE, item[1] * PATH_SCALE))
    return points


@dataclass(frozen=True)
class PathBatch:
    """One decoded `cmd:102` path batch."""

    pathid: int | None
    count: int | None
    startno: int
    points: list[tuple[float, float]]

    @property
    def curnums(self) -> int:
        """How many points this batch actually carried."""
        return len(self.points)


def decode_path_reply(raw: Any) -> PathBatch | None:
    """Decode a DP 104 value into a `PathBatch`, or None if it is not a reply.

    `None` is the answer for anything that is not a `cmd:102` object carrying a
    point list -- including the app's `cmd:104` `startno` requests and our own
    write echoes, both of which appear on this datapoint constantly. The caller
    is expected to count those separately rather than to treat them as errors.
    """
    if raw is None:
        return None

    payload: Any = raw
    if isinstance(payload, (bytes, bytearray)):
        payload = payload.decode("utf-8", "ignore")
    if isinstance(payload, str):
        text = payload.strip()
        for decoder in (_b64_to_text, _hex_to_text):
            decoded = decoder(text)
            if decoded is not None:
                text = decoded
                break
        try:
            payload = json.loads(text)
        except (ValueError, TypeError):
            return None

    if not isinstance(payload, dict) or payload.get("cmd") != 102:
        return None
    data = payload.get("data")
    if not isinstance(data, dict):
        return None
    points = decode_path_points(payload)
    if not points:
        return None

    startno = data.get("startno")
    if not isinstance(startno, int) or isinstance(startno, bool) or startno < 0:
        return None

    def _opt_int(value: Any) -> int | None:
        return value if isinstance(value, int) and not isinstance(value, bool) else None

    return PathBatch(
        pathid=_opt_int(data.get("pathid")),
        count=_opt_int(data.get("count")),
        startno=startno,
        points=points,
    )


class PathTrailTracker:
    """Accumulates DP 104 `cmd:102` batches into one trail. **Passive only.**

    One per config entry, owned by the coordinator, fed every value the poller
    sees. It never writes anything: the robot emits new points on its own
    schedule while a map session is running, and a request of ours has never
    been answered (see the module docstring), so asking would only put write
    traffic on the robot's map channel for nothing.

    Three properties of the real feed shape the design, all observed in a
    passive capture of a real job:

    * **Batches overlap and repeat.** `startno` 636 (5 points, count 641) was
      followed by `startno` 640 (1 point, count 641) -- the same point, twice.
      Two listeners on the LAN see overlapping streams and the app re-requests
      ranges it already has. So points are stored **by absolute index**
      (`startno + i`), which makes a repeat idempotent instead of a duplicate.
    * **`count` is the robot's total, not ours.** We only ever see the batches
      that happened to be on the wire while we were listening, so `count` (649
      in that capture) legitimately exceeds `points_held` (84). Reporting the
      robot's number verbatim is the honest thing; conflating the two would
      claim a complete trail we do not have.
    * **`pathid` changes per job.** A new id means a new trail, so the
      accumulation resets rather than splicing two jobs' coordinate streams
      together.
    """

    def __init__(self) -> None:
        """Start with an empty trail."""
        #: Absolute index -> point, so overlapping batches collapse.
        self._points: dict[int, tuple[float, float]] = {}
        #: The current job's path id, or None if none has been seen.
        self.pathid: int | None = None
        #: The robot's own running total for this path. May exceed what we hold.
        self.count: int | None = None
        #: The highest-indexed point seen, i.e. the newest known position.
        self.last_point: tuple[float, float] | None = None
        #: `time.monotonic()` when `last_point` was observed.
        self.observed_at: float | None = None
        #: How many `cmd:102` batches have been ingested.
        self.batches_seen = 0
        #: Values that decoded but were not replies -- requests and echoes.
        self.echoes_ignored = 0
        #: How many times a new `pathid` forced a reset.
        self.path_changes = 0
        self._last_index: int | None = None

    def ingest(self, value: Any, *, now: float | None = None) -> PathBatch | None:
        """Feed one raw DP 104 value in. Returns the batch, or None.

        Safe to call with anything, including None. A value that decodes to a
        `cmd:104` request -- the app's, or one of ours echoed back -- counts
        against `echoes_ignored` and changes nothing else.
        """
        batch = decode_path_reply(value)
        if batch is None:
            if value is not None:
                self.echoes_ignored += 1
            return None

        if batch.pathid is not None and batch.pathid != self.pathid:
            if self.pathid is not None:
                self.path_changes += 1
            self.reset(keep_counters=True)
            self.pathid = batch.pathid

        self.batches_seen += 1
        if batch.count is not None:
            # Monotonic in practice; take the larger so an out-of-order batch
            # cannot walk the robot's total backwards.
            self.count = (
                batch.count if self.count is None else max(self.count, batch.count)
            )

        for offset, point in enumerate(batch.points):
            self._points[batch.startno + offset] = point

        highest = batch.startno + len(batch.points) - 1
        if self._last_index is None or highest >= self._last_index:
            self._last_index = highest
            self.last_point = self._points[highest]
            self.observed_at = time.monotonic() if now is None else now
        return batch

    @property
    def points(self) -> list[tuple[float, float]]:
        """Every point held, ordered by its index in the robot's trail."""
        return [self._points[index] for index in sorted(self._points)]

    @property
    def points_held(self) -> int:
        """How many distinct trail indices we have actually seen."""
        return len(self._points)

    @property
    def age(self) -> float | None:
        """Seconds since the newest point was observed, or None if never."""
        if self.observed_at is None:
            return None
        return max(0.0, time.monotonic() - self.observed_at)

    def as_attributes(self) -> dict[str, Any]:
        """Attribute payload for the path-trail sensor."""
        age = self.age
        return {
            "pathid": self.pathid,
            "count": self.count,
            "points_held": self.points_held,
            "last_point": list(self.last_point) if self.last_point else None,
            "age": None if age is None else round(age, 1),
            "batches": self.batches_seen,
            "path_changes": self.path_changes,
            "echoes_ignored": self.echoes_ignored,
        }

    def reset(self, *, keep_counters: bool = False) -> None:
        """Forget the trail -- for a new path id, a new job, or tests.

        `keep_counters` preserves the lifetime diagnostics (`batches_seen`,
        `echoes_ignored`, `path_changes`) across an automatic path change, so
        the sensor's attributes do not appear to rewind mid-session.
        """
        self._points.clear()
        self._last_index = None
        self.pathid = None
        self.count = None
        self.last_point = None
        self.observed_at = None
        if not keep_counters:
            self.batches_seen = 0
            self.echoes_ignored = 0
            self.path_changes = 0


class PathTrailPositionSource(PositionSource):
    """Exact position, taken from the newest point of the DP 104 trail.

    This is the good source: real coordinates from the robot's own localisation,
    in map cells, with no inference in between. Its weakness is not accuracy but
    **availability** -- the trail only flows while a map session is running, and
    we cannot start one (see the module docstring). So it produces exact fixes
    for as long as something is driving the feed and honest `None`s the rest of
    the time.

    `available` is a *capability* answer and is True on any family with the
    datapoint, which is the right contract: the family can report position, and
    the capture services should not refuse up front. Whether a fix exists right
    now is `async_get_position`'s question, and it answers None once the newest
    point is older than `TRAIL_STALE_SECONDS`.

    **It never writes.** A position read that pokes the robot's map channel is
    exactly the failure this module exists to prevent.
    """

    key = "dp_path_trail"

    def __init__(self, dp: str, tracker: PathTrailTracker) -> None:
        """Bind to the path datapoint id and the shared trail tracker."""
        self._dp = dp
        self._tracker = tracker

    @property
    def available(self) -> bool:
        """True: the datapoint is real and decodes on this family."""
        return True

    @property
    def unavailable_reason(self) -> str:
        """Never used while `available` is True, but kept truthful anyway."""
        return (
            f"DP {self._dp} carries a real position trail but nothing has been "
            "received yet; it only flows while a map session is running"
        )

    @property
    def last_fix_age(self) -> float | None:
        """Seconds since the newest trail point, or None if there is none."""
        return self._tracker.age

    @property
    def fresh(self) -> bool:
        """Whether the trail is recent enough to be treated as a position."""
        age = self.last_fix_age
        return age is not None and age <= TRAIL_STALE_SECONDS

    async def async_get_position(
        self, dps: Mapping[str, Any]
    ) -> RobotPosition | None:
        """Return the newest trail point, if it is fresh enough to mean anything.

        `dps` is ignored: the tracker is fed by the coordinator as values arrive,
        which is the only place that sees DP 104 values a later value has already
        overwritten in the merged snapshot.
        """
        point = self._tracker.last_point
        if point is None or not self.fresh:
            return None
        return RobotPosition(
            x=point[0],
            y=point[1],
            source=self.key,
            approximate=False,
            observed_at=self._tracker.observed_at,
        )


class CompositePositionSource(PositionSource):
    """Asks each source in turn and returns the first fix. **Order, not age.**

    The obvious alternative -- take whichever source has the freshest fix -- is
    wrong here, because the sources are not of comparable quality. The trail is
    the robot's own localisation, exact, in map cells. An AI-object sighting is
    an *obstacle's* position standing in for the robot's, offset by the sensing
    distance and capable of landing on the wrong side of a doorway. A
    five-second-old sighting is still worse evidence than a forty-second-old
    trail point, so a freshness rule would routinely downgrade a good fix to a
    bad one. Staleness is handled where it belongs instead: each source refuses
    to answer once its own evidence is too old, and the composite simply takes
    the best source that is still willing to speak.

    `key` stays `"composite"`; the fix itself carries the real provenance in
    `RobotPosition.source`, which `rooms.py` surfaces as `position_fix_source`.
    """

    key = "composite"

    def __init__(self, sources: Sequence[PositionSource]) -> None:
        """Store the sources in priority order, best first."""
        self.sources = list(sources)

    @property
    def available(self) -> bool:
        """True when any member source could ever produce a fix."""
        return any(source.available for source in self.sources)

    @property
    def unavailable_reason(self) -> str:
        """Every member's reason, so nothing is hidden behind the composite."""
        return "; ".join(source.unavailable_reason for source in self.sources)

    @property
    def last_fix_age(self) -> float | None:
        """The freshest age any member reports, or None if none tracks age."""
        ages = [
            age
            for age in (
                getattr(source, "last_fix_age", None) for source in self.sources
            )
            if age is not None
        ]
        return min(ages) if ages else None

    async def async_get_position(
        self, dps: Mapping[str, Any]
    ) -> RobotPosition | None:
        """Return the first fix any source offers, in priority order."""
        for source in self.sources:
            position = await source.async_get_position(dps)
            if position is not None:
                return position
        return None


# --- the AI-object sighting source (opt-in) ----------------------------------

#: How long a sighting stays interesting, in seconds. Past this the source stops
#: describing the fix as fresh in its diagnostics. GUESS: 300 s, chosen because
#: it is a few coordinator polls plus the observed gap between detections in the
#: reference capture (up to ~60 s of active cleaning between new objects). It
#: does not gate anything -- no fix is invented or suppressed by it -- it only
#: labels how much trust the age deserves.
SIGHTING_FRESH_SECONDS = 300.0


class AiObjectSightingPositionSource(PositionSource):
    """Approximate position inferred from *where the robot spotted something*.

    **This is not localisation, and the naming is deliberate.** The robot's
    camera reports obstacles it detects (DP 105, cmd 0x37 `eAiObjectToAPP`), and
    it detects them directly in front of itself -- so the coordinates of a
    brand-new detection are, within a robot-length or so, where the robot was
    standing when it saw the thing. Across the reference capture the newest
    entry traced a plausible route::

        (-712,-834) -> (-638,-1058) -> (46,-1416) -> (204,-1378)

    (A fifth point, (244,-1366), appears in the raw frames but is the fourth
    object being re-reported 42 cells away, not a fifth object -- the robot's
    own final count byte is 4. This is exactly why dedup needs a tolerance.)

    That is genuinely useful and genuinely weak, in these specific ways, all of
    which a consumer must assume:

    * **Sparse.** Four distinct objects across a 15-minute clean. Long stretches
      of a job produce no fix at all, and a robot cleaning a tidy empty room
      produces none ever.
    * **Event-driven, not periodic.** Fixes appear when the house happens to
      contain clutter, not on a schedule. The rate is a property of the floor,
      not of the robot.
    * **Offset by the sensing distance.** The fix is where the *object* is, not
      where the robot is. Near a room boundary that offset can fall on the wrong
      side of a doorway.
    * **Lossy sampling.** Frames arrive far faster than the coordinator polls,
      so most are never seen. This under-reports; it does not fabricate.

    Every fix is therefore stamped `approximate=True` and carries `observed_at`
    so age is computable. The room classifier's hysteresis then does the real
    work: `DEFAULT_ENTER_SAMPLES` consecutive agreeing samples are needed to
    confirm a room, and because a fix is only produced on a *new* detection,
    confirming a room this way needs several distinct obstacles found in it.
    That is a high bar on purpose.

    Deliberately: **no fix is emitted on polls where nothing new was detected.**
    Re-emitting the last sighting would manufacture the consecutive samples the
    hysteresis is counting, converting one obstacle into a confirmed room. The
    classifier already treats "no fix" correctly -- it holds state and keeps
    `last_known_room` -- so silence is both honest and safe.
    """

    key = "ai_object_sighting"

    def __init__(self, dp: str, tracker: AiObjectTracker) -> None:
        """Bind to the transportation datapoint and the shared object tracker."""
        self._dp = dp
        self._tracker = tracker

    @property
    def available(self) -> bool:
        """True: the datapoint is verified and the robot pushes it unprompted."""
        return True

    @property
    def unavailable_reason(self) -> str:
        """Never used while `available` is True, but kept truthful anyway."""
        return (
            f"DP {self._dp} AI-object sightings are enabled but nothing has been "
            "detected yet"
        )

    @property
    def last_fix_age(self) -> float | None:
        """Seconds since the last new detection, or None if there never was one.

        Surfaced on the `current_room` sensor so a human (or a template) can see
        how old the evidence behind a room name actually is.
        """
        return self._tracker.last_sighting_age

    @property
    def fresh(self) -> bool:
        """Whether the last sighting is recent enough to be worth much."""
        age = self.last_fix_age
        return age is not None and age <= SIGHTING_FRESH_SECONDS

    async def async_get_position(
        self, dps: Mapping[str, Any]
    ) -> RobotPosition | None:
        """Return a fix only if a genuinely new object appeared since last poll.

        `dps` is ignored: the tracker is fed by the coordinator as values arrive,
        which is the only place that sees DP 105 values a later value has already
        overwritten in the merged snapshot.
        """
        sighting = self._tracker.consume_new_sighting()
        if sighting is None:
            return None
        return RobotPosition(
            x=float(sighting.obj.x),
            y=float(sighting.obj.y),
            source=self.key,
            approximate=True,
            observed_at=sighting.at,
        )


def create_position_source(
    spec: FamilySpec,
    set_dp: Any = None,
    *,
    ai_object_tracker: AiObjectTracker | None = None,
    ai_object_position: bool = False,
    path_trail_tracker: PathTrailTracker | None = None,
) -> PositionSource:
    """Return the position source for a robot. **The single swap point.**

    Composition, best source first:

    * **`PathTrailPositionSource`** whenever the family has DP 104 *and* the
      caller supplied a `path_trail_tracker`. Exact coordinates, read passively.
      The tracker is a required argument rather than something built here
      because the coordinator has to feed it every DP 104 value as it arrives --
      a source that only sees the merged snapshot would miss most of the trail.
    * **`AiObjectSightingPositionSource`** when the user opted in and a tracker
      was supplied. Approximate, sparse, event-driven; a *user* decision, which
      is why it is a config-entry option defaulting to off. A coarse source
      feeding room classification can produce confidently wrong room names, and
      `rooms.py` exists specifically to avoid that failure mode.

    Both present means a `CompositePositionSource` in that order -- see its
    docstring for why the rule is priority and not freshness. Neither present
    means `NullPositionSource`, which is still the shipping default and still
    correct for Vision (its DP 216 `COMMAND_POSITION` payload shape is
    undocumented and no value from one has ever been observed) and for Random
    (no position concept at all; it does not map).

    `set_dp` is accepted for backwards compatibility and deliberately unused:
    nothing on the position path writes to the robot any more.
    """
    del set_dp  # nothing here writes; kept so existing callers keep working.

    sources: list[PositionSource] = []

    if spec.dp_path_data is not None and path_trail_tracker is not None:
        sources.append(
            PathTrailPositionSource(spec.dp_path_data, path_trail_tracker)
        )

    if ai_object_position and spec.dp_transportation is not None:
        if ai_object_tracker is not None:
            sources.append(
                AiObjectSightingPositionSource(
                    spec.dp_transportation, ai_object_tracker
                )
            )
        else:
            _LOGGER.warning(
                "bObsweep: AI-object position was requested but no object tracker "
                "was supplied; that source will not be used"
            )

    if len(sources) == 1:
        return sources[0]
    if sources:
        return CompositePositionSource(sources)

    if spec.dp_path_data is not None:
        return NullPositionSource(
            f"no position source is wired up. {PATH_DATA_EVIDENCE}. Supply a "
            "PathTrailTracker to read it, or switch on the approximate "
            "AI-object sighting source in this integration's options"
        )
    if spec.dp_transportation is not None:
        return NullPositionSource(
            "no position source is enabled: this family has no path-trail "
            "datapoint. The approximate AI-object sighting source can be "
            "switched on in this integration's options, but it is off by "
            "default because it is sparse and only roughly indicates where the "
            "robot is"
        )
    return NullPositionSource(
        f"the {spec.key} bObsweep family has no known position datapoint"
    )
