"""Robot position acquisition -- the one place that knows where the robot is.

**Read this before touching anything else in the room-awareness stack.**

**DP 104 carries a real position trail. The earlier "settled negative" was our
own encoding bug.** (Corrected 2026-09-02, superseding the 2026-09-01 finding.)

Observed live on the LAN, decoded, with real coordinates:

```
{"cmd":102,"data":{"pathid":814,"type":2,"count":488,"curnums":6,"startno":482,
                   "point":[[810,539],[796,535],[750,493],[718,461],[660,437],[594,411]]}}
```

So the reply shape is no longer a guess: the robot answers a
`{"cmd":104,"data":{"startno":N}}` request with a `cmd:102` object whose points
live under `data.point` -- **singular**, which is not a key the earlier decoder
looked for. `count` is the total trail length so far, `startno` the index this
batch begins at, and `curnums` its length, so the feed is incremental and
resumable.

Why it looked dead: the request was **hex**-encoded. tinytuya base64-decodes a
raw datapoint, so a hex string went out as garbage bytes and the robot never
saw a valid request. The "26 reads, 26 echoes" were our own mangled writes
bouncing back. The same bug independently broke every DP 105 getter on
2026-09-02 until it was found; the vendor app publishes hex because the Tuya
*SDK* hex-decodes, which is a different client convention, not the wire format.

**What is still unresolved: we cannot yet *elicit* the trail.** Correctly
base64-encoded requests (`startno` 0, 1, 478, 900, 100000) sent during a real,
active clean drew no reply -- verified in a passive capture that confirms our
requests reached the wire intact. The vendor app calls the native
`startMapLister()` and `initStartNo()` before its request loop, and that map
session is the part not yet replicated.

What does work today is **passive harvesting**: when the app's map screen is
driving the loop, the robot's `cmd:102` replies are broadcast as ordinary DP 104
updates and anything listening on the LAN sees the whole trail.

`PathDataPositionSource` below therefore stays unreachable, but the reason has
changed: not "this datapoint is empty" -- it demonstrably is not -- but "we
cannot make it answer on demand, so a source built on it would silently produce
nothing while claiming to be available".

So position lives behind this interface and nowhere else. Zones, the room
classifier, the capture services, staleness detection and the segment API are
all built against `PositionSource`, and all of them are correct with the source
being `NullPositionSource` -- i.e. with the answer permanently `None`.

The one source that *does* produce coordinates is
`AiObjectSightingPositionSource`: the robot's obstacle detector reports what it
sees roughly where it is standing. It is **opt-in and off by default**, and its
docstring is emphatic about what it is not. See `create_position_source()`.

Nothing outside this module should mention DP 104, `startno`, the 0.1 scale, or
any other property of the eventual source.

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
            "this robot has no working position datapoint -- DP 104 "
            "(COMMAND_PATH_DATA) is confirmed dead on this firmware (see "
            "position.py), and the optional AI-object sighting source is off"
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


# --- DP 104: REAL, BUT NOT YET ON DEMAND -------------------------------------
# The datapoint carries a genuine position trail -- that is confirmed against
# live hardware, with decoded coordinates. What is missing is a way to make the
# robot answer *our* request; today the trail only flows while the vendor app's
# map session drives it. Everything here is therefore correct-but-unreachable
# code, kept ready for the day the map session is replicated. See the module
# docstring.

#: **Still do not flip this -- but not for the reason it used to say.** The
#: datapoint is real and its replies decode correctly. What is unverified is
#: whether we can *elicit* one: correctly encoded requests during an active
#: clean drew no answer, because the vendor app establishes a native map
#: session (`startMapLister`) first. Enabling this would give the room
#: classifier a source that produces nothing while claiming to be available.
#: Flip it only once a request of ours has actually been answered.
PATH_DATA_VERIFIED = False

#: The evidence, in one line, so it travels with the error messages.
PATH_DATA_EVIDENCE = (
    "DP 104 does carry a position trail (cmd:102 with data.point, confirmed "
    "2026-09-02), but it only flows while the vendor app's map session drives "
    "it; our own correctly-encoded startno requests go unanswered"
)

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

    UNVERIFIED, and written to be maximally forgiving because the shape of a
    *populated* response is genuinely unknown -- the only response ever seen
    was an echo of the request with an empty point list. Handles: a hex string
    wrapping JSON, plain JSON, a bare list, and the point list nested under
    `data.path`/`data.points`/`path`/`points`.

    GUESS: the 0.1 scale factor. The app divides trail coordinates by 10 for
    display, which implies the wire values are decimetre-ish sub-cells; applying
    it here is what makes the result comparable with DP 105 map-cell geometry.
    If the probe shows otherwise, this constant is the only thing to change.
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


class PathDataPositionSource(PositionSource):
    """DP 104 path-trail position source. **DEAD -- this does not work.**

    Kept as documentation of a refuted hypothesis, not as code anyone should
    enable. It writes a `startno` request to the path datapoint and reads the
    trail back out of the next coordinator poll. On this firmware the robot
    answers that request with the request, unchanged, and never with a trail --
    while docked *and* through a full active cleaning job. See
    `PATH_DATA_EVIDENCE` and the module docstring.

    `create_position_source()` cannot return this class: `PATH_DATA_VERIFIED` is
    False and nothing sets it.
    """

    key = "dp_path_data"

    def __init__(self, dp: str, set_dp: Any = None) -> None:
        """Bind to the path datapoint id and an optional DP writer."""
        self._dp = dp
        self._set_dp = set_dp
        self._warned = False

    @property
    def available(self) -> bool:
        """Always False. DP 104 is a settled negative, not a pending probe."""
        return PATH_DATA_VERIFIED

    @property
    def unavailable_reason(self) -> str:
        """Explain the dead datapoint precisely, with the evidence attached."""
        return (
            f"DP {self._dp} (COMMAND_PATH_DATA) does not report position on this "
            f"firmware -- {PATH_DATA_EVIDENCE}"
        )

    async def async_get_position(
        self, dps: Mapping[str, Any]
    ) -> RobotPosition | None:
        """Ask for the path trail and return its last point, if there is one."""
        if self._set_dp is not None:
            try:
                await self._set_dp(self._dp, encode_path_request(0))
            except Exception:  # noqa: BLE001 - a poll helper must not raise
                if not self._warned:
                    _LOGGER.debug(
                        "bObsweep: DP %s path request failed", self._dp, exc_info=True
                    )
                    self._warned = True
                return None

        points = decode_path_points(dps.get(self._dp))
        if not points:
            return None
        x, y = points[-1]
        return RobotPosition(x=x, y=y, source=self.key)


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
) -> PositionSource:
    """Return the position source for a robot. **The single swap point.**

    The default is `NullPositionSource`, because no datapoint on any family
    reports real position:

    * SLAM's DP 104 is refuted -- see `PATH_DATA_EVIDENCE`.
    * Vision has a `COMMAND_POSITION` datapoint (DP 216) whose payload shape is
      undocumented in the app bundle -- no value from one has ever been observed.
    * Random has no position concept at all; it does not map.

    Passing `ai_object_position=True` (SLAM, and only when a tracker is supplied)
    swaps in `AiObjectSightingPositionSource`. That is a *user* decision, not
    ours, which is why it is a config-entry option defaulting to off: a coarse
    source feeding room classification can produce confidently wrong room names,
    and `rooms.py` exists specifically to avoid that failure mode.
    """
    if ai_object_position and spec.dp_transportation is not None:
        if ai_object_tracker is not None:
            return AiObjectSightingPositionSource(
                spec.dp_transportation, ai_object_tracker
            )
        _LOGGER.warning(
            "bObsweep: AI-object position was requested but no object tracker "
            "was supplied; falling back to no position source"
        )

    if PATH_DATA_VERIFIED and spec.dp_path_data is not None:
        return PathDataPositionSource(spec.dp_path_data, set_dp)

    if spec.dp_transportation is not None:
        return NullPositionSource(
            "no position source is enabled: DP 104 is dead on this firmware "
            f"({PATH_DATA_EVIDENCE}). The approximate AI-object sighting source "
            "can be switched on in this integration's options, but it is off by "
            "default because it is sparse and only roughly indicates where the "
            "robot is"
        )
    if spec.dp_path_data is not None:
        return NullPositionSource(
            PathDataPositionSource(spec.dp_path_data).unavailable_reason
        )
    return NullPositionSource(
        f"the {spec.key} bObsweep family has no known position datapoint"
    )
