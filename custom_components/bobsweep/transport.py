"""DP 105 (`COMMAND_TRANSPORTATION`) frame codec and AI-object decoding.

DP 105 is the robot's *transparent command channel*: map geometry, no-go zones,
room tables, clean order and the on-board object detector all ride on it as
small framed binary messages. The datapoint itself carries no structure — every
value is one frame, and the frame's `cmd` byte says what it is.

This module is deliberately free of Home Assistant *and* of any I/O, so the
whole codec can be exercised offline against a raw capture.

Framing
-------

Two headers occur in practice, and **the same command has been observed under
both**, so a decoder must branch on the header byte rather than assume one::

    0xAA:  [AA] [len]        [cmd] [data…] [chk]
    0xBB:  [BB] [len_hi=00]  [len] [cmd] [data…] [chk]

* `len` is `1 + len(data)` — it counts the command byte, not the checksum.
* `chk` is `(cmd + sum(data)) & 0xFF`. The header and length bytes are **not**
  part of the sum; that was confirmed by checking every framed value in the
  2026-09-01 live capture (33 frames, 33 checksums correct).
* Total frame length is therefore `3 + len` for 0xAA and `4 + len` for the
  two-header forms.

Three headers, three command tables
-----------------------------------

The header byte is not decoration: it **selects which command table the command
byte is read against**. The vendor app defines three frame formats and one
sender per table, and the byte at index 1 of the wider forms is a *version*
byte (always written 0), not a length high byte:

```
cBasicTransCmdFormat         = {cHeader: 170 (0xAA), cLengthIdx: 1, cCmdIdx: 2, cDataIdx: 3}
cExtendTransCmdFormat        = {cHeader: 171 (0xAB), cVersionIdx: 1, cLengthIdx: 2, cCmdIdx: 3, cDataIdx: 4}
cBobCustomizedTransCMDFormat = {cHeader: 187 (0xBB), cVersionIdx: 1, cLengthIdx: 2, cCmdIdx: 3, cDataIdx: 4}
```

So 0xAA commands mean what the basic table says, 0xBB commands what the
bObsweep-customized table says, and 0xAB what the extend table says. The tables
overlap heavily in numbering and disagree on meaning, so a command byte alone is
ambiguous. Length is a single byte in every form: the maximum payload is 254
bytes, and there is no 16-bit length variant.

0xAB has never been observed from this unit; it is decoded here so that it
cannot be silently dropped if it ever appears.

Encoding on the wire
--------------------

Robot-originated values are **base64**. Writes made by this integration echo
back as the *hex* string that was sent, so `decode_wire_value` reports which
encoding actually produced a valid frame and callers can discard hex-decoded
values as local write traffic rather than mistaking an echo for a robot
observation. (This is not theoretical: the whole DP 104 dead end was originally
misread as a success because an echo was counted as data — see `position.py`.)

Selected rooms (cmd 0x22) are an **ack, not a state**
-----------------------------------------------------

`eCleanSelectRoomsToApp` is emitted only when the app *commands* a room clean
(`eCleanSelectRooms`, 0x12), echoing the same payload back. It is **not** part
of the `eAll` report set — verified on hardware 2026-09-02, where two `eAll`
dumps during a live room clean returned the no-go zones, no-mop zones, rotate
angle and three empty area reports, and no 0x22 either time.

The consequence for this integration: a room selection can only be observed by
*listening* when the command goes past, and then remembered. Polling will never
surface it. The payload is room **ids**, not geometry (`01 01 01` for a
one-room job), and its exact layout is not yet pinned down.

Note also that a room-targeted clean reports work mode `part` / status
`part_clean` on this firmware — `selectroom` is in the app's enum table but is
never emitted, the same way DP 119 is tabled but absent.

Nothing here sends anything. `encode_frame` / `encode_frame_b64` exist as
tested pure functions for a future writer; wiring a write path to DP 105 is a
deliberate, separate decision, because *every* dangerous map operation on this
robot is a write to this one datapoint and a datapoint-level denylist cannot
protect it.
"""

from __future__ import annotations

import base64
import binascii
import logging
import math
import time
from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Sequence

_LOGGER = logging.getLogger(__name__)

#: The three framing headers the vendor app defines, one per command table.
HEADER_AA = 0xAA    # BASIC_TRANS_CMD
HEADER_AB = 0xAB    # EXTEND_TRANS_CMD — defined by the app, never yet observed
HEADER_BB = 0xBB    # BOB_CUSTOMIZED_TRANS_CMD
HEADERS = (HEADER_AA, HEADER_AB, HEADER_BB)

#: Headers whose frames carry a version byte at index 1 and the length at index 2.
_VERSIONED_HEADERS = (HEADER_AB, HEADER_BB)

#: Command bytes this module understands. The full tables were transcribed from
#: the vendor app's bundle and are not published in this repo; only the ones
#: actually decoded are named here, so this is not mistaken for the
#: authoritative list.
#:
#: **The header byte selects which table a command byte is read against**:
#: 0xAA indexes the app's `BASIC_TRANS_CMD`, 0xBB indexes its
#: `BOB_CUSTOMIZED_TRANS_CMD`. The two tables overlap heavily in numbering and
#: disagree on meaning, so a command byte alone is ambiguous. Every distinct
#: (header, command) pair observed on real hardware resolves in exactly the
#: table its header names, and several 0xBB commands have no entry in the basic
#: table at all. Reading a 0xBB command against the basic table yields a
#: plausible, wrong answer — that mistake is how 0x12 (no-mop zones) was briefly
#: taken for the room partition.

# --- 0xAA frames: BASIC_TRANS_CMD ---
CMD_CLEAN_SELECT_ROOMS = 0x12         # eCleanSelectRooms (18) — app commands a room clean
CMD_CLEAN_SELECT_ROOMS_TO_APP = 0x22  # eCleanSelectRoomsToApp (34) — the robot's ack
CMD_RESTRICTED_TO_APP = 0x24    # eRestrictedToApp (36) — the no-go rectangles
CMD_AI_OBJECT_TO_BOT = 0x36     # eAiObjectToBot (54)
CMD_AI_OBJECT_TO_APP = 0x37     # eAiObjectToAPP (55) — detected obstacles

# --- 0xBB frames: BOB_CUSTOMIZED_TRANS_CMD ---
CMD_NO_MOP_ZONE_TO_APP = 0x12   # eNoMopZoneToApp (18) — no-mop rectangles,
                                # same payload shape as CMD_RESTRICTED_TO_APP
CMD_SUPPORT_FEATURES_TO_APP = 0x02  # eSupportFeaturesToAPP (2) — feature bitmap
CMD_NAMED_SCHEDULE_TO_APP = 0x14    # eNamedScheduleToApp (20) — named schedules
CMD_SAVED_MAP_NAMES_TO_APP = 0x18   # eGetSavedMapNameToApp (24) — saved map names
CMD_MAP_ROTATE_ANGLE_TO_APP = 0x31  # eMapRotateAngleToApp (49) — uint16 degrees
CMD_MOP_CLOTH_DIRTY_TO_APP = 0x39   # eMopClothAssemDirtyToApp (57) — [ver, pct]

#: `TuyaAiObjects.java` class table, transcribed key for key. Index 255 is the
#: vendor's own "unknown", which is a real reported value, not a fallback
#: invented here.
AI_OBJECT_CLASSES: Mapping[int, str] = {
    0: "wire",
    1: "shoes",
    2: "socks",
    3: "toys",
    4: "chair",
    5: "table",
    6: "trash_can",
    7: "potted_plant",
    8: "bowl",
    9: "key",
    10: "other",
    255: "unknown",
}

#: How far apart (in map cells) two reported objects must be before they are
#: treated as two objects rather than one refined estimate.
#:
#: The robot *re-reports* an object as it gets a better look at it, so identical
#: position matching does not dedupe. Measured over the 2026-09-01 capture: the
#: largest total drift of a single tracked object was ~58 cells
#: ((-638,-1058) -> (-594,-1096)), while the smallest gap between two genuinely
#: distinct objects was ~160 cells. 80 sits between those with margin on both
#: sides. It is tuned to *one* capture and is the first thing to revisit if
#: obstacles start double-counting (raise it) or merging (lower it).
DEFAULT_DEDUP_TOLERANCE = 80.0


# --- framing ----------------------------------------------------------------


@dataclass(frozen=True)
class TransportFrame:
    """One decoded DP 105 frame."""

    header: int
    cmd: int
    data: bytes

    @property
    def checksum(self) -> int:
        """The checksum this frame's contents imply."""
        return frame_checksum(self.cmd, self.data)

    def encode(self) -> bytes:
        """Re-encode this frame back to its exact wire bytes."""
        return encode_frame(self.cmd, self.data, header=self.header)


def frame_checksum(cmd: int, data: bytes | Sequence[int] = b"") -> int:
    """Return `(cmd + sum(data)) & 0xFF`."""
    return (cmd + sum(data)) & 0xFF


def encode_frame(
    cmd: int, data: bytes | Sequence[int] = b"", *, header: int = HEADER_AA
) -> bytes:
    """Build the raw bytes of one DP 105 frame.

    `header` selects the framing; 0xAA is the compact form and the default.
    Raises ValueError for anything that cannot be represented rather than
    emitting a frame the robot would reject.
    """
    if header not in HEADERS:
        raise ValueError(f"unknown DP 105 header 0x{header:02X}")
    if not 0 <= cmd <= 0xFF:
        raise ValueError(f"command byte out of range: {cmd}")
    payload = bytes(data)
    length = 1 + len(payload)
    checksum = frame_checksum(cmd, payload)

    if length > 0xFF:
        raise ValueError(
            f"payload of {len(payload)} bytes does not fit a DP 105 frame; the "
            "length field is one byte in every framing the app defines"
        )
    if header == HEADER_AA:
        return bytes([HEADER_AA, length, cmd]) + payload + bytes([checksum])
    # 0xAB / 0xBB: header, version (always 0), length, cmd, data, checksum
    return bytes([header, 0x00, length, cmd]) + payload + bytes([checksum])


def encode_frame_b64(
    cmd: int, data: bytes | Sequence[int] = b"", *, header: int = HEADER_AA
) -> str:
    """Encode a frame the way a tinytuya client must hand it over: base64 text.

    An earlier probe wrote DP 105 as *hex* and was malformed at the encoding
    layer before the framing layer ever saw it. That was confirmed against real
    hardware on 2026-09-02: a hex write produced only an echo, because the
    string is base64-decoded on the way out and hex text decodes to garbage.

    **Do not "fix" this to hex.** The vendor app does publish hex, and its own
    source says so plainly — but the Tuya SDK hex-decodes a raw datapoint
    before transmitting, while tinytuya base64-decodes it. Both clients put the
    same bytes on the wire; only what you hand the library differs. Anything
    here that writes this datapoint must go through this function.
    """
    return base64.b64encode(encode_frame(cmd, data, header=header)).decode("ascii")


def decode_frame(raw: bytes | bytearray | None) -> TransportFrame | None:
    """Decode raw frame bytes, or return None if they are not a valid frame.

    Validates the header, the declared length against the actual buffer, and the
    checksum. `None` — not an exception — is the answer for anything that does
    not check out: this decodes attacker-adjacent bytes off a LAN socket on
    every poll and a raise here would take the whole coordinator down.
    """
    if not raw:
        return None
    buf = bytes(raw)

    if buf[0] == HEADER_AA:
        if len(buf) < 4:
            return None
        length = buf[1]
        header_size = 2
    elif buf[0] in _VERSIONED_HEADERS:
        if len(buf) < 5:
            return None
        # buf[1] is a version byte, always 0 in practice; the length is buf[2].
        length = buf[2]
        header_size = 3
    else:
        return None

    if length < 1:
        return None
    # header + cmd + data + checksum
    if len(buf) != header_size + length + 1:
        return None

    cmd = buf[header_size]
    data = buf[header_size + 1 : header_size + length]
    if buf[-1] != frame_checksum(cmd, data):
        return None
    return TransportFrame(header=buf[0], cmd=cmd, data=data)


@dataclass(frozen=True)
class DecodedValue:
    """A DP 105 wire value that decoded into a frame, plus how it decoded."""

    frame: TransportFrame
    #: "base64" for robot traffic, "hex" for this integration's own writes echoed back.
    encoding: str
    raw: bytes

    @property
    def from_robot(self) -> bool:
        """True when this value is robot-originated rather than an echoed local write."""
        return self.encoding == "base64"


def _try_base64(value: str) -> bytes | None:
    try:
        return base64.b64decode(value, validate=True)
    except (binascii.Error, ValueError):
        return None


def _try_hex(value: str) -> bytes | None:
    try:
        return bytes.fromhex(value)
    except ValueError:
        return None


def decode_wire_value(value: Any) -> DecodedValue | None:
    """Decode one DP 105 datapoint value into a frame, or None.

    Tries base64 first and hex second, and accepts whichever produces a
    structurally valid, checksum-correct frame. Order matters: a short base64
    string can consist entirely of hex-alphabet characters, and preferring
    base64 is what keeps a real robot frame from being mislabelled as an echo.
    The *reverse* mislabelling cannot happen for real traffic here, because a
    hex echo of a frame decodes as base64 to bytes that do not begin 0xAA/0xBB.
    """
    if isinstance(value, (bytes, bytearray)):
        frame = decode_frame(value)
        return (
            DecodedValue(frame=frame, encoding="raw", raw=bytes(value))
            if frame is not None
            else None
        )
    if not isinstance(value, str) or not value:
        return None

    for encoding, decoder in (("base64", _try_base64), ("hex", _try_hex)):
        raw = decoder(value)
        if raw is None:
            continue
        frame = decode_frame(raw)
        if frame is not None:
            return DecodedValue(frame=frame, encoding=encoding, raw=raw)
    return None


# --- eAiObjectToAPP (cmd 0x37) ----------------------------------------------


@dataclass(frozen=True)
class AiObject:
    """One obstacle the robot's camera reported, in raw map cells.

    `x`/`y` are in the **same frame as the DP 105 no-go rectangles** — signed
    int16 map cells, origin-relative, y-axis inverted. The vendor app renders
    them at `x * 0.1 + mapOx` / `mapOy - y * 0.1`; that 0.1 belongs to the
    *display* transform, not to the coordinate, so nothing here scales.
    """

    x: int
    y: int
    class_id: int

    @property
    def class_name(self) -> str:
        """The vendor's name for this class, or `class_<n>` for an unknown id.

        Note the deliberate distinction: `unknown` is the vendor's own class 255
        ("the robot saw something and could not classify it"), whereas
        `class_<n>` means there is no name on record for an id the robot sent —
        firmware newer than the transcribed table.
        """
        name = AI_OBJECT_CLASSES.get(self.class_id)
        if name is not None:
            return name
        return f"class_{self.class_id}"

    @property
    def point(self) -> tuple[float, float]:
        """The object's position as a plain `(x, y)` tuple."""
        return (float(self.x), float(self.y))

    def as_dict(self) -> dict[str, Any]:
        """Attribute-friendly form for the obstacle sensor."""
        return {"x": self.x, "y": self.y, "class": self.class_name}


def _int16(hi: int, lo: int) -> int:
    """Big-endian signed 16-bit from two bytes."""
    value = (hi << 8) | lo
    return value - 0x10000 if value & 0x8000 else value


def decode_ai_objects(data: bytes) -> list[AiObject] | None:
    """Decode a cmd-0x37 payload into obstacles, or None if it is malformed.

    Payload layout: one count byte, then 5 bytes per object — signed int16 `x`
    (big-endian), signed int16 `y`, one class byte.

    The `len(data) == count * 5 + 1` test is the **vendor's own** validation,
    reproduced deliberately: a frame that fails it is rejected outright rather
    than partially decoded. Every one of the 19 real frames in the 2026-09-01
    capture passes it.

    An empty list (count 0) and `None` (malformed) are different answers and
    callers must not conflate them: count 0 is the robot reporting that it
    currently sees nothing.
    """
    if not data:
        return None
    count = data[0]
    if len(data) != count * 5 + 1:
        return None
    objects: list[AiObject] = []
    for index in range(count):
        offset = 1 + index * 5
        objects.append(
            AiObject(
                x=_int16(data[offset], data[offset + 1]),
                y=_int16(data[offset + 2], data[offset + 3]),
                class_id=data[offset + 4],
            )
        )
    return objects


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def is_new_object(
    known: Iterable[AiObject],
    candidate: AiObject,
    tolerance: float = DEFAULT_DEDUP_TOLERANCE,
) -> bool:
    """True when `candidate` is not a re-report of something already in `known`.

    Distance-only, class-blind on purpose: the robot has been observed refining
    an object's *position* between frames, and there is no evidence either way
    about whether it also revises the class. Matching on class as well would
    turn a reclassification into a phantom new obstacle, which is the more
    damaging error for everything downstream.
    """
    point = candidate.point
    return all(_distance(point, other.point) > tolerance for other in known)


def dedupe_objects(
    objects: Sequence[AiObject], tolerance: float = DEFAULT_DEDUP_TOLERANCE
) -> list[AiObject]:
    """Collapse near-coincident entries, keeping the first (newest) of each.

    The robot's list is ordered newest-first, so "keep the first" keeps the most
    recently refined estimate of each object.
    """
    kept: list[AiObject] = []
    for obj in objects:
        if is_new_object(kept, obj, tolerance):
            kept.append(obj)
    return kept


@dataclass(frozen=True)
class AiSighting:
    """A newly-detected object plus when it was noticed.

    `at` is a monotonic timestamp, so ages computed from it are immune to the
    system clock being stepped.
    """

    obj: AiObject
    at: float


class AiObjectTracker:
    """Accumulates cmd-0x37 obstacle reports across polls.

    One per config entry, owned by the coordinator, because two consumers need
    the same state: the obstacle sensor (how many, and which) and the optional
    AI-object position source (did a *new* one just appear, and where).

    Why it must be stateful: the coordinator merges datapoints, so `dps["105"]`
    holds only the most recent DP 105 value at poll time — which is frequently a
    map or geometry frame, not an obstacle frame. Without somewhere to keep the
    last obstacle list, the sensor would blink to unknown every time any other
    kind of frame arrived.

    **Sampling is lossy and that is inherent.** Frames arrive on the robot's
    schedule (roughly every 0.5 s during a job in the reference capture) while
    the coordinator polls every 15 s, so most frames are never seen. Nothing
    here pretends otherwise; the obstacle list is the robot's own accumulation,
    which is why sampling it sparsely still gives a truthful count.
    """

    def __init__(self, tolerance: float = DEFAULT_DEDUP_TOLERANCE) -> None:
        """Start with nothing seen."""
        self.tolerance = tolerance
        #: The most recent decoded obstacle list, newest-first. None = never
        #: seen a valid 0x37 frame on this entry.
        self.objects: list[AiObject] | None = None
        #: Every distinct object seen this session, newest first.
        self.known: list[AiObject] = []
        #: Frames seen / frames rejected by the vendor length check.
        self.frames_seen = 0
        self.frames_rejected = 0
        self.echoes_ignored = 0
        #: The last genuinely-new object, and whether it has been consumed.
        self.last_sighting: AiSighting | None = None
        self._pending: AiSighting | None = None

    def ingest(self, value: Any, *, now: float | None = None) -> AiSighting | None:
        """Feed one raw DP 105 value in. Returns a new sighting, or None.

        Safe to call with anything, including None and frames of other commands;
        only a base64-decoded, checksum-valid cmd-0x37 frame does anything.
        """
        decoded = decode_wire_value(value)
        if decoded is None:
            return None
        if not decoded.from_robot:
            # A local hex-encoded write echoed back. Explicitly not data.
            self.echoes_ignored += 1
            return None
        if decoded.frame.cmd != CMD_AI_OBJECT_TO_APP:
            return None

        objects = decode_ai_objects(decoded.frame.data)
        if objects is None:
            self.frames_rejected += 1
            _LOGGER.debug(
                "bObsweep: rejected malformed AI-object frame: %s",
                decoded.frame.data.hex(),
            )
            return None

        self.frames_seen += 1
        self.objects = objects
        if not objects:
            return None

        # Newest first: only the head can be a brand-new detection. Trailing
        # entries are the robot re-reporting objects it has already reported.
        head = objects[0]
        if not is_new_object(self.known, head, self.tolerance):
            return None

        sighting = AiSighting(obj=head, at=time.monotonic() if now is None else now)
        self.known.insert(0, head)
        self.last_sighting = sighting
        self._pending = sighting
        return sighting

    def consume_new_sighting(self) -> AiSighting | None:
        """Take the pending new sighting, if any. Returns it at most once."""
        sighting, self._pending = self._pending, None
        return sighting

    @property
    def count(self) -> int | None:
        """How many obstacles the robot currently lists, or None if unknown."""
        return None if self.objects is None else len(self.objects)

    @property
    def last_sighting_age(self) -> float | None:
        """Seconds since the last genuinely-new detection, or None."""
        if self.last_sighting is None:
            return None
        return max(0.0, time.monotonic() - self.last_sighting.at)

    def as_attributes(self) -> dict[str, Any]:
        """Attribute payload for the obstacle sensor."""
        objects = self.objects or []
        return {
            "objects": [obj.as_dict() for obj in objects],
            "classes": sorted({obj.class_name for obj in objects}),
            "distinct_seen": len(self.known),
            "frames_decoded": self.frames_seen,
            "frames_rejected": self.frames_rejected,
        }

    def reset(self) -> None:
        """Forget everything — for a new job, or for tests."""
        self.objects = None
        self.known.clear()
        self.frames_seen = 0
        self.frames_rejected = 0
        self.echoes_ignored = 0
        self.last_sighting = None
        self._pending = None


# --- eCleanSelectRooms / …ToApp (cmd 0x12 / 0x22) ----------------------------


def decode_room_selection(data: bytes) -> tuple[str, list[int] | None, list[int] | None]:
    """Decode a room-selection payload into `(layout, room_ids, passes)`.

    **The layout is not pinned down.** The only real sample is `01 01 01` — one
    room — and one byte of it is the count, which leaves the remaining two bytes
    genuinely ambiguous. Two readings fit every frame seen so far:

    * `[count, id, id, …]` — a plain list of room ids;
    * `[count, (id, passes), …]` — each room with its pass count, which is what
      the vendor app's room-clean UI collects.

    So this measures the payload instead of assuming: `1 + count` bytes means
    ids, `1 + 2*count` means id/pass pairs, and anything else is reported as
    `"unknown"` with the raw bytes preserved rather than force-fitted. With
    `count == 1` both formulas cannot both match — `1 + 1 = 2 != 3 = 1 + 2*1` —
    and the real frame is three bytes, so `01 01 01` reads as one room with one
    pass. That is a *decode*, not a confirmation; a two-room job will settle it
    in one frame, which is why the layout is surfaced as an attribute.

    Returns `("empty", [], [])` for a well-formed zero-room selection, which is
    a different answer from "we could not read it".
    """
    if not data:
        return ("unknown", None, None)
    count = data[0]
    body = data[1:]
    if count == 0 and not body:
        return ("empty", [], None)
    if len(body) == count:
        return ("ids", list(body), None)
    if len(body) == 2 * count:
        return ("id_passes", list(body[0::2]), list(body[1::2]))
    return ("unknown", None, None)


@dataclass(frozen=True)
class RoomSelection:
    """One observed room-selection frame."""

    #: 0x22 for the robot's ack, 0x12 for the app's command.
    cmd: int
    layout: str
    room_ids: list[int] | None
    passes: list[int] | None
    raw_hex: str
    #: `time.monotonic()` when it was seen.
    at: float

    def as_dict(self) -> dict[str, Any]:
        """Attribute-friendly form."""
        return {
            "cmd": f"0x{self.cmd:02X}",
            "layout": self.layout,
            "room_ids": self.room_ids,
            "passes": self.passes,
            "raw": self.raw_hex,
        }


class RoomSelectionTracker:
    """Remembers which rooms were last selected for a room-targeted clean.

    **Listening is the only way to know this.** `eCleanSelectRoomsToApp` (0x22)
    is an *ack*: the robot emits it when the app commands a room clean
    (`eCleanSelectRooms`, 0x12) and never otherwise — verified on hardware
    2026-09-02, where two `eAll` dumps during a live room clean returned no 0x22
    either time. Polling will never surface it, so this tracker exists to catch
    the frame as it goes past and hold onto it.

    The 0x12 command is recorded separately as `last_command`. It is the app
    talking, not the robot answering, so it is evidence of intent rather than of
    state; the two are kept apart on purpose, and only the ack updates the
    selection this tracker reports.

    Nothing here writes. See the module docstring on why a DP 105 write path is
    a separate, deliberate decision.
    """

    def __init__(self) -> None:
        """Start with nothing observed."""
        #: The rooms in the last ack, or None if the layout could not be read.
        self.room_ids: list[int] | None = None
        #: Per-room pass counts, when the payload turned out to carry them.
        self.passes: list[int] | None = None
        #: The last ack's payload bytes, always kept even when undecodable.
        self.raw_hex: str | None = None
        #: "ids" | "id_passes" | "empty" | "unknown" — how the payload read.
        self.layout: str = "unknown"
        #: `time.monotonic()` of the last ack.
        self.observed_at: float | None = None
        #: The last 0x12 the app sent, for comparison against the ack.
        self.last_command: RoomSelection | None = None
        #: Acks seen / commands seen / values that were our own echoed writes.
        self.acks_seen = 0
        self.commands_seen = 0
        self.echoes_ignored = 0

    def ingest(self, value: Any, *, now: float | None = None) -> RoomSelection | None:
        """Feed one raw DP 105 value in. Returns the ack it decoded, or None.

        Safe to call with anything, including None and frames of other commands.
        Only a base64-decoded, checksum-valid 0xAA frame with cmd 0x12 or 0x22
        does anything, and only 0x22 updates the reported selection.
        """
        decoded = decode_wire_value(value)
        if decoded is None:
            return None
        if not decoded.from_robot:
            # A local hex-encoded write echoed back. Explicitly not data.
            self.echoes_ignored += 1
            return None
        frame = decoded.frame
        if frame.header != HEADER_AA:
            return None
        if frame.cmd not in (CMD_CLEAN_SELECT_ROOMS, CMD_CLEAN_SELECT_ROOMS_TO_APP):
            return None

        layout, room_ids, passes = decode_room_selection(frame.data)
        observation = RoomSelection(
            cmd=frame.cmd,
            layout=layout,
            room_ids=room_ids,
            passes=passes,
            raw_hex=frame.data.hex(),
            at=time.monotonic() if now is None else now,
        )

        if frame.cmd == CMD_CLEAN_SELECT_ROOMS:
            self.commands_seen += 1
            self.last_command = observation
            return None

        self.acks_seen += 1
        self.room_ids = room_ids
        self.passes = passes
        self.raw_hex = observation.raw_hex
        self.layout = layout
        self.observed_at = observation.at
        return observation

    @property
    def age(self) -> float | None:
        """Seconds since the last ack, or None if there has never been one."""
        if self.observed_at is None:
            return None
        return max(0.0, time.monotonic() - self.observed_at)

    def as_attributes(self) -> dict[str, Any]:
        """Attribute payload for the selected-rooms sensor."""
        age = self.age
        return {
            "room_ids": self.room_ids,
            "passes": self.passes,
            "layout": self.layout,
            "raw": self.raw_hex,
            "age": None if age is None else round(age, 1),
            "acks_seen": self.acks_seen,
            "commands_seen": self.commands_seen,
            "last_command": (
                self.last_command.as_dict() if self.last_command else None
            ),
        }

    def reset(self) -> None:
        """Forget everything — for a new job, or for tests."""
        self.room_ids = None
        self.passes = None
        self.raw_hex = None
        self.layout = "unknown"
        self.observed_at = None
        self.last_command = None
        self.acks_seen = 0
        self.commands_seen = 0
        self.echoes_ignored = 0


# --- robot self-description: saved maps, schedules, features, wear -----------
#
# Everything below decodes the *replies* to the app's own named getters, all of
# which came back on 2026-09-05 once DP 105 was written as base64 rather than
# hex (an earlier run predates that fix and every reply in it is a mangled
# echo).
#
# Two of these layouts (saved maps, schedules) are **not** transcribed from the
# vendor app: it parses both in its native layer, so there is no JS parser to
# copy and the layouts rest on the frames themselves — five real frames from one
# unit. The other three (mop cloth, rotate angle, supported features) reproduce
# the app's own parsers byte for byte, including their validation.


#: The exact frames the vendor app itself sends from a named getter, base64.
#:
#: tinytuya base64-decodes a raw datapoint; the app's hex notation is its SDK's
#: convention (the Tuya SDK hex-decodes instead). Both put the same bytes on the
#: wire — see `encode_frame_b64`. The hex each entry corresponds to is in the
#: comment beside it, and the round-trip is asserted by the test suite.
#:
#: **Nothing else may ever be written to DP 105 by this integration.** Every
#: dangerous map operation on this robot — reset the map, delete a saved map,
#: overwrite the no-go zones — is also a write to this one datapoint, so a
#: datapoint-level denylist cannot protect it. The protection is that this dict
#: is the only source of DP 105 write payloads, and every frame in it is a
#: read-only getter.
GETTER_FRAMES: dict[str, str] = {
    # bb00010101 — eSupportFeatures (0x01), no payload
    "supported_features": encode_frame_b64(0x01, header=HEADER_BB),
    # bb00011717 — eGetSavedMapName (0x17), no payload
    "saved_map_names": encode_frame_b64(0x17, header=HEADER_BB),
    # bb00031300ff12 — eGetNamedSchedule (0x13); 00 = event/version, ff = "all"
    "local_schedule": encode_frame_b64(0x13, b"\x00\xff", header=HEADER_BB),
    # bb000238ff37 — eMopClothAssemDirty (0x38), ff = "report now"
    "mop_cloth_dirty": encode_frame_b64(0x38, b"\xff", header=HEADER_BB),
    # bb000230ff2f — eGetMapRotateAngle (0x30), ff = "report now"
    "rotate_angle": encode_frame_b64(0x30, b"\xff", header=HEADER_BB),
    # aa013030 — eAll (0x30) in the *basic* table: dump every area report
    "all_areas": encode_frame_b64(0x30, header=HEADER_AA),
}


@dataclass(frozen=True)
class SavedMap:
    """One stored map on the robot: its id and the name the user gave it."""

    map_id: int
    name: str

    def as_dict(self) -> dict[str, Any]:
        """Attribute-friendly form."""
        return {"id": self.map_id, "name": self.name}


@dataclass(frozen=True)
class ScheduleEntry:
    """One entry from the robot's on-board named-schedule store.

    The only field here that is *interpreted* rather than read is `time`. In
    particular:

    * `days_mask` is seven meaningful bits, and **which bit is which weekday is
      unverified**. Every schedule on the reference unit is 0x7F (all seven
      days), so the real frames cannot distinguish Monday-first from
      Sunday-first, nor bit 0 from bit 6. Do not render a weekday list from this
      until a partial-week schedule has been captured; expose the mask.

      The leading hypothesis is **bit 0 = Sunday** through bit 6 = Saturday:
      the vendor app declares exactly one weekday enum,
      ``WEEK_DAYS={SUNDAY:0,...,SATURDAY:6}``, and uses it everywhere it
      indexes a day. Suggestive, not proof — every use of it in the app's
      JavaScript belongs to the *older* one-time-per-day cloud schedule, while
      these named schedules are parsed in the app's native layer, which is
      where the bitmask is actually built. Settling it costs one experiment:
      save a schedule for a single weekday in the app, re-read the getter, and
      see which bit is set.
    * `unknown_tail` is the four bytes between the room list and the name
      length, kept as hex because nothing is known about them. On every real
      entry they are `00020001`. Candidates floated in the field notes: fan
      speed, water level, repeat count, map id (the trailing `01` matches the
      current map id) — all guesses, none tested.
    """

    enabled: bool
    #: 7 bits, one per weekday. **Bit→weekday order is UNVERIFIED** (see above).
    days_mask: int
    hour: int
    minute: int
    room_ids: tuple[int, ...]
    name: str
    #: Hex of the 4 undecoded bytes that follow the room list.
    unknown_tail: str

    @property
    def time(self) -> str:
        """The start time as `HH:MM`."""
        return f"{self.hour:02d}:{self.minute:02d}"

    def as_dict(self) -> dict[str, Any]:
        """Attribute-friendly form."""
        return {
            "enabled": self.enabled,
            "days_mask": self.days_mask,
            "time": self.time,
            "room_ids": list(self.room_ids),
            "name": self.name,
            "unknown_tail": self.unknown_tail,
        }


def decode_saved_maps(data: bytes) -> list[SavedMap] | None:
    """Decode a 0xBB cmd-0x18 payload into saved maps, or None if malformed.

    Layout: `[count] { [map_id] [name_len] [utf-8 name] }*`. Verified byte-exact
    against the real two-map frame from 2026-09-05.

    The payload must be consumed *exactly*: a trailing byte, a short name or a
    count that disagrees with the body is rejected outright rather than
    partially decoded, on the same principle as `decode_ai_objects`. An empty
    list (count 0) and None (malformed) are different answers.
    """
    if not data:
        return None
    count = data[0]
    offset = 1
    maps: list[SavedMap] = []
    for _ in range(count):
        if offset + 2 > len(data):
            return None
        map_id = data[offset]
        name_len = data[offset + 1]
        offset += 2
        if offset + name_len > len(data):
            return None
        maps.append(
            SavedMap(
                map_id=map_id,
                name=data[offset : offset + name_len].decode("utf-8", errors="replace"),
            )
        )
        offset += name_len
    if offset != len(data):
        return None
    return maps


def decode_schedules(data: bytes) -> list[ScheduleEntry] | None:
    """Decode a 0xBB cmd-0x14 payload into schedules, or None if malformed.

    Layout::

        [00] [count]
        per entry: [enabled] [days_mask] [HH] [MM] [room_count]
                   [room_id × room_count] [4 unknown bytes] [name_len] [name]

    The leading byte is presumably the schedule event/version byte the request
    echoes back (the getter sends `00 ff`); it is **not interpreted here** —
    only skipped. Verified byte-exact against the real five-entry frame from
    2026-09-05, which consumes 118 of 118 data bytes.

    Nothing range-checks `HH`/`MM` deliberately: no out-of-range value has ever
    been observed, and inventing a rejection rule risks discarding a real frame
    in which firmware uses a sentinel for "unset". Length consistency is the
    only validation, and it must hold exactly.
    """
    if len(data) < 2:
        return None
    count = data[1]
    offset = 2
    entries: list[ScheduleEntry] = []
    for _ in range(count):
        if offset + 5 > len(data):
            return None
        enabled, days_mask, hour, minute, room_count = data[offset : offset + 5]
        offset += 5
        if offset + room_count > len(data):
            return None
        room_ids = tuple(data[offset : offset + room_count])
        offset += room_count
        # 4 undecoded bytes, then the name length byte.
        if offset + 5 > len(data):
            return None
        tail = data[offset : offset + 4]
        name_len = data[offset + 4]
        offset += 5
        if offset + name_len > len(data):
            return None
        name = data[offset : offset + name_len].decode("utf-8", errors="replace")
        offset += name_len
        entries.append(
            ScheduleEntry(
                enabled=bool(enabled),
                days_mask=days_mask,
                hour=hour,
                minute=minute,
                room_ids=room_ids,
                name=name,
                unknown_tail=tail.hex(),
            )
        )
    if offset != len(data):
        return None
    return entries


def decode_mop_cloth_dirty(data: bytes) -> tuple[int, int] | None:
    """Decode a 0xBB cmd-0x39 payload into `(version, percent)`, or None.

    Exactly two bytes, `[version, dirtyPercent]` — the app's own
    `parseMopClothDirty`, including its length check, so any other length is
    rejected.

    This command was misread as a "map frame" throughout the 2026-09-01/02
    capture logs: every "0x39 MAP FRAME, 2 bytes" line there is really the
    mop-cloth dirt report (`01 06` = 6 % dirty).
    """
    if len(data) != 2:
        return None
    return (data[0], data[1])


def decode_supported_features(data: bytes) -> bytes | None:
    """Decode a 0xBB cmd-0x02 payload into the feature bitmap, or None.

    The payload is `[bitmap_len] [bitmap…]`, and the app validates
    `data[0] + 1 == len(data)` before using it. That check is reproduced here,
    and the **length byte is stripped**: what comes back is the bitmap alone.

    The bits are not decoded. The vendor's feature-flag table has not been
    transcribed, and guessing at flags that gate map and mop behaviour is
    exactly the kind of plausible-but-wrong reading that cost this project the
    0x12 no-mop-zone detour. Callers get the bytes.
    """
    if not data:
        return None
    if data[0] + 1 != len(data):
        return None
    return bytes(data[1:])


def decode_rotate_angle(data: bytes) -> int | None:
    """Decode a 0xBB cmd-0x31 payload into degrees, or None if too short.

    `(256 * data[0] + data[1]) % 360`, from the app's `handleRotateAngleCmd`,
    which requires two bytes and ignores anything past them. Reproduced as-is,
    over-long payloads included: the app tolerates them and there is no evidence
    a longer payload means something different.
    """
    if len(data) < 2:
        return None
    return (256 * data[0] + data[1]) % 360


def derive_room_names(schedules: Iterable[ScheduleEntry]) -> dict[int, str]:
    """Map room id → name using **single-room schedules only**.

    The robot has no room-name table on the LAN protocol, but this unit's owner
    named each schedule after the rooms it targets, which makes the schedule
    store an accidental room-name source: `classroom` targets room 0, so room 0
    is the classroom.

    **Multi-room schedules are excluded, and that exclusion is the whole point.**
    A two-room entry's name describes the *pair* — "hall and kitchen" for ids
    3 and 2 — and there is no way to tell which half belongs to which id, or
    even that the name decomposes at all ("mudd" covers ids 5 and 4). Splitting
    on " and " would produce confident, wrong names; assigning the whole name to
    both ids would produce two rooms called "hall and kitchen". Neither is
    better than saying nothing, so nothing is what this says.

    Entries with an empty name contribute nothing. Where two single-room
    schedules name the same room, the later one wins — arbitrary, but the
    situation has never been observed and the alternative is equally arbitrary.

    This is a *heuristic keyed to one household's naming habit*, not a protocol
    feature. It is right for this unit and may be empty or nonsense for another,
    which is why the caller must treat an empty dict as normal.
    """
    names: dict[int, str] = {}
    for entry in schedules:
        if len(entry.room_ids) != 1:
            continue
        if not entry.name:
            continue
        names[entry.room_ids[0]] = entry.name
    return names


class RobotInfoTracker:
    """Accumulates the robot's replies to the app's named getters.

    One per config entry, owned by the coordinator, for the same reason
    `AiObjectTracker` is stateful: `dps["105"]` holds only the most recent DP
    105 value at poll time, and these replies arrive one frame at a time in
    response to separate getter writes. Without somewhere to hold them, each
    reply would erase the last.

    Five (header, cmd) pairs are owned, all in the 0xBB table — saved maps
    (0x18), named schedules (0x14), supported features (0x02), mop-cloth dirt
    (0x39) and map rotate angle (0x31). Matching is on the **pair**, never the
    command byte alone: 0x12 means no-mop zones under 0xBB and a room-clean
    command under 0xAA, and reading one against the other's table is how that
    command was briefly mistaken for the room partition.

    Everything is None or empty until a frame actually arrives. Nothing here
    writes; `GETTER_FRAMES` holds the payloads a caller may send, and sending
    them is the caller's decision.
    """

    #: The (header, cmd) pairs this tracker consumes.
    _OWNED = frozenset(
        {
            (HEADER_BB, CMD_SUPPORT_FEATURES_TO_APP),
            (HEADER_BB, CMD_NAMED_SCHEDULE_TO_APP),
            (HEADER_BB, CMD_SAVED_MAP_NAMES_TO_APP),
            (HEADER_BB, CMD_MAP_ROTATE_ANGLE_TO_APP),
            (HEADER_BB, CMD_MOP_CLOTH_DIRTY_TO_APP),
        }
    )

    def __init__(self) -> None:
        """Start with nothing observed."""
        #: Saved maps from the last 0x18 reply, or None if never seen.
        self.saved_maps: list[SavedMap] | None = None
        #: Schedules from the last 0x14 reply, or None if never seen.
        self.schedules: list[ScheduleEntry] | None = None
        #: Mop-cloth dirt, percent and its version byte, from the last 0x39.
        self.mop_cloth_percent: int | None = None
        self.mop_cloth_version: int | None = None
        #: The raw feature bitmap from the last 0x02, length byte stripped.
        self.feature_bits: bytes | None = None
        #: Map rotate angle in degrees from the last 0x31.
        self.rotate_angle: int | None = None
        #: room id → name, derived from `schedules`; recomputed on every change.
        self.room_names: dict[int, str] = {}
        #: Owned frames decoded / owned frames rejected as malformed.
        self.frames_seen = 0
        self.frames_rejected = 0
        #: `time.monotonic()` of the last consumed frame.
        self.observed_at: float | None = None

    def ingest(self, value: Any, *, now: float | None = None) -> bool:
        """Feed one raw DP 105 value in. True when this tracker consumed it.

        Safe to call with anything, including None, junk, 0xAA frames and frames
        of other commands. Values this tracker does not own return False without
        touching any state or counter — including our own hex-encoded writes
        echoed back, which are traffic we sent, not observations of the robot.

        A frame this tracker *does* own but cannot decode also returns False,
        but increments `frames_rejected`: nothing was consumed, because nothing
        was stored. Only a successful decode returns True.
        """
        decoded = decode_wire_value(value)
        if decoded is None:
            return False
        if not decoded.from_robot:
            # A local hex-encoded write echoed back. Explicitly not data, and
            # explicitly not counted — an echo is our own traffic.
            return False
        frame = decoded.frame
        if (frame.header, frame.cmd) not in self._OWNED:
            return False

        if not self._apply(frame):
            self.frames_rejected += 1
            _LOGGER.debug(
                "bObsweep: rejected malformed 0x%02X/0x%02X frame: %s",
                frame.header,
                frame.cmd,
                frame.data.hex(),
            )
            return False

        self.frames_seen += 1
        self.observed_at = time.monotonic() if now is None else now
        return True

    def _apply(self, frame: TransportFrame) -> bool:
        """Decode one owned frame into state. False if it was malformed."""
        if frame.cmd == CMD_SAVED_MAP_NAMES_TO_APP:
            maps = decode_saved_maps(frame.data)
            if maps is None:
                return False
            self.saved_maps = maps
            return True

        if frame.cmd == CMD_NAMED_SCHEDULE_TO_APP:
            schedules = decode_schedules(frame.data)
            if schedules is None:
                return False
            self.schedules = schedules
            self.room_names = derive_room_names(schedules)
            return True

        if frame.cmd == CMD_MOP_CLOTH_DIRTY_TO_APP:
            reading = decode_mop_cloth_dirty(frame.data)
            if reading is None:
                return False
            self.mop_cloth_version, self.mop_cloth_percent = reading
            return True

        if frame.cmd == CMD_SUPPORT_FEATURES_TO_APP:
            bits = decode_supported_features(frame.data)
            if bits is None:
                return False
            self.feature_bits = bits
            return True

        if frame.cmd == CMD_MAP_ROTATE_ANGLE_TO_APP:
            angle = decode_rotate_angle(frame.data)
            if angle is None:
                return False
            self.rotate_angle = angle
            return True

        return False  # unreachable while _OWNED and this branch table agree

    @property
    def age(self) -> float | None:
        """Seconds since the last consumed frame, or None if there is none."""
        if self.observed_at is None:
            return None
        return max(0.0, time.monotonic() - self.observed_at)

    def as_attributes(self) -> dict[str, Any]:
        """Attribute payload for the robot-info diagnostic sensor."""
        age = self.age
        return {
            "saved_maps": (
                None
                if self.saved_maps is None
                else [entry.as_dict() for entry in self.saved_maps]
            ),
            "schedules": (
                None
                if self.schedules is None
                else [entry.as_dict() for entry in self.schedules]
            ),
            "room_names": dict(self.room_names),
            "mop_cloth_percent": self.mop_cloth_percent,
            "mop_cloth_version": self.mop_cloth_version,
            "feature_bits": None if self.feature_bits is None else self.feature_bits.hex(),
            "rotate_angle": self.rotate_angle,
            "age": None if age is None else round(age, 1),
            "frames_seen": self.frames_seen,
            "frames_rejected": self.frames_rejected,
        }

    def reset(self) -> None:
        """Forget everything — for a re-probe, or for tests."""
        self.saved_maps = None
        self.schedules = None
        self.mop_cloth_percent = None
        self.mop_cloth_version = None
        self.feature_bits = None
        self.rotate_angle = None
        self.room_names = {}
        self.frames_seen = 0
        self.frames_rejected = 0
        self.observed_at = None
