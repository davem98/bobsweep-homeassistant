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
CMD_MAP_ROTATE_ANGLE_TO_APP = 0x31  # eMapRotateAngleToApp (49) — uint16 degrees

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
