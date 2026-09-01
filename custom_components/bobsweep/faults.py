"""Decoding of the bObsweep fault/error datapoints into named faults.

The robot reports faults on up to three datapoints, and none of them carry a
name — each is a bitmask indexed against a table that lives in the vendor app's
JS bundle. Those tables are transcribed into `FamilySpec` (see `const.py`); this
module turns a raw datapoint value plus the family's tables into fault *names*.

Three channels, two wire formats:

* **DP 18** (`COMMAND_ERROR_REPORTED1`, all families) — plain integer bitmask,
  `bit i` set means `fault_bits[i]`. The tables differ per family: 30 entries on
  SLAM, 15 on Vision, 15 on Random.
* **DP 113** (`COMMAND_ERROR_REPORTED2`, SLAM only) — plain integer bitmask over
  the 30-entry `fault_bits2` table.
* **DP 131** (`COMMAND_FAULT_BITS`, SLAM only) — *not* an integer: a byte array
  (hex string over the wire) packed LSB-first, `bitIndex = byteIndex * 8 + bit`,
  over the 40-entry `fault_bits_raw` table. Bits 0-29 repeat DP 113's list; bits
  30-39 are ten station/water faults that exist nowhere else.

Two behaviours are inherited from the app itself:

* `SKIP_TROUBLES` (`NoteMopWaterLow`, `NoteMoppingIsOff`) are informational
  notes, not faults. They are decoded and reported separately as `notes` and
  never counted as a problem.
* Entries that are not faults at all (Random's bit 0, `Normal`) are dropped.

Nothing here raises: every entry point degrades to "nothing decodable" and logs
at debug, because the callers are HA state properties.
"""

from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field
from typing import Any, Iterable

from .const import NON_FAULT_KEYS, SKIP_TROUBLES, FamilySpec

_LOGGER = logging.getLogger(__name__)

# The sensor's state when nothing is wrong. A real state (not `unknown`) so an
# automation can trigger on the transition away from it.
FAULT_STATE_NONE = "none"

# Values that mean "no fault reported" on any of the three channels.
_EMPTY_VALUES: tuple[Any, ...] = (None, "", "0", 0, False, "none", "no_error")

_CAMEL_BOUNDARY = re.compile(r"(?<=[a-z0-9])(?=[A-Z])")


def slugify_trouble(key: str) -> str:
    """Turn an app trouble identifier into a stable snake_case state slug.

    `TroubleBobStuck` -> `bob_stuck`, `NoteMopWaterLow` -> `mop_water_low`,
    `LowBattery` -> `low_battery`. The `Trouble`/`Note` prefix is dropped because
    it carries no information — the entity is already a fault sensor — and the
    human-facing English lives in `strings.json` under these slugs.
    """
    name = key
    for prefix in ("Trouble", "Note"):
        if name.startswith(prefix) and len(name) > len(prefix):
            name = name[len(prefix):]
            break
    return _CAMEL_BOUNDARY.sub("_", name).lower()


@dataclass(frozen=True)
class FaultReport:
    """The decoded fault state of one robot at one moment."""

    # Active fault slugs, most-significant first, de-duplicated.
    faults: tuple[str, ...] = ()
    # Active informational notes (the app's SKIP_TROUBLES), never faults.
    notes: tuple[str, ...] = ()
    # Raw datapoint values, keyed by the attribute name of the channel.
    raw: dict[str, Any] = field(default_factory=dict)
    # True when a channel held a non-empty value that could not be parsed. Callers
    # keep the old "any truthy value means trouble" behaviour in that case
    # rather than silently reporting a healthy robot.
    undecodable: bool = False

    @property
    def has_fault(self) -> bool:
        """True when at least one real (non-note) fault is active."""
        return bool(self.faults) or self.undecodable

    @property
    def state(self) -> str:
        """The single fault name to show as the sensor state."""
        if self.faults:
            return self.faults[0]
        return FAULT_STATE_NONE


# --- raw value parsing -------------------------------------------------------
def _int_bits(value: Any) -> set[int] | None:
    """Bit indices set in an integer bitmask DP (DP 18 / DP 113).

    Accepts an int, a bool, an integral float, a decimal string or — as a last
    resort, because Tuya is inconsistent — a hex string. Returns None when the
    value is non-empty but unparseable.
    """
    number: int | None
    if isinstance(value, bool):
        number = int(value)
    elif isinstance(value, int):
        number = value
    elif isinstance(value, float):
        number = int(value) if value.is_integer() else None
    elif isinstance(value, str):
        text = value.strip()
        number = None
        for base in (10, 16):
            try:
                number = int(text, base)
            except ValueError:
                continue
            break
    else:
        number = None

    if number is None or number < 0:
        return None
    return {i for i in range(number.bit_length()) if number >> i & 1}


def _packed_bits(value: Any) -> set[int] | None:
    """Bit indices set in a byte-array DP (DP 131), read LSB-first.

    `bitIndex = byteIndex * 8 + bit`. A hex string is parsed byte by byte in the
    order it arrives; a list of ints is treated as those bytes; a plain int is
    read directly, which is equivalent (bit i of a little-endian integer is bit
    i%8 of byte i//8). Returns None when the value is non-empty but unparseable.
    """
    data: bytes | None = None

    if isinstance(value, (bytes, bytearray)):
        data = bytes(value)
    elif isinstance(value, (list, tuple)):
        if all(
            isinstance(b, int) and not isinstance(b, bool) and 0 <= b <= 255
            for b in value
        ):
            data = bytes(value)
    elif isinstance(value, bool):
        data = bytes([int(value)])
    elif isinstance(value, int):
        # Already an integer: LSB-first packing makes this identical to bytes.
        return _int_bits(value)
    elif isinstance(value, str):
        text = value.strip().lower()
        if text.startswith("0x"):
            text = text[2:]
        if text and all(c in "0123456789abcdef" for c in text):
            if len(text) % 2:
                text = "0" + text
            try:
                data = bytes.fromhex(text)
            except ValueError:
                data = None

    if data is None:
        return None
    return {
        index * 8 + bit
        for index, byte in enumerate(data)
        for bit in range(8)
        if byte >> bit & 1
    }


# --- table lookup ------------------------------------------------------------
def _table_for(spec: FamilySpec, table_attr: str) -> tuple[str, ...]:
    """Return a family's fault-name table, applying the left/right swap.

    See `FamilySpec.left_right_faults_reversed` — the app swaps indices 2 and 3
    of the DP 18 table on some models.
    """
    table: tuple[str, ...] = getattr(spec, table_attr, ()) or ()
    if (
        table_attr == "fault_bits"
        and spec.left_right_faults_reversed
        and len(table) > 3
    ):
        swapped = list(table)
        swapped[2], swapped[3] = swapped[3], swapped[2]
        return tuple(swapped)
    return table


def _names(bits: Iterable[int], table: tuple[str, ...]) -> list[str]:
    """Map bit indices onto table entries, dropping bits with no entry."""
    names: list[str] = []
    for bit in sorted(bits):
        if bit >= len(table):
            # A bit the app's own table doesn't name: newer firmware, or the
            # value isn't really this channel. Ignore rather than invent a name.
            _LOGGER.debug(
                "bObsweep fault bit %s has no entry in a %s-entry table",
                bit,
                len(table),
            )
            continue
        names.append(table[bit])
    return names


# The three fault channels, in reporting-priority order: DP 18 is the app's
# primary "topic" list, DP 113 the secondary one, DP 131 the superset that only
# adds station/water faults at its tail. Within a channel, the lower bit index
# wins. This ordering is mechanical and stable rather than a severity judgement
# the datapoint tables don't actually support.
FAULT_CHANNELS: tuple[tuple[str, str, str], ...] = (
    ("dp_error", "fault_bits", "int"),
    ("dp_error2", "fault_bits2", "int"),
    ("dp_error3", "fault_bits_raw", "packed"),
)


def decode_channel(
    spec: FamilySpec, value: Any, table_attr: str, kind: str
) -> tuple[list[str], bool]:
    """Decode one channel's raw value into app trouble keys.

    Returns `(keys, undecodable)`. `undecodable` is True when the value was
    non-empty but could not be parsed at all.
    """
    try:
        if value in _EMPTY_VALUES:
            return [], False
    except Exception:  # noqa: BLE001 - an exotic value may not compare cleanly
        pass

    table = _table_for(spec, table_attr)
    if not table:
        # No transcribed table for this family/channel: nothing can be named,
        # but a non-empty value still means something is wrong.
        return [], True

    try:
        bits = _packed_bits(value) if kind == "packed" else _int_bits(value)
    except Exception:  # noqa: BLE001 - a state property must never raise
        _LOGGER.debug("bObsweep: failed to parse fault value %r", value, exc_info=True)
        return [], True

    if bits is None:
        _LOGGER.debug("bObsweep: unparseable %s fault value %r", kind, value)
        return [], True

    return _names(bits, table), False


def decode_faults(spec: FamilySpec, data: dict[str, Any] | None) -> FaultReport:
    """Decode every fault channel the family has into a single report."""
    data = data or {}
    raw: dict[str, Any] = {}
    fault_slugs: list[str] = []
    note_slugs: list[str] = []
    undecodable = False

    for dp_attr, table_attr, kind in FAULT_CHANNELS:
        dp = getattr(spec, dp_attr, None)
        if dp is None:
            continue
        value = data.get(dp)
        raw[dp_attr] = value
        keys, bad = decode_channel(spec, value, table_attr, kind)
        undecodable = undecodable or bad
        for key in keys:
            if key in NON_FAULT_KEYS:
                continue
            slug = slugify_trouble(key)
            target = note_slugs if key in SKIP_TROUBLES else fault_slugs
            if slug not in target:
                target.append(slug)

    return FaultReport(
        faults=tuple(fault_slugs),
        notes=tuple(note_slugs),
        raw=raw,
        undecodable=undecodable,
    )


def fault_state_options(spec: FamilySpec) -> list[str]:
    """Every state the fault sensor can report for this family.

    Used for the enum sensor's `options`, so the state is always one of a
    declared, translated set.
    """
    options = [FAULT_STATE_NONE]
    for _dp_attr, table_attr, _kind in FAULT_CHANNELS:
        for key in getattr(spec, table_attr, ()) or ():
            if key in NON_FAULT_KEYS or key in SKIP_TROUBLES:
                continue
            slug = slugify_trouble(key)
            if slug not in options:
                options.append(slug)
    return options
