"""User-supplied room names for bObsweep room awareness.

The robot does **not** hand over a room-name table. `eRoomName` is a write-only
command on DP 105 (there is no `eRoomNameToAPP` handler in the vendor bundle),
so names live in app/cloud state, not on the device. The one exception found on
2026-09-05 is indirect: `getLocalSchedule` returns *named schedules*, and a
schedule that targets exactly one room effectively names that room. That gets a
handful of rooms for free (see `transport.RobotInfoTracker.room_names`) and
leaves the rest anonymous.

This module is the "rest": a per-config-entry `{room_id: name}` map the user
fills in with the `bobsweep.set_room_name` service. It is an *override* layer —
`BobsweepCoordinator.room_names()` overlays it on the robot-derived names, so a
user entry always wins.

**Why `Store` and not config-entry options** — identical reasoning to
`zones.ZoneStore`: options changes reload the entry, and naming a room should
not tear down the coordinator, the listener thread and any in-flight zone
capture. See that class's docstring; this one deliberately mirrors it, including
handing `async_save` an already-materialised plain dict rather than a closure
(HA 2025.11+ serialises `Store` data on a worker thread).

Storage key: `bobsweep_room_names.<entry_id>`.

On-disk schema (version 1)::

    {"version": 1, "names": {"<room_id>": "<name>"}}

Room ids are integers everywhere in this integration (the robot reports them as
bytes in a 0x22 ack), but JSON object keys can only be strings. The conversion
happens exactly here, in `_parse` and `_serialise`, so nothing downstream has to
remember which side of the wire it is on.
"""

from __future__ import annotations

import logging
from typing import Any

_LOGGER = logging.getLogger(__name__)

STORAGE_VERSION = 1
STORAGE_KEY_TEMPLATE = "bobsweep_room_names.{entry_id}"

#: Long enough for any sensible room name, short enough that a pasted blob or a
#: mis-wired template cannot quietly become a permanent entity attribute.
MAX_ROOM_NAME_LENGTH = 64


class RoomNameError(ValueError):
    """A room-name payload could not be accepted."""


def _parse(raw: Any) -> dict[int, str]:
    """Turn stored JSON into `{int: str}`, raising `RoomNameError` if unusable."""
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise RoomNameError(
            f"room-name data must be a mapping, got {type(raw).__name__}"
        )

    version = raw.get("version", STORAGE_VERSION)
    if isinstance(version, int) and version > STORAGE_VERSION:
        raise RoomNameError(
            f"room-name data is version {version}, this integration understands "
            f"up to version {STORAGE_VERSION}"
        )

    names_raw = raw.get("names", {})
    if not isinstance(names_raw, dict):
        raise RoomNameError("'names' must be a mapping of room id to name")

    names: dict[int, str] = {}
    for key, value in names_raw.items():
        # JSON gives string keys; a hand-edited file may well give int ones.
        if isinstance(key, bool) or not isinstance(key, (int, str)):
            raise RoomNameError(f"room id {key!r} is not an integer")
        try:
            room_id = int(key)
        except (TypeError, ValueError) as err:
            raise RoomNameError(f"room id {key!r} is not an integer") from err
        if not isinstance(value, str) or not value.strip():
            raise RoomNameError(f"room {room_id}: name must be a non-empty string")
        names[room_id] = value.strip()[:MAX_ROOM_NAME_LENGTH]
    return names


def _serialise(names: dict[int, str]) -> dict[str, Any]:
    """Snapshot to plain JSON-safe data (string keys, as JSON requires)."""
    return {
        "version": STORAGE_VERSION,
        "names": {str(room_id): name for room_id, name in sorted(names.items())},
    }


class RoomNameStore:
    """`homeassistant.helpers.storage.Store` wrapper around `{room_id: name}`.

    Kept thin and HA-aware only in the constructor, matching `zones.ZoneStore`.
    """

    def __init__(self, hass: Any, entry_id: str) -> None:
        """Create (but do not yet load) the store for one config entry."""
        # Imported lazily for the same reason `zones.py` does it: the module
        # should be importable with no Home Assistant installed.
        from homeassistant.helpers.storage import Store  # noqa: PLC0415

        self._store: Any = Store(
            hass, STORAGE_VERSION, STORAGE_KEY_TEMPLATE.format(entry_id=entry_id)
        )
        self._names: dict[int, str] = {}

    @property
    def names(self) -> dict[int, str]:
        """The current overrides, `{room_id: name}`. Live, not a copy."""
        return self._names

    async def async_load(self) -> dict[int, str]:
        """Load the overrides, degrading to none on unusable data.

        A corrupt or future-versioned file must not break integration setup --
        the robot still vacuums, and every room simply stays unnamed. The file
        is left on disk so the user can fix it by hand.
        """
        raw = await self._store.async_load()
        try:
            self._names = _parse(raw)
        except RoomNameError as err:
            _LOGGER.error(
                "bObsweep: stored room-name data is unusable (%s); continuing "
                "with no room-name overrides. The file has been left in place",
                err,
            )
            self._names = {}
        return self._names

    async def async_set(self, room_id: int, name: str) -> None:
        """Name one room and persist immediately."""
        cleaned = str(name).strip()
        if not cleaned:
            raise RoomNameError("a room name cannot be empty")
        self._names[int(room_id)] = cleaned[:MAX_ROOM_NAME_LENGTH]
        await self._async_save()

    async def async_delete(self, room_id: int) -> None:
        """Drop one override, if it exists. Persists only when it did."""
        if self._names.pop(int(room_id), None) is None:
            return
        await self._async_save()

    async def async_remove(self) -> None:
        """Delete the backing file (used when the config entry is removed)."""
        await self._store.async_remove()

    async def _async_save(self) -> None:
        """Persist. The dict is materialised here, on the event loop."""
        await self._store.async_save(_serialise(self._names))
