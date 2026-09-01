"""Zone storage and lookup for bObsweep room awareness.

A *zone* is a named polygon in raw map-cell units, plus enough metadata to know
when it has stopped meaning anything. Rectangles are not a separate concept --
they are polygons with four vertices -- so there is exactly one shape type to
store, validate and hit-test.

**Why `Store` and not config-entry options.** The integration reloads the config
entry whenever its options change (`__init__._async_update_listener`). Zones are
written *during* a capture session, by services the user is calling while
driving the robot around a room; putting them in options would tear down the
entry, the coordinator and the in-flight capture buffer on every save. `Store`
is a separate file with no reload semantics, which is what this data wants.

Storage key: `bobsweep_zones.<entry_id>` -- per config entry, so two robots (or
one robot re-added) never share or clobber a zone set.

On-disk schema (version 1)::

    {"version": 1, "map_id": <int|null>, "dock_ref": [x, y]|null,
     "extent": [minx, miny, maxx, maxy]|null,
     "zones": [{"id": <uuid>, "name": str, "points": [[x, y], ...],
                "source": "taught"|"learned", "room_id": <int|null>,
                "created": <iso8601>}]}

`id` is a stable uuid4 assigned once at creation. It is what the HA vacuum
segment API uses as a segment id, so it must survive renames, re-imports and
edits -- never derive it from the name.
"""

from __future__ import annotations

import logging
import uuid
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from typing import Any, Iterable, Sequence

from .geometry import (
    BBox,
    Point,
    bounding_box,
    point_in_bbox,
    point_in_polygon,
    polygon_area,
)

_LOGGER = logging.getLogger(__name__)

STORAGE_VERSION = 1
STORAGE_KEY_TEMPLATE = "bobsweep_zones.{entry_id}"

SOURCE_TAUGHT = "taught"
SOURCE_LEARNED = "learned"
VALID_SOURCES = (SOURCE_TAUGHT, SOURCE_LEARNED)

# A polygon needs three vertices to enclose anything.
MIN_ZONE_POINTS = 3


class ZoneError(ValueError):
    """A zone payload could not be accepted."""


def _utcnow_iso() -> str:
    """Timestamp helper, kept local so this module needs no HA imports."""
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


@dataclass(frozen=True)
class Zone:
    """One named polygon in raw map-cell units."""

    id: str
    name: str
    points: tuple[Point, ...]
    source: str = SOURCE_TAUGHT
    # The robot's own room id, once a way is found to read one off DP 105's
    # eRoomSettings/eRoomName tables. Always None today, and deliberately kept
    # distinct from `id`: this id is stable across firmware, the robot's is not.
    room_id: int | None = None
    created: str = ""

    @property
    def area(self) -> float:
        """Polygon area in square map cells (used for the overlap rule)."""
        return polygon_area(self.points)

    @property
    def bbox(self) -> BBox | None:
        """Axis-aligned bounds, used as the point-in-polygon fast path."""
        return bounding_box(self.points)

    def contains(self, x: float, y: float) -> bool:
        """True when `(x, y)` is inside this zone (boundary counts as inside)."""
        box = self.bbox
        if box is None or not point_in_bbox(x, y, box):
            return False
        return point_in_polygon(x, y, self.points)

    def to_dict(self) -> dict[str, Any]:
        """Serialise to the on-disk / export representation."""
        return {
            "id": self.id,
            "name": self.name,
            "points": [[p[0], p[1]] for p in self.points],
            "source": self.source,
            "room_id": self.room_id,
            "created": self.created,
        }

    @classmethod
    def from_dict(cls, raw: Any) -> Zone:
        """Parse one zone, raising `ZoneError` on anything unusable.

        Strict on purpose: this is the import path as well as the load path, and
        a malformed hand-edited polygon should be rejected loudly rather than
        silently classifying the robot into a nonsense room later.
        """
        if not isinstance(raw, dict):
            raise ZoneError(f"zone must be a mapping, got {type(raw).__name__}")

        name = raw.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ZoneError("zone needs a non-empty 'name'")

        points_raw = raw.get("points")
        if not isinstance(points_raw, (list, tuple)):
            raise ZoneError(f"zone {name!r}: 'points' must be a list")
        points: list[Point] = []
        for item in points_raw:
            if (
                not isinstance(item, (list, tuple))
                or len(item) < 2
                or any(
                    not isinstance(v, (int, float)) or isinstance(v, bool)
                    for v in item[:2]
                )
            ):
                raise ZoneError(f"zone {name!r}: {item!r} is not an [x, y] pair")
            points.append((float(item[0]), float(item[1])))
        if len(points) < MIN_ZONE_POINTS:
            raise ZoneError(
                f"zone {name!r}: a polygon needs at least {MIN_ZONE_POINTS} points, "
                f"got {len(points)}"
            )

        source = raw.get("source", SOURCE_TAUGHT)
        if source not in VALID_SOURCES:
            raise ZoneError(
                f"zone {name!r}: source must be one of {VALID_SOURCES}, got {source!r}"
            )

        room_id = raw.get("room_id")
        if room_id is not None and (
            not isinstance(room_id, int) or isinstance(room_id, bool)
        ):
            raise ZoneError(f"zone {name!r}: 'room_id' must be an integer or null")

        zone_id = raw.get("id")
        if not isinstance(zone_id, str) or not zone_id:
            # Hand-written imports are allowed to omit ids; minting one here is
            # what makes a hand-edited export a first-class input.
            zone_id = uuid.uuid4().hex

        created = raw.get("created")
        if not isinstance(created, str) or not created:
            created = _utcnow_iso()

        return cls(
            id=zone_id,
            name=name.strip(),
            points=tuple(points),
            source=source,
            room_id=room_id,
            created=created,
        )


def new_zone(
    name: str,
    points: Sequence[Point],
    *,
    source: str = SOURCE_TAUGHT,
    room_id: int | None = None,
) -> Zone:
    """Mint a new zone with a fresh stable id and a creation timestamp."""
    return Zone.from_dict(
        {
            "id": uuid.uuid4().hex,
            "name": name,
            "points": [[p[0], p[1]] for p in points],
            "source": source,
            "room_id": room_id,
            "created": _utcnow_iso(),
        }
    )


@dataclass
class ZoneSet:
    """The whole zone document: zones plus the frame they were taught in.

    `dock_ref` and `extent` are what make staleness detectable. The dock does not
    move, so the map cell the robot reports while charging is a fingerprint of
    the coordinate frame; if it drifts, every polygon here is measuring the wrong
    floor. See `rooms.DockDriftMonitor`.
    """

    zones: list[Zone]
    map_id: int | None = None
    dock_ref: Point | None = None
    extent: BBox | None = None

    # --- lookup --------------------------------------------------------------
    def resolve(self, x: float, y: float) -> Zone | None:
        """Return the zone containing `(x, y)`, smallest-area first.

        The overlap rule is "smallest zone wins": a "Pantry" polygon drawn inside
        "Kitchen" must resolve to the pantry, and it is the only rule that gives
        a sensible answer without asking the user to declare a hierarchy. Ties
        (identical areas, e.g. a duplicated zone) break on zone id so the answer
        is stable across restarts rather than dependent on dict ordering.
        """
        hits = [zone for zone in self.zones if zone.contains(x, y)]
        if not hits:
            return None
        hits.sort(key=lambda z: (z.area, z.id))
        return hits[0]

    def by_name(self, name: str) -> Zone | None:
        """Case-insensitive name lookup -- what the delete service resolves with."""
        needle = name.strip().casefold()
        for zone in self.zones:
            if zone.name.casefold() == needle:
                return zone
        return None

    def by_id(self, zone_id: str) -> Zone | None:
        """Look a zone up by its stable id (the vacuum-segment id)."""
        for zone in self.zones:
            if zone.id == zone_id:
                return zone
        return None

    @property
    def names(self) -> list[str]:
        """Every zone name, in storage order."""
        return [zone.name for zone in self.zones]

    # --- mutation ------------------------------------------------------------
    def add(self, zone: Zone, *, replace_existing: bool = True) -> Zone:
        """Add a zone, replacing any same-named one (keeping its stable id).

        Re-teaching a room is the common case, and the segment id must survive
        it: the user has likely already mapped that segment onto an HA area in
        the entity registry, and minting a new id would silently orphan it.
        """
        existing = self.by_name(zone.name)
        if existing is not None:
            if not replace_existing:
                raise ZoneError(f"a zone named {zone.name!r} already exists")
            zone = replace(zone, id=existing.id, created=existing.created)
            self.zones = [z for z in self.zones if z.id != existing.id]
        self.zones.append(zone)
        return zone

    def remove(self, name: str) -> Zone | None:
        """Delete a zone by name; returns it, or None if there was no such zone."""
        zone = self.by_name(name)
        if zone is None:
            return None
        self.zones = [z for z in self.zones if z.id != zone.id]
        return zone

    # --- serialisation -------------------------------------------------------
    def to_dict(self) -> dict[str, Any]:
        """Snapshot the whole document as plain JSON-safe data."""
        return {
            "version": STORAGE_VERSION,
            "map_id": self.map_id,
            "dock_ref": list(self.dock_ref) if self.dock_ref else None,
            "extent": list(self.extent) if self.extent else None,
            "zones": [zone.to_dict() for zone in self.zones],
        }

    @classmethod
    def from_dict(cls, raw: Any) -> ZoneSet:
        """Parse a stored/imported document. Raises `ZoneError` if unusable."""
        if raw is None:
            return cls(zones=[])
        if not isinstance(raw, dict):
            raise ZoneError(f"zone data must be a mapping, got {type(raw).__name__}")

        version = raw.get("version", STORAGE_VERSION)
        if isinstance(version, int) and version > STORAGE_VERSION:
            raise ZoneError(
                f"zone data is version {version}, this integration understands "
                f"up to version {STORAGE_VERSION}"
            )

        zones_raw = raw.get("zones", [])
        if not isinstance(zones_raw, (list, tuple)):
            raise ZoneError("'zones' must be a list")
        zones = [Zone.from_dict(item) for item in zones_raw]

        map_id = raw.get("map_id")
        if map_id is not None and (
            not isinstance(map_id, int) or isinstance(map_id, bool)
        ):
            raise ZoneError("'map_id' must be an integer or null")

        dock_ref = _parse_point(raw.get("dock_ref"), "dock_ref")
        extent = _parse_extent(raw.get("extent"))

        return cls(zones=zones, map_id=map_id, dock_ref=dock_ref, extent=extent)


def _parse_point(raw: Any, field: str) -> Point | None:
    """Parse an optional `[x, y]` pair."""
    if raw is None:
        return None
    if (
        not isinstance(raw, (list, tuple))
        or len(raw) < 2
        or any(not isinstance(v, (int, float)) or isinstance(v, bool) for v in raw[:2])
    ):
        raise ZoneError(f"{field!r} must be an [x, y] pair or null")
    return (float(raw[0]), float(raw[1]))


def _parse_extent(raw: Any) -> BBox | None:
    """Parse an optional `[minx, miny, maxx, maxy]` box."""
    if raw is None:
        return None
    if (
        not isinstance(raw, (list, tuple))
        or len(raw) != 4
        or any(not isinstance(v, (int, float)) or isinstance(v, bool) for v in raw)
    ):
        raise ZoneError("'extent' must be [minx, miny, maxx, maxy] or null")
    return (float(raw[0]), float(raw[1]), float(raw[2]), float(raw[3]))


def zones_extent(zones: Iterable[Zone]) -> BBox | None:
    """Bounding box of every zone in the set, or None when there are none."""
    points: list[Point] = []
    for zone in zones:
        points.extend(zone.points)
    return bounding_box(points)


class ZoneStore:
    """`homeassistant.helpers.storage.Store` wrapper around a `ZoneSet`.

    Kept thin, and kept as the only HA-aware thing in this module, so the zone
    model itself stays testable with no HA installed.

    **HA 2025.11+ serialises `Store` data in a worker thread by default.** Every
    save here therefore hands `async_save` an already-materialised plain-dict
    snapshot built on the event loop -- never a `data_func` closure, and never
    anything that would touch `hass` or a live `ZoneSet` from the worker.
    """

    def __init__(self, hass: Any, entry_id: str) -> None:
        """Create (but do not yet load) the store for one config entry."""
        # Imported lazily so `zones.py` can be imported outside Home Assistant.
        from homeassistant.helpers.storage import Store  # noqa: PLC0415

        self._store: Any = Store(
            hass, STORAGE_VERSION, STORAGE_KEY_TEMPLATE.format(entry_id=entry_id)
        )
        self.data = ZoneSet(zones=[])

    async def async_load(self) -> ZoneSet:
        """Load the zone set, degrading to an empty one on unusable data.

        A corrupt or future-versioned file must not break integration setup --
        the robot still vacuums. The old file is left on disk untouched so the
        user can fix it by hand, and the log says exactly what was wrong.
        """
        raw = await self._store.async_load()
        try:
            self.data = ZoneSet.from_dict(raw)
        except ZoneError as err:
            _LOGGER.error(
                "bObsweep: stored zone data is unusable (%s); continuing with no "
                "zones. The file has been left in place -- fix it and reload, or "
                "re-import from a known-good export",
                err,
            )
            self.data = ZoneSet(zones=[])
        return self.data

    async def async_save(self) -> None:
        """Persist the current zone set."""
        self.data.extent = zones_extent(self.data.zones)
        await self._store.async_save(self.data.to_dict())

    async def async_remove(self) -> None:
        """Delete the backing file (used when the config entry is removed)."""
        await self._store.async_remove()


# --- vacuum segment API helpers ---------------------------------------------
# Home Assistant 2026.3 added a first-class vacuum segment/area system
# (`VacuumEntityFeature.CLEAN_AREA`, `Segment(id, name, group)`,
# `async_get_segments()`, `async_clean_segments()`), with the segment -> HA area
# mapping owned by the user in entity-registry options and a
# `HassVacuumCleanArea` voice intent on top. Taught zones are exactly the right
# thing to back it with.
#
# These two helpers are plain data in / plain data out so they can be exercised
# with no Home Assistant installed; `vacuum.py` wraps the first one in whatever
# `Segment` class the running HA provides, if it provides one at all.


def segment_specs(zone_set: ZoneSet) -> list[dict[str, Any]]:
    """Describe every zone as a vacuum segment: `{id, name, group}`.

    The segment id is the zone's stable uuid, never its name: the user maps
    segments onto HA areas in the entity registry, and that mapping is keyed by
    segment id, so a rename must not orphan it.

    `group` is the map id, stringified, when one is known. That is how a
    multi-floor home is namespaced -- "Bedroom" upstairs and "Bedroom"
    downstairs are different segments in different groups. It is None today
    because nothing has yet been verified to report a map id.
    """
    group = None if zone_set.map_id is None else str(zone_set.map_id)
    return [
        {"id": zone.id, "name": zone.name, "group": group}
        for zone in sorted(zone_set.zones, key=lambda z: (z.name.casefold(), z.id))
    ]


def resolve_segment_boxes(
    zone_set: ZoneSet, segment_ids: Iterable[Any]
) -> list[tuple[Zone, BBox]]:
    """Resolve segment ids to their zones and bounding boxes, in call order.

    Bounding boxes because the robot's own zone-clean command takes rectangles,
    not polygons -- a segment clean of an L-shaped room will over-cover into the
    notch. Raises `ZoneError` naming every id that does not resolve, so a caller
    can fail the whole request rather than silently cleaning a subset.
    """
    resolved: list[tuple[Zone, BBox]] = []
    missing: list[str] = []
    for raw_id in segment_ids:
        zone = zone_set.by_id(str(raw_id))
        if zone is None:
            missing.append(str(raw_id))
            continue
        box = zone.bbox
        if box is None:
            missing.append(str(raw_id))
            continue
        resolved.append((zone, box))
    if missing:
        raise ZoneError(
            "unknown segment id(s): " + ", ".join(missing)
        )
    return resolved
