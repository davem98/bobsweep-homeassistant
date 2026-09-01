"""Zone capture / management services for the bObsweep integration.

Six services, registered as *entity* services on the vacuum platform (matching
the existing `set_mode` / `empty_dustbin` / `set_dp` precedent), so each call
targets a specific robot:

* `bobsweep.capture_point` -- record one fix into a scratch buffer. The
  primitive and the escape hatch: it works when nothing else does, and its
  output is hand-assemblable into a polygon for `import_zones`.
* `bobsweep.start_zone_capture` / `bobsweep.stop_zone_capture` -- sample
  position while the user drives the robot around a room, then derive a
  footprint by density binning (see `geometry.density_footprint`).
* `bobsweep.delete_zone` -- remove one by name.
* `bobsweep.export_zones` / `bobsweep.import_zones` -- the backup and
  hand-editing pair. `Store` gives persistence but no way to see, diff, back up
  or fix the data; these do. Export output is exactly what import accepts.

**Everything that needs a live position fails loudly when there is none**, which
is the expected case today (see `position.py`): `ServiceValidationError` naming
the specific reason, not a silent no-op that records an empty zone. The three
services that only touch stored data -- delete, export, import -- deliberately
keep working without a position source, because losing your backup path when
the robot is offline would be perverse.
"""

from __future__ import annotations

import json
import logging
from datetime import datetime, timezone
from typing import Any

import voluptuous as vol
from homeassistant.core import ServiceResponse, SupportsResponse
from homeassistant.exceptions import ServiceValidationError
from homeassistant.helpers import config_validation as cv

from .rooms import ActiveCapture, CapturePoint, RoomTracker, derive_zone
from .zones import ZoneError, ZoneSet, zones_extent

_LOGGER = logging.getLogger(__name__)


def _utcnow() -> datetime:
    """Timezone-aware now."""
    return datetime.now(timezone.utc)


SERVICE_CAPTURE_POINT = "capture_point"
SERVICE_START_ZONE_CAPTURE = "start_zone_capture"
SERVICE_STOP_ZONE_CAPTURE = "stop_zone_capture"
SERVICE_DELETE_ZONE = "delete_zone"
SERVICE_EXPORT_ZONES = "export_zones"
SERVICE_IMPORT_ZONES = "import_zones"


def _tracker(entity: Any) -> RoomTracker:
    """Return the entity's room tracker, or explain that there is not one."""
    tracker = getattr(entity.coordinator, "rooms", None)
    if tracker is None:
        raise ServiceValidationError(
            "Room awareness is not set up for this bObsweep robot."
        )
    return tracker


def _require_position(tracker: RoomTracker) -> None:
    """Refuse a capture service when no position source can ever answer."""
    if not tracker.available:
        raise ServiceValidationError(
            "This bObsweep robot cannot report its position, so zones cannot be "
            f"captured: {tracker.unavailable_reason}. Zones can still be written "
            "by hand with the bobsweep.import_zones service."
        )


async def async_capture_point(entity: Any, call: Any) -> ServiceResponse:
    """Record the robot's current position into the scratch buffer."""
    tracker = _tracker(entity)
    _require_position(tracker)

    label: str = call.data["label"]
    position = await tracker.async_get_position(entity.coordinator.data or {})
    if position is None:
        raise ServiceValidationError(
            f"Could not capture {label!r}: the robot did not report a position. "
            "It may need to be awake or mid-clean for position to be available."
        )

    point = CapturePoint(
        label=label,
        x=position.x,
        y=position.y,
        at=_utcnow().isoformat(timespec="seconds"),
    )
    tracker.scratch.append(point)
    _LOGGER.debug("bObsweep: captured point %s", point)
    return {"point": point.as_dict(), "scratch_size": len(tracker.scratch)}


async def async_start_zone_capture(entity: Any, call: Any) -> ServiceResponse:
    """Begin sampling position into a named capture session."""
    tracker = _tracker(entity)
    _require_position(tracker)

    name: str = call.data["name"].strip()
    if not name:
        raise ServiceValidationError("A zone name is required.")
    if tracker.capture is not None:
        raise ServiceValidationError(
            f"A zone capture for {tracker.capture.name!r} is already running; "
            "call bobsweep.stop_zone_capture first."
        )

    tracker.capture = ActiveCapture(name=name, started=_utcnow())
    # Seed with an immediate fix so a very short capture still has a point.
    position = await tracker.async_get_position(entity.coordinator.data or {})
    if position is not None:
        tracker.capture.points.append(position.point)

    return {"name": name, "points": len(tracker.capture.points)}


async def async_stop_zone_capture(entity: Any, call: Any) -> ServiceResponse:
    """End the capture session and derive a zone from the sampled points."""
    tracker = _tracker(entity)
    capture = tracker.capture
    if capture is None:
        raise ServiceValidationError("No zone capture is running.")
    tracker.capture = None

    zone, footprint = derive_zone(capture.name, capture.points)
    if zone is None:
        raise ServiceValidationError(
            f"Captured {len(capture.points)} position(s) for {capture.name!r} -- "
            "not enough to derive a room footprint. Drive the robot around the "
            "whole room and try again, or build the zone by hand from "
            "bobsweep.capture_point output."
        )

    stored = tracker.zones.add(zone)
    await tracker.zone_store.async_save()
    # A freshly taught zone is measured in the *current* frame, so whatever the
    # old dock reference said is now irrelevant.
    tracker.clear_stale()

    return {
        "zone": stored.to_dict(),
        "sampled_points": len(capture.points),
        "missed_samples": capture.misses,
        "kept_bins": len(footprint.kept_cells),
        "total_bins": footprint.total_cells,
        "bin_threshold": footprint.threshold,
    }


async def async_delete_zone(entity: Any, call: Any) -> ServiceResponse:
    """Delete a zone by name."""
    tracker = _tracker(entity)
    name: str = call.data["name"]
    removed = tracker.zones.remove(name)
    if removed is None:
        known = ", ".join(sorted(tracker.zones.names)) or "(none)"
        raise ServiceValidationError(
            f"No bObsweep zone named {name!r}. Known zones: {known}"
        )
    await tracker.zone_store.async_save()
    return {"deleted": removed.to_dict()}


async def async_export_zones(entity: Any, call: Any) -> ServiceResponse:
    """Return the whole zone document, ready to save or hand-edit."""
    tracker = _tracker(entity)
    return dict(tracker.zones.to_dict())


async def async_import_zones(entity: Any, call: Any) -> ServiceResponse:
    """Replace or merge the zone set from an exported document.

    Accepts either the document (`{"version": 1, "zones": [...]}`), a bare list
    of zones, or a JSON string of either -- a YAML-typed service field and a
    pasted export should both work without the user having to think about it.
    """
    tracker = _tracker(entity)
    raw: Any = call.data["data"]
    replace_all: bool = call.data.get("replace", False)

    if isinstance(raw, str):
        try:
            raw = json.loads(raw)
        except ValueError as err:
            raise ServiceValidationError(
                f"Zone data is not valid JSON: {err}"
            ) from err
    if isinstance(raw, list):
        raw = {"version": 1, "zones": raw}

    try:
        incoming = ZoneSet.from_dict(raw)
    except ZoneError as err:
        raise ServiceValidationError(f"Zone data rejected: {err}") from err

    if replace_all:
        tracker.zone_store.data = incoming
    else:
        target = tracker.zones
        for zone in incoming.zones:
            target.add(zone)
        if incoming.map_id is not None:
            target.map_id = incoming.map_id
        if incoming.dock_ref is not None:
            target.dock_ref = incoming.dock_ref

    tracker.zone_store.data.extent = zones_extent(tracker.zone_store.data.zones)
    await tracker.zone_store.async_save()
    # An imported set defines its own frame; anything considered stale about
    # the previous set no longer applies.
    tracker.clear_stale()

    return {
        "imported": len(incoming.zones),
        "total": len(tracker.zones.zones),
        "replaced": replace_all,
    }


def async_register_zone_services(platform: Any) -> None:
    """Register every zone service on an entity platform."""
    platform.async_register_entity_service(
        SERVICE_CAPTURE_POINT,
        {vol.Required("label"): cv.string},
        async_capture_point,
        supports_response=SupportsResponse.OPTIONAL,
    )
    platform.async_register_entity_service(
        SERVICE_START_ZONE_CAPTURE,
        {vol.Required("name"): cv.string},
        async_start_zone_capture,
        supports_response=SupportsResponse.OPTIONAL,
    )
    platform.async_register_entity_service(
        SERVICE_STOP_ZONE_CAPTURE,
        {},
        async_stop_zone_capture,
        supports_response=SupportsResponse.OPTIONAL,
    )
    platform.async_register_entity_service(
        SERVICE_DELETE_ZONE,
        {vol.Required("name"): cv.string},
        async_delete_zone,
        supports_response=SupportsResponse.OPTIONAL,
    )
    platform.async_register_entity_service(
        SERVICE_EXPORT_ZONES,
        {},
        async_export_zones,
        supports_response=SupportsResponse.ONLY,
    )
    platform.async_register_entity_service(
        SERVICE_IMPORT_ZONES,
        {
            vol.Required("data"): vol.Any(dict, list, cv.string),
            vol.Optional("replace", default=False): cv.boolean,
        },
        async_import_zones,
        supports_response=SupportsResponse.OPTIONAL,
    )
