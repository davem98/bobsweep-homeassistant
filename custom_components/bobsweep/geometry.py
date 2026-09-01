"""Pure geometry helpers for bObsweep room awareness.

Everything in this module is deliberately dependency-free: no Home Assistant
imports, no `shapely`, no numpy, nothing that needs a compiler. A HACS custom
integration cannot pull a native wheel onto every supported architecture, and
the shapes involved here are a handful of small polygons evaluated a few times a
minute -- a pure-Python ray cast is far below the noise floor.

**Units.** Every coordinate is a raw *map cell* in the robot's own frame, as
carried on DP 105 (`(86,-136) (416,-136) (416,-666) (86,-666)` and friends:
signed int16, origin-relative, y-axis inverted). Nothing here converts to
metres, rotates, or otherwise transforms -- a zone taught from robot positions
and a zone decoded from a DP 105 frame are directly comparable, and the moment
this module started applying a transform they would not be.

Three things live here:

* `point_in_polygon` -- ray casting, with an explicit on-edge test first so a
  point exactly on a boundary is deterministic rather than falling wherever the
  floating-point crossing count lands.
* `polygon_area` / `bounding_box` -- used by the overlap rule (smallest zone
  wins, so a "Pantry" polygon nested inside "Kitchen" resolves to the pantry)
  and by the bounding-box fast path.
* `density_footprint` -- turns a cloud of sampled robot positions into a room
  footprint by coarse density binning rather than a convex hull. See its
  docstring for why the hull is the wrong tool.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass
from typing import Iterable, Sequence

# Coordinates arrive as integer map cells, so an exact zero cross-product is the
# honest on-edge test. The epsilon only exists so a caller that passes in floats
# (an interpolated position, a scaled DP 104 point) still behaves sanely.
_EDGE_EPS = 1e-9

Point = tuple[float, float]
BBox = tuple[float, float, float, float]


def bounding_box(points: Sequence[Point]) -> BBox | None:
    """Return `(minx, miny, maxx, maxy)` for a point sequence, or None if empty."""
    if not points:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), min(ys), max(xs), max(ys))


def point_in_bbox(x: float, y: float, box: BBox) -> bool:
    """Cheap inclusive bounding-box test -- the fast path before ray casting."""
    minx, miny, maxx, maxy = box
    return minx <= x <= maxx and miny <= y <= maxy


def polygon_area(points: Sequence[Point]) -> float:
    """Absolute shoelace area of a polygon, in square map cells.

    Winding direction is irrelevant here (the result is absolute) -- the value is
    only ever used to rank overlapping zones by size.
    """
    n = len(points)
    if n < 3:
        return 0.0
    total = 0.0
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        total += x1 * y2 - x2 * y1
    return abs(total) / 2.0


def point_on_segment(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> bool:
    """True when `(px, py)` lies on the closed segment `(ax,ay)-(bx,by)`."""
    cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
    if abs(cross) > _EDGE_EPS:
        return False
    return (
        min(ax, bx) - _EDGE_EPS <= px <= max(ax, bx) + _EDGE_EPS
        and min(ay, by) - _EDGE_EPS <= py <= max(ay, by) + _EDGE_EPS
    )


def point_in_polygon(x: float, y: float, points: Sequence[Point]) -> bool:
    """Ray-casting point-in-polygon test; points exactly on an edge count as in.

    The polygon is treated as closed (the last vertex joins the first), so a
    rectangle is simply its four corners. Fewer than three vertices is not a
    polygon and is never "inside".

    On-edge is resolved *before* the ray cast, deliberately. The horizontal-ray
    algorithm is only well-defined for points strictly inside or outside; a
    point sitting on a boundary lands on whichever side the crossing parity
    happens to fall, and two zones sharing a wall would then both claim it (or
    neither would). Declaring the closed polygon -- boundary included -- as
    "inside" makes that case deterministic. Zones that share a wall therefore
    both contain the shared edge; the smallest-area tie-break in `zones.py` is
    what picks a single winner.
    """
    n = len(points)
    if n < 3:
        return False

    for i in range(n):
        ax, ay = points[i]
        bx, by = points[(i + 1) % n]
        if point_on_segment(x, y, ax, ay, bx, by):
            return True

    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = points[i]
        xj, yj = points[j]
        # Half-open comparison on y: a vertex is counted for the edge below it
        # only, which is what keeps a ray passing exactly through a vertex from
        # being counted twice.
        if (yi > y) != (yj > y):
            x_cross = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < x_cross:
                inside = not inside
        j = i
    return inside


def rect_polygon(box: BBox) -> list[Point]:
    """Return a bounding box as a 4-vertex polygon.

    Rectangles are not a separate zone type anywhere in this integration -- they
    are just polygons with four vertices -- so anything that produces a box
    (`density_footprint`, the segment resolution path) converts here.
    """
    minx, miny, maxx, maxy = box
    return [(minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy)]


@dataclass(frozen=True)
class Footprint:
    """The result of deriving a room footprint from sampled positions."""

    polygon: list[Point]
    # Grid cells that survived the density threshold, as (col, row) indices.
    kept_cells: tuple[tuple[int, int], ...]
    total_cells: int
    total_points: int
    threshold: int
    # Size of one binning cell in map cells, for diagnostics.
    cell_size: tuple[float, float]


def density_footprint(
    points: Sequence[Point],
    *,
    grid: int = 10,
    min_points: int = 3,
    keep_fraction: float = 0.25,
) -> Footprint | None:
    """Derive a room footprint from a cloud of sampled robot positions.

    **Why not a convex hull.** A capture session starts at the dock and ends at
    the dock, so the point cloud is always "thin line across the house, dense
    blob in the room, thin line back". A convex hull is defined by its extreme
    points, so those two transit lines drag the hull across every hallway and
    room between the dock and the target -- the derived "Bedroom" would swallow
    the living room. Density binning throws exactly that away: a single-pass
    transit deposits one or two samples per bin, while covering a room deposits
    many, so a per-bin count threshold separates them cleanly with no notion of
    "route" needed.

    The point cloud is binned onto a `grid` x `grid` lattice spanning its own
    extent. A bin is kept when its count reaches `threshold`, defined as
    `max(min_points, ceil(keep_fraction * busiest_bin))` -- absolute floor so a
    tiny sample set cannot keep everything, relative component so the threshold
    scales with how long the user drove around. GUESS: `keep_fraction=0.25` and
    `min_points=3` are picked to be obviously-safe defaults rather than tuned
    against real capture data, which does not exist yet; both are parameters.

    The returned polygon is the bounding rectangle of the samples that fell in
    kept bins. That is a deliberate simplification: an L-shaped room comes back
    as its bounding box, slightly over-claiming into the notch. The kept
    bins are returned alongside so a caller (or a future refinement) can do
    better -- and the user can always hand-edit via `export_zones` /
    `import_zones`. Over-claiming into a corner is a much cheaper failure than a
    hull spike across the house.

    Returns None when there is nothing defensible to derive from.
    """
    if grid < 1:
        raise ValueError("grid must be >= 1")
    if not points:
        return None

    box = bounding_box(points)
    if box is None:
        return None
    minx, miny, maxx, maxy = box

    # A capture that never moved (or moved less than a cell) still has a valid
    # answer: the single cell it sat in. Force a non-zero span so the binning
    # arithmetic stays finite.
    span_x = max(maxx - minx, 1.0)
    span_y = max(maxy - miny, 1.0)
    cell_w = span_x / grid
    cell_h = span_y / grid

    def _cell(point: Point) -> tuple[int, int]:
        """Which bin a sample falls in (the last row/column is closed)."""
        col = min(int((point[0] - minx) / cell_w), grid - 1)
        row = min(int((point[1] - miny) / cell_h), grid - 1)
        return (col, row)

    counts: Counter[tuple[int, int]] = Counter()
    for point in points:
        counts[_cell(point)] += 1

    if not counts:
        return None

    busiest = max(counts.values())
    threshold = max(min_points, math.ceil(keep_fraction * busiest))
    kept = sorted(cell for cell, n in counts.items() if n >= threshold)
    if not kept:
        return None

    # Bound the footprint by the *samples in the kept bins*, not by the bin
    # edges. A bin is a coarse container -- its edge can sit most of a cell away
    # from the nearest sample it holds, which on a 10x10 grid over a whole floor
    # is tens of map cells of floor the robot was never on. Re-measuring from
    # the surviving samples snaps the footprint back onto real observations,
    # which is the entire point of preferring density to a hull.
    kept_set = set(kept)
    inliers = [point for point in points if _cell(point) in kept_set]
    box = bounding_box(inliers)
    if box is None:
        return None

    polygon = [
        (float(round(px)), float(round(py))) for px, py in rect_polygon(box)
    ]

    return Footprint(
        polygon=polygon,
        kept_cells=tuple(kept),
        total_cells=len(counts),
        total_points=len(points),
        threshold=threshold,
        cell_size=(cell_w, cell_h),
    )


def median_point(points: Iterable[Point]) -> Point | None:
    """Component-wise median of a set of points.

    Component-wise rather than a true geometric median: it is O(n log n), needs
    no iteration, and for the only caller (a cluster of dock observations that
    should all be the same cell) the difference is nil. A median rather than a
    mean because one bad relocalisation sample would drag a mean permanently.
    """
    pts = list(points)
    if not pts:
        return None
    xs = sorted(p[0] for p in pts)
    ys = sorted(p[1] for p in pts)
    mid = len(pts) // 2
    if len(pts) % 2:
        return (xs[mid], ys[mid])
    return ((xs[mid - 1] + xs[mid]) / 2.0, (ys[mid - 1] + ys[mid]) / 2.0)


def distance(a: Point, b: Point) -> float:
    """Euclidean distance between two points, in map cells."""
    return math.hypot(a[0] - b[0], a[1] - b[1])
