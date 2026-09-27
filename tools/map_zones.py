#!/usr/bin/env python3
"""Turn a screenshot of the bObsweep app's map into a zone document.

The integration can tell you which room the robot is in, but it needs room
polygons in the robot's own map-cell coordinates, and the robot never sends its
map. What it *does* send is its no-go rectangles, in map cells. The vendor app
draws those same rectangles on top of the room-coloured map. So a screenshot of
the app's Map screen plus the no-go rectangles is enough to recover the
screen-to-cell transform and, with it, every room's outline in cells.

Requirements: Python 3.12+, Pillow, numpy. No Home Assistant, no OpenCV.

    python tools/map_zones.py --screenshot map.png --no-go no_go.json \\
        --names Kitchen=peach,Hallway=pink --out zones.json --overlay overlay.png

How it works, honestly
----------------------
1. **Find the map.** The background colour is the most common colour in the
   image. Full-width uniform bars (the app's header and any toolbar) and
   anything that contains no room colours are ignored; the map is the bounding
   box of what is left. Dark or light theme is read off the background
   luminance unless `--dark-mode` / `--light-mode` says otherwise.
2. **Find the no-go rectangles on screen.** They are drawn as a translucent red
   diagonal hatch. Pixels that are clearly red and are not one of the app's
   flat room colours are grouped into rectangles.
3. **Match them to the robot's rectangles and fit the transform.** Every
   pixel-rectangle / cell-rectangle pairing with a compatible aspect ratio
   proposes a scale and offset; the proposal that the most other rectangles
   agree with wins, then a least-squares fit over all matched corners refines
   it. The transform is a uniform scale plus an axis flip: the app renders the
   map rotated 180 degrees, so map +x is screen-left and map +y is screen-down.
   The tool tries all four axis orientations and reports which one fitted best,
   with a warning if it is not the expected one.
4. **Segment the rooms.** Each pixel is snapped to the nearest known room
   colour. Under the hatched rectangles the app's overlay is un-blended first
   (the overlay colours and opacities were measured from the app's own
   rendering). No-mop (blue hatched) stripes are dropped, the white dust
   speckles, icons and hatch remnants are filled with the surrounding room
   colour, and enclosed fragments are absorbed into whatever surrounds them.
5. **Trace and simplify.** Each room's outer contour is traced and simplified
   (Douglas-Peucker) to between 12 and 40 vertices, mapped to cells and
   rounded to integers.
6. **Write the document.** Version-1 zone document, `source: "taught"`, no ids
   (the integration mints stable ids on import), `dock_ref` set if the dock
   icon (a dark disc with a white centre dot inside a white ring) was found.

What it does not do: the room colours it knows are the ones the app has used
on the maps this was developed against. A colour it has not seen is still
picked up if it covers a meaningful area, and is named by the nearest generic
colour word, but check the overlay. Residuals of a few tens of cells are
normal: the app draws its rectangles with rounded corners and a border, and
the hatch edge is not the geometric edge. Accuracy is limited by screenshot
resolution: at a typical phone resolution one pixel is about three cells.
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import sys
import types
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

try:
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
except ImportError as err:  # pragma: no cover - environment check
    sys.exit(f"error: {err.name} is not installed. Run: pip install pillow numpy")

# ---------------------------------------------------------------------------
# App rendering constants
# ---------------------------------------------------------------------------

#: Flat room colours the vendor app has been seen to use, with the word each
#: zone is named by when `--names` does not rename it. Order is only cosmetic.
APP_PALETTE: dict[str, tuple[int, int, int]] = {
    "Teal": (177, 225, 226),
    "Salmon": (240, 159, 123),
    "Pink": (240, 149, 204),
    "Green": (191, 215, 135),
    "Light blue": (188, 224, 252),
    "Peach": (251, 233, 207),
    "Coral": (238, 122, 142),
    "Pale green": (210, 228, 173),
    "Grey": (173, 173, 173),
    "Orange": (246, 199, 137),
}

#: Extra shades that belong to a palette entry (the app draws a mopping-water
#: texture over some floors, which lightens or darkens the flat colour).
APP_SHADES: dict[str, list[tuple[int, int, int]]] = {
    "Light blue": [
        (175, 214, 251),
        (207, 231, 253),
        (214, 233, 253),
        (192, 232, 252),
        (152, 218, 251),
        (145, 203, 250),
    ],
}

#: Generic colour words for a room colour the palette does not know.
COLOUR_WORDS: dict[str, tuple[int, int, int]] = {
    "Red": (220, 60, 60),
    "Orange": (245, 160, 70),
    "Yellow": (245, 225, 90),
    "Lime": (200, 235, 90),
    "Mint": (150, 230, 190),
    "Cyan": (120, 220, 235),
    "Blue": (90, 140, 240),
    "Navy": (70, 90, 170),
    "Purple": (160, 110, 220),
    "Lavender": (205, 185, 240),
    "Magenta": (230, 90, 200),
    "Brown": (160, 110, 70),
    "Tan": (215, 185, 140),
    "Dark grey": (110, 110, 110),
}

#: The app's hatch overlays, as (opacity, overlay colour). Each hatched stripe
#: is `opacity * overlay + (1 - opacity) * floor`; un-blending recovers the
#: floor colour. Measured from the app's rendering over a known background and
#: a known room colour. The red hatch has two stripe shades; the blue no-mop
#: hatch has one visible stripe (its other stripe is nearly transparent).
RED_HATCH: list[tuple[float, tuple[float, float, float]]] = [
    (0.50, (230.0, 66.0, 58.0)),
    (0.76, (195.0, 104.0, 120.0)),
]
BLUE_HATCH: list[tuple[float, tuple[float, float, float]]] = [
    (0.50, (140.0, 190.0, 250.0)),
]

#: Snap tolerance (RGB distance) for "this pixel is that room colour".
PALETTE_TOL = 20.0
#: The expected orientation: map +x to screen-left, +y to screen-down.
EXPECTED_ORIENTATION = (-1, 1)
#: Vertex budget for the simplified polygons.
MIN_VERTICES, MAX_VERTICES = 12, 40


class ToolError(Exception):
    """A failure the user can act on; printed in plain English, exit 1."""


# ---------------------------------------------------------------------------
# numpy image primitives (what OpenCV/SciPy would otherwise supply)
# ---------------------------------------------------------------------------


def _odd(value: float) -> int:
    """Round to the nearest odd integer >= 3, for kernel sizes."""
    k = int(round(value))
    if k % 2 == 0:
        k += 1
    return max(3, k)


def dilate(mask: np.ndarray, k: int) -> np.ndarray:
    """Binary dilation with a k x k square structuring element."""
    if k <= 1:
        return mask.copy()
    r = k // 2
    h, w = mask.shape
    pad = np.zeros((h, w + 2 * r), bool)
    pad[:, r : r + w] = mask
    out = np.zeros_like(mask)
    for d in range(k):
        out |= pad[:, d : d + w]
    pad = np.zeros((h + 2 * r, w), bool)
    pad[r : r + h] = out
    out = np.zeros_like(mask)
    for d in range(k):
        out |= pad[d : d + h]
    return out


def erode(mask: np.ndarray, k: int) -> np.ndarray:
    """Binary erosion with a k x k square (border counts as background)."""
    return ~dilate(~mask, k)


def close(mask: np.ndarray, k: int) -> np.ndarray:
    """Dilate then erode: bridges gaps narrower than k."""
    return erode(dilate(mask, k), k)


def open_(mask: np.ndarray, k: int) -> np.ndarray:
    """Erode then dilate: removes specks thinner than k."""
    return dilate(erode(mask, k), k)


@dataclass
class Components:
    """Connected-component labelling result (8-connected)."""

    labels: np.ndarray  # int32, 0 = background, 1..n
    count: int
    area: np.ndarray  # index by label; area[0] is the background
    bbox: np.ndarray  # (n+1, 4) as x0, y0, x1, y1 (exclusive)


def label_components(mask: np.ndarray) -> Components:
    """Label 8-connected components using row runs and union-find."""
    h, w = mask.shape
    padded = np.zeros((h, w + 1), np.int8)
    padded[:, :w] = mask
    flat = padded.ravel()
    diff = np.diff(np.concatenate(([0], flat, [0])).astype(np.int8))
    starts = np.flatnonzero(diff == 1)
    ends = np.flatnonzero(diff == -1)
    n_runs = len(starts)
    labels = np.zeros((h, w), np.int32)
    if n_runs == 0:
        return Components(labels, 0, np.zeros(1, np.int64), np.zeros((1, 4), np.int64))
    rows = starts // (w + 1)
    s = starts % (w + 1)
    e = ends - rows * (w + 1)  # exclusive end column
    parent = np.arange(n_runs)

    def find(i: int) -> int:
        root = i
        while parent[root] != root:
            root = parent[root]
        while parent[i] != root:
            parent[i], i = root, parent[i]
        return root

    row_starts = np.searchsorted(rows, np.arange(h + 1))
    for r in range(1, h):
        a0, a1 = row_starts[r], row_starts[r + 1]
        p0, p1 = row_starts[r - 1], row_starts[r]
        if a0 == a1 or p0 == p1:
            continue
        ps, pe = s[p0:p1], e[p0:p1]
        # 8-connectivity: previous run [ps, pe) touches [si, ei) diagonally too.
        lo = np.searchsorted(pe, s[a0:a1], "left")  # first prev with pe >= si
        hi = np.searchsorted(ps, e[a0:a1], "right")  # first prev with ps > ei
        for i in range(a1 - a0):
            for j in range(lo[i], hi[i]):
                ri, rj = find(a0 + i), find(p0 + j)
                if ri != rj:
                    parent[max(ri, rj)] = min(ri, rj)
    roots = np.array([find(i) for i in range(n_runs)])
    uniq, compact = np.unique(roots, return_inverse=True)
    compact = compact + 1
    for i in range(n_runs):
        labels[rows[i], s[i] : e[i]] = compact[i]
    count = len(uniq)
    lengths = e - s
    area = np.bincount(compact, weights=lengths, minlength=count + 1).astype(np.int64)
    bbox = np.zeros((count + 1, 4), np.int64)
    bbox[:, 0] = w
    bbox[:, 1] = h
    np.minimum.at(bbox[:, 0], compact, s)
    np.minimum.at(bbox[:, 1], compact, rows)
    np.maximum.at(bbox[:, 2], compact, e)
    np.maximum.at(bbox[:, 3], compact, rows + 1)
    return Components(labels, count, area, bbox)


def fill_holes(mask: np.ndarray) -> np.ndarray:
    """Fill background regions that do not touch the image border."""
    comps = label_components(~mask)
    if comps.count == 0:
        return mask.copy()
    border = np.zeros(comps.count + 1, bool)
    lab = comps.labels
    for edge in (lab[0], lab[-1], lab[:, 0], lab[:, -1]):
        border[np.unique(edge)] = True
    border[0] = True
    return mask | ~border[lab]


def box_count(mask: np.ndarray, k: int) -> np.ndarray:
    """Number of true pixels in the k x k box around each pixel."""
    r = k // 2
    h, w = mask.shape
    integ = np.zeros((h + 2 * r + 2, w + 2 * r + 2), np.int32)
    integ[r + 1 : r + 1 + h, r + 1 : r + 1 + w] = mask
    integ = integ.cumsum(0).cumsum(1)
    return (
        integ[k : k + h, k : k + w]
        - integ[0:h, k : k + w]
        - integ[k : k + h, 0:w]
        + integ[0:h, 0:w]
    )


def mode_filter(labels: np.ndarray, k: int, n_labels: int) -> np.ndarray:
    """Replace each labelled pixel by the most common non-zero label around it."""
    best = np.zeros(labels.shape, np.int32)
    best_n = np.zeros(labels.shape, np.int32)
    for lab in range(1, n_labels + 1):
        cnt = box_count(labels == lab, k)
        better = cnt > best_n
        best[better] = lab
        best_n[better] = cnt[better]
    return best


def propagate_labels(labels: np.ndarray, allowed: np.ndarray, steps: int) -> np.ndarray:
    """Grow labels into unlabelled `allowed` pixels, one pixel per step."""
    out = labels.copy()
    shifts = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    for _ in range(steps):
        unknown = (out == 0) & allowed
        if not unknown.any():
            break
        for dy, dx in shifts:
            src = np.roll(out, (dy, dx), axis=(0, 1))
            take = unknown & (src > 0)
            out[take] = src[take]
            unknown &= out == 0
    return out


def trace_boundary(mask: np.ndarray) -> list[tuple[int, int]]:
    """Outer boundary of an 8-connected blob (Moore neighbour tracing)."""
    pad = np.zeros((mask.shape[0] + 2, mask.shape[1] + 2), bool)
    pad[1:-1, 1:-1] = mask
    ys, xs = np.nonzero(pad)
    if len(ys) == 0:
        return []
    start = (int(ys[0]), int(xs[0]))
    # Clockwise from east, as (dy, dx).
    nbr = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]
    contour = [start]
    cur = start
    back = 4  # the start pixel is topmost-leftmost, so its west neighbour is background
    second: tuple[int, int] | None = None
    limit = 8 * int(mask.sum()) + 16
    for _ in range(limit):
        found = None
        for i in range(8):
            d = (back + i) % 8
            y, x = cur[0] + nbr[d][0], cur[1] + nbr[d][1]
            if pad[y, x]:
                found = (y, x)
                # Next scan starts at the last background pixel examined,
                # expressed relative to the pixel just entered.
                back = (d - (d % 2) + 6) % 8
                break
        if found is None:
            break  # a lone pixel
        if second is None:
            second = found
        elif cur == start and found == second:
            break  # back at the start, leaving it the same way as the first time
        cur = found
        contour.append(cur)
    if len(contour) > 1 and contour[-1] == start:
        contour.pop()
    return [(x - 1, y - 1) for y, x in contour]


def douglas_peucker(points: np.ndarray, eps: float) -> np.ndarray:
    """Simplify a closed polygon; returns the kept vertices in order."""
    n = len(points)
    if n < 4:
        return points
    d0 = ((points - points[0]) ** 2).sum(1)
    far = int(d0.argmax())
    keep = np.zeros(n, bool)
    keep[0] = keep[far] = True
    stack = [(0, far), (far, n - 1)]
    keep[n - 1] = True if far != n - 1 else keep[n - 1]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        pa, pb = points[a], points[b]
        seg = pb - pa
        length = math.hypot(*seg)
        mid = points[a + 1 : b]
        if length < 1e-9:
            dist = np.hypot(*(mid - pa).T)
        else:
            dist = np.abs(seg[0] * (mid[:, 1] - pa[1]) - seg[1] * (mid[:, 0] - pa[0])) / length
        i = int(dist.argmax())
        if dist[i] > eps:
            keep[a + 1 + i] = True
            stack.append((a, a + 1 + i))
            stack.append((a + 1 + i, b))
    return points[keep]


def simplify_polygon(points: np.ndarray, eps0: float) -> np.ndarray:
    """Douglas-Peucker with the tolerance tuned into the vertex budget."""
    eps = eps0
    poly = douglas_peucker(points, eps)
    guard = 0
    while len(poly) > MAX_VERTICES and guard < 40:
        eps *= 1.25
        poly = douglas_peucker(points, eps)
        guard += 1
    while len(poly) < MIN_VERTICES and eps > 0.3 and guard < 80:
        eps /= 1.5
        poly = douglas_peucker(points, eps)
        guard += 1
    return poly


def polygon_area(points: list[tuple[float, float]]) -> float:
    """Shoelace area."""
    total = 0.0
    for i, (x0, y0) in enumerate(points):
        x1, y1 = points[(i + 1) % len(points)]
        total += x0 * y1 - x1 * y0
    return abs(total) / 2.0


# ---------------------------------------------------------------------------
# Colour helpers
# ---------------------------------------------------------------------------


def nearest_colour(pixels: np.ndarray, colours: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Index of and distance to the nearest colour for each pixel (N, 3)."""
    best_i = np.zeros(len(pixels), np.int32)
    best_d = np.full(len(pixels), np.inf)
    for i, c in enumerate(colours):
        d = np.sqrt(((pixels - c) ** 2).sum(1))
        upd = d < best_d
        best_i[upd] = i
        best_d[upd] = d[upd]
    return best_i, best_d


def rgb_distance(image: np.ndarray, colour: tuple[float, float, float]) -> np.ndarray:
    """Per-pixel RGB distance from one colour."""
    return np.sqrt(((image - np.array(colour, float)) ** 2).sum(-1))


def luminance(colour: tuple[float, float, float]) -> float:
    """Rec.601 luma."""
    return 0.299 * colour[0] + 0.587 * colour[1] + 0.114 * colour[2]


def colour_word(colour: tuple[float, float, float], taken: set[str]) -> str:
    """Nearest generic colour word for a colour not in the palette."""
    table = {**APP_PALETTE, **COLOUR_WORDS}
    order = sorted(table, key=lambda n: sum((a - b) ** 2 for a, b in zip(table[n], colour)))
    for name in order:
        if name not in taken:
            return name
    return f"Colour {len(taken) + 1}"


# ---------------------------------------------------------------------------
# Stage 1: background, chrome and map area
# ---------------------------------------------------------------------------


@dataclass
class Frame:
    """Everything about the screenshot's layout the later stages need."""

    image: np.ndarray  # float RGB, full size
    background: tuple[float, float, float]
    dark: bool
    crop: tuple[int, int, int, int]  # x0, y0, x1, y1 of the map area
    unit: float  # 1.0 at 1284 px wide; kernel sizes scale with it

    @property
    def map(self) -> np.ndarray:
        x0, y0, x1, y1 = self.crop
        return self.image[y0:y1, x0:x1]


def detect_background(image: np.ndarray) -> tuple[float, float, float]:
    """The most common colour, refined to the mean of the pixels near it."""
    q = (image[::4, ::4] // 8).astype(np.int32).reshape(-1, 3)
    keys = q[:, 0] * 1024 + q[:, 1] * 32 + q[:, 2]
    vals, counts = np.unique(keys, return_counts=True)
    top = vals[counts.argmax()]
    guess = np.array([top // 1024, (top // 32) % 32, top % 32], float) * 8 + 4
    sample = image[::4, ::4].reshape(-1, 3)
    near = sample[rgb_distance(sample, tuple(guess)) < 16]
    mean = near.mean(0) if len(near) else guess
    return (float(mean[0]), float(mean[1]), float(mean[2]))


def is_red_hatch(image: np.ndarray, palette: np.ndarray) -> np.ndarray:
    """Pixels that read as the red no-go hatch and not as a flat room colour."""
    r, g, b = image[..., 0], image[..., 1], image[..., 2]
    red = (r > g + 40) & (r > b + 30)
    if not red.any():
        return red
    flat = image[red]
    _, dist = nearest_colour(flat, palette)
    red[red] = dist > 25
    return red


def detect_frame(image: np.ndarray, theme: str | None) -> Frame:
    """Find the background, the theme, and the map's bounding box."""
    h, w, _ = image.shape
    unit = w / 1284.0
    background = detect_background(image)
    dark = luminance(background) < 128 if theme is None else theme == "dark"
    bg_mask = rgb_distance(image, background) < 14

    # Full-width uniform bars: rows where nearly every pixel is one non-background colour.
    row_q = (image[:, ::2] // 6).astype(np.int32)
    bar_rows = np.zeros(h, bool)
    for y in range(h):
        row = row_q[y]
        keys = row[:, 0] * 4096 + row[:, 1] * 64 + row[:, 2]
        vals, counts = np.unique(keys, return_counts=True)
        if counts.max() >= 0.9 * len(keys):
            top = vals[counts.argmax()]
            colour = (top // 4096 * 6 + 3, (top // 64) % 64 * 6 + 3, top % 64 * 6 + 3)
            if math.dist(colour, background) > 14:
                bar_rows[y] = True

    palette = np.array(list(APP_PALETTE.values()), float)
    shades = [c for lst in APP_SHADES.values() for c in lst]
    palette_all = np.vstack([palette, np.array(shades, float)]) if shades else palette
    stride = 2
    small = image[::stride, ::stride]
    _, dist = nearest_colour(small.reshape(-1, 3), palette_all)
    roomish = (dist.reshape(small.shape[:2]) < PALETTE_TOL) | is_red_hatch(small, palette)
    roomish_full = np.zeros((h, w), bool)
    roomish_full[::stride, ::stride] = roomish
    roomish_full &= ~bar_rows[:, None]

    content = ~bg_mask & ~bar_rows[:, None]
    content = close(content, _odd(13 * unit))
    comps = label_components(content)
    if comps.count == 0:
        raise ToolError("the screenshot has no content other than a flat background")
    score = np.bincount(comps.labels.ravel(), weights=roomish_full.ravel(), minlength=comps.count + 1)
    score[0] = 0
    total = score.sum()
    if total == 0:
        raise ToolError(
            "no room colours or red no-go hatch found in the screenshot; is this the app's Map screen?"
        )
    keep = []
    for lab in range(1, comps.count + 1):
        x0, y0, x1, y1 = comps.bbox[lab]
        solid_bar = (x1 - x0) >= 0.95 * w and comps.area[lab] >= 0.9 * (x1 - x0) * (y1 - y0)
        if score[lab] >= 0.01 * total and not solid_bar:
            keep.append(lab)
    boxes = comps.bbox[keep]
    margin = int(round(12 * unit))
    crop = (
        max(0, int(boxes[:, 0].min()) - margin),
        max(0, int(boxes[:, 1].min()) - margin),
        min(w, int(boxes[:, 2].max()) + margin),
        min(h, int(boxes[:, 3].max()) + margin),
    )
    return Frame(image=image, background=background, dark=dark, crop=crop, unit=unit)


# ---------------------------------------------------------------------------
# Stage 2: no-go rectangles on screen
# ---------------------------------------------------------------------------


@dataclass
class PixelRect:
    """An axis-aligned rectangle in full-image pixel coordinates."""

    x0: int
    y0: int
    x1: int
    y1: int

    @property
    def w(self) -> int:
        return self.x1 - self.x0

    @property
    def h(self) -> int:
        return self.y1 - self.y0

    @property
    def centre(self) -> tuple[float, float]:
        return ((self.x0 + self.x1) / 2.0, (self.y0 + self.y1) / 2.0)


def detect_red_rects(frame: Frame) -> list[PixelRect]:
    """Group red-hatch pixels into rectangles."""
    x_off, y_off, _, _ = frame.crop
    image = frame.map
    palette = np.array(list(APP_PALETTE.values()), float)
    red = is_red_hatch(image, palette)
    if not red.any():
        return []
    merged = close(red, _odd(9 * frame.unit))
    comps = label_components(merged)
    min_side = 19 * frame.unit
    rects: list[PixelRect] = []
    for lab in range(1, comps.count + 1):
        x0, y0, x1, y1 = (int(v) for v in comps.bbox[lab])
        if (x1 - x0) < min_side or (y1 - y0) < min_side:
            continue
        sub = red[y0:y1, x0:x1] & (comps.labels[y0:y1, x0:x1] == lab)
        if sub.mean() < 0.45:
            continue
        # A hatch has two stripe shades; a flat blob of some unknown reddish
        # room colour has one, and is not a no-go rectangle.
        q = (image[y0:y1, x0:x1][sub] // 12).astype(np.int32)
        _, counts = np.unique(q[:, 0] * 4096 + q[:, 1] * 64 + q[:, 2], return_counts=True)
        counts = np.sort(counts)[::-1]
        if len(counts) < 2 or counts[1] < 0.12 * counts.sum():
            continue
        # Tighten to rows/columns that are mostly red; drops the soft border.
        rows = np.flatnonzero(sub.mean(1) > 0.4)
        cols = np.flatnonzero(sub.mean(0) > 0.4)
        if len(rows) < min_side or len(cols) < min_side:
            continue
        rects.append(
            PixelRect(
                x_off + x0 + int(cols[0]),
                y_off + y0 + int(rows[0]),
                x_off + x0 + int(cols[-1]) + 1,
                y_off + y0 + int(rows[-1]) + 1,
            )
        )
    rects.sort(key=lambda r: -(r.w * r.h))
    return rects


# ---------------------------------------------------------------------------
# Stage 3: the robot's rectangles, matching and the fit
# ---------------------------------------------------------------------------


@dataclass
class CellRect:
    """An axis-aligned rectangle in map cells."""

    x0: int
    y0: int
    x1: int
    y1: int

    @property
    def w(self) -> int:
        return self.x1 - self.x0

    @property
    def h(self) -> int:
        return self.y1 - self.y0

    @property
    def centre(self) -> tuple[float, float]:
        return ((self.x0 + self.x1) / 2.0, (self.y0 + self.y1) / 2.0)


def _as_rect(item: object) -> CellRect | None:
    """One rectangle from any of the shapes the integration or a user may give."""
    if isinstance(item, dict):
        for key in ("points", "corners", "rect", "rectangle"):
            if key in item:
                return _as_rect(item[key])
        if all(k in item for k in ("x0", "y0", "x1", "y1")):
            return CellRect(*(int(item[k]) for k in ("x0", "y0", "x1", "y1")))
        return None
    if not isinstance(item, (list, tuple)):
        return None
    if len(item) == 8 and all(isinstance(v, (int, float)) for v in item):
        pts = [(item[i], item[i + 1]) for i in range(0, 8, 2)]
    elif len(item) == 4 and all(isinstance(v, (int, float)) for v in item):
        # Already a box: minx, miny, maxx, maxy.
        pts = [(item[0], item[1]), (item[2], item[3])]
    else:
        pts = []
        for p in item:
            if isinstance(p, dict) and "x" in p and "y" in p:
                pts.append((p["x"], p["y"]))
            elif isinstance(p, (list, tuple)) and len(p) >= 2:
                pts.append((p[0], p[1]))
            else:
                return None
    if len(pts) < 2:
        return None
    xs = [float(p[0]) for p in pts]
    ys = [float(p[1]) for p in pts]
    rect = CellRect(int(round(min(xs))), int(round(min(ys))), int(round(max(xs))), int(round(max(ys))))
    if rect.w <= 0 or rect.h <= 0:
        return None
    return rect


def load_no_go(arg: str) -> list[CellRect]:
    """Parse `--no-go`: a file path or inline JSON, in any of the accepted shapes."""
    text = arg.strip()
    if not (text.startswith("[") or text.startswith("{")):
        path = Path(arg)
        if not path.is_file():
            raise ToolError(f"--no-go: {arg!r} is neither a file nor JSON text")
        text = path.read_text(encoding="utf-8")
    try:
        data = json.loads(text)
    except json.JSONDecodeError as err:
        raise ToolError(f"--no-go: not valid JSON ({err})") from err
    if isinstance(data, dict):
        for key in ("no_go_zones", "no_go", "rectangles", "zones", "attributes"):
            if key in data:
                data = data[key]
                if isinstance(data, dict) and "no_go_zones" in data:
                    data = data["no_go_zones"]
                break
    if not isinstance(data, list):
        raise ToolError("--no-go: expected a JSON list of rectangles (each four [x, y] corners)")
    rects = []
    for i, item in enumerate(data):
        rect = _as_rect(item)
        if rect is None:
            raise ToolError(f"--no-go: rectangle {i + 1} is not four [x, y] corners: {item!r}")
        rects.append(rect)
    if not rects:
        raise ToolError("--no-go: the list is empty; the robot has no no-go zones to calibrate against")
    return rects


@dataclass
class Transform:
    """cell = offset + k * (sx * px, sy * py)."""

    k: float
    ox: float
    oy: float
    sx: int
    sy: int

    def to_cell(self, px: float, py: float) -> tuple[float, float]:
        return (self.ox + self.sx * self.k * px, self.oy + self.sy * self.k * py)

    def to_px(self, cx: float, cy: float) -> tuple[float, float]:
        return ((cx - self.ox) / (self.sx * self.k), (cy - self.oy) / (self.sy * self.k))

    def describe(self) -> str:
        xs = "screen-left" if self.sx < 0 else "screen-right"
        ys = "screen-down" if self.sy > 0 else "screen-up"
        return f"map +x = {xs}, +y = {ys}"


@dataclass
class Fit:
    """A fitted transform plus its evidence."""

    transform: Transform
    pairs: list[tuple[int, int]]  # (pixel rect index, cell rect index)
    residuals: list[tuple[float, float, float, float]]  # left, right, top, bottom in cells
    rms: float


def _corner_pairs(pr: PixelRect, cr: CellRect, sx: int, sy: int) -> list[tuple[float, float, float, float]]:
    """Corner correspondences (px, py, cx, cy) honouring the axis orientation."""
    xs = (cr.x0, cr.x1) if sx > 0 else (cr.x1, cr.x0)
    ys = (cr.y0, cr.y1) if sy > 0 else (cr.y1, cr.y0)
    out = []
    for px, cx in ((pr.x0, xs[0]), (pr.x1, xs[1])):
        for py, cy in ((pr.y0, ys[0]), (pr.y1, ys[1])):
            out.append((float(px), float(py), float(cx), float(cy)))
    return out


def _lstsq(pairs: list[tuple[int, int]], px_rects: list[PixelRect], cell_rects: list[CellRect], sx: int, sy: int) -> Transform:
    """Least-squares k, ox, oy for a fixed orientation over matched corners."""
    rows = []
    rhs = []
    for i, j in pairs:
        for px, py, cx, cy in _corner_pairs(px_rects[i], cell_rects[j], sx, sy):
            rows.append((sx * px, 1.0, 0.0))
            rhs.append(cx)
            rows.append((sy * py, 0.0, 1.0))
            rhs.append(cy)
    sol, *_ = np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=None)
    return Transform(float(sol[0]), float(sol[1]), float(sol[2]), sx, sy)


def _residuals(fit_t: Transform, pr: PixelRect, cr: CellRect) -> tuple[float, float, float, float]:
    """Predicted minus actual cell edges: left-x, right-x, top-y, bottom-y."""
    ax0, ay0 = fit_t.to_cell(pr.x0, pr.y0)
    ax1, ay1 = fit_t.to_cell(pr.x1, pr.y1)
    px_x = sorted((ax0, ax1))
    px_y = sorted((ay0, ay1))
    return (px_x[0] - cr.x0, px_x[1] - cr.x1, px_y[0] - cr.y0, px_y[1] - cr.y1)


def fit_transform(px_rects: list[PixelRect], cell_rects: list[CellRect], image_width: int) -> tuple[Fit, list[tuple[Transform, int, float]]]:
    """Match rectangles and fit the transform; returns the best fit and all tried."""
    tried: list[tuple[Transform, int, float]] = []
    best: Fit | None = None
    for sx, sy in ((-1, 1), (1, -1), (1, 1), (-1, -1)):
        hypotheses = []
        for i, pr in enumerate(px_rects):
            for j, cr in enumerate(cell_rects):
                if abs(math.log((pr.w / pr.h) / (cr.w / cr.h))) > 0.45:
                    continue
                k = (cr.w + cr.h) / (pr.w + pr.h)
                pc, cc = pr.centre, cr.centre
                hypotheses.append((k, cc[0] - sx * k * pc[0], cc[1] - sy * k * pc[1]))
        best_pairs: list[tuple[int, int]] = []
        best_cost = math.inf
        for k, ox, oy in hypotheses:
            t = Transform(k, ox, oy, sx, sy)
            tol = 0.05 * k * image_width
            cand = []
            for i, pr in enumerate(px_rects):
                pc = t.to_cell(*pr.centre)
                for j, cr in enumerate(cell_rects):
                    if abs(math.log((pr.w / pr.h) / (cr.w / cr.h))) > 0.45:
                        continue
                    if abs(math.log((cr.w + cr.h) / (k * (pr.w + pr.h)))) > 0.35:
                        continue
                    d = math.dist(pc, cr.centre)
                    if d < tol:
                        cand.append((d, i, j))
            cand.sort()
            used_i: set[int] = set()
            used_j: set[int] = set()
            pairs = []
            cost = 0.0
            for d, i, j in cand:
                if i in used_i or j in used_j:
                    continue
                used_i.add(i)
                used_j.add(j)
                pairs.append((i, j))
                cost += d
            if len(pairs) > len(best_pairs) or (len(pairs) == len(best_pairs) and cost < best_cost):
                best_pairs, best_cost = pairs, cost
        if len(best_pairs) < 2:
            tried.append((Transform(0.0, 0.0, 0.0, sx, sy), len(best_pairs), math.inf))
            continue
        t = _lstsq(best_pairs, px_rects, cell_rects, sx, sy)
        res = [_residuals(t, px_rects[i], cell_rects[j]) for i, j in best_pairs]
        rms = float(np.sqrt(np.mean(np.square(np.array(res)))))
        tried.append((t, len(best_pairs), rms))
        fit = Fit(t, best_pairs, res, rms)
        if best is None or len(fit.pairs) > len(best.pairs) or (len(fit.pairs) == len(best.pairs) and fit.rms < best.rms):
            best = fit
    if best is None:
        raise ToolError(
            f"fewer than 2 of the {len(cell_rects)} no-go rectangles could be matched to the "
            f"{len(px_rects)} red rectangles found on screen; check --dump-detected and that the "
            "screenshot shows the same map the rectangles belong to"
        )
    return best, tried


# ---------------------------------------------------------------------------
# Stage 4: room segmentation
# ---------------------------------------------------------------------------


@dataclass
class Segmentation:
    """Per-pixel room labels for the map crop plus the palette that produced them."""

    labels: np.ndarray  # int32 on the map crop, 0 = not floor
    names: list[str]  # names[label - 1]
    colours: list[tuple[float, float, float]]
    blue_rects: list[PixelRect]
    map_mask: np.ndarray
    unclassified_fraction: float


def detect_frames(frame: Frame, candidates: np.ndarray) -> list[PixelRect]:
    """Rectangular outlines drawn as 50% white over the floor/background."""
    image = frame.map
    x_off, y_off, _, _ = frame.crop
    blend = 0.5 * 255 + 0.5 * candidates
    _, dist = nearest_colour(image.reshape(-1, 3), blend)
    outline = (dist < 9).reshape(image.shape[:2])
    outline = close(outline, _odd(5 * frame.unit))
    comps = label_components(outline)
    min_side = 38 * frame.unit
    out = []
    for lab in range(1, comps.count + 1):
        x0, y0, x1, y1 = (int(v) for v in comps.bbox[lab])
        w, h = x1 - x0, y1 - y0
        if w < min_side or h < min_side:
            continue
        area = comps.area[lab]
        if area > 2 * (w + h) * frame.unit and area < 0.35 * w * h:
            out.append(PixelRect(x_off + x0, y_off + y0, x_off + x1, y_off + y1))
    return out


def segment(frame: Frame, red_rects: list[PixelRect], min_area_px: int) -> Segmentation:
    """Classify every map pixel into a room colour and clean the result up."""
    image = frame.map
    h, w, _ = image.shape
    x_off, y_off, _, _ = frame.crop
    unit = frame.unit
    bg = frame.background

    names = list(APP_PALETTE)
    colours: list[tuple[float, float, float]] = [tuple(map(float, APP_PALETTE[n])) for n in names]
    pal_rows: list[tuple[float, float, float]] = list(colours)
    pal_idx: list[int] = list(range(len(names)))
    for name, shades in APP_SHADES.items():
        for c in shades:
            pal_rows.append(tuple(map(float, c)))
            pal_idx.append(names.index(name))

    def classify(
        pixels: np.ndarray,
        transforms: list[tuple[float, tuple[float, float, float]]],
        tol: float,
        identity: bool = True,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Labels (0 = none) and a white mask, snapping through each transform."""
        # White is a pseudo-entry so that a speckle under the hatch un-blends
        # to "white" rather than to whichever room colour is nearest.
        pal = np.array(pal_rows + [(255.0, 255.0, 255.0)], float)
        idx = np.array(pal_idx + [-1])
        best_d = np.full(len(pixels), np.inf)
        best_l = np.full(len(pixels), -1, np.int32)
        chain = ([(0.0, (0.0, 0.0, 0.0))] if identity else []) + transforms
        for a, hcol in chain:
            floor = (pixels - a * np.array(hcol)) / (1 - a) if a > 0 else pixels
            j, d = nearest_colour(floor, pal)
            upd = d < best_d
            best_d[upd] = d[upd]
            best_l[upd] = idx[j[upd]]
        ok = best_d <= tol
        is_white = ok & (best_l < 0)
        out = np.where(ok & (best_l >= 0), best_l + 1, 0).astype(np.int32)
        return out, is_white

    flat = image.reshape(-1, 3)
    labels, _ = classify(flat, [], PALETTE_TOL)
    labels = labels.reshape(h, w)

    # Unknown flat colours that cover real area become extra rooms.
    red_mask = np.zeros((h, w), bool)
    for r in red_rects:
        red_mask[r.y0 - y_off : r.y1 - y_off, r.x0 - x_off : r.x1 - x_off] = True
    bg_mask = rgb_distance(image, bg) < 14
    white = image.min(-1) > 235
    unknown = (labels == 0) & ~bg_mask & ~white & ~red_mask
    non_bg = int((~bg_mask).sum())
    for _ in range(6):
        pts = image[unknown]
        if len(pts) == 0:
            break
        q = (pts // 8).astype(np.int32)
        keys = q[:, 0] * 1024 + q[:, 1] * 32 + q[:, 2]
        vals, counts = np.unique(keys, return_counts=True)
        top = counts.argmax()
        if counts[top] < 0.004 * non_bg:
            break
        colour = tuple(float(v) for v in pts[keys == vals[top]].mean(0))
        # Hatch stripes and outlines are blends, not rooms: skip anything that
        # un-blends onto a known colour or the background.
        explained = False
        for a, hcol in RED_HATCH + BLUE_HATCH + [(0.5, (255.0, 255.0, 255.0))]:
            floor = tuple((c - a * hc) / (1 - a) for c, hc in zip(colour, hcol))
            for known in colours + [bg]:
                if math.dist(floor, known) < 30:
                    explained = True
        if math.dist(colour, bg) < 30 or luminance(colour) < 60:
            explained = True
        if not explained:
            name = colour_word(colour, set(names))
            names.append(name)
            colours.append(colour)
            pal_rows.append(colour)
            pal_idx.append(len(names) - 1)
        near = rgb_distance(image, colour) < PALETTE_TOL
        if not explained:
            labels[near & (labels == 0)] = len(names)
        unknown &= ~near

    # No-mop rectangles: the outline frames that are not the red rectangles.
    cand = np.vstack([np.array(colours, float), np.array([bg], float)])
    frames = detect_frames(frame, cand)
    blue_rects = []
    for fr in frames:
        overlaps = False
        for rr in red_rects:
            ix = max(0, min(fr.x1, rr.x1) - max(fr.x0, rr.x0))
            iy = max(0, min(fr.y1, rr.y1) - max(fr.y0, rr.y0))
            if ix * iy > 0.6 * min(fr.w * fr.h, rr.w * rr.h) and abs(fr.w - rr.w) < 10 * unit:
                overlaps = True
        if not overlaps:
            blue_rects.append(fr)

    hatch = red_mask.copy()
    for r in blue_rects:
        hatch[r.y0 - y_off : r.y1 - y_off, r.x0 - x_off : r.x1 - x_off] = True

    # Under the hatch, un-blend before snapping to the palette. Inside a red
    # rectangle every pixel is under a stripe, so the plain colour is never the
    # right reading there; a no-mop rectangle's second stripe is nearly
    # transparent, so there the plain colour stays in play.
    white_under_hatch = np.zeros((h, w), bool)
    if red_mask.any():
        lab, wh = classify(image[red_mask], RED_HATCH + BLUE_HATCH, PALETTE_TOL + 5, identity=False)
        labels[red_mask] = lab
        white_under_hatch[red_mask] = wh
    blue_only = hatch & ~red_mask
    if blue_only.any():
        lab, wh = classify(image[blue_only], BLUE_HATCH, PALETTE_TOL + 5)
        labels[blue_only] = lab
        white_under_hatch[blue_only] = wh
    if hatch.any():
        # Blue stripes over anything, and the frames themselves, are not floor.
        stripe = []
        for a, hcol in BLUE_HATCH:
            for c in list(colours) + [bg]:
                stripe.append(tuple(a * hc + (1 - a) * cc for hc, cc in zip(hcol, c)))
        _, dstripe = nearest_colour(image[hatch], np.array(stripe, float))
        sub = labels[hatch]
        sub[dstripe < 6] = 0
        labels[hatch] = sub
    blend = 0.5 * 255 + 0.5 * cand
    _, dout = nearest_colour(flat, blend)
    labels[(dout < 9).reshape(h, w)] = 0

    # Floor mask: labelled pixels plus (dark theme) white speckles and icons.
    map_mask = (labels > 0) | white_under_hatch
    if frame.dark:
        map_mask |= white
    map_mask = close(map_mask, _odd(7 * unit))
    if hatch.any():
        big = close(map_mask, _odd(21 * unit))
        map_mask[hatch] = big[hatch]
    map_mask = fill_holes(map_mask)
    map_mask = open_(map_mask, _odd(5 * unit))

    # Fill unlabelled floor from the nearest label, then smooth label noise.
    n_labels = len(names)
    labels[~map_mask] = 0
    unclassified = float(((labels == 0) & map_mask).sum() / max(1, map_mask.sum()))
    filled = propagate_labels(labels, map_mask, int(60 * unit))
    filled = mode_filter(filled, _odd(9 * unit), n_labels)
    filled[~map_mask] = 0

    # Absorb fragments whose border is dominated by one other room.
    ring_k = _odd(9 * unit)
    for _ in range(6):
        changed = False
        for lab in range(1, n_labels + 1):
            comps = label_components(filled == lab)
            if comps.count <= 1:
                continue
            biggest = comps.area[1:].max()
            for c in range(1, comps.count + 1):
                if comps.area[c] >= 0.5 * biggest:
                    continue
                cm = comps.labels == c
                ring = dilate(cm, ring_k) & ~cm
                vals = filled[ring]
                vals = vals[(vals > 0) & (vals != lab)]
                if len(vals) == 0:
                    continue
                frac_room = len(vals) / max(1, ring.sum())
                u, cnt = np.unique(vals, return_counts=True)
                if cnt.max() / len(vals) > 0.7 and frac_room > 0.55:
                    filled[cm] = u[cnt.argmax()]
                    changed = True
        if not changed:
            break

    return Segmentation(filled, names, colours, blue_rects, map_mask, unclassified)


# ---------------------------------------------------------------------------
# Stage 5: polygons, dock, naming
# ---------------------------------------------------------------------------


@dataclass
class ZoneOut:
    """One traced room."""

    name: str
    colour: str
    px: list[tuple[int, int]]
    cells: list[tuple[int, int]]
    area_px: int
    room_id: int | None = None

    @property
    def area_cells(self) -> float:
        return polygon_area([(float(x), float(y)) for x, y in self.cells])


def extract_zones(frame: Frame, seg: Segmentation, transform: Transform, min_area_px: int) -> tuple[list[ZoneOut], list[tuple[str, int]]]:
    """Trace each room component into a simplified polygon."""
    x_off, y_off, _, _ = frame.crop
    unit = frame.unit
    zones: list[ZoneOut] = []
    leftovers: list[tuple[str, int]] = []
    counts: dict[str, int] = {}
    for lab, name in enumerate(seg.names, 1):
        comps = label_components(seg.labels == lab)
        order = sorted(range(1, comps.count + 1), key=lambda c: -comps.area[c])
        for c in order:
            area = int(comps.area[c])
            if area < min_area_px:
                leftovers.append((name, area))
                continue
            cm = open_(comps.labels == c, _odd(5 * unit))
            if not cm.any():
                leftovers.append((name, area))
                continue
            inner = label_components(cm)
            main = 1 + int(inner.area[1:].argmax())
            cm = inner.labels == main
            contour = trace_boundary(cm)
            if len(contour) < 3:
                continue
            poly = simplify_polygon(np.array(contour, float), 2.0 * unit)
            if len(poly) < 3:
                continue
            counts[name] = counts.get(name, 0) + 1
            zone_name = name if counts[name] == 1 else f"{name} {counts[name]}"
            px = [(int(x) + x_off, int(y) + y_off) for x, y in poly]
            cells = [tuple(int(round(v)) for v in transform.to_cell(x, y)) for x, y in px]
            zones.append(ZoneOut(zone_name, zone_name, px, cells, area))  # type: ignore[arg-type]
    return zones, leftovers


def detect_dock(frame: Frame) -> tuple[float, float] | None:
    """The dock icon: a dark disc with a white centre dot, inside a white ring."""
    image = frame.map
    x_off, y_off, _, _ = frame.crop
    unit = frame.unit
    dark = image.max(-1) < 75
    white = image.min(-1) > 225
    comps = label_components(dark)
    h, w = dark.shape
    best: tuple[float, tuple[float, float]] | None = None
    for lab in range(1, comps.count + 1):
        x0, y0, x1, y1 = (int(v) for v in comps.bbox[lab])
        bw, bh = x1 - x0, y1 - y0
        if not (12 * unit <= bw <= 52 * unit and 12 * unit <= bh <= 52 * unit):
            continue
        if not 0.75 <= bw / bh <= 1.33:
            continue
        if x0 <= 0 or y0 <= 0 or x1 >= w or y1 >= h:
            continue
        fill = comps.area[lab] / (bw * bh)
        if not 0.4 <= fill <= 0.95:
            continue
        cm = comps.labels == lab
        holes = fill_holes(cm) & ~cm
        if holes.sum() < 0.02 * bw * bh or not white[holes].mean() > 0.5:
            continue
        ring = dilate(cm, _odd(5 * unit)) & ~cm
        white_ring = float(white[ring].mean())
        if white_ring < 0.5:
            continue
        centre = (x_off + (x0 + x1) / 2.0, y_off + (y0 + y1) / 2.0)
        if best is None or white_ring > best[0]:
            best = (white_ring, centre)
    return best[1] if best else None


def _norm_word(word: str) -> str:
    return "".join(ch for ch in word.casefold() if ch.isalnum())


def parse_mapping(arg: str | None, flag: str) -> dict[str, str]:
    """Parse `A=b,C=d` (values may not contain commas)."""
    out: dict[str, str] = {}
    if not arg:
        return out
    for item in arg.split(","):
        if not item.strip():
            continue
        if "=" not in item:
            raise ToolError(f"{flag}: expected NAME=VALUE, got {item!r}")
        key, value = item.split("=", 1)
        if not key.strip() or not value.strip():
            raise ToolError(f"{flag}: expected NAME=VALUE, got {item!r}")
        out[key.strip()] = value.strip()
    return out


def apply_names(zones: list[ZoneOut], names: dict[str, str], room_ids: dict[str, str], warnings: list[str]) -> None:
    """Rename zones by colour word and attach room ids by final name."""
    by_colour = {_norm_word(z.colour): z for z in zones}
    for name, colour in names.items():
        zone = by_colour.get(_norm_word(colour))
        if zone is None:
            warnings.append(f"--names: no zone has the colour {colour!r} (have: {', '.join(z.colour for z in zones)})")
            continue
        zone.name = name
    by_name = {z.name.casefold(): z for z in zones}
    for name, rid in room_ids.items():
        zone = by_name.get(name.casefold())
        if zone is None:
            warnings.append(f"--room-ids: no zone named {name!r}")
            continue
        try:
            zone.room_id = int(rid)
        except ValueError as err:
            raise ToolError(f"--room-ids: {rid!r} is not an integer room id") from err


# ---------------------------------------------------------------------------
# Stage 6: outputs
# ---------------------------------------------------------------------------


def build_document(zones: list[ZoneOut], dock: tuple[int, int] | None) -> dict:
    """The import document: version 1, taught, integer cells, no ids."""
    created = datetime.now(timezone.utc).isoformat(timespec="seconds")
    xs = [x for z in zones for x, _ in z.cells]
    ys = [y for z in zones for _, y in z.cells]
    return {
        "version": 1,
        "map_id": None,
        "dock_ref": [dock[0], dock[1]] if dock else None,
        "extent": [min(xs), min(ys), max(xs), max(ys)] if xs else None,
        "zones": [
            {
                "name": z.name,
                "points": [[x, y] for x, y in z.cells],
                "source": "taught",
                "room_id": z.room_id,
                "created": created,
            }
            for z in zones
        ],
    }


def validate_with_integration(doc: dict) -> str | None:
    """Round-trip through the integration's own parser when it is next door."""
    component = Path(__file__).resolve().parent.parent / "custom_components" / "bobsweep"
    if not (component / "zones.py").is_file():
        return None
    pkg = types.ModuleType("_bobs")
    pkg.__path__ = [str(component)]  # type: ignore[attr-defined]
    sys.modules.setdefault("_bobs", pkg)
    try:
        zmod = importlib.import_module("_bobs.zones")
        zset = zmod.ZoneSet.from_dict(json.loads(json.dumps(doc)))
        return f"validated with the integration's parser: {len(zset.zones)} zones accepted"
    except Exception as err:  # noqa: BLE001 - report, do not crash
        return f"warning: the integration's parser rejected the document: {err}"


def _font(size: int) -> ImageFont.ImageFont:
    for name in ("DejaVuSans.ttf", "arial.ttf", "Arial.ttf", "LiberationSans-Regular.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def draw_overlay(
    frame: Frame,
    zones: list[ZoneOut],
    fit: Fit,
    px_rects: list[PixelRect],
    cell_rects: list[CellRect],
    blue_rects: list[PixelRect],
    dock: tuple[float, float] | None,
    path: Path,
) -> None:
    """Polygons, re-projected wire rectangles and the dock, on the screenshot."""
    im = Image.fromarray(frame.image.astype(np.uint8)).convert("RGB")
    d = ImageDraw.Draw(im)
    unit = frame.unit
    font = _font(max(12, int(30 * unit)))
    small = _font(max(10, int(22 * unit)))
    lw = max(2, int(4 * unit))
    for z in zones:
        d.polygon(z.px, outline=(255, 255, 255), width=lw + 2)
        d.polygon(z.px, outline=(0, 0, 0), width=lw)
        cx = sum(p[0] for p in z.px) / len(z.px)
        cy = sum(p[1] for p in z.px) / len(z.px)
        d.text((cx + 1, cy + 1), z.name, fill=(255, 255, 255), font=font, anchor="mm")
        d.text((cx, cy), z.name, fill=(0, 0, 0), font=font, anchor="mm")
    t = fit.transform
    matched_cells = {j for _, j in fit.pairs}
    for j, cr in enumerate(cell_rects):
        a = t.to_px(cr.x0, cr.y0)
        b = t.to_px(cr.x1, cr.y1)
        box = [min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])]
        colour = (255, 230, 0) if j in matched_cells else (255, 120, 0)
        d.rectangle(box, outline=colour, width=lw)
        d.text((box[0] + lw, box[1] + lw), f"#{j + 1}", fill=colour, font=small)
    for i, pr in enumerate(px_rects):
        d.rectangle([pr.x0, pr.y0, pr.x1, pr.y1], outline=(0, 255, 255), width=max(1, lw // 2))
    for br in blue_rects:
        d.rectangle([br.x0, br.y0, br.x1, br.y1], outline=(80, 160, 255), width=max(1, lw // 2))
    if dock:
        r = 14 * unit
        d.ellipse([dock[0] - r, dock[1] - r, dock[0] + r, dock[1] + r], outline=(255, 0, 255), width=lw)
        d.text((dock[0] + r + 2, dock[1] - r), "dock", fill=(255, 0, 255), font=small)
    im.save(path)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="map_zones.py",
        description="Build a bObsweep zone document from a screenshot of the app's map.",
        epilog=(
            "The no-go rectangles come from the no_go_zones attribute of "
            "sensor.<name>_current_room in Home Assistant (Developer tools > States); "
            "save the attribute's JSON list to a file or paste it inline. Import the result with "
            "the bobsweep.import_zones service."
        ),
    )
    p.add_argument("--screenshot", required=True, metavar="PATH", help="screenshot of the app's Map screen (a full-phone screenshot is fine)")
    p.add_argument(
        "--no-go",
        required=True,
        metavar="PATH-or-JSON",
        help="the robot's no-go rectangles in map cells: a JSON list of rectangles, each four [x, y] corners (file path or inline JSON)",
    )
    p.add_argument("--names", metavar="NAME=COLOUR,...", help="rename zones by colour word, e.g. Kitchen=peach,Hallway=pink")
    p.add_argument("--room-ids", metavar="NAME=ID,...", help="attach the robot's room ids to zones by name, e.g. Kitchen=3")
    p.add_argument("--out", default="zones.json", metavar="PATH", help="where to write the zone document (default: zones.json)")
    p.add_argument("--overlay", default="overlay.png", metavar="PATH", help="where to write the check image (default: overlay.png)")
    p.add_argument("--min-area-px", type=int, default=None, metavar="N", help="ignore room fragments smaller than N pixels (default: 0.02%% of the image)")
    theme = p.add_mutually_exclusive_group()
    theme.add_argument("--dark-mode", dest="theme", action="store_const", const="dark", help="the app was in dark mode (default: auto-detect)")
    theme.add_argument("--light-mode", dest="theme", action="store_const", const="light", help="the app was in light mode (default: auto-detect)")
    p.add_argument("--dump-detected", action="store_true", help="print the red rectangles found on screen, in pixels, and continue")
    return p


def run(args: argparse.Namespace) -> int:
    warnings: list[str] = []
    names_map = parse_mapping(args.names, "--names")
    room_ids = parse_mapping(args.room_ids, "--room-ids")
    cell_rects = load_no_go(args.no_go)

    path = Path(args.screenshot)
    if not path.is_file():
        raise ToolError(f"screenshot not found: {path}")
    try:
        image = np.array(Image.open(path).convert("RGB")).astype(float)
    except Exception as err:  # noqa: BLE001
        raise ToolError(f"could not read the screenshot: {err}") from err
    h, w, _ = image.shape
    if w < 300 or h < 300:
        raise ToolError(f"the screenshot is only {w}x{h} pixels; that is too small to work from")

    frame = detect_frame(image, args.theme)
    min_area = args.min_area_px if args.min_area_px is not None else max(50, int(0.0002 * w * h))
    print(f"Screenshot: {w}x{h} px, {'dark' if frame.dark else 'light'} theme, background {tuple(int(v) for v in frame.background)}")
    print(f"Map area: x {frame.crop[0]}..{frame.crop[2]}, y {frame.crop[1]}..{frame.crop[3]}")

    px_rects = detect_red_rects(frame)
    if args.dump_detected:
        print(f"Detected {len(px_rects)} red rectangle(s) (pixels: x0, y0, x1, y1, w x h):")
        for i, r in enumerate(px_rects, 1):
            print(f"  R{i}: {r.x0}, {r.y0}, {r.x1}, {r.y1}  ({r.w} x {r.h})")
    if not px_rects:
        raise ToolError("no red (no-go) rectangles found in the screenshot; the map screen must show the no-go zones")
    print(f"Found {len(px_rects)} red rectangle(s) on screen; {len(cell_rects)} no-go rectangle(s) from the robot")

    fit, tried = fit_transform(px_rects, cell_rects, w)
    t = fit.transform
    expected = (t.sx, t.sy) == EXPECTED_ORIENTATION
    print(f"Fit: cell_x = {t.ox:.1f} {'-' if t.sx < 0 else '+'} {t.k:.4f} * px_x ; cell_y = {t.oy:.1f} {'-' if t.sy < 0 else '+'} {t.k:.4f} * px_y")
    print(f"     {t.describe()} ({'the expected rotate-180 layout' if expected else 'NOT the expected layout'}); {t.k:.3f} cells per pixel; rms {fit.rms:.1f} cells")
    print("     orientations tried: " + ", ".join(f"{'left' if s.sx < 0 else 'right'}/{'down' if s.sy > 0 else 'up'}: {n} matched, rms {r:.1f}" if n >= 2 else f"{'left' if s.sx < 0 else 'right'}/{'down' if s.sy > 0 else 'up'}: {n} matched" for s, n, r in tried))
    if not expected:
        warnings.append(f"the best fit is {t.describe()}, not the rotate-180 layout the app normally uses; check the overlay carefully")
    print("Rectangle residuals (predicted minus actual, cells: left, right, top, bottom):")
    worst = 0.0
    for (i, j), res in zip(fit.pairs, fit.residuals):
        pr, cr = px_rects[i], cell_rects[j]
        rms = math.sqrt(sum(v * v for v in res) / 4)
        worst = max(worst, max(abs(v) for v in res))
        print(f"  no-go #{j + 1} ({cr.w}x{cr.h} cells) <- screen R{i + 1} ({pr.w}x{pr.h} px): {tuple(round(v, 1) for v in res)}  rms {rms:.1f}")
    unmatched = [j + 1 for j in range(len(cell_rects)) if j not in {j for _, j in fit.pairs}]
    if unmatched:
        print(f"  unmatched no-go rectangles: {unmatched}")
    if len(fit.pairs) < 3:
        warnings.append(f"only {len(fit.pairs)} rectangles matched; the scale is poorly constrained")
    if worst > 40:
        warnings.append(f"a rectangle edge is off by {worst:.0f} cells; the map may not be the one the rectangles belong to, or a rectangle was mis-detected")

    # Every red hatch on screen is un-blended, matched to a wire rectangle or not.
    seg = segment(frame, px_rects, min_area)
    if seg.unclassified_fraction > 0.15:
        warnings.append(f"{seg.unclassified_fraction:.0%} of the floor was not a known room colour before filling; check the overlay")
    if seg.blue_rects:
        print(f"No-mop rectangles on screen: {len(seg.blue_rects)}")
    zones, leftovers = extract_zones(frame, seg, t, min_area)
    if not zones:
        raise ToolError("no rooms were found; the map must show coloured rooms (raise or lower --min-area-px if it does)")
    apply_names(zones, names_map, room_ids, warnings)

    dock_px = detect_dock(frame)
    dock_cell = None
    if dock_px:
        dock_cell = tuple(int(round(v)) for v in t.to_cell(*dock_px))
        print(f"Dock: pixel ({dock_px[0]:.0f}, {dock_px[1]:.0f}) -> cell {dock_cell}")
    else:
        print("Dock: not found (dock_ref will be null)")

    print(f"Zones ({len(zones)}; areas in cells^2 and screen pixels):")
    for z in zones:
        rid = f", room_id {z.room_id}" if z.room_id is not None else ""
        print(f"  {z.name}: {len(z.cells)} vertices, {z.area_cells:,.0f} cells^2, {z.area_px:,} px{rid}")
    if leftovers:
        print(f"  dropped {len(leftovers)} fragment(s) under {min_area} px")

    doc = build_document(zones, dock_cell)  # type: ignore[arg-type]
    out = Path(args.out)
    out.write_text(json.dumps(doc, indent=1), encoding="utf-8")
    note = validate_with_integration(doc)
    if note:
        print(note)
        if note.startswith("warning"):
            warnings.append(note)
    draw_overlay(frame, zones, fit, px_rects, cell_rects, seg.blue_rects, dock_px, Path(args.overlay))
    print(f"Wrote {out} and {args.overlay}")
    for wmsg in warnings:
        print(f"WARNING: {wmsg}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return run(args)
    except ToolError as err:
        print(f"error: {err}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
