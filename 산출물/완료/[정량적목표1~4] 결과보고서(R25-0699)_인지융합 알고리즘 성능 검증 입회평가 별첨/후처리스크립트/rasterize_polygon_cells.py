#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Accurate polygon-to-grid rasterization (occupied cell extraction) in world coordinates.

- World coords unit: dm (or any unit)
- Grid: origin (0,0), cell size = cell_size (default 1.0)
- Occupied definition (default): polygon and cell AABB overlap with positive area.
  * If you want "touching on edge/point counts as occupied", set include_touch=True.

This implementation uses SAT (Separating Axis Theorem) for Convex Polygon vs AABB.

Usage examples:
  1) Run with the sample corners from the chat:
     python rasterize_polygon_cells.py

  2) Provide your own corners:
     python rasterize_polygon_cells.py --corners 144.7 136.6 143.8 127.2 140.8 127.5 141.7 136.6

  3) Change cell size:
     python rasterize_polygon_cells.py --cell-size 1.0

Output:
  Prints occupied cells as a sorted list of [ix, iy].
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Set, Tuple

EPS = 1e-12

Pt = Tuple[float, float]
Cell = Tuple[int, int]


def dot(a: Pt, b: Pt) -> float:
    return a[0] * b[0] + a[1] * b[1]


def sub(a: Pt, b: Pt) -> Pt:
    return (a[0] - b[0], a[1] - b[1])


def aabb_corners(x0: float, y0: float, x1: float, y1: float) -> List[Pt]:
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def proj_interval(points: Sequence[Pt], axis: Pt) -> Tuple[float, float]:
    mn = float("inf")
    mx = -float("inf")
    for p in points:
        v = dot(p, axis)
        if v < mn:
            mn = v
        if v > mx:
            mx = v
    return mn, mx


def overlap_1d(a: Tuple[float, float], b: Tuple[float, float], *, include_touch: bool) -> bool:
    """
    If include_touch is False:
      - requires positive-length overlap (touching only at boundary => not overlapping)
    If include_touch is True:
      - touching counts as overlap
    """
    a0, a1 = a
    b0, b1 = b
    if include_touch:
        return not (a1 < b0 - EPS or b1 < a0 - EPS)
    return not (a1 <= b0 + EPS or b1 <= a0 + EPS)


def convex_poly_intersects_aabb_sat(
    poly: Sequence[Pt],
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    *,
    include_touch: bool = False,
) -> bool:
    """
    SAT test for convex polygon vs axis-aligned rectangle [x0,x1]x[y0,y1].
    """
    rect = aabb_corners(x0, y0, x1, y1)

    axes: List[Pt] = [(1.0, 0.0), (0.0, 1.0)]  # rectangle axes

    n = len(poly)
    for i in range(n):
        p0 = poly[i]
        p1 = poly[(i + 1) % n]
        e = sub(p1, p0)
        axis = (-e[1], e[0])  # edge normal
        if abs(axis[0]) < EPS and abs(axis[1]) < EPS:
            continue
        axes.append(axis)

    for ax in axes:
        pmin, pmax = proj_interval(poly, ax)
        rmin, rmax = proj_interval(rect, ax)
        if not overlap_1d((pmin, pmax), (rmin, rmax), include_touch=include_touch):
            return False
    return True


def rasterize_polygon_to_cells(
    poly: Sequence[Pt],
    *,
    cell_size: float = 1.0,
    include_touch: bool = False,
) -> Set[Cell]:
    """
    Rasterize convex polygon to occupied grid cells.

    Grid cell (ix,iy) represents AABB: [ix*cell_size,(ix+1)*cell_size] x [iy*cell_size,(iy+1)*cell_size]

    Returns a set of (ix,iy).
    """
    if cell_size <= 0:
        raise ValueError("cell_size must be > 0")

    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)

    ix0 = int(math.floor(minx / cell_size))
    ix1 = int(math.ceil(maxx / cell_size)) - 1
    iy0 = int(math.floor(miny / cell_size))
    iy1 = int(math.ceil(maxy / cell_size)) - 1

    occupied: Set[Cell] = set()
    for ix in range(ix0, ix1 + 1):
        x0 = ix * cell_size
        x1 = (ix + 1) * cell_size
        for iy in range(iy0, iy1 + 1):
            y0 = iy * cell_size
            y1 = (iy + 1) * cell_size
            if convex_poly_intersects_aabb_sat(poly, x0, y0, x1, y1, include_touch=include_touch):
                occupied.add((ix, iy))

    return occupied


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--corners",
        type=float,
        nargs=8,
        default=[
            144.72350882, 136.6155826,
            143.88391929, 127.28468621,
            140.89599051, 127.55353859,
            141.73558004, 136.6155826,
        ],
        help="8 floats: xLU yLU xRU yRU xRL yRL xLL yLL (world coordinates).",
    )
    p.add_argument("--cell-size", type=float, default=1.0, help="Grid cell size in same unit as corners (default 1.0).")
    p.add_argument(
        "--include-touch",
        action="store_true",
        help="Count boundary touching (edge/point contact) as occupied.",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    c = args.corners
    poly: List[Pt] = [(c[0], c[1]), (c[2], c[3]), (c[4], c[5]), (c[6], c[7])]

    occ = rasterize_polygon_to_cells(poly, cell_size=args.cell_size, include_touch=args.include_touch)
    out = sorted([[ix, iy] for (ix, iy) in occ], key=lambda t: (t[1], t[0]))  # sort by y then x
    print(out)
    print(f"count={len(out)}")


if __name__ == "__main__":
    main()
