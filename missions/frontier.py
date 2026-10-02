"""FRONTIER3D selection rule, embodiment-agnostic, on a GridMap.

A frontier cell is a known-free cell with an unknown 8-neighbour, counted on the raw
map, not the inflated one, so a doorway's frontier keeps its full width (a 0.9 m door is
18 cells; inside the 0.35 m inflation band only 4 would remain and the door would never
qualify: the rooms behind it would never be explored). Frontier cells are clustered by
8-connectivity; clusters smaller than `min_cells` (10 cells = 0.5 m at 0.05 m/px) are
ignored. Each cluster's goal is its centroid pulled into ADMISSIBLE free space (the
nearest known-free cell outside the inflation band within 1 m): for a doorway, the
opening's centre line.
The selection is the nearest goal by A*/Dijkstra path length over the current map with
unknown cells impassable (not inflated: the goals sit at the edge of the known space), never
Euclidean. Every attempted goal is blacklisted afterwards, failed or reached (no
candidate within `blacklist_m` of it is chosen again): a reached frontier normally
dissolves, one that survives its own visit is an unresolvable sliver. Two vehicle-level
filters (from an exploration test flight, 2026-09-24): a goal in a slot between two known obstacles
narrower than `min_passage_m` (the vehicle cannot fit: a 0.61 m gap between a cylinder and
the arena wall) is skipped, and the path length is measured to the nearest reachable cell
within `tol_m` of the goal (the planner's goal tolerance: with the unknown margin of the
planning mask the goal cell itself may sit inside the band). No further parameters.

Pure numpy; shared by the aerial and the wheeled exploration runners.
"""
import math
from collections import deque

import numpy as np

from gridmap import GridMap


def frontier_cells(gm, radius_m=0.35):
    """Boolean mask of frontier cells: known-free cells (raw map) next to unknown."""
    free = gm.cls == GridMap.FREE
    unk = gm.cls == GridMap.UNK
    adm = free
    near_unk = np.zeros_like(unk)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            shifted = np.zeros_like(unk)
            src = unk[max(0, -dr):gm.h - max(0, dr), max(0, -dc):gm.w - max(0, dc)]
            shifted[max(0, dr):gm.h - max(0, -dr), max(0, dc):gm.w - max(0, -dc)] = src
            near_unk |= shifted
    return adm & near_unk


def clusters(gm, radius_m=0.35, min_cells=10, min_passage_m=0.0):
    """List of frontier clusters: dict(cells=[(r,c)...], size, centroid=(x,y), goal=(x,y)).
    Clusters whose goal lies in a slot narrower than min_passage_m (raw map, known
    obstacles) are left out."""
    mask = frontier_cells(gm, radius_m)
    seen = np.zeros_like(mask)
    lethal = gm.lethal(radius_m, "free")
    out = []
    rs, cs = np.nonzero(mask)
    for r0, c0 in zip(rs, cs):
        if seen[r0, c0]:
            continue
        cells = []
        dq = deque([(r0, c0)])
        seen[r0, c0] = True
        while dq:
            r, c = dq.popleft()
            cells.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    r2, c2 = r + dr, c + dc
                    if gm.inside(r2, c2) and mask[r2, c2] and not seen[r2, c2]:
                        seen[r2, c2] = True
                        dq.append((r2, c2))
        if len(cells) < min_cells:
            continue
        cr = sum(x[0] for x in cells) / len(cells)
        cc = sum(x[1] for x in cells) / len(cells)
        centroid = gm.to_xy(cr, cc)
        goal = _pull_into_free(gm, lethal, cr, cc, max_m=1.0)
        if goal is None:
            continue
        if min_passage_m > 0 and gm.min_free_width(goal) < min_passage_m:
            continue
        out.append(dict(cells=cells, size=len(cells), centroid=centroid, goal=goal))
    return out


def _pull_into_free(gm, lethal, r, c, max_m=1.0):
    """Nearest admissible known-free cell to (r, c) within max_m, as (x, y)."""
    k = int(math.ceil(max_m / gm.resolution))
    best, bestd = None, float("inf")
    r0, c0 = int(round(r)), int(round(c))
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            r2, c2 = r0 + dr, c0 + dc
            d = math.hypot(dr, dc)
            if d < bestd and gm.inside(r2, c2) and gm.cls[r2, c2] == GridMap.FREE and not lethal[r2, c2]:
                best, bestd = (r2, c2), d
    return gm.to_xy(*best) if best else None


def choose(gm, pose_xy, cl, blacklist, radius_m=0.35, blacklist_m=0.5, tol_m=0.5):
    """The nearest cluster goal by path length (unknown impassable), skipping blacklisted
    ones; the length is to the nearest reachable cell within tol_m of the goal (the
    planner's goal tolerance). Returns (cluster, path_len_m) or (None, None)."""
    dist = gm.dijkstra(pose_xy, radius_m, "wall")
    if dist is None:
        return None, None
    k = int(math.ceil(tol_m / gm.resolution))
    best, bestd = None, float("inf")
    for c in cl:
        gx, gy = c["goal"]
        if any(math.hypot(gx - bx, gy - by) < blacklist_m for bx, by in blacklist):
            continue
        r, col = gm.to_cell(gx, gy)
        if not gm.inside(r, col):
            continue
        d = dist[r, col]
        if not math.isfinite(d):
            r0, r1 = max(0, r - k), min(gm.h - 1, r + k)
            c0, c1 = max(0, col - k), min(gm.w - 1, col + k)
            win = dist[r0:r1 + 1, c0:c1 + 1]
            yy, xx = np.mgrid[r0:r1 + 1, c0:c1 + 1]
            ok = ((yy - r) ** 2 + (xx - col) ** 2) * gm.resolution ** 2 <= tol_m ** 2
            vals = win[ok]
            d = float(vals.min()) if len(vals) else float("inf")
        if d < bestd:
            best, bestd = c, d
    return best, (bestd if best else None)
