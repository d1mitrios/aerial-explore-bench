#!/usr/bin/env python3
"""Frozen 2D occupancy map + inflated A* for the aerial mission executive.

Reads a map in the ROS map_server format (YAML + PGM, 0.05 m/px, trinary), classifies
cells as occupied / free / unknown exactly as map_server does (occupied_thresh,
free_thresh, negate), inflates occupied cells by the planner radius (lethal disc) and
plans 8-connected A* with an octile heuristic and a proximity penalty near obstacles
(paths prefer the middle of a doorway, as NavFn's cost gradient does).

Unknown cells are handled per the `unknown` argument of plan():
  "free"   - traversable, as the wheeled baseline's global costmap treated them
             (track_unknown_space: false)
  "lethal" - never entered; the planner refuses goals through unexplored space
  "wall"   - never entered but not inflated (exploration: frontier goals sit at the edge)

Goal tolerance mirrors NavFn: if the goal cell is not plannable, the nearest plannable
cell within `tolerance` metres is used instead.

Pure numpy; importable from the executive or usable standalone:
  python3 missions/gridmap.py worlds/maps/world_20260723008.yaml 0 0 4.5 -3.0
"""
import heapq
import math
import os
import sys

import numpy as np
import yaml


class GridMap:
    OCC, FREE, UNK = 1, 0, -1

    def __init__(self, yaml_path=None, cls=None, resolution=0.05, origin=(0.0, 0.0), unknown_margin=0.0):
        """From a map_server YAML/PGM (yaml_path) or from a classified grid (cls: row 0 =
        bottom, values OCC / FREE / UNK) with its resolution and origin. unknown_margin (m):
        in the "wall" mode, unknown cells are dilated by this much in the planning mask;
        the true wall hides in the unknown strip next to the known free space (an exploration
        test flight: a 0.6 m slot between a cylinder and an unmapped wall was planned through
        at 0.15 m from the wall)."""
        if yaml_path is not None:
            with open(yaml_path) as f:
                meta = yaml.safe_load(f)
            self.yaml_path = yaml_path
            self.resolution = float(meta["resolution"])
            self.origin = [float(v) for v in meta["origin"]][:2]
            negate = int(meta.get("negate", 0))
            occ_t = float(meta.get("occupied_thresh", 0.65))
            free_t = float(meta.get("free_thresh", 0.196))
            img = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta["image"])
            pix = self._read_pgm(img)                       # row 0 = top of the image
            p = pix / 255.0 if negate else (255.0 - pix) / 255.0
            cls = np.full(pix.shape, self.UNK, dtype=np.int8)
            cls[p > occ_t] = self.OCC
            cls[p < free_t] = self.FREE
            self.cls = cls[::-1, :].copy()                  # row 0 = bottom (y = origin_y)
        else:
            self.yaml_path = None
            self.resolution = float(resolution)
            self.origin = [float(origin[0]), float(origin[1])]
            self.cls = np.asarray(cls, dtype=np.int8).copy()
        self.h, self.w = self.cls.shape
        self.unknown_margin = float(unknown_margin)
        self._inflated = {}

    @classmethod
    def from_occupancy(cls_, data, width, height, resolution, origin_xy, occ_min=65, free_max=25,
                       unknown_margin=0.0):
        """From a ROS OccupancyGrid (row-major, row 0 = the origin row; -1 unknown, 0-100
        occupancy). Thresholds as map_server's trinary output: >= occ_min occupied,
        <= free_max free (and >= 0), else unknown."""
        arr = np.asarray(data, dtype=np.int16).reshape(height, width)
        cls = np.full(arr.shape, cls_.UNK, dtype=np.int8)
        cls[arr >= occ_min] = cls_.OCC
        cls[(arr >= 0) & (arr <= free_max)] = cls_.FREE
        return cls_(cls=cls, resolution=resolution, origin=origin_xy, unknown_margin=unknown_margin)

    def known_free_area(self):
        """Known-free area, m^2 (the coverage measure of the exploration phase)."""
        return float((self.cls == self.FREE).sum()) * self.resolution ** 2

    def save(self, path_base):
        """Write <path_base>.pgm + .yaml in map_server's trinary format (254 free, 0
        occupied, 205 unknown; row 0 of the image = the top)."""
        img = np.full(self.cls.shape, 205, dtype=np.uint8)
        img[self.cls == self.FREE] = 254
        img[self.cls == self.OCC] = 0
        img = img[::-1, :]
        with open(path_base + ".pgm", "wb") as f:
            f.write(f"P5\n{self.w} {self.h}\n255\n".encode())
            f.write(img.tobytes())
        with open(path_base + ".yaml", "w") as f:
            f.write(f"image: {os.path.basename(path_base)}.pgm\nmode: trinary\nresolution: {self.resolution}\n"
                    f"origin: [{self.origin[0]:.6f}, {self.origin[1]:.6f}, 0]\nnegate: 0\n"
                    f"occupied_thresh: 0.65\nfree_thresh: 0.196\n")
        return path_base + ".yaml"

    def dijkstra(self, start_xy, radius_m=0.35, unknown="wall"):
        """Path length (m) from start to every cell over the inflated map (inf = unreachable);
        8-connected, octile costs, no corner cutting; the frontier metric."""
        lethal = self.lethal(radius_m, unknown)
        sr, sc = self.to_cell(*start_xy)
        if not self.inside(sr, sc):
            return None
        if lethal[sr, sc]:
            near = self._nearest_ok(sr, sc, lethal, 0.5)
            if near is None:
                return None
            sr, sc = near
        dist = np.full(self.cls.shape, np.inf)
        dist[sr, sc] = 0.0
        pq = [(0.0, sr, sc)]
        steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
        while pq:
            d, r, c = heapq.heappop(pq)
            if d > dist[r, c]:
                continue
            for dr, dc, w in steps:
                r2, c2 = r + dr, c + dc
                if not self.inside(r2, c2) or lethal[r2, c2]:
                    continue
                if dr and dc and (lethal[r, c2] or lethal[r2, c]):
                    continue
                d2 = d + w
                if d2 < dist[r2, c2]:
                    dist[r2, c2] = d2
                    heapq.heappush(pq, (d2, r2, c2))
        return dist * self.resolution

    @staticmethod
    def _read_pgm(path):
        with open(path, "rb") as f:
            magic = f.readline().strip()
            if magic != b"P5":
                raise ValueError(f"{path}: not a binary PGM (P5)")
            tokens = []
            while len(tokens) < 3:
                line = f.readline()
                if line.startswith(b"#"):
                    continue
                tokens += line.split()
            w, h, maxval = int(tokens[0]), int(tokens[1]), int(tokens[2])
            data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
        return data.astype(np.float64) * (255.0 / maxval)

    # ---------------------------------------------------------------- frames
    def to_cell(self, x, y):
        return (int(math.floor((y - self.origin[1]) / self.resolution)),
                int(math.floor((x - self.origin[0]) / self.resolution)))

    def to_xy(self, r, c):
        return (self.origin[0] + (c + 0.5) * self.resolution,
                self.origin[1] + (r + 0.5) * self.resolution)

    def inside(self, r, c):
        return 0 <= r < self.h and 0 <= c < self.w

    # ------------------------------------------------------------- inflation
    def _disc(self, radius_m):
        k = int(math.ceil(radius_m / self.resolution))
        yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
        return (xx * xx + yy * yy) * self.resolution ** 2 <= radius_m ** 2 + 1e-9

    def _dilate(self, mask, radius_m):
        disc = self._disc(radius_m)
        k = disc.shape[0] // 2
        out = np.zeros_like(mask)
        rs, cs = np.nonzero(mask)
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                if not disc[dr + k, dc + k]:
                    continue
                r2, c2 = rs + dr, cs + dc
                ok = (r2 >= 0) & (r2 < self.h) & (c2 >= 0) & (c2 < self.w)
                out[r2[ok], c2[ok]] = True
        return out

    def lethal(self, radius_m, unknown="free"):
        """Boolean mask of cells the vehicle centre may not enter. unknown: "free" (only
        occupied cells inflated; the baseline costmap's behaviour), "lethal" (unknown cells
        inflated like obstacles), "wall" (unknown cells impassable but not inflated; the
        exploration planner's mode: the vehicle may fly up to the edge of the known space,
        which is where every frontier goal is)."""
        key = (round(radius_m, 3), unknown, round(self.unknown_margin, 3))
        if key not in self._inflated:
            occ = self.cls == self.OCC
            if unknown == "lethal":
                occ = occ | (self.cls == self.UNK)
            mask = self._dilate(occ, radius_m)
            if unknown == "wall":
                unk = self.cls == self.UNK
                mask = mask | (self._dilate(unk, self.unknown_margin) if self.unknown_margin > 0 else unk)
            self._inflated[key] = mask
        return self._inflated[key]

    def clearance_cost(self, radius_m, soft_m, unknown="free"):
        """Extra step cost inside the soft band [radius, radius+soft] around obstacles."""
        key = ("cost", round(radius_m, 3), round(soft_m, 3), unknown)
        if key not in self._inflated:
            occ = self.cls == self.OCC
            if unknown == "lethal":
                occ = occ | (self.cls == self.UNK)
            band = self._dilate(occ, radius_m + soft_m) & ~self._dilate(occ, radius_m)
            self._inflated[key] = band
        return self._inflated[key]

    # ----------------------------------------------------------------- A*
    def plan(self, start_xy, goal_xy, radius_m=0.35, unknown="free", tolerance=0.5,
             soft_m=0.15, soft_cost=1.5):
        """A* from start to goal (metres). Returns (path_xy, goal_used_xy) or (None, None)."""
        lethal = self.lethal(radius_m, unknown)
        band = self.clearance_cost(radius_m, soft_m, unknown)
        sr, sc = self.to_cell(*start_xy)
        gr, gc = self.to_cell(*goal_xy)
        if not self.inside(sr, sc):
            return None, None
        if lethal[sr, sc]:
            # the vehicle may sit inside the inflation band after a drift: start from the
            # nearest plannable cell instead of refusing outright
            near = self._nearest_ok(sr, sc, lethal, 0.5)
            if near is None:
                return None, None
            sr, sc = near
        if not self.inside(gr, gc) or lethal[gr, gc]:
            near = self._nearest_ok(gr, gc, lethal, tolerance)
            if near is None:
                return None, None
            gr, gc = near
        goal_used = self.to_xy(gr, gc)

        steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]

        def h(r, c):
            dr, dc = abs(r - gr), abs(c - gc)
            return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)

        g = {(sr, sc): 0.0}
        came = {}
        pq = [(h(sr, sc), 0.0, (sr, sc))]
        closed = set()
        while pq:
            f, gcur, (r, c) = heapq.heappop(pq)
            if (r, c) in closed:
                continue
            closed.add((r, c))
            if (r, c) == (gr, gc):
                cells = [(r, c)]
                while cells[-1] in came:
                    cells.append(came[cells[-1]])
                cells.reverse()
                return [self.to_xy(*rc) for rc in cells], goal_used
            for dr, dc, w in steps:
                r2, c2 = r + dr, c + dc
                if not self.inside(r2, c2) or lethal[r2, c2] or (r2, c2) in closed:
                    continue
                if dr and dc and (lethal[r, c2] or lethal[r2, c]):
                    continue                            # no corner cutting through lethal cells
                g2 = gcur + w * (soft_cost if band[r2, c2] else 1.0)
                if g2 < g.get((r2, c2), float("inf")):
                    g[(r2, c2)] = g2
                    came[(r2, c2)] = (r, c)
                    heapq.heappush(pq, (g2 + h(r2, c2), g2, (r2, c2)))
        return None, None

    def _nearest_ok(self, r, c, lethal, tol_m):
        k = int(math.ceil(tol_m / self.resolution))
        best, bestd = None, float("inf")
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                r2, c2 = r + dr, c + dc
                d = math.hypot(dr, dc) * self.resolution
                if d <= tol_m and d < bestd and self.inside(r2, c2) and not lethal[r2, c2]:
                    best, bestd = (r2, c2), d
        return best

    def los_free(self, p, q, radius_m=0.35, unknown="free", escape_m=0.0):
        """True if the straight segment p->q (metres) crosses no lethal cell (Bresenham).
        Lethal cells within escape_m of p are tolerated: a belief that sits slightly inside
        the inflation band (a 0.1 m localization error in a doorway whose inflated corridor
        is one cell wide) may still look along the path ahead, out of the band."""
        lethal = self.lethal(radius_m, unknown)
        r0, c0 = self.to_cell(*p)
        r1, c1 = self.to_cell(*q)
        dr, dc = abs(r1 - r0), abs(c1 - c0)
        sr, sc = (1 if r1 > r0 else -1), (1 if c1 > c0 else -1)
        err = dr - dc
        r, c = r0, c0
        while True:
            if not self.inside(r, c):
                return False
            if lethal[r, c] and math.hypot(r - r0, c - c0) * self.resolution > escape_m:
                return False
            if (r, c) == (r1, c1):
                return True
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc

    def carrot(self, path, idx, p, lookahead, radius_m=0.35, unknown="free", escape_m=0.25):
        """Progress index + carrot for pure pursuit: the farthest path point within
        `lookahead` that is in free line of sight (on the inflated map, tolerating the
        first escape_m from p), so the follower never cuts a corner into the inflation
        band. Without any such point the carrot is the point half a lookahead AHEAD ALONG
        the path, never the next cell: from a belief that sits beside the path inside a
        doorway's inflation band, the next cell lies sideways, and the follower would turn
        the vehicle across the door (in an early test flight). Returns (new_idx, carrot_xy)."""
        best, bestd = idx, float("inf")
        for j in range(idx, min(idx + 40, len(path))):
            d = math.hypot(path[j][0] - p[0], path[j][1] - p[1])
            if d < bestd:
                best, bestd = j, d
        idx = best
        last = idx
        for j in range(idx, len(path)):
            if math.hypot(path[j][0] - p[0], path[j][1] - p[1]) > lookahead:
                break
            last = j
        for j in range(last, idx, -1):
            if self.los_free(p, path[j], radius_m, unknown, escape_m):
                return idx, path[j]
        return idx, self.point_along(path, idx, lookahead / 2)

    @staticmethod
    def index_along(path, idx, dist_m):
        """Index of the path point about dist_m further along the path from index idx."""
        acc, j = 0.0, idx
        while j + 1 < len(path) and acc < dist_m:
            acc += math.hypot(path[j + 1][0] - path[j][0], path[j + 1][1] - path[j][1])
            j += 1
        return j

    @staticmethod
    def point_along(path, idx, dist_m):
        """The path point about dist_m further along the path from index idx."""
        return path[GridMap.index_along(path, idx, dist_m)]

    @staticmethod
    def dist_along(path, i, j):
        """Path length between indices i <= j."""
        return sum(math.hypot(path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1]) for k in range(i, j))

    def narrowest_ahead(self, path, idx, ahead_m, step=3):
        """(width, index) of the narrowest raw-map free width across the path between idx and
        ahead_m further along it (sampled every `step` points)."""
        j_end = self.index_along(path, idx, ahead_m)
        best = (float("inf"), idx)
        for j in range(idx, j_end + 1, step):
            wl, wr = self.free_width(path[j], self.tangent(path, j))
            if wl + wr < best[0]:
                best = (wl + wr, j)
        return best

    @staticmethod
    def tangent(path, idx, span_m=0.5):
        """Unit direction of the path around index idx (over +-span_m along it): the axis
        the vehicle should align its footprint with, independent of where the belief sits
        relative to the path."""
        a = b = idx
        acc = 0.0
        while a > 0 and acc < span_m:
            acc += math.hypot(path[a][0] - path[a - 1][0], path[a][1] - path[a - 1][1])
            a -= 1
        acc = 0.0
        while b + 1 < len(path) and acc < span_m:
            acc += math.hypot(path[b + 1][0] - path[b][0], path[b + 1][1] - path[b][1])
            b += 1
        dx, dy = path[b][0] - path[a][0], path[b][1] - path[a][1]
        n = math.hypot(dx, dy)
        return (dx / n, dy / n) if n > 1e-6 else (1.0, 0.0)

    def free_width(self, p, direction, max_m=3.0):
        """Free width of the raw map across `direction` at p: distance to the nearest
        occupied cell on each side, perpendicular to the direction (left, right)."""
        px, py = -direction[1], direction[0]
        out = []
        for s in (1.0, -1.0):
            d = 0.0
            while d < max_m:
                r, c = self.to_cell(p[0] + s * px * d, p[1] + s * py * d)
                if not self.inside(r, c) or self.cls[r, c] == 1:
                    break
                d += self.resolution
            out.append(d)
        return out[0], out[1]

    def min_free_width(self, p, max_m=3.0):
        """The narrowest raw-map free width through p over four directions (0, 45, 90, 135
        deg): a goal in a slot between two KNOWN obstacles narrower than the vehicle has a
        small value here whatever the slot's orientation (unknown cells count as free)."""
        best = float("inf")
        for ang in (0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4):
            wl, wr = self.free_width(p, (math.cos(ang), math.sin(ang)), max_m)
            best = min(best, wl + wr)
        return best

    @staticmethod
    def path_length(path):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])) if path else 0.0


def main():
    if len(sys.argv) < 6:
        raise SystemExit("usage: gridmap.py <map.yaml> <sx> <sy> <gx> <gy> [radius] [unknown]")
    gm = GridMap(sys.argv[1])
    s = (float(sys.argv[2]), float(sys.argv[3]))
    g = (float(sys.argv[4]), float(sys.argv[5]))
    radius = float(sys.argv[6]) if len(sys.argv) > 6 else 0.35
    unknown = sys.argv[7] if len(sys.argv) > 7 else "free"
    print(f"map {gm.w}x{gm.h} @ {gm.resolution} m, origin {gm.origin}; "
          f"occupied {int((gm.cls == 1).sum())}, free {int((gm.cls == 0).sum())}, unknown {int((gm.cls == -1).sum())}")
    path, used = gm.plan(s, g, radius_m=radius, unknown=unknown)
    if path is None:
        print(f"NO PATH from {s} to {g} (radius {radius}, unknown={unknown})")
        return 1
    print(f"path: {len(path)} cells, {gm.path_length(path):.2f} m, goal used {used[0]:.2f},{used[1]:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
