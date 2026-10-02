#!/usr/bin/env python
"""
| File: lidar_raycast.py
| Description: The benchmark's single lidar model, on the quadrotor. Direct PhysX
|              scene-query raycasting (360 rays, 0.1-100 m, ~20 Hz) -> sensor_msgs/LaserScan
|              on /scan. Port of the wheeled baseline's publisher (scan_raycast_publisher2.py,
|              Isaac 6.0.x: the RTX lidar is broken there, the scene-query raycast is not)
|              into the standalone flight loop (aeb_flight.Component).
|              Ray model, rate, range, frame name and the self-hit skip are UNCHANGED, so both
|              embodiments carry the same lidar.
|
| Mount (the two things that differ on a quadrotor):
|  1. A body-fixed scan plane tilts with every roll/pitch: at 1.2-1.5 m altitude a few
|     degrees of tilt sends rays over the 2.0 m walls (phantom free space) or into the
|     floor (phantom obstacles). Default here is STABILIZED = the scan plane stays
|     horizontal and follows yaw only (a gimballed / attitude-compensated 2D lidar), which
|     is what a planar SLAM front end assumes. stabilize=False (AEB_LIDAR_STABILIZE=0)
|     scans in the body plane instead.
|  2. The ray origin sits 0.10 m ABOVE the body origin, on top of the frame (top plate
|     at +0.047 m, propeller plane at +0.028 m). Inside the frame the ray starts within
|     the Iris body's collider, a convexDecomposition of many small solid pieces, and the
|     self-hit skip cannot chain through 0.15-0.27 m of them (measured on the first A7
|     flight: ~60 % of rays lost along the body axis, the four arm directions and, on the
|     ground, the skids). The skip is kept, made robust (10 steps of hit+0.05 m), as the
|     safety net for a propeller tip entering the plane during a banked turn; rays that
|     still exhaust it are counted separately (`selfblk` in the telemetry).
|
| Sample CSV (optional): every `sample_every`-th scan is written with the TRUE origin pose,
| out-of-band, for the offline geometry check (analysis/plot_lidar_check.py). Ground truth
| never goes on a topic.
"""

import math
import os

import carb

from aeb_flight import Component, RUNS_DIR, stamp


class RaycastLidar2D(Component):
    name = "lidar"

    SELF_SKIP_STEPS = 10           # max self-hit skips per ray (the baseline used 5 x 0.02 m)
    SELF_SKIP_M = 0.05             # advance past each self hit by hit distance + this

    def __init__(self, n_rays=360, rmin=0.1, rmax=100.0, z_offset=0.10, stabilize=True,
                 frame_id="lidar_link", topic="/scan", every_n_frames=3,
                 sample_csv=True, sample_every=20):
        self.n = n_rays
        self.a_min = -math.pi
        self.a_inc = 2.0 * math.pi / n_rays
        self.rmin, self.rmax = rmin, rmax
        self.z_offset = z_offset
        self.stabilize = stabilize
        self.frame_id = frame_id
        self.topic = topic
        self.every = every_n_frames            # 60 fps render / 3 = ~20 Hz (as the baseline)
        self.sample_csv = sample_csv
        self.sample_every = sample_every       # every 20th scan ~ 1 Hz of samples
        self.q = None
        self.pub = None
        self.scans = 0
        self.samples = 0
        self.f = None
        self.last_ranges = None
        self.last_origin = None
        self.self_prefix = None
        self._warned = False
        self.self_blocked = 0              # rays (this scan) that exhausted the self-hit skip
        self.self_hit_rays = 0             # rays (this scan) that skipped at least one self hit
        self._self_paths = set()           # distinct self-collider paths seen (logged once each)
        self._cos = [math.cos(self.a_min + self.a_inc * i) for i in range(n_rays)]
        self._sin = [math.sin(self.a_min + self.a_inc * i) for i in range(n_rays)]

    # ------------------------------------------------------------ setup
    def attach(self, app):
        super().attach(app)
        from omni.physx import get_physx_scene_query_interface
        import carb as _carb
        self._carb = _carb
        self.q = get_physx_scene_query_interface()
        self.self_prefix = app.vehicle._stage_prefix          # ignore hits on the drone itself
        if app.ros_node is not None:
            from sensor_msgs.msg import LaserScan
            self._LaserScan = LaserScan
            self.pub = app.ros_node.create_publisher(LaserScan, self.topic, 10)
        if self.sample_csv:
            os.makedirs(RUNS_DIR, exist_ok=True)
            path = os.path.join(RUNS_DIR, f"{app.tag}_scans_{app.seed}_{app.run_id}.csv")
            self.f = open(path, "w", buffering=1)
            self.f.write(f"# seed={app.seed} wall_start={app.wall_start_str} n={self.n} "
                         f"angle_min={self.a_min:.6f} angle_inc={self.a_inc:.6f} "
                         f"rmin={self.rmin} rmax={self.rmax} stabilize={int(self.stabilize)} "
                         f"z_offset={self.z_offset} frames=ENU\n")
            self.f.write("sim_t,x,y,z,yaw_deg," + ",".join(f"r{i}" for i in range(self.n)) + "\n")
            carb.log_warn(f"[{app.tag}] lidar samples -> {path} (every {self.sample_every}th scan)")
        carb.log_warn(f"[{app.tag}] raycast lidar attached: {self.n} rays, {self.rmin}-{self.rmax} m, "
                      f"~{60 // self.every} Hz, plane={'stabilized (yaw only)' if self.stabilize else 'body-fixed'}, "
                      f"origin=body+{self.z_offset:.2f} m, self-skip {self.SELF_SKIP_STEPS}x{self.SELF_SKIP_M} m, "
                      f"self-prefix={self.self_prefix}, topic={self.topic if self.pub else 'none (no ROS)'}")

    # ------------------------------------------------------------ cast
    def _cast(self, ox, oy, oz, dx, dy, dz):
        """Raycast; skip self-hits by advancing the origin past them (the baseline's logic,
        with more and longer steps: the Iris collider is a convex decomposition, so a ray
        crossing the frame meets several solid pieces, each reported at distance ~0)."""
        acc = 0.0
        skipped = False
        for _ in range(self.SELF_SKIP_STEPS):
            hit = self.q.raycast_closest(self._carb.Float3(ox, oy, oz),
                                         self._carb.Float3(dx, dy, dz), self.rmax - acc)
            if not hit["hit"]:
                return self.rmax                        # true miss (nothing along the ray)
            dist = float(hit["distance"])
            path = str(hit.get("collision", ""))
            if not path.startswith(self.self_prefix):
                if skipped:
                    self.self_hit_rays += 1
                return acc + dist                       # real obstacle
            if path not in self._self_paths and len(self._self_paths) < 8:
                self._self_paths.add(path)
                carb.log_warn(f"[{self.app.tag}] lidar self-hit collider: {path} at {acc + dist:.3f} m")
            skipped = True
            step = dist + self.SELF_SKIP_M              # self hit -> step past it
            ox, oy, oz = ox + dx * step, oy + dy * step, oz + dz * step
            acc += step
        self.self_blocked += 1                          # exhausted: the ray never left the vehicle
        return self.rmax

    def scan(self):
        """One full scan from the vehicle's current true pose. Returns (ranges, origin, yaw)."""
        st = self.app.vehicle._state
        p, q = st.position, st.attitude                 # ENU position, FLU-in-ENU quaternion (x,y,z,w)
        ox, oy, oz = float(p[0]), float(p[1]), float(p[2]) + self.z_offset
        x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        if self.stabilize:
            cy, sy = math.cos(yaw), math.sin(yaw)
            R = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))   # yaw only, plane horizontal
        else:
            R = ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
                 (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
                 (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))
        ranges = []
        self.self_blocked = 0
        self.self_hit_rays = 0
        for i in range(self.n):
            lx, ly = self._cos[i], self._sin[i]              # ray in the body/scan frame (z=0)
            dx = R[0][0] * lx + R[0][1] * ly
            dy = R[1][0] * lx + R[1][1] * ly
            dz = R[2][0] * lx + R[2][1] * ly
            r = self._cast(ox, oy, oz, dx, dy, dz)
            ranges.append(r if r > self.rmin else self.rmax)
        return ranges, (ox, oy, oz), yaw

    # ------------------------------------------------------------ loop
    def on_frame(self, sim_t, n):
        if n % self.every:
            return
        try:
            ranges, origin, yaw = self.scan()
        except Exception as exc:  # noqa: BLE001
            if not self._warned:
                self._warned = True
                carb.log_warn(f"[{self.app.tag}] lidar scan failed: {exc}; will keep trying")
            return
        self._warned = False
        self.scans += 1
        self.last_ranges, self.last_origin = ranges, origin
        if self.pub is not None:
            msg = self._LaserScan()
            stamp(msg, sim_t)
            msg.header.frame_id = self.frame_id
            msg.angle_min = float(self.a_min)
            msg.angle_max = float(self.a_min + self.a_inc * (self.n - 1))
            msg.angle_increment = float(self.a_inc)
            msg.range_min = float(self.rmin)
            msg.range_max = float(self.rmax)
            msg.ranges = [float(r) for r in ranges]
            self.pub.publish(msg)
        if self.f is not None and self.scans % self.sample_every == 0:
            self.f.write(f"{sim_t:.3f},{origin[0]:.4f},{origin[1]:.4f},{origin[2]:.4f},"
                         f"{math.degrees(yaw):.2f}," + ",".join(f"{r:.3f}" for r in ranges) + "\n")
            self.samples += 1

    def telemetry(self):
        if self.last_ranges is None:
            return "scan=0"
        hits = [r for r in self.last_ranges if r < self.rmax]
        near = min(hits) if hits else float("nan")
        return (f"scan={self.scans} hits={len(hits)}/{self.n} selfskip={self.self_hit_rays} "
                f"selfblk={self.self_blocked} near={near:.2f}m samples={self.samples}")

    def close(self):
        if self.f is not None:
            try:
                self.f.close()
            except Exception:  # noqa: BLE001
                pass
