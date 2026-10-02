#!/usr/bin/env python3
"""Wheeled arm: ROS-side probe of a running wheeled Isaac run (sim/wheeled_bootstrap.py).

  python3 missions/wheeled_probe.py [--discover 60] [--listen 10] [--drive 0.2]
                                    [--drive-sim-s 5] [--gt <RunDir>/wheeled_gt_*.csv]

1. discover: waits for the first /aeb/sim_time and /scan messages (FastDDS discovery across
   the WSL <-> Windows boundary takes a few seconds)
2. listen: for --listen wall seconds counts /scan, /odom, /tf (odom -> base_link),
   /tf_static (base_link -> lidar_link) and /aeb/sim_time; reports the wall rates, frame ids,
   the scan's returns and nearest range, and the sim-time rate (RTF)
3. drive (--drive V > 0): publishes /cmd_vel linear.x = V at 20 Hz until --drive-sim-s of sim
   time have passed on /aeb/sim_time (wall cap 90 s), then zero for 2 wall-s; reports the
   /odom displacement and heading change and - with --gt - the ground truth's displacement
   over the same sim-time window, and whether the motion went forward along the heading
   (the robot starts at the spawn, facing +x, inside the generator's 2 m clear radius:
   0.2 m/s x 5 sim-s = 1 m stays inside it)
Prints KEY=VALUE lines and one "CHECK <name> PASS|FAIL <detail>" line per check.
Exit 0 = every check passed.
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage


def yaw_of(qz, qw, qx=0.0, qy=0.0):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Probe:
    def __init__(self):
        self.node = rclpy.create_node("aeb_wheeled_probe")
        self.lock = threading.Lock()
        self.reset()
        self.sim_last = None               # (wall, sim)
        self.odom_last = None              # (wall, x, y, yaw)
        self.static_ok = False
        self.scan_last = None
        n = self.node
        n.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        n.create_subscription(Odometry, "/odom", self.on_odom, 20)
        n.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        latched = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        n.create_subscription(TFMessage, "/tf_static", self.on_tf_static, latched)
        n.create_subscription(Float64, "/aeb/sim_time", self.on_sim, 20)
        self.cmd = n.create_publisher(Twist, "/cmd_vel", 10)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(n)
        self.th = threading.Thread(target=self.ex.spin, daemon=True)
        self.th.start()

    def reset(self):
        with self.lock:
            self.c = dict(scan=0, odom=0, tf=0, sim=0)
            self.sim_first = None
            self.scan_frames, self.odom_frames = set(), set()

    def on_scan(self, m):
        finite = [r for r in m.ranges if math.isfinite(r) and m.range_min <= r < m.range_max]
        with self.lock:
            self.c["scan"] += 1
            self.scan_frames.add(m.header.frame_id)
            self.scan_last = (len(m.ranges), len(finite), min(finite) if finite else float("nan"),
                              m.range_max)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        with self.lock:
            self.c["odom"] += 1
            self.odom_frames.add(f"{m.header.frame_id}->{m.child_frame_id}")
            self.odom_last = (time.time(), p.x, p.y, yaw_of(q.z, q.w, q.x, q.y))

    def on_tf(self, m):
        if any(t.header.frame_id == "odom" and t.child_frame_id == "base_link" for t in m.transforms):
            with self.lock:
                self.c["tf"] += 1

    def on_tf_static(self, m):
        if any(t.header.frame_id == "base_link" and t.child_frame_id == "lidar_link" for t in m.transforms):
            self.static_ok = True

    def on_sim(self, m):
        now = time.time()
        with self.lock:
            self.c["sim"] += 1
            if self.sim_first is None:
                self.sim_first = (now, m.data)
            self.sim_last = (now, m.data)

    def sim_now(self):
        with self.lock:
            return self.sim_last[1] if self.sim_last else None

    def twist(self, v):
        t = Twist()
        t.linear.x = float(v)
        self.cmd.publish(t)

    def close(self):
        self.ex.shutdown()
        self.th.join(timeout=3.0)
        self.node.destroy_node()


def gt_rows(path):
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or line.startswith("sim_t"):
                continue
            p = line.strip().split(",")
            if len(p) >= 9:
                try:
                    rows.append([float(v) for v in p[:9]])
                except ValueError:
                    pass
    return rows


def gt_at(rows, t):
    return min(rows, key=lambda r: abs(r[0] - t)) if rows else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--discover", type=float, default=60.0, help="wall s to wait for the first messages")
    ap.add_argument("--listen", type=float, default=10.0, help="wall s of counting")
    ap.add_argument("--drive", type=float, default=0.2, help="m/s forward (0 = no drive)")
    ap.add_argument("--drive-sim-s", type=float, default=5.0, help="sim s of driving")
    ap.add_argument("--gt", default="", help="the run's ground-truth CSV (wheeled_gt_*.csv)")
    a = ap.parse_args()

    rclpy.init()
    pr = Probe()
    fails = 0

    def check(name, ok, detail):
        nonlocal fails
        fails += 0 if ok else 1
        print(f"CHECK {name} {'PASS' if ok else 'FAIL'} {detail}", flush=True)

    # 1. discovery
    t0 = time.time()
    while time.time() - t0 < a.discover:
        with pr.lock:
            ready = pr.c["sim"] > 0 and pr.c["scan"] > 0
        if ready:
            break
        time.sleep(0.5)
    with pr.lock:
        seen = dict(pr.c)
    print(f"DISCOVERY_S={time.time() - t0:.1f} FIRST_COUNTS={seen}", flush=True)
    if not (seen["sim"] and seen["scan"]):
        check("discovery", False, f"no /aeb/sim_time or /scan within {a.discover:g} s (counts {seen}) - "
              "Isaac not publishing, or DDS cannot cross WSL <-> Windows (~/.ros/fastdds.xml peer, firewall)")
        pr.close()
        rclpy.shutdown()
        return 1

    # 2. listen
    pr.reset()
    time.sleep(a.listen)
    with pr.lock:
        c = dict(pr.c)
        s0, s1 = pr.sim_first, pr.sim_last
        scan, sf, of = pr.scan_last, set(pr.scan_frames), set(pr.odom_frames)
    rtf = (s1[1] - s0[1]) / (s1[0] - s0[0]) if s0 and s1 and s1[0] > s0[0] else float("nan")
    rates = {k: v / a.listen for k, v in c.items()}
    print("RATES_WALL_HZ=" + " ".join(f"{k}:{v:.1f}" for k, v in rates.items()), flush=True)
    print(f"RTF={rtf:.3f}", flush=True)
    check("sim_time", c["sim"] >= 3 and rtf == rtf and rtf > 0.02,
          f"{c['sim']} msgs, sim advanced {s1[1] - s0[1]:.2f} s in {s1[0] - s0[0]:.1f} wall-s (RTF {rtf:.3f})")
    ok_scan = c["scan"] >= 3 and sf == {"lidar_link"} and scan and scan[0] == 360 and scan[1] >= 90
    check("scan", bool(ok_scan), f"{c['scan']} msgs ({rates['scan']:.1f} Hz wall), frames {sorted(sf)}, "
          + (f"{scan[0]} rays, {scan[1]} returns, nearest {scan[2]:.2f} m, range_max {scan[3]:g}" if scan else "no scan"))
    check("odom", c["odom"] >= 3 and of == {"odom->base_link"},
          f"{c['odom']} msgs ({rates['odom']:.1f} Hz wall), frames {sorted(of)}")
    check("tf", c["tf"] >= 3, f"{c['tf']} odom->base_link transforms on /tf")
    check("tf_static", pr.static_ok, "base_link->lidar_link on /tf_static" + ("" if pr.static_ok else " NOT seen"))

    # 3. drive
    if a.drive > 0:
        with pr.lock:
            o0 = pr.odom_last
        ts0 = pr.sim_now()
        w0 = time.time()
        while time.time() - w0 < 90.0:
            pr.twist(a.drive)
            now = pr.sim_now()
            if now is not None and now - ts0 >= a.drive_sim_s:
                break
            time.sleep(0.05)
        ts_cmd_end, w_cmd_end = pr.sim_now(), time.time()
        w1 = time.time()
        while time.time() - w1 < 2.0:
            pr.twist(0.0)
            time.sleep(0.05)
        time.sleep(1.0)
        ts1 = pr.sim_now()
        with pr.lock:
            o1 = pr.odom_last
        cmd_sim = ts_cmd_end - ts0
        expect = a.drive * cmd_sim
        print(f"DRIVE_SIM_S={cmd_sim:.2f} DRIVE_WALL_S={w_cmd_end - w0:.1f} EXPECTED_M={expect:.3f} "
              f"WINDOW_SIM={ts0:.2f}..{ts1:.2f}", flush=True)
        if o0 and o1:
            od = math.hypot(o1[1] - o0[1], o1[2] - o0[2])
            dyaw = math.degrees(wrap(o1[3] - o0[3]))
            print(f"ODOM_M={od:.3f} ODOM_DYAW_DEG={dyaw:.1f}", flush=True)
        else:
            od = float("nan")
            print("ODOM_M=nan (no /odom)", flush=True)
        gd = float("nan")
        if a.gt:
            try:
                rows = gt_rows(a.gt)
                g0, g1 = gt_at(rows, ts0), gt_at(rows, ts1)
                gd = math.hypot(g1[2] - g0[2], g1[3] - g0[3])
                yaw0 = yaw_of(g0[7], g0[8], g0[5], g0[6])
                fwd = ((g1[2] - g0[2]) * math.cos(yaw0) + (g1[3] - g0[3]) * math.sin(yaw0))
                print(f"GT_M={gd:.3f} GT_FORWARD_M={fwd:.3f} GT_START=({g0[2]:.2f},{g0[3]:.2f}) yaw {math.degrees(yaw0):.1f} deg "
                      f"GT_END=({g1[2]:.2f},{g1[3]:.2f}) GT_ROWS={len(rows)} (sim {g0[0]:.2f} / {g1[0]:.2f})", flush=True)
                check("drive_gt", expect > 0 and 0.75 * expect <= gd <= 1.30 * expect and fwd > 0.5 * gd,
                      f"ground truth moved {gd:.3f} m ({fwd:+.3f} m along the start heading) for {expect:.3f} m commanded")
            except Exception as exc:  # noqa: BLE001
                check("drive_gt", False, f"ground truth unreadable: {exc}")
        if gd == gd and od == od:
            check("drive_odom", abs(od - gd) <= 0.15 * gd + 0.03,
                  f"odometry {od:.3f} m vs ground truth {gd:.3f} m (noise 3 % per step)")
        elif od == od:
            check("drive_odom", 0.75 * expect <= od <= 1.30 * expect, f"odometry {od:.3f} m for {expect:.3f} m commanded")
    pr.close()
    rclpy.shutdown()
    print(f"PROBE_FAILS={fails}", flush=True)
    return 1 if fails else 0


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush()
    sys.stderr.flush()
    import os
    os._exit(rc)      # rclpy's C++ side can abort at interpreter teardown ("terminate called ...", 2026-09-26)
