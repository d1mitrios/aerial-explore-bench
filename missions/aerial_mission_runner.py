#!/usr/bin/env python3
"""Aerial mission executive: frozen-map goal tours for the quadrotor.

Runs in WSL2 next to PX4 SITL. Mirrors the wheeled baseline's mission runner and its
five fixed protocol rules, with the aerial additions (takeoff, dart, relative setpoints):

  planner   A* on the frozen 2D map (missions/gridmap.py) at the fixed altitude,
            planner radius 0.35 m, plan-once per attempt, no replanning within an attempt
  mover     PX4 OFFBOARD position setpoints over MAVLink (pymavlink, UDP 14540)
  belief    --estimator amcl AMCL (nav2, over the frozen map, from the drone's lidar) over
                             the drone's odometry: the belief is AMCL's map->odom
                             correction applied to the latest odometry pose, the TF
                             semantics map->odom->base_link, continuous between updates.
                             Needs missions/aerial_odom.sh + missions/aerial_amcl.sh.
            --odom-source    lidar (default): rf2o range-flow odometry from /scan
                             (nav_msgs/Odometry on --odom-topic /odom_rf2o); vio: OpenVINS
                             /poseimu (the original source, replaced after the measured scale
                             excursions 0.6-4.4x of a monocular VIO in exploration flight)
            --estimator vio  the raw odometry anchored to the map frame at the spawn
                             (dead reckoning; in testing it could not pass the doorways)
            --estimator px4  PX4's own local estimate (integration test only)
            Setpoints are RELATIVE in every mode: sp_px4 = p_px4 + (carrot_map -
            p_belief_map), so estimator error physically displaces the drone
  claim     |p_est - goal|_xy <= 0.35 m, yaw free, serial goals, 240 s per-goal timeout,
            retry-once, attempt column; claim uses the belief, never truth
  safety    live /scan: hold if an obstacle is closer than --stop-dist in the travel
            direction; the attempt is ABORTED after --blocked-s of holding
  doorways  inside passages narrower than 1.0 m (raw-map free width across the path)
            the carrot shortens to 0.3 m (a slow, settled pass) and the setpoint is shifted
            to the centre the lidar measures between the two frames; the yaw always follows
            the path tangent (crab: the 0.52 m side across the corridor)
  takeoff   parked and DISARMED for the static window, then arm + climb in one move with an
            abrupt ramp (MPC_TKO_RAMP_T 0.5 s): the liftoff is the jerk the VIO static
            initializer waits for (a slow ramp or an armed-idle wait smears it into the
            0.07-0.11 band that overlaps the parked noise; measured, 2026-09-06)
  dart      right after takeoff: 1.5 m out-and-back along +x, then the odometry gate (10 s:
            poses streaming; the VIO's initialization, or rf2o's first scans); on failure:
            land, sit still, take off again, once; else ODOM_INIT_FAIL; dart and
            gate time are outside the policy clock. With lidar odometry the dart still
            gives AMCL its first ~20 updates before the policy clock starts
  amcl gate after the dart (amcl mode): AMCL must hold a converged pose of the hovering
            drone (correction known, covariance < --amcl-cov-max, last pose within 0.5 m of
            odometry); normally already true, the dart fed it ~20 updates; otherwise a
            hover wiggle for up to --amcl-gate-s sim seconds, else AMCL_INIT_FAIL

  vehicle   the same executive drives both bodies. --vehicle air (default) is the
            quadrotor above; --vehicle ground is the wheeled robot: GroundLink stands in
            Px4Link's place (odometry pose in, /cmd_vel out through a point controller, the
            simulation clock from /aeb/sim_time), the body's own values replace the
            quadrotor's for --alt, --radius, --yaw (nose-first), --speed (0.30 m/s),
            --min-passage and --yaw-clear (VEHICLE_DEFAULTS), everything else (planner,
            carrot, doorway strategy, guards, claim rule, AMCL over rf2o) is the same code.
            The takeoff, the dart and the landing run as bookkeeping on the ground; the IMU
            impact detector has no IMU (contact is read from the ground truth).

Ground truth is NOT read here: the verdict comes from the Isaac-side GT CSV, joined
offline (both clocks are logged on both sides).

Outputs (runs/raw): missions_<seed>_<ts>.csv (one row per attempt, baseline schema plus
sim clocks and the claim position), manifest_<seed>_<ts>.json (every parameter, the anchor
transform, dart and gate timings, tour summary), belief_<seed>_<ts>.csv (the belief the
executive consumed, every mode), odom_<seed>_<ts>.csv (the raw odometry consumed: rf2o or
VIO, odom and map frames), amcl_<seed>_<ts>.csv (AMCL poses + the implied correction),
imu_excitation_<seed>_<ts>.csv.

Usage (ROS 2 Jazzy sourced; PX4 SITL running; the Isaac app flying the world; for amcl
also aerial_odom.sh <seed> and aerial_amcl.sh <seed> [map] running):
  python3 missions/aerial_mission_runner.py --seed 20260723008 --estimator amcl --tour-limit 3
  python3 missions/aerial_mission_runner.py --seed 20260723008 --estimator px4 --tour-limit 3
"""
import argparse
import csv
import json
import math
import os
import struct
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gridmap import GridMap  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def heading_from_quat(x, y, z, w):
    """Heading (rad) of the body x-axis projected on the horizontal plane."""
    r00 = 1 - 2 * (y * y + z * z)
    r10 = 2 * (x * y + w * z)
    return math.atan2(r10, r00)


# ============================================================== PX4 link
class Px4Link:
    """pymavlink connection to PX4 SITL with a reader thread and ENU accessors."""
    OFFBOARD_MAIN = 6
    AUTO_MAIN, AUTO_LAND_SUB = 4, 6

    def __init__(self, url, log):
        os.environ.setdefault("MAVLINK20", "1")           # PX4 speaks MAVLink 2
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.log = log
        self.m = mavutil.mavlink_connection(url, source_system=245, source_component=190)
        log(f"[px4] waiting for heartbeat on {url} ...")
        hb = self.m.wait_heartbeat(timeout=60)
        if hb is None:
            raise SystemExit("[px4] no heartbeat; is PX4 SITL running and connected to Isaac?")
        self.ts, self.tc = self.m.target_system, self.m.target_component
        log(f"[px4] heartbeat from system {self.ts} component {self.tc}")
        self.lock = threading.Lock()
        self.pos_ned = None            # (x_n, y_e, z_d)
        self.vel_ned = None
        self.t_boot = None             # PX4 clock (lockstep = sim time), seconds
        self.yaw_ned = None
        self.armed = False
        self.main_mode = None
        self.landed_state = None
        self.acks = []
        self.alive = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
                           (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20),
                           (mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2)):
            self.m.mav.command_long_send(self.ts, self.tc, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                         0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def _reader(self):
        while self.alive:
            msg = self.m.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            t = msg.get_type()
            with self.lock:
                if t == "LOCAL_POSITION_NED":
                    self.pos_ned = (msg.x, msg.y, msg.z)
                    self.vel_ned = (msg.vx, msg.vy, msg.vz)
                    self.t_boot = msg.time_boot_ms / 1000.0
                elif t == "ATTITUDE":
                    self.yaw_ned = msg.yaw
                elif t == "HEARTBEAT" and msg.get_srcSystem() == self.ts:
                    self.armed = bool(msg.base_mode & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.main_mode = (msg.custom_mode >> 16) & 0xFF
                elif t == "EXTENDED_SYS_STATE":
                    self.landed_state = msg.landed_state
                elif t == "COMMAND_ACK":
                    self.acks.append((msg.command, msg.result))
                    if len(self.acks) > 50:
                        self.acks = self.acks[-50:]

    # --------------------------------------------------------- accessors
    def enu(self):
        """(x_e, y_n, z_u, yaw_enu, t_boot) or None."""
        with self.lock:
            if self.pos_ned is None or self.yaw_ned is None:
                return None
            xn, ye, zd = self.pos_ned
            return (ye, xn, -zd, wrap(math.pi / 2 - self.yaw_ned), self.t_boot)

    def speed(self):
        with self.lock:
            return math.hypot(self.vel_ned[0], self.vel_ned[1]) if self.vel_ned else 0.0

    # ---------------------------------------------------------- commands
    def send_setpoint_enu(self, x_e, y_n, z_u, yaw_enu):
        mask = 0b100111111000            # use position + yaw; ignore velocity, accel, yaw rate
        t_ms = int((self.t_boot or 0.0) * 1000)
        self.m.mav.set_position_target_local_ned_send(
            t_ms, self.ts, self.tc, self.mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            y_n, x_e, -z_u, 0, 0, 0, 0, 0, 0, wrap(math.pi / 2 - yaw_enu), 0)

    def set_mode(self, main, sub=0):
        self.m.mav.command_long_send(self.ts, self.tc, self.mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                     self.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub,
                                     0, 0, 0, 0)

    def arm(self, arm=True, force=False):
        self.m.mav.command_long_send(self.ts, self.tc, self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                     1 if arm else 0, 21196 if force else 0, 0, 0, 0, 0, 0)

    def set_param(self, name, value, integer=False):
        mt = self.mavutil.mavlink
        if integer:
            packed = struct.unpack("f", struct.pack("i", int(value)))[0]
            self.m.mav.param_set_send(self.ts, self.tc, name.encode(), packed, mt.MAV_PARAM_TYPE_INT32)
        else:
            self.m.mav.param_set_send(self.ts, self.tc, name.encode(), float(value), mt.MAV_PARAM_TYPE_REAL32)

    def close(self):
        self.alive = False


class GroundLink:
    """The wheeled robot in Px4Link's role (one executive for both vehicles). Same
    accessors and commands, so the executive's phases run unchanged:

      enu()              the latest odometry pose (rf2o, odom frame) + the height last commanded
                         (a ground vehicle is always at its "altitude") + the simulation clock
                         (/aeb/sim_time from the Isaac side, the lockstep clock's counterpart)
      send_setpoint_enu  a position target in the odom frame, turned into /cmd_vel by a point
                         controller every call (the executive calls it at --rate): forward when
                         the target is within --ground-turn rad of the heading, in reverse when
                         it is behind (the retreat and the dart's return leg keep the body's
                         orientation, as the drone's frozen yaw does), a turn in place between.
                         Commands never fall below --ground-v-min / --ground-w-min while moving:
                         the simulated drive does not execute turns in place slower than about
                         0.2 rad/s (measured 2026-10-01), and a controller that commands them
                         waits for ever. The yaw setpoint is ignored: a differential drive faces
                         where it goes
      set_param          MPC_XY_VEL_MAX / MPC_XY_CRUISE set the speed cap (the executive's --speed)
      set_mode / arm     the OFFBOARD / arm sequence is bookkeeping; AUTO.LAND stops the vehicle
                         (landed_state 1), as the drone's landing ends its commands
    A watchdog zeroes /cmd_vel when no target arrives for 0.5 wall-s. Without a ROS node
    (the mock harness) the subclass feeds pose_update() and overrides _publish()."""
    OFFBOARD_MAIN = 6
    AUTO_MAIN, AUTO_LAND_SUB = 4, 6

    def __init__(self, a, log, ros=None, odom_topic="/odom_rf2o", cmd_topic="/cmd_vel", clock_topic="/aeb/sim_time"):
        self.log = log
        self.vmax = float(getattr(a, "speed", 0.3))
        self.wmax = float(getattr(a, "ground_w_max", 1.0))
        self.v_min = float(getattr(a, "ground_v_min", 0.10))
        self.w_min = float(getattr(a, "ground_w_min", 0.5))
        self.turn = float(getattr(a, "ground_turn", 0.8))
        self.kv, self.kw = 0.6, 2.0
        self.lock = threading.Lock()
        self.pose = None               # (x, y, yaw, t_stamp)
        self.hist = []                 # (t_sim, x, y) for speed()
        self.t_sim = None
        self.z = 0.0
        self.armed = False
        self.main_mode = None
        self.landed_state = 1
        self.acks = []
        self.alive = True
        self.last_send = 0.0
        self.last_cmd = (0.0, 0.0)
        self.node = getattr(ros, "node", None) if ros is not None else None
        if self.node is not None:
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from std_msgs.msg import Float64
            self.Twist = Twist
            self.pub = self.node.create_publisher(Twist, cmd_topic, 10)
            self.node.create_subscription(Odometry, odom_topic, self._on_odom, 10)
            self.node.create_subscription(Float64, clock_topic, self._on_clock, 10)
            self.node.create_timer(0.2, self._watchdog)
            log(f"[ground] /cmd_vel from {odom_topic} targets; clock {clock_topic}; v {self.v_min}-{self.vmax} m/s, "
                f"w {self.w_min}-{self.wmax} rad/s, turn in place beyond {self.turn:.2f} rad")

    # --------------------------------------------------------- inputs
    def _on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pose_update(p.x, p.y, heading_from_quat(q.x, q.y, q.z, q.w),
                         m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

    def _on_clock(self, m):
        with self.lock:
            self.t_sim = float(m.data)

    def pose_update(self, x, y, yaw, t_stamp, t_sim=None):
        with self.lock:
            self.pose = (x, y, yaw, t_stamp)
            if t_sim is not None:
                self.t_sim = t_sim
            t = self.t_sim if self.t_sim is not None else t_stamp
            self.hist.append((t, x, y))
            if len(self.hist) > 60:
                del self.hist[:20]

    def _watchdog(self):
        if self.alive and self.last_send and time.time() - self.last_send > 0.5 and self.last_cmd != (0.0, 0.0):
            self._publish(0.0, 0.0)

    # ------------------------------------------------------ accessors
    def enu(self):
        """(x, y, z_commanded, yaw, t_sim) in the odom frame, or None before the first pose."""
        with self.lock:
            if self.pose is None or self.t_sim is None:
                return None
            x, y, yaw, _ = self.pose
            return (x, y, self.z, wrap(yaw), self.t_sim)

    def speed(self):
        """Ground speed over the last ~0.3 s of simulation time, from the odometry poses."""
        with self.lock:
            h = self.hist
            if len(h) < 2:
                return 0.0
            t1, x1, y1 = h[-1]
            for t0, x0, y0 in reversed(h[:-1]):
                if t1 - t0 >= 0.3:
                    break
            dt = t1 - t0
            return math.hypot(x1 - x0, y1 - y0) / dt if dt > 1e-3 else 0.0

    # ------------------------------------------------------- commands
    def control(self, sx, sy):
        """(v, w) of a differential drive toward the target (odom frame); (0, 0) at the target."""
        with self.lock:
            pose = self.pose
        if pose is None:
            return 0.0, 0.0
        x, y, yaw, _ = pose
        dx, dy = sx - x, sy - y
        d = math.hypot(dx, dy)
        if d < 0.06:
            return 0.0, 0.0
        e = wrap(math.atan2(dy, dx) - yaw)
        if abs(e) <= self.turn:                                   # ahead: drive forward, steer
            v = max(self.v_min, min(self.vmax, self.kv * d * max(0.3, math.cos(e))))
            w = max(-self.wmax, min(self.wmax, self.kw * e))
            return v, w
        if abs(e) >= math.pi - self.turn:                        # behind: reverse, body orientation kept
            er = wrap(e - math.pi)                                 # heading error of the reversing motion
            v = -max(self.v_min, min(self.vmax, self.kv * d * max(0.3, math.cos(er))))
            w = max(-self.wmax, min(self.wmax, -self.kw * er))
            return v, w
        w = math.copysign(max(self.w_min, min(self.wmax, self.kw * abs(e))), e)
        return 0.0, w                                              # turn in place toward the target

    def send_setpoint_enu(self, x_e, y_n, z_u, yaw_enu):
        self.z = z_u
        v, w = self.control(x_e, y_n) if (self.armed and self.landed_state != 1) else (0.0, 0.0)
        self.last_send = time.time()
        self._publish(v, w)

    def _publish(self, v, w):
        self.last_cmd = (v, w)
        if self.node is not None:
            m = self.Twist()
            m.linear.x = float(v)
            m.angular.z = float(w)
            self.pub.publish(m)

    def set_mode(self, main, sub=0):
        self.main_mode = main
        if main == self.AUTO_MAIN:                                 # "land": stop and stay stopped
            self.landed_state = 1
            self._publish(0.0, 0.0)
        elif main == self.OFFBOARD_MAIN and self.armed:
            self.landed_state = 2

    def arm(self, arm=True, force=False):
        self.armed = arm
        if arm and self.main_mode == self.OFFBOARD_MAIN:
            self.landed_state = 2                                  # "in the air": commands are executed
        if not arm:
            self.landed_state = 1
            self._publish(0.0, 0.0)

    def set_param(self, name, value, integer=False):
        if name in ("MPC_XY_VEL_MAX", "MPC_XY_CRUISE"):
            self.vmax = float(value)

    def close(self):
        self.alive = False
        self._publish(0.0, 0.0)


def install_stop_signals():
    """SIGINT and SIGTERM both end a run the way Ctrl+C does (KeyboardInterrupt: land, write
    the manifest as INTERRUPTED). A batch runner starts us in the background, where the shell
    leaves SIGINT ignored and SIGTERM would kill us without landing or a manifest. Installed
    before rclpy.init, whose own handlers chain to these."""
    import signal

    def _stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, _stop)


# ============================================================== ROS I/O
class RosIO:
    """The odometry (rf2o /odom_rf2o, or OpenVINS /poseimu), /scan (Isaac lidar), the IMU and
    the localizer's pose, spun in a thread. The odometry is kept under the historical name
    `vio` inside (the VIO was the source until the switch to lidar odometry, 2026-09-24): (x, y, z, yaw, t_stamp, wall_rx)."""

    def __init__(self, log, vio_topic="/poseimu", scan_topic="/scan", imu_topic="/a6/imu",
                 amcl_topic="/amcl_pose", vio_log=None, imu_log=None, amcl_log=None, imu_window_s=2.0,
                 map_topic=None, odom_topic=None):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from sensor_msgs.msg import Imu, LaserScan
        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("aerial_mission_runner")
        self.lock = threading.Lock()
        self.vio = None                 # (x, y, z, yaw, t_stamp, wall_rx)
        self.vio_rx_sim = None          # PX4 clock when the latest VIO pose arrived (set once sim_clock exists)
        self.sim_clock = None           # callable -> PX4 sim time, attached by the executive
        self.vio_count = 0
        self.vio_hist = []              # recent (t, x, y, yaw) for the AMCL correction lookup
        self.scan = None                # (angle_min, angle_inc, ranges np.array, wall_rx)
        self.anchor_T = None            # set by Belief.anchor(); used to log the map-frame pose
        self.amcl = None                # (x, y, yaw, t_stamp, wall_rx, cov_xx, cov_yy, cov_tt)
        self.amcl_count = 0
        self.amcl_odom = None           # odometry (VIO) pose AMCL corrected last: (x, y, yaw)
        self.odom_source = "lidar" if odom_topic else "vio"
        self.vio_f = open(vio_log, "w", buffering=1) if vio_log else None
        if self.vio_f:
            src = f"rf2o {odom_topic}" if odom_topic else f"OpenVINS {vio_topic}"
            self.vio_f.write(f"# raw odometry ({src}) as received by the executive, odom frame; map_* valid after the anchor\n")
            self.vio_f.write("t_odom,wall,odom_x,odom_y,odom_z,odom_yaw,map_x,map_y,map_yaw\n")
        self.amcl_f = open(amcl_log, "w", buffering=1) if amcl_log else None
        if self.amcl_f:
            self.amcl_f.write("# AMCL /amcl_pose (map frame) + the odom pose it corrected + the implied map->odom correction\n")
            self.amcl_f.write("t,wall,amcl_x,amcl_y,amcl_yaw,cov_xx,cov_yy,cov_yawyaw,odom_x,odom_y,odom_yaw,corr_theta,corr_tx,corr_ty\n")
        # IMU excitation diagnostic: the static initializer's own metric, the sample standard
        # deviation of the accelerometer vector over a window of init_window_time (2 s),
        # logged twice per window so hover / dart / parked levels can be read against
        # init_imu_thresh (0.08) instead of guessed.
        self.imu_window_s = imu_window_s
        self.imu_buf = []               # (t, ax, ay, az)
        self.imu_std = None             # (t_last, std, n)
        self.imu_f = open(imu_log, "w", buffering=1) if imu_log else None
        self.imu_next_log = None
        if self.imu_f:
            self.imu_f.write(f"# accelerometer std over {imu_window_s:.1f} s windows (OpenVINS static-init metric)\n")
            self.imu_f.write("t_imu,wall,acc_std,n\n")
        if odom_topic:
            from nav_msgs.msg import Odometry
            self.node.create_subscription(Odometry, odom_topic, self._on_odom, 10)          # rf2o publishes reliable, depth 5
        else:
            self.node.create_subscription(PoseWithCovarianceStamped, vio_topic, self._on_vio, qos_profile_sensor_data)
        self.node.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.node.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)
        self.node.create_subscription(PoseWithCovarianceStamped, amcl_topic, self._on_amcl, 10)   # AMCL / slam_toolbox pose: reliable
        self.map_msg, self.map_count = None, 0
        if map_topic:
            from nav_msgs.msg import OccupancyGrid
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.node.create_subscription(OccupancyGrid, map_topic, self._on_map, qos)      # the mapper's latched map
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        log(f"[ros] subscribed {odom_topic or vio_topic} (odometry: {self.odom_source}), {scan_topic}, {imu_topic}, {amcl_topic}")

    def _on_amcl(self, m):
        """AMCL's corrected pose. The correction map->odom is derived against the odometry
        pose at the same stamp (the TF semantics map->odom->base_link, without a TF listener),
        so the belief can be propagated with every VIO pose between AMCL updates."""
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        yaw = heading_from_quat(q.x, q.y, q.z, q.w)
        cov = m.pose.covariance
        with self.lock:
            od = self._odom_at(t)
            if od is not None:
                ox, oy, oyaw = od
                theta = wrap(yaw - oyaw)
                c, s = math.cos(theta), math.sin(theta)
                tx = p.x - (c * ox - s * oy)
                ty = p.y - (s * ox + c * oy)
                self.correction = (theta, tx, ty)
                self.amcl_odom = od
            elif self.vio is not None:
                self.amcl_odom = (self.vio[0], self.vio[1], self.vio[3])
            self.amcl = (p.x, p.y, yaw, t, time.time(), cov[0], cov[7], cov[35])
            self.amcl_count += 1
            corr = getattr(self, "correction", None)
        if self.amcl_f:
            od_s = f"{od[0]:.4f},{od[1]:.4f},{od[2]:.4f}" if od is not None else "nan,nan,nan"
            corr_s = f"{corr[0]:.5f},{corr[1]:.4f},{corr[2]:.4f}" if corr else "nan,nan,nan"
            self.amcl_f.write(f"{t:.4f},{time.time():.3f},{p.x:.4f},{p.y:.4f},{yaw:.4f},"
                              f"{cov[0]:.5f},{cov[7]:.5f},{cov[35]:.5f},{od_s},{corr_s}\n")

    def _odom_at(self, t):
        """Odometry (VIO) pose nearest to stamp t from the recent history (lock held)."""
        if not self.vio_hist:
            return None
        best = min(self.vio_hist, key=lambda h: abs(h[0] - t))
        if abs(best[0] - t) > 0.5:
            return None
        return best[1], best[2], best[3]

    def latest_amcl(self):
        with self.lock:
            return self.amcl, self.amcl_count, getattr(self, "correction", None)

    def odom_since_amcl(self):
        """Displacement of the latest VIO pose from the one AMCL corrected last (m), or None.
        AMCL's own update trigger is this displacement against update_min_d, so a stack that
        has moved far beyond it without a new /amcl_pose is a localizer that stopped."""
        with self.lock:
            if self.amcl_odom is None or self.vio is None:
                return None
            return math.hypot(self.vio[0] - self.amcl_odom[0], self.vio[1] - self.amcl_odom[1])

    def _on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        a = m.linear_acceleration
        with self.lock:
            self.imu_buf.append((t, a.x, a.y, a.z))
            while self.imu_buf and t - self.imu_buf[0][0] > self.imu_window_s:
                self.imu_buf.pop(0)
            n = len(self.imu_buf)
            if n >= 10:
                arr = np.array([b[1:] for b in self.imu_buf])
                dev = arr - arr.mean(axis=0)
                std = math.sqrt(float((dev * dev).sum()) / (n - 1))
                self.imu_std = (t, std, n)
                if self.imu_f and (self.imu_next_log is None or t >= self.imu_next_log):
                    self.imu_f.write(f"{t:.3f},{time.time():.3f},{std:.4f},{n}\n")
                    self.imu_next_log = t + self.imu_window_s / 2

    def imu_excitation(self):
        with self.lock:
            return self.imu_std

    def _on_odom(self, m):
        """nav_msgs/Odometry (rf2o): the same record as a VIO pose."""
        self._on_vio(m)

    def _on_vio(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        yaw = heading_from_quat(q.x, q.y, q.z, q.w)
        with self.lock:
            self.vio = (p.x, p.y, p.z, yaw, t, time.time())
            self.vio_rx_sim = self.sim_clock() if self.sim_clock else None
            self.vio_count += 1
            self.vio_hist.append((t, p.x, p.y, yaw))
            if len(self.vio_hist) > 400:                 # ~20-40 s of history
                del self.vio_hist[:100]
            T = self.anchor_T
        if self.vio_f:
            if T:
                th, tx, ty = T
                c, s = math.cos(th), math.sin(th)
                mx, my, myaw = c * p.x - s * p.y + tx, s * p.x + c * p.y + ty, wrap(yaw + th)
                tail = f"{mx:.4f},{my:.4f},{myaw:.4f}"
            else:
                tail = "nan,nan,nan"
            self.vio_f.write(f"{t:.4f},{time.time():.3f},{p.x:.4f},{p.y:.4f},{p.z:.4f},{yaw:.4f},{tail}\n")

    def _on_scan(self, m):
        with self.lock:
            self.scan = (m.angle_min, m.angle_increment, np.array(m.ranges, dtype=float), time.time())

    def _on_map(self, m):
        with self.lock:
            self.map_msg = m
            self.map_count += 1

    def latest_map(self):
        """(OccupancyGrid message, count) of the mapper's latest map, or (None, 0)."""
        with self.lock:
            return self.map_msg, self.map_count

    def latest_vio(self):
        with self.lock:
            return self.vio, self.vio_count

    def min_range_towards(self, rel_angle, half_width=math.radians(30)):
        """Minimum lidar range within +-half_width of rel_angle (rad, lidar frame). None if no scan."""
        with self.lock:
            s = self.scan
        if s is None or time.time() - s[3] > 2.0:
            return None
        a_min, inc, ranges, _ = s
        angles = a_min + inc * np.arange(len(ranges))
        sel = np.abs(wrap_arr(angles - rel_angle)) <= half_width
        r = ranges[sel]
        r = r[(r > 0.05) & (r < 50.0)]
        return float(r.min()) if len(r) else None

    def sector_returns(self, a_from, a_to, max_range):
        """Lidar returns (ranges, angles in the lidar frame) within the sector [a_from, a_to]
        (rad, relative to the lidar x-axis, a_from < a_to, width < pi) closer than max_range.
        None if no scan."""
        with self.lock:
            s = self.scan
        if s is None or time.time() - s[3] > 2.0:
            return None
        a_min, inc, ranges, _ = s
        angles = a_min + inc * np.arange(len(ranges))
        mid, half = (a_from + a_to) / 2.0, (a_to - a_from) / 2.0
        sel = (np.abs(wrap_arr(angles - mid)) <= half) & (ranges > 0.05) & (ranges < max_range)
        return ranges[sel], wrap_arr(angles[sel] - mid) + mid

    def min_range_all(self):
        """Minimum lidar range in any direction (imminent-contact guard). None if no scan."""
        n = self.nearest_return()
        return None if n is None else n[0]

    def nearest_return(self):
        """(range, angle in the lidar frame) of the nearest lidar return. None if no scan."""
        with self.lock:
            s = self.scan
        if s is None or time.time() - s[3] > 2.0:
            return None
        a_min, inc, ranges, _ = s
        ok = (ranges > 0.05) & (ranges < 50.0)
        if not ok.any():
            return None
        k = int(np.argmin(np.where(ok, ranges, np.inf)))
        return float(ranges[k]), float(a_min + inc * k)

    def close(self):
        try:
            for f in (self.vio_f, self.imu_f, self.amcl_f):
                if f:
                    f.close()
            self.executor.shutdown(timeout_sec=1.0)
            self.thread.join(timeout=2.0)
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


def wrap_arr(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ============================================================== belief
class Belief:
    """The estimate the controller consumes and the claim uses.
    px4:   PX4's own estimate (integration tests only)
    vio:   OpenVINS pose anchored to the map frame at the spawn (dead reckoning)
    amcl:  AMCL's map->odom correction applied to the latest VIO pose: the TF
           semantics map->odom->base_link, continuous between AMCL updates"""

    def __init__(self, mode, px4, ros, belief_log=None):
        self.mode = mode
        self.px4, self.ros = px4, ros
        self.anchor_T = None            # (theta, tx, ty): p_map = R(theta) p_vio + t
        self.f = open(belief_log, "w", buffering=1) if belief_log else None
        if self.f:
            self.f.write(f"# the belief the executive consumed (mode={mode}), map frame, sampled at the control rate\n")
            self.f.write("t,wall,x,y,yaw\n")
        self._last_logged_t = None

    def anchor(self):
        """Align the VIO frame with the map frame at the spawn (initial-pose convention)."""
        e = self.px4.enu()
        v, _ = self.ros.latest_vio()
        if e is None or v is None:
            return None
        theta = wrap(e[3] - v[3])
        c, s = math.cos(theta), math.sin(theta)
        tx = e[0] - (c * v[0] - s * v[1])
        ty = e[1] - (s * v[0] + c * v[1])
        self.anchor_T = (theta, tx, ty)
        with self.ros.lock:
            self.ros.anchor_T = self.anchor_T
        return dict(theta=theta, tx=tx, ty=ty, px4=list(e[:4]), vio=list(v[:4]), t_vio=v[4], t_px4=e[4])

    def pose(self):
        """(x, y, yaw, t_sim) in the map frame, or None."""
        out = self._pose()
        if out is not None and self.f and out[3] != self._last_logged_t:
            self._last_logged_t = out[3]
            self.f.write(f"{out[3]:.4f},{time.time():.3f},{out[0]:.4f},{out[1]:.4f},{out[2]:.4f}\n")
        return out

    def _pose(self):
        if self.mode == "px4":
            e = self.px4.enu()
            return None if e is None else (e[0], e[1], e[3], e[4])
        v, _ = self.ros.latest_vio()
        if v is None:
            return None
        if self.mode == "amcl":
            amcl, _n, corr = self.ros.latest_amcl()
            if amcl is None:
                return None
            if corr is None:                              # no odom at the AMCL stamp yet: use AMCL's own pose
                return (amcl[0], amcl[1], amcl[2], amcl[3])
            theta, tx, ty = corr
        else:
            if self.anchor_T is None:
                return None
            theta, tx, ty = self.anchor_T
        c, s = math.cos(theta), math.sin(theta)
        return (c * v[0] - s * v[1] + tx, s * v[0] + c * v[1] + ty, wrap(v[3] + theta), v[4])

    def vio_fresh(self, max_sim_s=1.5, max_wall_s=90.0):
        """The VIO is alive if the SIMULATION has not advanced more than max_sim_s since its
        last pose (a wall-clock criterion aborted an early test flight on a 2.1 s stall of the pipeline;
        1-1.6 s wall gaps are routine at RTF 0.15, and a paused simulation is no loss at all);
        the wall backstop only catches a dead pipeline while the simulation is dead too (90 s:
        the simulator itself stalls 15-20 s at a time when its hub cannot be reached)."""
        v, _ = self.ros.latest_vio()
        if v is None or time.time() - v[5] > max_wall_s:
            return False
        with self.ros.lock:
            rx = self.ros.vio_rx_sim
        if rx is None or self.ros.sim_clock is None:
            return time.time() - v[5] <= 2.0 * max_sim_s / 0.15          # no sim clock: wall, RTF 0.15 assumed
        return self.ros.sim_clock() - rx <= max_sim_s

    def amcl_fresh(self, max_m=3.0):
        """AMCL updates only after update_min_d (0.15 m) of odometry motion, never while the
        vehicle holds, so freshness is measured in odometry displacement since the last
        /amcl_pose, not in time: beyond max_m the localizer has missed >= 20 updates."""
        d = self.ros.odom_since_amcl()
        return d is not None and d <= max_m

    def close(self):
        if self.f:
            try:
                self.f.close()
            except Exception:  # noqa: BLE001
                pass


# ============================================================== executive
class Executive:
    def __init__(self, a, log):
        self.a, self.log = a, log
        self.seed = a.seed
        # a frozen map + a goal file (missions), or neither (exploration: the live map arrives
        # from the mapper, the goals are frontiers)
        self.gm = GridMap(a.map) if a.map else GridMap(cls=np.full((400, 400), GridMap.UNK, dtype=np.int8),
                                                       resolution=0.05, origin=(-10.0, -10.0))
        self.goals = self._read_goals(a.goals) if a.goals else []
        if a.tour_limit:
            self.goals = self.goals[:a.tour_limit]
        self.run_id = getattr(a, "run_id", None) or time.strftime("%Y%m%d_%H%M%S")
        os.makedirs(a.out_dir, exist_ok=True)
        self.vehicle = getattr(a, "vehicle", "air")
        uses_vio = a.estimator in ("vio", "amcl")
        self.odom_source = getattr(a, "odom_source", "lidar")
        # `px4` is the executive's name for the vehicle link whichever vehicle it drives:
        # the quadrotor over MAVLink, or the wheeled robot over /cmd_vel from the same odometry
        if self.vehicle == "air":
            self.px4 = Px4Link(a.mav, log)
        self.ros = RosIO(log, vio_log=os.path.join(a.out_dir, f"odom_{self.seed}_{self.run_id}.csv") if uses_vio else None,
                         imu_log=os.path.join(a.out_dir, f"imu_excitation_{self.seed}_{self.run_id}.csv"),
                         amcl_log=os.path.join(a.out_dir, f"amcl_{self.seed}_{self.run_id}.csv") if a.estimator == "amcl" else None,
                         amcl_topic=getattr(a, "pose_topic", "/amcl_pose"), map_topic=getattr(a, "map_topic", None),
                         vio_topic=getattr(a, "vio_topic", "/poseimu"),
                         odom_topic=(getattr(a, "odom_topic", "/odom_rf2o") if self.odom_source == "lidar" else None))
        if self.vehicle == "ground":
            self.px4 = GroundLink(a, log, self.ros, odom_topic=getattr(a, "odom_topic", "/odom_rf2o"))
        self.ros.sim_clock = lambda: (self.px4.enu() or (0, 0, 0, 0, 0.0))[4]
        self.belief = Belief(a.estimator, self.px4, self.ros,
                             belief_log=os.path.join(a.out_dir, f"belief_{self.seed}_{self.run_id}.csv"))
        self.csv_path = os.path.join(a.out_dir, f"missions_{self.seed}_{self.run_id}.csv")
        self.manifest_path = os.path.join(a.out_dir, f"manifest_{self.seed}_{self.run_id}.json")
        self.manifest = dict(seed=self.seed, run_id=self.run_id, wall_start=time.strftime("%Y-%m-%d %H:%M:%S"),
                             params=vars(a), map=os.path.relpath(a.map, ROOT) if a.map else None,
                             goals=os.path.relpath(a.goals, ROOT) if a.goals else None, events=[], results=[])
        self.sp_z = 0.0
        self.sp_xy = None
        self.sp_yaw = 0.0
        self.tick_dt = 1.0 / a.rate
        self.impacts = 0

    @staticmethod
    def _read_goals(path):
        rows = [r for r in csv.DictReader(l for l in open(path) if not l.startswith("#"))]
        return [(r["mission"], float(r["x"]), float(r["y"]), r.get("room", "")) for r in rows]

    def event(self, name, **kw):
        e = self.px4.enu()
        rec = dict(event=name, wall=time.time(), t_px4=(e[4] if e else None), **kw)
        self.manifest["events"].append(rec)
        self.log(f"[exec] {name} " + " ".join(f"{k}={v}" for k, v in kw.items()))

    # ------------------------------------------------------- primitives
    def clock(self):
        """Seconds on the chosen timeout clock."""
        if self.a.timeout_clock == "wall":
            return time.time()
        e = self.px4.enu()
        return e[4] if e else 0.0

    def stream(self, duration_s, xy_enu=None, z_u=None, yaw=None):
        """Send the current setpoint at --rate for duration_s wall seconds."""
        end = time.time() + duration_s
        while time.time() < end:
            self.send(xy_enu, z_u, yaw)
            time.sleep(self.tick_dt)

    def send(self, xy_enu=None, z_u=None, yaw=None):
        if xy_enu is not None:
            self.sp_xy = xy_enu
        if z_u is not None:
            self.sp_z = z_u
        if yaw is not None:
            # yaw slew limit (--yaw-rate, deg/s of SIM time): a 195 deg/s turn at a goal flipped
            # the mono VIO's scale from 1.3 to 0.6 (in a test flight); the features it tracks sweep out
            # of the sideways camera; the wheeled baseline turns at <= 1 rad/s too
            t = self.sim_t()
            dt = t - self._yaw_t if getattr(self, "_yaw_t", None) is not None else 0.0
            self._yaw_t = t
            step = math.radians(self.a.yaw_rate) * max(0.0, min(dt, 0.5))
            d = wrap(yaw - self.sp_yaw)
            self.sp_yaw = yaw if self.a.yaw_rate <= 0 or abs(d) <= step else wrap(self.sp_yaw + math.copysign(step, d))
        if self.sp_xy is None:
            e = self.px4.enu()
            self.sp_xy = (e[0], e[1]) if e else (0.0, 0.0)
        self.px4.send_setpoint_enu(self.sp_xy[0], self.sp_xy[1], self.sp_z, self.sp_yaw)

    def wait_reached_px4(self, xy, z, tol=0.15, settle_s=1.0, timeout_s=30.0, yaw=None):
        """Fly to a point in PX4's own frame (takeoff / dart) and wait until settled.
        settle_s and timeout_s are SIM seconds (the clock the vehicle flies on: 20 wall
        seconds at RTF 0.15 are 3 sim seconds, less than one dart leg); a wall-clock cap of
        timeout_s / 0.1 still ends the wait if the simulation stalls."""
        t0, w0, ok_since = self.sim_t(), time.time(), None
        while self.sim_t() - t0 < timeout_s and time.time() - w0 < timeout_s / 0.1:
            self.send(xy, z, yaw)
            e = self.px4.enu()
            if e and math.hypot(e[0] - xy[0], e[1] - xy[1]) < tol and abs(e[2] - z) < tol:
                ok_since = ok_since or self.sim_t()
                if self.sim_t() - ok_since >= settle_s:
                    return True
            else:
                ok_since = None
            time.sleep(self.tick_dt)
        return False

    # ------------------------------------------------------------ phases
    def connect(self):
        """Parameters and the home position; called once."""
        a = self.a
        self.px4.set_param("COM_RCL_EXCEPT", 4, integer=True)      # no RC in offboard
        self.px4.set_param("COM_OF_LOSS_T", 1.0)
        self.px4.set_param("MPC_XY_VEL_MAX", a.speed)               # cruise cap
        self.px4.set_param("MPC_XY_CRUISE", a.speed)
        # Abrupt takeoff, by protocol: the VIO static initializer needs a truly static
        # window (the disarmed, parked drone: accelerometer std ~0.05) followed by a sharp
        # visual + inertial transition. PX4's default 3 s takeoff ramp smears the transition
        # into the 0.07-0.11 band that overlaps the parked noise (measured in the first flight tests); a 0.5 s ramp
        # makes the liftoff the jerk the initializer is waiting for.
        self.px4.set_param("MPC_TKO_RAMP_T", a.tko_ramp)
        self.px4.set_param("MPC_TKO_SPEED", a.tko_speed)
        t0 = time.time()
        while self.px4.enu() is None:
            if time.time() - t0 > 60.0:
                raise RuntimeError("no LOCAL_POSITION_NED from PX4 within 60 wall-s (estimator not running?)")
            time.sleep(0.1)
        e = self.px4.enu()
        self.home = (e[0], e[1])
        self.sp_yaw = e[3]
        self.event("connected", px4_enu=[round(v, 3) for v in e[:4]],
                   tko_ramp=a.tko_ramp, tko_speed=a.tko_speed)

    def takeoff(self):
        """Parked and DISARMED for a few seconds (the static window), then arm and climb in one
        move, never armed-idle on the ground, never a slow ramp: the transition is the jerk."""
        a = self.a
        self._landed = False
        self.stream(4.0, self.home, 0.0)                            # setpoint stream before OFFBOARD
        ex = self.ros.imu_excitation()
        self.event("parked_imu", acc_std=round(ex[1], 4) if ex else None, armed=self.px4.armed)
        self.px4.set_mode(Px4Link.OFFBOARD_MAIN)
        self.stream(0.5, self.home, 0.0)
        self.px4.arm(True)
        self.event("takeoff_cmd", alt=a.alt)
        # PX4's heartbeat runs at 1 Hz of ITS clock (one per ~5 wall seconds at RTF 0.2): the
        # armed / mode flags are checked while the climb setpoint is already streaming
        t0, ok_since, ok, forced = time.time(), None, False, False
        while time.time() - t0 < 45.0:
            self.send(self.home, a.alt)
            e = self.px4.enu()
            if e and abs(e[2] - a.alt) < 0.15 and math.hypot(e[0] - self.home[0], e[1] - self.home[1]) < 0.3:
                ok_since = ok_since or time.time()
                if time.time() - ok_since >= 2.0:
                    ok = True
                    break
            else:
                ok_since = None
            if time.time() - t0 > 15.0 and not forced and not self.px4.armed:
                self.log(f"[exec] WARN not armed after 15 s (acks={self.px4.acks[-5:]}); force-arming")
                self.px4.arm(True, force=True)
                forced = True
            if time.time() - t0 > 15.0 and self.px4.main_mode != Px4Link.OFFBOARD_MAIN and not forced:
                self.log(f"[exec] WARN main_mode={self.px4.main_mode} (expected OFFBOARD=6); resending mode")
                self.px4.set_mode(Px4Link.OFFBOARD_MAIN)
                forced = True
            time.sleep(self.tick_dt)
        e = self.px4.enu()
        ex = self.ros.imu_excitation()
        self.event("takeoff_done" if ok else "takeoff_timeout", z=round(e[2], 3) if e else None,
                   armed=self.px4.armed, main_mode=self.px4.main_mode,
                   acc_std=round(ex[1], 4) if ex else None)
        self.dart_pending_back = False
        return ok

    def relaunch(self):
        """VIO gate failed: land, disarm, sit still (a fresh static window), take off again."""
        self.event("relaunch")
        self._landed = False
        self.land()
        self._landed = False
        self.px4.set_mode(Px4Link.OFFBOARD_MAIN)
        return self.takeoff()

    def dart(self):
        """1.5 m out-and-back along +x inside the clear zone, right after the takeoff,
        then hover at the spawn. Real horizontal motion makes scale and biases observable
        (and is a second chance for the initializer if the liftoff jerk was missed)."""
        a = self.a
        out = (self.home[0] + a.dart, self.home[1])
        ex = self.ros.imu_excitation()
        self.event("dart_out", length=a.dart, hover_acc_std=round(ex[1], 4) if ex else None)
        self.wait_reached_px4(out, a.alt, tol=0.15, settle_s=0.5, timeout_s=20.0)
        ex = self.ros.imu_excitation()
        self.event("dart_back", acc_std=round(ex[1], 4) if ex else None)
        ok = self.wait_reached_px4(self.home, a.alt, tol=0.15, settle_s=1.0, timeout_s=20.0)
        return ok

    def vio_gate(self):
        """Wait up to --gate-s for the odometry to stream: OpenVINS initialized (vio), or
        rf2o's poses arriving (lidar, normally true since the first scan on the ground)."""
        t0 = time.time()
        _, n0 = self.ros.latest_vio()
        lidar = self.odom_source == "lidar"
        while time.time() - t0 < self.a.gate_s:
            self.send(self.home, self.a.alt)
            v, n = self.ros.latest_vio()
            if v is not None and n > n0 + 5 and time.time() - v[5] < 1.0:
                self.event("odom_ready" if lidar else "vio_initialized", n=n, t_odom=round(v[4], 3), source=self.odom_source)
                return True
            time.sleep(self.tick_dt)
        ex = self.ros.imu_excitation()
        self.event("odom_gate_timeout", poses_seen=n0, source=self.odom_source, hover_acc_std=round(ex[1], 4) if ex else None)
        return False

    def sim_t(self):
        e = self.px4.enu()
        return e[4] if e else 0.0

    def amcl_converged(self):
        """The latest /amcl_pose, if it localizes the drone where it is now: a correction
        exists, at least one real filter update happened (the first pose is the initial-pose
        broadcast, covariance 0), the covariance has settled below --amcl-cov-max and the
        odometry has not moved more than 0.5 m since that pose. Returns (amcl, n, corr, d)."""
        amcl, n, corr = self.ros.latest_amcl()
        if amcl is None or corr is None or n < 2:
            return None
        d = self.ros.odom_since_amcl()
        if d is None or d > 0.5:
            return None
        if amcl[5] > self.a.amcl_cov_max or amcl[6] > self.a.amcl_cov_max:
            return None
        return amcl, n, corr, d

    def amcl_gate(self):
        """Wait up to --amcl-gate-s SIM seconds for AMCL to hold a converged pose of the
        hovering drone. VIO initializes at the liftoff, so the dart's 3 m of odometry
        normally gives AMCL its first ~20 updates before this gate runs and the gate passes
        at once (in testing: 18 poses, covariance 0.04 by the end of the dart). If AMCL
        came up late, a hover wiggle (+-0.25 m along x, 3 sim-seconds per side: the clock
        PX4 flies on, not the wall) provides the >= update_min_d of odometry motion AMCL
        wants for each update, until a converged pose arrives."""
        a = self.a
        t0 = self.sim_t()
        side, t_flip = 1.0, t0
        while self.sim_t() - t0 < a.amcl_gate_s:
            ok = self.amcl_converged()
            if ok:
                amcl, n, corr, d = ok
                self.event("amcl_initialized", n=n, t=round(amcl[3], 3),
                           corr_theta_deg=round(math.degrees(corr[0]), 2), corr_tx=round(corr[1], 3), corr_ty=round(corr[2], 3),
                           cov_xx=round(amcl[5], 4), cov_yy=round(amcl[6], 4), odom_since_amcl_m=round(d, 2))
                self.wait_reached_px4(self.home, a.alt, tol=0.15, settle_s=1.0, timeout_s=15.0)
                return True
            if self.sim_t() - t_flip > 3.0:
                side, t_flip = -side, self.sim_t()
            self.send((self.home[0] + 0.25 * side, self.home[1]), a.alt)
            time.sleep(self.tick_dt)
        amcl, n, corr = self.ros.latest_amcl()
        d = self.ros.odom_since_amcl()
        self.event("amcl_gate_timeout", poses_seen=n, corr=bool(corr),
                   cov_xx=round(amcl[5], 4) if amcl else None, cov_yy=round(amcl[6], 4) if amcl else None,
                   odom_since_amcl_m=round(d, 2) if d is not None else None)
        return False

    def land(self):
        if getattr(self, "_landed", False):
            return
        self._landed = True
        self.event("land")
        try:
            self.stream(1.0, None, self.a.alt)
            self.px4.set_mode(Px4Link.AUTO_MAIN, Px4Link.AUTO_LAND_SUB)
            t0 = time.time()
            while time.time() - t0 < 40.0 and self.px4.landed_state != 1:
                time.sleep(0.2)
            self.px4.arm(False)
        except Exception as exc:  # noqa: BLE001
            self.log(f"[exec] land error: {exc}")

    def door_shift(self, h_rel, d_door, min_width=0.45):
        """Lateral offset (m, + = left of the path axis) of the opening's centre seen in the
        lidar, and the opening's width, or (None, None). h_rel = path axis in the lidar
        frame; d_door = distance along the path to the narrowest point. Openings narrower
        than min_width are still reported (as the narrowest bounded run) so the caller can
        refuse them; the centring target is the widest admissible one nearest the axis."""
        reach = math.hypot(max(d_door, 0.0) + 0.8, 0.9)                 # the wall around the opening, its inner faces included
        ret = self.ros.sector_returns(h_rel - math.radians(100), h_rel + math.radians(100), 99.0)
        if ret is None or len(ret[0]) < 8:
            return None, None
        rr, aa = ret
        rel = wrap_arr(aa - h_rel)
        order = np.argsort(rel)
        rr, rel = rr[order], rel[order]
        near = rr < reach
        n = len(rr)
        best, slot = None, None
        i = 0
        while i < n:
            if near[i]:
                i += 1
                continue
            j = i
            while j + 1 < n and not near[j + 1]:
                j += 1
            if i > 0 and j < n - 1:                                     # bounded on both sides
                y_l = rr[j + 1] * math.sin(rel[j + 1])                  # left edge (larger angle)
                y_r = rr[i - 1] * math.sin(rel[i - 1])                  # right edge
                centre_ang = (rel[i - 1] + rel[j + 1]) / 2
                width = y_l - y_r
                if min_width <= width <= 1.4 and abs(centre_ang) <= math.radians(70):
                    if best is None or abs(centre_ang) < abs(best[0]):
                        best = (centre_ang, (y_l + y_r) / 2, width)
                elif 0.25 <= width < min_width and abs(centre_ang) <= math.radians(30):
                    if slot is None or abs(centre_ang) < abs(slot[0]):    # a slot on the axis, too narrow
                        slot = (centre_ang, (y_l + y_r) / 2, width)
            i = j + 1
        if best is not None:
            return best[1], round(best[2], 2)
        if slot is not None:
            return None, round(slot[2], 2)                              # (no centring target, its width)
        return None, None

    def lateral_clearance(self, h_rel, half=math.radians(40)):
        """(left, right) nearest lidar return in the two lateral cones (90 deg +- half) about
        the path axis h_rel (lidar frame), or None per side: the passage width the vehicle
        is actually in is their sum."""
        out = []
        for side in (1.0, -1.0):
            ret = self.ros.sector_returns(h_rel + side * math.pi / 2 - half, h_rel + side * math.pi / 2 + half, 3.0)
            out.append(float(ret[0].min()) if ret is not None and len(ret[0]) else None)
        return out[0], out[1]

    def retreat(self, path, idx, h_rel_fn, back_m=1.5, cap_s=20.0):
        """Back out along the path already flown, yaw frozen (the footprint stays aligned
        with the slot it is in), until the lateral clearances sum to --min-passage + 0.15
        or back_m of path is undone. In an early test flight the vehicle entered a 0.61 m slot, every
        path out of it started with a yaw change and the diagonal met the walls twice."""
        a = self.a
        pose0 = self.belief.pose()
        px, py = (pose0[0], pose0[1]) if pose0 else (path[idx][0], path[idx][1])
        x0, y0 = px, py
        back = path[:idx + 1][::-1]
        if len(back) >= 2:
            tgt = GridMap.point_along(back, 0, back_m)
        else:
            # the verdict came at the start of the path (the vehicle already sits in the
            # slot): back out along the path's own axis if the lidar sees room behind
            tx, ty = GridMap.tangent(path, 0)
            rear = self.ros.min_range_towards(h_rel_fn(pose0[2]) + math.pi, half_width=math.radians(20)) if pose0 else None
            if rear is None or rear < 1.0:
                self.event("retreat_blocked", rear_m=None if rear is None else round(rear, 2))
                return
            tgt = (px - 0.8 * tx, py - 0.8 * ty)
        t0 = self.sim_t()
        while self.sim_t() - t0 < cap_s:
            pose, e = self.belief.pose(), self.px4.enu()
            if pose is None or e is None:
                time.sleep(self.tick_dt)
                continue
            px, py, pyaw, _t = pose
            moved = math.hypot(px - x0, py - y0)
            cl_l, cl_r = self.lateral_clearance(h_rel_fn(pyaw))
            clear = cl_l is None or cl_r is None or cl_l + cl_r >= a.min_passage + 0.15
            dx, dy = tgt[0] - px, tgt[1] - py
            d = math.hypot(dx, dy)
            # at least 0.6 m back before the clearance may end the retreat (one open side
            # right at the mouth ended it after 0.15 m in the mock; the next path re-entered)
            if d < 0.2 or (moved >= 0.6 and clear):
                break
            k = min(1.0, 0.5 / d)
            self.send((e[0] + dx * k, e[1] + dy * k), a.alt)             # yaw: unchanged
            time.sleep(self.tick_dt)
        e = self.px4.enu()
        if e:
            self.send((e[0], e[1]), a.alt)
        self.settle(0.5, cap_s=3.0)
        self.event("retreat_done", x=round(px, 2), y=round(py, 2), moved_m=round(math.hypot(px - x0, py - y0), 2))

    # -------------------------------------------------------------- tour
    def run_goal(self, name, gx, gy, attempt):
        """One attempt: plan once, follow, claim. Returns (result, reason, extras)."""
        a = self.a
        pose = self.belief.pose()
        if pose is None:
            return "ABORTED", "no_estimate", {}
        path, used = self.gm.plan((pose[0], pose[1]), (gx, gy), radius_m=a.radius, unknown=a.unknown)
        if path is None:
            return "ABORTED", "no_path", {}
        plen = GridMap.path_length(path)
        self.log(f"[exec]   path {len(path)} pts, {plen:.1f} m")
        t_start = self.clock()
        idx = 0
        contact_since = None
        narrow_state = False
        opening_ok = False
        hold_state = False
        progress_idx, progress_t = 0, t_start                    # `blocked` = no progress along the path
        prev_pose, fast_since = None, None
        while True:
            now = self.clock()
            if now - t_start > a.timeout:
                return "TIMEOUT", "timeout", dict(path_m=plen)
            if a.estimator in ("vio", "amcl") and not self.belief.vio_fresh(a.vio_stale_s, a.odom_wall_stale_s):
                self.event("odom_lost", source=self.odom_source,
                           sim_since_last_pose=round(self.ros.sim_clock() - (self.ros.vio_rx_sim or 0.0), 2)
                           if self.ros.vio_rx_sim is not None else None)
                return "ABORTED", "odom_lost", dict(path_m=plen)
            if a.estimator == "amcl" and not self.belief.amcl_fresh(a.amcl_stale_m):
                self.event("amcl_lost", odom_since_amcl_m=round(self.ros.odom_since_amcl() or -1.0, 2))
                return "ABORTED", "amcl_lost", dict(path_m=plen)
            pose = self.belief.pose()
            if pose is None:
                time.sleep(self.tick_dt)
                continue
            px, py, pyaw, _t = pose
            # --- guards (a belief that has left physics, an impact, an imminent contact) ---
            if prev_pose is not None and _t > prev_pose[3] + 1e-3:
                v_belief = math.hypot(px - prev_pose[0], py - prev_pose[1]) / (_t - prev_pose[3])
                if v_belief > a.max_belief_speed:
                    fast_since = fast_since or now
                    if now - fast_since > 1.0:
                        self.event("odom_diverged", belief_speed=round(v_belief, 2), source=self.odom_source)
                        return "ABORTED", "odom_diverged", dict(path_m=plen)
                else:
                    fast_since = None
            prev_pose = pose
            ex = self.ros.imu_excitation()
            if ex is not None and ex[1] > a.impact_std:
                self.impacts += 1
                self.event("impact", acc_std=round(ex[1], 2), n=self.impacts)
                self.settle(2.0)                                   # hold until the IMU is calm again
                return "ABORTED", "impact", dict(path_m=plen)
            if math.hypot(px - gx, py - gy) <= a.claim_tol:
                # stop where the claim happens: the last setpoint is the carrot, up to
                # --lookahead further on, at a frontier goal possibly against a wall the map
                # does not have yet (smoke test 2026-09-25: 0.25 m more after f06's claim, contact)
                self.hold_here()
                return "SUCCEEDED", "claimed", dict(path_m=plen, claim_x=px, claim_y=py)
            if (math.hypot(px - path[-1][0], py - path[-1][1]) <= 0.2 and self.px4.speed() < 0.1
                    and math.hypot(path[-1][0] - gx, path[-1][1] - gy) > a.claim_tol):
                # the planner's tolerance moved the goal; the belief sits at the path end but
                # outside the claim radius; the baseline's stack aborts here too
                return "ABORTED", "path_end", dict(path_m=plen, claim_x=px, claim_y=py)
            # carrot: farthest path point within the lookahead with free line of sight on the
            # inflated map (no corner cutting into the inflation band)
            idx, carrot = self.gm.carrot(path, idx, (px, py), a.lookahead, a.radius, a.unknown)
            tangent = self.gm.tangent(path, idx)
            slow = False
            d_door = None
            if a.narrow_lookahead > 0:
                # doorway within --narrow-ahead metres along the path? The check spans
                # the path AHEAD of the progress point: the frames are met before the progress
                # point enters the wall (an early test flight reached a 0.91 m door at 1 m/s, unprepared)
                wmin, j_narrow = self.gm.narrowest_ahead(path, idx, a.narrow_ahead)
                # hysteresis: enter below --narrow-width, leave 0.3 m above it (a test flight
                # toggled seven times in five seconds at a 0.85 m door measured 0.85 / 1.0)
                narrow = wmin < (a.narrow_width + 0.3 if narrow_state else a.narrow_width)
                if narrow != narrow_state:
                    narrow_state = narrow
                    opening_ok = False
                    self.event("narrow_passage" if narrow else "narrow_passage_end", width=round(wmin, 2),
                               x=round(path[j_narrow][0], 2), y=round(path[j_narrow][1], 2))
                if narrow:
                    slow = True
                    d_door = GridMap.dist_along(path, idx, j_narrow)
            if a.slow_cov > 0 and a.estimator == "amcl":
                # an uncertain localizer is flown slowly too (a lost filter met a door frame at
                # 1 m/s in a test flight)
                amcl, _n, _c = self.ros.latest_amcl()
                if amcl is not None and max(amcl[5], amcl[6]) > a.slow_cov:
                    slow = True
            if slow:
                idx, carrot = self.gm.carrot(path, idx, (px, py), a.narrow_lookahead, a.radius, a.unknown)
            dx, dy = carrot[0] - px, carrot[1] - py
            if narrow_state and a.center_gain > 0 and d_door is not None:
                # live centring on the opening: in the forward half-plane of the path
                # axis, the returns nearer than the door plane are the wall around it and the
                # run of far returns between two near ones is the opening; its two bounding
                # returns are the gap edges (the frame corners while approaching, the frames
                # themselves inside). The setpoint is shifted onto the edges' midpoint, so the
                # pass does not depend on the belief's lateral error (the wheeled baseline's
                # local costmap does this job for it).
                h_rel = wrap(math.atan2(tangent[1], tangent[0]) - pyaw)         # path axis in the lidar frame
                shift, gap = self.door_shift(h_rel, d_door, a.min_passage)
                # (door_shift's gap is NOT a passability verdict: its lateral projection of the
                # run edges read real doors as 0.37-0.46 m in the mocks when approached
                # obliquely; the lateral-clearance rule below is the only judge)
                if shift is not None:
                    opening_ok = True                                   # stays set until the zone ends
                if shift is not None:
                    # the lidar owns the lateral axis through the door: the carrot's lateral
                    # component (which pulls toward the belief's idea of the path) is replaced
                    shift = max(-0.25, min(0.25, a.center_gain * shift))          # + = to the left
                    along = dx * tangent[0] + dy * tangent[1]
                    dx = along * tangent[0] - shift * tangent[1]
                    dy = along * tangent[1] + shift * tangent[0]
                if os.environ.get("AEB_DEBUG_DOOR"):
                    e = self.px4.enu()
                    self.log(f"[door] d_door={d_door:.2f} gap={gap} shift={shift} belief=({px:.2f},{py:.2f}) "
                             f"px4=({e[0]:.2f},{e[1]:.2f})" if e else "[door] no px4")
            # the passage the vehicle is IN, every tick: the lateral clearances (nearest return
            # in the two 90 deg +- 40 deg cones about the BODY's travel axis: body -y in crab
            # flight, body x nose-first) sum to the local width; for a doorway the frames
            # (0.78 m door: 0.39 + 0.39), for the slot of the early test flights (cylinder vs arena wall,
            # 0.61 m, unmapped on the wall side so no map rule saw it) less than the vehicle
            # can pass (>= ~0.75 m sideways). Below --min-passage: back out the way it
            # came, yaw frozen, and give the goal up. Below --yaw-clear (the 0.77 m diagonal
            # plus a margin): no yaw change; in the early test flights two impacts were yaw slews inside the
            # slot toward the tangent of the next path, and three more were the same slews
            # measured about the PATH axis, whose cones looked past the frames beside the body
            # (a bend right after a doorway, a path out of the slot).
            h_axis = -math.pi / 2 if a.yaw == "crab" else 0.0
            cl_l, cl_r = self.lateral_clearance(h_axis)
            clr = (cl_l + cl_r) if (cl_l is not None and cl_r is not None) else None
            if clr is not None and clr < a.min_passage and math.hypot(dx, dy) > 0.05:
                self.event("too_narrow", width=round(clr, 2), left=round(cl_l, 2), right=round(cl_r, 2),
                           x=round(px, 2), y=round(py, 2), where="here")
                self.retreat(path, idx, lambda yaw: h_axis)
                return "ABORTED", "too_narrow", dict(path_m=plen)
            yaw_frozen = clr is not None and clr < getattr(a, "yaw_clear", 0.85)
            # imminent contact (anything nearer than --contact-dist, any direction). Outside a
            # narrow passage: with --repel-gain the setpoint is pushed away from the nearest
            # return (a belief 0.2-0.4 m off next to a wall would otherwise hold until the
            # abort); without it, hold and abort after --blocked-s. Inside a narrow passage the
            # frames legitimately sit 0.3-0.45 m off the centre and the centring keeps the
            # vehicle off them (a hold would freeze it in the doorway).
            # ... but only while an admissible opening is actually measured: a corner pocket
            # also reads as "narrow" on the map (in a test flight at f04: box end + arena wall), and
            # there the hold must stay armed
            near = self.ros.nearest_return()
            if near is not None and near[0] < a.contact_dist and not (narrow_state and opening_ok):
                if a.repel_gain > 0 and near[0] >= 0.18:
                    # never move toward the return: drop the carrot's component toward it, then
                    # push away (in a test flight a 0.15 m push against a 0.8 m carrot pull crept into
                    # a box corner over six seconds)
                    ang = near[1] + pyaw                           # obstacle bearing, map frame
                    ux, uy = math.cos(ang), math.sin(ang)
                    toward = dx * ux + dy * uy
                    if toward > 0:
                        dx -= toward * ux
                        dy -= toward * uy
                    push = min(0.15, a.repel_gain * (a.contact_dist + 0.05 - near[0]))
                    dx -= push * ux
                    dy -= push * uy
                    contact_since = None
                else:
                    contact_since = contact_since or now
                    if now - contact_since > a.blocked_s:
                        return "ABORTED", "contact", dict(path_m=plen)
                    e = self.px4.enu()
                    if e:
                        self.send((e[0], e[1]), a.alt)             # hold, do not push further
                    time.sleep(self.tick_dt)
                    continue
            else:
                contact_since = None
            travel = math.atan2(dy, dx)
            # lidar safety: hold if something is closer than --stop-dist in the travel direction
            # (mapped obstacles stay >= planner radius from the path; anything nearer is unmapped).
            # Inside a narrow passage the sector is centred on the path axis: the lateral
            # centring shift must not swing it onto the door frames, which are mapped and near.
            look = math.atan2(tangent[1], tangent[0]) if narrow_state else travel
            rmin = self.ros.min_range_towards(wrap(look - pyaw), half_width=math.radians(a.stop_sector))
            e = self.px4.enu()
            if rmin is not None and rmin < a.stop_dist and math.hypot(dx, dy) > 0.2:
                if not hold_state:
                    hold_state = True
                    self.event("safety_hold", range_m=round(rmin, 2), x=round(px, 2), y=round(py, 2))
                if a.repel_gain > 0:
                    # slide: drop the component toward the nearest return in the sector, keep
                    # the rest (the wheeled baseline's local planner steers around unmapped
                    # obstacles the same way); no progress along the path for --blocked-s
                    # sim seconds still ends the attempt as `blocked`
                    nr = self.ros.nearest_return()
                    if nr is not None:
                        ang = nr[1] + pyaw
                        ux, uy = math.cos(ang), math.sin(ang)
                        toward = dx * ux + dy * uy
                        if toward > 0:
                            dx -= toward * ux
                            dy -= toward * uy
                else:
                    self.send((e[0], e[1]), a.alt)             # hold
                    time.sleep(self.tick_dt)
                    if now - progress_t > a.blocked_s:
                        return "ABORTED", "blocked", dict(path_m=plen)
                    continue
            elif hold_state:
                hold_state = False
                self.event("safety_hold_end", x=round(px, 2), y=round(py, 2))
            if idx > progress_idx:
                progress_idx, progress_t = idx, now
            elif now - progress_t > a.blocked_s:
                return "ABORTED", "blocked", dict(path_m=plen)
            # relative setpoint: PX4 target = PX4 pose + (carrot - belief) in the map frame.
            # The yaw follows the PATH TANGENT, not the carrot vector: the footprint must align
            # with the corridor axis; the carrot vector carries the lateral correction, which
            # in a doorway turned the vehicle's long diagonal across the opening (in an early test flight).
            sp = (e[0] + dx, e[1] + dy)
            yaw_des = math.atan2(tangent[1], tangent[0]) + (math.pi / 2 if a.yaw == "crab" else 0.0)
            yaw_sp = None if yaw_frozen else e[3] + wrap(yaw_des - pyaw)
            self.send(sp, a.alt, yaw_sp)
            time.sleep(self.tick_dt)

    def tour(self):
        a = self.a
        f = open(self.csv_path, "w", newline="")
        w = csv.writer(f)
        w.writerow(["mission", "goal_x", "goal_y", "attempt", "result", "wall_s", "sim_s",
                    "epoch_start", "epoch_end", "sim_start", "sim_end", "claim_x", "claim_y",
                    "path_m", "reason", "px4_x", "px4_y"])
        f.flush()
        self.t0 = dict(wall=time.time(), sim=self.clock())
        self.event("policy_clock_start", t0_sim=round(self.t0["sim"], 3))
        ok = first = 0
        for name, gx, gy, room in self.goals:
            result = None
            for attempt in (1, 2):
                self.log(f"[exec] -> {name} {room} (attempt {attempt}): ({gx:.2f}, {gy:.2f})")
                e0, s0 = time.time(), self.clock()
                result, reason, extra = self.run_goal(name, gx, gy, attempt)
                e1, s1 = time.time(), self.clock()
                e = self.px4.enu()
                self.log(f"[exec]    {result} ({reason}) in {e1 - e0:.0f}s wall / {s1 - s0:.0f}s sim")
                row = [name, f"{gx:.2f}", f"{gy:.2f}", attempt, result, f"{e1 - e0:.1f}", f"{s1 - s0:.1f}",
                       f"{e0:.3f}", f"{e1:.3f}", f"{s0:.3f}", f"{s1:.3f}",
                       f"{extra.get('claim_x', float('nan')):.3f}", f"{extra.get('claim_y', float('nan')):.3f}",
                       f"{extra.get('path_m', float('nan')):.2f}", reason,
                       f"{e[0]:.3f}" if e else "", f"{e[1]:.3f}" if e else ""]
                w.writerow(row)
                f.flush()
                self.manifest["results"].append(dict(zip(["mission", "goal_x", "goal_y", "attempt", "result",
                                                          "wall_s", "sim_s", "epoch_start", "epoch_end",
                                                          "sim_start", "sim_end", "claim_x", "claim_y",
                                                          "path_m", "reason", "px4_x", "px4_y"], row)))
                if result == "SUCCEEDED":
                    first += attempt == 1
                    break
                if reason in ("odom_lost", "amcl_lost", "odom_diverged", "no_estimate") or self.impact_limit():
                    break
                if attempt == 1:
                    self.settle(a.retry_wait)
            ok += result == "SUCCEEDED"
            if result != "SUCCEEDED" and reason in ("odom_lost", "amcl_lost", "odom_diverged", "no_estimate"):
                self.event("tour_stopped", reason=reason)
                break
            if self.impact_limit():
                self.event("tour_stopped", reason="impacts", n=self.impacts)
                break
            self.settle(0.7, cap_s=3.0)                            # short breather between goals, guarded
            if self.impact_limit():
                self.event("tour_stopped", reason="impacts", n=self.impacts)
                break
        f.close()
        self.manifest["summary"] = dict(succeeded=ok, first_try=first, goals=len(self.goals))
        self.log(f"[exec] TOUR DONE: {ok}/{len(self.goals)} SUCCEEDED ({first} first-try) -> {self.csv_path}")

    def impact_limit(self):
        """True once --max-impacts impacts happened (0 = never: the batch's protocol;
        an impact aborts the attempt and is counted, the tour or exploration goes on; the
        estimator guards still stop it)."""
        return self.a.max_impacts > 0 and self.impacts >= self.a.max_impacts

    def hold_here(self):
        """Position setpoint = where PX4 is now (stop; the controller brakes from its speed)."""
        e = self.px4.enu()
        if e:
            self.send((e[0], e[1]), self.a.alt)

    def settle(self, min_s, cap_s=12.0):
        """Hold the current PX4 position for at least min_s SIM seconds and until the IMU
        window is calm again (std < --impact-std / 2). After a contact the 2 s excitation
        window keeps reporting the hit: a retry started 0.5 sim-seconds later counted the
        same contact as a second impact and stopped the tour (in an early test flight).
        The hold is guarded like a goal attempt (smoke test 2026-09-25: a contact 2 s after a
        frontier claim went uncounted): a lidar return nearer than --stop-dist moves the
        hold point away from it until the clearance is --stop-dist (`settle_standoff`), and a
        NEW contact (the IMU calm when the settle began) counts as an impact."""
        a = self.a
        e = self.px4.enu()
        xy = (e[0], e[1]) if e else None
        ex = self.ros.imu_excitation()
        armed = ex is None or ex[1] < a.impact_std / 2      # a settle after a contact waits for calm instead
        standoff = False
        t0 = self.sim_t()
        while True:
            e = self.px4.enu()
            nr = self.ros.nearest_return()
            if e and nr is not None and nr[0] < a.stop_dist:
                ang = nr[1] + e[3]                           # the lidar turns with the body's yaw
                back = a.stop_dist - nr[0]
                xy = (e[0] - back * math.cos(ang), e[1] - back * math.sin(ang))
                if not standoff:
                    standoff = True
                    self.event("settle_standoff", range_m=round(nr[0], 2), x=round(e[0], 2), y=round(e[1], 2))
            self.send(xy, a.alt)
            dt = self.sim_t() - t0
            ex = self.ros.imu_excitation()
            if armed and ex is not None and ex[1] > a.impact_std:
                armed = False
                self.impacts += 1
                self.event("impact", acc_std=round(ex[1], 2), n=self.impacts, where="settle")
            calm = ex is None or ex[1] < a.impact_std / 2
            if (dt >= min_s and calm) or dt >= cap_s:
                return
            time.sleep(self.tick_dt)

    def go_home(self):
        """Return to the spawn with the goal follower and its guards (the return is not
        scored, but an unguarded return bounced through a doorway in an early test flight with a belief
        the contacts had corrupted). On an abort the drone lands where it is."""
        self.event("return_home")
        result, reason, _ = self.run_goal("home", self.home[0], self.home[1], 1)
        if result == "SUCCEEDED":
            self.event("home_reached")
        else:
            self.event("home_abort", reason=reason)

    def save_manifest(self, status):
        self.manifest["status"] = status
        self.manifest["wall_end"] = time.strftime("%Y-%m-%d %H:%M:%S")
        with open(self.manifest_path, "w") as f:
            json.dump(self.manifest, f, indent=1)
        self.log(f"[exec] manifest -> {self.manifest_path}")

    # --------------------------------------------------------------- main
    def run(self):
        a = self.a
        status = "OK"
        try:
            self.connect()
            if not self.takeoff():
                status = "TAKEOFF_FAIL"                 # the finally block lands and writes the manifest
                return 3
            uses_vio = a.estimator in ("vio", "amcl")
            if uses_vio or a.force_dart:
                gated = False
                for k in (1, 2):
                    if k > 1:
                        if not self.relaunch():
                            break
                    if not self.dart():
                        self.log("[exec] WARN dart did not settle")
                    if not uses_vio:
                        gated = True
                        break
                    if self.vio_gate():
                        gated = True
                        break
                    self.event("odom_retry", k=k, source=self.odom_source)
                if not gated:
                    self.event("ODOM_INIT_FAIL", source=self.odom_source)
                    status = "ODOM_INIT_FAIL"                 # the finally block lands and writes the manifest
                    return 2
                if uses_vio:
                    self.stream(1.0, self.home, a.alt)             # settle before anchoring
                    anc = self.belief.anchor()                     # odom->map offset at the spawn (diagnostic in amcl mode)
                    self.manifest["anchor"] = anc
                    if anc:
                        self.event("anchor", theta_deg=round(math.degrees(anc["theta"]), 2),
                                   tx=round(anc["tx"], 3), ty=round(anc["ty"], 3))
                if a.estimator == "amcl" and not self.amcl_gate():
                    self.event("AMCL_INIT_FAIL")
                    status = "AMCL_INIT_FAIL"                 # the finally block lands and writes the manifest
                    return 4
            self.tour()
            self.go_home()
        except KeyboardInterrupt:
            status = "INTERRUPTED"
        except Exception as exc:  # noqa: BLE001
            import traceback
            self.log(f"[exec] ERROR {exc}\n{traceback.format_exc()}")
            status = f"ERROR: {exc}"
        finally:
            self.land()
            self.save_manifest(status)
            self.belief.close()
            self.px4.close()
            self.ros.close()
        return 0 if status == "OK" else 1


# ============================================================ vehicles
VEHICLE_DEFAULTS = {
    # the values that depend on the body, for the options the command line leaves unset
    "air":    dict(alt=1.3, radius=0.35, yaw="crab", speed=1.0, min_passage=0.70, yaw_clear=0.85, narrow_lookahead=0.3),
    # the wheeled robot: 0.40 x 0.30 m body, half its diagonal 0.25 + 0.05 for the planner (doors
    # from 0.60 m), nose-first, 0.30 m/s (the baseline's cap), a 0.50 m passage for the 0.30 m
    # width, the yaw frozen below 0.55 m (the 0.50 m diagonal + a margin), no altitude; the doorway
    # carrot 0.5 m: a differential drive corrects laterally only by driving forward, and with the
    # drone's 0.3 m it cut the bend before world 008's north door onto the jamb (the mock harness)
    "ground": dict(alt=0.0, radius=0.30, yaw="forward", speed=0.30, min_passage=0.50, yaw_clear=0.55, narrow_lookahead=0.5),
}


def add_vehicle_args(ap):
    ap.add_argument("--vehicle", choices=["air", "ground"], default="air",
                    help="air = the quadrotor over PX4 (MAVLink); ground = the wheeled robot over /cmd_vel from the "
                         "same odometry (GroundLink): the same executive, the body's values for --alt, --radius, "
                         "--yaw, --speed, --min-passage and --yaw-clear unless given")
    ap.add_argument("--ground-v-min", type=float, default=0.10, help="ground: slowest forward command while moving, m/s")
    ap.add_argument("--ground-w-min", type=float, default=0.5,
                    help="ground: slowest turn in place, rad/s (the simulated drive ignores turns below ~0.2 rad/s)")
    ap.add_argument("--ground-w-max", type=float, default=1.0, help="ground: fastest turn, rad/s")
    ap.add_argument("--ground-turn", type=float, default=0.8,
                    help="ground: heading error (rad) beyond which the robot turns in place instead of steering")


def resolve_vehicle_defaults(a):
    """The body-dependent options left unset take the vehicle's values (VEHICLE_DEFAULTS)."""
    for k, v in VEHICLE_DEFAULTS[getattr(a, "vehicle", "air")].items():
        if getattr(a, k, None) is None:
            setattr(a, k, v)
    return a


def main():
    install_stop_signals()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", required=True)
    ap.add_argument("--map", default=None, help="frozen map YAML (default worlds/maps/world_<seed>.yaml)")
    ap.add_argument("--goals", default=None, help="goal CSV (default missions/goals/goals_<seed>.csv)")
    ap.add_argument("--estimator", choices=["px4", "vio", "amcl"], default="amcl",
                    help="px4 = PX4's own estimate (tests); vio = the raw odometry, dead reckoning; "
                         "amcl = AMCL correction over the odometry (the benchmark's mode)")
    ap.add_argument("--odom-source", choices=["lidar", "vio"], default="lidar",
                    help="lidar = rf2o range-flow odometry from /scan (the benchmark's source); vio = OpenVINS")
    ap.add_argument("--odom-topic", default="/odom_rf2o", help="nav_msgs/Odometry of the lidar odometry (rf2o)")
    ap.add_argument("--vio-topic", default="/poseimu", help="OpenVINS pose topic (--odom-source vio)")
    add_vehicle_args(ap)
    ap.add_argument("--alt", type=float, default=None, help="flight altitude above the spawn (band 1.2-1.5 m; air 1.3, ground 0)")
    ap.add_argument("--radius", type=float, default=None, help="planner radius, m (air 0.35, ground 0.30)")
    ap.add_argument("--unknown", choices=["free", "lethal", "wall"], default="free",
                    help="unknown map cells: free (baseline costmap behaviour), lethal (inflated) or wall (impassable, not inflated)")
    ap.add_argument("--yaw", choices=["crab", "forward"], default=None,
                    help="crab = body x-axis perpendicular to travel (0.52 m presented to doors; the air default), forward = nose-first (ground)")
    ap.add_argument("--yaw-rate", type=float, default=60.0,
                    help="yaw setpoint slew limit, deg/s of sim time (0 = none); a 195 deg/s turn broke the VIO's scale in a test flight")
    ap.add_argument("--speed", type=float, default=None, help="speed cap, m/s: MPC_XY_VEL_MAX (air 1.0) or the ground controller's (0.30)")
    ap.add_argument("--lookahead", type=float, default=0.8, help="pure-pursuit carrot distance, m")
    ap.add_argument("--narrow-lookahead", type=float, default=None,
                    help="carrot distance inside passages narrower than --narrow-width: a slower, settled "
                         "doorway pass; 0 = off (same lookahead everywhere); air 0.3, ground 0.5")
    ap.add_argument("--narrow-width", type=float, default=1.0,
                    help="free width across the path below which a passage counts as narrow, m")
    ap.add_argument("--narrow-ahead", type=float, default=1.0,
                    help="how far along the path ahead of the progress point a narrow passage is looked for, m")
    ap.add_argument("--min-passage", type=float, default=None,
                    help="an opening the lidar measures narrower than this (m) is a slot, not a doorway: the attempt is "
                         "ABORTED (too_narrow); the Iris passes >= ~0.75 m doors sideways (0.52 m side): air 0.70; "
                         "ground 0.50 (the 0.30 m wide body)")
    ap.add_argument("--yaw-clear", type=float, default=None,
                    help="below this lateral clearance sum (m) the yaw setpoint is frozen: the 0.77 m diagonal must not turn "
                         "inside a slot: air 0.85; ground 0.55 (the 0.50 m diagonal)")
    ap.add_argument("--center-gain", type=float, default=1.0,
                    help="in and before narrow passages, shift the setpoint onto the opening's centre measured "
                         "in the lidar (gap edges on both sides; +-0.2 m cap); 0 = off. Needs --narrow-lookahead > 0")
    ap.add_argument("--repel-gain", type=float, default=0.0,
                    help="outside narrow passages, push the setpoint away from a lidar return nearer than "
                         "--contact-dist (gain on the intrusion, 0.15 m cap) instead of holding; 0 = hold")
    ap.add_argument("--slow-cov", type=float, default=0.15,
                    help="AMCL covariance (xx or yy, m^2) above which the carrot shortens to --narrow-lookahead "
                         "(fly slowly while the localizer is uncertain); 0 = off")
    ap.add_argument("--claim-tol", type=float, default=0.35, help="claim tolerance |p_est - goal|_xy, m")
    ap.add_argument("--timeout", type=float, default=240.0, help="per-goal timeout, s")
    ap.add_argument("--timeout-clock", choices=["sim", "wall"], default="sim")
    ap.add_argument("--retry-wait", type=float, default=3.0,
                    help="hold before the second attempt, SIM seconds (and until the IMU is calm)")
    ap.add_argument("--stop-dist", type=float, default=0.40,
                    help="lidar safety hold distance in the travel direction (planner radius + 0.05)")
    ap.add_argument("--stop-sector", type=float, default=20.0, help="half-width of the safety sector, deg")
    ap.add_argument("--blocked-s", type=float, default=20.0, help="hold time before ABORTED (blocked)")
    ap.add_argument("--contact-dist", type=float, default=0.30,
                    help="360-degree imminent-contact guard: hold when any lidar return is nearer (body envelope 0.26 m)")
    ap.add_argument("--impact-std", type=float, default=5.0,
                    help="accelerometer 2 s std above this = a collision; the attempt is ABORTED (impact)")
    ap.add_argument("--max-impacts", type=int, default=2,
                    help="impacts per tour before the tour stops; 0 = never (the batch's protocol: an impact "
                         "aborts only the attempt and is counted)")
    ap.add_argument("--max-belief-speed", type=float, default=3.0,
                    help="belief moving faster than this for >1 s = estimator divergence; the tour stops")
    ap.add_argument("--dart", type=float, default=1.5, help="init dart length, m")
    ap.add_argument("--tko-ramp", type=float, default=0.5, help="MPC_TKO_RAMP_T: abrupt liftoff = the VIO jerk")
    ap.add_argument("--tko-speed", type=float, default=2.0, help="MPC_TKO_SPEED")
    ap.add_argument("--force-dart", action="store_true", help="fly the dart with --estimator px4 too")
    ap.add_argument("--gate-s", type=float, default=10.0, help="VIO initialization gate, s")
    ap.add_argument("--amcl-gate-s", type=float, default=40.0,
                    help="SIM seconds to wait for a converged AMCL pose at the hover (normally immediate: the dart feeds AMCL)")
    ap.add_argument("--amcl-cov-max", type=float, default=0.25,
                    help="AMCL pose covariance (xx and yy, m^2) below which the localizer counts as converged")
    ap.add_argument("--vio-stale-s", type=float, default=1.5,
                    help="SIM seconds without an odometry pose = odometry lost -> ABORTED (odom_lost); wall gaps are "
                         "not counted (a 2.1 s wall stall of the pipeline aborted an early test flight)")
    ap.add_argument("--odom-wall-stale-s", type=float, default=90.0,
                    help="WALL seconds without an odometry pose = odometry lost (the backstop behind --vio-stale-s "
                         "for a dead pipeline; 20 s until 2026-10-02, when a 15-20 s simulator stall during an "
                         "internet outage cut world 001's exploration at 142 s)")
    ap.add_argument("--amcl-stale-m", type=float, default=3.0,
                    help="odometry displacement since the last /amcl_pose beyond which the localizer counts as "
                         "lost -> ABORTED (amcl_lost); AMCL updates every update_min_d = 0.15 m of motion")
    ap.add_argument("--tour-limit", type=int, default=0, help="only the first N goals (tests)")
    ap.add_argument("--rate", type=float, default=20.0, help="setpoint rate, Hz wall")
    ap.add_argument("--mav", default="udpin:0.0.0.0:14540")
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "runs", "raw"))
    a = ap.parse_args()
    resolve_vehicle_defaults(a)
    a.map = a.map or os.path.join(ROOT, "worlds", "maps", f"world_{a.seed}.yaml")
    a.goals = a.goals or os.path.join(ROOT, "missions", "goals", f"goals_{a.seed}.csv")

    def log(s):
        print(f"{time.strftime('%H:%M:%S')} {s}", flush=True)

    log(f"[exec] world {a.seed} vehicle={a.vehicle} map={os.path.relpath(a.map, ROOT)} estimator={a.estimator} "
        f"alt={a.alt} radius={a.radius} unknown={a.unknown} yaw={a.yaw} speed={a.speed} "
        f"timeout={a.timeout}s ({a.timeout_clock})")
    return Executive(a, log).run()


if __name__ == "__main__":
    sys.exit(main())
