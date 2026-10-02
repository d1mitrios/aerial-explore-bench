# === Isaac: ground-truth odometry with CONTROLLED noise -> /odom + TF ===
# Reads the robot pose from the same prim as the scan/metrics stack and integrates a
# noisy body-frame delta into an "odom" frame, like a real wheel-odometry would drift.
# Noise is PARAMETRIC (NOISE_V/NOISE_W multiplicative sigma + optional yaw-rate bias),
# so map-accuracy-vs-odometry-noise becomes an ablation knob. NOISE 0/0 = perfect odom.
#
# Publishes (wall-clock stamps, SAME clock as scan_raycast_publisher2 -> consistent):
#   /odom      nav_msgs/Odometry   (odom -> base_link), ~20 Hz
#   /tf        odom -> base_link
#   /tf_static base_link -> lidar_link (identity: the pose prim IS the lidar)
# Resets integration on every PLAY (batch runner / respawns). Run-once guard.
# NOTE: English prints only (Windows console mangles Greek).
#
# aerial-explore-bench copy (sim/wheeled/README.md): the wheeled baseline's publisher v5,
# unchanged except the two file locations below (NOISE_FILE, RUNS_DIR), which come from the
# run's environment instead of the baseline's folder, the `import os` they need, and PUBLISH_TF:
# with AEB_ODOM_TF=0 the /odom topic and the log stay but no TF is sent, neither
# odom->base_link nor base_link->lidar_link - the shared executive's runs take both from the
# lidar odometry chain (missions/aerial_odom.sh: rf2o + lidar_odom_bridge.py), and two
# publishers of one transform would fight.
import math
import os
import random
import time
import builtins
import omni.usd
import omni.physx
import omni.timeline
from pxr import UsdGeom
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

POSE_PATH = "/my_custom_robot/Geometry/chassis/lidar_link"
PUBLISH_EVERY = 3          # 60 Hz physics -> ~20 Hz (same as /scan)
NOISE_V = 0.03             # multiplicative sigma on the linear delta (3%)
NOISE_W = 0.05             # multiplicative sigma on the angular delta (5%)
BIAS_W = 0.0               # rad/s systematic yaw drift (knob for ablation)
SEED = 20260727
# v3: the noise file is read ON EVERY PLAY (not only at load); one Stop->Play
# is enough to change tier, and the active tier is printed/logged per run so the
# run<->noise mapping is PROVABLE after the fact (lesson, 27/7: the
# "n810" run ran with 0/0 because no restart happened; one continuous CSV gave it away).
NOISE_FILE = os.environ.get("AEB_ODOM_NOISE_FILE", "")   # unset: the defaults above apply
RUNS_DIR = os.environ.get("AEB_RUNS_DIR", "runs")          # the run directory (current_seed.txt, odom log)
PUBLISH_TF = os.environ.get("AEB_ODOM_TF", "1") != "0"     # 0 = no TF from here (rf2o owns odom->base_link)


def _read_noise():
    try:
        v, w, b = open(NOISE_FILE).read().split()
        return float(v), float(w), float(b)
    except Exception:
        return NOISE_V, NOISE_W, BIAS_W

if getattr(builtins, "_odom_pub", None):
    print("[odom] already running - guard skip")
else:
    class _OdomPub:
        def __init__(self):
            self.stage = omni.usd.get_context().get_stage()
            self.tl = omni.timeline.get_timeline_interface()
            self.xf = UsdGeom.Xformable(self.stage.GetPrimAtPath(POSE_PATH))
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node("isaac_odom_pub")
            self.pub = self.node.create_publisher(Odometry, "/odom", 10)
            self.tf_pub = self.node.create_publisher(TFMessage, "/tf", 10)
            qos = QoSProfile(depth=1)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.tfs_pub = self.node.create_publisher(TFMessage, "/tf_static", qos)
            self.rng = random.Random(SEED)
            self.n = 0
            self.last = None           # (x, y, yaw) ground truth of the previous step
            self.o = [0.0, 0.0, 0.0]   # integrated odom pose
            self.last_wall = time.time()
            self.nv, self.nw, self.nb = _read_noise()
            self.log = None            # v3: odom log per run (drift = measurable ground truth)
            if PUBLISH_TF:
                self._send_static()
            self.phys_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)
            self.tl_sub = self.tl.get_timeline_event_stream().create_subscription_to_pop(self._on_ev)
            if self.tl.is_playing():
                self._open_log()
            print(f"[odom] v5 running: noise v={self.nv} w={self.nw} bias_w={self.nb} "
                  f"(noise re-read + stage rebind on every PLAY); TF {'on' if PUBLISH_TF else 'OFF (AEB_ODOM_TF=0)'}")

        def _open_log(self):
            try:
                seed = open(f"{RUNS_DIR}/current_seed.txt").read().strip()
            except Exception:
                seed = "unknown"
            path = f"{RUNS_DIR}/odom_{seed}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            self.log = open(path, "w", buffering=1)
            self.log.write(f"# noise_v={self.nv} noise_w={self.nw} bias_w={self.nb}\n")
            self.log.write("ox,oy,oyaw,tx,ty,tyaw\n")
            print(f"[odom] run log -> {path} (noise v={self.nv} w={self.nw})")

        def _send_static(self):
            t = TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = "base_link"
            t.child_frame_id = "lidar_link"
            t.transform.rotation.w = 1.0
            m = TFMessage(); m.transforms = [t]
            self.tfs_pub.publish(m)

        def _rebind(self):
            # v5: fresh stage+prim handles; the double bootstrap (29/7) reopened the
            # stage and the cached xf died ("Accessed schema on invalid prim").
            self.stage = omni.usd.get_context().get_stage()
            self.xf = UsdGeom.Xformable(self.stage.GetPrimAtPath(POSE_PATH))

        def _on_ev(self, e):
            if e.type == int(omni.timeline.TimelineEventType.PLAY):
                self._rebind()
                self.last = None
                self.o = [0.0, 0.0, 0.0]
                self.nv, self.nw, self.nb = _read_noise()   # v3: fresh tier on every PLAY
                if self.log:
                    self.log.close()
                self._open_log()
                print(f"[odom] PLAY - re-zeroed, noise v={self.nv} w={self.nw} bias={self.nb}")
            elif e.type == int(omni.timeline.TimelineEventType.STOP):
                if self.log:
                    self.log.close()
                    self.log = None

        def _pose(self):
            m = self.xf.ComputeLocalToWorldTransform(0)
            t = m.ExtractTranslation()
            r = m.ExtractRotationMatrix()
            return float(t[0]), float(t[1]), math.atan2(r[0][1], r[0][0])

        def _on_step(self, dt):
            try:
                x, y, yaw = self._pose()
            except Exception:
                try:
                    self._rebind()   # v5: self-heal after a stage reopen
                    x, y, yaw = self._pose()
                except Exception:
                    return           # prim unavailable - skip the step
            if self.last is None:
                self.last = (x, y, yaw)
                return
            lx, ly, lyaw = self.last
            self.last = (x, y, yaw)
            # world delta -> body frame (of the previous yaw)
            dxw, dyw = x - lx, y - ly
            c, s = math.cos(-lyaw), math.sin(-lyaw)
            dxb = dxw * c - dyw * s
            dyb = dxw * s + dyw * c
            dyaw = math.atan2(math.sin(yaw - lyaw), math.cos(yaw - lyaw))
            # odometry noise (v3: the run's active tier)
            dxb *= 1.0 + self.rng.gauss(0.0, self.nv)
            dyb *= 1.0 + self.rng.gauss(0.0, self.nv)
            dyaw = dyaw * (1.0 + self.rng.gauss(0.0, self.nw)) + self.nb * dt
            # integration in the odom frame
            oc, os_ = math.cos(self.o[2]), math.sin(self.o[2])
            self.o[0] += dxb * oc - dyb * os_
            self.o[1] += dxb * os_ + dyb * oc
            self.o[2] = math.atan2(math.sin(self.o[2] + dyaw), math.cos(self.o[2] + dyaw))

            self.n += 1
            if self.log and self.n % 6 == 0:   # v3: ~10 Hz odom-vs-truth (drift measurable)
                self.log.write(f"{self.o[0]:.3f},{self.o[1]:.3f},{self.o[2]:.3f},"
                               f"{x:.3f},{y:.3f},{yaw:.3f}\n")
            if self.n % PUBLISH_EVERY:
                return
            now = time.time()
            wall_dt = max(now - self.last_wall, 1e-3)
            self.last_wall = now
            stamp = self.node.get_clock().now().to_msg()
            qz, qw = math.sin(self.o[2] / 2.0), math.cos(self.o[2] / 2.0)
            od = Odometry()
            od.header.stamp = stamp
            od.header.frame_id = "odom"
            od.child_frame_id = "base_link"
            od.pose.pose.position.x = self.o[0]
            od.pose.pose.position.y = self.o[1]
            od.pose.pose.orientation.z = qz
            od.pose.pose.orientation.w = qw
            # v4: velocities in SIM time (same semantics as the cmd_vel Isaac executes,
            # needed for Nav2's DWB rollouts; stamps stay wall, consistent with /scan)
            od.twist.twist.linear.x = dxb / dt
            od.twist.twist.angular.z = dyaw / dt
            self.pub.publish(od)
            if not PUBLISH_TF:
                return
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.o[0]
            t.transform.translation.y = self.o[1]
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            m = TFMessage(); m.transforms = [t]
            self.tf_pub.publish(m)

    builtins._odom_pub = _OdomPub()
    print("[odom] loaded")
