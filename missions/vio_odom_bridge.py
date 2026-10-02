#!/usr/bin/env python3
"""VIO -> odometry bridge for the quadrotor's AMCL (the odometry source until 2026-09-24).

Turns the OpenVINS pose stream (/poseimu, PoseWithCovarianceStamped in the VIO world
frame) into what a 2D localizer expects from a wheel-odometry source, exactly the shape
the wheeled baseline's odometry publisher had:

  /odom        nav_msgs/Odometry   frame odom -> child base_link, ~VIO rate
  /tf          odom -> base_link   (x, y, yaw of the IMU/body frame; z = 0)
  /tf_static   base_link -> lidar_link  (0, 0, +0.10 m, identity rotation: the scan
               plane is stabilized yaw-only, so the lidar frame is the yawed body frame)

The odom frame is born where OpenVINS initialized (at the spawn by protocol);
AMCL's initial pose at the spawn absorbs the small offset. Stamps are the VIO stamps (the
simulation clock), the same clock as /scan, so AMCL's transform lookups at the scan stamp
are consistent. Nothing is published while VIO is silent (a stale odometry would freeze
the localizer with a lying pose).

Odometry scale (on by default): the monocular VIO's metric scale wanders
(measured 0.65-1.26 over 3 m windows, test flights of 2026-09-23), which AMCL's likelihood
field cannot correct along obliquely seen walls. The bridge therefore calibrates the
odometry online, the ratio of AMCL's displacement to the raw VIO displacement over a
4-10 m baseline, smoothed (missions/odom_scale.py), and integrates the VIO INCREMENTS
times that scale into the odom frame (a change of scale never jumps the odometry). The
yaw is passed through. `--odom-scale 1.0` fixes the scale (calibration off).

Run in WSL (ROS 2 Jazzy sourced) before or after OpenVINS, before the executive:
  python3 missions/vio_odom_bridge.py [--vio-topic /poseimu] [--lidar-z 0.10] [--odom-scale 0|<fixed>]
                                      [--pose-topic /amcl_pose|/pose]
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from odom_scale import OdomScale, ScaledIntegrator  # noqa: E402


def heading_from_quat(x, y, z, w):
    """Heading (rad) of the body x-axis projected on the horizontal plane."""
    r00 = 1 - 2 * (y * y + z * z)
    r10 = 2 * (x * y + w * z)
    return math.atan2(r10, r00)


class VioOdomBridge(Node):
    def __init__(self, vio_topic, lidar_z, log_every, odom_scale, amcl_topic="/amcl_pose"):
        super().__init__("vio_odom_bridge")
        self.n = 0
        self.log_every = log_every
        self.last = None
        self.t_wall0 = time.time()
        self.fixed_scale = odom_scale if odom_scale > 0 else None
        self.scale_est = OdomScale() if self.fixed_scale is None else None
        self.integ = ScaledIntegrator()
        self.n_amcl = 0
        self.last_logged_scale = 1.0
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = "base_link"
        st.child_frame_id = "lidar_link"
        st.transform.translation.z = float(lidar_z)
        st.transform.rotation.w = 1.0
        self.tf_static.sendTransform(st)
        self.create_subscription(PoseWithCovarianceStamped, vio_topic, self.on_vio, qos_profile_sensor_data)
        if self.scale_est is not None:
            self.create_subscription(PoseWithCovarianceStamped, amcl_topic, self.on_amcl, 10)
        mode = f"fixed scale {self.fixed_scale:.3f}" if self.fixed_scale else f"online scale from {amcl_topic}"
        self.get_logger().info(f"[vio-odom] {vio_topic} -> /odom + tf odom->base_link ({mode}); "
                               f"static base_link->lidar_link z={lidar_z:.2f}; waiting for VIO ...")

    @property
    def scale(self):
        return self.fixed_scale if self.fixed_scale is not None else self.scale_est.scale

    def on_amcl(self, m):
        p = m.pose.pose.position
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        cov = m.pose.covariance
        inst = self.scale_est.add_localizer(t, p.x, p.y, cov[0], cov[7])
        self.n_amcl += 1
        if inst is not None and abs(self.scale_est.scale - self.last_logged_scale) >= 0.02:
            self.last_logged_scale = self.scale_est.scale
            self.get_logger().info(f"[vio-odom] odom scale {self.scale_est.scale:.3f} (instantaneous {inst:.3f}, "
                                   f"estimate {self.scale_est.n_est}, amcl poses {self.n_amcl})")

    def on_vio(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = heading_from_quat(q.x, q.y, q.z, q.w)
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        stamp = m.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.scale_est is not None:
            self.scale_est.add_vio(t, p.x, p.y)
        ox, oy = self.integ.step(p.x, p.y, self.scale)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = ox
        od.pose.pose.position.y = oy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        # 2D covariance from the VIO's (x, y, yaw) terms; the rest untouched (zeros)
        cov = list(m.pose.covariance)
        od.pose.covariance[0] = cov[0]
        od.pose.covariance[7] = cov[7]
        od.pose.covariance[35] = cov[35]
        if self.last is not None:
            lt, lx, ly, lyaw = self.last
            dt = t - lt
            if dt > 1e-4:
                dxw, dyw = ox - lx, oy - ly
                c, s = math.cos(-lyaw), math.sin(-lyaw)
                od.twist.twist.linear.x = (dxw * c - dyw * s) / dt
                od.twist.twist.linear.y = (dxw * s + dyw * c) / dt
                od.twist.twist.angular.z = math.atan2(math.sin(yaw - lyaw), math.cos(yaw - lyaw)) / dt
        self.last = (t, ox, oy, yaw)
        self.pub_odom.publish(od)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = ox
        tf.transform.translation.y = oy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf.sendTransform(tf)

        self.n += 1
        if self.n == 1:
            self.get_logger().info(f"[vio-odom] VIO alive: first pose t={t:.3f} at ({p.x:.2f}, {p.y:.2f}) yaw={math.degrees(yaw):.1f} deg")
        elif self.n % self.log_every == 0:
            self.get_logger().info(f"[vio-odom] n={self.n} t_vio={t:.2f} odom=({ox:.2f}, {oy:.2f}) raw=({p.x:.2f}, {p.y:.2f}) "
                                   f"scale={self.scale:.3f} yaw={math.degrees(yaw):.1f} deg wall={time.time() - self.t_wall0:.0f}s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vio-topic", default="/poseimu")
    ap.add_argument("--lidar-z", type=float, default=0.10, help="lidar origin above the body origin (sim/lidar_raycast.py z_offset)")
    ap.add_argument("--log-every", type=int, default=200)
    ap.add_argument("--pose-topic", default="/amcl_pose",
                    help="the localizer's corrected pose the scale is calibrated against (/amcl_pose in the "
                         "mission phase, /pose = slam_toolbox in the exploration phase)")
    ap.add_argument("--odom-scale", type=float, default=0.0,
                    help="0 = calibrate the odometry scale online against /amcl_pose; "
                         "any other value = fixed scale (1.0 = the raw VIO)")
    a = ap.parse_args()
    rclpy.init()
    node = VioOdomBridge(a.vio_topic, a.lidar_z, a.log_every, a.odom_scale, amcl_topic=a.pose_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    print(f"[vio-odom] stopped after {node.n} poses")
    return 0


if __name__ == "__main__":
    sys.exit(main())
