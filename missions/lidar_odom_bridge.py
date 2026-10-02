#!/usr/bin/env python3
"""Lidar-odometry bridge for the quadrotor: what rf2o_laser_odometry needs around it.

rf2o (range-flow 2D odometry, MAPIRlab, ROS 2 branch) turns the drone's /scan into
/odom_rf2o + TF odom->base_link, the odometry the localizers (AMCL in the mission phase,
slam_toolbox in the exploration phase) and the executive consume, in the role the VIO
played until 2026-09-24. Two things it does not do for itself:

  1. misses: the Isaac lidar reports a ray that hits nothing as range_max (100 m), and
     rf2o takes every positive range as a real return (0 is its "invalid" marker); a ring
     of 100 m points would wreck the range flow. This node republishes /scan as /scan_odom
     with misses (>= range_max) and non-finite ranges set to 0.0. Everything else is
     copied verbatim (stamp, frame, angles), so rf2o's odometry carries the scan's
     simulation stamps like the rest of the chain.
  2. the static TF base_link -> lidar_link (+0.10 m, identity rotation: the scan plane is
     stabilized yaw-only), which rf2o looks up once on its first scan and AMCL / slam
     need for the scan frame.

Also reports the chain's health: /scan_odom relayed, /odom_rf2o poses seen (rate, latest
pose, sim-time gap to the scan); the terminal shows within seconds whether the odometry
is flowing. rf2o's angle convention (ray u at -fov/2 + u*fov/(n-1)) sits half a ray (0.5
deg) from the lidar's (-180 deg + u): a constant rotation of the odometry frame, absorbed
by the localizer's map->odom correction like any mount yaw.

  source /opt/ros/jazzy/setup.bash && python3 missions/lidar_odom_bridge.py [--lidar-z 0.10]
        [--scan-in /scan] [--scan-out /scan_odom] [--odom-topic /odom_rf2o] [--log-every 100]
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def heading_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LidarOdomBridge(Node):
    def __init__(self, scan_in, scan_out, odom_topic, lidar_z, log_every):
        super().__init__("lidar_odom_bridge")
        self.log_every = log_every
        self.n_scan = self.n_odom = 0
        self.last_scan_t = None
        self.last_odom = None           # (t, x, y, yaw, wall)
        self.t_wall0 = time.time()
        self.static_br = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "lidar_link"
        tf.transform.translation.z = float(lidar_z)
        tf.transform.rotation.w = 1.0
        self.static_br.sendTransform(tf)
        # rf2o subscribes best-effort: a reliable publisher (this one) is compatible either way
        self.pub = self.create_publisher(LaserScan, scan_out, 10)
        self.create_subscription(LaserScan, scan_in, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.get_logger().info(f"[lidar-odom] static TF base_link->lidar_link z={lidar_z:.2f}; "
                               f"{scan_in} -> {scan_out} (misses -> 0.0); watching {odom_topic}")

    def on_scan(self, m):
        out = LaserScan()
        out.header = m.header
        out.angle_min, out.angle_max, out.angle_increment = m.angle_min, m.angle_max, m.angle_increment
        out.time_increment, out.scan_time = m.time_increment, m.scan_time
        out.range_min, out.range_max = m.range_min, m.range_max
        lim = m.range_max - 1e-3
        out.ranges = [r if (math.isfinite(r) and m.range_min <= r < lim) else 0.0 for r in m.ranges]
        out.intensities = []
        self.pub.publish(out)
        self.n_scan += 1
        self.last_scan_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.n_scan % self.log_every == 0:
            self.report()

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.last_odom = (t, p.x, p.y, heading_from_quat(q.x, q.y, q.z, q.w), time.time())
        self.n_odom += 1

    def report(self):
        el = time.time() - self.t_wall0
        if self.last_odom is None:
            self.get_logger().info(f"[lidar-odom] scans relayed {self.n_scan} ({self.n_scan / el:.1f}/s wall), "
                                   f"no odometry yet (is rf2o up and subscribed to /scan_odom?)")
            return
        t, x, y, yaw, w = self.last_odom
        gap = (self.last_scan_t - t) if self.last_scan_t is not None else float("nan")
        self.get_logger().info(f"[lidar-odom] scans {self.n_scan} ({self.n_scan / el:.1f}/s wall), odom poses {self.n_odom} "
                               f"({self.n_odom / el:.1f}/s wall): x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f} deg, "
                               f"odom lags the scan by {gap:.2f} sim-s")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scan-in", default="/scan")
    ap.add_argument("--scan-out", default="/scan_odom")
    ap.add_argument("--odom-topic", default="/odom_rf2o")
    ap.add_argument("--lidar-z", type=float, default=0.10, help="lidar origin above the body origin (sim/lidar_raycast.py z_offset)")
    ap.add_argument("--log-every", type=int, default=100, help="report every N scans (100 = ~30 wall-s at RTF 0.17)")
    a = ap.parse_args()
    rclpy.init()
    node = LidarOdomBridge(a.scan_in, a.scan_out, a.odom_topic, a.lidar_z, a.log_every)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
