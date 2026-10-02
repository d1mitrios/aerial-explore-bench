#!/usr/bin/env python3
"""AERIAL-EXPLORE-BENCH A6: record OpenVINS output to CSV for the drift plot.

Subscribes to /poseimu (geometry_msgs/PoseWithCovarianceStamped, the ov_msckf
IMU pose in the VIO world frame) and appends rows to a CSV on the Windows side
(via /mnt/c) so the analysis step can pick it up next to the ground-truth CSV.

Run in WSL (ROS 2 Jazzy sourced):  python3 vio_to_csv.py
Stop with Ctrl+C when the flight is over.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

OUT = "/mnt/c/PegasusSimulator/a6_vio.csv"


class VioRecorder(Node):
    def __init__(self):
        super().__init__("a6_vio_recorder")
        self.f = open(OUT, "w", buffering=1)
        self.f.write("# OpenVINS /poseimu (VIO world frame)\n")
        self.f.write("t,x,y,z,qx,qy,qz,qw\n")
        self.n = 0
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, "/poseimu", self.cb, 10)
        self.get_logger().info(f"recording /poseimu -> {OUT}")

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.f.write(f"{t:.6f},{p.x:.6f},{p.y:.6f},{p.z:.6f},"
                     f"{q.x:.6f},{q.y:.6f},{q.z:.6f},{q.w:.6f}\n")
        self.n += 1
        if self.n % 100 == 0:
            self.get_logger().info(f"{self.n} poses recorded")


def main():
    rclpy.init()
    rec = VioRecorder()
    try:
        rclpy.spin(rec)
    except KeyboardInterrupt:
        pass
    rec.f.close()
    print(f"\nsaved {rec.n} poses -> {OUT}")


if __name__ == "__main__":
    main()
