# === Isaac Sim 6.0.x : DIRECT PhysX-raycast 2D lidar -> ROS 2 /scan ===
# Uses the stable PhysX scene-query raycast (the experimental RaycastSensor is broken).
# Ignores hits on the robot itself (prims under /my_custom_robot).
# IMPORTANT: obstacles must NOT be children of /my_custom_robot (put them under /World),
#            otherwise they are treated as "self" and ignored.
#
# USE: Window > Script Editor -> paste -> Run -> press Play. Then in WSL2: ros2 topic echo /scan
#
# v3 (29/7): run-once guard + STAGE REBIND on every PLAY. The double bootstrap reopened
# the stage and left this publisher casting from an EXPIRED stage handle
# (Boost ArgumentError in GetPrimAtPath, zero /scan). The raycast stays UNCHANGED.
import math
import builtins
import omni.usd
import omni.physx
import omni.timeline
from pxr import UsdGeom
from omni.physx import get_physx_scene_query_interface
import carb
import rclpy
from sensor_msgs.msg import LaserScan

N            = 360
A_MIN        = -math.pi
A_INC        = 2.0 * math.pi / N
RMIN, RMAX   = 0.1, 100.0
LIDAR_PATH   = "/my_custom_robot/Geometry/chassis/lidar_link"  # cast origin
SELF_PREFIX  = "/my_custom_robot"      # ignore hits on the robot's own body
PUBLISH_EVERY = 3                      # ~20 Hz if physics is 60 Hz

if getattr(builtins, "_scan_pub", None):
    print("[scan] already running - guard skip")
else:
    _S = {"stage": omni.usd.get_context().get_stage(), "warned": False}
    q = get_physx_scene_query_interface()

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("isaac_scan_pub")
    pub  = node.create_publisher(LaserScan, "/scan", 10)

    def _rebind(reason):
        _S["stage"] = omni.usd.get_context().get_stage()
        print(f"[scan] stage handle rebound ({reason})")

    def _lidar_pose():
        m = UsdGeom.Xformable(_S["stage"].GetPrimAtPath(LIDAR_PATH)).ComputeLocalToWorldTransform(0)
        return m.ExtractTranslation(), m.ExtractRotationMatrix()

    def _cast(ox, oy, oz, dx, dy, dz):
        # raycast and skip self-hits by advancing the origin past them
        acc = 0.0
        for _ in range(5):
            hit = q.raycast_closest(carb.Float3(ox, oy, oz), carb.Float3(dx, dy, dz), RMAX - acc)
            if not hit["hit"]:
                return RMAX
            dist = float(hit["distance"])
            if not str(hit.get("collision", "")).startswith(SELF_PREFIX):
                return acc + dist                      # real obstacle
            step = dist + 0.02                          # self hit -> step past it
            ox, oy, oz = ox + dx*step, oy + dy*step, oz + dz*step
            acc += step
        return RMAX

    _cnt = {"n": 0}
    def _on_step(dt):
        _cnt["n"] += 1
        if _cnt["n"] % PUBLISH_EVERY != 0:
            return
        try:
            p, R = _lidar_pose()
        except Exception:
            try:
                _rebind("invalid handle in step")      # v3: self-heal instead of an error wall
                p, R = _lidar_pose()
            except Exception:
                if not _S["warned"]:
                    _S["warned"] = True
                    print("[scan] WARN: lidar prim unavailable - skipping until it returns")
                return
        _S["warned"] = False
        ox, oy, oz = float(p[0]), float(p[1]), float(p[2]) + 0.03
        ranges = []
        for i in range(N):
            a = A_MIN + A_INC * i
            lx, ly = math.cos(a), math.sin(a)
            dx = R[0][0]*lx + R[1][0]*ly
            dy = R[0][1]*lx + R[1][1]*ly
            dz = R[0][2]*lx + R[1][2]*ly
            r = _cast(ox, oy, oz, dx, dy, dz)
            ranges.append(r if r > RMIN else RMAX)
        msg = LaserScan()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "lidar_link"
        msg.angle_min = float(A_MIN)
        msg.angle_max = float(A_MIN + A_INC * (N - 1))
        msg.angle_increment = float(A_INC)
        msg.range_min = RMIN
        msg.range_max = RMAX
        msg.ranges = ranges
        pub.publish(msg)

    def _on_tl(e):
        if e.type == int(omni.timeline.TimelineEventType.PLAY):
            _rebind("PLAY")

    sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_step)
    tl_sub = omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop(_on_tl)
    builtins._scan_pub = {"sub": sub, "tl_sub": tl_sub, "node": node, "pub": pub}
    print("[scan] v3 direct-PhysX-raycast publisher running (stage-rebind on PLAY) -- press Play")
