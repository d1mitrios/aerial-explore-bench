#!/usr/bin/env python3
"""ROS side shared by the wheeled arm's runners (wheeled_explore_runner.py,
wheeled_mission_runner.py): the wheeled baseline's Nav2 as the mover, the Isaac run's sim
clock as the budget and timeout clock.

  WheeledIO   rclpy node spun in a background thread:
                /aeb/sim_time (std_msgs/Float64, sim/wheeled_bootstrap.py) -> sim_t()
                TF map -> base_link (the localizer's belief: AMCL in the tours, slam_toolbox
                  while exploring - the pose Nav2's goal checker uses) -> pose()
                /odom (the baseline's noisy wheel odometry) -> odom()
                /amcl_pose (tours) -> amcl_count(); /map (exploration) -> latest_map()
                the Nav2 action navigate_to_pose -> send() / Goal
  Goal        one NavigateToPose goal: poll() -> "pending" | "active" | "rejected" |
              SUCCEEDED | ABORTED | CANCELED | STATUS_<n>, cancel(), path_m (the first
              feedback's distance_remaining), recoveries, error (Nav2's error code / message)
  RunLog      manifest events, the belief CSV, the run log - the aerial runners' formats

The runners take any object with WheeledIO's methods (missions/mock/ drives them with a
kinematic stand-in).
"""
import json
import math
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATUS_NAMES = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def install_stop_signals():
    """SIGINT and SIGTERM both end a run the way Ctrl+C does (KeyboardInterrupt: the goal is
    cancelled, the manifest written). A batch starts the runners in the background, where the
    shell leaves SIGINT ignored and SIGTERM would kill them without a manifest."""
    import signal

    def _stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, _stop)


class Goal:
    """One NavigateToPose goal, polled from the runner's loop (the futures complete in the
    executor thread)."""

    def __init__(self, io, x, y):
        from nav2_msgs.action import NavigateToPose
        self.io, self.x, self.y = io, x, y
        self.path_m = float("nan")
        self.recoveries = 0
        self.error = ""
        self.gh = None
        self.res_fut = None
        self.state = "pending"
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"                  # stamp 0 = the latest transform (the baseline's runner)
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.w = 1.0                 # yaw free: yaw_goal_tolerance 6.28 in nav2_params.yaml
        self.send_fut = io.client.send_goal_async(g, feedback_callback=self._on_feedback)

    def _on_feedback(self, fb):
        f = fb.feedback
        if self.path_m != self.path_m and f.distance_remaining > 0.0:
            self.path_m = float(f.distance_remaining)
        self.recoveries = int(getattr(f, "number_of_recoveries", 0))

    def poll(self):
        if self.state == "pending":
            if not self.send_fut.done():
                return "pending"
            self.gh = self.send_fut.result()
            if self.gh is None or not self.gh.accepted:
                self.state = "rejected"
                return "rejected"
            self.res_fut = self.gh.get_result_async()
            self.state = "active"
        if self.state == "active" and self.res_fut.done():
            res = self.res_fut.result()
            self.state = STATUS_NAMES.get(res.status, f"STATUS_{res.status}")
            r = res.result
            code, msg = getattr(r, "error_code", 0), getattr(r, "error_msg", "")
            if code or msg:
                self.error = f"{code}:{msg}".strip(":")
        return self.state

    def cancel(self, wait_s=10.0):
        """Cancel and wait (wall) for the result, so the next goal never races this one."""
        if self.state != "active" or self.gh is None:
            return
        self.gh.cancel_goal_async()
        t0 = time.time()
        while time.time() - t0 < wait_s and not self.res_fut.done():
            time.sleep(0.05)
        self.poll()


class WheeledIO:
    def __init__(self, log, node_name="wheeled_runner", map_topic=None, amcl_topic=None, odom_topic="/odom",
                 clock_topic="/aeb/sim_time", action="navigate_to_pose"):
        import rclpy
        import tf2_ros
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float64
        from nav_msgs.msg import Odometry
        self.rclpy, self.log = rclpy, log
        rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.lock = threading.Lock()
        self._sim = None                  # (sim_t, wall_rx)
        self._odom = None                 # (x, y, yaw, wall_rx)
        self._amcl_n = 0
        self._amcl = None                 # (x, y, yaw, cov_xx, cov_yy, wall_rx)
        self._map, self._map_n = None, 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(Float64, clock_topic, self._on_clock, 20)
        self.node.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        if amcl_topic:
            from geometry_msgs.msg import PoseWithCovarianceStamped
            # AMCL publishes amcl_pose latched (transient local, depth 1): a late subscriber still
            # gets the pose of a robot standing at the spawn, which AMCL does not republish
            self.node.create_subscription(PoseWithCovarianceStamped, amcl_topic, self._on_amcl, latched)
        if map_topic:
            from nav_msgs.msg import OccupancyGrid
            self.node.create_subscription(OccupancyGrid, map_topic, self._on_map, latched)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self.node, spin_thread=False)
        self.client = ActionClient(self.node, NavigateToPose, action)
        from geometry_msgs.msg import Twist
        self._Twist = Twist
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)   # the dart only: Nav2 idle
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        log(f"[ros] {clock_topic} (sim clock), TF map->base_link (belief), {odom_topic}"
            f"{', ' + amcl_topic if amcl_topic else ''}{', ' + map_topic if map_topic else ''}, action {action}")

    # ------------------------------------------------------------ callbacks
    def _on_clock(self, m):
        with self.lock:
            self._sim = (float(m.data), time.time())

    def _on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        with self.lock:
            self._odom = (p.x, p.y, yaw_of(q.x, q.y, q.z, q.w), time.time())

    def _on_amcl(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        c = m.pose.covariance
        with self.lock:
            self._amcl_n += 1
            self._amcl = (p.x, p.y, yaw_of(q.x, q.y, q.z, q.w), c[0], c[7], time.time())

    def _on_map(self, m):
        with self.lock:
            self._map, self._map_n = m, self._map_n + 1

    # ------------------------------------------------------------ accessors
    def sim_t(self):
        """Latest sim time (s since Play) or None."""
        with self.lock:
            return self._sim[0] if self._sim else None

    def sim_age(self):
        """Wall seconds since the last sim-clock message (inf: never)."""
        with self.lock:
            return time.time() - self._sim[1] if self._sim else float("inf")

    def pose(self):
        """The belief (x, y, yaw) in the map frame: TF map->base_link, latest; None if unknown."""
        from rclpy.time import Time
        try:
            t = self.tf.lookup_transform("map", "base_link", Time())
        except Exception:  # noqa: BLE001 - not yet available / extrapolation
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return (tr.x, tr.y, yaw_of(q.x, q.y, q.z, q.w))

    def odom(self):
        with self.lock:
            return self._odom[:3] if self._odom else None

    def amcl(self):
        with self.lock:
            return self._amcl, self._amcl_n

    def latest_map(self):
        with self.lock:
            return self._map, self._map_n

    def nav_ready(self, timeout_s):
        return self.client.wait_for_server(timeout_sec=timeout_s)

    def send(self, x, y):
        return Goal(self, x, y)

    def cmd(self, v, w=0.0):
        """A direct /cmd_vel (the robot's OmniGraph holds the last one): only while Nav2 has no goal."""
        t = self._Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def close(self):
        try:
            self.executor.shutdown()
            self.thread.join(timeout=3.0)
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


class RunLog:
    """The run's manifest, event log and belief CSV (the aerial runners' formats)."""

    def __init__(self, seed, run_id, out_dir, log, params, kind):
        self.seed, self.run_id, self.out_dir, self.log = seed, run_id, out_dir, log
        self.manifest_path = os.path.join(out_dir, f"manifest_{seed}_{run_id}.json")
        self.manifest = dict(seed=seed, run_id=run_id, embodiment="wheeled", kind=kind,
                             wall_start=time.strftime("%Y-%m-%d %H:%M:%S"), params=params, events=[], results=[])
        self.io = None
        self.belief_f = open(os.path.join(out_dir, f"belief_{seed}_{run_id}.csv"), "w", buffering=1)
        self.belief_f.write("# the belief the runner consumed (TF map->base_link: AMCL in tours, slam_toolbox while "
                            "exploring), map frame, t = /aeb/sim_time; odom_* = the wheel odometry /odom (odom frame)\n")
        self.belief_f.write("t,wall,x,y,yaw,odom_x,odom_y,odom_yaw\n")
        self._belief_next = 0.0

    def event(self, name, **kw):
        sim = self.io.sim_t() if self.io else None
        rec = dict(event=name, wall=time.time(), sim=(round(sim, 3) if sim is not None else None), **kw)
        self.manifest["events"].append(rec)
        self.log(f"[exec] {name} " + " ".join(f"{k}={v}" for k, v in kw.items()))

    def belief(self, every_wall_s=0.2):
        """One belief row at most every every_wall_s (called from the runner's loop)."""
        now = time.time()
        if now < self._belief_next or self.io is None:
            return
        self._belief_next = now + every_wall_s
        p, o, s = self.io.pose(), self.io.odom(), self.io.sim_t()
        f = (lambda v: f"{v:.4f}")
        self.belief_f.write(f"{s if s is not None else float('nan'):.3f},{now:.3f},"
                            + (",".join(f(v) for v in p) if p else "nan,nan,nan") + ","
                            + (",".join(f(v) for v in o) if o else "nan,nan,nan") + "\n")

    def save(self, status):
        self.manifest["status"] = status
        self.manifest["wall_end"] = time.strftime("%Y-%m-%d %H:%M:%S")
        with open(self.manifest_path, "w") as f:
            json.dump(self.manifest, f, indent=1)
        self.log(f"[exec] manifest -> {self.manifest_path}")
        try:
            self.belief_f.close()
        except Exception:  # noqa: BLE001
            pass


def exit_now(rc):
    """Leave without the interpreter teardown: rclpy's C++ side can abort there ("terminate
    called without an active exception", the bring-up check 2026-09-26) - the manifest and the
    CSVs are closed by then."""
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)


def make_logger(path):
    f = open(path, "a", buffering=1)

    def log(msg):
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        f.write(line + "\n")
    return log


def wait_sim(io, dt_sim, cap_wall_s, min_rtf=0.05, poll=0.05, tick=None):
    """Sleep dt_sim seconds of SIM time (wall cap = dt_sim / min_rtf, also cap_wall_s)."""
    s0, w0 = io.sim_t(), time.time()
    cap = min(cap_wall_s, dt_sim / min_rtf)
    while time.time() - w0 < cap:
        s = io.sim_t()
        if s is not None and s0 is not None and s - s0 >= dt_sim:
            return True
        if s0 is None:
            s0 = s
        if tick:
            tick()
        time.sleep(poll)
    return False


def gates(io, rl, need_map, need_amcl, wait_wall_s, stale_s):
    """The readiness gates before the policy clock (their time is not counted, as the aerial
    arm's dart and gates): the sim clock ticking, Nav2's action server, the map (the
    exploration's live map) or a first AMCL pose (tours), and the belief TF map->base_link.
    Returns None when all pass, else the failing gate's status."""
    t0 = time.time()
    while io.sim_t() is None or io.sim_age() > stale_s:
        if time.time() - t0 > wait_wall_s:
            return "NO_SIM_CLOCK"
        time.sleep(0.2)
    s0, w0 = io.sim_t(), time.time()
    time.sleep(2.0)
    s1, w1 = io.sim_t(), time.time()
    rl.event("sim_clock", rtf=round((s1 - s0) / (w1 - w0), 3))
    if not io.nav_ready(max(1.0, wait_wall_s - (time.time() - t0))):
        return "NAV2_NOT_READY"
    rl.event("nav2_ready")
    if need_map:
        while io.latest_map()[0] is None:
            if time.time() - t0 > wait_wall_s:
                return "NO_MAP"
            time.sleep(0.2)
        rl.event("map_ready")
    if need_amcl:                     # informational: wheeled_nav.sh gated on /amcl_pose already
        t1 = time.time()
        while io.amcl()[1] == 0 and time.time() - t1 < 20.0:
            time.sleep(0.2)
        a = io.amcl()[0]
        if a:
            rl.event("amcl_initialized", x=round(a[0], 3), y=round(a[1], 3), cov_xx=round(a[3], 4), cov_yy=round(a[4], 4))
        else:
            rl.event("amcl_pose_not_seen", note="no latched /amcl_pose within 20 s; the belief TF decides")
    while io.pose() is None:
        if time.time() - t0 > wait_wall_s:
            return "NO_BELIEF_TF"
        time.sleep(0.2)
    p = io.pose()
    rl.event("belief_ready", x=round(p[0], 3), y=round(p[1], 3), yaw_deg=round(math.degrees(p[2]), 1),
             gates_wall_s=round(time.time() - t0, 1))
    return None
