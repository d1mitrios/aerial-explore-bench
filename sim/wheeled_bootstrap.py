# === Wheeled arm, Isaac side: one run of the wheeled baseline's robot in a seeded world ===
# Executed at Isaac launch (the full GUI app, the way the baseline starts):
#     isaac-sim.bat --exec <repo>\sim\wheeled_bootstrap.py        (sim\launch_wheeled.ps1)
# A fresh Isaac process per run (as the aerial arm), so there is no Stop/Play re-binding.
#
#   1. opens the robot stage sim/wheeled/robot.usda (the baseline's stage, copied into this
#      repo: the 0.42 m diff-drive robot, its /Graph/ROS_DiffDrive OmniGraph cmd_vel ->
#      DifferentialController, the arena's four boundary walls); the stage is never saved
#   2. rebuilds the world from worlds/manifests/world_<seed>.csv with the aerial app's rules
#      (sim/aeb_flight.py build_world): box / cyl rows = 2.0 m prisms with colliders under
#      /Arena; door / gate / path rows = metadata; person rows skipped (no pedestrians in the benchmark)
#   3. starts the baseline's own publishers (sim/wheeled/, copies): scan_raycast_publisher2.py
#      (/scan, the PhysX-raycast lidar, unchanged) and odom_publisher.py (/odom + TF with the
#      noise 0.03 / 0.05 / 0; only its two file locations come from the environment)
#   4. adds this repo's run services on the physics step: the ground truth (10 Hz, both clocks,
#      the aerial CSV format: <RunDir>/wheeled_gt_<seed>_<ts>.csv), /aeb/sim_time (std_msgs/
#      Float64, the physics-step sum since Play: the WSL runners' budget clock) and the one-line
#      status file + stop file of the batch runner (AEB_STATUS_FILE / AEB_STOP_FILE, the
#      protocol of sim/aeb_flight.py)
#   5. presses Play
# Environment (set by sim/launch_wheeled.ps1): AEB_REPO, AEB_WORLD_SEED, AEB_RUNS_DIR,
# AEB_STATUS_FILE, AEB_STOP_FILE, AEB_ODOM_TF, and the ROS 2 variables (domain 0, FastDDS with
# sim/fastdds_win.xml; unlike sim/env.ps1, no ROS library folder on PATH).
# Status line states: loading -> playing -> closing -> closed (or error, with the reason).
import asyncio
import builtins
import os
import time
import traceback


def _repo():
    r = os.environ.get("AEB_REPO", "")
    if not r:
        try:
            r = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        except NameError:          # --exec without __file__ and no AEB_REPO: fail in _boot
            r = ""
    return r.replace("\\", "/")


REPO = _repo()
SEED = os.environ.get("AEB_WORLD_SEED", "20260723008")
RUN_DIR = os.environ.get("AEB_RUNS_DIR", REPO + "/runs/raw").replace("\\", "/")
STATUS_FILE = os.environ.get("AEB_STATUS_FILE") or None
STOP_FILE = os.environ.get("AEB_STOP_FILE") or None
WHEELED = REPO + "/sim/wheeled"
ROBOT_USD = WHEELED + "/robot.usda"
MANIFEST = f"{REPO}/worlds/manifests/world_{SEED}.csv"
POSE_PATH = "/my_custom_robot/Geometry/chassis/lidar_link"   # the baseline's pose prim (= the lidar)
ARENA = "/Arena"
KEEP = ("boundary_", "SunLight", "DomeLight")
GT_EVERY = 6                     # 60 Hz physics -> 10 Hz ground truth (the baseline's metrics logger)
STATUS_EVERY = 30                # ~0.5 sim-s
CLOCK_EVERY = 3                  # /aeb/sim_time at ~20 Hz of sim time


def _log(msg):
    print(f"[wheeled] {msg}", flush=True)


def _write_status(line):
    if not STATUS_FILE:
        return
    try:
        tmp = STATUS_FILE + ".tmp"
        with open(tmp, "w") as f:
            f.write(line + "\n")
        os.replace(tmp, STATUS_FILE)
    except Exception:
        pass


def _color_for(name):            # sim/aeb_flight.py's palette (visual only; the lidar ignores it)
    if name.startswith("partition_"):
        return (0.55, 0.35, 0.65)
    if name.startswith("gate_"):
        return (0.95, 0.45, 0.10)
    return (0.60, 0.60, 0.60)


def _build_world(stage):
    """Clear the stage's arena (keep the boundary walls and the lights) and rebuild it from the
    manifest - sim/aeb_flight.py build_world's rules, under /Arena where the baseline's lidar
    sees everything that is not the robot."""
    from pxr import UsdGeom, UsdPhysics, Gf
    arena = stage.GetPrimAtPath(ARENA)
    removed = 0
    for child in list(arena.GetChildren()):
        if not child.GetName().startswith(KEEP):
            stage.RemovePrim(child.GetPath())
            removed += 1
    walls = sorted(c.GetName() for c in arena.GetChildren() if c.GetName().startswith("boundary_"))
    if len(walls) != 4:
        raise RuntimeError(f"the robot stage should carry 4 boundary walls under {ARENA}, found {walls}")

    def box(name, pos, size, yaw_deg):
        c = UsdGeom.Cube.Define(stage, f"{ARENA}/{name}")
        c.GetSizeAttr().Set(1.0)
        c.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
        xf = UsdGeom.Xformable(c.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        if yaw_deg:
            xf.AddRotateZOp().Set(yaw_deg)
        xf.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        c.GetDisplayColorAttr().Set([Gf.Vec3f(*_color_for(name))])

    def cyl(name, pos, radius, height):
        c = UsdGeom.Cylinder.Define(stage, f"{ARENA}/{name}")
        c.GetRadiusAttr().Set(radius)
        c.GetHeightAttr().Set(height)
        c.GetAxisAttr().Set("Z")
        c.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -height / 2), Gf.Vec3f(radius, radius, height / 2)])
        xf = UsdGeom.Xformable(c.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        c.GetDisplayColorAttr().Set([Gf.Vec3f(*_color_for(name))])

    n = meta = persons = 0
    with open(MANIFEST, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("type,"):
                continue
            p = line.split(",")
            if len(p) < 7:
                _log(f"manifest row ignored (short): {line}")
                continue
            if p[0] == "box":
                box(p[1], (float(p[2]), float(p[3]), 1.0), (float(p[4]), float(p[5]), 2.0), float(p[6]))
                n += 1
            elif p[0] == "cyl":
                cyl(p[1], (float(p[2]), float(p[3]), 1.0), float(p[4]), float(p[5]))
                n += 1
            elif p[0] == "person":
                persons += 1           # pedestrians deleted in every benchmark run
            else:
                meta += 1              # door / xdoor / path rows: metadata, not geometry
    _log(f"world {SEED} rebuilt from {MANIFEST}: {n} prims + the 4 boundary walls, {meta} metadata rows "
         f"kept out, {persons} person rows skipped, {removed} old arena prims removed")
    return n


class RunServices:
    """Ground truth, /aeb/sim_time and the status line, all on the physics step."""

    def __init__(self, stage):
        import omni.physx
        import omni.timeline
        import rclpy
        from pxr import UsdGeom
        from std_msgs.msg import Float64
        self.Float64 = Float64
        self.tl = omni.timeline.get_timeline_interface()
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("aeb_wheeled_services")
        self.clock_pub = self.node.create_publisher(Float64, "/aeb/sim_time", 10)
        prim = stage.GetPrimAtPath(POSE_PATH)
        if not prim.IsValid():
            raise RuntimeError(f"pose prim {POSE_PATH} not found - is the robot's payload loaded?")
        self.xf = UsdGeom.Xformable(prim)
        self.steps = 0
        self.sim_t = 0.0
        self.wall0 = None
        self.rate_mark = None
        self.rtf_now = 0.0
        self.pose = (0.0, 0.0, 0.0)
        self.state = "loading"
        self.stopping = False
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.gt_path = f"{RUN_DIR}/wheeled_gt_{SEED}_{ts}.csv"
        self.gt = open(self.gt_path, "w", buffering=1)
        self.gt.write(f"# seed={SEED} wall_start={time.strftime('%Y-%m-%d %H:%M:%S')} fmt=gt1 frames=ENU/FLU "
                      f"pose={POSE_PATH}\nsim_t,wall_t,x,y,z,qx,qy,qz,qw\n")
        self.sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)
        _log(f"ground truth -> {self.gt_path} at 10 Hz; /aeb/sim_time at ~20 Hz; "
             f"status {STATUS_FILE}; stop file {STOP_FILE}")

    def _on_step(self, dt):
        if self.stopping or not self.tl.is_playing():
            return
        if self.wall0 is None:
            self.wall0 = time.time()
            self.rate_mark = (self.sim_t, self.wall0)
            self.state = "playing"
        self.steps += 1
        self.sim_t += dt
        if self.steps % CLOCK_EVERY == 0:
            m = self.Float64()
            m.data = self.sim_t
            self.clock_pub.publish(m)
        if self.steps % GT_EVERY == 0:
            try:
                mt = self.xf.ComputeLocalToWorldTransform(0)
                tr = mt.ExtractTranslation()
                q = mt.ExtractRotationQuat()
                im = q.GetImaginary()
                self.pose = (float(tr[0]), float(tr[1]), float(tr[2]))
                self.gt.write(f"{self.sim_t:.3f},{time.time():.3f},{tr[0]:.4f},{tr[1]:.4f},{tr[2]:.4f},"
                              f"{im[0]:.6f},{im[1]:.6f},{im[2]:.6f},{q.GetReal():.6f}\n")
            except Exception:
                pass
        if self.steps % STATUS_EVERY == 0:
            now = time.time()
            s0, w0 = self.rate_mark
            if now - w0 >= 5.0:
                self.rtf_now = (self.sim_t - s0) / (now - w0)
                self.rate_mark = (self.sim_t, now)
            self.write_status()

    def write_status(self):
        wall = time.time() - self.wall0 if self.wall0 else 0.0
        x, y, _ = self.pose
        _write_status(f"state={self.state} t={time.time():.1f} pid={os.getpid()} frame={self.steps} "
                      f"sim_t={self.sim_t:.2f} wall={wall:.1f} rtf_now={self.rtf_now:.3f} "
                      f"x={x:.2f} y={y:.2f}")

    def close(self):
        self.stopping = True
        try:
            self.gt.close()
        except Exception:
            pass


async def _boot():
    import omni.kit.app
    import omni.timeline
    import omni.usd
    app = omni.kit.app.get_app()
    _write_status(f"state=loading t={time.time():.1f} pid={os.getpid()} frame=0 sim_t=0.00 wall=0.0 rtf_now=0.000")
    for _ in range(30):
        await app.next_update_async()
    err = None
    for _ in range(300):                             # the ROS 2 extension's internal rclpy (autoload)
        try:
            import rclpy  # noqa: F401
            from rclpy.impl.implementation_singleton import rclpy_implementation  # noqa: F401 - the C extension
            err = None
            break
        except ImportError as exc:
            err = exc
            await app.next_update_async()
    if err is not None:
        raise RuntimeError(f"rclpy not importable in the app: {err} (a ROS library folder on PATH? "
                           f"see sim/launch_wheeled.ps1)")
    _log(f"rclpy available; ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '<unset>')} "
         f"RMW={os.environ.get('RMW_IMPLEMENTATION', '<unset>')} "
         f"profile={os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', '<unset>')}")
    for path in (ROBOT_USD, MANIFEST):
        if not os.path.isfile(path):
            raise RuntimeError(f"missing {path} (AEB_REPO={REPO!r})")
    os.makedirs(RUN_DIR, exist_ok=True)
    with open(f"{RUN_DIR}/current_seed.txt", "w") as f:   # names the odom publisher's log
        f.write(SEED + "\n")
    ctx = omni.usd.get_context()
    ctx.open_stage(ROBOT_USD)
    for _ in range(60):
        await app.next_update_async()
    stage = ctx.get_stage()
    _log(f"stage {stage.GetRootLayer().identifier} (never saved by this run)")
    if stage.GetPrimAtPath("/Graph/ROS_Camera"):     # the baseline's raw-camera graph: not used
        stage.RemovePrim("/Graph/ROS_Camera")
    if not stage.GetPrimAtPath("/Graph/ROS_DiffDrive").IsValid():
        raise RuntimeError("the robot stage has no /Graph/ROS_DiffDrive (cmd_vel) graph")
    _build_world(stage)
    ns = {}
    for name in ("scan_raycast_publisher2.py", "odom_publisher.py"):   # the baseline's publishers
        path = f"{WHEELED}/{name}"
        with open(path, encoding="utf-8") as fh:
            exec(compile(fh.read(), path, "exec"), ns)
    _log(f"odometry noise file: {os.environ.get('AEB_ODOM_NOISE_FILE') or 'none - the publisher defaults 0.03 / 0.05 / 0 apply'}")
    svc = RunServices(stage)
    builtins._aeb_wheeled = (ns, svc)                # keep everything alive
    svc.write_status()
    for _ in range(10):
        await app.next_update_async()
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    _log("PLAY pressed")
    # stop-file watcher (the batch runner's clean stop)
    while True:
        for _ in range(30):
            await app.next_update_async()
        if STOP_FILE and os.path.exists(STOP_FILE):
            _log(f"stop file found - closing (steps={svc.steps}, sim_t={svc.sim_t:.1f} s)")
            svc.state = "closing"
            svc.write_status()
            tl.stop()
            for _ in range(20):
                await app.next_update_async()
            svc.close()
            try:
                ctx.close_stage()                     # discard the run's edits: no save prompt at exit
            except Exception as exc:                  # noqa: BLE001
                _log(f"close_stage: {exc}")
            svc.state = "closed"
            svc.write_status()
            _log("closed - quitting the app")
            for _ in range(5):
                await app.next_update_async()
            app.post_quit()
            return


async def _main():
    try:
        await _boot()
    except Exception as exc:                          # noqa: BLE001
        _log("BOOT FAILED: " + "".join(traceback.format_exception(type(exc), exc, exc.__traceback__)))
        _write_status(f"state=error t={time.time():.1f} pid={os.getpid()} reason={type(exc).__name__}:"
                      + str(exc).replace(" ", "_")[:200])


asyncio.ensure_future(_main())
