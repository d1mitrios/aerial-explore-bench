#!/usr/bin/env python
"""
| File: aeb_flight.py
| Description: Shared scaffolding for the AERIAL-EXPLORE-BENCH standalone Isaac Sim
|              flight apps (Isaac Sim 6.0.1 + the local Pegasus Isaac-6 port + PX4 SITL).
|              Everything the installation tests (docs/INSTALL_PEGASUS.md) learned is
|              consolidated here so that every later app (lidar test, exploration
|              runner, batch runner) reuses one loop:
|   - standalone loop driven at PHYSICS rate: N substeps of
|     world.step(render=False) + dc.tick + manual vehicle pipeline per rendered frame
|   - every PhysX schema (force APIs, propeller drives) applied BEFORE Play
|   - world rebuilt 1:1 from a seed manifest in worlds/manifests/ (boundary walls +
|     box/cyl rows; door/gate/xdoor rows are ground-truth metadata; person rows are
|     skipped)
|   - ROS 2 bridge enabled first (Isaac's internal humble libs must be on PATH; see
|     sim/README.md); sensor components publish with the shared sim clock as stamp
|   - optional components: VioSensorKit (IMU 250 Hz + forward camera ~20 Hz, the A6
|     pattern) and GroundTruthLogger (10 Hz out-of-band CSV with BOTH clocks, never a topic)
|
| Usage pattern (the SimulationApp must exist before any other isaacsim import):
|     import aeb_flight
|     simulation_app = aeb_flight.start_simulation_app()
|     from aeb_flight import FlightApp, VioSensorKit, GroundTruthLogger
|     app = FlightApp(tag="a7"); app.add(GroundTruthLogger()); app.run()
|
| Environment variables:
|     AEB_WORLD_SEED   seed of the world to build (default 20260723001) ->
|                      <repo>/worlds/manifests/world_<seed>.csv
|     AEB_WORLD_CSV    explicit manifest path (overrides AEB_WORLD_SEED)
|     AEB_RUNS_DIR     output directory for run CSVs (default <repo>/runs/raw, git-ignored)
|     AEB_HEADLESS     "1" -> headless Isaac (for RTF measurements / batches)
|     AEB_STATUS_FILE  path the app keeps overwritten with one status line (state, frame,
|                      sim_t, rtf_now, PX4 heartbeat) every 50 frames; the batch runner's
|                      readiness / liveness probe (sim/launch_a7.ps1 sets it)
|     AEB_STOP_FILE    when this file appears the app stops cleanly (logs closed, ROS node
|                      destroyed, Kit closed); the batch runner's stop request
|
| Adapted from the Pegasus Simulator example examples/1_px4_single_vehicle.py
| (BSD-3-Clause, Copyright (c) 2023, Marcelo Jacinto).
"""

import os
import time

import carb
from isaacsim import SimulationApp

# ------------------------------------------------------------------ paths
SIM_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SIM_DIR)
MANIFEST_DIR = os.path.join(REPO_ROOT, "worlds", "manifests")
RUNS_DIR = os.environ.get("AEB_RUNS_DIR", os.path.join(REPO_ROOT, "runs", "raw"))
STATUS_FILE = os.environ.get("AEB_STATUS_FILE") or None
STOP_FILE = os.environ.get("AEB_STOP_FILE") or None
DEFAULT_SEED = "20260723001"
ARENA_PRIM = "/World/Arena"
VEHICLE_PRIM = "/World/quadrotor"
SPAWN_POS = (0.0, 0.0, 0.07)          # generator guarantees a 2 m clear radius at (0, 0)

# ------------------------------------------------------------- app state
_app = None
ROS2_AVAILABLE = False
rclpy = None


def resolve_world_csv():
    """Manifest path from AEB_WORLD_CSV or AEB_WORLD_SEED (repo-relative, no external paths)."""
    explicit = os.environ.get("AEB_WORLD_CSV")
    if explicit:
        return explicit
    seed = os.environ.get("AEB_WORLD_SEED", DEFAULT_SEED)
    return os.path.join(MANIFEST_DIR, f"world_{seed}.csv")


def seed_from_path(csv_path):
    base = os.path.splitext(os.path.basename(csv_path))[0]
    return base[len("world_"):] if base.startswith("world_") else base


def _ros_env_report():
    """Which of the launcher's ROS variables are present (sim/env.ps1 sets all of them)."""
    path_ok = any("isaacsim.ros2.core" in p for p in os.environ.get("PATH", "").split(os.pathsep))
    prof = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    return (f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '<unset>')} "
            f"RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION', '<unset>')} "
            f"FASTRTPS_DEFAULT_PROFILES_FILE={'ok' if prof and os.path.isfile(prof) else ('MISSING FILE ' + prof if prof else '<unset>')} "
            f"humble-lib-on-PATH={'yes' if path_ok else 'NO'}")


def start_simulation_app(headless=None, require_ros=False):
    """Create the SimulationApp (once), enable the ROS 2 bridge, import the sim modules.
    With require_ros=True the app refuses to run when rclpy cannot be imported: a flight
    without its sensor topics is a wasted flight, not a degraded one."""
    global _app, ROS2_AVAILABLE, rclpy
    if _app is not None:
        return _app
    if headless is None:
        headless = os.environ.get("AEB_HEADLESS", "0") == "1"
    _app = SimulationApp({"headless": headless})

    # ROS 2 bridge FIRST: it loads Isaac's internal humble libs (the launcher must
    # have added ...\exts\isaacsim.ros2.core\humble\lib to PATH, else error 126).
    from isaacsim.core.utils.extensions import enable_extension
    ok = enable_extension("isaacsim.ros2.bridge")
    carb.log_warn(f"[aeb] enable_extension(isaacsim.ros2.bridge) -> {ok} (headless={headless})")
    carb.log_warn(f"[aeb] ROS env: {_ros_env_report()}")
    for _ in range(10):
        _app.update()
    try:
        import rclpy as _rclpy
        rclpy = _rclpy
        ROS2_AVAILABLE = True
        carb.log_warn("[aeb] rclpy import OK")
    except Exception as exc:  # noqa: BLE001
        msg = (f"[aeb] rclpy unavailable ({exc}). The ROS variables must be set in THIS PowerShell "
               f"window before python.bat; dot-source sim\\env.ps1. Env seen: {_ros_env_report()}")
        if require_ros:
            carb.log_error(msg + "; REFUSING TO FLY (ROS required for this run)")
            print(msg, flush=True)
            _app.close()
            raise SystemExit(2)
        carb.log_warn(msg + "; flying WITHOUT ROS output")
    _import_sim_modules()
    return _app


def _import_sim_modules():
    """Imports that are only legal after the SimulationApp exists (module globals)."""
    global np, omni_timeline, omni_usd, Gf, PhysxSchema, UsdGeom, UsdPhysics, World, Camera
    global ROBOTS, SIMULATION_ENVIRONMENTS, PX4MavlinkBackend, PX4MavlinkBackendConfig
    global Multirotor, MultirotorConfig, acquire_dynamic_control_interface, PegasusInterface, Rotation
    import numpy as np
    import omni.timeline as omni_timeline
    import omni.usd as omni_usd
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from isaacsim.core.api.world import World          # Isaac 6 location
    from isaacsim.sensors.camera import Camera
    from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
    from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
    from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
    from pegasus.simulator.logic.vehicles.dc_shim import acquire_dynamic_control_interface
    from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
    from scipy.spatial.transform import Rotation


# ------------------------------------------------------------ world build
def _add_box(stage, name, pos, size, yaw_deg=0.0, color=(0.6, 0.6, 0.6)):
    """Box helper with the world generator's exact semantics (center pos, full size, yaw)."""
    c = UsdGeom.Cube.Define(stage, ARENA_PRIM + "/" + name)
    c.GetSizeAttr().Set(1.0)
    c.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    xf = UsdGeom.Xformable(c.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if yaw_deg:
        xf.AddRotateZOp().Set(yaw_deg)
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    c.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])


def _add_cyl(stage, name, pos, radius, height, color=(0.6, 0.6, 0.6)):
    c = UsdGeom.Cylinder.Define(stage, ARENA_PRIM + "/" + name)
    c.GetRadiusAttr().Set(radius)
    c.GetHeightAttr().Set(height)
    c.GetAxisAttr().Set("Z")
    c.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -height / 2.0),
                        Gf.Vec3f(radius, radius, height / 2.0)])
    xf = UsdGeom.Xformable(c.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    c.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])


def _color_for(name):
    if name.startswith("partition_"):
        return (0.55, 0.35, 0.65)
    if name.startswith("gate_"):
        return (0.95, 0.45, 0.10)
    if name.startswith("boundary_"):
        return (0.40, 0.40, 0.40)
    return (0.60, 0.60, 0.60)


def build_world(csv_path, tag="aeb"):
    """Rebuild a seeded world 1:1 from its manifest: 4 boundary walls (20x20 m arena,
    0.5 m thick, 2.0 m tall) + every box/cyl row (2.0 m tall prisms). Returns prim count."""
    stage = omni_usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, ARENA_PRIM)
    _add_box(stage, "boundary_north", (0, 10, 1), (20, 0.5, 2), 0.0, _color_for("boundary_"))
    _add_box(stage, "boundary_south", (0, -10, 1), (20, 0.5, 2), 0.0, _color_for("boundary_"))
    _add_box(stage, "boundary_east", (10, 0, 1), (0.5, 20, 2), 0.0, _color_for("boundary_"))
    _add_box(stage, "boundary_west", (-10, 0, 1), (0.5, 20, 2), 0.0, _color_for("boundary_"))
    n, meta, persons = 4, 0, 0
    with open(csv_path, "r") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("type,"):
                continue
            parts = line.split(",")
            if len(parts) < 7:
                carb.log_warn(f"[{tag}] manifest row ignored (short): {line}")
                continue
            typ, name = parts[0], parts[1]
            if typ == "box":
                x, y = float(parts[2]), float(parts[3])
                p1, p2, yaw = float(parts[4]), float(parts[5]), float(parts[6])
                _add_box(stage, name, (x, y, 1.0), (p1, p2, 2.0), yaw, _color_for(name))
                n += 1
            elif typ == "cyl":
                x, y = float(parts[2]), float(parts[3])
                p1, p2 = float(parts[4]), float(parts[5])
                _add_cyl(stage, name, (x, y, 1.0), p1, p2, _color_for(name))
                n += 1
            elif typ == "person":
                persons += 1   # pedestrians deleted in every benchmark run
            else:
                meta += 1      # door / xdoor / path rows: ground-truth metadata, not geometry
    carb.log_warn(f"[{tag}] world rebuilt from {os.path.basename(csv_path)}: "
                  f"{n} prims, {meta} metadata rows kept out, {persons} person rows skipped")
    return n


# ------------------------------------------------------------- components
class Component:
    """Hook protocol for anything that rides along in the flight loop."""
    name = "component"

    def attach(self, app):
        """Called before Play, once the vehicle exists (define USD prims / publishers here)."""
        self.app = app

    def on_play(self):
        """Called right after Play + the first rendered frames (renderer-dependent init)."""

    def on_substep(self, sim_t, k):
        """Called after every physics substep (250 Hz); k = global substep counter."""

    def on_frame(self, sim_t, n):
        """Called after every rendered frame (60 Hz); n = global frame counter."""

    def telemetry(self):
        """Short status fragment appended to the periodic [tag] log line."""
        return ""

    def close(self):
        """Called once at shutdown."""


def stamp(msg, t):
    """Stamp a ROS message header with the shared sim clock (no /clock topic yet)."""
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = min(int((t - int(t)) * 1e9), 999999999)


class GroundTruthLogger(Component):
    """10 Hz out-of-band ground truth to CSV, never a ROS topic (the pattern of v1, the predecessor project).
    Both clocks are logged (sim_t and wall_t) for the RTF-gated offline join."""
    name = "gt"

    def __init__(self, every_n_substeps=25, path=None):
        self.every = every_n_substeps          # 250 Hz / 25 = 10 Hz
        self.path = path
        self.f = None
        self.rows = 0

    def attach(self, app):
        super().attach(app)
        os.makedirs(RUNS_DIR, exist_ok=True)
        if self.path is None:
            self.path = os.path.join(RUNS_DIR, f"{app.tag}_gt_{app.seed}_{app.run_id}.csv")
        self.f = open(self.path, "w", buffering=1)
        self.f.write(f"# seed={app.seed} wall_start={app.wall_start_str} fmt=gt1 frames=ENU/FLU\n")
        self.f.write("sim_t,wall_t,x,y,z,qx,qy,qz,qw\n")
        carb.log_warn(f"[{app.tag}] ground truth -> {self.path} at {250 // self.every} Hz")

    def on_substep(self, sim_t, k):
        if k % self.every:
            return
        try:
            st = self.app.vehicle._state
            p, q = st.position, st.attitude
            self.f.write(f"{sim_t:.3f},{time.time():.3f},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},"
                         f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}\n")
            self.rows += 1
        except Exception:  # noqa: BLE001
            pass

    def telemetry(self):
        return f"gt_rows={self.rows}"

    def close(self):
        try:
            self.f.close()
        except Exception:  # noqa: BLE001
            pass


class VioSensorKit(Component):
    """A6's VIO sensors: Pegasus IMU on /a6/imu (250 Hz, FRD body frame, sensor-data QoS)
    and a forward pinhole camera on /a6/cam0/image_raw (mono8 640x480, ~20 Hz, RELIABLE:
    ov_msckf subscribes reliable; a best-effort publisher delivers nothing). With
    camera=False (AEB_CAMERA=0, the benchmark configuration since 2026-09-24) only the IMU is
    published: the executive's parked / liftoff / impact metrics read it, the odometry
    comes from the lidar (rf2o), and the RTF is the lidar-only one.
    Camera intrinsics/extrinsics must match policies/openvins/kalibr_imucam_chain.yaml:
    fx=fy=380.06 px, cx=320, cy=240; R_CtoI=[[0,0,1],[1,0,0],[0,1,0]], p_CinI=[0.10,0,0].
    Topic names keep the A6 prefix so the pinned OpenVINS config works unchanged."""
    name = "vio"
    CAM_RES = (640, 480)
    CAM_FOCAL_MM = 12.443          # fx = 640 * 12.443 / 20.955 = 380.06 px
    CAM_H_APERTURE_MM = 20.955
    CAM_V_APERTURE_MM = 15.716     # square pixels: 20.955 * 480 / 640
    CAM_OFFSET_FLU = (0.10, 0.0, 0.0)

    def __init__(self, cam_every_n_frames=3, imu_topic="/a6/imu", cam_topic="/a6/cam0/image_raw",
                 camera=True):
        """camera=False: the IMU only (the benchmark runs since 2026-09-24, lidar odometry, keep
        the IMU for the executive's parked / liftoff / impact metrics and drop the camera,
        the one sensor that costs RTF: ~0.17 with it, ~0.4 without)."""
        self.cam_every = cam_every_n_frames    # 60 fps render / 3 = ~20 Hz images
        self.imu_topic, self.cam_topic = imu_topic, cam_topic
        self.with_camera = camera
        self.camera = None
        self.camera_ready = False
        self.pub_imu = self.pub_img = None
        self.imu_sent = self.img_sent = 0
        self.imu_sensor = None

    def attach(self, app):
        super().attach(app)
        if self.with_camera:
            self._attach_camera(app)
        else:
            carb.log_warn(f"[{app.tag}] vio_cam NOT created (AEB_CAMERA=0): IMU only")
        self.imu_sensor = next(
            (s for s in getattr(app.vehicle, "_sensors", [])
             if getattr(s, "sensor_type", "") == "IMU"), None)
        carb.log_warn(f"[{app.tag}] IMU sensor found: {self.imu_sensor is not None}")

        if app.ros_node is not None:
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Image, Imu
            self._Image, self._Imu = Image, Imu
            self.pub_imu = app.ros_node.create_publisher(Imu, self.imu_topic, qos_profile_sensor_data)
            if self.with_camera:
                self.pub_img = app.ros_node.create_publisher(Image, self.cam_topic, 10)   # RELIABLE
            carb.log_warn(f"[{app.tag}] ROS 2 publishers ready: {self.imu_topic} (250 Hz)"
                          + (f", {self.cam_topic} (~{60 // self.cam_every} Hz)" if self.with_camera else ""))

    def _attach_camera(self, app):
        stage = omni_usd.get_context().get_stage()
        # USD camera view axis is -Z with +Y up. Columns of R = camera axes in the FLU
        # body frame: x_cam=(0,-1,0), y_cam=(0,0,1), z_cam=(-1,0,0)
        # => optical axes in FRD body: x->right, y->down, z->forward.
        self.cam_path = app.vehicle._stage_prefix + "/body/vio_cam"
        cam_usd = UsdGeom.Camera.Define(stage, self.cam_path)
        cam_usd.GetFocalLengthAttr().Set(self.CAM_FOCAL_MM)
        cam_usd.GetHorizontalApertureAttr().Set(self.CAM_H_APERTURE_MM)
        cam_usd.GetVerticalApertureAttr().Set(self.CAM_V_APERTURE_MM)
        cam_usd.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))
        r_cam = np.array([[0.0, 0.0, -1.0],
                          [-1.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0]])
        q_cam = Rotation.from_matrix(r_cam).as_quat()  # x,y,z,w
        xf = UsdGeom.Xformable(cam_usd.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*self.CAM_OFFSET_FLU))
        xf.AddOrientOp().Set(Gf.Quatf(float(q_cam[3]), float(q_cam[0]),
                                      float(q_cam[1]), float(q_cam[2])))
        self.camera = Camera(prim_path=self.cam_path, resolution=self.CAM_RES)
        fx = self.CAM_RES[0] * self.CAM_FOCAL_MM / self.CAM_H_APERTURE_MM
        carb.log_warn(f"[{app.tag}] vio_cam defined at {self.cam_path} "
                      f"({self.CAM_RES[0]}x{self.CAM_RES[1]}, fx=fy={fx:.2f})")

    def on_play(self):
        if not self.with_camera:
            return
        try:
            self.camera.initialize()
            self.camera_ready = True
            carb.log_warn(f"[{self.app.tag}] camera initialized")
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"[{self.app.tag}] camera initialize FAILED: {exc}; no image stream")

    def on_substep(self, sim_t, k):
        if self.pub_imu is None or self.imu_sensor is None:
            return
        try:
            s = self.imu_sensor.state
            msg = self._Imu()
            stamp(msg, sim_t)
            msg.header.frame_id = "imu_frd"
            q = s["orientation"]
            msg.orientation.x, msg.orientation.y = float(q[0]), float(q[1])
            msg.orientation.z, msg.orientation.w = float(q[2]), float(q[3])
            av = s["angular_velocity"]
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = \
                float(av[0]), float(av[1]), float(av[2])
            la = s["linear_acceleration"]
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = \
                float(la[0]), float(la[1]), float(la[2])
            self.pub_imu.publish(msg)
            self.imu_sent += 1
        except Exception as exc:  # noqa: BLE001
            if self.imu_sent == 0:
                carb.log_warn(f"[{self.app.tag}] IMU publish failed: {exc}")

    def on_frame(self, sim_t, n):
        if self.pub_img is None or not self.camera_ready or n % self.cam_every:
            return
        try:
            rgba = self.camera.get_rgba()
            if rgba is None or getattr(rgba, "size", 0) == 0:
                return
            if rgba.dtype != np.uint8:
                rgba = (np.clip(rgba, 0.0, 1.0) * 255.0).astype(np.uint8)
            gray = (rgba[:, :, :3] @ np.array([0.299, 0.587, 0.114])).astype(np.uint8)
            msg = self._Image()
            stamp(msg, sim_t)
            msg.header.frame_id = "vio_cam"
            msg.height, msg.width = gray.shape[0], gray.shape[1]
            msg.encoding = "mono8"
            msg.is_bigendian = 0
            msg.step = gray.shape[1]
            msg.data = gray.tobytes()
            self.pub_img.publish(msg)
            self.img_sent += 1
        except Exception as exc:  # noqa: BLE001
            if self.img_sent == 0:
                carb.log_warn(f"[{self.app.tag}] image publish failed: {exc}")

    def telemetry(self):
        return f"imu={self.imu_sent} img={self.img_sent if self.with_camera else 'off'}"


# -------------------------------------------------------------- the app
class FlightApp:
    """One seeded world + one Iris on external PX4, driven at physics rate."""

    def __init__(self, tag="aeb", world_csv=None, spawn=SPAWN_POS):
        if _app is None:
            raise RuntimeError("call aeb_flight.start_simulation_app() before FlightApp()")
        self.tag = tag
        self.run_id = time.strftime("%Y%m%d_%H%M%S")
        self.wall_start_str = time.strftime("%Y-%m-%d %H:%M:%S")
        self.timeline = omni_timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        carb.log_warn(f"[{tag}] loading environment ...")
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Default Environment"])
        carb.log_warn(f"[{tag}] environment loaded")

        self.world_csv = world_csv or resolve_world_csv()
        if not os.path.isfile(self.world_csv):
            raise FileNotFoundError(f"[{tag}] world manifest not found: {self.world_csv}")
        self.seed = seed_from_path(self.world_csv)
        build_world(self.world_csv, tag)

        config_multirotor = MultirotorConfig()
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": False,          # PX4 SITL is launched by hand in WSL2
        })
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]
        self.vehicle = Multirotor(
            VEHICLE_PRIM,
            ROBOTS["Iris"],
            0,
            list(spawn),
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )
        carb.log_warn(f"[{tag}] vehicle spawned at {tuple(spawn)}")

        # dc shim singleton: the run loop advances its clock (tick())
        self.dc = acquire_dynamic_control_interface()
        self._pre_apply_physx_schemas()

        # one ROS 2 node shared by every component
        self.ros_node = None
        if ROS2_AVAILABLE:
            try:
                rclpy.init()
                self.ros_node = rclpy.create_node(f"aeb_{tag}")
            except Exception as exc:  # noqa: BLE001
                carb.log_warn(f"[{tag}] ROS 2 node setup failed: {exc}; flying without ROS output")
                self.ros_node = None

        self._detach_vehicle_callbacks()
        self.components = []
        self.sim_time = 0.0
        self.stop_sim = False
        self.frame = 0
        self.substep = 0
        self.wall_play = None

    # ----------------------------------------------------------- setup
    def _pre_apply_physx_schemas(self):
        """Force APIs and propeller drives must exist BEFORE Play (an Isaac 6.0.1 standalone rule)."""
        stage = omni_usd.get_context().get_stage()
        for suffix in ("/body", "/rotor0", "/rotor1", "/rotor2", "/rotor3"):
            path = self.vehicle._stage_prefix + suffix
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                api = PhysxSchema.PhysxForceAPI.Apply(prim)
                for call in (lambda: api.CreateWorldFrameEnabledAttr(False),
                             lambda: api.CreateModeAttr("force")):
                    try:
                        call()
                    except Exception:  # noqa: BLE001
                        pass
                for m_name in ("CreateForceEnabledAttr", "CreateEnabledAttr"):
                    m = getattr(api, m_name, None)
                    if m is not None:
                        try:
                            m(True)
                        except Exception:  # noqa: BLE001
                            pass
                        break
                for call in (lambda: api.CreateForceAttr(Gf.Vec3f(0.0, 0.0, 0.0)),
                             lambda: api.CreateTorqueAttr(Gf.Vec3f(0.0, 0.0, 0.0))):
                    try:
                        call()
                    except Exception:  # noqa: BLE001
                        pass
                carb.log_warn(f"[{self.tag}] pre-applied PhysxForceAPI on {path}")
            else:
                carb.log_warn(f"[{self.tag}] prim MISSING for force API: {path}")
        for i in range(4):
            jpath = f"{self.vehicle._stage_prefix}/rotor{i}/joint{i}"
            jprim = stage.GetPrimAtPath(jpath)
            if jprim and jprim.IsValid():
                drive = UsdPhysics.DriveAPI.Apply(jprim, "angular")
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(0.02)
                drive.CreateMaxForceAttr(0.2)
                drive.CreateTargetVelocityAttr(0.0)
                carb.log_warn(f"[{self.tag}] pre-applied propeller velocity drive on {jpath}")
            else:
                carb.log_warn(f"[{self.tag}] joint MISSING for propeller drive: {jpath}")

    def _detach_vehicle_callbacks(self):
        """The core-API physics callbacks never fire in standalone runs; drop them."""
        v = self.vehicle
        for suffix in ("/state", "/update", "/Sensors", "/mav_state"):
            name = v._stage_prefix + suffix
            try:
                if self.world.physics_callback_exists(name):
                    self.world.remove_physics_callback(name)
                    carb.log_warn(f"[{self.tag}] detached dead callback: {name} (manual drive)")
            except Exception as exc:  # noqa: BLE001
                carb.log_warn(f"[{self.tag}] could not detach {name}: {exc}")

    def add(self, component):
        component.attach(self)
        self.components.append(component)
        return component

    # ------------------------------------------------------------- run
    def body_altitude(self):
        try:
            return float(self.dc.get_rigid_body_pose(self.vehicle._stage_prefix + "/body").p[2])
        except Exception:  # noqa: BLE001
            return float("nan")

    def run(self, telemetry_every=250):
        self.timeline.play()
        carb.log_warn(f"[{self.tag}] timeline.play() called; is_playing={self.timeline.is_playing()}")

        phys_dt = 1.0 / 250.0
        try:
            phys_dt = float(self.world.get_physics_dt())
        except Exception:  # noqa: BLE001
            pass
        render_dt = 1.0 / 60.0
        try:
            render_dt = float(self.world.get_rendering_dt())
        except Exception:  # noqa: BLE001
            pass
        substeps = max(1, int(round(render_dt / phys_dt)))
        carb.log_warn(f"[{self.tag}] control at physics rate: dt={phys_dt:.4f}s x {substeps} "
                      f"substeps per rendered frame")

        # renderer-dependent init (camera annotators) needs a few frames after Play
        for _ in range(3):
            try:
                self.world.render()
            except Exception:  # noqa: BLE001
                pass
        for c in self.components:
            c.on_play()

        v = self.vehicle
        update_errors = 0
        self.wall_play = time.time()
        self._write_status("playing")
        if STOP_FILE:
            carb.log_warn(f"[{self.tag}] stop file: {STOP_FILE}  status file: {STATUS_FILE}")
        while _app.is_running() and not self.stop_sim:
            for _ in range(substeps):
                self.world.step(render=False)
                self.dc.tick(phys_dt)
                self.sim_time += phys_dt
                try:
                    v.update_state(phys_dt)
                    v.update(phys_dt)
                    v.update_sensors(phys_dt)
                    v.update_sim_state(phys_dt)
                except Exception as exc:  # noqa: BLE001
                    update_errors += 1
                    if update_errors <= 3 or update_errors % 2000 == 0:
                        import traceback
                        carb.log_warn(f"[{self.tag}] manual update error #{update_errors}: {exc}")
                        carb.log_warn(traceback.format_exc()[-1200:])
                self.substep += 1
                for c in self.components:
                    c.on_substep(self.sim_time, self.substep)
            try:
                self.world.render()
            except Exception:  # noqa: BLE001
                self.world.step(render=True)
            self.frame += 1
            for c in self.components:
                c.on_frame(self.sim_time, self.frame)
            if self.frame % telemetry_every == 0:
                self._telemetry()
            if self.frame % 50 == 0:
                self._write_status("playing")
                if STOP_FILE and os.path.exists(STOP_FILE):
                    carb.log_warn(f"[{self.tag}] stop file found; stopping cleanly")
                    self.stop_sim = True

        carb.log_warn(f"[{self.tag}] closing (frames={self.frame}, sim_t={self.sim_time:.1f}s, "
                      f"update_errors={update_errors}).")
        self._write_status("closing")
        for c in self.components:
            try:
                c.close()
            except Exception:  # noqa: BLE001
                pass
        if self.ros_node is not None:
            try:
                self.ros_node.destroy_node()
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass
        self.timeline.stop()
        self._write_status("closed")
        _app.close()

    def _heartbeat(self):
        b = self.vehicle._backends[0] if getattr(self.vehicle, "_backends", None) else None
        return getattr(b, "_received_first_hearbeat", "n/a")

    def _write_status(self, state):
        """One line, overwritten (write + rename): the batch runner's readiness and liveness
        probe. A failed write (the reader holding the file on the Windows side) is skipped;
        the next one comes 50 frames later."""
        if not STATUS_FILE:
            return
        try:
            wall = time.time() - self.wall_play if self.wall_play else 0.0
            last = getattr(self, "_status_last", None)
            rtf_now = ((self.sim_time - last[0]) / (wall - last[1])) if last and wall > last[1] else float("nan")
            self._status_last = (self.sim_time, wall)
            line = (f"state={state} t={time.time():.1f} pid={os.getpid()} frame={self.frame} "
                    f"sim_t={self.sim_time:.2f} wall={wall:.1f} rtf_now={rtf_now:.3f} "
                    f"alt={self.body_altitude():.2f} heartbeat={self._heartbeat()}\n")
            tmp = STATUS_FILE + ".tmp"
            with open(tmp, "w") as f:
                f.write(line)
            os.replace(tmp, STATUS_FILE)
        except Exception:  # noqa: BLE001
            pass

    def _telemetry(self):
        try:
            b = self.vehicle._backends[0] if getattr(self.vehicle, "_backends", None) else None
            wall = time.time() - self.wall_play
            rtf = self.sim_time / wall if wall > 0 else float("nan")          # cumulative since Play
            last = getattr(self, "_tele_last", None)                         # instantaneous since the last line
            rtf_now = ((self.sim_time - last[0]) / (wall - last[1])) if last and wall > last[1] else float("nan")
            self._tele_last = (self.sim_time, wall)
            extra = " | ".join(t for t in (c.telemetry() for c in self.components) if t)
            carb.log_warn(
                f"[{self.tag}] frame={self.frame} sim_t={self.sim_time:.2f} wall={wall:.1f} "
                f"rtf={rtf:.3f} rtf_now={rtf_now:.3f} alt={self.body_altitude():.2f} "
                f"heartbeat={getattr(b, '_received_first_hearbeat', 'n/a')} | {extra}")
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"[{self.tag}] telemetry error: {exc}")
