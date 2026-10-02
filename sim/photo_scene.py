# === Photo scene: both robots in one seeded world, for screenshots (docs/images) ===
# Executed at Isaac launch, the full GUI app, the way the wheeled arm starts:
#     isaac-sim.bat --exec <repo>\sim\photo_scene.py --/app/file/ignoreUnsavedOnExit=true
# Opens the wheeled robot's stage (sim/wheeled/robot.usda), rebuilds the world of
# AEB_WORLD_SEED from its manifest with the run-time rules (sim/wheeled_bootstrap.py
# _build_world), places the wheeled robot and the quadrotor (Pegasus's Iris, referenced as a
# USD, no PX4) at the poses below, points the viewport at them and leaves the app open. No
# ROS, no Play: a still scene for the camera. Take screenshots with F10 (Edit > Preferences >
# Capture Screenshot sets the folder). The stage is never saved.
#
# Environment (a fresh PowerShell, without sim\env.ps1: the full app must not see a ROS
# library folder on PATH, see sim\launch_wheeled.ps1):
#   AEB_REPO            the repo (default: this file's parent's parent)
#   AEB_WORLD_SEED      the world (default 20260723008)
#   AEB_PHOTO_WHEELED   "x,y,yaw_deg" of the wheeled robot (default "1.2,-1.2,200")
#   AEB_PHOTO_QUAD      "x,y,z,yaw_deg" of the quadrotor (default "-1.2,1.2,1.3,30")
#   AEB_IRIS_USD        the Iris USD, when Pegasus is not importable in the app (default: the
#                       Pegasus install's ROBOTS["Iris"], else a search under C:\PegasusSimulator)
import asyncio
import glob
import os
import traceback


def _repo():
    r = os.environ.get("AEB_REPO", "")
    if not r:
        try:
            r = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        except NameError:
            r = ""
    return r.replace("\\", "/")


REPO = _repo()
SEED = os.environ.get("AEB_WORLD_SEED", "20260723008")
os.environ.setdefault("AEB_REPO", REPO)
os.environ.setdefault("AEB_WORLD_SEED", SEED)


class _WB:
    """sim/wheeled_bootstrap.py's world builder, loaded without its boot: the module schedules a
    wheeled run at import, so its source is executed here with that last line left out."""

    def __init__(self):
        path = REPO + "/sim/wheeled_bootstrap.py"
        src = open(path, encoding="utf-8").read().replace("asyncio.ensure_future(_main())", "")
        ns = {"__file__": path}
        exec(compile(src, path, "exec"), ns)
        self.ROBOT_USD, self.MANIFEST, self._build_world = ns["ROBOT_USD"], ns["MANIFEST"], ns["_build_world"]


wb = _WB()

WHEELED_POSE = [float(v) for v in os.environ.get("AEB_PHOTO_WHEELED", "1.2,-1.2,200").split(",")]
QUAD_POSE = [float(v) for v in os.environ.get("AEB_PHOTO_QUAD", "-1.2,1.2,1.3,30").split(",")]
QUAD_PRIM = "/Quadrotor"


def _log(msg):
    print(f"[photo] {msg}", flush=True)


def _iris_usd():
    p = os.environ.get("AEB_IRIS_USD", "")
    if p and os.path.isfile(p):
        return p
    try:
        from pegasus.simulator.params import ROBOTS
        p = ROBOTS["Iris"]
        if os.path.isfile(p):
            return p
        _log(f"Pegasus names {p}, which does not exist")
    except Exception as exc:  # noqa: BLE001
        _log(f"pegasus.simulator not importable here ({exc}); searching for iris.usd")
    for root in (os.environ.get("PEGASUS_PATH", ""), "C:/PegasusSimulator"):
        if root:
            hits = glob.glob(os.path.join(root, "**", "Iris", "iris.usd"), recursive=True)
            if hits:
                return hits[0]
    raise RuntimeError("no Iris USD found: set AEB_IRIS_USD to Pegasus's Robots/Iris/iris.usd")


def _place_wheeled(stage):
    from pxr import Gf, UsdGeom
    prim = stage.GetPrimAtPath("/my_custom_robot")
    if not prim.IsValid():
        raise RuntimeError("the robot stage has no /my_custom_robot")
    x, y, yaw = WHEELED_POSE
    xf = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
    t = ops.get("xformOp:translate")
    z = t.Get()[2] if t is not None and t.Get() is not None else 0.1
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddRotateZOp().Set(yaw)
    _log(f"wheeled robot at ({x}, {y}, {z}) yaw {yaw} deg")


def _place_quadrotor(stage):
    from pxr import Gf, UsdGeom
    usd = _iris_usd()
    x, y, z, yaw = QUAD_POSE
    xform = UsdGeom.Xform.Define(stage, QUAD_PRIM)
    xform.GetPrim().GetReferences().AddReference(usd.replace("\\", "/"))
    xf = UsdGeom.Xformable(xform.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddRotateZOp().Set(yaw)
    _log(f"quadrotor {os.path.basename(usd)} at ({x}, {y}, {z}) yaw {yaw} deg")


def _aim_camera():
    """The viewport's camera: an oblique view of both robots, from above and to the south."""
    try:
        from isaacsim.core.utils.viewports import set_camera_view
        cx = (WHEELED_POSE[0] + QUAD_POSE[0]) / 2
        cy = (WHEELED_POSE[1] + QUAD_POSE[1]) / 2
        set_camera_view(eye=[cx + 4.0, cy - 6.0, 4.5], target=[cx, cy, 0.5])
        _log("camera aimed at the two robots (right mouse + WASD to fly, F10 to capture)")
    except Exception as exc:  # noqa: BLE001
        _log(f"camera not aimed ({exc}); frame the shot by hand")


async def _boot():
    import omni.kit.app
    import omni.usd
    app = omni.kit.app.get_app()
    for _ in range(30):
        await app.next_update_async()
    for path in (wb.ROBOT_USD, wb.MANIFEST):
        if not os.path.isfile(path):
            raise RuntimeError(f"missing {path} (AEB_REPO={REPO!r})")
    ctx = omni.usd.get_context()
    ctx.open_stage(wb.ROBOT_USD)
    for _ in range(60):
        await app.next_update_async()
    stage = ctx.get_stage()
    for graph in ("/Graph/ROS_Camera", "/Graph/ROS_DiffDrive"):   # no ROS in this scene
        if stage.GetPrimAtPath(graph):
            stage.RemovePrim(graph)
    wb._build_world(stage)
    _place_wheeled(stage)
    _place_quadrotor(stage)
    for _ in range(30):
        await app.next_update_async()
    _aim_camera()
    _log(f"scene ready: world {SEED}; the stage is never saved, close the app when done")


async def _main():
    try:
        await _boot()
    except Exception:  # noqa: BLE001
        _log("ERROR\n" + traceback.format_exc())


asyncio.ensure_future(_main())
