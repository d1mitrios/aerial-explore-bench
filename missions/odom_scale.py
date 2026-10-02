"""Online scale of a monocular-VIO odometry against the localizer's poses (for the VIO
odometry source used until 2026-09-24).

OpenVINS's metric scale comes from the IMU and settles differently from flight to flight
(measured 2026-09-23: 0.95 on one test flight, 0.76 on the next; the same configuration, the
same world). AMCL's likelihood field cannot correct an along-track shortfall where the
beams hit walls obliquely (a shift along a wall leaves every endpoint on the wall), so a
20 % scale error accumulated into a 0.5-0.8 m lag over one 11 m leg and a doorway miss.
The wheel odometry of the baseline has no such error, so its AMCL never faces it; the
practitioner's fix for a wheel odometry with a scale error is to calibrate it, and this
is that calibration done online: the ratio of the localizer's displacement to the raw
VIO displacement over a baseline of a few metres, smoothed, applied to the VIO
INCREMENTS that the bridge integrates into the odom frame (never to the absolute pose,
so a change of scale never jumps the odometry).

Limits learned in a test flight (2026-09-23): the ratio is only meaningful where the scan
constrains the localizer along the direction of travel. In a straight corridor it does
not (no along-wall contrast), AMCL follows the odometry it is given, the measured ratio
then reports the scale it was given, and a scale pulled down by a transient VIO burst
locked itself in (0.6 for the rest of the return, belief 3 m off). Hence: learn only from
poses whose covariance is small AND isotropic (an elongated cloud is the corridor
signature), an asymmetric clamp (0.85-1.6: the trap pulls the scale DOWN, a slow VIO needs it UP), a longer baseline and slow adaptation; the calibration
corrects a persistent scale error over tens of metres, it does not chase bursts.

Pure Python, no ROS: imported by missions/vio_odom_bridge.py.
"""
import math
from collections import deque


class OdomScale:
    def __init__(self, baseline_m=4.0, max_baseline_m=10.0, cov_max=0.10, aniso_max=2.0,
                 smin=0.85, smax=1.6, alpha=0.15, time_tol=0.3):
        self.baseline_m = baseline_m
        self.max_baseline_m = max_baseline_m
        self.cov_max = cov_max
        self.aniso_max = aniso_max
        self.smin, self.smax = smin, smax
        self.alpha = alpha
        self.time_tol = time_tol
        self.vio = deque(maxlen=6000)      # raw VIO (t, x, y), ~5 min at 20 Hz
        self.pairs = deque(maxlen=600)     # (t, loc_x, loc_y, vio_x, vio_y)
        self.scale = 1.0
        self.n_est = 0
        self.last_inst = None

    def add_vio(self, t, x, y):
        self.vio.append((t, x, y))

    def _vio_at(self, t):
        best, bestd = None, self.time_tol
        for v in reversed(self.vio):
            d = abs(v[0] - t)
            if d < bestd:
                best, bestd = v, d
            elif v[0] < t - self.time_tol:
                break
        return best

    def add_localizer(self, t, x, y, cov_xx=0.0, cov_yy=0.0):
        """A localizer pose (map frame) with its covariance. Returns the instantaneous
        scale when a baseline was available, else None. self.scale holds the smoothed value."""
        if max(cov_xx, cov_yy) > self.cov_max:
            return None
        if max(cov_xx, cov_yy) > self.aniso_max * max(min(cov_xx, cov_yy), 1e-4):
            return None                                        # elongated cloud: a corridor, no along-track contrast
        v = self._vio_at(t)
        if v is None:
            return None
        self.pairs.append((t, x, y, v[1], v[2]))
        base = None
        for p in self.pairs:                       # oldest first
            dv = math.hypot(v[1] - p[3], v[2] - p[4])
            if dv > self.max_baseline_m:
                continue
            if dv >= self.baseline_m:
                base = (p, dv)
                break
        if base is None:
            return None
        p, dv = base
        da = math.hypot(x - p[1], y - p[2])
        inst = min(self.smax, max(self.smin, da / dv))
        self.scale = inst if self.n_est == 0 else self.scale + self.alpha * (inst - self.scale)
        self.n_est += 1
        self.last_inst = inst
        return inst


class ScaledIntegrator:
    """Integrates raw VIO increments times the current scale into a continuous odometry."""

    def __init__(self):
        self.prev = None
        self.x = self.y = 0.0

    def step(self, raw_x, raw_y, scale):
        if self.prev is None:
            self.x, self.y = raw_x, raw_y
        else:
            self.x += scale * (raw_x - self.prev[0])
            self.y += scale * (raw_y - self.prev[1])
        self.prev = (raw_x, raw_y)
        return self.x, self.y
