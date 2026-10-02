"""Localizer failure injection for the mission mocks (the --loc-guard tests); off unless AMCL_FAIL is set.

  AMCL_FAIL=x0,x1,y0,y1   a box of the map where the mock AMCL loses track the way AMCL did in the
                          first tour batches: the covariance rises first (0.8 m^2) while the pose
                          creeps off (0.05 m per update); after 6 sim-s in the box the pose jumps
                          4 m (toward +y) and the filter is confident again (0.05 m^2) in that wrong
                          place, and stays there after the box until a reseed. A vehicle that leaves
                          the box before the jump finds the mock AMCL recovering by itself
  AMCL_FAIL_MODE=jump     the pose jumps 3 m at once in the box, with a low covariance
  AMCL_FAIL_NOSEED=1      the mock AMCL ignores /initialpose (it never recovers from the jump)

A reseed (/initialpose) outside the box restarts the filter at the seed: its offset from the truth
halves with every update and the covariance falls from the seed's 0.16 m^2 to 0.02 m^2."""
import os


class AmclFault:
    def __init__(self):
        z = os.environ.get("AMCL_FAIL")
        self.zone = tuple(float(v) for v in z.split(",")) if z else None
        self.mode = os.environ.get("AMCL_FAIL_MODE", "drift")
        self.noseed = bool(os.environ.get("AMCL_FAIL_NOSEED"))
        self.off = [0.0, 0.0]
        self.cov = 0.01
        self.t_in = None
        self.kidnapped = False
        self.seeded = False
        self.k = 0
        self.reseeds = 0

    def inside(self, x, y):
        z = self.zone
        return z is not None and z[0] <= x <= z[1] and z[2] <= y <= z[3]

    def step(self, x, y, t):
        """One AMCL update with the truth at (x, y), sim time t: (dx, dy, cov) to apply."""
        if self.zone is None:
            return 0.0, 0.0, 0.01
        if self.inside(x, y):
            self.seeded = False
            if self.t_in is None:
                self.t_in = t
            if self.mode == "jump":
                self.kidnapped, self.off, self.cov = True, [0.0, 3.0], 0.05
            elif t - self.t_in < 6.0 and not self.kidnapped:
                self.off[1] += 0.05
                self.cov = 0.8
            else:
                self.kidnapped, self.off, self.cov = True, [0.0, 4.0], 0.05
        else:
            self.t_in = None
            if self.seeded:
                self.k += 1
                self.off = [self.off[0] * 0.5, self.off[1] * 0.5]
                self.cov = max(0.02, 0.16 * 0.5 ** self.k)
            elif self.kidnapped:
                self.cov = 0.05                    # confident in the wrong place
            elif abs(self.off[0]) + abs(self.off[1]) > 0.01:
                self.off = [self.off[0] * 0.5, self.off[1] * 0.5]
                self.cov = 0.02                    # left the box before the jump: recovers by itself
            else:
                self.off, self.cov = [0.0, 0.0], 0.01
        return self.off[0], self.off[1], self.cov

    def reseed(self, sx, sy, x, y):
        """/initialpose at (sx, sy) with the truth at (x, y)."""
        self.reseeds += 1
        if self.noseed:
            return
        self.off, self.cov, self.seeded, self.kidnapped, self.k = [sx - x, sy - y], 0.16, True, False, 0
