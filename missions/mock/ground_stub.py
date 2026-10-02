"""The ground vehicle stand-in of the mock harnesses: the real GroundLink controller on a
unicycle that executes /cmd_vel the way the simulated robot does (a turn in place below 0.2 rad/s
moves nothing, measured 2026-10-01); the odometry fed from the truth scaled 0.8 and rotated 2 deg
(the drone mock's dead-reckoning error) at 20 Hz of sim time. Used by ground_mock_test.py and
mock_explore_test.py (GROUND=1)."""
import math
import threading
import time

import aerial_mission_runner as amr

SPEEDUP = 20.0
cmd_log = []     # (t, v, w) of every command the controller issued


class MockGround(amr.GroundLink):
    ground = True

    def __init__(self, a, log, ros=None, **k):
        super().__init__(a, log, None)
        self.x = self.y = self.yaw = 0.0
        self.t_boot = 0.0
        self.v_true = 0.0
        self._next_odom = 0.0
        self.thread = threading.Thread(target=self._sim, daemon=True)
        self.thread.start()

    def _sim(self):
        dt = 0.01
        while self.alive:
            time.sleep(dt / SPEEDUP)
            self.t_boot += dt
            v, w = self.last_cmd
            if abs(v) < 0.01 and abs(w) < 0.2:                 # the simulated drive ignores slow turns in place
                w = 0.0
            self.yaw = amr.wrap(self.yaw + w * dt)
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            self.v_true = abs(v)
            if self.t_boot >= self._next_odom:                  # odometry at 20 Hz of sim time
                self._next_odom += 0.05
                th = math.radians(2.0); c, s = math.cos(th), math.sin(th)
                ox, oy = 0.8 * (c * self.x - s * self.y), 0.8 * (s * self.x + c * self.y)
                self.pose_update(ox, oy, amr.wrap(self.yaw + th), self.t_boot, t_sim=self.t_boot)

    def _publish(self, v, w):
        super()._publish(v, w)
        cmd_log.append((self.t_boot, v, w))


def command_rule(v_min=0.10, w_min=0.5):
    """(commands, slow turns in place, slow forward commands): the controller must never command
    what the simulated drive does not execute."""
    slow_turn = [c for c in cmd_log if abs(c[1]) < 0.01 and 0.0 < abs(c[2]) < w_min - 1e-6]
    slow_fwd = [c for c in cmd_log if 0.0 < abs(c[1]) < v_min - 1e-6]
    return len(cmd_log), len(slow_turn), len(slow_fwd)
