"""
controller.py - Task 2: safety + mode controller (no UI).

Pipeline each tick:
    operator input / hold loops -> mode logic -> watchdog -> armed guard -> sim.step()

Sign conventions (same as rov_sim.py):
    heave +1 = down, yaw +1 = turn right, heading 0 = North, 90 = East.
"""

from rov_sim import ROVSimulator, DT

MANUAL = "Manual"
DEPTH_HOLD = "Depth Hold"
HEADING_HOLD = "Heading Hold"
MODES = (MANUAL, DEPTH_HOLD, HEADING_HOLD)

INPUT_TIMEOUT = 1.0  # s without operator input -> manual thrust command = 0

# PD gains (tuned in test_task2.py)
DEPTH_KP = 0.8    # heave command per metre of error
DEPTH_KD = 1.2    # brake per m/s of heave velocity
HEADING_KP = 0.02  # yaw command per degree of error
HEADING_KD = 0.015  # brake per deg/s of yaw rate


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_180(angle):
    """Wrap an angle difference into -180..+180 (shortest way round)."""
    return (angle + 540.0) % 360.0 - 180.0


class ROVController:
    def __init__(self, sim=None):
        self.sim = sim if sim is not None else ROVSimulator()
        self.armed = False
        self.mode = MANUAL
        self.target_depth = 0.0
        self.target_heading = 0.0
        self._manual = (0.0, 0.0, 0.0, 0.0)
        self._last_input_time = float("-inf")
        self.last_command = (0.0, 0.0, 0.0, 0.0)  # what was sent to the sim

    # ---- operator actions -------------------------------------------------
    def _clear_input(self):
        self._manual = (0.0, 0.0, 0.0, 0.0)
        self._last_input_time = float("-inf")

    def arm(self):
        self._clear_input()  # start from zero commands
        self.armed = True

    def disarm(self):
        self.armed = False
        self._clear_input()

    def set_mode(self, mode):
        if mode not in MODES:
            raise ValueError(f"unknown mode: {mode}")
        if mode == self.mode:
            return
        # Start holding where we are now -> no sudden lurch on switching
        self.target_depth = self.sim.state.depth
        self.target_heading = self.sim.state.yaw
        self._clear_input()  # no stale key input carried over
        self.mode = mode

    def set_target_depth(self, metres):
        self.target_depth = max(0.0, float(metres))

    def set_target_heading(self, degrees):
        self.target_heading = float(degrees) % 360.0

    def set_manual_input(self, surge=0.0, sway=0.0, heave=0.0, yaw=0.0):
        """Call on every key/joystick event. Also refreshes the watchdog."""
        self._manual = (_clamp(surge, -1, 1), _clamp(sway, -1, 1),
                        _clamp(heave, -1, 1), _clamp(yaw, -1, 1))
        self._last_input_time = self.sim.time

    # ---- one control tick -------------------------------------------------
    def update(self):
        s = self.sim.state

        # 1. Manual input + watchdog
        surge, sway, heave, yaw = self._manual
        if self.sim.time - self._last_input_time > INPUT_TIMEOUT:
            surge = sway = heave = yaw = 0.0

        # 2. Hold modes replace one axis with a PD controller
        if self.mode == DEPTH_HOLD:
            error = self.target_depth - s.depth
            heave = _clamp(DEPTH_KP * error - DEPTH_KD * s.heave_vel, -1, 1)
        elif self.mode == HEADING_HOLD:
            error = wrap_180(self.target_heading - s.yaw)
            yaw = _clamp(HEADING_KP * error - HEADING_KD * s.yaw_rate, -1, 1)

        # 3. Armed guard - always the LAST step so nothing can bypass it
        if not self.armed:
            surge = sway = heave = yaw = 0.0

        self.last_command = (surge, sway, heave, yaw)
        return self.sim.step(surge, sway, heave, yaw, armed=self.armed)
