"""
rov_sim.py - Task 1: ROV physics and state simulation (no UI).

Conventions
-----------
- Units: metres, seconds, degrees (for yaw / roll / pitch), volts.
- Depth is positive downward.
- Heading (yaw): 0 = North, 90 = East, clockwise positive.
- World frame: x = North, y = East.
- Commands are normalised to -1..+1:
    surge +1 = forward, sway +1 = right, heave +1 = DOWN, yaw +1 = turn right.
"""

import math
import random
from dataclasses import dataclass

DT = 0.02  # fixed time step: 20 ms (50 Hz)

# ---- Tunable parameters ----------------------------------------------------
THRUSTER_TAU = 0.3  # thruster response time constant (s)

# Acceleration at full thrust (m/s^2, or deg/s^2 for yaw)
ACCEL = {"surge": 1.0, "sway": 0.8, "heave": 0.6, "yaw": 90.0}

# Linear drag coefficient (1/s). Terminal speed = ACCEL / DRAG
#   surge 1.0 m/s, sway 0.8 m/s, heave 0.5 m/s, yaw 60 deg/s
DRAG = {"surge": 1.0, "sway": 1.0, "heave": 1.2, "yaw": 1.5}

BATTERY_FULL = 16.8      # 4S pack
BATTERY_EMPTY = 12.0     # floor
BATTERY_BASE_DRAIN = 0.001   # V/s always
BATTERY_THRUST_DRAIN = 0.002  # V/s per unit of |thrust|, summed over axes

MAX_TILT = 5.0  # deg of roll/pitch at full thrust (simplified)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


@dataclass
class ROVState:
    # Position
    x: float = 0.0      # North (m)
    y: float = 0.0      # East (m)
    depth: float = 0.0  # m, positive down
    # Orientation (deg)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    # Velocities (body frame)
    surge_vel: float = 0.0
    sway_vel: float = 0.0
    heave_vel: float = 0.0
    yaw_rate: float = 0.0  # deg/s
    # Actual thrust after thruster delay
    surge_thrust: float = 0.0
    sway_thrust: float = 0.0
    heave_thrust: float = 0.0
    yaw_thrust: float = 0.0
    # Battery
    battery_v: float = BATTERY_FULL


class ROVSimulator:
    def __init__(self):
        self.state = ROVState()
        self.time = 0.0

    def step(self, surge=0.0, sway=0.0, heave=0.0, yaw=0.0, armed=True, dt=DT):
        """Advance the simulation by dt seconds using commanded thrust.

        Safety guard (second layer): when armed is False, the commands are
        forced to zero AND the actual thrust is zeroed immediately, so the
        thruster delay cannot leave any thrust behind after a disarm.
        """
        s = self.state
        if not armed:
            surge = sway = heave = yaw = 0.0
            s.surge_thrust = s.sway_thrust = s.heave_thrust = s.yaw_thrust = 0.0
        cmd = {
            "surge": _clamp(surge, -1.0, 1.0),
            "sway": _clamp(sway, -1.0, 1.0),
            "heave": _clamp(heave, -1.0, 1.0),
            "yaw": _clamp(yaw, -1.0, 1.0),
        }

        # 1. Thruster delay: first-order low-pass filter
        alpha = dt / (THRUSTER_TAU + dt)
        s.surge_thrust += alpha * (cmd["surge"] - s.surge_thrust)
        s.sway_thrust += alpha * (cmd["sway"] - s.sway_thrust)
        s.heave_thrust += alpha * (cmd["heave"] - s.heave_thrust)
        s.yaw_thrust += alpha * (cmd["yaw"] - s.yaw_thrust)

        # 2. Velocities: acceleration = thrust*gain - drag*velocity
        s.surge_vel += (ACCEL["surge"] * s.surge_thrust - DRAG["surge"] * s.surge_vel) * dt
        s.sway_vel += (ACCEL["sway"] * s.sway_thrust - DRAG["sway"] * s.sway_vel) * dt
        s.heave_vel += (ACCEL["heave"] * s.heave_thrust - DRAG["heave"] * s.heave_vel) * dt
        s.yaw_rate += (ACCEL["yaw"] * s.yaw_thrust - DRAG["yaw"] * s.yaw_rate) * dt

        # 3. Integrate position / heading
        s.yaw = (s.yaw + s.yaw_rate * dt) % 360.0
        s.depth += s.heave_vel * dt
        if s.depth < 0.0:  # can't go above the surface
            s.depth = 0.0
            s.heave_vel = max(0.0, s.heave_vel)

        # Rotate body-frame velocity into world frame (North/East)
        yaw_rad = math.radians(s.yaw)
        north = s.surge_vel * math.cos(yaw_rad) - s.sway_vel * math.sin(yaw_rad)
        east = s.surge_vel * math.sin(yaw_rad) + s.sway_vel * math.cos(yaw_rad)
        s.x += north * dt
        s.y += east * dt

        # 4. Roll / pitch: simplified approximation from thrust + small noise
        target_pitch = -MAX_TILT * s.surge_thrust + random.uniform(-0.2, 0.2)
        target_roll = (MAX_TILT * s.sway_thrust
                       + 0.02 * s.yaw_rate
                       + random.uniform(-0.2, 0.2))
        s.pitch += 0.1 * (target_pitch - s.pitch)
        s.roll += 0.1 * (target_roll - s.roll)

        # 5. Battery drain
        total_thrust = (abs(s.surge_thrust) + abs(s.sway_thrust)
                        + abs(s.heave_thrust) + abs(s.yaw_thrust))
        drain = BATTERY_BASE_DRAIN + BATTERY_THRUST_DRAIN * total_thrust
        s.battery_v = max(BATTERY_EMPTY, s.battery_v - drain * dt)

        self.time += dt
        return s
