"""test_task2.py - checks Task 2 safety + modes. Run: python test_task2.py"""

from controller import (ROVController, MANUAL, DEPTH_HOLD, HEADING_HOLD,
                        wrap_180)
from rov_sim import DT


def check(name, ok):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    assert ok, name


def run(c, seconds, refresh=None):
    """Run the controller; refresh = manual input tuple re-sent every tick."""
    for _ in range(int(round(seconds / DT))):
        if refresh:
            c.set_manual_input(*refresh)
        c.update()


def thrusts(s):
    return (s.surge_thrust, s.sway_thrust, s.heave_thrust, s.yaw_thrust)


# 1. Disarmed: full input does nothing
c = ROVController()
run(c, 3, refresh=(1, 1, 1, 1))
s = c.sim.state
check("disarmed: thrust is zero", all(t == 0 for t in thrusts(s)))
check("disarmed: no movement", s.x == 0 and s.y == 0 and s.depth == 0)

# 2. Disarm mid-motion: thrust zero immediately, ROV coasts
c = ROVController()
c.arm()
run(c, 3, refresh=(1, 0, 0, 0))
check("armed: thrust is non-zero while driving", c.sim.state.surge_thrust > 0.5)
v_before = c.sim.state.surge_vel
c.disarm()
c.update()
check("disarm: actual thrust zero on the very next tick",
      all(t == 0 for t in thrusts(c.sim.state)))
check("disarm: ROV still coasting (velocity not instantly zero)",
      c.sim.state.surge_vel > 0.5 * v_before)
run(c, 6)
check("disarm: coasts to a stop", abs(c.sim.state.surge_vel) < 0.01)

# 3. Manual watchdog (1 s)
c = ROVController()
c.arm()
c.set_manual_input(1, 0, 0, 0)  # single input, then nothing
run(c, 0.5)
check("watchdog: command still active at 0.5 s", c.last_command[0] == 1.0)
run(c, 0.6)  # t = 1.1 s
check("watchdog: command zero after 1 s without input", c.last_command == (0, 0, 0, 0))
run(c, 4)
check("watchdog: thrust fades to zero", abs(c.sim.state.surge_thrust) < 0.01)

# 4. Depth hold
c = ROVController()
c.arm()
c.set_mode(DEPTH_HOLD)
c.set_target_depth(3.0)
max_depth = 0.0
for _ in range(int(25 / DT)):
    c.update()
    max_depth = max(max_depth, c.sim.state.depth)
d = c.sim.state.depth
print(f"    depth hold: final={d:.3f} m, max={max_depth:.3f} m")
check("depth hold: settles at target", abs(d - 3.0) < 0.05)
check("depth hold: small overshoot", max_depth < 3.3)
c.set_target_depth(-5)
check("depth hold: target never below 0", c.target_depth == 0.0)

# 4b. Depth hold + manual surge still works
run(c, 1, refresh=(1, 0, 0, 0))
check("depth hold: surge still works from keyboard", c.sim.state.x != 0.0)

# 5. Heading hold
check("wrap_180: 350 vs 10 -> -20", abs(wrap_180(350 - 10) - (-20)) < 1e-9)
c = ROVController()
c.arm()
c.sim.state.yaw = 90.0
c.set_mode(HEADING_HOLD)
c.set_target_heading(270.0)
run(c, 20)
print(f"    heading hold 90->270: final={c.sim.state.yaw:.2f}")
check("heading hold: reaches 270", abs(wrap_180(c.sim.state.yaw - 270)) < 1.0)

c = ROVController()
c.arm()
c.sim.state.yaw = 10.0
c.set_mode(HEADING_HOLD)
c.set_target_heading(350.0)
yaws = []
for _ in range(int(15 / DT)):
    c.update()
    yaws.append(c.sim.state.yaw)
print(f"    heading hold 10->350: final={yaws[-1]:.2f}")
check("heading hold: settles at 350", abs(wrap_180(yaws[-1] - 350)) < 1.0)
check("heading hold: took the short way (through 0)",
      all(y >= 330 or y <= 30 for y in yaws))

# 6. Mode switch: no lurch
c = ROVController()
c.arm()
c.sim.state.depth = 5.0
c.sim.state.yaw = 123.0
c.set_mode(DEPTH_HOLD)
check("switch: target depth = current depth", abs(c.target_depth - 5.0) < 1e-9)
c.update()
check("switch: no lurch (heave command ~0)", abs(c.last_command[2]) < 0.05)
c.set_mode(HEADING_HOLD)
check("switch: target heading = current heading", abs(c.target_heading - 123.0) < 1.0)

print("\nAll Task 2 checks passed.")
