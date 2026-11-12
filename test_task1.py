"""test_task1.py - checks the six Task 1 behaviours. Run: python test_task1.py"""

from rov_sim import ROVSimulator, DT


def run(sim, seconds, **cmd):
    for _ in range(int(seconds / DT)):
        sim.step(**cmd)


def check(name, ok):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    assert ok, name


# 1. Zero thrust -> stationary
sim = ROVSimulator()
run(sim, 5)
s = sim.state
check("zero thrust: no movement", s.x == 0 and s.y == 0 and s.depth == 0)

# 2. Full surge -> ramps up gradually, then levels off
sim = ROVSimulator()
run(sim, 0.5, surge=1.0)
v_early = sim.state.surge_vel
run(sim, 9.5, surge=1.0)
v_9_5 = sim.state.surge_vel
run(sim, 0.5, surge=1.0)
v_10 = sim.state.surge_vel
print(f"    surge speed: 0.5s={v_early:.3f}  10s={v_10:.3f} m/s")
check("surge: gradual ramp", 0 < v_early < 0.3)
check("surge: levels off near terminal speed (~1 m/s)", v_10 > 0.95 and abs(v_10 - v_9_5) < 0.01)

# 3. Release thrust -> coasts to a stop (not instant)
run(sim, 0.2, surge=0.0)
check("release: still moving just after release", sim.state.surge_vel > 0.1)
run(sim, 6, surge=0.0)
check("release: coasts to stop", abs(sim.state.surge_vel) < 0.01)

# 4. Heave changes depth, never below 0
sim = ROVSimulator()
run(sim, 5, heave=1.0)
print(f"    depth after 5s descent: {sim.state.depth:.2f} m")
check("heave: descends", sim.state.depth > 0.5)
run(sim, 30, heave=-1.0)
check("heave: depth never below 0", sim.state.depth >= 0.0)

# 5. Yaw turns heading, surge follows heading
sim = ROVSimulator()
run(sim, 2, yaw=1.0)
check("yaw: heading changes", sim.state.yaw > 10)
sim = ROVSimulator()
sim.state.yaw = 90.0  # facing East
run(sim, 3, surge=1.0)
print(f"    facing East, surge 3s: north={sim.state.x:.2f} east={sim.state.y:.2f}")
check("surge at heading 90 moves East", sim.state.y > 1.0 and abs(sim.state.x) < 0.1)

# 6. Battery drains
sim = ROVSimulator()
v0 = sim.state.battery_v
run(sim, 10, surge=1.0)
print(f"    battery: {v0:.3f} -> {sim.state.battery_v:.3f} V")
check("battery: decreases", sim.state.battery_v < v0)

print("\nAll Task 1 checks passed.")
