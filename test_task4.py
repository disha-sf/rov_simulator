"""test_task4.py - checks the x/y -> lat/lon conversion. Run: python test_task4.py"""

import math
from geo import local_to_latlon, LAT0, LON0, METRES_PER_DEG_LAT


def check(name, ok):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    assert ok, name


lat, lon = local_to_latlon(0, 0)
check("origin maps to start coordinate", lat == LAT0 and lon == LON0)

lat, lon = local_to_latlon(111.32, 0)
check("111.32 m north = +0.001 deg latitude", abs((lat - LAT0) - 0.001) < 1e-9 and lon == LON0)

lat, lon = local_to_latlon(0, 100.0)
check("east only changes longitude", lat == LAT0 and lon > LON0)

# distance round trip: 100 m east should be ~100 m on the ground
d_east = (lon - LON0) * METRES_PER_DEG_LAT * math.cos(math.radians(LAT0))
check("100 m east converts back to ~100 m", abs(d_east - 100.0) < 1e-6)

lat, lon = local_to_latlon(-50, -50)
check("south/west gives smaller lat/lon", lat < LAT0 and lon < LON0)

print("\nAll Task 4 checks passed.")
