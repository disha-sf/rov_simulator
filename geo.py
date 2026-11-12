"""
geo.py - convert simulator x/y (metres North/East of start) to lat/lon.

Flat-earth approximation: accurate to well under a metre over the
few hundred metres this simulation covers. No Qt in this file.
"""

import math

# Start position of the ROV (Sukhna Lake, Chandigarh). Change freely.
LAT0 = 30.7421
LON0 = 76.8188

METRES_PER_DEG_LAT = 111_320.0


def local_to_latlon(north, east, lat0=LAT0, lon0=LON0):
    lat = lat0 + north / METRES_PER_DEG_LAT
    lon = lon0 + east / (METRES_PER_DEG_LAT * math.cos(math.radians(lat0)))
    return lat, lon
