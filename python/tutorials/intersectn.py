# %%
# We setup an embree device and scene as we did in the minimal tutorial
# We also declare some helper functions to create geometry, rays and output intersection results
from common import create_triangle, create_ray, print_hit
import pyembree as pe
import ctypes
import numpy as np

d = pe.rtcNewDevice(None)
s = pe.rtcNewScene(d)

# We crate two triangles
create_triangle(d, s)
create_triangle(d, s, 3)
pe.rtcCommitScene(s)

# %%
# Collect rays in a python list
rayhits = [create_ray(0.33, 0.33, -1, 0, 0, 1), create_ray(1.00, 1.00, -1, 0, 0, 1), create_ray(3.33, 0.33, -1, 0, 0, 1), create_ray(4, 1, -1, 0, 0, 1)]
# Intersect all rays in the list.
# Intersection calls are done in parallel, we can define the number of threads used in the last function argument. 0 will use the maximum of the hardware capabilities.
pe.rtcIntersectN(s, rayhits, None, 0)
for rh in rayhits:
  print_hit(rh)

pe.rtcReleaseScene(s)
pe.rtcReleaseDevice(d)


