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
def intersect_filter(args):
  print("Hello World from intersect!")

will_hit = create_ray(0.33, 0.33, -1, 0, 0, 1)
wont_hit = create_ray(1.00, 1.00, -1, 0, 0, 1)

intersect_args = pe.RTCIntersectArguments()
intersect_args.flags = pe.RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER
intersect_args.filter = intersect_filter

pe.rtcIntersect1(s, will_hit, intersect_args)
pe.rtcIntersect1(s, wont_hit, intersect_args)
print_hit(will_hit)
print_hit(wont_hit)

pe.rtcReleaseScene(s)
pe.rtcReleaseDevice(d)


