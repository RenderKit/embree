# %%
# We setup an embree device and scene as we did in the minimal tutorial
# We also declare some helper functions to create geometry, rays and output intersection results, see the minimal tutorial for more information on this
from common import create_triangle, create_ray, print_hit
import pyembree as pe
import ctypes
import numpy as np

d = pe.rtcNewDevice(None)
s = pe.rtcNewScene(d)

# We crate a triangle
create_triangle(d, s)
create_triangle(d, s, 3)
pe.rtcCommitScene(s)


# %%
# This is our intersection filter function
# This function will be called for every hit that is reported during an intersict call
def intersect_filter(args):
  print(f"Hello World from intersect! >>> geomID = {pe.RTCHitN_geomID(args.hit, args.N, 0)}")

# To set the filter function per geometry we use rtcSetGeomotryIntersectFilterFunction
g = pe.rtcGetGeometry(s, 0)
pe.rtcSetGeometryIntersectFilterFunction(g, intersect_filter)

# Two rays, one will hit the first triangle with the filter function set
will_hit = create_ray(0.33, 0.33, -1, 0, 0, 1)
wont_hit = create_ray(1.00, 1.00, -1, 0, 0, 1)

pe.rtcIntersect1(s, will_hit, None)
pe.rtcIntersect1(s, wont_hit, None)
print_hit(will_hit)
print_hit(wont_hit)

# %%
# Two rays, one will hit the second triangle but no filter function is set for this geometry
# No "Hello world" will be printed this time
will_hit = create_ray(3.33, 0.33, -1, 0, 0, 1)
wont_hit = create_ray(4.00, 1.00, -1, 0, 0, 1)
pe.rtcIntersect1(s, will_hit, None)
pe.rtcIntersect1(s, wont_hit, None)
print_hit(will_hit)
print_hit(wont_hit)

pe.rtcReleaseScene(s)
pe.rtcReleaseDevice(d)


