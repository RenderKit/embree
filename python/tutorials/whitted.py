# %%
# We setup an embree device and scene as we did in the minimal tutorial
# We also declare some helper functions to create geometry, rays and output intersection results, see the minimal tutorial for more information on this
#from common import create_triangle, create_ray, print_hit

import pyrootutils
import os
pyrootutils.setup_root(os.getcwd(), indicator="CMakeLists.txt", pythonpath=True)

from tutorials.common import create_triangle, create_cornell_box, create_ray, print_hit

import pyembree as pe
import ctypes
import numpy as np

d = pe.rtcNewDevice(None)
s = pe.rtcNewScene(d)

# We crate two triangles
create_cornell_box(d, s)
pe.rtcCommitScene(s)

# Create an image
from PIL import Image
im = Image.new(mode = "RGB", size=(256, 256), color = (0, 0, 0))

# Collect rays in a python list, use an othrographic projection to create rays
camera_pos = (0,15,-15)
camera_at = (0,10,0)
camera_up = (0,1,0)

import math
def vector_add(a, b):
  return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def vector_sub(a, b):
  return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def vector_mul(a, t):
  return (a[0] * t, a[1] * t, a[2] * t)
def vector_len(a):
  return math.sqrt(pow(a[0], 2) + pow(a[1], 2) + pow(a[2], 2))
def vector_norm(a):
  return vector_mul(a, 1.0/vector_len(a))
def vector_cross(a, b):
  return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0])
def vector_dot(a, b):
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
def vector_color(a):
  return (int(a[0]*255), int(a[1]*255), int(a[2]*255))

# local camera coordinates
camera_z = vector_norm(vector_sub(camera_at, camera_pos))
camera_x = vector_norm(vector_cross(camera_up, camera_z))
camera_y = vector_norm(vector_cross(camera_z, camera_x))

print("camera")
print(f"  pos: {camera_pos}")
print(f"  at:  {camera_at}")
print(f"  up:  {camera_up}")
print(f"  dz:  {camera_z}")
print(f"  dy:  {camera_y}")
print(f"  dx:  {camera_x}")

fov = 45.0
aspect = float(im.size[0]) / float(im.size[1])

camera_dx = vector_mul(camera_x, math.tan(fov * math.pi / 180))
camera_dy = vector_mul(camera_y, math.tan(fov * math.pi / 180) / aspect)
camera_botleft = vector_sub(camera_z,       camera_dx)
camera_botleft = vector_sub(camera_botleft, camera_dy)
camera_dx = vector_mul(camera_dx, 2.0/im.size[0])
camera_dy = vector_mul(camera_dy, 2.0/im.size[1])

def project_pinhole(x,y):
  d = vector_add(camera_botleft, vector_mul(camera_dx, x))
  d = vector_add(d,              vector_mul(camera_dy, y))
  x,y,z = camera_pos
  dx,dy,dz = vector_norm(d)

  return (x,y,z,dx,dy,dz)
  
import itertools
xy = list(itertools.product(range(0, im.size[0]), range(0, im.size[1])))
print(f"generating {len(xy)} rays...")
rays = [project_pinhole(x,y) for x,y in xy]
rays = list(zip(*rays))
ox = np.array(rays[0])
oy = np.array(rays[1])
oz = np.array(rays[2])
dx = np.array(rays[3])
dy = np.array(rays[4])
dz = np.array(rays[5])
ids = np.arange(0, len(xy))
rayhits = pe.rtcCreateRayHits(ox, oy, oz, dx, dy, dz, ids)

# Intersect all rays in the list.
# Intersection calls are done in parallel, we can define the number of threads used in the last function argument. 0 will use the maximum of the hardware capabilities.
print(f"intersecting primary rays...")
pe.rtcIntersectN(s, rayhits, None, 0)

print(f"generating shadow rays... "),
LQ = (0,20,0)
def shadowray(rh):
  if rh.hit.geomID == pe.RTC_INVALID_GEOMETRY_ID:
    return (0, rh)
  else:
    # intersection with geometry of rh is origin of shadow ray
    o = (rh.ray.org_x + rh.ray.dir_x * rh.ray.tfar, rh.ray.org_y + rh.ray.dir_y * rh.ray.tfar, rh.ray.org_z + rh.ray.dir_z * rh.ray.tfar)
    # connect geometry intersection with light source
    d = vector_norm(vector_sub(LQ, o))
    # prevent self intersection
    o = vector_add(o, vector_mul(d, 0.001))

    rh.ray.org_x = o[0]
    rh.ray.org_y = o[1]
    rh.ray.org_z = o[2]
    rh.ray.dir_x = d[0]
    rh.ray.dir_y = d[1]
    rh.ray.dir_z = d[2]

    return (1, rh)

shadowrayhits = pe.rtcTransformRayHits(rayhits, shadowray)
print(f"    ...{len(shadowrayhits)} generated")

print(f"intersecting shadow rays...")
pe.rtcIntersectN(s, shadowrayhits, None, 0)

print("shading...")
pixels = im.load()
for rh in shadowrayhits:
  y = int(rh.ray.id % im.size[0])
  x = int(rh.ray.id / im.size[0])

  interesect_geom = rayhits[rh.ray.id].hit.geomID
  diffuse = (1.0, 1.0, 1.0)
  if interesect_geom == 1:
    diffuse = (1.0,0.0,0.0)
  elif interesect_geom == 2:
    diffuse = (0.0,1.0,0.0)

  if rh.hit.geomID == pe.RTC_INVALID_GEOMETRY_ID:
    L = vector_sub(LQ, (rh.ray.org_x, rh.ray.org_y, rh.ray.org_z))
    V = vector_sub(camera_pos, (rh.ray.org_x, rh.ray.org_y, rh.ray.org_z))
    N = vector_norm((rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z))
    H = vector_norm(vector_add(L, V))
    pixels[x, y] = vector_color(vector_mul(diffuse, vector_dot(N, H)))
  else:
    pixels[x, y] = vector_color(vector_mul(diffuse, 0.1))
  
im = im.transpose(method=Image.FLIP_TOP_BOTTOM)
im.show()

pe.rtcReleaseScene(s)
pe.rtcReleaseDevice(d)