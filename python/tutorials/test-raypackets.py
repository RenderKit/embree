import pyembree as pe
import ctypes
import numpy as np


def cast_ptr(capsule, restype):
    ctypes.pythonapi.PyCapsule_GetPointer.restype = ctypes.POINTER(restype)
    ctypes.pythonapi.PyCapsule_GetPointer.argtypes = [ctypes.py_object, ctypes.c_char_p]
    return ctypes.pythonapi.PyCapsule_GetPointer(capsule, None)

class Hit:
    def __init__(self):
        self.found_hit = False
        self.geomID = pe.RTC_INVALID_GEOMETRY_ID
        self.primID = pe.RTC_INVALID_GEOMETRY_ID
        self.tfar = float('inf')

def intersect_filter(args):
    return
    print("Hello World from intersect!")

d = pe.rtcNewDevice(None)
s = pe.rtcNewScene(d)

def create_triangle(scene, xshift = 0.0):
    g = pe.rtcNewGeometry(d, pe.RTC_GEOMETRY_TYPE_TRIANGLE)
    vertices = pe.rtcSetNewGeometryBuffer(g, pe.RTC_BUFFER_TYPE_VERTEX, 0, pe.RTC_FORMAT_FLOAT3, 3*ctypes.sizeof(ctypes.c_float), 3).as_float()
    indices = pe.rtcSetNewGeometryBuffer(g, pe.RTC_BUFFER_TYPE_INDEX, 0, pe.RTC_FORMAT_UINT3, 3*ctypes.sizeof(ctypes.c_uint), 1).as_uint()
    vertices[0] = 0.0 + xshift
    vertices[1] = 0.0
    vertices[2] = 0.0
    vertices[3] = 1.0 + xshift
    vertices[4] = 0.0
    vertices[5] = 0.0
    vertices[6] = 0.0 + xshift
    vertices[7] = 1.0
    vertices[8] = 0.0
    indices[0] = 0
    indices[1] = 1
    indices[2] = 2
    pe.rtcCommitGeometry(g)
    pe.rtcAttachGeometry(s, g)
    pe.rtcReleaseGeometry(g)
    
create_triangle(s)
create_triangle(s, 3)
pe.rtcCommitScene(s)

def setup_Ray4(rayhit, N, ox, oy, oz, dx, dy, dz):
    rayhit.ray.org_x[N] = ox
    rayhit.ray.org_y[N] = oy
    rayhit.ray.org_z[N] = oz
    rayhit.ray.dir_x[N] = dx
    rayhit.ray.dir_y[N] = dy
    rayhit.ray.dir_z[N] = dz
    rayhit.ray.tnear[N] = 0
    rayhit.ray.tfar[N] = float('inf')
    rayhit.ray.mask[N] = 0xffffffff
    rayhit.ray.flags[N] = 0
    rayhit.hit.geomID[N] = pe.RTC_INVALID_GEOMETRY_ID

rayhit4 = pe.RTCRayHit4()
setup_Ray4(rayhit4, 0, 0.33, 0.33, -1, 0, 0, 1)
setup_Ray4(rayhit4, 1, 1.00, 1.00, -1, 0, 0, 1)
setup_Ray4(rayhit4, 2, 3.33, 0.33, -1, 0, 0, 1)
setup_Ray4(rayhit4, 3, 4.00, 1.00, -1, 0, 0, 1)

valid = np.array([1,1,1,1])

pe.rtcIntersect4(valid, s, rayhit4, None)

for i in range(0, 4):
    print(f"ray ({rayhit4.ray.org_x[i]}, {rayhit4.ray.org_y[i]}, {rayhit4.ray.org_z[i]})")
    if rayhit4.hit.geomID[i] != pe.RTC_INVALID_GEOMETRY_ID:
      print(f"Found intersection with geometry {rayhit4.hit.geomID[i]}, primitive {rayhit4.hit.primID[i]} at tfar={rayhit4.ray.tfar[i]}")

    else:
      print("Did not find any intersection")


pe.rtcReleaseScene(s)
pe.rtcReleaseDevice(d)