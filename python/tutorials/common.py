import pyembree as pe
import ctypes
import numpy as np

def create_triangle(device, scene, xshift = 0.0):
    g = pe.rtcNewGeometry(device, pe.RTC_GEOMETRY_TYPE_TRIANGLE)
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
    pe.rtcAttachGeometry(scene, g)
    pe.rtcReleaseGeometry(g)

def create_cornell_box(device, scene):
    def add_tri(v0, v1, v2, vert, indi, i):
        vert[i * 9 + 0] = v0[0]
        vert[i * 9 + 1] = v0[1]
        vert[i * 9 + 2] = v0[2]
        vert[i * 9 + 3] = v1[0]
        vert[i * 9 + 4] = v1[1]
        vert[i * 9 + 5] = v1[2]
        vert[i * 9 + 6] = v2[0]
        vert[i * 9 + 7] = v2[1]
        vert[i * 9 + 8] = v2[2]
        indi[i * 3 + 0] = i * 3 + 0
        indi[i * 3 + 1] = i * 3 + 1
        indi[i * 3 + 2] = i * 3 + 2
        return i + 1

    g = pe.rtcNewGeometry(device, pe.RTC_GEOMETRY_TYPE_TRIANGLE)
    N = 8
    vertices = pe.rtcSetNewGeometryBuffer(g, pe.RTC_BUFFER_TYPE_VERTEX, 0, pe.RTC_FORMAT_FLOAT3, 3*ctypes.sizeof(ctypes.c_float), N*3).as_float()
    indices = pe.rtcSetNewGeometryBuffer(g, pe.RTC_BUFFER_TYPE_INDEX, 0, pe.RTC_FORMAT_UINT3, 3*ctypes.sizeof(ctypes.c_uint), N).as_uint()

    # left
    index = add_tri((-5, 0, -5), (-5, 20, -5), (-5,  0, 5), vertices, indices, 0)
    index = add_tri((-5, 0,  5), (-5, 20, -5), (-5, 20, 5), vertices, indices, index)
    # back
    index = add_tri((-5, 0,  5), (-5, 20,  5), ( 5,  0, 5), vertices, indices, index)
    index = add_tri(( 5, 0,  5), (-5, 20,  5), ( 5, 20, 5), vertices, indices, index)
    # right
    index = add_tri(( 5, 0,  5), ( 5, 20,  5), ( 5,  0, -5), vertices, indices, index)
    index = add_tri(( 5, 0, -5), ( 5, 20,  5), ( 5, 20, -5), vertices, indices, index)
    # bottom
    index = add_tri((-5, 0, -5), (-5,  0,  5), ( 5,  0,  5), vertices, indices, index)
    index = add_tri((-5, 0, -5), ( 5,  0,  5), ( 5,  0, -5), vertices, indices, index)

    pe.rtcCommitGeometry(g)
    pe.rtcAttachGeometry(scene, g)
    pe.rtcReleaseGeometry(g)

    def add_box(cx, cy, cz, lx, ly, lz):
        g = pe.rtcNewGeometry(device, pe.RTC_GEOMETRY_TYPE_TRIANGLE)
        N = 12
        vertices = pe.rtcSetNewGeometryBuffer(g, pe.RTC_BUFFER_TYPE_VERTEX, 0, pe.RTC_FORMAT_FLOAT3, 3*ctypes.sizeof(ctypes.c_float), N*3).as_float()
        indices = pe.rtcSetNewGeometryBuffer(g, pe.RTC_BUFFER_TYPE_INDEX, 0, pe.RTC_FORMAT_UINT3, 3*ctypes.sizeof(ctypes.c_uint), N).as_uint()

        # left
        index = add_tri((cx-lx, cy-ly, cz-lz), (cx-lx, cy-ly, cz+lz), (cx-lx, cy+ly, cz-lz), vertices, indices, 0)
        index = add_tri((cx-lx, cy-ly, cz+lz), (cx-lx, cy+ly, cz+lz), (cx-lx, cy+ly, cz-lz), vertices, indices, index)
        # right
        index = add_tri((cx+lx, cy-ly, cz+lz), (cx+lx, cy-ly, cz-lz), (cx+lx, cy+ly, cz+lz), vertices, indices, index)
        index = add_tri((cx+lx, cy-ly, cz-lz), (cx+lx, cy+ly, cz-lz), (cx+lx, cy+ly, cz+lz), vertices, indices, index)
        # back
        index = add_tri((cx-lx, cy-ly, cz+lz), (cx+lx, cy-ly, cz+lz), (cx-lx, cy+ly, cz+lz), vertices, indices, index)
        index = add_tri((cx+lx, cy-ly, cz+lz), (cx+lx, cy+ly, cz+lz), (cx-lx, cy+ly, cz+lz), vertices, indices, index)
        # front
        index = add_tri((cx-lx, cy-ly, cz-lz), (cx-lx, cy+ly, cz-lz), (cx+lx, cy-ly, cz-lz), vertices, indices, index)
        index = add_tri((cx+lx, cy-ly, cz-lz), (cx-lx, cy+ly, cz-lz), (cx+lx, cy+ly, cz-lz), vertices, indices, index)
        # bottom
        index = add_tri((cx-lx, cy-ly, cz-lz), (cx+lx, cy-ly, cz+lz), (cx-lx, cy-ly, cz+lz), vertices, indices, index)
        index = add_tri((cx-lx, cy-ly, cz-lz), (cx+lx, cy-ly, cz-lz), (cx+lx, cy-ly, cz+lz), vertices, indices, index)
        # top
        index = add_tri((cx-lx, cy+ly, cz-lz), (cx-lx, cy+ly, cz+lz), (cx+lx, cy+ly, cz+lz), vertices, indices, index)
        index = add_tri((cx-lx, cy+ly, cz-lz), (cx+lx, cy+ly, cz+lz), (cx+lx, cy+ly, cz-lz), vertices, indices, index)

        pe.rtcCommitGeometry(g)
        pe.rtcAttachGeometry(scene, g)
        pe.rtcReleaseGeometry(g)

    add_box( 2, 5,-2,1,1,1)
    add_box(-2,15, 2,1,1,1)


    g = pe.rtcNewGeometry(device, pe.RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE)
    N = 1
    vertices = pe.rtcSetNewGeometryBuffer(g, pe.RTC_BUFFER_TYPE_VERTEX, 0, pe.RTC_FORMAT_FLOAT3, 3*ctypes.sizeof(ctypes.c_float), N).as_float()
    indices = pe.rtcSetNewGeometryBuffer(g, pe.RTC_BUFFER_TYPE_INDEX, 0, pe.RTC_FORMAT_UINT, ctypes.sizeof(ctypes.c_uint), N).as_uint()

    #vertices[ 0] = 0.0
    #vertices[ 1] = 0.0
    #vertices[ 2] = 0.0

    #vertices[ 3] = 3.0
    #vertices[ 4] = 0.0
    #vertices[ 5] = 0.0

    #vertices[ 6] = 3.0
    #vertices[ 7] = 9.0
    #vertices[ 8] = 0.0

    #vertices[ 9] = 0.0
    #vertices[10] = 9.0
    #vertices[11] = 0.0

    #indices[0] = 0

    pe.rtcCommitGeometry(g)
    pe.rtcAttachGeometry(scene, g)
    pe.rtcReleaseGeometry(g)




def create_ray(ox, oy, oz, dx, dy, dz):
    rayhit = pe.RTCRayHit()
    rayhit.ray.org_x = ox
    rayhit.ray.org_y = oy
    rayhit.ray.org_z = oz
    rayhit.ray.dir_x = dx
    rayhit.ray.dir_y = dy
    rayhit.ray.dir_z = dz
    rayhit.ray.tnear = 0
    rayhit.ray.tfar = float('inf')
    rayhit.ray.mask = 0xffffffff
    rayhit.ray.flags = 0
    rayhit.hit.geomID = pe.RTC_INVALID_GEOMETRY_ID
    rayhit.hit.instID[0] = pe.RTC_INVALID_GEOMETRY_ID
    return rayhit

def print_hit(rayhit):
    def fmt(x):
        return "{:.2f}".format(x)

    if rayhit.hit.geomID != pe.RTC_INVALID_GEOMETRY_ID:
        print(f"Ray ({fmt(rayhit.ray.org_x)}, {fmt(rayhit.ray.org_y)}, {fmt(rayhit.ray.org_z)}) -> ({fmt(rayhit.ray.dir_x)}, {fmt(rayhit.ray.dir_y)}, {fmt(rayhit.ray.dir_z)}):  Found intersection on geometry {rayhit.hit.geomID}, primitive {rayhit.hit.primID} at tfar={rayhit.ray.tfar}")
    else:
        print(f"Ray ({fmt(rayhit.ray.org_x)}, {fmt(rayhit.ray.org_y)}, {fmt(rayhit.ray.org_z)}) -> ({fmt(rayhit.ray.dir_x)}, {fmt(rayhit.ray.dir_y)}, {fmt(rayhit.ray.dir_z)}):  Did not find any intersection")



