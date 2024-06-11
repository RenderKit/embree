# import pyembree
# make sure that the path with pyembree binaries is added to the $PYTHONPATH environment variable
import pyembree as pe
import ctypes

# We first create several Embree objects:
#  * an embree device with a default configuration (see sec. 5.1 and 7.1 in the documention for details)
#  * an embree scene to hold our geometry
d = pe.rtcCreateDevice(None, True, False)
s = pe.rtcNewScene(d)

# A scene is a collection of geometry objects. Scenes are what the intersect / occluded API functions work on. 
# You can think of a scene as an acceleration structure, e.g. a bounding-volume hierarchy.
#
# Next, we instruct Embree to create vertex and index buffers into which we copy the geomety data. 
# For complex scenes, shared buffers are often better choice but special care must be taken to ensure proper alignment and padding. 
# This is described in more detail in the API documentation.
g = pe.rtcNewGeometry(d, pe.RTC_GEOMETRY_TYPE_TRIANGLE)
vertices = pe.rtcSetNewGeometryBuffer(g, pe.RTC_BUFFER_TYPE_VERTEX, 0, pe.RTC_FORMAT_FLOAT3, 3*ctypes.sizeof(ctypes.c_float), 3).as_float()
indices = pe.rtcSetNewGeometryBuffer(g, pe.RTC_BUFFER_TYPE_INDEX, 0, pe.RTC_FORMAT_UINT3, 3*ctypes.sizeof(ctypes.c_uint), 1).as_uint()
vertices[0] = 0.0
vertices[1] = 0.0
vertices[2] = 0.0
vertices[3] = 1.0
vertices[4] = 0.0
vertices[5] = 0.0
vertices[6] = 0.0
vertices[7] = 1.0
vertices[8] = 0.0
indices[0] = 0
indices[1] = 1
indices[2] = 2

# After setting the geometry data the geometry can be committed and attached to the scene. 
# Embree objects such as RTCDevice, RTCScene and RTCGeometry are reference-counted. 
# This means, that the scene takes ownership of the geometry when we attach it and we can release the geometry handle. 
# The API function rtcAttachGeometry returns a geometry ID which can be used identify intersected objects when the scene contains multiple geometry objects. 
# We finish the setup by committing the scene, after which the scene can be intersected.
pe.rtcCommitGeometry(g)
pe.rtcAttachGeometry(s, g)
pe.rtcReleaseGeometry(g)
    
pe.rtcCommitScene(s)

# To perform a ray intersect, we obviously have to create a ray first. 
# Embree also expects an intersection context which can be used for advanced features such as intersection filters and instancing.
def create_ray(ox, oy, oz, dx, dy, dz):
    rayhit = pe.RTCRayHit()
    rayhit.ray.org_x = ox
    rayhit.ray.org_y = oy
    rayhit.ray.org_z = oz
    rayhit.ray.dir_x = dx
    rayhit.ray.dir_y = dy
    rayhit.ray.dir_z = dz
    # We set the ray values tnear to 0 and tfar to infinity to indicate that the ray starts at origin (org_x, org_y, org_z) and is unbounded. 
    rayhit.ray.tnear = 0
    rayhit.ray.tfar = float('inf')
    rayhit.ray.mask = 0xffffffff
    rayhit.ray.flags = 0
    # We also set geomID to RTC_INVALID_GEOMETRY_ID. 
    rayhit.hit.geomID = pe.RTC_INVALID_GEOMETRY_ID
    rayhit.hit.instID[0] = pe.RTC_INVALID_GEOMETRY_ID
    return rayhit

will_hit = create_ray(0.33, 0.33, -1, 0, 0, 1)
wont_hit = create_ray(1, 1, -1, 0, 0, 1)

# Now we can call Embree's intersect function and check if the ray intersects the triangle.
pe.rtcIntersect1(s, will_hit, None)
pe.rtcIntersect1(s, wont_hit, None)

# We can read out the results now:
# If an intersection occured
#   * geomID will contain the ID of the geometry that has been intersected. 
#   * the value tfar will contain the ray parameter which we can compute the point at which the ray and triangle intersect as (org_x, org_y, org_z) + t * (dir_x, dir_y, dir_z).
def print_hit(rayhit):
    if rayhit.hit.geomID != pe.RTC_INVALID_GEOMETRY_ID:
        print(f"Ray ({rayhit.ray.org_x}, {rayhit.ray.org_y}, {rayhit.ray.org_z}) -> ({rayhit.ray.dir_x}, {rayhit.ray.dir_y}, {rayhit.ray.dir_z}):  Found intersection on geometry {rayhit.hit.geomID}, primitive {rayhit.hit.primID} at tfar={rayhit.ray.tfar}")
    else:
        print(f"Ray ({rayhit.ray.org_x}, {rayhit.ray.org_y}, {rayhit.ray.org_z}) -> ({rayhit.ray.dir_x}, {rayhit.ray.dir_y}, {rayhit.ray.dir_z}):  Did not find any intersection")

print_hit(will_hit)
print_hit(wont_hit)

# At the end of an application all resources that where allocated through Embree should be released. 
# Note that the geometry will be released automatically, when the (last) scene the geometry is attached to is released.
pe.rtcReleaseScene(s)
pe.rtcReleaseDevice(d)




