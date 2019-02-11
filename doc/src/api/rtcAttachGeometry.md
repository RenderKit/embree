% rtcAttachGeometry(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcAttachGeometry - attaches a geometry to the scene

#### SYNOPSIS

    #include <embree3/rtcore.h>

    unsigned int rtcAttachGeometry(
      RTCScene scene,
      RTCGeometry geometry
    );

#### DESCRIPTION

The `rtcAttachGeometry` function attaches a geometry (`geometry`
argument) to a scene (`scene` argument) and assigns a geometry ID to
that geometry. All geometries attached to a scene are defined to be
included inside the scene. A geometry can only get attached to a
single scene at a given time. However, it is possible to detach and
re-attach a geometry to a different scene. The geometry ID is unique
for the scene, and is used to identify the geometry when hit by a ray
during ray queries.

This function is thread-safe, thus multiple threads can attach
geometries to a scene in parallel.

The geometry IDs are assigned sequentially, starting from 0, as long
as no geometry got detached. If geometries got detached, the
implementation will reuse IDs in an implementation dependent
way. Consequently sequential assignment is no longer guaranteed, but a
compact range of IDs.

These rules allow the application to manage a dynamic array to
efficiently map from geometry IDs to its own geometry representation.
Alternatively, the application can also use per-geometry user data to
map to its geometry representation. See `rtcSetGeometryUserData` and
`rtcGetGeometryUserData` for more information.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryUserData], [rtcGetGeometryUserData]
