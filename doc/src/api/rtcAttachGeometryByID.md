% rtcAttachGeometryByID(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcAttachGeometryByID - attaches a geometry to the scene
      using a specified geometry ID

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcAttachGeometryByID(
      RTCScene scene,
      RTCGeometry geometry,
      unsigned int geomID
    );

#### DESCRIPTION

The `rtcAttachGeometryByID` function attaches a geometry (`geometry`
argument) to a scene (`scene` argument) and assigns a user provided
geometry ID (`geomID` argument) to that geometry. All geometries
attached to a scene are defined to be included inside the scene. A
geometry can only get attached to a single scene at a given
time. However, it is possible to detach and re-attach a geometry to a
different scene. The passed user-defined geometry ID is used to
identify the geometry when hit by a ray during ray queries. Using this
function, it is possible to share the same IDs to refer to geometries
inside the application and Embree.

This function is thread-safe, thus multiple threads can attach
geometries to a scene in parallel.

The user-provided geometry ID must be unused in the scene, otherwise
the creation of the geometry will fail. Further, the user-provided
geometry IDs should be compact, as Embree internally creates a vector
which size is equal to the largest geometry ID used. Creating very
large geometry IDs for small scenes would thus cause a memory
consumption and performance overhead.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcAttachGeometry]
