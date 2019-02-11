% rtcDetachGeometry(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcDetachGeometry - detaches a geometry from the scene

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcDetachGeometry(RTCScene scene, unsigned int geomID);

#### DESCRIPTION

This function detaches a geometry identified by its geometry ID
(`geomID` argument) from a scene (`scene` argument). When detached, the
geometry is no longer contained in the scene.

This function is thread-safe, thus multiple threads can detach
geometries from a scene at the same time.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcAttachGeometry], [rtcAttachGeometryByID]
