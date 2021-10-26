% rtcGetGeometryThreadSafe(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetGeometryThreadSafe - returns the geometry bound to
      the specified geometry ID

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCGeometry rtcGetGeometryThreadSafe(RTCScene scene, unsigned int geomID);

#### DESCRIPTION

The `rtcGetGeometryThreadSafe` function returns the geometry that is bound to
the specified geometry ID (`geomID` argument) for the specified scene
(`scene` argument). This function just looks up the handle and does
*not* increment the reference count. If you want to get ownership of
the handle, you need to additionally call `rtcRetainGeometry`.

This function is thread safe and should NOT get used during rendering.
If you need a fast non-thread safe version during rendering please use
the [rtcGetGeometry] function.


#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcAttachGeometry], [rtcAttachGeometryByID], [rtcGetGeometry]
