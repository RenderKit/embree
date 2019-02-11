% rtcGetGeometry(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetGeometry - returns the geometry bound to
      the specified geometry ID

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCGeometry rtcGetGeometry(RTCScene scene, unsigned int geomID);

#### DESCRIPTION

The `rtcGetGeometry` function returns the geometry that is bound to
the specified geometry ID (`geomID` argument) for the specified scene
(`scene` argument). This function just looks up the handle and does
*not* increment the reference count. If you want to get ownership of
the handle, you need to additionally call `rtcRetainGeometry`. For this
reason, this function is fast and can be used during rendering.
However, it is generally recommended to store the geometry handle
inside the application's geometry representation and look up the
geometry handle from that representation directly.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcAttachGeometry], [rtcAttachGeometryByID]
