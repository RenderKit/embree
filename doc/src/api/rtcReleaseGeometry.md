% rtcReleaseGeometry(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcReleaseGeometry - decrements the geometry reference count

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcReleaseGeometry(RTCGeometry geometry);

#### DESCRIPTION

Geometry objects are reference counted. The `rtcReleaseGeometry`
function decrements the reference count of the passed geometry object
(`geometry` argument). When the reference count falls to 0, the
geometry gets destroyed.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry], [rtcRetainGeometry]
