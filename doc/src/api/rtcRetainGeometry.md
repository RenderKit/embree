% rtcRetainGeometry(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcRetainGeometry - increments the geometry reference count

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcRetainGeometry(RTCGeometry geometry);

#### DESCRIPTION

Geometry objects are reference counted. The `rtcRetainGeometry`
function increments the reference count of the passed geometry object
(`geometry` argument). This function together with `rtcReleaseGeometry`
allows to use the internal reference counting in a C++ wrapper class to
handle the ownership of the object.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry], [rtcReleaseGeometry]
