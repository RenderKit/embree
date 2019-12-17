% rtcCommitGeometry(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcCommitGeometry - commits geometry changes

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcCommitGeometry(RTCGeometry geometry);

#### DESCRIPTION

The `rtcCommitGeometry` function is used to commit all geometry
changes performed to a geometry (`geometry` parameter). After a
geometry gets modified, this function must be called to properly
update the internal state of the geometry to perform interpolations
using `rtcInterpolate` or to commit a scene containing the geometry
using `rtcCommitScene`.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcInterpolate], [rtcCommitScene]
