% rtcDisableGeometry(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcDisableGeometry - disables the geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcDisableGeometry(RTCGeometry geometry);

#### DESCRIPTION

The `rtcDisableGeometry` function disables the specified geometry
(`geometry` argument). A disabled geometry is not rendered. Each
geometry is enabled by default at construction time.

After disabling a geometry, the scene containing that geometry must be
committed using `rtcCommitScene` for the change to have effect.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry], [rtcEnableGeometry], [rtcCommitScene]
