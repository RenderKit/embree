% rtcEnableGeometry(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcEnableGeometry - enables the geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcEnableGeometry(RTCGeometry geometry);

#### DESCRIPTION

The `rtcEnableGeometry` function enables the specified geometry
(`geometry` argument). Only enabled geometries are rendered. Each
geometry is enabled by default at construction time.

After enabling a geometry, the scene containing that geometry must be
committed using `rtcCommitScene` for the change to have effect.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry], [rtcDisableGeometry], [rtcCommitScene]
