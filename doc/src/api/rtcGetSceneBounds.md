% rtcGetSceneBounds(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetSceneBounds - returns the axis-aligned bounding box of the scene

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTCORE_ALIGN(16) RTCBounds
    {
      float lower_x, lower_y, lower_z, align0;
      float upper_x, upper_y, upper_z, align1;
    };

    void rtcGetSceneBounds(
      RTCScene scene,
      struct RTCBounds* bounds_o
    );

#### DESCRIPTION

The `rtcGetSceneBounds` function queries the axis-aligned bounding box
of the specified scene (`scene` argument) and stores that bounding box
to the provided destination pointer (`bounds_o` argument). The stored
bounding box consists of lower and upper bounds for the x, y, and z
dimensions as specified by the `RTCBounds` structure.

The provided destination pointer must be aligned to 16 bytes. The
function may be invoked only after committing the scene; otherwise the
result is undefined.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcGetSceneLinearBounds], [rtcCommitScene], [rtcJoinCommitScene]
