% rtcGetSceneLinearBounds(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetSceneLinearBounds - returns the linear bounds of the scene

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTCORE_ALIGN(16) RTCLinearBounds
    {
      RTCBounds bounds0;
      RTCBounds bounds1;
    };

    void rtcGetSceneLinearBounds(
      RTCScene scene,
      struct RTCLinearBounds* bounds_o
    );

#### DESCRIPTION

The `rtcGetSceneLinearBounds` function queries the linear bounds of the
specified scene (`scene` argument) and stores them to the provided
destination pointer (`bounds_o` argument). The stored linear bounds
consist of bounding boxes for time 0 (`bounds0` member) and time 1
(`bounds1` member) as specified by the `RTCLinearBounds`
structure. Linearly interpolating these bounds to a specific time `t`
yields bounds for the geometry at that time.

The provided destination pointer must be aligned to 16 bytes. The
function may be called only after committing the scene, otherwise
the result is undefined.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcGetSceneBounds], [rtcCommitScene], [rtcJoinCommitScene]
