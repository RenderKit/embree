% rtcGetSceneFlags(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetSceneFlags - returns the flags of the scene

#### SYNOPSIS

    #include <embree3/rtcore.h>

    enum RTCSceneFlags rtcGetSceneFlags(RTCScene scene);

#### DESCRIPTION

Queries the flags of a scene. This function can be useful when
setting individual flags, e.g. to just set the robust mode without
changing other flags the following way:

    RTCSceneFlags flags = rtcGetSceneFlags(scene);
    rtcSetSceneFlags(scene, RTC_SCENE_FLAG_ROBUST | flags);

#### EXIT STATUS

On failure `RTC_SCENE_FLAG_NONE` is returned and an error code is set
that can be queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcSetSceneFlags]
