% rtcNewScene(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcNewScene - creates a new scene

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCScene rtcNewScene(RTCDevice device);

#### DESCRIPTION

This function creates a new scene bound to the specified device
(`device` argument), and returns a handle to this scene. The scene
object is reference counted with an initial reference count of 1. The
scene handle can be released using the `rtcReleaseScene` API call.

#### EXIT STATUS

On success a scene handle is returned. On failure `NULL` is returned
and an error code is set that can be queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcRetainScene], [rtcReleaseScene]
