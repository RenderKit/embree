% rtcRetainScene(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcRetainScene - increments the scene reference count

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcRetainScene(RTCScene scene);

#### DESCRIPTION

Scene objects are reference counted. The `rtcRetainScene` function
increments the reference count of the passed scene object (`scene`
argument). This function together with `rtcReleaseScene` allows to
use the internal reference counting in a C++ wrapper class to handle
the ownership of the object.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewScene], [rtcReleaseScene]
