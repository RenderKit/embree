% rtcRetainBuffer(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcRetainBuffer - increments the buffer reference count

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcRetainBuffer(RTCBuffer buffer);

#### DESCRIPTION

Buffer objects are reference counted. The `rtcRetainBuffer` function
increments the reference count of the passed buffer object (`buffer`
argument). This function together with `rtcReleaseBuffer` allows to
use the internal reference counting in a C++ wrapper class to handle
the ownership of the object.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewBuffer], [rtcReleaseBuffer]
