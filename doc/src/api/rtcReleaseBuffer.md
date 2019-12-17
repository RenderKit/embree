% rtcReleaseBuffer(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcReleaseBuffer - decrements the buffer reference count

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcReleaseBuffer(RTCBuffer buffer);

#### DESCRIPTION

Buffer objects are reference counted. The `rtcReleaseBuffer` function
decrements the reference count of the passed buffer object (`buffer`
argument). When the reference count falls to 0, the buffer gets
destroyed.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewBuffer], [rtcRetainBuffer]
