% rtcGetBufferData(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetBufferData - gets a pointer to the buffer data

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void* rtcGetBufferData(RTCBuffer buffer);

#### DESCRIPTION

The `rtcGetBufferData` function returns a pointer to the buffer data
of the specified buffer object (`buffer` argument).

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewBuffer]
