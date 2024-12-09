% rtcGetBufferData(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetBufferData - gets a pointer to the buffer data

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void* rtcGetBufferData(RTCBuffer buffer);

#### DESCRIPTION

The `rtcGetBufferData` function returns a pointer to the buffer data
of the specified buffer object (`buffer` argument). If the buffer
was created using `rtcNewBufferHostDevice` and the SYCL device has no
host unified memory, this pointer is only valid on the host. To get
a device pointer in this case, use `rtcGetBufferDataDevice`.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcGetBufferDataDevice], [rtcNewBuffer], [rtcNewBufferHostDevice]
