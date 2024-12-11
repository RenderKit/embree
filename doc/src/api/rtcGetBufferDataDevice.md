% rtcGetBufferData(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetBufferDataDevice - gets a device pointer to the buffer data

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void* rtcGetBufferDataDevice(RTCBuffer buffer);

#### DESCRIPTION

The `rtcGetBufferDataDevice` function returns a pointer to the buffer data
of the specified buffer object (`buffer` argument) which can be used for
accessing the data on the device. If Embree has no SYCL support or the SYCL
device has host unified memory, the pointer is equal to the pointer returned
by `rtcGetBufferData`.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcGetBufferData], [rtcNewBuffer]
