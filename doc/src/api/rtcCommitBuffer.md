% rtcCommitBuffer(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcCommitBuffer - commits buffer content from host to device

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcCommitBuffer(RTCBuffer buffer);

#### DESCRIPTION

If the buffer was created using `rtcNewBufferHostDevice` the
`rtcCommitBuffer` function commits changes of the host buffer data
to the device. This call is not necessary if the buffer was created
with a non SYCL Embree device.

The call to `rtcCommitBuffer` will internally use a temporary SYCL
queue and wait for the memory copy to finish. The function
`rtcCommitBufferWithQueue` can be used to asyncronously copy the
data to the device.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcCommitBufferWithQueue] [rtcNewBufferHostDevice]
