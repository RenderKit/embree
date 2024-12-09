% rtcCommitBufferWithQueue(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcCommitBufferWithQueue - commits buffer content from host to device using a given SYCL queue

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcCommitBufferWithQueue(RTCBuffer buffer,
        sycl::queue queue, sycl::event* event);

#### DESCRIPTION

If the buffer was created using `rtcNewBufferHostDevice` the 
`rtcCommitBufferWithQueue` function commits changes of the host buffer data 
to the device. If the buffer was created with a non SYCL Embree device 
or the SYCL device has host unified memory, this call has no effect.

The call to `rtcCommitBufferWithQueue` will use the given SYCL queue
to copy the memory asynchronously. If the SYCL event argument `event`
is a valid pointer, Embree will use this pointer to return a copy of 
the SYCL event associated to the memory copy. The parameter `event` 
is optional and will be ignored if it is a null pointer.

The user is responsible for synchronization using the SYCL queue or 
the optional SYCL event.

This function is only avaiable on Embree versions with enabled SYCL support.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcCommitBuffer] [rtcNewBufferHostDevice]
