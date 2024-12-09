% rtcNewBuffer(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcNewBufferHostDevice - creates a new data buffer with 
      explicitly managed host and device memory allocations

#### SYNOPSIS

    #include <embree4/rtcore.h>

    RTCBuffer rtcNewBufferHostDevice(
      RTCDevice device,
      size_t byteSize
    );

#### DESCRIPTION

The `rtcNewBufferHostDevice` function creates a new data buffer object of
specified size in bytes (`byteSize` argument) that is bound to the
specified device (`device` argument). The buffer object is reference
counted with an initial reference count of 1. The returned buffer
object can be released using the `rtcReleaseBuffer` API call. If Embree has SYCL
support enabled and the SYCL device has no host unifed memory (e.g, a discrete GPU), 
the buffer allocates memory on the host and device explicitly.
After the buffer is modified on the host `rtcCommitBuffer` can be used to synchronize
host and device memory by copying the buffer content from the host to device. If the Embree
version has no SYCL support or the SYCL device has host unified memory, the buffer will behave 
the same as a buffer created using `rtcNewBuffer`. The
specified number of bytes are allocated at buffer construction time
and deallocated when the buffer is destroyed.

``` {include=src/api/inc/buffer_padding.md}
```

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcCommitBuffer], [rtcNewBuffer], [rtcRetainBuffer], [rtcReleaseBuffer]
