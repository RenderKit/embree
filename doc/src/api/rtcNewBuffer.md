% rtcNewBuffer(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcNewBuffer - creates a new data buffer

#### SYNOPSIS

    #include <embree4/rtcore.h>

    RTCBuffer rtcNewBuffer(
      RTCDevice device,
      size_t byteSize
    );

#### DESCRIPTION

The `rtcNewBuffer` function creates a new data buffer object of
specified size in bytes (`byteSize` argument) that is bound to the
specified device (`device` argument). The buffer object is reference
counted with an initial reference count of 1. The returned buffer
object can be released using the `rtcReleaseBuffer` API call. The
specified number of bytes are allocated at buffer construction time
and deallocated when the buffer is destroyed.

``` {include=src/api/inc/buffer_padding.md}
```

If the `device` is a Embree SYCL device, the buffer will be allocated
using SYCL USM shared memory, i.e. the buffer can be accessed on the host
and device (GPU) and the SYCL runtime will handle buffer transfers automatically.

For precise control over when memory is copied from host to device, 
a buffer can also be created using `rtcNewBufferHostDevice`.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcNewBufferHostDevice], [rtcRetainBuffer], [rtcReleaseBuffer]
