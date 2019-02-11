% rtcNewBuffer(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcNewBuffer - creates a new data buffer

#### SYNOPSIS

    #include <embree3/rtcore.h>

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

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcRetainBuffer], [rtcReleaseBuffer]
