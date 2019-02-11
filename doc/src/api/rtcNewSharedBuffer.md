% rtcNewSharedBuffer(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcNewSharedBuffer - creates a new shared data buffer

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCBuffer rtcNewSharedBuffer(
      RTCDevice device,
      void* ptr,
      size_t byteSize
    );

#### DESCRIPTION

The `rtcNewSharedBuffer` function creates a new shared data buffer
object bound to the specified device (`device` argument). The buffer
object is reference counted with an initial reference count of 1. The
buffer can be released using the `rtcReleaseBuffer` function.

At construction time, the pointer to the user-managed buffer data
(`ptr` argument) including its size in bytes (`byteSize` argument) is
provided to create the buffer. At buffer construction time no buffer
data is allocated, but the buffer data provided be the application is
used. The buffer data must remain valid for as long as the buffer may
be used, and the user is responsible to free the buffer data when no
longer required.

``` {include=src/api/inc/buffer_padding.md}
```

The data pointer (`ptr` argument) must be aligned to 4 bytes; otherwise
the `rtcNewSharedBuffer` function will fail.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcRetainBuffer], [rtcReleaseBuffer]
