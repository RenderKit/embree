% rtcSetSharedGeometryBuffer(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetSharedGeometryBuffer - assigns a view of a shared data buffer
      to a geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetSharedGeometryBuffer(
      RTCGeometry geometry,
      enum RTCBufferType type,
      unsigned int slot,
      enum RTCFormat format,
      const void* ptr,
      size_t byteOffset,
      size_t byteStride,
      size_t itemCount
    );

#### DESCRIPTION

The `rtcSetSharedGeometryBuffer` function binds a view of a
shared user-managed data buffer (`ptr` argument) to a geometry buffer
type and slot (`type` and `slot` argument) of the specified geometry
(`geometry` argument).

One can specify the start of the first buffer element in bytes
(`byteOffset` argument), the byte stride between individual buffer
elements (`byteStride` argument), the format of the buffer elements
(`format` argument), and the number of elements to bind (`itemCount`).

The start address (`byteOffset` argument) and stride (`byteStride`
argument) must be both aligned to 4 bytes; otherwise the
`rtcSetGeometryBuffer` function will fail.

``` {include=src/api/inc/buffer_padding.md}
```

The buffer data must remain valid for as long as the buffer may be
used, and the user is responsible for freeing the buffer data when no
longer required.

Sharing buffers can significantly reduce the memory required by the
application, thus we recommend using this feature. When enabling the
`RTC_SCENE_COMPACT` scene flag, the spatial index structures index
into the vertex buffer, resulting in even higher memory savings.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryBuffer], [rtcSetNewGeometryBuffer]
