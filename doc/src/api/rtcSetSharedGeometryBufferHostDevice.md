% rtcSetSharedGeometryBufferHostDevice(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcSetSharedGeometryBufferHostDevice - assigns views of shared data buffers
      to a geometry using explicit host and device memory

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcSetSharedGeometryBuffer(
      RTCGeometry geometry,
      enum RTCBufferType type,
      unsigned int slot,
      enum RTCFormat format,
      const void* ptr,
      const void* dptr,
      size_t byteOffset,
      size_t byteStride,
      size_t itemCount
    );

#### DESCRIPTION

The `rtcSetSharedGeometryBufferHostDevice` function binds views of a
shared user-managed data buffers to a geometry buffer
type and slot (`type` and `slot` argument) of the specified geometry
(`geometry` argument). The user-managed data buffers are passed using the
`ptr` argument for the host side allocated memory and the `dptr` parameter
for the memory allocated on the device.

One can specify the start of the first buffer element in bytes
(`byteOffset` argument), the byte stride between individual buffer
elements (`byteStride` argument), the format of the buffer elements
(`format` argument), and the number of elements to bind (`itemCount`).

The start address (`byteOffset` argument) and stride (`byteStride`
argument) must be both aligned to 4 bytes; otherwise the
`rtcSetSharedGeometryBufferHostDevice` function will fail.

``` {include=src/api/inc/buffer_padding.md}
```

The buffer data must remain valid for as long as the buffer may be
used, and the user is responsible for freeing the buffer data when no
longer required.

The application is responsible of keeping the host and device memory
in sync. The host memory has to be updated before calls of
`rtcCommitScene` involving the associated geometry.

If Embree has no SYCL support or the associated Embree device is no
SYCL device the `dptr` argument must be a null pointer. In this case
the function `rtcSetSharedGeometryBufferHostDevice` will behave like
`rtcSetSharedGeometryBuffer`.

Sharing buffers can significantly reduce the memory required by the
application, thus we recommend using this feature. When enabling the
`RTC_SCENE_FLAG_COMPACT` scene flag, the spatial index structures index
into the vertex buffer, resulting in even higher memory savings.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetSharedGeometryBuffer], [rtcSetGeometryBuffer], [rtcSetNewGeometryBuffer]
