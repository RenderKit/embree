% rtcSetNewGeometryBufferHostDevice(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcSetNewGeometryBufferHostDevice - creates and assigns a new host/device
      data buffer pair to the geometry

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcSetNewGeometryBufferHostDevice(
      RTCGeometry geometry,
      enum RTCBufferType type,
      unsigned int slot,
      enum RTCFormat format,
      size_t byteStride,
      size_t itemCount,
      void** ptr,
      void** dptr
    );

#### DESCRIPTION

The `rtcSetNewGeometryBufferHostDevice` function creates a new host/device data buffer pair of
specified format (`format` argument), byte stride (`byteStride`
argument), and number of items (`itemCount` argument), and assigns it
to a geometry buffer slot (`type` and `slot` argument) of the
specified geometry (`geometry` argument). The buffer data is managed
internally and automatically freed when the geometry is destroyed.

The byte stride (`byteStride` argument) must be aligned to 4 bytes;
otherwise the `rtcSetNewGeometryBufferHostDevice` function will fail.

The allocated buffer will be automatically over-allocated slightly
when used as a vertex buffer, where a requirement is that each buffer
element should be readable using 16-byte SSE load instructions.

If `ptr` is not null, it will be used to return the address of the
host data buffer. If `dptr` is not null, it will be used to return
the address of the device data buffer. Either `ptr` or `dptr` or both
can be null. In this case `rtcGetGeometryBufferData` and
`rtcGetGeometryBufferDataDevice` can be used to get the addresses of the
host and device data buffers.

The application is responsible of keeping the host and device memory
in sync. The host memory has to be updated before calls of
`rtcCommitScene` involving the associated geometry.

If Embree has no SYCL support or the associated Embree device is no
SYCL device the `dptr` argument will return the same address as `ptr` if on null.
In this case the function `rtcSetNewGeometryBufferHostDevice` will behave like
`rtcSetSharedGeometryBuffer`.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO
[rtcSetNewGeometryBuffer], [rtcSetGeometryBuffer], [rtcSetSharedGeometryBuffer],
[rtcGetGeometryBufferData], [rtcGetGeometryBufferDataDevice]
