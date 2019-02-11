% rtcSetGeometryBuffer(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryBuffer - assigns a view of a buffer to the geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryBuffer(
      RTCGeometry geometry,
      enum RTCBufferType type,
      unsigned int slot,
      enum RTCFormat format,
      RTCBuffer buffer,
      size_t byteOffset,
      size_t byteStride,
      size_t itemCount
    );

#### DESCRIPTION

The `rtcSetGeometryBuffer` function binds a view of a buffer object
(`buffer` argument) to a geometry buffer type and slot (`type` and
`slot` argument) of the specified geometry (`geometry` argument). 

One can specify the start of the first buffer element in bytes
(`byteOffset` argument), the byte stride between individual buffer
elements (`byteStride` argument), the format of the buffer elements
(`format` argument), and the number of elements to bind (`itemCount`).

The start address (`byteOffset` argument) and stride (`byteStride`
argument) must be both aligned to 4 bytes, otherwise the
`rtcSetGeometryBuffer` function will fail.

After successful completion of this function, the geometry
will hold a reference to the buffer object.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetSharedGeometryBuffer], [rtcSetNewGeometryBuffer]

