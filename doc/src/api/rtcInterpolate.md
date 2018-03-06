% rtcInterpolate(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcInterpolate - interpolates vertex attributes

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTCInterpolateArguments
    {
      RTCGeometry geometry;
      unsigned int primID;
      float u;
      float v;
      enum RTCBufferType bufferType;
      unsigned int bufferSlot;
      float* P;
      float* dPdu;
      float* dPdv;
      float* ddPdudu;
      float* ddPdvdv;
      float* ddPdudv;
      unsigned int valueCount;
    };
  
    void rtcInterpolate(
      const struct RTCInterpolateArguments* args
    );

#### DESCRIPTION

The `rtcInterpolate` function smoothly interpolates per-vertex data
over the geometry. This interpolation is supported for triangle
meshes, quad meshes, curve geometries, and subdivision geometries.
Apart from interpolating the vertex attribute itself, it is also
possible to get the first and second order derivatives of that
value. This interpolation ignores displacements of subdivision
surfaces and always interpolates the underlying base surface. 

The `rtcInterpolate` call gets passed a number of arguments inside a
structure of type `RTCInterpolateArguments`. For some geometry
(`geometry` parameter) this function smoothly interpolates the
per-vertex data stored inside the specified geometry buffer
(`bufferType` and `bufferSlot` parameters) to the u/v location (`u`
and `v` parameters) of the primitive (`primID` parameter). The number
of floating point values to interpolate and store to the destination
arrays can be specified using the `valueCount` parameter. As
interpolation buffer, one can specify vertex buffers
(`RTC_BUFFER_TYPE_VERTEX`) and vertex attribute buffers
(`RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE`) as well.

The `rtcInterpolate` call stores `valueCount` number of interpolated
floating point values to the memory location pointed to by `P`. One
can avoid storing the interpolated value by setting `P` to `NULL`.

The first order derivative of the interpolation by u and v are stored
at the `dPdu` and `dPdv` memory locations. One can avoid storing first
order derivatives by setting both `dPdu` and `dPdv` to `NULL`.

The second order derivatives are stored at the `ddPdudu`, `ddPdvdv`,
and `ddPdudv` memory locations. One can avoid storing second order
derivatives by setting these three pointers to `NULL`.

To use `rtcInterpolate` for a geometry, all changes to that
geometry must be properly committed using `rtcCommitGeometry`.

All input buffers and output arrays must be padded to 16 bytes, as the
implementation uses 16-byte SSE instructions to read and write into
these buffers.

See tutorial [Interpolation] for an example of using the
`rtcInterpolate` function.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcInterpolateN]
