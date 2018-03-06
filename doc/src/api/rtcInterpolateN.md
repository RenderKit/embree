% rtcInterpolateN(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcInterpolateN - performs N interpolations of vertex attribute data

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTCInterpolateNArguments
    {
      RTCGeometry geometry;
      const void* valid;
      const unsigned int* primIDs;
      const float* u;
      const float* v;
      unsigned int N;
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

    void rtcInterpolateN(
      const struct RTCInterpolateNArguments* args
    );

#### DESCRIPTION

The `rtcInterpolateN` is similar to `rtcInterpolate`, but performs
`N` many interpolations at once. It additionally gets an array of
u/v coordinates and a valid mask (`valid` parameter) that
specifies which of these coordinates are valid. The valid mask points
to `N` integers, and a value of -1 denotes valid and 0 invalid. If
the valid pointer is `NULL` all elements are considers valid. The
destination arrays are filled in structure of array (SOA) layout. The
value `N` must be divisible by 4.

To use `rtcInterpolateN` for a geometry, all changes to that
geometry must be properly committed using `rtcCommitGeometry`.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcInterpolate]
