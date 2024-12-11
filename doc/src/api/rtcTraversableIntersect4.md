% rtcTraversableIntersect4/8/16(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcTravesableIntersect4/8/16 - finds the closest hits for a ray packet

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcTraversableIntersect4(
      const int* valid,
      RTCTraversable traversable,
      struct RTCRayHit4* rayhit,
      struct RTCIntersectArguments* args = NULL
    );

    void rtcTraversableIntersect8(
      const int* valid,
      RTCTraversable traversable,
      struct RTCRayHit8* rayhit,
      struct RTCIntersectArguments* args = NULL
    );

    void rtcTraversableIntersect16(
      const int* valid,
      RTCTraversable traversable,
      struct RTCRayHit16* rayhit,
      struct RTCIntersectArguments* args = NULL
    );

#### DESCRIPTION


The `rtcTraversableIntersect4/8/16` functions are equivalent to `rtcIntersect4/8/16`
but take a traversable object (`traversable` argument) instead of a scene
object.

For more details, refer to the documentation of `rtcIntersect4/8/16`.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcIntersect4/8/16], [rtcGetSceneTraversable]
