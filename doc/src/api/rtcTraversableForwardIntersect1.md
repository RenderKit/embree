% rtcTraversableForwardIntersect1/Ex(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcTraversableForwardIntersect1/Ex - forwards a single ray to new scene
      from user geometry callback

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcTraversableForwardIntersect1(
      const struct RTCIntersectFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay* ray,
      unsigned int instID
    );

    void rtcTraversableForwardIntersect1Ex(
      const struct RTCIntersectFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay* ray,
      unsigned int instID,
      unsigned int instPrimID,
    );

#### DESCRIPTION

The `rtcTraversalbeForwardIntersect1` and `rtcTraversableForwardIntersect1Ex` functions are
equivalent to `rtcForwardIntersect1` and `rtcForwardIntersect1Ex` respectively
but take a traversable object (`traversable` argument) instead of a scene
object.

For more details, refer to the documentation of `rtcForwardIntersect1/Ex`.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcForwardIntersect1], [rtcGetSceneTraversable]
