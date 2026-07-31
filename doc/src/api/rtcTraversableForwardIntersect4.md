% rtcTraversableForwardIntersect4/8/16/Ex(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcTraversableForwardIntersect4/8/16/Ex - forwards a ray packet to new scene
      from user geometry callback

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcTraversableForwardIntersect4(
      void int* valid,
      const struct RTCIntersectFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay4* ray,
      unsigned int instID
    );

    void rtcTraversableForwardIntersect8(
      void int* valid,
      const struct RTCIntersectFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay8* ray,
      unsigned int instID
    );

    void rtcTraversableForwardIntersect16(
      void int* valid,
      const struct RTCIntersectFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay16* ray,
      unsigned int instID,
      unsigned int instPrimID
    );

    void rtcTraversableForwardIntersect4Ex(
      void int* valid,
      const struct RTCIntersectFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay4* ray,
      unsigned int instID,
      unsigned int instPrimID
    );

    void rtcTraversableForwardIntersect8Ex(
      void int* valid,
      const struct RTCIntersectFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay8* ray,
      unsigned int instID,
      unsigned int instPrimID
    );

    void rtcTraversableForwardIntersect16Ex(
      void int* valid,
      const struct RTCIntersectFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay16* ray,
      unsigned int instID,
      unsigned int instPrimID
    );

#### DESCRIPTION

The `rtcTraversableForwardIntersect4/8/16/Ex` functions
are equivalent to `rtcForwardIntersect4/8/16/Ex`
but take a traversable object (`traversable` argument)
instead of a scene object.

For more details, refer to the documentation of `rtcForwardIntersect4/8/16/Ex`.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcForwardIntersect4/8/16/Ex], [rtcGetSceneTraversable]

