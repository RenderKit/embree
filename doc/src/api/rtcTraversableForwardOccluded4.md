% rtcTraversableForwardOccluded4/8/16/Ex(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcTraversableForwardOccluded4/8/16/Ex - forwards a ray packet to new scene
      from user geometry callback

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcTraversableForwardOccluded4(
      void int* valid,
      const struct RTCOccludedFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay4* ray,
      unsigned int instID
    );

    void rtcTraversableForwardOccluded8(
      void int* valid,
      const struct RTCOccludedFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay8* ray,
      unsigned int instID
    );

    void rtcTraversableForwardOccluded16(
      void int* valid,
      const struct RTCOccludedFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay16* ray,
      unsigned int instID
    );

    void rtcTraversableForwardOccluded4Ex(
      void int* valid,
      const struct RTCOccludedFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay4* ray,
      unsigned int instID,
      unsigned int instPrimID
    );

    void rtcTraversableForwardOccluded8Ex(
      void int* valid,
      const struct RTCOccludedFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay8* ray,
      unsigned int instID,
      unsigned int instPrimID
    );

    void rtcTraversableForwardOccluded16Ex(
      void int* valid,
      const struct RTCOccludedFunctionNArguments* args,
      RTCTraversable traversable,
      struct RTCRay16* ray,
      unsigned int instID,
      unsigned int instPrimID
    );

#### DESCRIPTION

The `rtcTraversableForwardOccluded4/8/16/Ex` functions
are equivalent to `rtcForwardOccluded4/8/16/Ex`
but take a traversable object (`traversable` argument)
instead of a scene object.

For more details, refer to the documentation of `rtcForwardOccluded4/8/16/Ex`.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcForwardOccluded4/8/16/Ex], [rtcGetSceneTraversable]
