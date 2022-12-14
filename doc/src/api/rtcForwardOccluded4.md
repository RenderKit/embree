% rtcForwardOccluded4/8/16(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcForwardOccluded4/8/16 - forwards a ray packet to new scene
      from user geometry callback

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcForwardOccluded4(
      void int* valid,
      const struct RTCOccludedFunctionNArguments* args,
      RTCScene scene,
      struct RTCRay4* ray,
      unsigned int instID
    );

    void rtcForwardOccluded4(
      void int* valid,
      const struct RTCOccludedFunctionNArguments* args,
      RTCScene scene,
      struct RTCRay4* ray,
      unsigned int instID
    );

    void rtcForwardOccluded16(
      void int* valid,
      const struct RTCOccludedFunctionNArguments* args,
      RTCScene scene,
      struct RTCRay16* ray,
      unsigned int instID
    );

#### DESCRIPTION

The `rtcForwardOccluded4/8/16` functions forward the traversal of a
transformed ray packet (`ray` argument) into a scene (`scene` argument) from
a user geometry callback. The function can only get invoked from a
user geometry callback for a ray traversal initiated with the
`rtcOccluded4/8/16` function. The callback arguments structure of the
callback invokation has to get passed to the ray forwarding (`args`
argument). The user geometry callback should instantly terminate after
invoking the `rtcForwardOccluded4/8/16` function.

Only the ray origin and ray direction members of the ray
argument are used for forwarding, all additional ray properties are
inherited from the initial ray traversal invokation of
`rtcOccluded4/8/16`.

The implementation of the `rtcForwardOccluded4/8/16` function
recursively continues the ray traversal into the specified scene and
pushes the provided instance ID (`instID` argument) to the instance ID
stack. Hit information is updated into the ray structure passed to the
original `rtcOccluded4/8/16` invokation.

This function can get used to implement user defined instancing using
user geometries, e.g. by transforming the ray in a special way, and/or
selecting between different scenes to instantiate.

When using Embree on the CPU it is possible to recursively invoke
`rtcOccluded4/8/16` directly from a user geometry callback. However, when
SYCL is used, recursively tracing rays is not directly supported, and
the `rtcForwardOccluded4/8/16` function must be used.

For `rtcForwardOccluded4` the ray packet must be aligned to 16 bytes, for
`rtcForwardOccluded8` the alignment must be 32 bytes, and for
`rtcForwardOccluded16` the alignment must be 64 bytes.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcOccluded4/8/16]
