% rtcForwardOccluded1/Ex(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcForwardOccluded1/Ex - forwards a single ray to new scene
      from user geometry callback

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcForwardOccluded1(
      const struct RTCOccludedFunctionNArguments* args,
      RTCScene scene,
      struct RTCRay* ray,
      unsigned int instID
    );

    void rtcForwardOccluded1Ex(
      const struct RTCOccludedFunctionNArguments* args,
      RTCScene scene,
      struct RTCRay* ray,
      unsigned int instID,
      unsigned int instPrimID
    );

#### DESCRIPTION

The `rtcForwardOccluded1` and `rtcForwardOccluded1Ex` functions forward the
traversal of a transformed ray (`ray` argument) into a scene (`scene` argument)
from a user geometry callback. The function can only get invoked from a user
geometry callback for a ray traversal initiated with the `rtcOccluded1`
function. The callback arguments structure of the callback invokation has to
get passed to the ray forwarding (`args` argument). The user geometry callback
should instantly terminate after invoking the `rtcForwardOccluded1/Ex` function.

Only the ray origin and ray direction members of the ray
argument are used for forwarding, all additional ray properties are
inherited from the initial ray traversal invokation of
`rtcOccluded1`.

The implementation of the `rtcForwardOccluded1` function recursively
continues the ray traversal into the specified scene and pushes the
provided instance ID (`instID` argument) to the instance ID stack. Hit
information is updated into the ray structure passed to the original
`rtcOccluded1` invokation.

This function can get used to implement user defined instancing using
user geometries, e.g. by transforming the ray in a special way, and/or
selecting between different scenes to instantiate.

For user defined instance arrays, the `rtcForwardIntersect1Ex` variant has an
additional `instPrimID` argument which is pushed to the instance primitive ID
stack. Instance primitive IDs identify which instance of an instance array was
hit.

When using Embree on the CPU it is possible to recursively invoke
`rtcOccluded1` directly from a user geometry callback. However, when
SYCL is used, recursively tracing rays is not directly supported, and
the `rtcForwardOccluded1/Ex` function must be used.

The ray structure must be aligned to 16 bytes.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcOccluded1], [RTCRay]
