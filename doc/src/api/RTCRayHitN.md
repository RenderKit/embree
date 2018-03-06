% RTCRayHitN(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTCRayHitN - combined ray/hit packet of runtime size

#### SYNOPSIS

    #include <embree3/rtcore_ray.h>

    struct RTCRayHitN;

    struct RTCRayN* RTCRayHitN_RayN(struct RTCRayHitN* rayhit, unsigned int N);
    struct RTCHitN* RTCRayHitN_HitN(struct RTCRayHitN* rayhit, unsigned int N);

#### DESCRIPTION

When the packet size of a ray/hit structure is not known at compile
time (e.g. when Embree returns a ray/hit packet in the
`RTCIntersectFunctionN` callback function), Embree uses the
`RTCRayHitN` type for ray packets. These ray/hit packets can only have
sizes of 1, 4, 8, or 16. No other packet size will be used.

You can either implement different special code paths for each of
these possible packet sizes and cast the ray/hit to the appropriate
ray/hit packet type, or extract the `RTCRayN` and `RTCHitN` components
using the `rtcGetRayN` and `rtcGetHitN` helper functions and use the
`RTCRayN_XXX` and `RTCHitN_XXX` functions to access the ray and hit
parts of the structure.

#### EXIT STATUS

#### SEE ALSO

[RTCHitN]
