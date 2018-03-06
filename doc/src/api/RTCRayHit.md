% RTCRay(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTCRayHit - combined single ray/hit structure

#### SYNOPSIS

    #include <embree3/rtcore_ray.h>

    struct RTCORE_ALIGN(16) RTCRayHit
    {
      struct RTCRay ray;
      struct RTCHit hit;
    };

#### DESCRIPTION

The `RTCRayHit` structure is used as input for the `rtcIntersect`-type
functions and stores the ray to intersect and some hit fields that
hold the intersection result afterwards.

The `embree3/rtcore_ray.h` header additionally defines the same
ray/hit structure in structure of array (SOA) layout for API functions
accepting ray packets of size 4 (`RTCRayHit4` type), size 8
(`RTCRayHit8` type), and size 16 (`RTCRayHit16` type). The header
additionally defines an `RTCRayHitNt` template to generate ray/hit
packets of an arbitrary compile-time size.

#### EXIT STATUS

#### SEE ALSO

[RTCRay], [RTCHit]
