% RTCRay(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTCRay - single ray structure

#### SYNOPSIS

    #include <embree3/rtcore_ray.h>

    struct RTC_ALIGN(16) RTCRay
    {
      float org_x;        // x coordinate of ray origin
      float org_y;        // y coordinate of ray origin
      float org_z;        // z coordinate of ray origin
      float tnear;        // start of ray segment

      float dir_x;        // x coordinate of ray direction
      float dir_y;        // y coordinate of ray direction
      float dir_z;        // z coordinate of ray direction
      float time;         // time of this ray for motion blur

      float tfar;         // end of ray segment (set to hit distance)
      unsigned int mask;  // ray mask
      unsigned int id;    // ray ID
      unsigned int flags; // ray flags
    };

#### DESCRIPTION

The `RTCRay` structure defines the ray layout for a single ray. The
ray contains the origin (`org_x`, `org_y`, `org_z` members), direction
vector (`dir_x`, `dir_y`, `dir_z` members), and ray segment (`tnear`
and `tfar` members). The ray direction does not have to be normalized,
and only the parameter range specified by the `tnear`/`tfar` interval
is considered valid.

The ray segment must be in the range $[0, ∞]$, thus ranges that
start behind the ray origin are not allowed, but ranges can reach to
infinity. For rays inside a ray stream, `tfar` < `tnear` identifies
an inactive ray.

The ray further contains a motion blur time in the range $[0, 1]$
(`time` member), a ray mask (`mask` member), a ray ID (`id` member),
and ray flags (`flags` member). The ray mask can be used to mask out
some geometries for some rays (see `rtcSetGeometryMask` for more
details). The ray ID can be used to identify a ray inside a callback
function, even if the order of rays inside a ray packet or stream has
changed. The ray flags are reserved.

The `embree3/rtcore_ray.h` header additionally defines the same ray
structure in structure of array (SOA) layout for API functions
accepting ray packets of size 4 (`RTCRay4` type), size 8 (`RTCRay8`
type), and size 16 (`RTCRay16` type). The header additionally defines
an `RTCRayNt` template for ray packets of an arbitrary compile-time
size.

#### EXIT STATUS

#### SEE ALSO

[RTCHit]
