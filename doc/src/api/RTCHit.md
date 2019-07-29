% RTCHit(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTCHit - single hit structure

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTCHit
    {
      float Ng_x;          // x coordinate of geometry normal
      float Ng_y;          // y coordinate of geometry normal
      float Ng_z;          // z coordinate of geometry normal

      float u;             // barycentric u coordinate of hit
      float v;             // barycentric v coordinate of hit

      unsigned int primID; // primitive ID
      unsigned int geomID; // geometry ID
      unsigned int instID; // instance ID
    };

#### DESCRIPTION

The `RTCHit` type defines the type of a ray/primitive intersection
result. The hit contains the unnormalized geometric normal in object
space at the hit location (`Ng_x`, `Ng_y`, `Ng_z` members), the
barycentric u/v coordinates of the hit (`u` and `v` members), as well
as the primitive ID (`primID` member), geometry ID (`geomID` member),
and instance ID (`instID` member) of the hit. The latter is only useful
in single-level instancing. For multi-level instancing, the user must
intercept the instance ID stack in an intersection filter. The parametric
intersection distance is not stored inside the hit, but stored inside
the `tfar` member of the ray.

The `embree3/rtcore_ray.h` header additionally defines the same hit
structure in structure of array (SOA) layout for hit packets of size 4
(`RTCHit4` type), size 8 (`RTCHit8` type), and size 16 (`RTCHit16`
type). The header additionally defines an `RTCHitNt` template for
hit packets of an arbitrary compile-time size.

#### EXIT STATUS

#### SEE ALSO

[RTCRay], [Multi-Level Instancing]
