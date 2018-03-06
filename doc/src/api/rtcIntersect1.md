% rtcIntersect1(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcIntersect1 - finds the closest hit for a single ray

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcIntersect1(
      RTCScene scene,
      struct RTCIntersectContext* context,
      struct RTCRayHit* rayhit
    );

#### DESCRIPTION

The `rtcIntersect1` function finds the closest hit of a single ray
with the scene (`scene` argument). The provided ray/hit structure
(`rayhit` argument) contains the ray to intersect and some hit output
fields that are filled when a hit is found.

The user has to initialize the ray origin (`org` ray member), ray
direction (`dir` ray member), ray segment (`tnear`, `tfar` ray
members), and set the ray flags to `0` (`flags` ray member). If the
scene contains motion blur geometries, also the ray time (`time` ray
member) must be initialized to a value in the range $[0, 1]$. If
ray masks are enabled at compile time, the ray mask (`mask` ray
member) must be initialized as well. The ray segment has to be in the
range $[0, ∞]$, thus ranges that start behind the ray origin are not
valid, but ranges can reach to infinity. See Section [RTCRay] for the
ray layout description.

The instance ID (`instID` hit member) and geometry ID (`geomID` hit
member) of the hit data must be initialized to
`RTC_INVALID_GEOMETRY_ID` (-1).

Further, an intersection context for the ray query function must be
created and initialized (see `rtcInitIntersectContext`).

When no intersection is found, the ray/hit data is not updated. When an
intersection is found, the hit distance is written into the `tfar`
member of the ray and all hit data is set, such as unnormalized
geometry normal in object space (`Ng` hit member), local hit
coordinates (`u`, `v` hit member), instance ID (`instID` hit member),
geometry ID (`geomID` hit member), and primitive ID (`primID` hit
member). See Section [RTCHit] for the hit layout description.

If the instance ID was set (thus it is not equal to
`RTC_INVALID_GEOMETRY_ID`), the instance ID corresponds to the geometry
ID of the hit instance of the top-level scene, the geometry ID
corresponds to the hit geometry inside the hit instanced scene, and the
primitive ID corresponds to the n-th primitive of that geometry.

If the instance ID was not set (thus it is still equal to
`RTC_INVALID_GEOMETRY_ID`), the geometry ID corresponds to the hit
geometry inside the scene, and the primitive ID corresponds to the
n-th primitive of that geometry.

The implementation makes no guarantees that primitives whose hit
distance is exactly at (or very close to) `tnear` or `tfar` are hit or
missed. If you want to exclude intersections at `tnear` just pass a
slightly enlarged `tnear`, and if you want to include intersections at
`tfar` pass a slightly enlarged `tfar`.

``` {include=src/api/inc/context.md}
```

``` {include=src/api/inc/raypointer.md}
```

The ray/hit structure must be aligned to 16 bytes.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcOccluded1], [RTCRayHit], [RTCRay], [RTCHit]
