% rtcOccluded1(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcOccluded1 - finds any hit for a single ray

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcOccluded1(
      RTCScene scene,
      struct RTCIntersectContext* context,
      struct RTCRay* ray
    );

#### DESCRIPTION

The `rtcOccluded1` function checks for a single ray (`ray` argument)
whether there is any hit with the scene (`scene` argument).

The user must initialize the ray origin (`org` ray member), ray
direction (`dir` ray member), ray segment (`tnear`, `tfar` ray
members), and must set the ray flags to `0` (`flags` ray member). If
the scene contains motion blur geometries, also the ray time (`time`
ray member) must be initialized to a value in the range $[0, 1]$. If
ray masks are enabled at compile time, the ray mask (`mask` ray member)
must be initialized as well. The ray segment must be in the range
$[0, ∞]$, thus ranges that start behind the ray origin are not valid,
but ranges can reach to infinity. See Section [RTCRay] for the ray
layout description.

When no intersection is found, the ray data is not updated. In case a
hit was found, the `tfar` component of the ray is set to `-inf`.

The implementation makes no guarantees that primitives whose hit
distance is exactly at (or very close to) `tnear` or `tfar` are hit or
missed. If you want to exclude intersections at `tnear` just pass a
slightly enlarged `tnear`, and if you want to include intersections at
`tfar` pass a slightly enlarged `tfar`.

``` {include=src/api/inc/context.md}
```

``` {include=src/api/inc/raypointer.md}
```

The ray must be aligned to 16 bytes.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcOccluded1], [RTCRay]
