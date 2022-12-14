% rtcOccluded1(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcOccluded1 - finds any hit for a single ray

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcOccluded1(
      RTCScene scene,
      struct RTCRay* ray,
      struct RTCOccludedArguments* args = NULL
    );

#### DESCRIPTION

The `rtcOccluded1` function checks for a single ray (`ray` argument)
whether there is any hit with the scene (`scene` argument). The passed
optional arguments struct (`args` argument) can get used for advanced
use cases, see section [rtcInitOccludedArguments] for more details.

To trace a ray, the user must initialize the ray origin (`org` ray
member), ray direction (`dir` ray member), ray segment (`tnear`,
`tfar` ray members), ray mask (`mask` ray member), and must set the
ray flags to `0` (`flags` ray member). The ray time (`time` ray
member) must be initialized to a value in the range $[0, 1]$. The ray
segment must be in the range $[0, \infty]$, thus ranges that start
behind the ray origin are not valid, but ranges can reach to
infinity. See Section [RTCRay] for the ray layout description.

When no intersection is found, the ray data is not updated. In case a
hit was found, the `tfar` component of the ray is set to `-inf`.

The implementation makes no guarantees that primitives whose hit
distance is exactly at (or very close to) `tnear` or `tfar` are hit or
missed. If you want to exclude intersections at `tnear` just pass a
slightly enlarged `tnear`, and if you want to include intersections at
`tfar` pass a slightly enlarged `tfar`.

``` {include=src/api/inc/raypointer.md}
```

The ray must be aligned to 16 bytes.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcIntersect1], [rtcOccluded4/8/16], [RTCRay], [rtcInitOccludedArguments]

