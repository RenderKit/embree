% rtcOccludedNp(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcOccludedNp - finds any hits for a SOA ray stream of size N

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcOccludedNp(
      RTCScene scene,
      struct RTCIntersectContext* context,
      struct RTCRayNp* ray,
      unsigned int N
    );

#### DESCRIPTION

The `rtcOccludedNp` function checks whether there are any hits for a
SOA ray stream (`ray` argument) of size `N` (basically a large ray
packet) with the scene (`scene` argument). The `ray` argument points
to a structure of pointers with one pointer for each ray component.
Each of these pointers points to an array with the ray component data
for each ray. This way the individual components of the SOA ray stream
do not need to be stored sequentially in memory, which makes it
possible to have large varying size ray packets in SOA layout. See
Section [rtcOccluded1] for a description of how to set up and trace
occlusion rays.

``` {include=src/api/inc/context.md}
```

``` {include=src/api/inc/reorder.md}
```

A ray in a ray stream is considered inactive if its `tnear` value is
larger than its `tfar` value.

The stream size `N` can be an arbitrary positive integer including 0.
Each ray component array must be aligned to 16 bytes.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcIntersectNp]
