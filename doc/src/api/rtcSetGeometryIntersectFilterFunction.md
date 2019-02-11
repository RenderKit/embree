% rtcSetGeometryIntersectFilterFunction(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryIntersectFilterFunction - sets the intersection filter
      for the geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTCFilterFunctionNArguments
    {
      int* valid;
      void* geometryUserPtr;
      const struct RTCIntersectContext* context;
      struct RTCRayN* ray;
      struct RTCHitN* hit;
      unsigned int N;
    };
  
    typedef void (*RTCFilterFunctionN)(
      const struct RTCFilterFunctionNArguments* args
    );

    void rtcSetGeometryIntersectFilterFunction(
      RTCGeometry geometry,
      RTCFilterFunctionN filter
    );

#### DESCRIPTION

The `rtcSetGeometryIntersectFilterFunction` function registers an
intersection filter callback function (`filter` argument) for the
specified geometry (`geometry` argument).

Only a single callback function can be registered per geometry, and
further invocations overwrite the previously set callback function.
Passing `NULL` as function pointer disables the registered callback
function.

The registered intersection filter function is invoked for every hit
encountered during the `rtcIntersect`-type ray queries and can accept
or reject that hit. The feature can be used to define a silhouette for
a primitive and reject hits that are outside the silhouette. E.g. a
tree leaf could be modeled with an alpha texture that decides whether
hit points lie inside or outside the leaf.

If the `RTC_SCENE_HIGH_QUALITY` mode is set, the filter functions may
be called multiple times for the same primitive hit. Further, rays
hitting exactly the edge might also report two hits for the same
surface. For certain use cases, the application may have to work around
this limitation by collecting already reported hits (`geomID`/`primID`
pairs) and ignoring duplicates.

The filter function callback of type `RTCFilterFunctionN` gets passed
a number of arguments through the `RTCFilterFunctionNArguments`
structure. The `valid` parameter of that structure points to an
integer valid mask (0 means invalid and -1 means valid). The
`geometryUserPtr` member is a user pointer optionally set per geometry
through the `rtcSetGeometryUserData` function. The `context` member
points to the intersection context passed to the ray query
function. The `ray` parameter points to `N` rays in SOA layout. The
`hit` parameter points to `N` hits in SOA layout to test. The `N`
parameter is the number of rays and hits in `ray` and `hit`. The hit
distance is provided as the `tfar` value of the ray. If the hit
geometry is instanced, the `instID` member of the ray is valid, and the
ray and the potential hit are in object space.

The filter callback function has the task to check for each valid ray
whether it wants to accept or reject the corresponding hit. To reject
a hit, the filter callback function just has to write `0` to the
integer valid mask of the corresponding ray. To accept the hit, it just
has to leave the valid mask set to `-1`. The filter function is further
allowed to change the hit and decrease the `tfar` value of the ray but
it should not modify other ray data nor any inactive components of the
ray or hit.

``` {include=src/api/inc/reorder_callback_intersect.md}
```

The implementation of the filter function can choose to implement a
single code path that uses the ray access helper functions
`RTCRay_XXX` and hit access helper functions `RTCHit_XXX` to access
ray and hit data. Alternatively the code can branch to optimized
implementations for specific sizes of `N` and cast the `ray` and
`hit` inputs to the proper packet types.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryOccludedFilterFunction]
