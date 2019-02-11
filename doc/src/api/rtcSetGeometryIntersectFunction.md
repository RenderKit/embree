% rtcSetGeometryIntersectFunction(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryIntersectFunction - sets the callback function to
      intersect a user geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTCIntersectFunctionNArguments
    {
      int* valid;
      void* geometryUserPtr;
      unsigned int primID;
      struct RTCIntersectContext* context;
      struct RTCRayHitN* rayhit;
      unsigned int N;
    };

    typedef void (*RTCIntersectFunctionN)(
      const struct RTCIntersectFunctionNArguments* args
    );

    void rtcSetGeometryIntersectFunction(
      RTCGeometry geometry,
      RTCIntersectFunctionN intersect
    );

#### DESCRIPTION

The `rtcSetGeometryIntersectFunction` function registers a
ray/primitive intersection callback function (`intersect` argument)
for the specified user geometry (`geometry` argument).

Only a single callback function can be registered per geometry and
further invocations overwrite the previously set callback function.
Passing `NULL` as function pointer disables the registered callback
function.

The registered callback function is invoked by `rtcIntersect`-type ray
queries to calculate the intersection of a ray packet of variable
size with one user-defined primitive. The callback function of type
`RTCIntersectFunctionN` gets passed a number of arguments through the
`RTCIntersectFunctionNArguments` structure. The value `N` specifies
the ray packet size, `valid` points to an array of integers that
specify whether the corresponding ray is valid (-1) or invalid (0), the
`geometryUserPtr` member points to the geometry user data previously set
through `rtcSetGeometryUserData`, the `context` member points to the
intersection context passed to the ray query, the `rayhit` member points
to a ray and hit packet of variable size `N`, and the `primID` member
identifies the primitive ID of the primitive to intersect.

The `ray` component of the `rayhit` structure contains valid data, in
particular the `tfar` value is the current closest hit distance
found. All data inside the `hit` component of the `rayhit` structure
are undefined and should not be read by the function.

The task of the callback function is to intersect each active ray from
the ray packet with the specified user primitive. If the user-defined
primitive is missed by a ray of the ray packet, the function should
return without modifying the ray or hit. If an intersection of the
user-defined primitive with the ray was found in the valid range (from
`tnear` to `tfar`), it should update the hit distance of the ray
(`tfar` member) and the hit (`u`, `v`, `Ng`, `instID`, `geomID`,
`primID` members). In particular, the currently intersected instance is
stored in the `instID` field of the intersection context, which must be
copied into the `instID` member of the hit.

As a primitive might have multiple intersections with a ray, the
intersection filter function needs to be invoked by the user geometry
intersection callback for each encountered intersection, if filtering
of intersections is desired. This can be achieved through the
`rtcFilterIntersection` call.

Within the user geometry intersect function, it is safe to trace new
rays and create new scenes and geometries.

``` {include=src/api/inc/reorder_callback_intersect.md}
```

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryOccludedFunction], [rtcSetGeometryUserData], [rtcFilterIntersection]
