% rtcSetGeometryOccludedFunction(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryOccludedFunction - sets the callback function to
      test a user geometry for occlusion

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTCOccludedFunctionNArguments
    {
      int* valid;
      void* geometryUserPtr;
      unsigned int primID;
      struct RTCIntersectContext* context;
      struct RTCRayN* ray;
      unsigned int N;
    };
  
    typedef void (*RTCOccludedFunctionN)(
      const struct RTCOccludedFunctionNArguments* args
    );
    
    void rtcSetGeometryOccludedFunction(
      RTCGeometry geometry,
      RTCOccludedFunctionN filter
    );

#### DESCRIPTION

The `rtcSetGeometryOccludedFunction` function registers a
ray/primitive occlusion callback function (`filter` argument) for the
specified user geometry (`geometry` argument).

Only a single callback function can be registered per geometry, and
further invocations overwrite the previously set callback function.
Passing `NULL` as function pointer disables the registered callback
function.

The registered callback function is invoked by `rtcOccluded`-type ray
queries to test whether the rays of a packet of variable size are
occluded by a user-defined primitive. The callback function of type
`RTCOccludedFunctionN` gets passed a number of arguments through the
`RTCOccludedFunctionNArguments` structure. The value `N` specifies the
ray packet size, `valid` points to an array of integers which specify
whether the corresponding ray is valid (-1) or invalid (0), the
`geometryUserPtr` member points to the geometry user data previously set
through `rtcSetGeometryUserData`, the `context` member points to the
intersection context passed to the ray query, the `ray` member points
to a ray packet of variable size `N`, and the `primID` member identifies
the primitive ID of the primitive to test for occlusion.

The task of the callback function is to intersect each active ray from
the ray packet with the specified user primitive. If the user-defined
primitive is missed by a ray of the ray packet, the function should
return without modifying the ray. If an intersection of the
user-defined primitive with the ray was found in the valid range (from
`tnear` to `tfar`), it should set the `tfar` member of the ray to
`-inf`.

As a primitive might have multiple intersections with a ray, the
occlusion filter function needs to be invoked by the user geometry
occlusion callback for each encountered intersection, if filtering
of intersections is desired. This can be achieved through the
`rtcFilterOcclusion` call.

Within the user geometry occlusion function, it is safe to trace new
rays and create new scenes and geometries.

``` {include=src/api/inc/reorder_callback_occluded.md}
```

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryIntersectFunction], [rtcSetGeometryUserData], [rtcFilterOcclusion]
