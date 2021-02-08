% rtcPointQuery(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcPointQuery - traverses the BVH with a point query object

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTC_ALIGN(16) RTCPointQuery
    {
      // location of the query
      float x;
      float y;
      float z;

      // radius and time of the query
      float radius;
      float time;
    };

    void rtcPointQuery(
      RTCScene scene,
      struct RTCPointQuery* query,
      struct RTCPointQueryContext* context,
      struct RTCPointQueryFunction* queryFunc,
      void* userPtr
    );

#### DESCRIPTION

The `rtcPointQuery` function traverses the BVH using a `RTCPointQuery` object
(`query` argument) and calls a user defined callback function (e.g
`queryFunc` argument) for each primitive of the scene (`scene` argument) that
intersects the query domain.

The user has to initialize the query location (`x`, `y` and `z` member) and
query radius in the range $[0, \infty]$. If the scene contains motion blur
geometries, also the query time (`time` member) must be initialized to a
value in the range $[0, 1]$.

Further, a `RTCPointQueryContext` (`context` argument) must be
created and initialized. It contains ID and transformation information of the
instancing hierarchy if (multilevel-)instancing is used. See
[rtcInitPointQueryContext] for further information.

For every primitive that intersects the query domain, the callback function
(`queryFunc` argument) is called, in which distance computations to the
primitive can be implemented. The user will be provided with the primID and
geomID of the according primitive, however, the geometry information (e.g.
triangle index and vertex data) has to be determined manually. The `userPtr`
argument can be used to input geometry data of the scene or output results of
the point query (e.g. closest point currently found on surface geometry (see
tutorial [ClosestPoint])).

The parameter `queryFunc` is optional and can be NULL, in which case
the callback function is not invoked. However, a callback function
can still get attached to a specific `RTCGeometry`
object using [rtcSetGeometryPointQueryFunction]. If a callback function is
attached to a geometry and (a potentially different) callback function is passed
as an argument to `rtcPointQuery`, both functions are called for the
primitives of the according geometries.

The query radius can be decreased inside the callback function, which allows
to efficiently cull parts of the scene during BVH traversal. Increasing the
query radius and modifying time or location of the query will result in
undefined behaviour.

The callback function will be called for all primitives in a leaf node of the
BVH even if the primitive is outside the query domain, since Embree does not
gather geometry information of primitives internally.

Point queries can be used with (multilevel)-instancing. However, care has to
be taken when the instance transformation contains anisotropic scaling or
sheering. In these cases distance computations have to be performed in world
space to ensure correctness and the ellipsoidal query domain (in instance
space) will be approximated with its axis aligned bounding box interally.
Therefore, the callback function might be invoked even for primitives in
inner BVH nodes that do not intersect the query domain. See
[rtcSetGeometryPointQueryFunction] for details.

The point query structure must be aligned to 16 bytes.

#### SUPPORTED PRIMITIVES

Currenly, all primitive types are supported by the point query API except of
points (see [RTC_GEOMETRY_TYPE_POINT]), curves (see
[RTC_GEOMETRY_TYPE_CURVE]) and sudivision surfaces (see
[RTC_GEOMETRY_SUBDIVISION]).

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcSetGeometryPointQueryFunction], [rtcInitPointQueryContext]
