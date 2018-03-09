% rtcInitIntersectContext(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcInitIntersectContext - initializes the intersection context

#### SYNOPSIS

    #include <embree3/rtcore.h>

    enum RTCIntersectContextFlags
    {
      RTC_INTERSECT_CONTEXT_FLAG_NONE,
      RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT,
      RTC_INTERSECT_CONTEXT_FLAG_COHERENT,
    };

    struct RTCIntersectContext
    {
      enum RTCIntersectContextFlags flags;
      RTCFilterFunctionN filter;
      unsigned int instID[RTC_MAX_INSTANCE_LEVEL_COUNT];
    };

    void rtcInitIntersectContext(
      struct RTCIntersectContext* context
    );

#### DESCRIPTION

A per ray-query intersection context (`RTCIntersectContext` type) is
supported that can be used to configure intersection flags (`flags`
member), specify a filter callback function (`filter` member), specify
the ID of the current instance (`instID` member), and to attach
arbitrary data to the query (e.g. per ray data).

The `rtcInitIntersectContext` function initializes the context to
default values and should be called to initialize every intersection
context. This function gets inlined, which minimizes overhead and allows
for compiler optimizations.

The intersection context flag can be used to tune the behavior of the
traversal algorithm. Using the `RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT`
flags uses an optimized traversal algorithm for incoherent rays
(default), while `RTC_INTERSECT_CONTEXT_FLAG_COHERENT` uses an
optimized traversal algorithm for coherent rays (e.g. primary camera
rays).

Best primary ray performance can be obtained by using the ray stream
API and setting the intersect context flag to
`RTC_INTERSECT_CONTEXT_FLAG_COHERENT`. For secondary rays, it is
typically better to use the `RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT`
flag, unless the rays are known to be very coherent too (e.g. for
primary transparency rays).

A filter function can be specified inside the context. This filter
function is invoked as a second filter stage after the per-geometry
intersect or occluded filter function is invoked. Only rays that
passed the first filter stage are valid in this second filter
stage. Having such a per ray-query filter function can be useful to
implement modifications of the behavior of the query, such as
collecting all hits or accumulating transparencies. The support for
the context filter function must be enabled for a scene by using
the `RTC_SCENE_FLAG_CONTEXT_FILTER_FUNCTION` scene flag.

It is guaranteed that the pointer to the intersection context passed
to a ray query is directly passed to the registered callback
functions. This way it is possible to attach arbitrary data to the end
of the intersection context, such as a per-ray payload.

Please note that the ray pointer is not guaranteed to be passed to the
callback functions, thus reading additional data from the ray pointer
passed to callbacks is not possible.

#### EXIT STATUS

No error code is set by this function.

#### SEE ALSO

[rtcIntersect1], [rtcOccluded1]
