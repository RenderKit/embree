% rtcInitOccludedArguments(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcInitOccludedArguments - initializes the occluded arguments struct

#### SYNOPSIS

    #include <embree4/rtcore.h>

    enum RTCRayQueryFlags
    {
      RTC_RAY_QUERY_FLAG_NONE,
      RTC_RAY_QUERY_FLAG_INCOHERENT,
      RTC_RAY_QUERY_FLAG_COHERENT,
      RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER
    };

    struct RTCOccludedArguments
    {
      enum RTCRayQueryFlags flags;
      enum RTCFeatureFlags feature_mask;
      struct RTCRayQueryContext* context;
      RTCFilterFunctionN filter;
      RTCOccludedFunctionN intersect;
    #if RTC_MIN_WIDTH
      float minWidthDistanceFactor;
    #endif
    };

    void rtcInitOccludedArguments(
      struct RTCOccludedArguments* args
    );

#### DESCRIPTION

The `rtcInitOccludedArguments` function initializes the optional
argument struct that can get passed to the `rtcOccluded1/4/8/16`
functions to default values. The arguments struct needs to get used
for more advanced Embree features as described here.

The `flags` member can get used to enable special traversal
mode. Using the `RTC_RAY_QUERY_FLAG_INCOHERENT` flag uses an
optimized traversal algorithm for incoherent rays (default), while
`RTC_RAY_QUERY_FLAG_COHERENT` uses an optimized traversal
algorithm for coherent rays (e.g. primary camera rays).

The `feature_mask` member should get used in SYCL to just enable ray
tracing features required to render a given scene. Please see section
[RTCFeatureFlags] for a more detailed description.

The `context` member can get used to pass an optional intersection
context. It is guaranteed that the pointer to the context passed to a
ray query is directly passed to all callback functions. This way it is
possible to attach arbitrary data to the end of the context, such as a
per-ray payload. Please note that the ray pointer is not guaranteed to
be passed to the callback functions, thus reading additional data from
the ray pointer passed to callbacks is not possible. See section
[rtcInitRayQueryContext] for more details.

The `filter` member specifies a filter function to invoked for each
encountered hit. The support for the argument filter function must be
enabled for a scene by using the
`RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS` scene flag. In case of
instancing this feature has to get enabled also for each instantiated
scene.

The argument filter function is invoked for each geometry for which it
got explicitely enabled using the
`rtcSetGeometryEnableFilterFunctionFromArguments` function. The
invokation of the argument filter function can also get enfored for
each geometry using the
`RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER` ray query flag. This
argument filter function is invoked as a second filter stage after the
per-geometry filter function is invoked. Only rays that passed the
first filter stage are valid in this second filter stage. Having such
a per ray-query filter function can be useful to implement
modifications of the behavior of the query, such as collecting all
hits or accumulating transparencies.

The `intersect` member specifies the user geometry callback to get
invoked for each user geometry encountered during traversal. The user
geometry callback specified this way has preference over the one
specified inside the geometry.

The `minWidthDistanceFactor` value controls the target size of the
curve radii when the min-width feature is enabled. Please see the
[rtcSetGeometryMaxRadiusScale] function for more details on the
min-width feature.


#### EXIT STATUS

No error code is set by this function.

#### SEE ALSO

[rtcOccluded1], [rtcOccluded4/8/16],
[RTCFeatureFlags], [rtcInitRayQueryContext], [RTC_GEOMETRY_TYPE_USER], [rtcSetGeometryMaxRadiusScale]
