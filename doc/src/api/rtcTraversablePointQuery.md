% rtcTraversablePointQuery(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcTraversablePointQuery - traverses the BVH with a point query object

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcTraversablePointQuery(
      RTCTraversable traversable,
      struct RTCPointQuery* query,
      struct RTCPointQueryContext* context,
      struct RTCPointQueryFunction* queryFunc,
      void* userPtr
    );

#### DESCRIPTION

The `rtcTraversablePointQuery` function is equivalent
to `rtcPointQuery` but takes a traversable object
(`traversable` argument) instead of a scene object.

For more details, refer to the documentation of `rtcPointQuery`.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcPointQuery], [rtcGetSceneTraversable]
