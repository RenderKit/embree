% rtcSetGeometryTimeStepCount(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryTimeStepCount - sets the number of time steps of the
      geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryTimeStepCount(
      RTCGeometry geometry,
      unsigned int timeStepCount
    );

#### DESCRIPTION

The `rtcSetGeometryTimeStepCount` function sets the number of time
steps for multi-segment motion blur (`timeStepCount` parameter) of the
specified geometry (`geometry` parameter).

For triangle meshes (`RTC_GEOMETRY_TYPE_TRIANGLE`), quad meshes
(`RTC_GEOMETRY_TYPE_QUAD`), curves (`RTC_GEOMETRY_TYPE_CURVE`), points
(`RTC_GEOMETRY_TYPE_POINT`), and
subdivision geometries (`RTC_GEOMETRY_TYPE_SUBDIVISION`), the number
of time steps directly corresponds to the number of vertex buffer
slots available (`RTC_BUFFER_TYPE_VERTEX` buffer type). For these
geometries, one vertex buffer per time step must be specified when
creating multi-segment motion blur geometries.

For instance geometries (`RTC_GEOMETRY_TYPE_INSTANCE`), a
transformation must be specified for each time step (see
`rtcSetGeometryTransform`).

For user geometries, the registered bounding callback function must
provide a bounding box per primitive and time step, and the
intersection and occlusion callback functions should properly intersect
the motion-blurred geometry at the ray time.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry], [rtcSetGeometryTimeRange]
