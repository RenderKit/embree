% rtcNewGeometry(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcNewGeometry - creates a new geometry object

#### SYNOPSIS

    #include <embree3/rtcore.h>

    enum RTCGeometryType
    {
     RTC_GEOMETRY_TYPE_TRIANGLE,
     RTC_GEOMETRY_TYPE_QUAD,
     RTC_GEOMETRY_TYPE_SUBDIVISION,
     RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE,
     RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE,
     RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE,
     RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE,
     RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE,
     RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE,
     RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE,
     RTC_GEOMETRY_TYPE_GRID,
     RTC_GEOMETRY_TYPE_SPHERE_POINT,
     RTC_GEOMETRY_TYPE_DISC_POINT,
     RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT,
     RTC_GEOMETRY_TYPE_USER,
     RTC_GEOMETRY_TYPE_INSTANCE
    };

    RTCGeometry rtcNewGeometry(
      RTCDevice device,
      enum RTCGeometryType type
    );

#### DESCRIPTION

Geometries are objects that represent an array of primitives of the
same type. The `rtcNewGeometry` function creates a new geometry of
specified type (`type` argument) bound to the specified device
(`device` argument) and returns a handle to this geometry. The
geometry object is reference counted with an initial reference count
of 1. The geometry handle can be released using the
`rtcReleaseGeometry` API call.

Supported geometry types are triangle meshes
(`RTC_GEOMETRY_TYPE_TRIANGLE` type), quad meshes (triangle pairs)
(`RTC_GEOMETRY_TYPE_QUAD` type), Catmull-Clark subdivision surfaces
(`RTC_GEOMETRY_TYPE_SUBDIVISION` type), curve geometries with different
bases (`RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE`,
`RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE`,
`RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE`,
`RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE`,
`RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE`,
`RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE`,
`RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE` types),
grid meshes (`RTC_GEOMETRY_TYPE_GRID`), point geometries
(`RTC_GEOMETRY_TYPE_SPHERE_POINT`, `RTC_GEOMETRY_TYPE_DISC_POINT`,
`RTC_TYPE_ORIENTED_DISC_POINT`),
user-defined geometries (`RTC_GEOMETRY_TYPE_USER`), and instances
(`RTC_GEOMETRY_TYPE_INSTANCE`).

The types `RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE` and
`RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE` will treat the curve as a
sweep surface of a varying-radius circle swept tangentially along the
curve. The types `RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE` and
`RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE` use ray-facing ribbons as a
faster-to-intersect approximation.

After construction, geometries are enabled by default and not attached
to any scene. Geometries can be disabled (`rtcDisableGeometry` call),
and enabled again (`rtcEnableGeometry` call). A geometry can be
attached to a single scene using the `rtcAttachGeometry` call (or
`rtcAttachGeometryByID` call), and detached using the
`rtcDetachGeometry` call. During attachment, a geometry ID is assigned
to the geometry (or assigned by the user when using the
`rtcAttachGeometryByID` call), which uniquely identifies the geometry
inside that scene. This identifier is returned when primitives of the
geometry are hit in later ray queries for the scene.

Geometries can also be modified, including their vertex and index
buffers. After modifying a buffer, `rtcUpdateGeometryBuffer` must be
called to notify that the buffer got modified.

The application can use the `rtcSetGeometryUserData` function to set a
user data pointer to its own geometry representation, and later read
out this pointer using the `rtcGetGeometryUserData` function.

After setting up the geometry or modifying it, `rtcCommitGeometry` must
be called to finish the geometry setup. After committing the geometry,
vertex data interpolation can be performed using the `rtcInterpolate`
and `rtcInterpolateN` functions.

A build quality can be specified for a geometry using the
`rtcSetGeometryBuildQuality` function, to balance between acceleration
structure build performance and ray query performance. The build
quality per geometry will be used if a two-level acceleration
structure is built internally, which is the case if the
`RTC_BUILD_QUALITY_LOW` is set as the scene build quality. See Section
[rtcSetSceneBuildQuality] for more details.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcEnableGeometry], [rtcDisableGeometry], [rtcAttachGeometry],
[rtcAttachGeometryByID], [rtcUpdateGeometryBuffer],
[rtcSetGeometryUserData], [rtcGetGeometryUserData],
[rtcCommitGeometry], [rtcInterpolate], [rtcInterpolateN],
[rtcSetGeometryBuildQuality], [rtcSetSceneBuildQuality],
[RTC_GEOMETRY_TYPE_TRIANGLE], [RTC_GEOMETRY_TYPE_QUAD],
[RTC_GEOMETRY_TYPE_SUBDIVISION], [RTC_GEOMETRY_TYPE_CURVE],
[RTC_GEOMETRY_TYPE_GRID], [RTC_GEOMETRY_TYPE_POINT],
[RTC_GEOMETRY_TYPE_USER], [RTC_GEOMETRY_TYPE_INSTANCE]
