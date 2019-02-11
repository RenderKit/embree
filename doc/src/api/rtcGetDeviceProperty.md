% rtcGetDeviceProperty(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetDeviceProperty - queries properties of the device

#### SYNOPSIS

    #include <embree3/rtcore.h>

    ssize_t rtcGetDeviceProperty(
      RTCDevice device,
      enum RTCDeviceProperty prop
    );

#### DESCRIPTION

The `rtcGetDeviceProperty` function can be used to query properties
(`prop` argument) of a device object (`device` argument). The returned
property is an integer of type `ssize_t`.

Possible properties to query are:

+   `RTC_DEVICE_PROPERTY_VERSION`: Queries the combined version number
    (MAJOR.MINOR.PATCH) with two decimal digits per component. E.g. for
    Embree 2.8.3 the integer 208003 is returned.
    
+   `RTC_DEVICE_PROPERTY_VERSION_MAJOR`: Queries the major version
    number of Embree.
  
+   `RTC_DEVICE_PROPERTY_VERSION_MINOR`: Queries the minor version
    number of Embree.

+   `RTC_DEVICE_PROPERTY_VERSION_PATCH`: Queries the patch version
    number of Embree.

+   `RTC_DEVICE_PROPERTY_NATIVE_RAY4_SUPPORTED`: Queries whether the
    `rtcIntersect4` and `rtcOccluded4` functions preserve packet size
    and ray order when invoking callback functions. This is only the
    case if Embree is compiled with `EMBREE_RAY_PACKETS` and `SSE2`
    (or `SSE4.2`) enabled, and if the machine it is running on
    supports `SSE2` (or `SSE4.2`).

+   `RTC_DEVICE_PROPERTY_NATIVE_RAY8_SUPPORTED`: Queries whether the
    `rtcIntersect8` and `rtcOccluded8` functions preserve packet size
    and ray order when invoking callback functions. This is only the
    case if Embree is compiled with `EMBREE_RAY_PACKETS` and `AVX`
    (or `AVX2`) enabled, and if the machine it is running on supports
    `AVX` (or `AVX2`).

+   `RTC_DEVICE_PROPERTY_NATIVE_RAY16_SUPPORTED`: Queries whether the
    `rtcIntersect16` and `rtcOccluded16` functions preserve packet
    size and ray order when invoking callback functions. This is only
    the case if Embree is compiled with `EMBREE_RAY_PACKETS` and
    `AVX512SKX` (or `AVX512KNL`) enabled, and if the machine it is
    running on supports `AVX512SKX` (or `AVX512KNL`).

+   `RTC_DEVICE_PROPERTY_RAY_STREAM_SUPPORTED`: Queries whether
    `rtcIntersect1M`, `rtcIntersect1Mp`, `rtcIntersectNM`,
    `rtcIntersectNp`, `rtcOccluded1M`, `rtcOccluded1Mp`,
    `rtcOccludedNM`, and `rtcOccludedNp` are supported. This is only
    the case if Embree is compiled with `EMBREE_RAY_PACKETS` enabled.

+   `RTC_DEVICE_PROPERTY_RAY_MASK_SUPPORTED`: Queries whether ray masks
    are supported. This is only the case if Embree is compiled with
    `EMBREE_RAY_MASK` enabled.

+   `RTC_DEVICE_PROPERTY_BACKFACE_CULLING_ENABLED`: Queries whether
    back face culling is enabled. This is only the case if Embree is
    compiled with `EMBREE_BACKFACE_CULLING` enabled.

+   `RTC_DEVICE_PROPERTY_FILTER_FUNCTION_SUPPORTED`: Queries whether
    filter functions are supported, which is the case if Embree is
    compiled with `EMBREE_FILTER_FUNCTION` enabled.

+   `RTC_DEVICE_PROPERTY_IGNORE_INVALID_RAYS_ENABLED`: Queries whether
    invalid rays are ignored, which is the case if Embree is compiled
    with `EMBREE_IGNORE_INVALID_RAYS` enabled.

+   `RTC_DEVICE_PROPERTY_TRIANGLE_GEOMETRY_SUPPORTED`: Queries whether
    triangles are supported, which is the case if Embree is compiled
    with `EMBREE_GEOMETRY_TRIANGLE` enabled.

+   `RTC_DEVICE_PROPERTY_QUAD_GEOMETRY_SUPPORTED`: Queries whether
    quads are supported, which is the case if Embree is compiled
    with `EMBREE_GEOMETRY_QUAD` enabled.

+   `RTC_DEVICE_PROPERTY_SUBDIVISION_GEOMETRY_SUPPORTED`: Queries
    whether subdivision meshes are supported, which is the case if
    Embree is compiled with `EMBREE_GEOMETRY_SUBDIVISION` enabled.

+   `RTC_DEVICE_PROPERTY_CURVE_GEOMETRY_SUPPORTED`: Queries whether
    curves are supported, which is the case if Embree is compiled
    with `EMBREE_GEOMETRY_CURVE` enabled.

+   `RTC_DEVICE_PROPERTY_POINT_GEOMETRY_SUPPORTED`: Queries whether
    points are supported, which is the case if Embree is compiled
    with `EMBREE_GEOMETRY_POINT` enabled.

+   `RTC_DEVICE_PROPERTY_USER_GEOMETRY_SUPPORTED`: Queries whether user
    geometries are supported, which is the case if Embree is compiled
    with `EMBREE_GEOMETRY_USER` enabled.

+   `RTC_DEVICE_PROPERTY_TASKING_SYSTEM`: Queries the tasking system
    Embree is compiled with. Possible return values are:

    0. internal tasking system
    1. Intel Threading Building Blocks (TBB)
    2. Parallel Patterns Library (PPL)

+   `RTC_DEVICE_PROPERTY_COMMIT_JOIN_SUPPORTED`: Queries whether
    `rtcJoinCommitScene` is supported. This is not the case when Embree is
    compiled with PPL or older versions of TBB.

#### EXIT STATUS

On success returns the value of the queried property. For properties
returning a boolean value, the return value 0 denotes `false` and 1
denotes `true`.

On failure zero is returned and an error code is set that can be
queried using `rtcGetDeviceError`.
