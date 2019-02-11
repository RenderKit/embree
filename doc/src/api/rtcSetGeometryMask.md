% rtcSetGeometryMask(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryMask - sets the geometry mask

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryMask(
      RTCGeometry geometry,
      unsigned int mask
    );

#### DESCRIPTION

The `rtcSetGeometryMask` function sets a 32-bit geometry mask (`mask`
argument) for the specified geometry (`geometry` argument).

This geometry mask is used together with the ray mask stored inside
the `mask` field of the ray. The primitives of the geometry are hit by
the ray only if the bitwise `and` operation of the geometry mask with
the ray mask is not 0. This feature can be used to disable selected
geometries for specifically tagged rays, e.g. to disable shadow casting
for certain geometries.

Ray masks are disabled in Embree by default at compile time, and can
be enabled through the `EMBREE_RAY_MASK` parameter in CMake. One can
query whether ray masks are enabled by querying the
`RTC_DEVICE_PROPERTY_RAY_MASK_SUPPORTED` device property using
`rtcGetDeviceProperty`.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[RTCRay], [rtcGetDeviceProperty]

