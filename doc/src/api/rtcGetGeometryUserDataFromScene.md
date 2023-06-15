% rtcGetGeometryUserDataFromScene(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetGeometryUserDataFromScene - returns the user data pointer
      of the geometry through the scene object

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void* rtcGetGeometryUserDataFromScene(RTCScene scene, unsigned int geomID);

#### DESCRIPTION

The `rtcGetGeometryUserDataFromScene` function queries the user data
pointer previously set with `rtcSetGeometryUserData` from the geometry
with index `geomID` from the specified scene `scene`. When
`rtcSetGeometryUserData` was not called yet, `NULL` is returned.

In contrast to the `rtcGetGeometryUserData` function, the
`rtcGetGeometryUserDataFromScene` function an get used during
rendering inside a SYCL kernel.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryUserData], [rtcGetGeometryUserData]
