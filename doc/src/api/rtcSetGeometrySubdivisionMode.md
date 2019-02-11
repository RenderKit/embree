% rtcSetGeometrySubdivisionMode(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometrySubdivisionMode - sets the subdivision mode
      of a subdivision geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometrySubdivisionMode(
      RTCGeometry geometry,
      unsigned int topologyID,
      enum RTCSubdivisionMode mode
    );

#### DESCRIPTION

The `rtcSetGeometrySubdivisionMode` function sets the subdivision mode
(`mode` parameter) for the topology (`topologyID` parameter) of the
specified subdivision geometry (`geometry` parameter).

The subdivision modes can be used to force linear interpolation for
certain parts of the subdivision mesh:

+ `RTC_SUBDIVISION_MODE_NO_BOUNDARY`: Boundary patches are ignored.
  This way each rendered patch has a full set of control vertices.

+ `RTC_SUBDIVISION_MODE_SMOOTH_BOUNDARY`: The sequence of boundary
  control points are used to generate a smooth B-spline boundary
  curve (default mode).

+ `RTC_SUBDIVISION_MODE_PIN_CORNERS`: Corner vertices are pinned to
  their location during subdivision.

+ `RTC_SUBDIVISION_MODE_PIN_BOUNDARY`: All vertices at the border are
  pinned to their location during subdivision. This way the boundary
  is interpolated linearly. This mode is typically used for texturing
  to also map texels at the border of the texture to the mesh.

+ `RTC_SUBDIVISION_MODE_PIN_ALL`: All vertices at the border are
  pinned to their location during subdivision. This way all patches
  are linearly interpolated.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[RTC_GEOMETRY_TYPE_SUBDIVISION]
