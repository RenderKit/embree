% rtcSetGeometryTimeStepCount(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryTopologyCount - sets the number of topologies of
      a subdivision geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryTopologyCount(
      RTCGeometry geometry,
      unsigned int topologyCount
    );

#### DESCRIPTION

The `rtcSetGeometryTopologyCount` function sets the number of
topologies (`topologyCount` parameter) for the specified subdivision
geometry (`geometry` parameter). The number of topologies of a
subdivision geometry must be greater or equal to 1.

To use multiple topologies, first the number of topologies must be
specified, then the individual topologies can be configured using
`rtcSetGeometrySubdivisionMode` and by setting an index buffer
(`RTC_BUFFER_TYPE_INDEX`) using the topology ID as the buffer slot.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[RTC_GEOMETRY_TYPE_SUBDIVISION], [rtcSetGeometrySubdivisionMode]
