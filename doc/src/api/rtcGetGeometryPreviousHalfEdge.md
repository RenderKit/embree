% rtcGetGeometryPreviousHalfEdge(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetGeometryPreviousHalfEdge - returns the previous half edge

#### SYNOPSIS

    #include <embree3/rtcore.h>

    unsigned int rtcGetGeometryPreviousHalfEdge(
      RTCGeometry geometry,
      unsigned int topologyID,
      unsigned int edgeID
    );

#### DESCRIPTION

The `rtcGetGeometryPreviousHalfEdge` function returns the ID of the
previous half edge of the specified half edge (`edgeID` argument)
in the specified topology (`topologyID` argument).

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcDeviceGetError`.

#### SEE ALSO

[rtcGetGeometryFirstHalfEdge], [rtcGetGeometryFace], [rtcGetGeometryOppositeHalfEdge],
[rtcGetGeometryNextHalfEdge], [rtcGetGeometryPreviousHalfEdge]
