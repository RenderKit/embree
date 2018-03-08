% rtcGetGeometryNextHalfEdge(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetGeometryNextHalfEdge - returns the next half edge

#### SYNOPSIS

    #include <embree3/rtcore.h>

    unsigned int rtcGetGeometryNextHalfEdge(
      RTCGeometry geometry,
      unsigned int topologyID,
      unsigned int edgeID
    );

#### DESCRIPTION

The `rtcGetGeometryNextHalfEdge` function returns the ID of the
next half edge of the specified half edge (`edgeID` argument)
in the specified topology (`topologyID` argument).

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcDeviceGetError`.

#### SEE ALSO

[rtcGetGeometryFirstHalfEdge], [rtcGetGeometryFace], [rtcGetGeometryOppositeHalfEdge],
[rtcGetGeometryNextHalfEdge], [rtcGetGeometryPreviousHalfEdge]
