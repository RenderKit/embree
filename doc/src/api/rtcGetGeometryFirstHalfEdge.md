% rtcGetGeometryFirstHalfEdge(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetGeometryFirstHalfEdge - returns the first half edge of a face

#### SYNOPSIS

    #include <embree3/rtcore.h>

    unsigned int rtcGetGeometryFirstHalfEdge(RTCGeometry geometry, unsigned int faceID);

#### DESCRIPTION

The `rtcGetGeometryFirstHalfEdge` function returns the ID of the first
half edge belonging to the specified face (`faceID` argument).

This function can only be used for subdivision geometries. As all
topologies of a subdivision geometry share the same face buffer the
function does not depend on the topology ID.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcDeviceGetError`.

#### SEE ALSO

[rtcGetGeometryFirstHalfEdge], [rtcGetGeometryFace], [rtcGetGeometryOppositeHalfEdge],
[rtcGetGeometryNextHalfEdge], [rtcGetGeometryPreviousHalfEdge]
