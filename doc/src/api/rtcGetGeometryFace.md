% rtcGetGeometryFace(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetGeometryFace - returns the face of some half edge

#### SYNOPSIS

    #include <embree3/rtcore.h>

    unsigned int rtcGetGeometryFace(RTCGeometry geometry, unsigned int edgeID);

#### DESCRIPTION

The `rtcGetGeometryFace` function returns the ID of the face the
specified half edge (`edgeID` argument) belongs to.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcDeviceGetError`.

#### SEE ALSO

[rtcGetGeometryFirstHalfEdge], [rtcGetGeometryFace], [rtcGetGeometryOppositeHalfEdge],
[rtcGetGeometryNextHalfEdge], [rtcGetGeometryPreviousHalfEdge]
