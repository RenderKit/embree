% rtcGetGeometryNextHalfEdge(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetGeometryNextHalfEdge - returns the next half edge

#### SYNOPSIS

    #include <embree3/rtcore.h>

    unsigned int rtcGetGeometryNextHalfEdge(
      RTCGeometry geometry,
      unsigned int edgeID
    );

#### DESCRIPTION

The `rtcGetGeometryNextHalfEdge` function returns the ID of the
next half edge of the specified half edge (`edgeID` argument). For
instance in the following example the next half edge of `e10` is
`e11`.

``` {image=imgHalfEdges}
```

This function can only be used for subdivision geometries. As all
topologies of a subdivision geometry share the same face buffer the
function does not depend on the topology ID.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcGetGeometryFirstHalfEdge], [rtcGetGeometryFace], [rtcGetGeometryOppositeHalfEdge],
[rtcGetGeometryNextHalfEdge], [rtcGetGeometryPreviousHalfEdge]
