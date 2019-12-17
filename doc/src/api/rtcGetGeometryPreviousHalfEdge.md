% rtcGetGeometryPreviousHalfEdge(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetGeometryPreviousHalfEdge - returns the previous half edge

#### SYNOPSIS

    #include <embree3/rtcore.h>

    unsigned int rtcGetGeometryPreviousHalfEdge(
      RTCGeometry geometry,
      unsigned int edgeID
    );

#### DESCRIPTION

The `rtcGetGeometryPreviousHalfEdge` function returns the ID of the
previous half edge of the specified half edge (`edgeID` argument). For
instance in the following example the previous half edge of `e6` is
`e5`.

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
