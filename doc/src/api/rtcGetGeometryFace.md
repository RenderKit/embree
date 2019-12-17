% rtcGetGeometryFace(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetGeometryFace - returns the face of some half edge

#### SYNOPSIS

    #include <embree3/rtcore.h>

    unsigned int rtcGetGeometryFace(
      RTCGeometry geometry,
      unsigned int edgeID
    );

#### DESCRIPTION

The `rtcGetGeometryFace` function returns the ID of the face the
specified half edge (`edgeID` argument) belongs to. For instance in
the following example the face `f1` is returned for edges `e4`,
`e5`, `e6`, and `e7`.

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
