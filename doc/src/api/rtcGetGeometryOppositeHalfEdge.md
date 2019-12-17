% rtcGetGeometryOppositeHalfEdge(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetGeometryOppositeHalfEdge - returns the opposite half edge

#### SYNOPSIS

    #include <embree3/rtcore.h>

    unsigned int rtcGetGeometryOppositeHalfEdge(
      RTCGeometry geometry,
      unsigned int topologyID,
      unsigned int edgeID
    );

#### DESCRIPTION

The `rtcGetGeometryOppositeHalfEdge` function returns the ID of the
opposite half edge of the specified half edge (`edgeID` argument) in
the specified topology (`topologyID` argument). For instance in the
following example the opposite half edge of `e6` is `e16`.

``` {image=imgHalfEdges}
```

An opposite half edge does not exist if the specified half edge has
either no neighboring face, or more than 2 neighboring faces. In these
cases the function just returns the same edge `edgeID` again.

This function can only be used for subdivision geometries. The
function depends on the topology as the topologies of a subdivision
geometry have different index buffers assigned.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcGetGeometryFirstHalfEdge], [rtcGetGeometryFace], [rtcGetGeometryOppositeHalfEdge],
[rtcGetGeometryNextHalfEdge], [rtcGetGeometryPreviousHalfEdge]
