% rtcGetGeometryFirstHalfEdge(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetGeometryFirstHalfEdge - returns the first half edge of a face

#### SYNOPSIS

    #include <embree3/rtcore.h>

    unsigned int rtcGetGeometryFirstHalfEdge(
      RTCGeometry geometry,
      unsigned int faceID
    );

#### DESCRIPTION

The `rtcGetGeometryFirstHalfEdge` function returns the ID of the first
half edge belonging to the specified face (`faceID` argument). For
instance in the following example the first half edge of face `f1` is
`e4`.

``` {image=imgHalfEdges}
```

This function can only be used for subdivision geometries. As all
topologies of a subdivision geometry share the same face buffer the
function does not depend on the topology ID.

Here f0 to f7 are 8 quadrilateral faces with 4 vertices each. The
edges e0 to e23 of these faces are shown with their orientation. For
each face the ID of the edges corresponds to the slots the face
occupies in the index array of the geometry. E.g. as the indices of
face f1 start at location 4 of the index array, the first edge is edge
e4, the next edge e5, etc.


#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcGetGeometryFirstHalfEdge], [rtcGetGeometryFace], [rtcGetGeometryOppositeHalfEdge],
[rtcGetGeometryNextHalfEdge], [rtcGetGeometryPreviousHalfEdge]
