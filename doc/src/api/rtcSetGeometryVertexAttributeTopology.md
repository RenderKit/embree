% rtcSetGeometryVertexAttributeTopology(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryVertexAttributeTopology - binds a vertex
      attribute to a topology of the geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryVertexAttributeTopology(
      RTCGeometry geometry,
      unsigned int vertexAttributeID,
      unsigned int topologyID
    );

#### DESCRIPTION

The `rtcSetGeometryVertexAttributeTopology` function binds a vertex
attribute buffer slot (`vertexAttributeID` argument) to a topology
(`topologyID` argument) for the specified subdivision geometry
(`geometry` argument). Standard vertex buffers are always bound to the
default topology (topology 0) and cannot be bound differently. A
vertex attribute buffer always uses the topology it is bound to when
used in the `rtcInterpolate` and `rtcInterpolateN` calls.

A topology with ID `i` consists of a subdivision mode set through
`rtcSetGeometrySubdivisionMode` and the index buffer bound to the
index buffer slot `i`. This index buffer can assign indices for each
face of the subdivision geometry that are different to the indices of
the default topology. These new indices can for example be used to
introduce additional borders into the subdivision mesh to map multiple
textures onto one subdivision geometry.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometrySubdivisionMode], [rtcInterpolate], [rtcInterpolateN]
