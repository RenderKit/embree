% rtcSetGeometryTimeStepCount(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryVertexAttributeCount - sets the number of vertex
      attributes of the geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryVertexAttributeCount(
      RTCGeometry geometry,
      unsigned int vertexAttributeCount
    );

#### DESCRIPTION

The `rtcSetGeometryVertexAttributeCount` function sets the number of
slots (`vertexAttributeCount` parameter) for vertex attribute buffers
(`RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE`) that can be used for the
specified geometry (`geometry` parameter).

This function is supported only for triangle meshes
(`RTC_GEOMETRY_TYPE_TRIANGLE`), quad meshes
(`RTC_GEOMETRY_TYPE_QUAD`), curves (`RTC_GEOMETRY_TYPE_CURVE`),
points (`RTC_GEOMETRY_TYPE_POINT`), and
subdivision geometries (`RTC_GEOMETRY_TYPE_SUBDIVISION`).

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry], [RTCBufferType]
