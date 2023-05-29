% rtcSetGeometryInstancedScenes(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcSetGeometryInstancedScenes - sets an array of scenes that can be
    instanced by an instance array geometry

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcSetGeometryInstancedScenes(
      RTCGeometry geometry,
      RTCScene* scene,
      size_t numScenes
    );

#### DESCRIPTION

The `rtcSetGeometryInstancedScenes` function sets an array of type `RTCScene`
with `numScenes` elements that the specified instance geometry (`geometry`
argument) can instance. This call also requires setting an index buffer using
either `rtcSetSharedGeometryBuffer` or `rtcSetNewGeometryBuffer` (similar to
index buffers for triangle meshes), that specifies which instance of the
instance array instances which scene in the scene array. If only one scene
should be instanced the call `rtcSetGeometryInstancedScene` should be
preferred.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[RTC_GEOMETRY_TYPE_INSTANCE_ARRAY], [rtcSetNewGeometryBuffer],
[rtcSetSharedGeometryBuffer], [rtcSetGeometryInstancedScene]
