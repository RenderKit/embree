% RTC_GEOMETRY_TYPE_INSTANCE_ARRAY(3) | Embree Ray Tracing Kernels 4

#### NAME

    RTC_GEOMETRY_TYPE_INSTANCE_ARRAY - instance array geometry type

#### SYNOPSIS

    #include <embree4/rtcore.h>

    RTCGeometry geometry =
       rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE_ARRAY);

#### DESCRIPTION

Embree supports instance arrays, which is a more memory efficient way to
represent large amounts of instances of the same or a small set of (sub)scenes.
The main difference to regular Embree instances is that Embree instance arrays
have a buffer of transformations (either affine transformations or quaternion
decompositions [RTCQuaternionDecomposition]) that can be allocated by
Embree or a shared buffer, similar to vertex buffers for triangle meshes.
Optionally, instead of instancing only one scene, an instance array can instance
multiple scenes by passing an array of scenes and a corresponding
index buffer that specifies which instance of the instance array instances
which of the scenes in the scenes array.

Instance arrays are created by passing `RTC_GEOMETRY_TYPE_INSTANCE_ARRAY` to
the `rtcNewGeometry` function call. The instanced scene can be either be set
using the `rtcSetGeometryInstancedScene` call, or if multiple scenes should be
instanced by passing an array of scenes using `rtcSetGeometryInstancedScenes`.
The latter also requires to specify an index buffer using
`rtcSetNewGeometryBuffer` or `rtcSetSharedGeometryBuffer` with
`RTC_BUFFER_TYPE_INDEX` as the buffer type.

Because the transformation information can become large for a large amount of
instances, the instance array allows to share the transformation buffer between
the user application and Embree. It can be either stored in a buffer created by
Embree with `rtcSetNewGeometryBuffer` or an already existing buffer can be
shared using `rtcSetSharedGeometryBuffer`. In either case, the buffer type has
to be `RTC_BUFFER_TYPE_TRANSFORM` and the allowed formats are
`RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR`, `RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR`,
`RTC_FORMAT_FLOAT3X4_ROW_MAJOR`, and `RTC_FORMAT_QUATERNION_DECOMPOSITION`.
Embree will not modify the data in the transformation buffer.

Embree instance arrays support both single-level instancing and multi-level instancing.
The maximum instance nesting depth is `RTC_MAX_INSTANCE_LEVEL_COUNT`; it
can be configured at compile-time using the constant `EMBREE_MAX_INSTANCE_LEVEL_COUNT`.
Users should adapt this constant to their needs: instances nested any deeper are silently
ignored in release mode, and cause assertions in debug mode.

Please note that `rtcCommitScene` on the instanced scene(s) should be
called first, followed by `rtcCommitGeometry` on the instance array,
followed by `rtcCommitScene` for the top-level scene containing the
instance array.

If a ray hits the instance, the `geomID` and `primID` members of the hit are
set to the geometry ID and primitive ID of the hit primitive in the instanced
scene. The `instID` member of the hit is set to the geometry ID of the instance
array in the top-level scene and the `instPrimID` member is set to the index of
the hit instance of the instance array.

For multi-segment motion blur, the number of time steps must be first specified
using the `rtcSetGeometryTimeStepCount` function. Then a transformation for
each time step can be specified using the `rtcSetNewGeometryBuffer` or
`rtcSetSharedGeometryBuffer` function and passing the time step as the `slot`
parameter of these calls.

See the [Instance Array Geometry] tutorial for an example of how to use instance arrays.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry], [rtcSetGeometryInstancedScene],
[rtcSetGeometryInstancedScenes], [rtcSetNewGeometryBuffer],
[rtcSetSharedGeometryBuffer], [rtcGetGeometryTransformEx]
