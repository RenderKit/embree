% RTC_GEOMETRY_TYPE_INSTANCE(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTC_GEOMETRY_TYPE_INSTANCE - instance geometry type

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCGeometry geometry =
       rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE);

#### DESCRIPTION

Embree supports instancing of scenes using affine transformations
(3×3 matrix plus translation). As the instanced scene is stored only a
single time, even if instanced to multiple locations, this feature can
be used to create very complex scenes with small memory footprint. Only
single-level instancing is supported natively by Embree, however,
multi-level instancing can be manually implemented through user
geometries.

Instances are created by passing `RTC_GEOMETRY_TYPE_INSTANCE` to the
`rtcNewGeometry` function call. The instanced scene can be set using
the `rtcSetGeometryInstancedScene` call, and the affine transformation
can be set using the `rtcSetGeometryTransform` function.

Please note that `rtcCommitScene` on the instanced scene should be
called first, followed by `rtcCommitGeometry` on the instance,
followed by `rtcCommitScene` for the top-level scene containing the
instance.

If a ray hits the instance, the `geomID` and `primID` members of the
hit are set to the geometry ID and primitive ID of the hit primitive
in the instanced scene, and the `instID` member of the hit is set to
the geometry ID of the instance in the top-level scene.

The instancing scheme can also be implemented using user geometries.
To achieve this, the user geometry code should set the `instID` member
of the intersection context to the geometry ID of the instance, then
trace the transformed ray, and finally set the `instID` field of the
intersection context again to -1. The `instID` field is copied
automatically by each primitive intersector into the `instID` field of
the hit structure when the primitive is hit. See the [User Geometry]
tutorial for an example.

For multi-segment motion blur, the number of time steps must be first
specified using the `rtcSetGeometryTimeStepCount` function. Then a
transformation for each time step can be specified using the
`rtcSetGeometryTransform` function.

See tutorial [Instanced Geometry] for an example of how to use
instances.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcGetDeviceError`.

#### SEE ALSO

[rtcNewGeometry], [rtcSetGeometryInstancedScene], [rtcSetGeometryTransform]
