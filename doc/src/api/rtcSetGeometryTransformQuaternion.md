% rtcSetGeometryTransformQuaternion(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryTransformQuaternion - sets the transformation for a particular
      time step of an instance geometry as a decomposition of the
      transformation matrix using quaternions to represent the rotation.

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryTransformQuaternion(
      RTCGeometry geometry,
      unsigned int timeStep,
      const struct RTCQuaternionDecomposition* qd
    );

#### DESCRIPTION

The `rtcSetGeometryTransformQuaternion` function sets the
local-to-world affine transformation (`qd` parameter) of an instance
geometry (`geometry` parameter) for a particular time step (`timeStep`
parameter). The transformation is specified as a
[RTCQuaternionDecomposition], which is a decomposition of an affine
transformation that represents the rotational component of an affine
transformation as a quaternion. This allows interpolating rotational
transformations exactly using spherical linear interpolation (such as
a turning wheel).

For more information about the decomposition see [RTCQuaternionDecomposition].
The quaternion given in the `RTCQuaternionDecomposition` struct will be normalized
internally.

For correct results, the transformation matrices for all time steps must be
set either using `rtcSetGeometryTransform` or
`rtcSetGeometryTransformQuaternion`. Mixing both representations is not
allowed. Spherical linear interpolation will be used, iff the transformation
matizes are set with `rtcSetGeometryTransformQuaternion`.

For an example of this feature see the tutorial [Quaternion Motion Blur].

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcInitQuaternionDecomposition], [rtcSetGeometryTransform]
