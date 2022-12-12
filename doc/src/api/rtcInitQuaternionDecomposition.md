% rtcInitQuaternionDecomposition(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcInitQuaternionDecomposition - initializes quaternion decomposition

#### SYNOPSIS

    void rtcInitQuaternionDecomposition(
      struct RTCQuaternionDecomposition* qd
    );

#### DESCRIPTION

The `rtcInitQuaternionDecomposition` function initializes a
`RTCQuaternionDecomposition` structure to represent an identity
transformation.

#### EXIT STATUS

No error code is set by this function.

#### SEE ALSO

[rtcSetGeometryTransformQuaternion], [RTCQuaternionDecomposition]
