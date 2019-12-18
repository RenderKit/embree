% rtcInitQuaternionDecomposition(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcInitQuaternionDecomposition - initializes the intersection context

#### SYNOPSIS

    struct RTCQuaternionDecomposition
    {
      float scale_x, scale_y, scale_z;
      float skew_xy, skew_xz, skew_yz;
      float shift_x, shift_y, shift_z;
      float quaternion_r, quaternion_i, quaternion_j, quaternion_k;
      float translation_x, translation_y, translation_z;
    };

    void rtcInitQuaternionDecomposition(
      struct RTCQuaternionDecomposition* qd
    );

#### DESCRIPTION

The struct `RTCQuaternionDecomposition` represents an affine transformation decomposed into three parts. An upper triangular scaling/skew/shift matrix 

$$
S = \left( \begin{array}{cccc}
scale_x & skew_{xy} & skew_{xz} & shift_x \\ 
0 & scale_y & skew_{yz} & shift_y \\ 
0 & 0 & scale_z & shift_z \\ 
0 & 0 & 0 & 1 \\ 
\end{array} \right),
$$

a translation matrix

$$
T = \left( \begin{array}{cccc}
1 & 0 & 0 & translation_x \\ 
0 & 1 & 0 & translation_y \\ 
0 & 0 & 1 & translation_z \\ 
0 & 0 & 0 & 1 \\ 
\end{array} \right),
$$

and a rotation matrix $R$, represented as a quaternion

$quaternion_r + quaternion_i \ \mathbf{i} + quaternion_j \ \mathbf{i} + quaternion_k \ \mathbf{k}$

where $\mathbf{i}$, $\mathbf{j}$ $\mathbf{k}$ are the imaginary quaternion units.

\noindent
The affine transformation matrix corresponding to a `RTCQuaternionDecomposition` is $TRS$ and a point $p = (p_x, p_y, p_z, 1)^T$ will be transformed as 
$$p' = T \ R \ S \ p.$$

The `rtcInitIntersectQuaternionDecomposition` function initializes a `RTCQuaternionDecomposition` struct to represent an identity transformation.

The functions `rtcQuaternionDecompositionSetQuaternion`, `rtcQuaternionDecompositionSetScale`, `rtcQuaternionDecompositionSetSkew`, `rtcQuaternionDecompositionSetShift`, and `rtcQuaternionDecompositionSetTranslation` allow to set the fields of the struct more conveniently.

#### EXIT STATUS

No error code is set by this function.

#### SEE ALSO

[rtcSetGeometryTransformQuaternion]
