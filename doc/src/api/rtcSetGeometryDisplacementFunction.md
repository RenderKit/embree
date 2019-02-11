% rtcSetGeometryDisplacementFunction(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryDisplacementFunction - sets the displacement function
      for a subdivision geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTCDisplacementFunctionNArguments
    {
      void* geometryUserPtr;
      RTCGeometry geometry;
      unsigned int primID;
      unsigned int timeStep;
      const float* u;
      const float* v;
      const float* Ng_x;
      const float* Ng_y;
      const float* Ng_z;
      float* P_x;
      float* P_y;
      float* P_z;
      unsigned int N;
    };
 
    typedef void (*RTCDisplacementFunctionN)(
       const struct RTCDisplacementFunctionNArguments* args
    );

    void rtcSetGeometryDisplacementFunction(
      RTCGeometry geometry,
      RTCDisplacementFunctionN displacement
    );

#### DESCRIPTION

The `rtcSetGeometryDisplacementFunction` function registers a
displacement callback function (`displacement` argument) for the
specified subdivision geometry (`geometry` argument).

Only a single callback function can be registered per geometry, and
further invocations overwrite the previously set callback function.
Passing `NULL` as function pointer disables the registered callback
function.

The registered displacement callback function is invoked to displace
points on the subdivision geometry during spatial acceleration
structure construction, during the `rtcCommitScene` call.

The callback function of type `RTCDisplacementFunctionN` is invoked
with a number of arguments stored inside the
`RTCDisplacementFunctionNArguments` structure. The provided user data
pointer of the geometry (`geometryUserPtr` member) can be used to
point to the application's representation of the subdivision mesh. A
number `N` of points to displace are specified in a structure of array
layout. For each point to displace, the local patch UV coordinates (`u`
and `v` arrays), the normalized geometry normal (`Ng_x`, `Ng_y`, and
`Ng_z` arrays), and the position (`P_x`, `P_y`, and `P_z` arrays) are
provided. The task of the displacement function is to use this
information and change the position data.

The geometry handle (`geometry` member) and primitive ID (`primID`
member) of the patch to displace are additionally provided as well as
the time step `timeStep`, which can be important if the displacement is
time-dependent and motion blur is used.

All passed arrays must be aligned to 64 bytes and properly padded to
make wide vector processing inside the displacement function easily
possible.

Also see tutorial [Displacement Geometry] for an example of how to use
the displacement mapping functions.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[RTC_GEOMETRY_TYPE_SUBDIVISION]
