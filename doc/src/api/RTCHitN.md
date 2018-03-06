% RTCHitN(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTCHitN - hit packet of runtime size

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct HitN;

    float& RTCHitN_Ng_x(RTCHitN* hit, unsigned int N, unsigned int i);
    float& RTCHitN_Ng_y(RTCHitN* hit, unsigned int N, unsigned int i);
    float& RTCHitN_Ng_z(RTCHitN* hit, unsigned int N, unsigned int i);

    float& RTCHitN_u(RTCHitN* hit, unsigned int N, unsigned int i);
    float& RTCHitN_v(RTCHitN* hit, unsigned int N, unsigned int i);

    unsigned& RTCHitN_primID(RTCHitN* hit, unsigned int N, unsigned int i);
    unsigned& RTCHitN_geomID(RTCHitN* hit, unsigned int N, unsigned int i);
    unsigned& RTCHitN_instID(RTCHitN* hit, unsigned int N, unsigned int i, unsigned int l);

#### DESCRIPTION

When the hit packet size is not known at compile time (e.g. when
Embree returns a hit packet in the `RTCFilterFuncN` callback
function), Embree uses the `RTCHitN` type for hit packets. These hit
packets can only have sizes of 1, 4, 8, or 16. No other packet size
will be used.

You can either implement different special code paths for each of these
possible packet sizes and cast the hit to the appropriate hit packet
type, or implement one general code path that uses the `RTCHitN_XXX`
helper functions to access hit packet components.

These helper functions get a pointer to the hit packet (`hit`
argument), the packet size (`N` argument), and returns a reference
to a component (e.g. x component of `Ng`) of the the i-th hit of
the packet (`i` argument).

#### EXIT STATUS

#### SEE ALSO

[RTCRayN]
