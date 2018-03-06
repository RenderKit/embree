% RTCRayN(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTCRayN - ray packet of runtime size

#### SYNOPSIS

    #include <embree3/rtcore_ray.h>

    struct RTCRayN;

    float& RTCRayN_org_x(RTCRayN* ray, unsigned int N, unsigned int i);
    float& RTCRayN_org_y(RTCRayN* ray, unsigned int N, unsigned int i);
    float& RTCRayN_org_z(RTCRayN* ray, unsigned int N, unsigned int i);
    float& RTCRayN_tnear(RTCRayN* ray, unsigned int N, unsigned int i);

    float& RTCRayN_dir_x(RTCRayN* ray, unsigned int N, unsigned int i);
    float& RTCRayN_dir_y(RTCRayN* ray, unsigned int N, unsigned int i);
    float& RTCRayN_dir_z(RTCRayN* ray, unsigned int N, unsigned int i);
    float& RTCRayN_time (RTCRayN* ray, unsigned int N, unsigned int i);
    
    float&        RTCRayN_tfar (RTCRayN* ray, unsigned int N, unsigned int i);
    unsigned int& RTCRayN_mask (RTCRayN* ray, unsigned int N, unsigned int i);
    unsigned int& RTCRayN_id   (RTCRayN* ray, unsigned int N, unsigned int i);
    unsigned int& RTCRayN_flags(RTCRayN* ray, unsigned int N, unsigned int i);

#### DESCRIPTION

When the ray packet size is not known at compile time (e.g. when
Embree returns a ray packet in the `RTCFilterFuncN` callback
function), Embree uses the `RTCRayN` type for ray packets. These ray
packets can only have sizes of 1, 4, 8, or 16. No other packet size
will be used.

You can either implement different special code paths for each of
these possible packet sizes and cast the ray to the appropriate ray
packet type, or implement one general code path that uses the
`RTCRayN_XXX` helper functions to access the ray packet components.

These helper functions get a pointer to the ray packet (`ray`
argument), the packet size (`N` argument), and returns a reference to
a component (e.g. x-component of origin) of the the i-th ray of the
packet (`i` argument).

#### EXIT STATUS

#### SEE ALSO

[RTCHitN]
