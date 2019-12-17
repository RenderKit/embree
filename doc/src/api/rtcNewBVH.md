% rtcNewBVH(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcNewBVH - creates a new BVH object

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCBVH rtcNewBVH(RTCDevice device);

#### DESCRIPTION

This function creates a new BVH object and returns a handle to this
BVH. The BVH object is reference counted with an initial
reference count of 1. The handle can be released using the
`rtcReleaseBVH` API call.

The BVH object can be used to build a BVH in a user-specified format
over user-specified primitives. See the documentation of the
`rtcBuildBVH` call for more details.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcRetainBVH], [rtcReleaseBVH], [rtcBuildBVH]
