% rtcReleaseBVH(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcReleaseBVH - decrements the BVH reference count

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcReleaseBVH(RTCBVH bvh);

#### DESCRIPTION

BVH objects are reference counted. The `rtcReleaseBVH` function
decrements the reference count of the passed BVH object (`bvh`
argument). When the reference count falls to 0, the BVH gets
destroyed.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewBVH], [rtcRetainBVH]
