% rtcRetainBVH(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcRetainBVH - increments the BVH reference count

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcRetainBVH(RTCBVH bvh);

#### DESCRIPTION

BVH objects are reference counted. The `rtcRetainBVH` function
increments the reference count of the passed BVH object (`bvh`
argument). This function together with `rtcReleaseBVH` allows to use
the internal reference counting in a C++ wrapper class to handle the
ownership of the object.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewBVH], [rtcReleaseBVH]
