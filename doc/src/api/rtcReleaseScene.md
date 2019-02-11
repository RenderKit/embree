% rtcReleaseScene(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcReleaseScene - decrements the scene reference count

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcReleaseScene(RTCScene scene);

#### DESCRIPTION

Scene objects are reference counted. The `rtcReleaseScene` function
decrements the reference count of the passed scene object (`scene`
argument). When the reference count falls to 0, the scene gets
destroyed.

The scene holds a reference to all attached geometries, thus if the
scene gets destroyed, all geometries get detached and their reference
count decremented.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewScene], [rtcRetainScene]
