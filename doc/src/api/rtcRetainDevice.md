% rtcRetainDevice(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcRetainDevice - increments the device reference count

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcRetainDevice(RTCDevice device);

#### DESCRIPTION

Device objects are reference counted. The `rtcRetainDevice` function
increments the reference count of the passed device object (`device`
argument). This function together with `rtcReleaseDevice` allows to use
the internal reference counting in a C++ wrapper class to manage the
ownership of the object.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewDevice], [rtcReleaseDevice]
