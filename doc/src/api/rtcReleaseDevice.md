% rtcReleaseDevice(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcReleaseDevice - decrements the device reference count

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcReleaseDevice(RTCDevice device);

#### DESCRIPTION

Device objects are reference counted. The `rtcReleaseDevice` function
decrements the reference count of the passed device object (`device`
argument). When the reference count falls to 0, the device gets
destroyed.

All objects created from the device (like scenes, geometries, etc.)
hold a reference to the device, thus the device will not get destroyed
unless these objects are destroyed first.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewDevice], [rtcRetainDevice]
