% rtcNewSYCLDevice(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcNewSYCLDevice - creates a new device to be used with SYCL

#### SYNOPSIS

    #include <embree4/rtcore.h>

    RTCDevice rtcNewSYCLDevice(sycl::context context, const char* config);

#### DESCRIPTION

This function creates a new device to be used with SYCL for GPU
rendering and returns a handle to this device. The device object is
reference counted with an initial reference count of 1. The handle can
get released using the `rtcReleaseDevice` API call.

The passed SYCL context (`context` argument) is used to allocate GPU
data, thus only devices contained inside this context can be used for
rendering. By default the GPU data is allocated on the first GPU
device of the context, but this behavior can get changed with the
[rtcSetDeviceSYCLDevice] function.

The device object acts as a class factory for all other object
types. All objects created from the device (like scenes, geometries,
etc.) hold a reference to the device, thus the device will not be
destroyed unless these objects are destroyed first.

Objects are only compatible if they belong to the same device, e.g it
is not allowed to create a geometry in one device and attach it to a
scene created with a different device.

For an overview of configurations that can get passed (`config`
argument) please see the [rtcNewDevice] function description.

#### EXIT STATUS

On success returns a handle of the created device. On failure returns
`NULL` as device and sets a per-thread error code that can be queried
using `rtcGetDeviceError(NULL)`.

#### SEE ALSO

[rtcRetainDevice], [rtcReleaseDevice], [rtcNewDevice]
