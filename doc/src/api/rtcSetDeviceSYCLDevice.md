% rtcSetDeviceSYCLDevice(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcSetDeviceSYCLDevice - sets the SYCL device to be used for memory allocations

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcSetDeviceSYCLDevice(RTCDevice device, const sycl::device sycl_device);

#### DESCRIPTION

This function sets the SYCL device (`sycl_device` argument) to be used
to allocate GPU memory when using the specified Embree device
(`device` argument). This SYCL device must be one of the SYCL devices
contained inside the SYCL context used to create the Embree device.

#### EXIT STATUS

On failure an error code is set that can get queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewSYCLDevice]
