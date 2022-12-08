% rtcSetDeviceSYCLDevice(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetDeviceSYCLDevice - sets the SYCL device to be used for memory allocations

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcSetDeviceSYCLDevice(RTCDevice device, const sycl::device sycl_device);

#### DESCRIPTION

This function sets the SYCL device to be used to allocate GPU
memory. This device must be one of the devices contained inside the
context used to create the Embree device.

#### EXIT STATUS

#### SEE ALSO

[rtcNewSYCLDevice]
