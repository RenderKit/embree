% rtcIsSYCLDeviceSupported(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcIsSYCLDeviceSupported - checks if some SYCL device is supported by Embree

#### SYNOPSIS

    #include <embree4/rtcore.h>

    bool rtcIsSYCLDeviceSupported(const sycl::device sycl_device);

#### DESCRIPTION

This function can be used to check if some SYCL device (`sycl_device`
argument) is supported by Embree.

#### EXIT STATUS

The function returns true if the SYCL device is supported by Embree
and false otherwise. On failure an error code is set that can get
queried using `rtcGetDeviceError`.


#### SEE ALSO

[rtcSYCLDeviceSelector]
