% rtcSYCLDeviceSelector(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcSYCLDeviceSelector - SYCL device selector function to select
      devices supported by Embree

#### SYNOPSIS

    #include <embree4/rtcore.h>

    int rtcSYCLDeviceSelector(const sycl::device sycl_device);

#### DESCRIPTION

This function checks if the passed SYCL device (`sycl_device`
arguments) is supported by Embree or not. This function can be used
directly to select some supported SYCL device by using it as SYCL
device selector function. For instance, the following code sequence
selects an Embree supported SYCL device and creates an Embree device
from it:

    sycl::device sycl_device(rtcSYCLDeviceSelector);
    sycl::queue sycl_queue(sycl_device);
    sycl::context(sycl_device);
    RTCDevice device = rtcNewSYCLDevice(sycl_context,nullptr);

#### EXIT STATUS

The function returns -1 if the SYCL device is supported by Embree and
1 otherwise. On failure an error code is set that can get queried
using `rtcGetDeviceError`.

#### SEE ALSO

[rtcIsSYCLDeviceSupported]
