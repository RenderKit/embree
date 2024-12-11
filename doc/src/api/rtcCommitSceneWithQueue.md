% rtcCommitSceneWithQueue(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcCommitSceneWithQueue - commits scene changes using a given
      SYCL queue for host-to-device memory transfers

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcCommitScene(RTCScene scene,
      sycl::queue queue,
      sycl::event* event);

#### DESCRIPTION

The `rtcCommitSceneWithQueue` function commits all changes for the specified
scene (`scene` argument). This internally triggers building of a
spatial acceleration structure for the scene using all available
worker threads. Ray queries can be performed only after committing
all scene changes.

The `rtcCommitSceneWithQueue` function differs from `rtcCommitScene` only
in that the former uses a SYCL queue passed by the application for asynchronous
host-to-device memory transfers and the latter uses an internal and temporary
SYCL queue and is blocking, i.e. the function returns after all memory
transfers are completed.

The application is responsible for synchronization when using `rtcCommitSceneWithQueue`
by either calling `wait()` on the `queue` or use the optional `event` argument
to receive a copy of the sycl::event associated with the last memcpy command
that was submitted to the queue. The argument `event` can also be a null pointer
in which case the argument will be ignored internally.

A SYCL kernel calling rtcTraversableIntersect or rtcTraversableOccluded
using a RTCTraversable associated with the RTCScene `scene` has a dependency on
the host-to-device memory transfers and therefore has to be synchronized properly.

This function is only available when SYCL support is enabled. In case the
RTCDevice associated with the `scene` is not a SYCL device, `rtcCommitSceneWithQueue`
will behave the same as `rtcCommitScene`.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcCommitScene]
