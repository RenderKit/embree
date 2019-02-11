% rtcSetDeviceMemoryMonitorFunction(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetDeviceMemoryMonitorFunction - registers a callback function
      to track memory consumption

#### SYNOPSIS

    #include <embree3/rtcore.h>

    typedef bool (*RTCMemoryMonitorFunction)(
      void* userPtr,
      ssize_t bytes,
      bool post
    );

    void rtcSetDeviceMemoryMonitorFunction(
      RTCDevice device,
      RTCMemoryMonitorFunction memoryMonitor,
      void* userPtr
    );

#### DESCRIPTION

Using the `rtcSetDeviceMemoryMonitorFunction` call, it is possible to
register a callback function (`memoryMonitor` argument) with payload
(`userPtr` argument) for a device (`device` argument), which is called
whenever internal memory is allocated or deallocated by objects of that
device. Using this memory monitor callback mechanism, the application
can track the memory consumption of an Embree device, and optionally
terminate API calls that consume too much memory.

Only a single callback function can be registered per device, and
further invocations overwrite the previously set callback function.
Passing `NULL` as function pointer disables the registered callback
function.

Once registered, the Embree device will invoke the memory monitor
callback function before or after it allocates or frees important
memory blocks. The callback function gets passed the payload as
specified at registration time (`userPtr` argument), the number of
bytes allocated or deallocated (`bytes` argument), and whether the
callback is invoked after the allocation or deallocation took place
(`post` argument). The callback function might get called from
multiple threads concurrently.

The application can track the current memory usage of the Embree
device by atomically accumulating the `bytes` input parameter provided
to the callback function. This parameter will be >0 for allocations
and <0 for deallocations.

Embree will continue its operation normally when returning `true` from
the callback function. If `false` is returned, Embree will cancel the
current operation with the `RTC_ERROR_OUT_OF_MEMORY` error
code. Issuing multiple cancel requests from different threads is
allowed. Canceling will only happen when the callback was called for
allocations (bytes > 0), otherwise the cancel request will be ignored.

If a callback to cancel was invoked before the allocation happens
(`post == false`), then the `bytes` parameter should not be
accumulated, as the allocation will never happen. If the callback to
cancel was invoked after the allocation happened (`post == true`),
then the `bytes` parameter should be accumulated, as the allocation
properly happened and a deallocation will later free that data block.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewDevice]
