% rtcNewDevice(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcNewDevice - creates a new device

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCDevice rtcNewDevice(const char* config);

#### DESCRIPTION

This function creates a new device and returns a handle to this
device. The device object is reference counted with an initial
reference count of 1. The handle can be released using the
`rtcReleaseDevice` API call.

The device object acts as a class factory for all other object
types. All objects created from the device (like scenes, geometries,
etc.) hold a reference to the device, thus the device will not be
destroyed unless these objects are destroyed first.

Objects are only compatible if they belong to the same device, e.g it
is not allowed to create a geometry in one device and attach it to a
scene created with a different device.

A configuration string (`config` argument) can be passed to the
device construction. This configuration string can be `NULL` to use
the default configuration.

The following configuration is supported:

+ `threads=[int]`: Specifies a number of build threads to use. A value
  of 0 enables all detected hardware threads. By default all hardware
  threads are used.

+ `user_threads=[int]`: Sets the number of user threads that can be
  used to join and participate in a scene commit using
  `rtcJoinCommitScene`. The tasking system will only use
  threads-user_threads many worker threads, thus if the app wants to
  solely use its threads to commit scenes, just set threads equal to
  user_threads. This option only has effect with the Intel(R)
  Threading Building Blocks (TBB) tasking system.

+ `set_affinity=[0/1]`: When enabled, build threads are affinitized to
  hardware threads. This option is disabled by default on standard
  CPUs, and enabled by default on Xeon Phi Processors.

+ `start_threads=[0/1]`: When enabled, the build threads are started 
  upfront. This can be useful for benchmarking to exclude thread
  creation time. This option is disabled by default.

+ `isa=[sse2,sse4.2,avx,avx2,avx512]`: Use specified
  ISA. By default the ISA is selected automatically.

+ `max_isa=[sse2,sse4.2,avx,avx2,avx512]`: Configures the
  automated ISA selection to use maximally the specified ISA.

+ `hugepages=[0/1]`: Enables or disables usage of huge pages. Under
  Linux huge pages are used by default but under Windows and macOS
  they are disabled by default.

+ `enable_selockmemoryprivilege=[0/1]`: When set to 1, this enables the
  `SeLockMemoryPrivilege` privilege with is required to use huge pages
  on Windows. This option has an effect only under Windows and is
  ignored on other platforms. See Section [Huge Page Support] for more
  details.

+  `verbose=[0,1,2,3]`: Sets the verbosity of the output. When set to
   0, no output is printed by Embree, when set to a higher level more
   output is printed. By default Embree does not print anything on the
   console.

+ `frequency_level=[simd128,simd256,simd512]`: Specifies the frequency
   level the application want to run on, which can be either:
   a) simd128 to run at highest frequency
   b) simd256 to run at AVX2-heavy frequency level
   c) simd512 to run at heavy AVX512 frequency level.
   When some frequency level is specified, Embree will avoid doing
   optimizations that may reduce the frequency level below the level
   specified. E.g. if your app does not use AVX instructions setting
   "frequency_level=simd128" will cause some CPUs to run at highest
   frequency, which may result in higher application performance if
   you do much shading. If you application heavily uses
   AVX code, you should best set the frequency level to simd256.
   Per default Embree tries to avoid reducing the frequency of the
   CPU by setting the simd256 level only when the CPU has no significant
   down clocking.

Different configuration options should be separated by commas, e.g.:

    rtcNewDevice("threads=1,isa=avx");

#### EXIT STATUS

On success returns a handle of the created device. On failure returns
`NULL` as device and sets a per-thread error code that can be queried
using `rtcGetDeviceError(NULL)`.

#### SEE ALSO

[rtcRetainDevice], [rtcReleaseDevice]
