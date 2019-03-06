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

When creating the device, Embree reads configurations for the device
from the following locations in order:

1) `config` string passed to the `rtcNewDevice` function
2) `.embree3` file in the application folder
3) `.embree3` file in the home folder

Settings performed later overwrite previous settings. This way the
configuration for the application can be changed globally (either
through the `rtcNewDevice` call or through the `.embree3` file in the
application folder), and each user has the option to modify the
configuration to fit their needs.

The following configuration is supported:

+ `threads=[int]`: Specifies a number of build threads to use. A value
  of 0 enables all detected hardware threads. By default all hardware
  threads are used.

+ `set_affinity=[0/1]`: When enabled, build threads are affinitized to
  hardware threads. This option is disabled by default on standard
  CPUs, and enabled by default on Xeon Phi Processors.

+ `start_threads=[0/1]`: When enabled, the build threads are started 
  upfront. This can be useful for benchmarking to exclude thread
  creation time. This option is disabled by default.

+ `isa=[sse2,sse4.2,avx,avx2,avx512knl,avx512skx]`: Use specified
  ISA. By default the ISA is selected automatically.

+ `max_isa=[sse2,sse4.2,avx,avx2,avx512knl,avx512skx]`: Configures the
  automated ISA selection to use maximally the specified ISA.

+ `hugepages=[0/1]`: Enables or disables usage of huge pages. Under
  Linux huge pages are used by default but under Windows and macOS
  they are disabled by default.

+ `enable_selockmemoryprivilege=[0/1]`: When set to 1, this enables the
  `SeLockMemoryPrivilege` privilege with is required to use huge pages
  on Windows. This option has an effect only under Windows and is
  ignored on other platforms. See Section [Huge Page Support] for more
  details.

+  `ignore_config_files=[0/1]`: When set to 1, configuration files are
   ignored. Default is 0.

+  `verbose=[0,1,2,3]`: Sets the verbosity of the output. When set to
   0, no output is printed by Embree, when set to a higher level more
   output is printed. By default Embree does not print anything on the
   console.

+ `frequency_level=[simd128,simd256,simd512]`: Specifies the
   frequency level the application want to run on, which can be
   either: a) simd128 for apps that do not use AVX instructions, b)
   simd256 for apps that use heavy AVX instruction, c) simd512 for
   apps that use heavy AVX-512 instructions. When some frequency level
   is specified, Embree will avoid doing optimizations that may reduce
   the frequency level below the level specified. E.g. if your app
   does not use AVX instructions setting "frequency_level=simd128"
   will cause some CPUs to run at highest frequency, which may result
   in higher application performance. However, this will prevent
   Embree from using AVX optimizations to achieve higher ray tracing
   performance, thus applications that trace many rays may still
   perform better with the default setting of simd256, even though
   this reduces frequency on some CPUs.

Different configuration options should be separated by commas, e.g.:

    rtcNewDevice("threads=1,isa=avx");

#### EXIT STATUS

On success returns a handle of the created device. On failure returns
`NULL` as device and sets a per-thread error code that can be queried
using `rtcGetDeviceError(NULL)`.

#### SEE ALSO

[rtcRetainDevice], [rtcReleaseDevice]
