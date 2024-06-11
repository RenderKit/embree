#include "pyembree.h"

namespace py = pybind11;


/* error reporting function */
void error_handler(void* userPtr, const RTCError code, const char* str = nullptr)
{
    if (code == RTC_ERROR_NONE)
        return;
  
    printf("Embree: ");
    switch (code) {
    case RTC_ERROR_UNKNOWN          : printf("RTC_ERROR_UNKNOWN"); break;
    case RTC_ERROR_INVALID_ARGUMENT : printf("RTC_ERROR_INVALID_ARGUMENT"); break;
    case RTC_ERROR_INVALID_OPERATION: printf("RTC_ERROR_INVALID_OPERATION"); break;
    case RTC_ERROR_OUT_OF_MEMORY    : printf("RTC_ERROR_OUT_OF_MEMORY"); break;
    case RTC_ERROR_UNSUPPORTED_CPU  : printf("RTC_ERROR_UNSUPPORTED_CPU"); break;
    case RTC_ERROR_CANCELLED        : printf("RTC_ERROR_CANCELLED"); break;
    default                         : printf("invalid error code"); break;
    }
    if (str) {
        printf(" (");
        while (*str) putchar(*str++);
        printf(")\n");
    }
    exit(1);
}

#if defined(EMBREE_SYCL_SUPPORT)
void exception_handler(sycl::exception_list exceptions)
{
  for (std::exception_ptr const& e : exceptions) {
    try {
      std::rethrow_exception(e);
    } catch(sycl::exception const& e) {
      std::cout << "Caught asynchronous SYCL exception: " << e.what() << std::endl;
    }
  }
};
#endif

void bind_rtcore_device(pybind11::module &m) {

    py::class_<RTCDeviceWrapper>(m, "RTCDeviceWrapper");

    /* Creates a new Embree device. */
    m.def("rtcNewDevice", [](const char* config){return RTCDeviceWrapper{rtcNewDevice(config)};});

    m.def("rtcCreateDevice", [](const char* config, bool enable_sycl, bool jit_cache) {
        #if defined(EMBREE_SYCL_SUPPORT)

            /* create SYCL device */
            if (enable_sycl)
            {
                printf("SYCLSYCLSYCLSYCLSYCLSYCLSYCLSYCL");
                printf("SYCLSYCLSYCLSYCLSYCLSYCLSYCLSYCL");
                printf("SYCLSYCLSYCLSYCLSYCLSYCLSYCLSYCL");
                printf("SYCLSYCLSYCLSYCLSYCLSYCLSYCLSYCL");
                printf("SYCLSYCLSYCLSYCLSYCLSYCLSYCLSYCL");
                printf("SYCLSYCLSYCLSYCLSYCLSYCLSYCLSYCL");
                printf("SYCLSYCLSYCLSYCLSYCLSYCLSYCLSYCL");
                //if (jit_cache)
                //{
                //    /* enable SYCL JIT caching */
                //    std::string exe = getExecutableFileName();
                //    std::string cache_dir = exe.path() + FileName("cache");

                //#if defined(__WIN32__)
                //    _putenv_s("SYCL_CACHE_PERSISTENT","1");
                //    _putenv_s("SYCL_CACHE_DIR",cache_dir.c_str());
                //#else
                //    setenv("SYCL_CACHE_PERSISTENT","1",1);
                //    setenv("SYCL_CACHE_DIR",cache_dir.c_str(),1);
                //#endif
                //}

                auto exception_handler = [](sycl::exception_list exceptions)
                {
                    for (std::exception_ptr const &e : exceptions) {
                        try {
                            std::rethrow_exception(e);
                        } catch (sycl::exception const &e) {
                            std::cout << "ERROR: Caught asynchronous SYCL exception:\n"
                                      << e.what() << std::endl;
                            exit(1);
                        }
                    }
                };

                //check_raytracing_support();

                RTCDeviceWrapper w;
                /* select device supported by Embree */
                try {
                    w.sycl_device = new sycl::device(rtcSYCLDeviceSelector);
                } catch(std::exception& e) {
                    std::cerr << "Caught exception creating sycl::device: " << e.what() << std::endl;
                    //printAllSYCLDevices();
                    throw;
                }
                sycl::platform platform = w.sycl_device->get_platform();
                std::cout << "Selected SYCL Platform: " + platform.get_info<sycl::info::platform::name>() << std::endl;
                std::cout << "Selected SYCL Device: " + w.sycl_device->get_info<sycl::info::device::name>() << std::endl;

                w.sycl_queue = new sycl::queue(*w.sycl_device, exception_handler, { sycl::property::queue::in_order(), sycl::property::queue::enable_profiling() });
                w.sycl_context = new sycl::context(*w.sycl_device);
                w.d = rtcNewSYCLDevice(*w.sycl_context, config);
                error_handler(nullptr, rtcGetDeviceError(w.d));

                if (verbosity >= 1) {
                  printAllSYCLDevices();
                }

                enableUSMAllocTutorial(w.sycl_context, w.sycl_device);
            }

            /* create standard device */
            else
        #endif

            RTCDeviceWrapper w;
            {
                w.d = rtcNewDevice(config);
                error_handler(nullptr,rtcGetDeviceError(w.d));
            }

        /* set error handler */
        rtcSetDeviceErrorFunction(w.d,error_handler,nullptr);

        return w;

        // #endif ..else
    });


#if defined(EMBREE_SYCL_SUPPORT) && defined(SYCL_LANGUAGE_VERSION)
    /* Creates a new Embree SYCL device. */
    m.def("rtcNewSYCLDevice", [](sycl::context context, const char* config){return RTCDeviceWrapper{rtcNewSYCLDevice(context, config)};});

    /* Checks if SYCL device is supported by Embree. */
    m.def("rtcIsSYCLDeviceSupported", [](const sycl::device sycl_device){return RTCDeviceWrapper{rtcIsSYCLDeviceSupported(sycl_device)};});

    /* SYCL selector for Embree supported devices */
    m.def("rtcSYCLDeviceSelector", [](const sycl::device sycl_device){return RTCDeviceWrapper{rtcSYCLDeviceSelector(sycl_device)};});

    /* Set the SYCL device to be used to allocate data */
    m.def("rtcSetDeviceSYCLDevice", [](RTCDevice device, const sycl::device sycl_device){return RTCDeviceWrapper{rtcSetDeviceSYCLDevice(device, sycl_device)};});
#endif


    /* Retains the Embree device (increments the reference count). */
    m.def("rtcRetainDevice", [](RTCDeviceWrapper device){rtcRetainDevice(device.d);});

    /* Releases an Embree device (decrements the reference count). */
    m.def("rtcReleaseDevice", [](RTCDeviceWrapper device){rtcReleaseDevice(device.d);});

    /* Device properties */
    py::enum_<RTCDeviceProperty>(m, "RTCDeviceProperty")
        .value("RTC_DEVICE_PROPERTY_VERSION", RTC_DEVICE_PROPERTY_VERSION)
        .value("RTC_DEVICE_PROPERTY_VERSION_MAJOR", RTC_DEVICE_PROPERTY_VERSION_MAJOR)
        .value("RTC_DEVICE_PROPERTY_VERSION_MINOR", RTC_DEVICE_PROPERTY_VERSION_MINOR)
        .value("RTC_DEVICE_PROPERTY_VERSION_PATCH", RTC_DEVICE_PROPERTY_VERSION_PATCH)

        .value("RTC_DEVICE_PROPERTY_NATIVE_RAY4_SUPPORTED", RTC_DEVICE_PROPERTY_NATIVE_RAY4_SUPPORTED)
        .value("RTC_DEVICE_PROPERTY_NATIVE_RAY8_SUPPORTED", RTC_DEVICE_PROPERTY_NATIVE_RAY8_SUPPORTED)
        .value("RTC_DEVICE_PROPERTY_NATIVE_RAY16_SUPPORTED", RTC_DEVICE_PROPERTY_NATIVE_RAY16_SUPPORTED)

        .value("RTC_DEVICE_PROPERTY_BACKFACE_CULLING_SPHERES_ENABLED", RTC_DEVICE_PROPERTY_BACKFACE_CULLING_SPHERES_ENABLED)
        .value("RTC_DEVICE_PROPERTY_BACKFACE_CULLING_CURVES_ENABLED", RTC_DEVICE_PROPERTY_BACKFACE_CULLING_CURVES_ENABLED)
        .value("RTC_DEVICE_PROPERTY_RAY_MASK_SUPPORTED", RTC_DEVICE_PROPERTY_RAY_MASK_SUPPORTED)
        .value("RTC_DEVICE_PROPERTY_BACKFACE_CULLING_ENABLED", RTC_DEVICE_PROPERTY_BACKFACE_CULLING_ENABLED)
        .value("RTC_DEVICE_PROPERTY_FILTER_FUNCTION_SUPPORTED", RTC_DEVICE_PROPERTY_FILTER_FUNCTION_SUPPORTED)
        .value("RTC_DEVICE_PROPERTY_IGNORE_INVALID_RAYS_ENABLED", RTC_DEVICE_PROPERTY_IGNORE_INVALID_RAYS_ENABLED)
        .value("RTC_DEVICE_PROPERTY_COMPACT_POLYS_ENABLED", RTC_DEVICE_PROPERTY_COMPACT_POLYS_ENABLED)

        .value("RTC_DEVICE_PROPERTY_TRIANGLE_GEOMETRY_SUPPORTED", RTC_DEVICE_PROPERTY_TRIANGLE_GEOMETRY_SUPPORTED)
        .value("RTC_DEVICE_PROPERTY_QUAD_GEOMETRY_SUPPORTED", RTC_DEVICE_PROPERTY_QUAD_GEOMETRY_SUPPORTED)
        .value("RTC_DEVICE_PROPERTY_SUBDIVISION_GEOMETRY_SUPPORTED", RTC_DEVICE_PROPERTY_SUBDIVISION_GEOMETRY_SUPPORTED)
        .value("RTC_DEVICE_PROPERTY_CURVE_GEOMETRY_SUPPORTED", RTC_DEVICE_PROPERTY_CURVE_GEOMETRY_SUPPORTED)
        .value("RTC_DEVICE_PROPERTY_USER_GEOMETRY_SUPPORTED", RTC_DEVICE_PROPERTY_USER_GEOMETRY_SUPPORTED)
        .value("RTC_DEVICE_PROPERTY_POINT_GEOMETRY_SUPPORTED", RTC_DEVICE_PROPERTY_POINT_GEOMETRY_SUPPORTED)

        .value("RTC_DEVICE_PROPERTY_TASKING_SYSTEM", RTC_DEVICE_PROPERTY_TASKING_SYSTEM)
        .value("RTC_DEVICE_PROPERTY_JOIN_COMMIT_SUPPORTED", RTC_DEVICE_PROPERTY_JOIN_COMMIT_SUPPORTED)
        .value("RTC_DEVICE_PROPERTY_PARALLEL_COMMIT_SUPPORTED", RTC_DEVICE_PROPERTY_PARALLEL_COMMIT_SUPPORTED)
        .export_values();

    /* Gets a device property. */
    m.def("rtcGetDeviceProperty", [](RTCDeviceWrapper device, enum RTCDeviceProperty prop){ return rtcGetDeviceProperty(device.d, prop);});

    /* Sets a device property. */
    m.def("rtcSetDeviceProperty", [](RTCDeviceWrapper device, enum RTCDeviceProperty prop, ssize_t value){ return rtcSetDeviceProperty(device.d, prop, value);});
  
    /* Error codes */
    py::enum_<RTCError>(m, "RTCError")
        .value("RTC_ERROR_NONE", RTC_ERROR_NONE)
        .value("RTC_ERROR_UNKNOWN", RTC_ERROR_UNKNOWN)
        .value("RTC_ERROR_INVALID_ARGUMENT", RTC_ERROR_INVALID_ARGUMENT)
        .value("RTC_ERROR_INVALID_OPERATION", RTC_ERROR_INVALID_OPERATION)
        .value("RTC_ERROR_OUT_OF_MEMORY", RTC_ERROR_OUT_OF_MEMORY)
        .value("RTC_ERROR_UNSUPPORTED_CPU", RTC_ERROR_UNSUPPORTED_CPU)
        .value("RTC_ERROR_CANCELLED", RTC_ERROR_CANCELLED)
        .export_values();

    /* Returns the error code. */
    m.def("rtcGetDeviceError", [](RTCDeviceWrapper device){ return rtcGetDeviceError(device.d); });

    /* Sets the error callback function. */
    m.def("rtcSetDeviceErrorFunction", [](RTCDeviceWrapper device, RTCErrorFunction error, void* userPtr){ return rtcSetDeviceErrorFunction(device.d, error, userPtr); });

    /* Sets the memory monitor callback function. */
    m.def("rtcSetDeviceMemoryMonitorFunction", [](RTCDeviceWrapper device, RTCMemoryMonitorFunction memoryMonitor, void* userPtr){ return rtcSetDeviceMemoryMonitorFunction(device.d, memoryMonitor, userPtr);});
}