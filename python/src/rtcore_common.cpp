#include "pyembree.h"

namespace py = pybind11;


void bind_rtcore_common(py::module &m) {
    /* Invalid geometry ID */
    m.attr("RTC_INVALID_GEOMETRY_ID") = RTC_INVALID_GEOMETRY_ID;

    /* Maximum number of time steps */
    m.attr("RTC_MAX_TIME_STEP_COUNT") = RTC_MAX_TIME_STEP_COUNT;

    /* Formats of buffers and other data structures */
    py::enum_<RTCFormat>(m, "RTCFormat")
      .value("RTC_FORMAT_UNDEFINED", RTC_FORMAT_UNDEFINED)

      /* 8-bit unsigned integer */
      .value("RTC_FORMAT_UCHAR", RTC_FORMAT_UCHAR)
      .value("RTC_FORMAT_UCHAR2", RTC_FORMAT_UCHAR2)
      .value("RTC_FORMAT_UCHAR3", RTC_FORMAT_UCHAR3)
      .value("RTC_FORMAT_UCHAR4", RTC_FORMAT_UCHAR4)

      /* 8-bit signed integer */
      .value("RTC_FORMAT_CHAR", RTC_FORMAT_CHAR)
      .value("RTC_FORMAT_CHAR2", RTC_FORMAT_CHAR2)
      .value("RTC_FORMAT_CHAR3", RTC_FORMAT_CHAR3)
      .value("RTC_FORMAT_CHAR4", RTC_FORMAT_CHAR4)

      /* 16-bit unsigned integer */
      .value("RTC_FORMAT_USHORT", RTC_FORMAT_USHORT)
      .value("RTC_FORMAT_USHORT2", RTC_FORMAT_USHORT2)
      .value("RTC_FORMAT_USHORT3", RTC_FORMAT_USHORT3)
      .value("RTC_FORMAT_USHORT4", RTC_FORMAT_USHORT4)

      /* 16-bit signed integer */
      .value("RTC_FORMAT_SHORT", RTC_FORMAT_SHORT)
      .value("RTC_FORMAT_SHORT2", RTC_FORMAT_SHORT2)
      .value("RTC_FORMAT_SHORT3", RTC_FORMAT_SHORT3)
      .value("RTC_FORMAT_SHORT4", RTC_FORMAT_SHORT4)

      /* 32-bit unsigned integer */
      .value("RTC_FORMAT_UINT", RTC_FORMAT_UINT)
      .value("RTC_FORMAT_UINT2", RTC_FORMAT_UINT2)
      .value("RTC_FORMAT_UINT3", RTC_FORMAT_UINT3)
      .value("RTC_FORMAT_UINT4", RTC_FORMAT_UINT4)

      /* 32-bit signed integer */
      .value("RTC_FORMAT_INT", RTC_FORMAT_INT)
      .value("RTC_FORMAT_INT2", RTC_FORMAT_INT2)
      .value("RTC_FORMAT_INT3", RTC_FORMAT_INT3)
      .value("RTC_FORMAT_INT4", RTC_FORMAT_INT4)

      /* 64-bit unsigned integer */
      .value("RTC_FORMAT_ULLONG", RTC_FORMAT_ULLONG)
      .value("RTC_FORMAT_ULLONG2", RTC_FORMAT_ULLONG2)
      .value("RTC_FORMAT_ULLONG3", RTC_FORMAT_ULLONG3)
      .value("RTC_FORMAT_ULLONG4", RTC_FORMAT_ULLONG4)

      /* 64-bit signed integer */
      .value("RTC_FORMAT_LLONG", RTC_FORMAT_LLONG)
      .value("RTC_FORMAT_LLONG2", RTC_FORMAT_LLONG2)
      .value("RTC_FORMAT_LLONG3", RTC_FORMAT_LLONG3)
      .value("RTC_FORMAT_LLONG4", RTC_FORMAT_LLONG4)

      /* 32-bit float */
      .value("RTC_FORMAT_FLOAT", RTC_FORMAT_FLOAT)
      .value("RTC_FORMAT_FLOAT2", RTC_FORMAT_FLOAT2)
      .value("RTC_FORMAT_FLOAT3", RTC_FORMAT_FLOAT3)
      .value("RTC_FORMAT_FLOAT4", RTC_FORMAT_FLOAT4)
      .value("RTC_FORMAT_FLOAT5", RTC_FORMAT_FLOAT5)
      .value("RTC_FORMAT_FLOAT6", RTC_FORMAT_FLOAT6)
      .value("RTC_FORMAT_FLOAT7", RTC_FORMAT_FLOAT7)
      .value("RTC_FORMAT_FLOAT8", RTC_FORMAT_FLOAT8)
      .value("RTC_FORMAT_FLOAT9", RTC_FORMAT_FLOAT9)
      .value("RTC_FORMAT_FLOAT10", RTC_FORMAT_FLOAT10)
      .value("RTC_FORMAT_FLOAT11", RTC_FORMAT_FLOAT11)
      .value("RTC_FORMAT_FLOAT12", RTC_FORMAT_FLOAT12)
      .value("RTC_FORMAT_FLOAT13", RTC_FORMAT_FLOAT13)
      .value("RTC_FORMAT_FLOAT14", RTC_FORMAT_FLOAT14)
      .value("RTC_FORMAT_FLOAT15", RTC_FORMAT_FLOAT15)
      .value("RTC_FORMAT_FLOAT16", RTC_FORMAT_FLOAT16)

      /* 32-bit float matrix (row-major order) */
      .value("RTC_FORMAT_FLOAT2X2_ROW_MAJOR", RTC_FORMAT_FLOAT2X2_ROW_MAJOR)
      .value("RTC_FORMAT_FLOAT2X3_ROW_MAJOR", RTC_FORMAT_FLOAT2X3_ROW_MAJOR)
      .value("RTC_FORMAT_FLOAT2X4_ROW_MAJOR", RTC_FORMAT_FLOAT2X4_ROW_MAJOR)
      .value("RTC_FORMAT_FLOAT3X2_ROW_MAJOR", RTC_FORMAT_FLOAT3X2_ROW_MAJOR)
      .value("RTC_FORMAT_FLOAT3X3_ROW_MAJOR", RTC_FORMAT_FLOAT3X3_ROW_MAJOR)
      .value("RTC_FORMAT_FLOAT3X4_ROW_MAJOR", RTC_FORMAT_FLOAT3X4_ROW_MAJOR)
      .value("RTC_FORMAT_FLOAT4X2_ROW_MAJOR", RTC_FORMAT_FLOAT4X2_ROW_MAJOR)
      .value("RTC_FORMAT_FLOAT4X3_ROW_MAJOR", RTC_FORMAT_FLOAT4X3_ROW_MAJOR)
      .value("RTC_FORMAT_FLOAT4X4_ROW_MAJOR", RTC_FORMAT_FLOAT4X4_ROW_MAJOR)

      /* 32-bit float matrix (column-major order) */
      .value("RTC_FORMAT_FLOAT2X2_COLUMN_MAJOR", RTC_FORMAT_FLOAT2X2_COLUMN_MAJOR)
      .value("RTC_FORMAT_FLOAT2X3_COLUMN_MAJOR", RTC_FORMAT_FLOAT2X3_COLUMN_MAJOR)
      .value("RTC_FORMAT_FLOAT2X4_COLUMN_MAJOR", RTC_FORMAT_FLOAT2X4_COLUMN_MAJOR)
      .value("RTC_FORMAT_FLOAT3X2_COLUMN_MAJOR", RTC_FORMAT_FLOAT3X2_COLUMN_MAJOR)
      .value("RTC_FORMAT_FLOAT3X3_COLUMN_MAJOR", RTC_FORMAT_FLOAT3X3_COLUMN_MAJOR)
      .value("RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR", RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR)
      .value("RTC_FORMAT_FLOAT4X2_COLUMN_MAJOR", RTC_FORMAT_FLOAT4X2_COLUMN_MAJOR)
      .value("RTC_FORMAT_FLOAT4X3_COLUMN_MAJOR", RTC_FORMAT_FLOAT4X3_COLUMN_MAJOR)
      .value("RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR", RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR)

      /* special 12-byte format for grids */
      .value("RTC_FORMAT_GRID", RTC_FORMAT_GRID)
      .export_values();

    /* Build quality levels */
    py::enum_<RTCBuildQuality>(m, "RTCBuildQuality")
      .value("RTC_BUILD_QUALITY_LOW",RTC_BUILD_QUALITY_LOW)
      .value("RTC_BUILD_QUALITY_MEDIUM",RTC_BUILD_QUALITY_MEDIUM)
      .value("RTC_BUILD_QUALITY_HIGH",RTC_BUILD_QUALITY_HIGH)
      .value("RTC_BUILD_QUALITY_REFIT",RTC_BUILD_QUALITY_REFIT)
      .export_values();
    
    /* Axis-aligned bounding box representation */
    py::class_<RTCBounds>(m, "RTCBounds")
      .def_readwrite("lower_x", &RTCBounds::lower_x)
      .def_readwrite("lower_y", &RTCBounds::lower_y)
      .def_readwrite("lower_z", &RTCBounds::lower_z)
      .def_readwrite("upper_x", &RTCBounds::upper_x)
      .def_readwrite("upper_y", &RTCBounds::upper_y)
      .def_readwrite("upper_z", &RTCBounds::upper_z);
    
    /* Linear axis-aligned bounding box representation */
    py::class_<RTCLinearBounds>(m, "RTCLinearBounds")
      .def_readwrite("bounds0", &RTCLinearBounds::bounds0)
      .def_readwrite("bounds1", &RTCLinearBounds::bounds1);
    
    /* Feature flags for SYCL specialization constants */
    py::enum_<RTCFeatureFlags>(m, "RTCFeatureFlags")
        .value("RTC_FEATURE_FLAG_NONE", RTC_FEATURE_FLAG_NONE)
        .value("RTC_FEATURE_FLAG_MOTION_BLUR", RTC_FEATURE_FLAG_MOTION_BLUR)
        .value("RTC_FEATURE_FLAG_TRIANGLE", RTC_FEATURE_FLAG_TRIANGLE)
        .value("RTC_FEATURE_FLAG_QUAD", RTC_FEATURE_FLAG_QUAD)
        .value("RTC_FEATURE_FLAG_GRID", RTC_FEATURE_FLAG_GRID)
        .value("RTC_FEATURE_FLAG_SUBDIVISION", RTC_FEATURE_FLAG_SUBDIVISION)
        .value("RTC_FEATURE_FLAG_CONE_LINEAR_CURVE", RTC_FEATURE_FLAG_CONE_LINEAR_CURVE)
        .value("RTC_FEATURE_FLAG_ROUND_LINEAR_CURVE", RTC_FEATURE_FLAG_ROUND_LINEAR_CURVE)
        .value("RTC_FEATURE_FLAG_FLAT_LINEAR_CURVE", RTC_FEATURE_FLAG_FLAT_LINEAR_CURVE)
        .value("RTC_FEATURE_FLAG_ROUND_BEZIER_CURVE", RTC_FEATURE_FLAG_ROUND_BEZIER_CURVE)
        .value("RTC_FEATURE_FLAG_FLAT_BEZIER_CURVE", RTC_FEATURE_FLAG_FLAT_BEZIER_CURVE)
        .value("RTC_FEATURE_FLAG_NORMAL_ORIENTED_BEZIER_CURVE", RTC_FEATURE_FLAG_NORMAL_ORIENTED_BEZIER_CURVE)
        .value("RTC_FEATURE_FLAG_ROUND_BSPLINE_CURVE", RTC_FEATURE_FLAG_ROUND_BSPLINE_CURVE)
        .value("RTC_FEATURE_FLAG_FLAT_BSPLINE_CURVE", RTC_FEATURE_FLAG_FLAT_BSPLINE_CURVE)
        .value("RTC_FEATURE_FLAG_NORMAL_ORIENTED_BSPLINE_CURVE", RTC_FEATURE_FLAG_NORMAL_ORIENTED_BSPLINE_CURVE)
        .value("RTC_FEATURE_FLAG_ROUND_HERMITE_CURVE", RTC_FEATURE_FLAG_ROUND_HERMITE_CURVE)
        .value("RTC_FEATURE_FLAG_FLAT_HERMITE_CURVE", RTC_FEATURE_FLAG_FLAT_HERMITE_CURVE)
        .value("RTC_FEATURE_FLAG_NORMAL_ORIENTED_HERMITE_CURVE", RTC_FEATURE_FLAG_NORMAL_ORIENTED_HERMITE_CURVE)
        .value("RTC_FEATURE_FLAG_ROUND_CATMULL_ROM_CURVE", RTC_FEATURE_FLAG_ROUND_CATMULL_ROM_CURVE)
        .value("RTC_FEATURE_FLAG_FLAT_CATMULL_ROM_CURVE", RTC_FEATURE_FLAG_FLAT_CATMULL_ROM_CURVE)
        .value("RTC_FEATURE_FLAG_NORMAL_ORIENTED_CATMULL_ROM_CURVE", RTC_FEATURE_FLAG_NORMAL_ORIENTED_CATMULL_ROM_CURVE)
        .value("RTC_FEATURE_FLAG_SPHERE_POINT", RTC_FEATURE_FLAG_SPHERE_POINT)
        .value("RTC_FEATURE_FLAG_DISC_POINT", RTC_FEATURE_FLAG_DISC_POINT)
        .value("RTC_FEATURE_FLAG_ORIENTED_DISC_POINT", RTC_FEATURE_FLAG_ORIENTED_DISC_POINT)
        .value("RTC_FEATURE_FLAG_POINT", RTC_FEATURE_FLAG_POINT)
        .value("RTC_FEATURE_FLAG_ROUND_CURVES", RTC_FEATURE_FLAG_ROUND_CURVES)
        .value("RTC_FEATURE_FLAG_FLAT_CURVES", RTC_FEATURE_FLAG_FLAT_CURVES)
        .value("RTC_FEATURE_FLAG_NORMAL_ORIENTED_CURVES", RTC_FEATURE_FLAG_NORMAL_ORIENTED_CURVES)
        .value("RTC_FEATURE_FLAG_LINEAR_CURVES", RTC_FEATURE_FLAG_LINEAR_CURVES)
        .value("RTC_FEATURE_FLAG_BEZIER_CURVES", RTC_FEATURE_FLAG_BEZIER_CURVES)
        .value("RTC_FEATURE_FLAG_BSPLINE_CURVES", RTC_FEATURE_FLAG_BSPLINE_CURVES)
        .value("RTC_FEATURE_FLAG_HERMITE_CURVES", RTC_FEATURE_FLAG_HERMITE_CURVES)
        .value("RTC_FEATURE_FLAG_CURVES", RTC_FEATURE_FLAG_CURVES)
        .value("RTC_FEATURE_FLAG_INSTANCE", RTC_FEATURE_FLAG_INSTANCE)
        .value("RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS", RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS)
        .value("RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_GEOMETRY", RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_GEOMETRY)
        .value("RTC_FEATURE_FLAG_FILTER_FUNCTION", RTC_FEATURE_FLAG_FILTER_FUNCTION)
        .value("RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_ARGUMENTS", RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_ARGUMENTS)
        .value("RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_GEOMETRY", RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_GEOMETRY)
        .value("RTC_FEATURE_FLAG_USER_GEOMETRY", RTC_FEATURE_FLAG_USER_GEOMETRY)
        .value("RTC_FEATURE_FLAG_32_BIT_RAY_MASK", RTC_FEATURE_FLAG_32_BIT_RAY_MASK)
        .value("RTC_FEATURE_FLAG_ALL", RTC_FEATURE_FLAG_ALL)
        .export_values();

    /* Ray query flags */
    py::enum_<RTCRayQueryFlags>(m, "RTCRayQueryFlags")
        /* matching intel_ray_flags_t layout */
        .value("RTC_RAY_QUERY_FLAG_NONE", RTC_RAY_QUERY_FLAG_NONE)
        .value("RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER", RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER)
        /* embree specific flags */
        .value("RTC_RAY_QUERY_FLAG_INCOHERENT", RTC_RAY_QUERY_FLAG_INCOHERENT)
        .value("RTC_RAY_QUERY_FLAG_COHERENT", RTC_RAY_QUERY_FLAG_COHERENT)
        .export_values();

    py::class_<RTCFilterFunctionNArguments>(m, "RTCFilterFunctionNArguments")
        .def_readwrite("valid", &RTCFilterFunctionNArguments::valid)
        .def_readwrite("geometryUserPtr", &RTCFilterFunctionNArguments::geometryUserPtr)
        .def_readwrite("context", &RTCFilterFunctionNArguments::context)
        .def_property("ray", [](RTCFilterFunctionNArguments& c)->RTCRayNWrapper { return {c.ray};}, [](RTCFilterFunctionNArguments& c, RTCRayNWrapper r){ c.ray = r.p; })
        .def_property("hit", [](RTCFilterFunctionNArguments& c)->RTCHitNWrapper { return {c.hit};}, [](RTCFilterFunctionNArguments& c, RTCHitNWrapper h){ c.hit = h.p; })
        .def_readwrite("N", &RTCFilterFunctionNArguments::N);
    
    ///* Filter callback function */
    //typedef void (*RTCFilterFunctionN)(const struct RTCFilterFunctionNArguments* args);
    //
    ///* Intersection callback function */
    //struct RTCIntersectFunctionNArguments;
    //typedef void (*RTCIntersectFunctionN)(const struct RTCIntersectFunctionNArguments* args);
    //
    ///* Occlusion callback function */
    //struct RTCOccludedFunctionNArguments;
    //typedef void (*RTCOccludedFunctionN)(const struct RTCOccludedFunctionNArguments* args);
    
    /* Ray query context passed to intersect/occluded calls */
    py::class_<RTCRayQueryContext>(m, "RTCRayQueryContext")
        #if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
        .def_readwrite("instStackSize", &RTCRayQueryContext::instStackSize)
        #endif
        BIND_PROPERTY_ARRAY(RTCRayQueryContext, instID, unsigned int, RTC_MAX_INSTANCE_LEVEL_COUNT);

    /* Initializes an ray query context. */
    m.def("rtcInitRayQueryContext", &rtcInitRayQueryContext);
    
    /* Point query structure for closest point query */
    py::class_<RTCPointQuery>(m, "RTCPointQuery")
        .def_readwrite("x", &RTCPointQuery::x)
        .def_readwrite("y", &RTCPointQuery::y)
        .def_readwrite("z", &RTCPointQuery::z)
        .def_readwrite("time", &RTCPointQuery::time)
        .def_readwrite("radius", &RTCPointQuery::radius);


#define DEFINE_POINT_QUERY(count)                                           \
    /* Ray structure for a packet of rays */                                \
    py::class_<RTCPointQuery##count>(m, "RTCPointQuery" STRINGIFY(count))   \
        BIND_PROPERTY_ARRAY(RTCPointQuery##count, x, float, count)          \
        BIND_PROPERTY_ARRAY(RTCPointQuery##count, y, float, count)          \
        BIND_PROPERTY_ARRAY(RTCPointQuery##count, z, float, count)          \
        BIND_PROPERTY_ARRAY(RTCPointQuery##count, time, float, count)       \
        BIND_PROPERTY_ARRAY(RTCPointQuery##count, radius, float, count);

    DEFINE_POINT_QUERY(4)
    DEFINE_POINT_QUERY(8)
    DEFINE_POINT_QUERY(16)
    
    py::class_<RTCPointQueryNWrapper>(m, "RTCPointQueryNWrapper");
    
    py::class_<RTCPointQueryContext>(m, "RTCPointQueryContext")
        BIND_PROPERTY_ARRAY_2D(RTCPointQueryContext, world2inst, float, RTC_MAX_INSTANCE_LEVEL_COUNT, 16)
        BIND_PROPERTY_ARRAY_2D(RTCPointQueryContext, inst2world, float, RTC_MAX_INSTANCE_LEVEL_COUNT, 16)
        BIND_PROPERTY_ARRAY(RTCPointQueryContext, instID, float, RTC_MAX_INSTANCE_LEVEL_COUNT)
        .def_readwrite("instStackSize", &RTCPointQueryContext::instStackSize);
    
    /* Initializes an ray query context. */
    m.def("rtcInitPointQueryContext", &rtcInitPointQueryContext);
    
    py::class_<RTCPointQueryFunctionArguments>(m, "RTCPointQueryFunctionArguments")
        .def_readwrite("query", &RTCPointQueryFunctionArguments::query)
        .def_readwrite("userPtr", &RTCPointQueryFunctionArguments::userPtr)
        .def_readwrite("primID", &RTCPointQueryFunctionArguments::primID)
        .def_readwrite("geomID", &RTCPointQueryFunctionArguments::geomID)
        .def_readwrite("context", &RTCPointQueryFunctionArguments::context)
        .def_readwrite("similarityScale", &RTCPointQueryFunctionArguments::similarityScale);
    
    //typedef bool (*RTCPointQueryFunction)(struct RTCPointQueryFunctionArguments* args);
    
    #if defined(EMBREE_SYCL_SUPPORT) && defined(SYCL_LANGUAGE_VERSION)
        /* returns function pointer to be usable in SYCL kernel */
        m.def("rtcGetSYCLDeviceFunctionPointer", &rtcGetSYCLDeviceFunctionPointer);
    #endif
}