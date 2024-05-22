#pragma once 
#include "pyembree.h"
#include <mutex>

typedef std::function<void(const struct RTCFilterFunctionNArguments* args)> PyRTCFilterFunctionN;
typedef std::function<void(const struct RTCIntersectFunctionNArguments* args)> PyRTCIntersectFunctionN;

void pyembree_rtcfilter_functionn_callback(const struct RTCFilterFunctionNArguments* args);
void pyembree_rtcintersect_functionn_callback(const struct RTCIntersectFunctionNArguments* args);

struct PyRTCIntersectArguments
{
    enum RTCRayQueryFlags flags;            // intersection flags
    enum RTCFeatureFlags feature_mask;      // selectively enable features for traversal
    struct RTCRayQueryContext* context;     // optional pointer to ray query context
    PyRTCFilterFunctionN filter;            // filter function to execute
    PyRTCIntersectFunctionN intersect;      // interesect function to execute
#if RTC_MIN_WIDTH
    float minWidthDistanceFactor;           // curve radius is set to this factor times distance to ray origin
#endif

    RTCIntersectArguments get_args();
};




class FilterFunctionGuard {
    struct FilterFunctionCb {
        PyRTCFilterFunctionN callback;
        std::mutex mtx;

        FilterFunctionCb(PyRTCFilterFunctionN cb) : callback(cb) {}
    };

private:
    FilterFunctionGuard() = default;

public:
    ~FilterFunctionGuard() = default;
    FilterFunctionGuard(const FilterFunctionGuard&) = delete;
    FilterFunctionGuard& operator=(const FilterFunctionGuard&) = delete;
    FilterFunctionGuard(FilterFunctionGuard&&) = delete;
    FilterFunctionGuard& operator=(FilterFunctionGuard&&) = delete;

    static FilterFunctionGuard& get();

    void call(unsigned int geomID, const struct RTCFilterFunctionNArguments* args);

    std::unique_ptr<RTCIntersectArguments> begin_intersect(RTCScene scene, PyRTCIntersectArguments* args);
    void end_intersect();

    void assign(PyRTCFilterFunctionN callback, RTCGeometry geom);
    void clear(RTCGeometry geom);
    void clear_all();


private:
    std::unordered_map<void*, FilterFunctionCb*> callbacks;
    std::mutex mtx;
    RTCScene scene;
};