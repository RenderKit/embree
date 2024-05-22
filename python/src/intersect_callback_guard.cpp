#pragma once 
#include "intersect_callback_guard.h"


void pyembree_rtcfilter_functionn_callback(const struct RTCFilterFunctionNArguments* args) {
    unsigned int geomID = RTCHitN_geomID(args->hit, args->N, 0);
    FilterFunctionGuard::get().call(geomID, args);
    FilterFunctionGuard::get().call(RTC_INVALID_GEOMETRY_ID, args);
}




RTCIntersectArguments PyRTCIntersectArguments::get_args() {
    RTCIntersectArguments a;
    a.flags = flags;
    a.feature_mask = feature_mask;
    a.context = context;
    a.filter = pyembree_rtcfilter_functionn_callback;
    a.intersect = nullptr;
    #if RTC_MIN_WIDTH
        a.minWidthDistanceFactor = minWidthDistanceFactor;
    #endif
    return a;
}




FilterFunctionGuard& FilterFunctionGuard::get() {
    static FilterFunctionGuard guard;
    return guard;
}

void FilterFunctionGuard::call(unsigned int geomID, const struct RTCFilterFunctionNArguments* args) {
    void* geom = (geomID == RTC_INVALID_GEOMETRY_ID) ? nullptr : (void*) rtcGetGeometry(this->scene, geomID);
    auto it = this->callbacks.find(geom);
    if (it != this->callbacks.end()) {
        auto& cb = this->callbacks[geom];
        std::lock_guard<std::mutex> guard(cb->mtx);
        cb->callback(args);
    }
}

std::unique_ptr<RTCIntersectArguments> FilterFunctionGuard::begin_intersect(RTCScene scene, PyRTCIntersectArguments* args) {
    this->mtx.lock();
    this->scene = scene;

    if (args) {
        FilterFunctionGuard::get().assign(args->filter, nullptr);
        return std::make_unique<RTCIntersectArguments>(args->get_args());
    }
    return nullptr;
}

void FilterFunctionGuard::end_intersect() {
    this->scene = nullptr;
    FilterFunctionGuard::get().clear(nullptr);
    this->mtx.unlock();
}

void FilterFunctionGuard::assign(PyRTCFilterFunctionN callback, RTCGeometry geom) {
    this->callbacks[geom] = new FilterFunctionCb(callback);
}

void FilterFunctionGuard::clear(RTCGeometry geom) {
    delete this->callbacks[(void*) geom];
    this->callbacks.erase((void*) geom);
}

void FilterFunctionGuard::clear_all() {
    for (auto cb = this->callbacks.begin(); cb != this->callbacks.end(); ++cb) {
        delete cb->second;
    }
    this->callbacks.clear();
}

