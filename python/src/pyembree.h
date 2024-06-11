#pragma once
#define PYBIND11_DETAILED_ERROR_MESSAGES
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>

#if defined(EMBREE_SYCL_SUPPORT)
#include "sycl/sycl.hpp"
#endif
#include <embree4/rtcore.h>
#include <cstdint>


struct RTCDeviceWrapper { 
    RTCDevice d; 
#if defined(EMBREE_SYCL_SUPPORT)
    sycl::device* sycl_device = nullptr;
    sycl::queue* sycl_queue = nullptr;
    sycl::context* sycl_context = nullptr;
#endif
};
struct RTCSceneWrapper { RTCScene s; };
struct RTCGeometryWrapper { RTCGeometry g; };

struct RTCPointQueryNWrapper { RTCPointQueryN *p; };

struct RTCBufferWrapper { RTCBuffer b; };

struct RTCRayNWrapper { RTCRayN *p; };
struct RTCHitNWrapper { RTCHitN *p; };
struct RTCRayHitNWrapper { RTCRayHitN *p; }; 

struct RTCBVHWrapper { RTCBVH p; };
struct RTCThreadLocalAllocatorWrapper { RTCThreadLocalAllocator p; };


void bind_rtcore_common(pybind11::module &m);
void bind_rtcore_device(pybind11::module &m);
void bind_rtcore_buffer(pybind11::module &m);
void bind_rtcore_ray(pybind11::module &m);
void bind_rtcore_geometry(pybind11::module &m);
void bind_rtcore_scene(pybind11::module &m);
void bind_rtcore_builder(pybind11::module &m);
void bind_rtcore_quaternion(pybind11::module &m);

#define STRINGIFY(x) #x

#define BIND_PROPERTY_ARRAY(classn, name, type, count)                                      \
    .def_property(#name, [](classn &c)->pybind11::array {                                   \
            auto dtype = pybind11::dtype(pybind11::format_descriptor<type>::format());      \
            auto base = pybind11::array(dtype, {count}, {sizeof(type)});                    \
            return pybind11::array(dtype, {count}, {sizeof(type)}, c.name, base);           \
        }, [](classn &h) {}                                                                 \
    )
    
#define BIND_PROPERTY_ARRAY_2D(classn, name, type, count, count2)                                   \
    .def_property(#name, [](classn &c)->pybind11::array {                                \
            auto dtype = pybind11::dtype(pybind11::format_descriptor<type>::format());              \
            return pybind11::array(dtype, { count, count2 }, { sizeof(float) }, c.name, nullptr);   \
        }, [](classn &h) {}                                                                         \
    )



template <class T> class EmbreePtr {
    public:
        EmbreePtr() : ptr(nullptr) {}
        EmbreePtr(T* p) : ptr(p) {}
        EmbreePtr(const EmbreePtr& other) = default;
        EmbreePtr& operator=(const EmbreePtr& rhs) = default;
        EmbreePtr(EmbreePtr&& other) = default;
        EmbreePtr& operator=(EmbreePtr&& rhs) = default;

        T& operator* () const { return *ptr; }
        T* operator->() const { return ptr; }
        T* get_raw() const { return ptr; }
        T& operator[](size_t i) const { return ptr[i]; }
    private:
        T* ptr;
};

template<class T> void bind_embree_ptr(pybind11::module &m, const char* name) {
    py::class_<EmbreePtr<T>>(m, name)
        .def("__setitem__", [](EmbreePtr<T> &self, unsigned index, T val) { self[index] = val; })
        .def("__getitem__", [](EmbreePtr<T> &self, unsigned index) { return self[index]; });
}

class EmbreeVoidPtr {
    public:
        EmbreeVoidPtr() : ptr(nullptr) {}
        EmbreeVoidPtr(void* p) : ptr(p) {}
        EmbreeVoidPtr(const EmbreeVoidPtr& other) = default;
        EmbreeVoidPtr& operator=(const EmbreeVoidPtr& rhs) = default;
        EmbreeVoidPtr(EmbreeVoidPtr&& other) = default;
        EmbreeVoidPtr& operator=(EmbreeVoidPtr&& rhs) = default;

        void*                get_raw()  { return ptr; }
        EmbreePtr<float>     as_float() { return EmbreePtr<float>((float*)ptr); }
        EmbreePtr<uint32_t>  as_uint()  { return EmbreePtr<uint32_t>((uint32_t*)ptr); }
        EmbreePtr<int32_t>   as_int()   { return EmbreePtr<int32_t>((int32_t*)ptr); }
    private:
        void* ptr;
};