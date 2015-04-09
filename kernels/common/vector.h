// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "default.h"

namespace embree
{
  /*! invokes the memory monitor callback */
  void memoryMonitor(ssize_t bytes, bool post);

  /*! allocator that performs aligned monitored allocations */
  template<typename T, size_t alignment = 64>
    struct aligned_monitored_allocator
    {
      typedef T value_type;
      typedef T* pointer;
      typedef const T* const_pointer;
      typedef T& reference;
      typedef const T& const_reference;
      typedef std::size_t size_type; // FIXME: also use std::size_t type under windows if available
      typedef std::ptrdiff_t difference_type;

      __forceinline pointer allocate( size_type n ) 
      {
        memoryMonitor(n*sizeof(T),false);
        return (pointer) alignedMalloc(n*sizeof(value_type),alignment);
      }

      __forceinline void deallocate( pointer p, size_type n ) 
      {
        alignedFree(p);
        memoryMonitor(-n*sizeof(T),true);
      }

      __forceinline void construct( pointer p, const_reference val ) {
        new (p) T(val);
      }

      __forceinline void destroy( pointer p ) {
        p->~T();
      }
    };

  template<typename T, typename allocator = aligned_monitored_allocator<T,std::alignment_of<T>::value> >
    class mvector
    {
    public:

      mvector () 
        : size_active(0), size_alloced(0), items(nullptr) {}

      mvector (size_t sz) 
        : size_active(0), size_alloced(0), items(nullptr) { resize(sz); }

      ~mvector() {
        clear();
      }

      mvector (const mvector& other)
      {
        size_active = other.size_active;
        size_alloced = other.size_alloced;
        items = alloc.allocate(size_alloced);
        for (size_t i=0; i<size_active; i++) items[i] = other.items[i];
      }

      mvector& operator=(const mvector& other) 
      {
        resize(other.size_active);
        for (size_t i=0; i<size_active; i++) items[i] = other.items[i];
        return *this;
      }

      /********************** Iterators  ****************************/

      __forceinline T* begin() const { return items; };
      __forceinline T* end  () const { return items+size_active; };


      /********************** Capacity ****************************/

      __forceinline bool   empty    () const { return size_active == 0; }
      __forceinline size_t size     () const { return size_active; }
      __forceinline size_t capacity () const { return size_alloced; }
            
      void resize(size_t new_size) {
        internal_resize(new_size,size_alloced < new_size ? new_size : size_alloced);
      }

      void reserve(size_t new_alloced) 
      {
        /* do nothing if container already large enough */
        if (new_alloced <= size_alloced) 
          return;

        /* resize exact otherwise */
        internal_resize(size_active,new_alloced);
      }

      void shrink_to_fit() {
        internal_resize(size_active,size_active);
      }

      /******************** Element access **************************/

      __forceinline       T& operator[](size_t i)       { assert(i < size_active); return items[i]; }
      __forceinline const T& operator[](size_t i) const { assert(i < size_active); return items[i]; }

      __forceinline       T& at(size_t i)       { assert(i < size_active); return items[i]; }
      __forceinline const T& at(size_t i) const { assert(i < size_active); return items[i]; }

      __forceinline T& front() const { assert(size_active > 0); return items[0]; };
      __forceinline T& back () const { assert(size_active > 0); return items[size_active-1]; };

      __forceinline       T* data()       { return items; };
      __forceinline const T* data() const { return items; };

     
      /******************** Modifiers **************************/

      void push_back(const T& nt) 
      {
        const T v = nt; // need local copy as input reference could point to this vector
        internal_grow(size_active+1);
        items[size_active++] = v;
      }

      void pop_back() {
        size_active--;
      }

      void clear() 
      {
        /* destroy elements */
        for (size_t i=0; i<size_active; i++)
          alloc.destroy(&items[i]);
        
        /* free memory */
        alloc.deallocate(items,size_alloced); items = nullptr;
        size_active = size_alloced = 0;
      }

    private:

      void internal_resize(size_t new_active, size_t new_alloced)
      {
        assert(new_active <= new_alloced); 

        /* destroy elements */
        for (size_t i=new_active; i<size_active; i++)
          alloc.destroy(&items[i]);

        size_t size_copy = new_active < size_active ? new_active : size_active;
        size_active = new_active;

        /* only reallocate if necessary */
        if (new_alloced == size_alloced) 
          return;
        
        /* reallocate and copy items */
        T* old_items = items;
        items = alloc.allocate(new_alloced);
        for (size_t i=0; i<size_copy; i++) items[i] = old_items[i];
        alloc.deallocate(old_items,size_alloced);
        size_alloced = new_alloced;
      }

      void internal_grow(size_t new_alloced)
      {
        /* do nothing if container already large enough */
        if (new_alloced <= size_alloced) 
          return;

        /* resize to next power of 2 otherwise */
        size_t new_size_alloced = size_alloced;
        while (new_size_alloced < new_alloced) {
          new_size_alloced = 2*new_size_alloced;
          if (new_size_alloced == 0) new_size_alloced = 1;
        }

        internal_resize(size_active,new_size_alloced);
      }

    private:
      allocator alloc;
      size_t size_active;    // number of valid items
      size_t size_alloced;   // number of items allocated
      T* items;              // data array
    };

  /*! monitored vector */
  //template<typename T> // FIXME: unfortunately not supported in VS2012, have to replicate vector implementation
  //using mvector = vector_t<T,aligned_monitored_allocator<T,std::alignment_of<T>::value> >;
}
