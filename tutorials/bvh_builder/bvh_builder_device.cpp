// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "../common/tutorial/tutorial_device.h"
#include "../../kernels/common/alloc.h"
#include "../../kernels/builders/bvh_builder_sah.h"
#include "../../kernels/builders/bvh_builder_morton.h"

namespace embree
{
  RTCDevice g_device = nullptr;
  RTCScene g_scene  = nullptr;

  /* This function is called by the builder to signal progress and to
   * report memory consumption. */
  bool memoryMonitor(void* userPtr, ssize_t bytes, bool post) {
    return true;
  }
  
  struct Node
  {
    virtual float sah() = 0;
  };
  
  struct InnerNode : public Node
  {
    BBox3fa bounds[2];
    Node* children[2];
    
    InnerNode() {
      bounds[0] = bounds[1] = empty;
      children[0] = children[1] = nullptr;
    }
    
    float sah() {
      return 1.0f + (area(bounds[0])*children[0]->sah() + area(bounds[1])*children[1]->sah())/area(merge(bounds[0],bounds[1]));
    }

    static void* create (void* threadLocalPtr, size_t numChildren) 
    {
      assert(numChildren == 2);
      RTCThreadLocalAllocator alloc = (RTCThreadLocalAllocator) threadLocalPtr;
      void* ptr = rtcThreadLocalMalloc(alloc,sizeof(InnerNode),16);
      return (void*) new (ptr) InnerNode;
    }
    
    static void  setChild (void* nodePtr, size_t i, void* childPtr)
    {
      assert(i<2);
      ((InnerNode*)nodePtr)->children[i] = (Node*) childPtr;
    }
    
    static void  setBounds (void* nodePtr, size_t i, RTCBounds& bounds)
    {
      ((InnerNode*)nodePtr)->bounds[i] = (BBox3fa&) bounds;
    }
  };
  
  struct LeafNode : public Node
  {
    unsigned id;
    BBox3fa bounds;
    
    LeafNode (unsigned id, const BBox3fa& bounds)
      : id(id), bounds(bounds) {}
    
    float sah() {
      return 1.0f;
    }
    
    static void* create (void* threadLocalPtr, const RTCBuildPrimitive* prims, size_t numPrims)
    {
      assert(numPrims == 1);
      RTCThreadLocalAllocator alloc = (RTCThreadLocalAllocator) threadLocalPtr;
      void* ptr = rtcThreadLocalMalloc(alloc,sizeof(LeafNode),16);
      return (void*) new (ptr) LeafNode(prims->primID,*(BBox3fa*)prims);
    }
  };

#if 0  // FIXME: remove
  void build_sah(PrimRef* prims, size_t N)
  {
    /* fast allocator that supports thread local operation */
    FastAllocator allocator(nullptr,false);
    
    for (size_t i=0; i<2; i++)
    {
      std::cout << "iteration " << i << ": building BVH over " << N << " primitives, " << std::flush;
      double t0 = getSeconds();
      
      allocator.reset();
      allocator.init(N);

      /* calculate priminfo */
      auto computeBounds = [&](const range<size_t>& r) -> isa::CentGeomBBox3fa
        {
          isa::CentGeomBBox3fa bounds(empty);
          for (size_t j=r.begin(); j<r.end(); j++)
            bounds.extend(prims[j].bounds());
          return bounds;
        };
      const isa::CentGeomBBox3fa bounds = 
        parallel_reduce(size_t(0),N,size_t(1024),size_t(1024),isa::CentGeomBBox3fa(empty), computeBounds, isa::CentGeomBBox3fa::merge2);
      
      const isa::PrimInfo pinfo(0,N,bounds.geomBounds,bounds.centBounds);

       /* settings for BVH build */
      isa::GeneralBVHBuilder::Settings settings;
      settings.branchingFactor = 2;
      settings.maxDepth = 1024;
      settings.logBlockSize = 0;
      settings.minLeafSize = 1;
      settings.maxLeafSize = 1;
      settings.travCost = 1.0f;
      settings.intCost = 1.0f;
      settings.singleThreadThreshold = Builder::DEFAULT_SINGLE_THREAD_THRESHOLD;
      
      Node* root = isa::BVHBuilderBinnedSAH::build<Node*>(

        /* thread local allocator for fast allocations */
        [&] () -> FastAllocator::ThreadLocal* { 
          return allocator.threadLocal(); 
        },

        /* lambda function that creates BVH nodes */
        [&](isa::BVHBuilderBinnedSAH::BuildRecord* children, const size_t N, FastAllocator::ThreadLocal* alloc) -> Node*
        {
          assert(N <= 2);
          InnerNode* node = new (alloc->malloc(sizeof(InnerNode))) InnerNode;
          for (size_t i=0; i<N; i++)
            node->bounds[i] = children[i].prims.geomBounds;
          return node;
        },

        /* lambda function that updates BVH nodes */
        [&](Node* ref, Node** children, const size_t N) -> Node*
        {
          assert(N <= 2);
          InnerNode* node = (InnerNode*) ref;
          for (size_t i=0; i<N; i++)
            node->children[i] = children[i];
          return ref;
        },
        
        /* lambda function that creates BVH leaves */
        [&](const isa::BVHBuilderBinnedSAH::BuildRecord& current, FastAllocator::ThreadLocal* alloc) -> Node*
        {
          assert(current.prims.size() == 1);
          Node* node = new (alloc->malloc(sizeof(LeafNode))) LeafNode(prims[current.prims.begin()].primID(),prims[current.prims.begin()].bounds());
          return node;
        },
        
        /* progress monitor function */
        [&] (size_t dn) { 
          // throw an exception here to cancel the build operation
        },
        
        prims,pinfo,settings);
      
      double t1 = getSeconds();
      
      std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(N)/(t1-t0) << " Mprims/s, sah = " << root->sah() << " [DONE]" << std::endl;
    }
  }
#endif

  void* createThreadLocal (void* userPtr) {
    return (void*) rtcGetThreadLocalAllocator((RTCAllocator)userPtr);
  }

  void buildProgress (void* userPtr, size_t dn) {
  }

  void build_sah_api(RTCBuildPrimitive* prims, size_t N, char* cfg)
  {
    RTCDevice device = rtcNewDevice(cfg);
    rtcDeviceSetMemoryMonitorFunction2(device,memoryMonitor,nullptr);

    RTCAllocator allocator = rtcNewAllocator(device,N);

    for (size_t i=0; i<2; i++)
    {
      std::cout << "iteration " << i << ": building BVH over " << N << " primitives, " << std::flush;
      double t0 = getSeconds();
      
      rtcResetAllocator(allocator);

      /* settings for BVH build */
      RTCBuildSettings settings;
      settings.size = sizeof(settings);
      settings.branchingFactor = 2;
      settings.maxDepth = 1024;
      settings.blockSize = 1;
      settings.minLeafSize = 1;
      settings.maxLeafSize = 1;
      settings.travCost = 1.0f;
      settings.intCost = 1.0f;
      
      Node* root = (Node*) rtcBVHBuildSAH(device,settings,
                                          prims,N,(void*)allocator,
                                          createThreadLocal,InnerNode::create,InnerNode::setChild,InnerNode::setBounds,LeafNode::create,buildProgress);
      
      double t1 = getSeconds();
      
      std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(N)/(t1-t0) << " Mprims/s, sah = " << root->sah() << " [DONE]" << std::endl;
    }
  }

#if 0 // FIXME: remove  
  void build_morton(PrimRef* prims, size_t N)
  {
    /* array for morton builder */
    avector<isa::MortonID32Bit> morton_src(N);
    avector<isa::MortonID32Bit> morton_tmp(N);
    for (unsigned i=0; i<N; i++) 
      morton_src[i].index = i;
    
    /* fast allocator that supports thread local operation */
    FastAllocator allocator(nullptr,true);
    
    for (size_t i=0; i<2; i++)
    {
      std::cout << "iteration " << i << ": building BVH over " << N << " primitives, " << std::flush;
      double t0 = getSeconds();
      
      allocator.reset();
      allocator.init(16*N);
      
      std::pair<Node*,BBox3fa> node_bounds = isa::bvh_builder_morton<std::pair<Node*,BBox3fa>>(
        
        /* thread local allocator for fast allocations */
        [&] () -> FastAllocator::ThreadLocal* { 
          return allocator.threadLocal(); 
        },
        
        /* lambda function that allocates BVH nodes */
        [&] ( FastAllocator::ThreadLocal* alloc, size_t N ) -> Node*
        {
          assert(N <= 2);
          InnerNode* node = new (alloc->malloc(sizeof(InnerNode))) InnerNode;
          return node;
        },
        
        /* lambda function that sets bounds */
        [&] (Node* ref, const std::pair<Node*,BBox3fa>* children, size_t N) -> std::pair<Node*,BBox3fa>
        {
          InnerNode* node = (InnerNode*) ref;
          BBox3fa bounds = empty;
          for (size_t i=0; i<N; i++) {
            node->children[i] = children[i].first;
            node->bounds[i] = children[i].second;
            bounds.extend(children[i].second);
          }
          return std::make_pair(ref,bounds);
        },
        
        /* lambda function that creates BVH leaves */
        [&]( isa::MortonBuildRecord& current, FastAllocator::ThreadLocal* alloc) -> std::pair<Node*,BBox3fa>
        {
          assert(current.size() == 1);
          const size_t id = morton_src[current.begin].index;
          const BBox3fa bounds = prims[id].bounds(); 
          Node* node = new (alloc->malloc(sizeof(LeafNode))) LeafNode(id,bounds);
          return std::make_pair(node,bounds);
        },
        
        /* lambda that calculates the bounds for some primitive */
        [&] (const isa::MortonID32Bit& morton) -> BBox3fa {
          return prims[morton.index].bounds();
        },
        
        /* progress monitor function */
        [&] (size_t dn) { 
          // throw an exception here to cancel the build operation
        },
        
        morton_src.data(),morton_tmp.data(),N,2,1024,1,1,Builder::DEFAULT_SINGLE_THREAD_THRESHOLD);
      
      Node* root = node_bounds.first;
      
      double t1 = getSeconds();
      
      std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(N)/(t1-t0) << " Mprims/s, sah = " << root->sah() << " [DONE]" << std::endl;
    }
  }
#endif

  void build_morton_api(RTCBuildPrimitive* prims, size_t N, char* cfg)
  {
    RTCDevice device = rtcNewDevice(cfg);
    rtcDeviceSetMemoryMonitorFunction2(device,memoryMonitor,nullptr);

    RTCAllocator allocator = rtcNewAllocator(device,16*N);

    for (size_t i=0; i<2; i++)
    {
      std::cout << "iteration " << i << ": building BVH over " << N << " primitives, " << std::flush;
      double t0 = getSeconds();
      
      rtcResetAllocator(allocator);

       /* settings for BVH build */
      RTCBuildSettings settings;
      settings.size = sizeof(settings);
      settings.branchingFactor = 2;
      settings.maxDepth = 1024;
      settings.blockSize = 1;
      settings.minLeafSize = 1;
      settings.maxLeafSize = 1;
      settings.travCost = 1.0f;
      settings.intCost = 1.0f;
      
      Node* root = (Node*) rtcBVHBuildMorton(device,settings,
                                             prims,N,(void*)allocator,
                                             createThreadLocal,InnerNode::create,InnerNode::setChild,InnerNode::setBounds,LeafNode::create,buildProgress);
      
      double t1 = getSeconds();
      
      std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(N)/(t1-t0) << " Mprims/s, sah = " << root->sah() << " [DONE]" << std::endl;
    }
  }
  
  /* called by the C++ code for initialization */
  extern "C" void device_init (char* cfg)
  {
    /* create new Embree device */
    g_device = rtcNewDevice(cfg);
    error_handler(nullptr,rtcDeviceGetError(g_device));
    
    /* set error handler */
    rtcDeviceSetErrorFunction2(g_device,error_handler,nullptr);
    
    /* set start render mode */
    renderTile = renderTileStandard;
    
    /* create random bounding boxes */
    const size_t N = 2300000;
    avector<RTCBuildPrimitive> prims; 
    for (size_t i=0; i<N; i++) 
    {
      const float x = float(drand48());
      const float y = float(drand48());
      const float z = float(drand48());
      const Vec3fa p = 1000.0f*Vec3fa(x,y,z);
      const BBox3fa b = BBox3fa(p,p+Vec3fa(1.0f));

      RTCBuildPrimitive prim;
      prim.lower_x = b.lower.x;
      prim.lower_y = b.lower.y;
      prim.lower_z = b.lower.z;
      prim.geomID = 0;
      prim.upper_x = b.upper.x;
      prim.upper_y = b.upper.y;
      prim.upper_z = b.upper.z;
      prim.primID = i;
      prims.push_back(prim);
    }

    std::cout << "SAH builder:" << std::endl;
    build_sah_api(prims.data(),prims.size(),cfg);
    //build_sah((PrimRef*)prims.data(),prims.size());
    
    std::cout << "Morton builder:" << std::endl;
    build_morton_api(prims.data(),prims.size(),cfg);
    //build_morton((PrimRef*)prims.data(),prims.size());
  }
  
  /* task that renders a single screen tile */
  void renderTileStandard(int taskIndex, int* pixels,
                          const unsigned int width,
                          const unsigned int height, 
                          const float time,
                          const ISPCCamera& camera,
                          const int numTilesX, 
                          const int numTilesY)
  {
  }
  
  /* task that renders a single screen tile */
  void renderTileTask(int taskIndex, int* pixels,
                      const unsigned int width,
                      const unsigned int height, 
                      const float time,
                      const ISPCCamera& camera,
                      const int numTilesX, 
                      const int numTilesY)
  {
  }
  
  /* called by the C++ code to render */
  extern "C" void device_render (int* pixels,
                                 const int width,
                                 const int height,
                                 const float time,
                                 const ISPCCamera& camera)
  {
  }
  
  /* called by the C++ code for cleanup */
  extern "C" void device_cleanup () {
    rtcDeleteDevice(g_device);
  } 
}
