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

    static void* create (RTCThreadLocalAllocator alloc, size_t numChildren, void* userPtr) 
    {
      assert(numChildren == 2);
      void* ptr = rtcThreadLocalAlloc(alloc,sizeof(InnerNode),16);
      return (void*) new (ptr) InnerNode;
    }
    
    static void  setChild (void* nodePtr, size_t i, void* childPtr, void* userPtr)
    {
      assert(i<2);
      ((InnerNode*)nodePtr)->children[i] = (Node*) childPtr;
    }
    
    static void  setBounds (void* nodePtr, size_t i, RTCBounds& bounds, void* userPtr)
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
    
    static void* create (RTCThreadLocalAllocator alloc, const RTCBuildPrimitive* prims, size_t numPrims, void* userPtr)
    {
      assert(numPrims == 1);
      void* ptr = rtcThreadLocalAlloc(alloc,sizeof(LeafNode),16);
      return (void*) new (ptr) LeafNode(prims->primID,*(BBox3fa*)prims);
    }
  };

  void buildProgress (size_t dn, void* userPtr) {
  }

  void build_sah(RTCBuildPrimitive* prims, size_t N, char* cfg)
  {
    RTCDevice device = rtcNewDevice(cfg);
    rtcDeviceSetMemoryMonitorFunction2(device,memoryMonitor,nullptr);

    RTCBVH bvh = rtcNewBVH(device);

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
    
    for (size_t i=0; i<10; i++)
    {
      std::cout << "iteration " << i << ": building BVH over " << N << " primitives, " << std::flush;
      double t0 = getSeconds();
      Node* root = (Node*) rtcBVHBuildSAH(bvh,settings,prims,N,nullptr,
                                          InnerNode::create,InnerNode::setChild,InnerNode::setBounds,LeafNode::create,buildProgress);
      double t1 = getSeconds();
      
      std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(N)/(t1-t0) << " Mprims/s, sah = " << root->sah() << " [DONE]" << std::endl;
    }
    rtcMakeStaticBVH(bvh);
    rtcDeleteBVH(bvh);
  }

  void build_morton(RTCBuildPrimitive* prims, size_t N, char* cfg)
  {
    RTCDevice device = rtcNewDevice(cfg);
    rtcDeviceSetMemoryMonitorFunction2(device,memoryMonitor,nullptr);

    RTCBVH bvh = rtcNewBVH(device);

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
    
    for (size_t i=0; i<10; i++)
    {
      std::cout << "iteration " << i << ": building BVH over " << N << " primitives, " << std::flush;
      double t0 = getSeconds();
      Node* root = (Node*) rtcBVHBuildMorton(bvh,settings,prims,N,nullptr,
                                             InnerNode::create,InnerNode::setChild,InnerNode::setBounds,LeafNode::create,buildProgress);
      double t1 = getSeconds();
      
      std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(N)/(t1-t0) << " Mprims/s, sah = " << root->sah() << " [DONE]" << std::endl;
    }
    rtcMakeStaticBVH(bvh);
    rtcDeleteBVH(bvh);
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
    build_sah(prims.data(),prims.size(),cfg);

    std::cout << "Morton builder:" << std::endl;
    build_morton(prims.data(),prims.size(),cfg);
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
