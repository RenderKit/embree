// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

  void buildProgress (size_t dn, void* userPtr) {
  }

  void splitPrimitive (const RTCBuildPrimitive& prim, unsigned dim, float pos, RTCBounds& lprim, RTCBounds& rprim, void* userPtr)
  {
    assert(dim < 3);
    assert(prim.geomID == 0);
    (BBox3fa&) lprim = (BBox3fa&) prim;
    (BBox3fa&) rprim = (BBox3fa&) prim;
    (&lprim.upper_x)[dim] = pos;
    (&rprim.lower_x)[dim] = pos;
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

    static void  setChildren (void* nodePtr, void** childPtr, size_t numChildren, void* userPtr)
    {
      assert(numChildren == 2);
      for (size_t i=0; i<2; i++)
        ((InnerNode*)nodePtr)->children[i] = (Node*) childPtr[i];
    }

    static void  setBounds (void* nodePtr, const RTCBounds** bounds, size_t numChildren, void* userPtr)
    {
      assert(numChildren == 2);
      for (size_t i=0; i<2; i++)
        ((InnerNode*)nodePtr)->bounds[i] = *(const BBox3fa*) bounds[i];
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

  void build(RTCBuildQuality quality, avector<RTCBuildPrimitive>& prims_i, char* cfg, size_t extraSpace = 0)
  {
    RTCDevice device = rtcNewDevice(cfg);
    rtcDeviceSetMemoryMonitorFunction2(device,memoryMonitor,nullptr);

    RTCBVH bvh = rtcNewBVH(device);

    /* settings for BVH build */
    RTCBuildSettings settings;
    settings.size = sizeof(settings);
    settings.quality = quality;
    settings.maxBranchingFactor = 2;
    settings.maxDepth = 1024;
    settings.sahBlockSize = 1;
    settings.minLeafSize = 1;
    settings.maxLeafSize = 1;
    settings.travCost = 1.0f;
    settings.intCost = 1.0f;
    settings.extraSpace = (unsigned int)extraSpace;

    avector<RTCBuildPrimitive> prims;
    prims.reserve(prims_i.size()+extraSpace);
    prims.resize(prims_i.size());

    for (size_t i=0; i<10; i++)
    {
      /* we recreate the prims array here, as the builders modify this array */
      for (size_t j=0; j<prims.size(); j++) prims[j] = prims_i[j];

      std::cout << "iteration " << i << ": building BVH over " << prims.size() << " primitives, " << std::flush;
      double t0 = getSeconds();
      Node* root = (Node*) rtcBuildBVH(bvh,settings,prims.data(),prims.size(),
                                       InnerNode::create,InnerNode::setChildren,InnerNode::setBounds,LeafNode::create,splitPrimitive,buildProgress,nullptr);
      double t1 = getSeconds();
      const float sah = root ? root->sah() : 0.0f;
      std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(prims.size())/(t1-t0) << " Mprims/s, sah = " << sah << " [DONE]" << std::endl;
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
    const size_t extraSpace = 1000000;
    avector<RTCBuildPrimitive> prims;
    prims.resize(N);
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
      prim.primID = (unsigned) i;
      prims[i] = prim;
    }

    std::cout << "Low quality BVH build:" << std::endl;
    build(RTC_BUILD_QUALITY_LOW,prims,cfg);

    std::cout << "Normal quality BVH build:" << std::endl;
    build(RTC_BUILD_QUALITY_NORMAL,prims,cfg);

    std::cout << "High quality BVH build:" << std::endl;
    build(RTC_BUILD_QUALITY_HIGH,prims,cfg,extraSpace);
  }

  /* task that renders a single screen tile */
  void renderTileStandard(int taskIndex, int threadIndex, int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera,
                          const int numTilesX,
                          const int numTilesY)
  {
  }

  /* task that renders a single screen tile */
  void renderTileTask(int taskIndex, int threadIndex, int* pixels,
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
