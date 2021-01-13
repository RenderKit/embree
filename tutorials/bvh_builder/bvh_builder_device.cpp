// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree
{
  RTCScene g_scene  = nullptr;

  /* This function is called by the builder to signal progress and to
   * report memory consumption. */
  bool memoryMonitor(void* userPtr, ssize_t bytes, bool post) {
    return true;
  }

  bool buildProgress (void* userPtr, double f) {
    return true;
  }

  void splitPrimitive (const RTCBuildPrimitive* prim, unsigned int dim, float pos, RTCBounds* lprim, RTCBounds* rprim, void* userPtr)
  {
    assert(dim < 3);
    assert(prim->geomID == 0);
    *(BBox3fa*) lprim = *(BBox3fa*) prim;
    *(BBox3fa*) rprim = *(BBox3fa*) prim;
    (&lprim->upper_x)[dim] = pos;
    (&rprim->lower_x)[dim] = pos;
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

    static void* create (RTCThreadLocalAllocator alloc, unsigned int numChildren, void* userPtr)
    {
      assert(numChildren == 2);
      void* ptr = rtcThreadLocalAlloc(alloc,sizeof(InnerNode),16);
      return (void*) new (ptr) InnerNode;
    }

    static void  setChildren (void* nodePtr, void** childPtr, unsigned int numChildren, void* userPtr)
    {
      assert(numChildren == 2);
      for (size_t i=0; i<2; i++)
        ((InnerNode*)nodePtr)->children[i] = (Node*) childPtr[i];
    }

    static void  setBounds (void* nodePtr, const RTCBounds** bounds, unsigned int numChildren, void* userPtr)
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
    rtcSetDeviceMemoryMonitorFunction(g_device,memoryMonitor,nullptr);

    RTCBVH bvh = rtcNewBVH(g_device);

    avector<RTCBuildPrimitive> prims;
    prims.reserve(prims_i.size()+extraSpace);
    prims.resize(prims_i.size());

    /* settings for BVH build */
    RTCBuildArguments arguments = rtcDefaultBuildArguments();
    arguments.byteSize = sizeof(arguments);
    arguments.buildFlags = RTC_BUILD_FLAG_DYNAMIC;
    arguments.buildQuality = quality;
    arguments.maxBranchingFactor = 2;
    arguments.maxDepth = 1024;
    arguments.sahBlockSize = 1;
    arguments.minLeafSize = 1;
    arguments.maxLeafSize = 1;
    arguments.traversalCost = 1.0f;
    arguments.intersectionCost = 1.0f;
    arguments.bvh = bvh;
    arguments.primitives = prims.data();
    arguments.primitiveCount = prims.size();
    arguments.primitiveArrayCapacity = prims.capacity();
    arguments.createNode = InnerNode::create;
    arguments.setNodeChildren = InnerNode::setChildren;
    arguments.setNodeBounds = InnerNode::setBounds;
    arguments.createLeaf = LeafNode::create;
    arguments.splitPrimitive = splitPrimitive;
    arguments.buildProgress = buildProgress;
    arguments.userPtr = nullptr;
    
    for (size_t i=0; i<10; i++)
    {
      /* we recreate the prims array here, as the builders modify this array */
      for (size_t j=0; j<prims.size(); j++) prims[j] = prims_i[j];

      std::cout << "iteration " << i << ": building BVH over " << prims.size() << " primitives, " << std::flush;
      double t0 = getSeconds();
      Node* root = (Node*) rtcBuildBVH(&arguments);
      double t1 = getSeconds();
      const float sah = root ? root->sah() : 0.0f;
      std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(prims.size())/(t1-t0) << " Mprims/s, sah = " << sah << " [DONE]" << std::endl;
    }

    rtcReleaseBVH(bvh);
  }

  /* called by the C++ code for initialization */
  extern "C" void device_init (char* cfg)
  {
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
    build(RTC_BUILD_QUALITY_MEDIUM,prims,cfg);

    std::cout << "High quality BVH build:" << std::endl;
    build(RTC_BUILD_QUALITY_HIGH,prims,cfg,extraSpace);
  }

  void renderFrameStandard (int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
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
  }
}
