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

#include "../common/tutorial/tutorial_device.h"
#include "../../kernels/common/alloc.h"
#include "../../kernels/xeon/builders/bvh_builder_sah.h"
#include "../../kernels/xeon/builders/bvh_builder_morton.h"

RTCDevice g_device = nullptr;
RTCScene g_scene  = nullptr;

/* render function to use */
renderPixelFunc renderPixel;

/* error reporting function */
void error_handler(const RTCError code, const char* str = NULL)
{
  if (code == RTC_NO_ERROR) 
    return;

  printf("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : printf("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : printf("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: printf("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : printf("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : printf("RTC_UNSUPPORTED_CPU"); break;
  case RTC_CANCELLED        : printf("RTC_CANCELLED"); break;
  default                   : printf("invalid error code"); break;
  }
  if (str) { 
    printf(" ("); 
    while (*str) putchar(*str++); 
    printf(")\n"); 
  }
  exit(1);
}

/* These function called by the builder to signal progress and to
 * report memory consumption. */
namespace embree
{
  void memoryMonitor(ssize_t bytes, bool post)
  {
    // throw an exception here when nprims>0 to cancel the build operation
  }
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
};

struct LeafNode : public Node
{
  size_t id;
  BBox3fa bounds;

  LeafNode (size_t id, const BBox3fa& bounds)
    : id(id), bounds(bounds) {}

  float sah() {
    return 1.0f;
  }
};

void build_sah(avector<PrimRef>& prims, isa::PrimInfo& pinfo)
{
  size_t N = pinfo.size();

  /* fast allocator that supports thread local operation */
  FastAllocator allocator(nullptr);

  for (size_t i=0; i<2; i++)
  {
    std::cout << "iteration " << i << ": building BVH over " << N << " primitives, " << std::flush;
    double t0 = getSeconds();
    
    allocator.reset();

    Node* root;
    isa::BVHBuilderBinnedSAH::build<Node*>(
      root,
      /* thread local allocator for fast allocations */
      [&] () -> FastAllocator::ThreadLocal* { 
        return allocator.threadLocal(); 
      },

      /* lambda function that creates BVH nodes */
      [&](const isa::BVHBuilderBinnedSAH::BuildRecord& current, isa::BVHBuilderBinnedSAH::BuildRecord* children, const size_t N, FastAllocator::ThreadLocal* alloc) -> int
      {
        assert(N <= 2);
        InnerNode* node = new (alloc->malloc(sizeof(InnerNode))) InnerNode;
        for (size_t i=0; i<N; i++) {
          node->bounds[i] = children[i].pinfo.geomBounds;
          children[i].parent = (size_t*) &node->children[i];
        }
        *current.parent = (size_t) node;
	return 0;
      },

      /* lambda function that creates BVH leaves */
      [&](const isa::BVHBuilderBinnedSAH::BuildRecord& current, FastAllocator::ThreadLocal* alloc) -> int
      {
        assert(current.prims.size() == 1);
        Node* node = new (alloc->malloc(sizeof(LeafNode))) LeafNode(prims[current.prims.begin()].ID(),prims[current.prims.begin()].bounds());
        *current.parent = (size_t) node;
	return 0;
      },

      /* progress monitor function */
      [&] (size_t dn) { 
        // throw an exception here to cancel the build operation
      },

      prims.data(),pinfo,2,1024,1,1,1,1.0f,1.0f);
    
    double t1 = getSeconds();

    std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(N)/(t1-t0) << " Mprims/s, sah = " << root->sah() << " [DONE]" << std::endl;
  }
}

void build_morton(avector<PrimRef>& prims, isa::PrimInfo& pinfo)
{
  size_t N = pinfo.size();
  /* array for morton builder */
  avector<isa::MortonID32Bit> morton_src(N);
  avector<isa::MortonID32Bit> morton_tmp(N);
  for (size_t i=0; i<N; i++) 
    morton_src[i].index = i;

  /* fast allocator that supports thread local operation */
  FastAllocator allocator(nullptr);

  for (size_t i=0; i<2; i++)
  {
    std::cout << "iteration " << i << ": building BVH over " << N << " primitives, " << std::flush;
    double t0 = getSeconds();
    
    allocator.reset();

    std::pair<Node*,BBox3fa> node_bounds = isa::bvh_builder_morton<Node*>(

      /* thread local allocator for fast allocations */
      [&] () -> FastAllocator::ThreadLocal* { 
        return allocator.threadLocal(); 
      },

      BBox3fa(empty),

      /* lambda function that allocates BVH nodes */
      [&] ( isa::MortonBuildRecord<Node*>& current, isa::MortonBuildRecord<Node*>* children, size_t N, FastAllocator::ThreadLocal* alloc ) -> InnerNode*
      {
        assert(N <= 2);
        InnerNode* node = new (alloc->malloc(sizeof(InnerNode))) InnerNode;
        *current.parent = node;
        for (size_t i=0; i<N; i++) 
          children[i].parent = &node->children[i];
        return node;
      },

      /* lambda function that sets bounds */
      [&] (InnerNode* node, const BBox3fa* bounds, size_t N) -> BBox3fa
      {
        BBox3fa res = empty;
        for (size_t i=0; i<N; i++) {
          const BBox3fa b = bounds[i];
          res.extend(b);
          node->bounds[i] = b;
        }
        return res;
      },

      /* lambda function that creates BVH leaves */
      [&]( isa::MortonBuildRecord<Node*>& current, FastAllocator::ThreadLocal* alloc, BBox3fa& box_o) -> Node*
      {
        assert(current.size() == 1);
        const size_t id = morton_src[current.begin].index;
        const BBox3fa bounds = prims[id].bounds(); 
        Node* node = new (alloc->malloc(sizeof(LeafNode))) LeafNode(id,bounds);
        *current.parent = node;
        box_o = bounds;
        return node;
      },

      /* lambda that calculates the bounds for some primitive */
      [&] (const isa::MortonID32Bit& morton) -> BBox3fa {
        return prims[morton.index].bounds();
      },

      /* progress monitor function */
      [&] (size_t dn) { 
        // throw an exception here to cancel the build operation
      },

      morton_src.data(),morton_tmp.data(),prims.size(),2,1024,1,1);

    Node* root = node_bounds.first;
    
    double t1 = getSeconds();

    std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(N)/(t1-t0) << " Mprims/s, sah = " << root->sah() << " [DONE]" << std::endl;
  }
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);
  
  /* set start render mode */
  renderPixel = renderPixelStandard;

  /* create random bounding boxes */
  const size_t N = 2300000;
  isa::PrimInfo pinfo(empty);
  avector<PrimRef> prims; 
  for (size_t i=0; i<N; i++) {
    const Vec3fa p = 1000.0f*Vec3fa(drand48(),drand48(),drand48());
    const BBox3fa b = BBox3fa(p,p+Vec3fa(1.0f));
    pinfo.add(b);
    const PrimRef prim = PrimRef(b,i);
    prims.push_back(prim);
  }

  build_sah(prims,pinfo);
  build_morton(prims,pinfo);
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  return Vec3fa(zero);
}

/* task that renders a single screen tile */
void renderTile(int taskIndex, int* pixels,
                     const int width,
                     const int height, 
                     const float time,
                     const Vec3fa& vx, 
                     const Vec3fa& vy, 
                     const Vec3fa& vz, 
                     const Vec3fa& p,
                     const int numTilesX, 
                     const int numTilesY)
{
  const int tileY = taskIndex / numTilesX;
  const int tileX = taskIndex - tileY * numTilesX;
  const int x0 = tileX * TILE_SIZE_X;
  const int x1 = min(x0+TILE_SIZE_X,width);
  const int y0 = tileY * TILE_SIZE_Y;
  const int y1 = min(y0+TILE_SIZE_Y,height);

  for (int y = y0; y<y1; y++) for (int x = x0; x<x1; x++)
  {
    /* calculate pixel color */
    Vec3fa color = renderPixel(x,y,vx,vy,vz,p);
    
    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                    const int width,
                    const int height,
                    const float time,
                    const Vec3fa& vx, 
                    const Vec3fa& vy, 
                    const Vec3fa& vz, 
                    const Vec3fa& p)
{
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch_renderTile(numTilesX*numTilesY,pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); 
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup () {
  rtcDeleteDevice(g_device);
}

