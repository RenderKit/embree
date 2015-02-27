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
#include "kernels/common/alloc.h"
#include "kernels/xeon/builders_new/bvh_builder.h"
#include "kernels/xeon/builders_new/bvh_builder_morton.h"
#include "kernels/xeon/builders/priminfo.h"

/* scene data */
RTCScene g_scene = NULL;
Vec3fa* colors = NULL;

/* render function to use */
renderPixelFunc renderPixel;

/* error reporting function */
void error_handler(const RTCError code, const int8* str)
{
  printf("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : printf("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : printf("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: printf("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : printf("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : printf("RTC_UNSUPPORTED_CPU"); break;
  default                   : printf("invalid error code"); break;
  }
  if (str) { 
    printf(" ("); 
    while (*str) putchar(*str++); 
    printf(")\n"); 
  }
  abort();
}

/* rtcCommitThread called by all ISPC worker threads to enable parallel build */
#if defined(PARALLEL_COMMIT)
task void parallelCommit(RTCScene scene) {
  rtcCommitThread (scene,threadIndex,threadCount); 
}
#endif

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
    children[0] = children[1] = NULL;
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

void build_sah(std::vector<PrimRef>& prims, isa::PrimInfo& pinfo)
{
  size_t N = pinfo.size();

  /* fast allocator that supports thread local operation */
  FastAllocator allocator;

  for (size_t i=0; i<2; i++)
  {
    std::cout << "iteration " << i << ": building BVH over " << N << " primitives, " << std::flush;
    double t0 = getSeconds();
    
    allocator.reset();

    Node* root = isa::bvh_builder_binned_sah<Node*>(

      /* thread local allocator for fast allocations */
      [&] () -> FastAllocator::ThreadLocal* { 
        return allocator.threadLocal(); 
      },

      /* lambda function that creates BVH nodes */
      [&](const isa::BuildRecord<Node*>& current, isa::BuildRecord<Node*>** children, const size_t N, FastAllocator::ThreadLocal* alloc)
      {
        assert(N <= 2);
        InnerNode* node = new (alloc->malloc(sizeof(InnerNode))) InnerNode;
        for (size_t i=0; i<N; i++) {
          node->bounds[i] = children[i]->geomBounds;
          children[i]->parent = &node->children[i];
        }
        *current.parent = node;
	return 0;
      },

      /* lambda function that creates BVH leaves */
      [&](const isa::BuildRecord<Node*>& current, FastAllocator::ThreadLocal* alloc) 
      {
        assert(current.size() == 1);
        Node* node = new (alloc->malloc(sizeof(LeafNode))) LeafNode(prims[current.begin].ID(),prims[current.begin].bounds());
        *current.parent = node;
	return 0;
      },
      prims.data(),pinfo,2,1024,1,1,1);
    
    double t1 = getSeconds();

    std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(N)/(t1-t0) << " Mprims/s, sah = " << root->sah() << " [DONE]" << std::endl;
  }
}

void build_morton(std::vector<PrimRef>& prims, isa::PrimInfo& pinfo)
{
  size_t N = pinfo.size();
  /* array for morton builder */
  std::vector<isa::MortonID32Bit> morton_src(N);
  std::vector<isa::MortonID32Bit> morton_tmp(N);
  for (size_t i=0; i<N; i++) 
    morton_src[i].index = i;

  /* fast allocator that supports thread local operation */
  FastAllocator allocator;

  for (size_t i=0; i<2; i++)
  {
    std::cout << "iteration " << i << ": building BVH over " << N << " primitives, " << std::flush;
    double t0 = getSeconds();
    
    allocator.reset();

    std::pair<Node*,BBox3fa> node_bounds = isa::bvh_builder_center<Node*>(

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
        const BBox3fa bounds = prims[id].bounds(); // FIXME: dont use morton_src, should be input
        Node* node = new (alloc->malloc(sizeof(LeafNode))) LeafNode(id,bounds);
        *current.parent = node;
        box_o = bounds;
        return node;
      },

      /* lambda that calculates the bounds for some primitive */
      [&] (const isa::MortonID32Bit& morton) -> BBox3fa {
        return prims[morton.index].bounds();
      },
      morton_src.data(),morton_tmp.data(),prims.size(),2,1024,1,1);

    Node* root = node_bounds.first;
    
    double t1 = getSeconds();

    std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(N)/(t1-t0) << " Mprims/s, sah = " << root->sah() << " [DONE]" << std::endl;
  }
}

/* called by the C++ code for initialization */
extern "C" void device_init (int8* cfg)
{
  /* initialize ray tracing core */
  rtcInit(cfg);

  /* set error handler */
  rtcSetErrorFunction(error_handler);
  
  /* set start render mode */
  renderPixel = renderPixelStandard;

  /* create random bounding boxes */
  const size_t N = 2300000;
  isa::PrimInfo pinfo(empty);
  std::vector<PrimRef> prims; // FIXME: does not support alignment
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
  rtcDebug();
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup () {
  rtcExit();
}

