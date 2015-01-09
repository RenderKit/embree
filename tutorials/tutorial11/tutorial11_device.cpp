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
#include "kernels/xeon/bvh4/bvh4_builder_generic.h" // FIXME: should not be in bvh4 folder
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
    return area(bounds[0])*children[0]->sah() + area(bounds[1])*children[1]->sah();
  }
};

struct LeafNode : public Node
{
  size_t id;

  LeafNode (size_t id)
    : id(id) {}

  float sah() {
    return 1.0f;
  }
};

PrimRef* tmp = new PrimRef[200];

struct Allocator
{
  void* curPtr() { return tmp; } // FIXME: application should provide temporary buffer
} g_alloc;

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
  isa::PrimInfo pinfo(empty);
  std::vector<PrimRef> prims; // FIXME: does not support alignment
  for (size_t i=0; i<100; i++) {
    const Vec3fa p = 100.0f*Vec3fa(drand48(),drand48(),drand48());
    const BBox3fa b = BBox3fa(p,p+Vec3fa(1.0f));
    pinfo.add(b);
    const PrimRef prim = PrimRef(b,i);
    prims.push_back(prim);
  }

  /* build BVH */
  Node* root = isa::build_bvh_sah_api<Node*>(
    [] () -> Allocator& { return g_alloc; },
    [&](isa::BuildRecord<Node*>* children, const size_t N, Allocator& alloc) -> Node* 
    {
      PRINT(N);
      assert(N <= 2);
      InnerNode* node = new InnerNode;
      for (size_t i=0; i<N; i++) {
        node->bounds[i] = children[i].geomBounds;
        children[i].parent = &node->children[i];
      }
      return node;
    },
    [&](const isa::BuildRecord<Node*>& current, PrimRef* prims, Allocator& alloc) -> Node*
    {
      PRINT(current.size());
      assert(current.size() == 1);
      return new LeafNode(prims[current.begin].ID());
    },
    prims.data(),pinfo,2,1024,0,1,1);
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

