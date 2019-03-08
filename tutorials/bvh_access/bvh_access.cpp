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

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/tutorial_device.h"
#include "../../include/embree3/rtcore.h"
RTC_NAMESPACE_OPEN
#include "../../kernels/bvh/bvh.h"
#include "../../kernels/geometry/trianglev.h"

namespace embree
{
  /* error reporting function */
  void error_handler(void* userPtr, const RTCError code, const char* str)
  {
    if (code == RTC_ERROR_NONE) 
      return;
    
    printf("Embree: ");
    switch (code) {
    case RTC_ERROR_UNKNOWN          : printf("RTC_ERROR_UNKNOWN"); break;
    case RTC_ERROR_INVALID_ARGUMENT : printf("RTC_ERROR_INVALID_ARGUMENT"); break;
    case RTC_ERROR_INVALID_OPERATION: printf("RTC_ERROR_INVALID_OPERATION"); break;
    case RTC_ERROR_OUT_OF_MEMORY    : printf("RTC_ERROR_OUT_OF_MEMORY"); break;
    case RTC_ERROR_UNSUPPORTED_CPU  : printf("RTC_ERROR_UNSUPPORTED_CPU"); break;
    case RTC_ERROR_CANCELLED        : printf("RTC_ERROR_CANCELLED"); break;
    default                         : printf("invalid error code"); break;
    }
    if (str) { 
      printf(" ("); 
      while (*str) putchar(*str++); 
      printf(")\n"); 
    }
    exit(1);
  }

  /* adds a cube to the scene */
  unsigned int addCube (RTCDevice device_i, RTCScene scene_i, const Vec3fa& pos)
  {
    /* create a triangulated cube with 12 triangles and 8 vertices */
    RTCGeometry mesh = rtcNewGeometry (device_i, RTC_GEOMETRY_TYPE_TRIANGLE);
    
    /* set vertices */
    Vec3fa* vertices = (Vec3fa*) rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vec3fa), 8); 
    vertices[0].x = pos.x + -1; vertices[0].y = pos.y + -1; vertices[0].z = pos.z + -1; 
    vertices[1].x = pos.x + -1; vertices[1].y = pos.y + -1; vertices[1].z = pos.z + +1; 
    vertices[2].x = pos.x + -1; vertices[2].y = pos.y + +1; vertices[2].z = pos.z + -1; 
    vertices[3].x = pos.x + -1; vertices[3].y = pos.y + +1; vertices[3].z = pos.z + +1; 
    vertices[4].x = pos.x + +1; vertices[4].y = pos.y + -1; vertices[4].z = pos.z + -1; 
    vertices[5].x = pos.x + +1; vertices[5].y = pos.y + -1; vertices[5].z = pos.z + +1; 
    vertices[6].x = pos.x + +1; vertices[6].y = pos.y + +1; vertices[6].z = pos.z + -1; 
    vertices[7].x = pos.x + +1; vertices[7].y = pos.y + +1; vertices[7].z = pos.z + +1; 
    
    /* set triangles */
    int tri = 0;
    Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), 12);
    
    // left side
    triangles[tri].v0 = 0; triangles[tri].v1 = 2; triangles[tri].v2 = 1; tri++;
    triangles[tri].v0 = 1; triangles[tri].v1 = 2; triangles[tri].v2 = 3; tri++;
    
    // right side
    triangles[tri].v0 = 4; triangles[tri].v1 = 5; triangles[tri].v2 = 6; tri++;
    triangles[tri].v0 = 5; triangles[tri].v1 = 7; triangles[tri].v2 = 6; tri++;
    
    // bottom side
    triangles[tri].v0 = 0; triangles[tri].v1 = 1; triangles[tri].v2 = 4; tri++;
    triangles[tri].v0 = 1; triangles[tri].v1 = 5; triangles[tri].v2 = 4; tri++;
    
    // top side
    triangles[tri].v0 = 2; triangles[tri].v1 = 6; triangles[tri].v2 = 3; tri++;
    triangles[tri].v0 = 3; triangles[tri].v1 = 6; triangles[tri].v2 = 7; tri++;
    
    // front side
    triangles[tri].v0 = 0; triangles[tri].v1 = 4; triangles[tri].v2 = 2; tri++;
    triangles[tri].v0 = 2; triangles[tri].v1 = 4; triangles[tri].v2 = 6; tri++;
    
    // back side
    triangles[tri].v0 = 1; triangles[tri].v1 = 3; triangles[tri].v2 = 5; tri++;
    triangles[tri].v0 = 3; triangles[tri].v1 = 7; triangles[tri].v2 = 5; tri++;

    rtcCommitGeometry(mesh);
    unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
    rtcReleaseGeometry(mesh);
    return geomID;
  }

  /* adds a ground plane to the scene */
  unsigned int addGroundPlane (RTCDevice device_i, RTCScene scene_i)
  {
    /* create a triangulated plane with 2 triangles and 4 vertices */
    RTCGeometry mesh = rtcNewGeometry (device_i, RTC_GEOMETRY_TYPE_TRIANGLE);
    
    /* set vertices */
    Vec3fa* vertices = (Vec3fa*) rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vec3fa), 4); 
    vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10; 
    vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10; 
    vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10; 
    vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
    
    /* set triangles */
    Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), 2);
    triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
    triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;

    rtcCommitGeometry(mesh);
    unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
    rtcReleaseGeometry(mesh);
    return geomID;
  }

  /* adds a hair to the scene */
  unsigned int addHair(RTCDevice device_i, RTCScene scene_i)
  {
    RTCGeometry geom = rtcNewGeometry (device_i, RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE);

    vfloat4* pos = (vfloat4*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, sizeof(vfloat4), 4);
    pos[0] = vfloat4(0.0f,0.0f,0.0f,0.1f);
    pos[1] = vfloat4(0.0f,1.0f,0.0f,0.1f);
    pos[2] = vfloat4(0.0f,2.0f,0.0f,0.1f);
    pos[3] = vfloat4(0.0f,3.0f,0.0f,0.1f);

    int* index = (int*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, sizeof(int), 1);
    index[0] = 0;

    rtcCommitGeometry(geom);
    unsigned int geomID = rtcAttachGeometry(scene_i,geom);
    rtcReleaseGeometry(geom);
    return geomID;
  }

  /* prints the bvh4.triangle4v data structure */
  void print_bvh4_triangle4v(BVH4::NodeRef node, size_t depth)
  {
    if (node.isAlignedNode())
    {
      BVH4::AlignedNode* n = node.alignedNode();
      
      std::cout << "AlignedNode {" << std::endl;
      for (size_t i=0; i<4; i++)
      {
        for (size_t k=0; k<depth; k++) std::cout << "  ";
        std::cout << "  bounds" << i << " = " << n->bounds(i) << std::endl;
      }

      for (size_t i=0; i<4; i++)
      {
        if (n->child(i) == BVH4::emptyNode)
          continue;

        for (size_t k=0; k<depth; k++) std::cout << "  ";
        std::cout << "  child" << i << " = ";
        print_bvh4_triangle4v(n->child(i),depth+1); 
      }
      for (size_t k=0; k<depth; k++) std::cout << "  ";
      std::cout << "}" << std::endl;
    }
    else
    {
      size_t num; 
      const Triangle4v* tri = (const Triangle4v*) node.leaf(num);

      std::cout << "Leaf {" << std::endl;
      for (size_t i=0; i<num; i++) {
        for (size_t j=0; j<tri[i].size(); j++) {
          for (size_t k=0; k<depth; k++) std::cout << "  ";
          std::cout << "  Triangle { v0 = (" << tri[i].v0.x[j] << ", " << tri[i].v0.y[j] << ", " << tri[i].v0.z[j] << "),  "
            "v1 = (" << tri[i].v1.x[j] << ", " << tri[i].v1.y[j] << ", " << tri[i].v1.z[j] << "), "
            "v2 = (" << tri[i].v2.x[j] << ", " << tri[i].v2.y[j] << ", " << tri[i].v2.z[j] << "), "
            "geomID = " << tri[i].geomID(j) << ", primID = " << tri[i].primID(j) << " }" << std::endl;
        }
      }
      for (size_t k=0; k<depth; k++) std::cout << "  ";
      std::cout << "}" << std::endl;
    }
  }

  /* prints the triangle BVH of a scene */
  void print_bvh(RTCScene scene)
  {
    BVH4* bvh4 = nullptr; 

    /* if the scene contains only triangles, the BVH4 acceleration structure can be obtained this way */
    AccelData* accel = ((Accel*)scene)->intersectors.ptr;
    if (accel->type == AccelData::TY_BVH4) 
      bvh4 = (BVH4*)accel;

    /* if there are also other geometry types, one has to iterate over the toplevel AccelN structure */
    else if (accel->type == AccelData::TY_ACCELN)
    {
      AccelN* accelN = (AccelN*)(accel);
      for (size_t i=0; i<accelN->accels.size(); i++) {
        if (accelN->accels[i]->intersectors.ptr->type == AccelData::TY_BVH4) {
          bvh4 = (BVH4*)accelN->accels[i]->intersectors.ptr;
          if (std::string(bvh4->primTy->name()) == "triangle4v") break;
          bvh4 = nullptr;
        }
      }
    }
    if (bvh4 == nullptr)
      throw std::runtime_error("cannot access BVH4 acceleration structure"); // will not happen if you use this Embree version
      
    /* now lets print the entire hierarchy */
    print_bvh4_triangle4v(bvh4->root,0);
  }

  /* main function in embree namespace */
  int main(int argc, char** argv) 
  {
    /* for best performance set FTZ and DAZ flags in MXCSR control and status register */
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    /* create new Embree device and force bvh4.triangle4v hierarchy for triangles */
    RTCDevice device = rtcNewDevice("tri_accel=bvh4.triangle4v");
    error_handler(nullptr,rtcGetDeviceError(device));
    
    /* set error handler */
    rtcSetDeviceErrorFunction(device,error_handler,nullptr);
    
    /* create scene */
    RTCScene scene = rtcNewScene(device);
    addCube(device,scene,Vec3fa(-1,0,0));
    addCube(device,scene,Vec3fa(1,0,0));
    addCube(device,scene,Vec3fa(0,0,-1));
    addCube(device,scene,Vec3fa(0,0,1));
    addHair(device,scene);
    addGroundPlane(device,scene);
    rtcCommitScene (scene);
    /* print triangle BVH */
    print_bvh(scene);

    /* cleanup */
    rtcReleaseScene (scene);
    rtcReleaseDevice(device);
    return 0;
  }
}

int main(int argc, char** argv)
{
  try {
    return embree::main(argc, argv);
  }
  catch (const std::exception& e) {
    std::cout << "Error: " << e.what() << std::endl;
    return 1;
  }
  catch (...) {
    std::cout << "Error: unknown exception caught." << std::endl;
    return 1;
  }
}
