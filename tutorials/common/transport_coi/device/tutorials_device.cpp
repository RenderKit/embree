// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "../../../common/tutorial/scene.h"
#include "../../transport/transport_host.h"
#include "../../transport/transport_device.h"
#include "../common.h"
#include "../../tutorial/scene_device.h"

#include <sink/COIPipeline_sink.h>
#include <sink/COIProcess_sink.h>
#include <common/COIMacros_common.h>
#include <common/COISysInfo_common.h>
#include <common/COIEvent_common.h>


extern "C" int64_t get_tsc() {
return embree::read_tsc();
}

float g_debug = 0.0f;

namespace embree
{
  /* scene */
  static size_t g_geometryID = 0;

  extern "C" ISPCScene* g_ispc_scene = nullptr;
  extern "C" int g_instancing_mode = 0;

  extern "C" void run_init(uint32_t         in_BufferCount,
                           void**           in_ppBufferPointers,
                           uint64_t*        in_pBufferLengths,
                           InitData*        in_pMiscData,
                           uint16_t         in_MiscDataLength,
                           void*            in_pReturnValue,
                           uint16_t         in_ReturnValueLength)
  {
    device_init(in_pMiscData->cfg);
  }

  extern "C" void run_key_pressed(uint32_t         in_BufferCount,
                                  void**           in_ppBufferPointers,
                                  uint64_t*        in_pBufferLengths,
                                  KeyPressedData* in_pMiscData,
                                  uint16_t         in_MiscDataLength,
                                  void*            in_pReturnValue,
                                  uint16_t         in_ReturnValueLength)
  {
    call_key_pressed_handler(in_pMiscData->key);
  }

  extern "C" void run_create_mesh(uint32_t         in_BufferCount,
                                  void**           in_ppBufferPointers,
                                  uint64_t*        in_pBufferLengths,
                                  CreateMeshData*  in_pMiscData,
                                  uint16_t         in_MiscDataLength,
                                  void*            in_pReturnValue,
                                  uint16_t         in_ReturnValueLength)
  {
    size_t geomID = g_geometryID++;

    ISPCTriangleMesh* mesh = new ISPCTriangleMesh(in_pMiscData->numTriangles,in_pMiscData->numQuads,in_pMiscData->numVertices,in_pMiscData->meshMaterialID);
    assert( mesh );
    assert( in_pMiscData->numTriangles*sizeof(TutorialScene::Triangle) == in_pBufferLengths[3] );
    assert( in_pMiscData->numQuads*sizeof(TutorialScene::Quad) == in_pBufferLengths[4] );

    //assert( in_pMiscData->numVertices*sizeof(Vec3fa) == in_pBufferLengths[1] );

    mesh->positions = (Vec3fa*)os_malloc(in_pBufferLengths[0]);
    mesh->normals   = (Vec3fa*)os_malloc(in_pBufferLengths[1]);
    mesh->texcoords = (Vec2f* )os_malloc(in_pBufferLengths[2]);
    mesh->triangles = (ISPCTriangle*)os_malloc(in_pBufferLengths[3]);
    mesh->quads     = (ISPCQuad*)os_malloc(in_pBufferLengths[4]);

    memcpy(mesh->positions,in_ppBufferPointers[0],in_pBufferLengths[0]);
    memcpy(mesh->normals  ,in_ppBufferPointers[1],in_pBufferLengths[1]);
    memcpy(mesh->texcoords,in_ppBufferPointers[2],in_pBufferLengths[2]);
    memcpy(mesh->triangles,in_ppBufferPointers[3],in_pBufferLengths[3]);
    memcpy(mesh->quads    ,in_ppBufferPointers[4],in_pBufferLengths[4]);

    //mesh->sizePositions = in_pBufferLengths[0];
    //mesh->sizeNormals   = in_pBufferLengths[1];
    //mesh->sizeTexCoords = in_pBufferLengths[2];
    //mesh->sizeTriangles = in_pBufferLengths[3];
    //mesh->sizeQuads     = in_pBufferLengths[4];
    
#if 1 // FIXME: what does this?
    if (mesh->quads[0].v0 == 0,
	mesh->quads[0].v1 == 0,
	mesh->quads[0].v2 == 0,
	mesh->quads[0].v3 == 0)
      {
	mesh->quads = nullptr;
	mesh->numQuads = 0;
	//mesh->sizeQuads = 0;
      }
#endif

    g_ispc_scene->geometries[geomID] = (ISPCGeometry*) mesh;
  }


  extern "C" void run_create_subdiv_mesh(uint32_t         in_BufferCount,
					 void**           in_ppBufferPointers,
					 uint64_t*        in_pBufferLengths,
					 CreateSubdivMeshData*  in_pMiscData,
					 uint16_t         in_MiscDataLength,
					 void*            in_pReturnValue,
					 uint16_t         in_ReturnValueLength)
  {
    size_t geomID = g_geometryID++;

    const size_t numVertices = in_pMiscData->numPositions;
    const size_t numEdges    = in_pMiscData->numPositionIndices;
    const size_t numFaces    = in_pMiscData->numVerticesPerFace;

    ISPCSubdivMesh* mesh = new ISPCSubdivMesh(numVertices,
                                              numFaces,
                                              numEdges,
                                              in_pMiscData->materialID);
    assert( mesh );
       
    assert( in_pMiscData->numPositions*sizeof(Vec3fa)    == in_pBufferLengths[0] );
    assert( in_pMiscData->numPositionIndices*sizeof(int) == in_pBufferLengths[1] );
    assert( in_pMiscData->numVerticesPerFace*sizeof(int) == in_pBufferLengths[2] );

    mesh->positions        = (Vec3fa*)os_malloc(in_pBufferLengths[0]);
    memcpy(mesh->positions       ,in_ppBufferPointers[0],in_pBufferLengths[0]);

    mesh->position_indices = (int*)   os_malloc(in_pBufferLengths[1]);
    memcpy(mesh->position_indices,in_ppBufferPointers[1],in_pBufferLengths[1]);

    mesh->verticesPerFace  = (int*)   os_malloc(in_pBufferLengths[2]);
    memcpy(mesh->verticesPerFace ,in_ppBufferPointers[2],in_pBufferLengths[2]);


    mesh->subdivlevel      = (float*) os_malloc(in_pBufferLengths[1]);
    mesh->face_offsets     = (int*)   os_malloc(sizeof(int) * in_pMiscData->numVerticesPerFace);
    
    if ( in_pMiscData->numEdgeCreases )
      {
	assert(in_pBufferLengths[3] == sizeof(Vec2i) * in_pMiscData->numEdgeCreases);
	mesh->edge_creases = (Vec2i*)os_malloc(sizeof(Vec2i) * in_pMiscData->numEdgeCreases); 
	memcpy(mesh->edge_creases ,in_ppBufferPointers[3],in_pBufferLengths[3]);	
	mesh->numEdgeCreases = in_pMiscData->numEdgeCreases;
      }

    if ( in_pMiscData->numEdgeCreaseWeights )
      {
	assert(in_pBufferLengths[4] == sizeof(float) * in_pMiscData->numEdgeCreaseWeights);
	mesh->edge_crease_weights = (float*)os_malloc(sizeof(float) * in_pMiscData->numEdgeCreaseWeights); 
	memcpy(mesh->edge_crease_weights ,in_ppBufferPointers[4],in_pBufferLengths[4]);	
      }

    if ( in_pMiscData->numVertexCreases )
      {
	mesh->numVertexCreases = in_pMiscData->numVertexCreases;
	assert(in_pBufferLengths[5] == sizeof(int) * in_pMiscData->numVertexCreases);
	mesh->vertex_creases = (int*)os_malloc(sizeof(int) *  in_pMiscData->numVertexCreases); 
	memcpy(mesh->vertex_creases ,in_ppBufferPointers[5],in_pBufferLengths[5]);	
      }

    if ( in_pMiscData->numVertexCreaseWeights )
      {
	assert(in_pBufferLengths[6] == sizeof(float) * in_pMiscData->numVertexCreaseWeights);
	mesh->vertex_crease_weights = (float*)os_malloc(sizeof(float) * in_pMiscData->numVertexCreaseWeights); 
	memcpy(mesh->vertex_crease_weights ,in_ppBufferPointers[6],in_pBufferLengths[6]);	
      }

    if ( in_pMiscData->numHoles )
      {
	mesh->numHoles = in_pMiscData->numHoles;
	assert(in_pBufferLengths[7] == sizeof(int) * in_pMiscData->numHoles);
	mesh->holes = (int*)os_malloc(sizeof(int) * in_pMiscData->numHoles); 
	memcpy(mesh->holes ,in_ppBufferPointers[7],in_pBufferLengths[7]);
      }

    for (size_t i=0; i<numEdges; i++) mesh->subdivlevel[i] = 1.0f;
    int offset = 0;
    for (size_t i=0; i<numFaces; i++)
      {
        mesh->face_offsets[i] = offset;
        offset+=mesh->verticesPerFace[i];       
      }
 
    g_ispc_scene->geometries[geomID] = (ISPCGeometry*) mesh;
  }

  extern "C" void run_create_hairset(uint32_t         in_BufferCount,
				     void**           in_ppBufferPointers,
				     uint64_t*        in_pBufferLengths,
				     CreateHairSetData*  in_pMiscData,
				     uint16_t         in_MiscDataLength,
				     void*            in_pReturnValue,
				     uint16_t         in_ReturnValueLength)
  {
    size_t geomID = g_geometryID++;
    ISPCHairSet* hairset = new ISPCHairSet(in_pMiscData->numHairs,in_pMiscData->numVertices,in_pMiscData->materialID);
    memcpy(hairset->v = (Vec3fa*)malloc(in_pBufferLengths[0]),in_ppBufferPointers[0],in_pBufferLengths[0]);
    memcpy(hairset->hairs = (ISPCHair*)malloc(in_pBufferLengths[1]),in_ppBufferPointers[1],in_pBufferLengths[1]);
    g_ispc_scene->geometries[geomID] = (ISPCGeometry*) hairset;
  }

  extern "C" void run_create_scene(uint32_t         in_BufferCount,
                                   void**           in_ppBufferPointers,
                                   uint64_t*        in_pBufferLengths,
                                   CreateSceneData* in_pMiscData,
                                   uint16_t         in_MiscDataLength,
                                   void*            in_pReturnValue,
                                   uint16_t         in_ReturnValueLength)
  {
    g_geometryID = 0;
    g_ispc_scene = new ISPCScene(in_pMiscData->numGeometries,
                                 in_ppBufferPointers[0],in_pMiscData->numMaterials,
                                 in_ppBufferPointers[1],in_pMiscData->numAmbientLights,
                                 in_ppBufferPointers[2],in_pMiscData->numPointLights,
                                 in_ppBufferPointers[3],in_pMiscData->numDirectionalLights,
                                 in_ppBufferPointers[4],in_pMiscData->numDistantLights);
  }

  extern "C" void run_pick(uint32_t         in_BufferCount,
                           void**           in_ppBufferPointers,
                           uint64_t*        in_pBufferLengths,
                           PickDataSend*    in_pMiscData,
                           uint16_t         in_MiscDataLength,
                           PickDataReceive* in_pReturnValue,
                           uint16_t         in_ReturnValueLength)
  {
    Vec3fa hitPos = zero;
    ISPCCamera camera(AffineSpace3fa(in_pMiscData->vx,
                                     in_pMiscData->vy,
                                     in_pMiscData->vz,
                                     in_pMiscData->p));
    bool hit = device_pick(in_pMiscData->x,
                           in_pMiscData->y,
                           camera,
                           hitPos);
    in_pReturnValue->pos = hitPos;
    in_pReturnValue->hit = hit;
  }

  extern "C" void run_render(uint32_t         in_BufferCount,
                             void**           in_ppBufferPointers,
                             uint64_t*        in_pBufferLengths,
                             RenderData*      in_pMiscData,
                             uint16_t         in_MiscDataLength,
                             void*            in_pReturnValue,
                             uint16_t         in_ReturnValueLength)
  {
    //double t0 = getSeconds();
    ISPCCamera camera(AffineSpace3fa(in_pMiscData->vx,
                                     in_pMiscData->vy,
                                     in_pMiscData->vz,
                                     in_pMiscData->p));
    device_render((int*)in_ppBufferPointers[0],
                  in_pMiscData->width,
                  in_pMiscData->height,
                  in_pMiscData->time, 
                  camera);
    //double dt = getSeconds() - t0;
    //printf("render %3.2f fps, %.2f ms\n",1.0f/dt,dt*1000.0f); flush(std::cout);
  }

  extern "C" void run_cleanup(uint32_t         in_BufferCount,
                              void**           in_ppBufferPointers,
                              uint64_t*        in_pBufferLengths,
                              void*            in_pMiscData,
                              uint16_t         in_MiscDataLength,
                              void*            in_pReturnValue,
                              uint16_t         in_ReturnValueLength)
  {
    device_cleanup();
    if (g_ispc_scene) delete g_ispc_scene; g_ispc_scene = nullptr;
  }
}

int main(int argc, char** argv) 
{
  UNUSED_ATTR COIRESULT result;
  UNREFERENCED_PARAM (argc);
  UNREFERENCED_PARAM (argv);

  /* enable wait to attach with debugger */
#if 0
  std::cout << "waiting for debugger to attach ..." << std::flush;
#if 0
  volatile int loop = 1;
  do {
    volatile int a = 1;
  } while (loop);
  
#else
  for (int i=0; i<20; i++) {
    sleep(1);
    std::cout << "." << std::flush;
  }
#endif
  std::cout << " [DONE]" << std::endl;
#endif
  
  // Functions enqueued on the sink side will not start executing until
  // you call COIPipelineStartExecutingRunFunctions(). This call is to
  // synchronize any initialization required on the sink side
  result = COIPipelineStartExecutingRunFunctions();
  assert(result == COI_SUCCESS);

  // This call will wait until COIProcessDestroy() gets called on the source
  // side. If COIProcessDestroy is called without force flag set, this call
  // will make sure all the functions enqueued are executed and does all
  // clean up required to exit gracefully.
  COIProcessWaitForShutdown();
  return 0;
}
