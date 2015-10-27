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

#include "../../../common/tutorial/scene.h"
#include "../../transport/transport_host.h"
#include "../../transport/transport_device.h"
#include "../common.h"

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
  enum ISPCType { TRIANGLE_MESH, SUBDIV_MESH, HAIR_SET, INSTANCE };

  struct ISPCGeometry
  {
    ISPCGeometry(ISPCType type) 
      : type(type) {}

    ISPCType type;
  };

  struct ISPCInstance : public ISPCGeometry
  {
    ISPCInstance() 
      : ISPCGeometry(INSTANCE) {}

    //ISPCGeometry geom;
    AffineSpace3fa space;
    int geomID;
  };

  /* ISPC compatible mesh */
  struct ISPCTriangleMesh : public ISPCGeometry // FIXME: why all these replicated classes!!!!!!!
  {
    ALIGNED_CLASS;
  public:
    ISPCTriangleMesh (int numTriangles, 
	      int numQuads, 
	      int numVertices, 
	      int meshMaterialID) 
      : ISPCGeometry(TRIANGLE_MESH), numTriangles(numTriangles), numQuads(numQuads), numVertices(numVertices),
        positions(nullptr), positions2(nullptr), normals(nullptr), texcoords(nullptr), triangles(nullptr), quads(nullptr), meshMaterialID(meshMaterialID)
    {
      sizePositions = 0;
      sizeNormals   = 0;
      sizeTexCoords = 0;
      sizeTriangles = 0;
      sizeQuads     = 0;
    }

    ~ISPCTriangleMesh () {
      if (positions)  os_free(positions ,sizePositions);
      if (positions2) os_free(positions2,sizePositions);
      if (normals)    os_free(normals   ,sizeNormals);
      if (texcoords)  os_free(texcoords ,sizeTexCoords);
      if (triangles)  os_free(triangles ,sizeTriangles);
      if (quads)      os_free(quads     ,sizeQuads);

      positions = nullptr;
      positions2 = nullptr;
      normals   = nullptr;
      texcoords = nullptr;
      triangles = nullptr;
      quads = nullptr;
    }

  public:
    //ISPCGeometry geom;
    Vec3fa* positions;     //!< vertex position array
    Vec3fa* positions2;    //!< vertex position array
    Vec3fa* normals;       //!< vertex normal array
    Vec2f*  texcoords;     //!< vertex texcoord array
    TutorialScene::Triangle* triangles;  //!< list of triangles
    TutorialScene::Quad* quads;          //!< list of quads
    int numVertices;
    int numTriangles;
    int numQuads;
    int geomID;
    int meshMaterialID;
    size_t sizePositions;
    size_t sizeNormals;
    size_t sizeTexCoords;
    size_t sizeTriangles;
    size_t sizeQuads;
  };


  struct ISPCSubdivMesh : public ISPCGeometry
{
  ALIGNED_CLASS;
public:
  ISPCSubdivMesh(int numVertices, int numFaces, int numEdges, int materialID) : 
    ISPCGeometry(SUBDIV_MESH), 
    numVertices(numVertices), numFaces(numFaces), numEdges(numEdges), materialID(materialID),
    numEdgeCreases(0),numVertexCreases(0),numHoles(0),geomID(0),
    positions(nullptr),normals(nullptr),position_indices(nullptr),
    normal_indices(nullptr),texcoord_indices(nullptr), verticesPerFace(nullptr),
    holes(nullptr), subdivlevel(nullptr), 
    edge_creases(nullptr), edge_crease_weights(nullptr), vertex_creases(nullptr), 
    vertex_crease_weights(nullptr)
  {
    //PRINT(numVertices);
    //PRINT(numFaces);
    //PRINT(numEdges);
  }

  //ISPCGeometry geom;
  Vec3fa* positions;       //!< vertex positions
  Vec3fa* normals;         //!< face vertex normals
  Vec2f* texcoords;        //!< face texture coordinates
  int* position_indices;   //!< position indices for all faces
  int* normal_indices;     //!< normal indices for all faces
  int* texcoord_indices;   //!< texcoord indices for all faces
  int* verticesPerFace;    //!< number of indices of each face
  int* holes;              //!< face ID of holes
  float* subdivlevel;      //!< subdivision level
  Vec2i* edge_creases;          //!< crease index pairs
  float* edge_crease_weights;   //!< weight for each crease
  int* vertex_creases;          //!< indices of vertex creases
  float* vertex_crease_weights; //!< weight for each vertex crease
  int* face_offsets;
  int numVertices;
  int numFaces;
  int numEdges;
  int numEdgeCreases;
  int numVertexCreases;
  int numHoles;
  int materialID;
  int geomID;
};

  /* ISPC compatible scene */
  struct ISPCHair
  {
  public:
    ISPCHair () {}
    ISPCHair (int vertex, int id)
      : vertex(vertex), id(id) {}

    int vertex,id;  //!< index of first control point and hair ID
  };

  /*! Hair Set. */
  struct ISPCHairSet : public ISPCGeometry
  {
    ALIGNED_CLASS;
  public:
    //ISPCGeometry geom;
    Vec3fa *positions;   //!< hair control points (x,y,z,r)
    Vec3fa *positions2;   //!< hair control points (x,y,z,r)
    ISPCHair *hairs;    //!< list of hairs
    int numVertices;
    int numHairs;
    int materialID;
    ISPCHairSet(int numHairs, int numVertices, int materialID) 
      : ISPCGeometry(HAIR_SET), numHairs(numHairs),numVertices(numVertices),positions(nullptr),positions2(nullptr),hairs(nullptr),materialID(materialID) {}
    ~ISPCHairSet() {
      if (positions) free(positions);
      if (positions2) free(positions2);
      if (hairs) free(hairs);
    }
  };

struct ISPCSubdivMeshKeyFrame {
  ISPCSubdivMesh** subdiv;                   //!< list of subdiv meshes
  int numSubdivMeshes;                       //!< number of subdiv meshes
};


  /* ISPC compatible scene */
  struct ISPCScene
  {
    ALIGNED_CLASS;
  public:
    ISPCScene (int numGeometries, 
               void* materials_in, int numMaterials,
               void* ambientLights_in, int numAmbientLights,
               void* pointLights_in, int numPointLights,
               void* directionalLights_in, int numDirectionalLights,
               void* distantLights_in, int numDistantLights)

      : geometry(nullptr), numGeometries(numGeometries),
        materials(nullptr), numMaterials(numMaterials),
        ambientLights(nullptr), numAmbientLights(numAmbientLights),
        pointLights(nullptr), numPointLights(numPointLights),
        directionalLights(nullptr), numDirectionalLights(numDirectionalLights),
        distantLights(nullptr), numDistantLights(numDistantLights),
        subdivMeshKeyFrames(nullptr), numSubdivMeshKeyFrames(0),
        instances(nullptr), numInstances(0)
      {
        geometry = new ISPCGeometry*[numGeometries];
        for (size_t i=0; i<numGeometries; i++)
          geometry[i] = nullptr;

        materials = new Material[numMaterials];
        memcpy(materials,materials_in,numMaterials*sizeof(Material));

	/* no texture support for Xeon Phi */
	for (size_t i=0;i<numMaterials;i++)
	  if (materials[i].ty == MATERIAL_OBJ)
	  {
	     OBJMaterial *objm = ( OBJMaterial*)&materials[i];
             objm->map_d     = NULL;
	     objm->map_Kd    = NULL;
             objm->map_Displ = NULL;
	  }
	
        ambientLights = new AmbientLight[numAmbientLights];
        memcpy(ambientLights,ambientLights_in,numAmbientLights*sizeof(AmbientLight));

        pointLights = new PointLight[numPointLights];
        memcpy(pointLights,pointLights_in,numPointLights*sizeof(PointLight));

        directionalLights = new DirectionalLight[numDirectionalLights];
        memcpy(directionalLights,directionalLights_in,numDirectionalLights*sizeof(DirectionalLight));

        distantLights = new DistantLight[numDistantLights];
        memcpy(distantLights,distantLights_in,numDistantLights*sizeof(DistantLight));
      }

    ~ISPCScene () 
    {
      delete[] materials;
      delete[] ambientLights;
      delete[] pointLights;
      delete[] directionalLights;
      delete[] distantLights;

      /*if (geometries)  // FIXME: would need virtual destructor for this to work !!!!!!
      {
        for (size_t i=0; i<numGeometries; i++)
          if (geometry[i]) delete meshes[i];
	delete[] geometry;
	meshes = geometry;
        }*/
    }

  public:
    ISPCGeometry** geometry;
    Material* materials;  //!< material list
    int numGeometries;
    int numMaterials;

    AmbientLight* ambientLights;
    int numAmbientLights;

    PointLight* pointLights;
    int numPointLights;

    DirectionalLight* directionalLights;
    int numDirectionalLights;

    DistantLight* distantLights;
    int numDistantLights;

    ISPCSubdivMeshKeyFrame** subdivMeshKeyFrames;
    int numSubdivMeshKeyFrames;

    ISPCInstance** instances;
    int numInstances;
  };

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
    mesh->triangles = (TutorialScene::Triangle*)os_malloc(in_pBufferLengths[3]);
    mesh->quads     = (TutorialScene::Quad*)os_malloc(in_pBufferLengths[4]);

    memcpy(mesh->positions,in_ppBufferPointers[0],in_pBufferLengths[0]);
    memcpy(mesh->normals  ,in_ppBufferPointers[1],in_pBufferLengths[1]);
    memcpy(mesh->texcoords,in_ppBufferPointers[2],in_pBufferLengths[2]);
    memcpy(mesh->triangles,in_ppBufferPointers[3],in_pBufferLengths[3]);
    memcpy(mesh->quads    ,in_ppBufferPointers[4],in_pBufferLengths[4]);

    mesh->sizePositions = in_pBufferLengths[0];
    mesh->sizeNormals   = in_pBufferLengths[1];
    mesh->sizeTexCoords = in_pBufferLengths[2];
    mesh->sizeTriangles = in_pBufferLengths[3];
    mesh->sizeQuads     = in_pBufferLengths[4];
    
#if 1 // FIXME: what does this?
    if (mesh->quads[0].v0 == 0,
	mesh->quads[0].v1 == 0,
	mesh->quads[0].v2 == 0,
	mesh->quads[0].v3 == 0)
      {
	mesh->quads = nullptr;
	mesh->numQuads = 0;
	mesh->sizeQuads = 0;
      }
#endif

    g_ispc_scene->geometry[geomID] = mesh;
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
 
    g_ispc_scene->geometry[geomID] = mesh;
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
    memcpy(hairset->positions = (Vec3fa*)malloc(in_pBufferLengths[0]),in_ppBufferPointers[0],in_pBufferLengths[0]);
    memcpy(hairset->hairs = (ISPCHair*)malloc(in_pBufferLengths[1]),in_ppBufferPointers[1],in_pBufferLengths[1]);
    g_ispc_scene->geometry[geomID] = hairset;
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
    bool hit = device_pick(in_pMiscData->x,
                           in_pMiscData->y,
                           in_pMiscData->vx,
                           in_pMiscData->vy,
                           in_pMiscData->vz,
                           in_pMiscData->p,
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
    device_render((int*)in_ppBufferPointers[0],
                  in_pMiscData->width,
                  in_pMiscData->height,
                  in_pMiscData->time, 
                  in_pMiscData->vx, 
                  in_pMiscData->vy, 
                  in_pMiscData->vz,
                  in_pMiscData->p);
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
