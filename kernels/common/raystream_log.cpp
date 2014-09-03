// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "raystream_log.h"
#include "common/scene.h"
#include "common/scene_triangle_mesh.h"
#include "sys/filename.h"

#if defined(__MIC__)

#define RAYSTREAM_FILENAME "/home/micuser/ray16.bin"
#define RAYSTREAM_VERIFY_FILENAME "/home/micuser/ray16_verify.bin"
#define GEOMETRY_FILENAME  "/home/micuser/geometry.bin"

#else

#define RAYSTREAM_FILENAME        "ray16.bin"
#define RAYSTREAM_VERIFY_FILENAME "ray16_verify.bin"
#define GEOMETRY_FILENAME         "geometry.bin"

#endif

#define DBG(x)

namespace embree
{
  using namespace std;
	 

  RayStreamLogger::RayStreamLogger()
  {
    ray16        = new DataStream( DEFAULT_FILENAME_RAY16 );
    ray16_verify = new DataStream( DEFAULT_FILENAME_RAY16_VERIFY );
    ray8         = new DataStream( DEFAULT_FILENAME_RAY8 );
    ray8_verify  = new DataStream( DEFAULT_FILENAME_RAY8_VERIFY );
    ray4         = new DataStream( DEFAULT_FILENAME_RAY4 );
    ray4_verify  = new DataStream( DEFAULT_FILENAME_RAY4_VERIFY );
    ray1         = new DataStream( DEFAULT_FILENAME_RAY1 );
    ray1_verify  = new DataStream( DEFAULT_FILENAME_RAY1_VERIFY );
  }


  RayStreamLogger::~RayStreamLogger()
    {
      if (ray16)        { delete ray16;        ray16        = NULL; }
      if (ray16_verify) { delete ray16_verify; ray16_verify = NULL; }
      if (ray8)         { delete ray8;         ray8         = NULL; }
      if (ray8_verify)  { delete ray8_verify;  ray8_verify  = NULL; }
      if (ray4)         { delete ray4;         ray4         = NULL; }
      if (ray4_verify)  { delete ray4_verify;  ray4_verify  = NULL; }
      if (ray1)         { delete ray1;         ray1         = NULL; }
      if (ray1_verify)  { delete ray1_verify;  ray1_verify  = NULL; }
    }

  void RayStreamLogger::dumpGeometry(void* ptr)
  {
    Scene *scene = (Scene*)ptr;

    const size_t numGroups = scene->size();

    size_t numTotalTriangles = 0;

    for (size_t g=0; g<numGroups; g++) {       
      if (unlikely(scene->get(g) == NULL)) continue;
      if (unlikely(scene->get(g)->type != TRIANGLE_MESH)) continue;
      const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(g);
      if (unlikely(!mesh->isEnabled())) continue;
      if (unlikely(mesh->numTimeSteps != 1)) continue;
      const size_t numTriangles = mesh->numTriangles;
      numTotalTriangles += numTriangles;
    }

    DBG(
	DBG_PRINT(numGroups);
	DBG_PRINT(numTotalTriangles);
	);

    std::ofstream geometryData;
    FileName geometry_filename(GEOMETRY_FILENAME);
    geometryData.open(geometry_filename.c_str(),ios::out | ios::binary);
    geometryData.seekp(0, ios::beg);

    if (!geometryData) FATAL("could not dump geometry data to file");

    size_t align_check = 0;
    geometryData.write((char*)&numGroups,sizeof(numGroups));
    align_check += sizeof(numGroups);
    geometryData.write((char*)&numTotalTriangles,sizeof(numTotalTriangles));
    align_check += sizeof(numTotalTriangles);

    for (size_t g=0; g<numGroups; g++) {       
      if (unlikely(scene->get(g) == NULL)) continue;
      if (unlikely(scene->get(g)->type != TRIANGLE_MESH)) continue;
      const TriangleMesh*  const mesh = scene->getTriangleMesh(g);
      if (unlikely(!mesh->isEnabled())) continue;
      if (unlikely(mesh->numTimeSteps != 1)) continue;

      DBG(
	  DBG_PRINT( mesh->numVertices );
	  DBG_PRINT( sizeof(Vec3fa)*mesh->numVertices );
	  DBG_PRINT( mesh->numTriangles );
	  DBG_PRINT( sizeof(TriangleMesh::Triangle)*mesh->numTriangles );
	  );

      geometryData.write((char*)&mesh->numVertices,sizeof(mesh->numVertices));
      align_check += sizeof(mesh->numVertices);
      geometryData.write((char*)&mesh->numTriangles,sizeof(mesh->numTriangles));
      align_check += sizeof(mesh->numTriangles);

      if ((align_check % 16) != 0)
	FATAL("vtx alignment");

      geometryData.write((char*)mesh->vertices[0].getPtr(),sizeof(Vec3fa)*mesh->numVertices);
      align_check += sizeof(Vec3fa)*mesh->numVertices;

      geometryData.write((char*)mesh->triangles.getPtr(),sizeof(TriangleMesh::Triangle)*mesh->numTriangles);     
      align_check += sizeof(TriangleMesh::Triangle)*mesh->numTriangles;
      if ((align_check % 16) != 0)
	{
	  size_t dummy_size = 16-(align_check % 16);
	  char dummy[16];
	  memset(dummy,0,16);      
	  geometryData.write(dummy,dummy_size);
	  align_check += dummy_size;
	}

      if ((align_check % 16) != 0)
	FATAL("vtx alignment 2");

    }

    geometryData << flush;
    geometryData.close();
  }

  void RayStreamLogger::logRay16Intersect(const void* valid_i, void* scene, RTCRay16& start, RTCRay16& end)
  {
    mutex.lock();

    LogRay16 logRay16;

    logRay16.type    = RAY_INTERSECT;
#if defined(__MIC__)
    logRay16.m_valid = *(mic_i*)valid_i != mic_i(0);
#endif
    /* ray16 before intersect */
    ray16->write((char*)&logRay16 ,sizeof(logRay16));

    /* ray16 after intersect */
    logRay16.ray16   = end;
    ray16_verify->write((char*)&logRay16 ,sizeof(logRay16));

    mutex.unlock();
  }

  void RayStreamLogger::logRay16Occluded(const void* valid_i, void* scene, RTCRay16& start, RTCRay16& end)
  {
    mutex.lock();

    LogRay16 logRay16;

    logRay16.type    = RAY_OCCLUDED;
#if defined(__MIC__)
    logRay16.m_valid = *(mic_i*)valid_i != mic_i(0);
#endif
    /* ray16 before intersect */
    ray16->write((char*)&logRay16 ,sizeof(logRay16));

    /* ray16 after intersect */
    logRay16.ray16   = end;
    ray16_verify->write((char*)&logRay16 ,sizeof(logRay16));

    mutex.unlock();
  }

 void RayStreamLogger::logRay1Intersect(void* scene, RTCRay& start, RTCRay& end)
  {
    mutex.lock();

    LogRay1 logRay1;

    logRay1.type  = RAY_INTERSECT;

    /* ray before intersect */
    logRay1.ray   = start;
    ray1->write((char*)&logRay1 ,sizeof(logRay1));

    /* ray after intersect */
    logRay1.ray   = end;
    ray1_verify->write((char*)&logRay1 ,sizeof(logRay1));

    mutex.unlock();
  }

  void RayStreamLogger::logRay1Occluded(void* scene, RTCRay& start, RTCRay& end)
  {
    mutex.lock();

    LogRay1 logRay1;

    logRay1.type  = RAY_OCCLUDED;

    /* ray before intersect */
    logRay1.ray   = start;
    ray1->write((char*)&logRay1 ,sizeof(logRay1));

    /* ray after intersect */
    logRay1.ray   = end;
    ray1_verify->write((char*)&logRay1 ,sizeof(logRay1));

    mutex.unlock();
  }

  RayStreamLogger RayStreamLogger::rayStreamLogger;

};
