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
#define GEOMETRY_FILENAME  "/home/micuser/geometry.bin"

#else

#define RAYSTREAM_FILENAME "ray16.bin"
#define GEOMETRY_FILENAME  "geometry.bin"

#endif

#define DBG(x)

namespace embree
{
  using namespace std;
	 

  RayStreamLogger::RayStreamLogger() : initialized(false)
  {
    int error = pthread_mutex_init(&mutex,NULL);
  }


  RayStreamLogger::~RayStreamLogger()
    {
      //PING;

      if (initialized)
	{
	  rayData.close();
	}
    }

  void RayStreamLogger::openRayDataStream()
  {
    FileName filename(RAYSTREAM_FILENAME);
    rayData.open(filename.c_str(),ios::out | ios::binary);
    rayData.seekp(0, ios::beg);
 
    if (!rayData)
      {
	FATAL("could not dump ray data to file");
      }    
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

    geometryData.write((char*)&numGroups,sizeof(numGroups));
    geometryData.write((char*)&numTotalTriangles,sizeof(numTotalTriangles));

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
      geometryData.write((char*)mesh->vertices[0].getPtr(),sizeof(Vec3fa)*mesh->numVertices);

      geometryData.write((char*)&mesh->numTriangles,sizeof(mesh->numTriangles));
      geometryData.write((char*)mesh->triangles.getPtr(),sizeof(TriangleMesh::Triangle)*mesh->numTriangles);     
    }

    geometryData << flush;
    geometryData.close();
  }

  void RayStreamLogger::logRay16Intersect(const void* valid_i, void* scene, RTCRay16& start, RTCRay16& end)
  {
    pthread_mutex_lock(&mutex);

    if (!initialized)
      {
	openRayDataStream();
	initialized = true;
      }

    LogRay16 logRay16;

    logRay16.type    = RAY_INTERSECT;
    logRay16.m_valid = *(mic_i*)valid_i != mic_i(0);
    logRay16.start   = start;
    logRay16.end     = end;
    rayData.write((char*)&logRay16 ,sizeof(logRay16));
    rayData << flush;
    pthread_mutex_unlock(&mutex);
  }

  void RayStreamLogger::logRay16Occluded(const void* valid_i, void* scene, RTCRay16& start, RTCRay16& end)
  {
    pthread_mutex_lock(&mutex);
    if (!initialized)
      {
	openRayDataStream();
	initialized = true;
      }

    LogRay16 logRay16;

    logRay16.type    = RAY_OCCLUDED;
    logRay16.m_valid = *(mic_i*)valid_i != mic_i(0);
    logRay16.start   = start;
    logRay16.end     = end;
    rayData.write((char*)&logRay16 ,sizeof(logRay16));
    rayData << flush;
    pthread_mutex_unlock(&mutex);
  }

  RayStreamLogger RayStreamLogger::rayStreamLogger;

};
