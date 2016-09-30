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

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/statistics.h"
#include <set>
#include "../../common/sys/mutex.h"

namespace embree
{
  RTCDevice g_device = nullptr;
  RTCScene g_scene = nullptr;
  RTCScene g_scene0 = nullptr;
  RTCScene g_scene1 = nullptr;
  TutorialScene g_tutorial_scene0;
  TutorialScene g_tutorial_scene1;
  std::set<std::pair<unsigned,unsigned>> set0;
  std::set<std::pair<unsigned,unsigned>> set1;

  size_t skipBenchmarkRounds = 0;
  size_t numBenchmarkRounds = 0;
  size_t numTotalCollisions = 0;
  SpinLock mutex;

  void CollideFunc (void* userPtr, RTCCollision* collisions, size_t num_collisions)
  {
    if (numBenchmarkRounds) return;
    numTotalCollisions++;

    Lock<SpinLock> lock(mutex);
    for (size_t i=0; i<num_collisions; i++)
    {
      const unsigned geomID0 = collisions[i].geomID0;
      const unsigned primID0 = collisions[i].primID0;
      const unsigned geomID1 = collisions[i].geomID1;
      const unsigned primID1 = collisions[i].primID1;
      //PRINT4(geomID0,primID0,geomID1,primID1);
      set0.insert(std::make_pair(geomID0,primID0));
      set1.insert(std::make_pair(geomID1,primID1));

      /* verify result */
      Ref<TutorialScene::TriangleMesh> mesh0 = g_tutorial_scene0.geometries[geomID0].dynamicCast<TutorialScene::TriangleMesh>();
      TutorialScene::Triangle tri0 = mesh0->triangles[primID0];
      BBox3fa bounds0 = empty;
      bounds0.extend(mesh0->positions[tri0.v0]);
      bounds0.extend(mesh0->positions[tri0.v1]);
      bounds0.extend(mesh0->positions[tri0.v2]);

      Ref<TutorialScene::TriangleMesh> mesh1 = 
        g_scene1 ? 
        g_tutorial_scene1.geometries[geomID1].dynamicCast<TutorialScene::TriangleMesh>() :
        g_tutorial_scene0.geometries[geomID1].dynamicCast<TutorialScene::TriangleMesh>();

      TutorialScene::Triangle tri1 = mesh1->triangles[primID1];
      BBox3fa bounds1 = empty;
      bounds1.extend(mesh1->positions[tri1.v0]);
      bounds1.extend(mesh1->positions[tri1.v1]);
      bounds1.extend(mesh1->positions[tri1.v2]);

      if (disjoint(bounds0,bounds1)) 
        std::cout << "WARNING: bounds do not overlap!" << std::endl;
    }
  }

  struct Tutorial : public TutorialApplication
  {
    Ref<SceneGraph::Node> scene0;
    Ref<SceneGraph::Node> scene1;

    Tutorial()
      : TutorialApplication("collide",FEATURE_RTCORE)
    {
      rtcore += ",tri_accel=bvh4.triangle4v";

      registerOption("i", [this] (Ref<ParseStream> cin, const FileName& path) {
          FileName filename = path + cin->getFileName();
          Ref<SceneGraph::Node> scene = SceneGraph::load(filename);
          if (scene0 && scene1) throw std::runtime_error("maximally two scenes can get collided");
          if (scene0) scene1 = scene; else scene0 = scene;
        }, "-i <filename>: parses scene from <filename>");

      registerOption("benchmark", [this] (Ref<ParseStream> cin, const FileName& path) {
          skipBenchmarkRounds = cin->getInt();
          numBenchmarkRounds = cin->getInt();
          interactive = false;
        }, "--benchmark <N> <M>: enabled benchmark mode, skips N collisions, measures M collisions");
    }
 
    unsigned int convertTriangleMesh(Ref<TutorialScene::TriangleMesh> mesh, RTCScene scene_out)
    {
      unsigned int geomID = rtcNewTriangleMesh (scene_out, RTC_GEOMETRY_STATIC, mesh->triangles.size(), mesh->numVertices, 1);
      rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions.data(), 0, sizeof(Vec3fa      ));
      rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER , mesh->triangles.data(), 0, sizeof(TutorialScene::Triangle));
      return geomID;
    }
    
    RTCScene convertScene(TutorialScene& scene_in)
    {
      RTCScene scene_out = rtcDeviceNewScene(g_device,RTC_SCENE_STATIC,RTC_INTERSECT1);
      
      for (unsigned int i=0; i<scene_in.geometries.size(); i++)
      {
        if (Ref<TutorialScene::TriangleMesh> mesh = scene_in.geometries[i].dynamicCast<TutorialScene::TriangleMesh>()) {
          unsigned int geomID MAYBE_UNUSED = convertTriangleMesh(mesh, scene_out);
          assert(geomID == i);
        }
        else assert(false);
      }
      
      rtcCommit(scene_out);
      return scene_out;
    }

    void benchmark()
    {
      Statistics stat;
      //FilteredStatistics stat(0.5f,0.0f);
      size_t numTotalRounds = skipBenchmarkRounds + numBenchmarkRounds;
      for (size_t i=0; i<skipBenchmarkRounds; i++) 
      {
        //double t0 = getSeconds();
        numTotalCollisions = 0;
        if (g_scene1) rtcCollide(g_scene0,g_scene1,CollideFunc,nullptr);
        else          rtcCollide(g_scene0,g_scene0,CollideFunc,nullptr);
        //double t1 = getSeconds();
        //float dt = float(t1-t0);
        //std::cout << "round [" << std::setw(3) << i << " / " << std::setw(3) << numTotalRounds << "]: " <<  std::setw(8) << 1000.0f*dt << " ms (skipped)" << std::endl << std::flush;
        //if (benchmarkSleep) sleepSeconds(0.1);
      }
        
      for (size_t i=skipBenchmarkRounds; i<numTotalRounds; i++) 
      {
        double t0 = getSeconds();
        numTotalCollisions = 0;
        if (g_scene1) rtcCollide(g_scene0,g_scene1,CollideFunc,nullptr);
        else          rtcCollide(g_scene0,g_scene0,CollideFunc,nullptr);
        double t1 = getSeconds();
        
        float dt = float(t1-t0);
        stat.add(dt);
        //if (benchmarkSleep) sleepSeconds(0.1);
      }

      std::cout << "Absolute:" << std::endl;
      std::cout << "  min = " << std::setw(8) << 1000.0f*stat.getMin() << " ms, " 
                << "avg = " << std::setw(8) << 1000.0f*stat.getAvg() << " ms, "
                << "max = " << std::setw(8) << 1000.0f*stat.getMax() << " ms " << std::endl;
      std::cout << "Per Collision ( " << numTotalCollisions << " collisions ):" << std::endl;
      std::cout << "  min = " << std::setw(8) << 1E9f*stat.getMin()/float(numTotalCollisions) << " ns, " 
                << "avg = " << std::setw(8) << 1E9f*stat.getAvg()/float(numTotalCollisions) << " ns, "
                << "max = " << std::setw(8) << 1E9f*stat.getMax()/float(numTotalCollisions) << " ns" << std::endl;
    }
        
    int main(int argc, char** argv) try
    {
      /* parse command line options */
      parseCommandLine(argc,argv);

      /* test if user has set two scenes */
      if (!scene0 && !scene1)
        throw std::runtime_error("you have to specify one or two scenes to collide");

      /* convert model */
      g_tutorial_scene0.add(scene0,TutorialScene::INSTANCING_NONE); 
      if (scene1) g_tutorial_scene1.add(scene1,TutorialScene::INSTANCING_NONE); 
      
      Ref<TutorialScene::TriangleMesh> mesh = g_tutorial_scene0.geometries[0].dynamicCast<TutorialScene::TriangleMesh>();

      /* initialize ray tracing core */
      g_device = rtcNewDevice(rtcore.c_str());
      //error_handler(rtcDeviceGetError(g_device));
      //device_init(rtcore.c_str());
      g_scene0 = convertScene(g_tutorial_scene0);
      if (scene1) g_scene1 = convertScene(g_tutorial_scene1);
      g_scene = g_scene0;

      if (numBenchmarkRounds)
        benchmark();
      else if (g_scene1)
        rtcCollide(g_scene0,g_scene1,CollideFunc,nullptr);
      else 
        rtcCollide(g_scene0,g_scene0,CollideFunc,nullptr);

      if (interactive)
        run(argc,argv);
      
      return 0;
    }  
    catch (const std::exception& e) {
      std::cout << "Error: " << e.what() << std::endl;
      return 1;
    }
    catch (...) {
      std::cout << "Error: unknown exception caught." << std::endl;
      return 1;
    }
  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
