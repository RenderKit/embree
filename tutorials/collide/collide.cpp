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
  std::shared_ptr<TutorialScene> g_tutorial_scene = nullptr;
  size_t cur_time = 0;
  std::vector<std::shared_ptr<TutorialScene>> g_animation;
  std::set<std::pair<unsigned,unsigned>> collision_candidates;
  //RTCScene g_scene0 = nullptr;
  //RTCScene g_scene1 = nullptr;
  //TutorialScene g_tutorial_scene0;
  //TutorialScene g_tutorial_scene1;
  //std::set<std::pair<unsigned,unsigned>> set0;
  //std::set<std::pair<unsigned,unsigned>> set1;

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
      collision_candidates.insert(std::make_pair(geomID0,primID0));
      collision_candidates.insert(std::make_pair(geomID1,primID1));
      //set0.insert(std::make_pair(geomID0,primID0));
      //set1.insert(std::make_pair(geomID1,primID1));

      /* verify result */
      Ref<TutorialScene::TriangleMesh> mesh0 = g_tutorial_scene->geometries[geomID0].dynamicCast<TutorialScene::TriangleMesh>();
      TutorialScene::Triangle tri0 = mesh0->triangles[primID0];
      BBox3fa bounds0 = empty;
      bounds0.extend(mesh0->positions[tri0.v0]);
      bounds0.extend(mesh0->positions[tri0.v1]);
      bounds0.extend(mesh0->positions[tri0.v2]);

      Ref<TutorialScene::TriangleMesh> mesh1 = g_tutorial_scene->geometries[geomID1].dynamicCast<TutorialScene::TriangleMesh>();
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
    bool pause;

    Tutorial()
      : TutorialApplication("collide",FEATURE_RTCORE), pause(false)
    {
      registerOption("i", [this] (Ref<ParseStream> cin, const FileName& path) {
          FileName filename = path + cin->getFileName();
          Ref<SceneGraph::Node> scene = SceneGraph::load(filename);
          std::shared_ptr<TutorialScene> tscene(new TutorialScene);
          tscene->add(scene,TutorialScene::INSTANCING_NONE); 
          g_animation.push_back(tscene);
        }, "-i <filename>: parses scene from <filename>");

      registerOption("benchmark", [this] (Ref<ParseStream> cin, const FileName& path) {
          skipBenchmarkRounds = cin->getInt();
          numBenchmarkRounds = cin->getInt();
          interactive = false;
        }, "--benchmark <N> <M>: enabled benchmark mode, skips N collisions, measures M collisions");
    }
 
    unsigned int convertTriangleMesh(Ref<TutorialScene::TriangleMesh> mesh, RTCScene scene_out)
    {
      RTCGeometryFlags gflags;
      if (g_animation.size() == 1) gflags = RTC_GEOMETRY_STATIC;
      else                         gflags = RTC_GEOMETRY_DYNAMIC;
      unsigned int geomID = rtcNewTriangleMesh (scene_out, gflags, mesh->triangles.size(), mesh->numVertices, 1);
      rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions.data(), 0, sizeof(Vec3fa      ));
      rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER , mesh->triangles.data(), 0, sizeof(TutorialScene::Triangle));
      return geomID;
    }
    
    RTCScene convertScene(std::shared_ptr<TutorialScene> scene_in)
    {
      RTCSceneFlags sflags;
      if (g_animation.size() == 1) sflags = RTC_SCENE_STATIC;
      else                         sflags = RTC_SCENE_DYNAMIC;
      RTCScene scene_out = rtcDeviceNewScene(g_device,sflags,RTC_INTERSECT1);
      
      for (unsigned int i=0; i<scene_in->geometries.size(); i++)
      {
        if (Ref<TutorialScene::TriangleMesh> mesh = scene_in->geometries[i].dynamicCast<TutorialScene::TriangleMesh>()) {
          unsigned int geomID MAYBE_UNUSED = convertTriangleMesh(mesh, scene_out);
          assert(geomID == i);
        }
        else assert(false);
      }
      
      rtcCommit(scene_out);
      return scene_out;
    }

    void updateTriangleMesh(Ref<TutorialScene::TriangleMesh> mesh, RTCScene scene_out, unsigned geomID)
    {
      rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions.data(), 0, sizeof(Vec3fa      ));
      rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER , mesh->triangles.data(), 0, sizeof(TutorialScene::Triangle));
      rtcUpdate(scene_out, geomID);
    }

    void updateScene(std::shared_ptr<TutorialScene> scene_in, RTCScene scene_out)
    {
      for (unsigned int i=0; i<scene_in->geometries.size(); i++)
      {
        if (Ref<TutorialScene::TriangleMesh> mesh = scene_in->geometries[i].dynamicCast<TutorialScene::TriangleMesh>()) {
          updateTriangleMesh(mesh, scene_out, i);
        }
        else assert(false);
      }
      
      rtcCommit(scene_out);
    }

    void updateScene()
    {
      g_tutorial_scene = g_animation[cur_time];
      cur_time = (cur_time+1)%g_animation.size();

      if (g_scene == nullptr)  
        g_scene = convertScene(g_tutorial_scene);
      else if (g_animation.size() > 1)
        updateScene(g_tutorial_scene,g_scene);
    }

    void keyboardFunc(unsigned char key, int x, int y)
    {
      if (key == ' ') pause = !pause;
      else TutorialApplication::keyboardFunc(key,x,y);
    }

    void render(unsigned* pixels, const unsigned width, const unsigned height, const float time, const ISPCCamera& camera) 
    {
      if (!pause) updateScene();
      collision_candidates.clear();
      rtcCollide(g_scene,g_scene,CollideFunc,nullptr);
      device_render(pixels,width,height,time,camera);
    }

    void benchmark()
    {
      Statistics stat0;
      Statistics stat1;
      size_t numTotalRounds = skipBenchmarkRounds + numBenchmarkRounds;
      for (size_t i=0; i<skipBenchmarkRounds; i++) 
      {
        updateScene();
        //double t0 = getSeconds();
        numTotalCollisions = 0;
        rtcCollide(g_scene,g_scene,CollideFunc,nullptr);
        //double t1 = getSeconds();
        //float dt = float(t1-t0);
        //std::cout << "round [" << std::setw(3) << i << " / " << std::setw(3) << numTotalRounds << "]: " <<  std::setw(8) << 1000.0f*dt << " ms (skipped)" << std::endl << std::flush;
        //if (benchmarkSleep) sleepSeconds(0.1);
      }
        
      for (size_t i=skipBenchmarkRounds; i<numTotalRounds; i++) 
      {
        double t0 = getSeconds();
        updateScene();
        double t1 = getSeconds();
        numTotalCollisions = 0;
        rtcCollide(g_scene,g_scene,CollideFunc,nullptr);
        double t2 = getSeconds();
        
        float dt0 = float(t1-t0);
        float dt1 = float(t2-t1);
        stat0.add(dt0);
        stat1.add(dt1);
        //if (benchmarkSleep) sleepSeconds(0.1);
      }

      std::cout << "Scene Update:        min = " << std::setw(8) << 1000.0f*stat0.getMin() << " ms, " 
                << "avg = " << std::setw(8) << 1000.0f*stat0.getAvg() << " ms, "
                << "max = " << std::setw(8) << 1000.0f*stat0.getMax() << " ms " << std::endl;
      std::cout << "Collision Detection: min = " << std::setw(8) << 1000.0f*stat1.getMin() << " ms, " 
                << "avg = " << std::setw(8) << 1000.0f*stat1.getAvg() << " ms, "
                << "max = " << std::setw(8) << 1000.0f*stat1.getMax() << " ms " << std::endl;
      /*std::cout << "Per Collision ( " << numTotalCollisions << " collisions ):" << std::endl;
      std::cout << "  min = " << std::setw(8) << 1E9f*stat.getMin()/float(numTotalCollisions) << " ns, " 
                << "avg = " << std::setw(8) << 1E9f*stat.getAvg()/float(numTotalCollisions) << " ns, "
                << "max = " << std::setw(8) << 1E9f*stat.getMax()/float(numTotalCollisions) << " ns" << std::endl;*/
    }
        
    int main(int argc, char** argv) try
    {
      /* parse command line options */
      parseCommandLine(argc,argv);

      /* test if user has set two scenes */
      if (g_animation.size() == 0)
        throw std::runtime_error("you have to specify at least one scene");

      /* initialize ray tracing core */
      rtcore += ",tri_accel=bvh4.triangle4v";
      if (g_animation.size() > 1) rtcore += ",tri_builder=morton";
      g_device = rtcNewDevice(rtcore.c_str());
      //error_handler(rtcDeviceGetError(g_device));

      if (numBenchmarkRounds)
        benchmark();

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
