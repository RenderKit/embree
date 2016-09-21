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

namespace embree
{
  RTCDevice g_device = nullptr;
  RTCScene g_scene = nullptr;
  RTCScene g_scene0 = nullptr;
  RTCScene g_scene1 = nullptr;
  TutorialScene g_tutorial_scene0;
  TutorialScene g_tutorial_scene1;

  void CollideFunc (void* userPtr, size_t geomID0, size_t primID0, size_t geomID1, size_t primID1) {
    PRINT4(geomID0,primID0,geomID1,primID1);
  }

  struct Tutorial : public TutorialApplication
  {
    Ref<SceneGraph::Node> scene0;
    Ref<SceneGraph::Node> scene1;

    Tutorial()
      : TutorialApplication("collide",FEATURE_RTCORE) 
    {
      registerOption("i", [this] (Ref<ParseStream> cin, const FileName& path) {
          FileName filename = path + cin->getFileName();
          Ref<SceneGraph::Node> scene = SceneGraph::load(filename);
          if (scene0 && scene1) throw std::runtime_error("maximally two scenes can get collided");
          if (scene0) scene1 = scene; else scene0 = scene;
      }, "-i <filename>: parses scene from <filename>");
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

    int main(int argc, char** argv) try
    {
      /* parse command line options */
      parseCommandLine(argc,argv);

      /* test if user has set two scenes */
      if (!scene0 || !scene1)
         throw std::runtime_error("you have to specify two scenes to collide");

      /* convert model */
      g_tutorial_scene0.add(scene0,TutorialScene::INSTANCING_NONE); 
      g_tutorial_scene1.add(scene1,TutorialScene::INSTANCING_NONE); 
      
      /* initialize ray tracing core */
      g_device = rtcNewDevice(rtcore.c_str());
      //error_handler(rtcDeviceGetError(g_device));
      //device_init(rtcore.c_str());
      g_scene0 = convertScene(g_tutorial_scene0);
      g_scene1 = convertScene(g_tutorial_scene1);
      g_scene = g_scene0;

      /* do collision detection */
      std::cout << "calculating collisions ... " << std::flush;
      rtcCollide(g_scene0,g_scene1,CollideFunc,nullptr);
      std::cout << "[DONE]" << std::endl;

      /* start tutorial */
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
