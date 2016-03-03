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

#include "../common/tutorial/tutorial.h"
#include "../common/scenegraph/obj_loader.h"
#include "../common/scenegraph/xml_loader.h"
#include "../common/tutorial/scene.h"
#include "../common/image/image.h"
#include "../common/tutorial/tutorial_device.h"

namespace embree
{
  /* name of the tutorial */
  const char* tutorialName = "pathtracer";

  /* configuration */
  static std::string g_rtcore = "";
  static size_t g_numThreads = 0;
  static std::string g_subdiv_mode = "";

  /* output settings */
  static size_t g_width = 512;
  static size_t g_height = 512;
  static bool g_fullscreen = false;
  static FileName outFilename = "";
  static int g_skipBenchmarkFrames = 0;
  static int g_numBenchmarkFrames = 0;
  static bool g_interactive = true;
  static bool g_anim_mode = false;
  extern "C" { int g_instancing_mode = 0; }
  static Shader g_shader = SHADER_DEFAULT;
  static bool convert_tris_to_quads = false;
  static bool convert_bezier_to_lines = false;

  /* scene */
  TutorialScene g_obj_scene;
  Ref<SceneGraph::GroupNode> g_scene = new SceneGraph::GroupNode;
  static FileName filename = "";

  std::vector<FileName> keyframeList;
  std::vector<TutorialScene*> g_keyframes;

  static void parseCommandLine(Ref<ParseStream> cin, const FileName& path)
  {
    while (true)
    {
      std::string tag = cin->getString();
      if (tag == "") return;

      /* parse command line parameters from a file */
      else if (tag == "-c") {
        FileName file = path + cin->getFileName();
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }

      /* load OBJ model*/
      else if (tag == "-i") {
        filename = path + cin->getFileName();
      }

      /* convert triangles to quads */
      else if (tag == "-convert-triangles-to-quads") {
        convert_tris_to_quads = true;
      }

      /* convert bezier to lines */
      else if (tag == "-convert-bezier-to-lines") {
        convert_bezier_to_lines = true;
      }

      /* parse camera parameters */
      else if (tag == "-vp") g_camera.from = cin->getVec3fa();
      else if (tag == "-vi") g_camera.to = cin->getVec3fa();
      else if (tag == "-vd") g_camera.to = g_camera.from + cin->getVec3fa();
      else if (tag == "-vu") g_camera.up = cin->getVec3fa();
      else if (tag == "-fov") g_camera.fov = cin->getFloat();

      /* frame buffer size */
      else if (tag == "-size") {
        g_width = cin->getInt();
        g_height = cin->getInt();
      }

      /* full screen mode */
      else if (tag == "-fullscreen") 
        g_fullscreen = true;

      /* output filename */
      else if (tag == "-o") {
        outFilename = cin->getFileName();
	g_interactive = false;
      }

      else if (tag == "-objlist") {
        while (cin->peek() != "" && cin->peek()[0] != '-')
          keyframeList.push_back(path + cin->getFileName());
      }

      /* subdivision mode */
      else if (tag == "-cache") 
	g_subdiv_mode = ",subdiv_accel=bvh4.subdivpatch1cached";

      else if (tag == "-pregenerate") 
	g_subdiv_mode = ",subdiv_accel=bvh4.grid.eager";
      
      else if (tag == "-anim") 
	g_anim_mode = true;

      else if (tag == "-instancing") {
        std::string mode = cin->getString();
        if      (mode == "none"    ) g_instancing_mode = TutorialScene::INSTANCING_NONE;
        //else if (mode == "geometry") g_instancing_mode = TutorialScene::INSTANCING_GEOMETRY;
        else if (mode == "scene_geometry") g_instancing_mode = TutorialScene::INSTANCING_SCENE_GEOMETRY;
        else if (mode == "scene_group"   ) g_instancing_mode = TutorialScene::INSTANCING_SCENE_GROUP;
        else throw std::runtime_error("unknown instancing mode: "+mode);
      }

      else if (tag == "-shader") {
        std::string mode = cin->getString();
        if      (mode == "default" ) g_shader = SHADER_DEFAULT;
        else if (mode == "eyelight") g_shader = SHADER_EYELIGHT;
        else if (mode == "uv"      ) g_shader = SHADER_UV;
        else if (mode == "Ng"      ) g_shader = SHADER_NG;
        else if (mode == "geomID"  ) g_shader = SHADER_GEOMID;
        else if (mode == "primID"  ) g_shader = SHADER_GEOMID_PRIMID;
        else if (mode == "ao"      ) g_shader = SHADER_AMBIENT_OCCLUSION;
        else throw std::runtime_error("invalid shader:" +mode);
      }

      /* number of frames to render in benchmark mode */
      else if (tag == "-benchmark") {
        g_skipBenchmarkFrames = cin->getInt();
        g_numBenchmarkFrames  = cin->getInt();
	g_interactive = false;
      }

      /* rtcore configuration */
      else if (tag == "-rtcore")
        g_rtcore += "," + cin->getString();

      /* number of threads to use */
      else if (tag == "-threads")
        g_numThreads = cin->getInt();

      /* ambient light source */
      else if (tag == "-ambientlight") 
      {
        const Vec3fa L = cin->getVec3fa();
        g_scene->add(new SceneGraph::LightNode<AmbientLight>(AmbientLight(L)));
      }

      /* point light source */
      else if (tag == "-pointlight") 
      {
        const Vec3fa P = cin->getVec3fa();
        const Vec3fa I = cin->getVec3fa();
        g_scene->add(new SceneGraph::LightNode<PointLight>(PointLight(P,I)));
      }

      /* directional light source */
      else if (tag == "-directionallight" || tag == "-dirlight") 
      {
        const Vec3fa D = cin->getVec3fa();
        const Vec3fa E = cin->getVec3fa();
        g_scene->add(new SceneGraph::LightNode<DirectionalLight>(DirectionalLight(D,E)));
      }

      /* distant light source */
      else if (tag == "-distantlight") 
      {
        const Vec3fa D = cin->getVec3fa();
        const Vec3fa L = cin->getVec3fa();
        const float halfAngle = cin->getFloat();
        g_scene->add(new SceneGraph::LightNode<DistantLight>(DistantLight(D,L,halfAngle)));
      }

      /* skip unknown command line parameter */
      else {
        std::cerr << "unknown command line parameter: " << tag << " ";
        while (cin->peek() != "" && cin->peek()[0] != '-') std::cerr << cin->getString() << " ";
        std::cerr << std::endl;
      }
    }
  }
  
  void renderBenchmark(const FileName& fileName)
  {
    resize(g_width,g_height);
    AffineSpace3fa pixel2world = g_camera.pixel2world(g_width,g_height);

    double dt = 0.0f;
    size_t numTotalFrames = g_skipBenchmarkFrames + g_numBenchmarkFrames;
    for (size_t i=0; i<numTotalFrames; i++) 
    {
      double t0 = getSeconds();
      render(0.0f,pixel2world.l.vx,pixel2world.l.vy,pixel2world.l.vz,pixel2world.p);
      double t1 = getSeconds();
      std::cout << "frame [" << i << " / " << numTotalFrames << "] ";
      std::cout << 1.0/(t1-t0) << "fps ";
      if (i < g_skipBenchmarkFrames) std::cout << "(skipped)";
      std::cout << std::endl;
      if (i >= g_skipBenchmarkFrames) dt += t1-t0;
    }
    std::cout << "frame [" << g_skipBenchmarkFrames << " - " << numTotalFrames << "] " << std::flush;
    std::cout << double(g_numBenchmarkFrames)/dt << "fps " << std::endl;
    std::cout << "BENCHMARK_RENDER " << double(g_numBenchmarkFrames)/dt << std::endl;
  }

  void renderToFile(const FileName& fileName)
  {
    resize(g_width,g_height);
    if (g_anim_mode) g_camera.anim = true;

    double msec = getSeconds();
    AffineSpace3fa pixel2world = g_camera.pixel2world(g_width,g_height);
    render(0.0f,pixel2world.l.vx,pixel2world.l.vy,pixel2world.l.vz,pixel2world.p);
    msec = getSeconds() - msec;
    std::cout << "render time " << 1.0/msec << " fps" << std::endl;

    void* ptr = map();
    Ref<Image> image = new Image4uc(g_width, g_height, (Col4uc*)ptr);
    storeImage(image, fileName);
    unmap();
    cleanup();
  }

  void loadKeyFrameAnimation(std::vector<FileName>& fileName)
  {
    for (size_t i=0; i<fileName.size(); i++)
    {
      std::cout << "." << std::flush;
      TutorialScene *scene = new TutorialScene;
      FileName keyframe = fileName[i];
      scene->add(loadOBJ(keyframe,true),TutorialScene::INSTANCING_NONE);
      g_keyframes.push_back(scene);
    }
    std::cout << std::endl;
  }


  /* main function in embree namespace */
  int main(int argc, char** argv) 
  {
    /* for best performance set FTZ and DAZ flags in MXCSR control and status register */
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    /* create stream for parsing */
    Ref<ParseStream> stream = new ParseStream(new CommandLineStream(argc, argv));

    /* parse command line */  
    parseCommandLine(stream, FileName());

    /* load default scene if none specified */
    if (filename.ext() == "") {
      FileName file = FileName::executableFolder() + FileName("models/cornell_box.ecs");
      parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
    }

    /* configure number of threads */
    if (g_numThreads) 
      g_rtcore += ",threads=" + toString(g_numThreads);
    if (g_numBenchmarkFrames)
      g_rtcore += ",benchmark=1";

    g_rtcore += g_subdiv_mode;

    /* load scene */
    if (toLowerCase(filename.ext()) == std::string("obj")) {
      g_scene->add(loadOBJ(filename,g_subdiv_mode != ""));
    }
    else if (toLowerCase(filename.ext()) == std::string("xml")) {
      g_scene->add(loadXML(filename,one));
    }
    else if (filename.ext() != "")
      THROW_RUNTIME_ERROR("invalid scene type: "+toLowerCase(filename.ext()));

    /* load keyframes */
    if (keyframeList.size())
      loadKeyFrameAnimation(keyframeList);

    /* convert triangles to quads */
    if (convert_tris_to_quads)
        g_scene->triangles_to_quads();

    /* convert bezier to lines */
    if (convert_bezier_to_lines)
        g_scene->bezier_to_lines();

    /* initialize ray tracing core */
    g_obj_scene.add(g_scene.dynamicCast<SceneGraph::Node>(),(TutorialScene::InstancingMode)g_instancing_mode); 
    g_scene = nullptr;
    init(g_rtcore.c_str());

    /* set shader mode */
    switch (g_shader) {
    case SHADER_DEFAULT : break;
    case SHADER_EYELIGHT: key_pressed(GLUT_KEY_F2); break;
    case SHADER_UV      : key_pressed(GLUT_KEY_F4); break;
    case SHADER_NG      : key_pressed(GLUT_KEY_F5); break;
    case SHADER_GEOMID  : key_pressed(GLUT_KEY_F6); break;
    case SHADER_GEOMID_PRIMID: key_pressed(GLUT_KEY_F7); break;
    case SHADER_AMBIENT_OCCLUSION: key_pressed(GLUT_KEY_F11); break;
    };
    
    /* send model */
    set_scene(&g_obj_scene);
    
    /* send keyframes */
    if (g_keyframes.size())
      set_scene_keyframes(&*g_keyframes.begin(),g_keyframes.size());

    /* benchmark mode */
    if (g_numBenchmarkFrames)
      renderBenchmark(outFilename);
    
    /* render to disk */
    if (outFilename.str() != "")
      renderToFile(outFilename);
    
    /* interactive mode */
    if (g_interactive) {
      initWindowState(argc,argv,tutorialName, g_width, g_height, g_fullscreen);
      enterWindowRunLoop(g_anim_mode);

    }

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
