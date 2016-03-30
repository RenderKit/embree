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

#include "tutorial.h"
#include "scene.h"
#include "tutorial_device.h"
#include "../scenegraph/obj_loader.h"
#include "../scenegraph/xml_loader.h"
#include "../image/image.h"

namespace embree
{
  TutorialApplication::TutorialApplication (const std::string& tutorialName)

    : tutorialName(tutorialName),
      g_rtcore(""),
      g_numThreads(0),
      g_subdiv_mode(""),
      g_width(512), g_height(512), g_fullscreen(false),
      outFilename(""),
      g_skipBenchmarkFrames(0),
      g_numBenchmarkFrames(0),
      g_interactive(true),
      g_instancing_mode(0),
      g_shader(SHADER_DEFAULT),
      convert_tris_to_quads(false),
      convert_bezier_to_lines(false),
      convert_hair_to_curves(false),
      g_scene(new SceneGraph::GroupNode),
      filename("")
  {
     /* for best performance set FTZ and DAZ flags in MXCSR control and status register */
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
   
    registerOption("help", [this] (Ref<ParseStream> cin, const FileName& path) {
        printCommandLineHelp();
        exit(1);
      }, "--help: prints help for all supported command line options");

    registerOption("c", [this] (Ref<ParseStream> cin, const FileName& path) {
        FileName file = path + cin->getFileName();
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }, "-c <filename>: parses command line option from <filename>");
    
    registerOption("i", [this] (Ref<ParseStream> cin, const FileName& path) {
        filename = path + cin->getFileName();
      }, "-i <filename>: parses scene from <filename>");
    
    registerOption("o", [this] (Ref<ParseStream> cin, const FileName& path) {
        outFilename = cin->getFileName();
        g_interactive = false;
      }, "-o: output image filename");
    
    registerOption("convert-triangles-to-quads", [this] (Ref<ParseStream> cin, const FileName& path) {
        convert_tris_to_quads = true;
      }, "--convert-triangles-to-quads: converts all triangles to quads when loading");
    
    registerOption("convert-bezier-to-lines", [this] (Ref<ParseStream> cin, const FileName& path) {
        convert_bezier_to_lines = true;
      }, "--convert-bezier-to-lines: converts all bezier curves to line segments when loading");
    
    registerOption("convert-hair-to-curves", [this] (Ref<ParseStream> cin, const FileName& path) {
        convert_hair_to_curves = true;
      }, "--convert-hair-to-curves: converts all hair geometry to curves when loading");
    
    /* camera settings */
    registerOption("vp", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_camera.from = cin->getVec3fa();
      }, "--vp <float> <float> <float>: camera position");
    
    registerOption("vi", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_camera.to = cin->getVec3fa();
      }, "--vi <float> <float> <float>: camera lookat position");
    
    registerOption("vd", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_camera.to = g_camera.from + cin->getVec3fa();
      }, "--vd <float> <float> <float>: camera direction vector");
    
    registerOption("vu", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_camera.up = cin->getVec3fa();
      }, "--vu <float> <float> <float>: camera up vector");
    
    registerOption("fov", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_camera.fov = cin->getFloat();
      }, "--fov <float>: vertical field of view");
    
    /* framebuffer settings */
    registerOption("size", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_width = cin->getInt();
        g_height = cin->getInt();
      }, "--size <width> <height>: image size");
    
    registerOption("fullscreen", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_fullscreen = true;
      }, "--fullscreen: starts in fullscreen mode");
    
    registerOption("rtcore", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_rtcore += "," + cin->getString();
      }, "--rtcore <string>: uses <string> to configure Embree device");
    
    registerOption("threads", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_numThreads = cin->getInt();
        g_rtcore += ",threads=" + toString(g_numThreads);
      }, "--threads <int>: number of threads to use");
    
    registerOption("benchmark", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_skipBenchmarkFrames = cin->getInt();
        g_numBenchmarkFrames  = cin->getInt();
        g_interactive = false;
        g_rtcore += ",benchmark=1";
      }, "--benchmark <N> <M>: enabled benchmark mode, skips N frames, renders M frames ");
    
    /* output filename */
    registerOption("shader", [this] (Ref<ParseStream> cin, const FileName& path) {
        std::string mode = cin->getString();
        if      (mode == "default" ) g_shader = SHADER_DEFAULT;
        else if (mode == "eyelight") g_shader = SHADER_EYELIGHT;
        else if (mode == "uv"      ) g_shader = SHADER_UV;
        else if (mode == "Ng"      ) g_shader = SHADER_NG;
        else if (mode == "geomID"  ) g_shader = SHADER_GEOMID;
        else if (mode == "primID"  ) g_shader = SHADER_GEOMID_PRIMID;
        else if (mode == "ao"      ) g_shader = SHADER_AMBIENT_OCCLUSION;
        else throw std::runtime_error("invalid shader:" +mode);
      }, 
      "--shader <string>: sets shader to use at startup\n"
      "  default: default tutorial shader\n"
      "  eyelight: eyelight shading\n"
      "  uv: uv debug shader\n"
      "  Ng: visualization of shading normal\n"
      "  geomID: visualization of geometry ID\n"
      "  primID: visualization of geometry and primitive ID\n"
      "  ao: ambient occlusion shader");
    
    
    /*else if (tag == "-objlist") {
      while (cin->peek() != "" && cin->peek()[0] != '-')
      keyframeList.push_back(path + cin->getFileName());
      }*/
    
    /* subdivision mode */
    //else if (tag == "-cache") 
    //g_subdiv_mode = ",subdiv_accel=bvh4.subdivpatch1cached";
    
    //else if (tag == "-pregenerate") 
//	g_subdiv_mode = ",subdiv_accel=bvh4.grid.eager";
    
    /*else if (tag == "-instancing") {
      std::string mode = cin->getString();
      if      (mode == "none"    ) g_instancing_mode = TutorialScene::INSTANCING_NONE;
      //else if (mode == "geometry") g_instancing_mode = TutorialScene::INSTANCING_GEOMETRY;
      else if (mode == "scene_geometry") g_instancing_mode = TutorialScene::INSTANCING_SCENE_GEOMETRY;
      else if (mode == "scene_group"   ) g_instancing_mode = TutorialScene::INSTANCING_SCENE_GROUP;
      else throw std::runtime_error("unknown instancing mode: "+mode);
      }*/
    
    
    
    
    
#if 0
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
#endif
  }

  void TutorialApplication::parseCommandLine(int argc, char** argv)
  {
    /* create stream for parsing */
    Ref<ParseStream> stream = new ParseStream(new CommandLineStream(argc, argv));
    
    /* parse command line */  
    parseCommandLine(stream, FileName());

    //g_rtcore += g_subdiv_mode;
  }

  void TutorialApplication::parseCommandLine(Ref<ParseStream> cin, const FileName& path)
  {
    while (true)
    {
      std::string tag = cin->getString();
      if (tag == "") return;

      /* remove - or -- and lookup command line option */
      if (tag.find("-") == 0) tag = tag.substr(1);
      if (tag.find("-") == 0) tag = tag.substr(1);
      auto option = commandLineOptionMap.find(tag);

      /* process command line option */
      if (option != commandLineOptionMap.end()) {
        option->second->parse(cin,path);
      }
      
      /* handle unknown command line options */
      else
      {
        std::cerr << "unknown command line parameter: " << tag << " ";
        while (cin->peek() != "" && cin->peek()[0] != '-') std::cerr << cin->getString() << " ";
        std::cerr << std::endl;
      }    
    }
  }

  void TutorialApplication::printCommandLineHelp()
  {
    for (auto c : commandLineOptionList) {
      std::cout << c->description << std::endl;
    }
  }

  void TutorialApplication::renderBenchmark(const FileName& fileName)
  {
    resize(g_width,g_height);
    ISPCCamera camera = g_camera.getISPCCamera(g_width,g_height);

    double dt = 0.0f;
    size_t numTotalFrames = g_skipBenchmarkFrames + g_numBenchmarkFrames;
    for (size_t i=0; i<numTotalFrames; i++) 
    {
      double t0 = getSeconds();
      render(0.0f,camera);
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

  void TutorialApplication::renderToFile(const FileName& fileName)
  {
    resize(g_width,g_height);
    ISPCCamera camera = g_camera.getISPCCamera(g_width,g_height);
    render(0.0f,camera);
    void* ptr = map();
    Ref<Image> image = new Image4uc(g_width, g_height, (Col4uc*)ptr);
    storeImage(image, fileName);
    unmap();
    cleanup();
  }

  
  int TutorialApplication::main(int argc, char** argv) try
  {
    /* parse command line options */
    parseCommandLine(argc,argv);

    /* load scene */
    if (toLowerCase(filename.ext()) == std::string("obj"))
      g_scene->add(loadOBJ(filename,g_subdiv_mode != ""));
    else if (filename.ext() != "")
      g_scene->add(SceneGraph::load(filename));

    /* convert triangles to quads */
    if (convert_tris_to_quads)
      g_scene->triangles_to_quads();
    
    /* convert bezier to lines */
    if (convert_bezier_to_lines)
      g_scene->bezier_to_lines();
    
    /* convert hair to curves */
    if (convert_hair_to_curves)
      g_scene->hair_to_curves();
    
    /* convert model */
    g_obj_scene.add(g_scene.dynamicCast<SceneGraph::Node>(),(TutorialScene::InstancingMode)g_instancing_mode); 
    g_scene = nullptr;

    /* send model */
    set_scene(&g_obj_scene);
    
    /* initialize ray tracing core */
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
    
    /* benchmark mode */
    if (g_numBenchmarkFrames)
      renderBenchmark(outFilename);
    
    /* render to disk */
    if (outFilename.str() != "")
      renderToFile(outFilename);
    
    /* interactive mode */
    if (g_interactive) {
      initWindowState(argc,argv,tutorialName, g_width, g_height, g_fullscreen);
      enterWindowRunLoop();
    }

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
}
