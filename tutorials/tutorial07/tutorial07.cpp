// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "tutorial/tutorial.h"
#include "tutorial/obj_loader.h"
#include "tutorial/hair_loader.h"
#include "sys/taskscheduler.h"
#include "image/image.h"

embree::Vec3fa g_dirlight_direction = embree::normalize(embree::Vec3fa(1,0,1));
embree::Vec3fa g_dirlight_intensity = embree::Vec3fa(1.0f);
embree::Vec3fa g_ambient_intensity = embree::Vec3fa(1.0f);

namespace embree
{
  /* name of the tutorial */
  const char* tutorialName = "tutorial07";

  /* configuration */
  static std::string g_rtcore = "";

  /* output settings */
  static size_t g_width = 512;
  static size_t g_height = 512;
  static bool g_fullscreen = false;
  static size_t g_numThreads = 0;

  static int tessellation_segments = -1;
  static int tessellation_strips   = -1;

  /* scene */
  OBJScene g_obj_scene;
  static FileName objFilename = "";
  static FileName hairFilename = "";
  static FileName outFilename = "";

  Vec3fa offset = 0.0f;

  static void parseCommandLine(Ref<ParseStream> cin, const FileName& path)
  {
    while (true)
    {
      std::string tag = cin->getString();
      if (tag == "") return;

      /* parse command line parameters from a file */
      if (tag == "-c") {
        FileName file = path + cin->getFileName();
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }

      /* load OBJ model */
      else if (tag == "-i") {
        objFilename = path + cin->getFileName();
      }

      /* load hair model */
      else if (tag == "--hair") {
        hairFilename = path + cin->getFileName();
      }

      /* scene offset */
      else if (tag == "--offset") {
        offset = cin->getVec3fa();
      }

      /* directional light */
      else if (tag == "--dirlight") {
        g_dirlight_direction = normalize(cin->getVec3fa());
        g_dirlight_intensity = cin->getVec3fa();
      }

      /* ambient light */
      else if (tag == "--ambient") {
        g_ambient_intensity = cin->getVec3fa();
      }

      /* tessellation flags */
      else if (tag == "--tessellate") {
        tessellation_segments = cin->getInt();
        tessellation_strips   = cin->getInt();
      }

      /* output filename */
      else if (tag == "-o") {
        outFilename = cin->getFileName();
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
      
      /* rtcore configuration */
      else if (tag == "-rtcore")
        g_rtcore = cin->getString();

      /* number of threads to use */
      else if (tag == "-threads")
        g_numThreads = cin->getInt();

      /* skip unknown command line parameter */
      else {
        std::cerr << "unknown command line parameter: " << tag << " ";
        while (cin->peek() != "" && cin->peek()[0] != '-') std::cerr << cin->getString() << " ";
        std::cerr << std::endl;
      }
    }
  }

  void renderToFile(const FileName& fileName)
  {
    resize(g_width,g_height);
    AffineSpace3fa pixel2world = g_camera.pixel2world(g_width,g_height);

    /* render image using ISPC */
    double t0 = getSeconds();
    render(0.0f,
           pixel2world.l.vx,
           pixel2world.l.vy,
           pixel2world.l.vz,
           pixel2world.p);
    double dt0 = getSeconds()-t0;

    void* ptr = map();
    Ref<Image> image = new Image4c(g_width, g_height, (Col4c*)ptr);
    storeImage(image, fileName);
    unmap();
  }

  /* main function in embree namespace */
  int main(int argc, char** argv) 
  {
    /* create stream for parsing */
    Ref<ParseStream> stream = new ParseStream(new CommandLineStream(argc, argv));

    /* parse command line */  
    parseCommandLine(stream, FileName());
    if (g_numThreads) 
      g_rtcore += ",threads=" + std::stringOf(g_numThreads);

    /* initialize task scheduler */
#if !defined(__EXPORT_ALL_SYMBOLS__)
    TaskScheduler::create(g_numThreads);
#endif

    /* initialize ray tracing core */
    init(g_rtcore.c_str());

    /* load scene */
    if (objFilename.str() != "")
      loadOBJ(objFilename,g_obj_scene,offset);

    /* load hair */
    if (hairFilename.str() != "")
      loadHair(hairFilename,g_obj_scene,offset);

    /* send model */
    if (objFilename.str() != "" && hairFilename.str() != "")
      set_scene(&g_obj_scene);

    /* render to disk */
    if (outFilename.str() != "") {
      renderToFile(outFilename);
      return 0;
    } 

    /* initialize GLUT */
    initGlut(tutorialName,g_width,g_height,g_fullscreen,true);
    
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
