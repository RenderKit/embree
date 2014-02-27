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

embree::Vec3fa g_dirlight_direction = embree::normalize(embree::Vec3fa(1,-1,1));
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

  static int tessellate_subdivisions = -1;
  static int tessellate_strips   = -1;
  extern float g_reduce_hair_segment_error;

  /* scene */
  OBJScene g_obj_scene;
  static FileName objFilename = "";
  static FileName hairFilename = "";
  static FileName outFilename = "";

  Vec3fa offset = 0.0f;

  static void tessellateHair(OBJScene &scene);

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
        tessellate_subdivisions = cin->getInt();
        tessellate_strips   = cin->getInt();
      }

      /* reduce number of hair segments */
      else if (tag == "--reduce-hair-segment-error") {
        g_reduce_hair_segment_error = cin->getFloat();
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

  void addHairSegment(OBJScene &scene, 
                      const Vec3fa &p00,
                      const Vec3fa &p01)
  {
    /* todo */
  }
  void tessellateHair(OBJScene &scene, 
                      const Vec3fa &p00,
                      const Vec3fa &p01,
                      const Vec3fa &p02,
                      const Vec3fa &p03,
                      int subdivisions)
  {
    if (subdivisions > 0) {
      const Vec3fa p10 = 0.5f*(p00+p01);
      const Vec3fa p11 = 0.5f*(p01+p02);
      const Vec3fa p12 = 0.5f*(p02+p03);

      const Vec3fa p20 = 0.5f*(p10+p11);
      const Vec3fa p21 = 0.5f*(p11+p12);

      const Vec3fa p30 = 0.5f*(p20+p21);
      
      tessellateHair(scene,p00,p10,p20,p30,subdivisions-1);
      tessellateHair(scene,p30,p21,p12,p03,subdivisions-1);
    } else {
      addHairSegment(scene,p00,p01);
      addHairSegment(scene,p01,p02);
      addHairSegment(scene,p02,p03);
    }
  }
  void tessellateHair(OBJScene &scene, 
                      const OBJScene::HairSet &hairSet)
  {
    for (int i=0;i<hairSet.hairs.size();i++) {
      OBJScene::Hair hair = hairSet.hairs[i];
      Vec3fa p00 = hairSet.v[hair.vertex+0];
      Vec3fa p01 = hairSet.v[hair.vertex+1];
      Vec3fa p02 = hairSet.v[hair.vertex+2];
      Vec3fa p03 = hairSet.v[hair.vertex+3];
      tessellateHair(scene,p00,p01,p02,p03,tessellate_subdivisions);
    }
  }
  void tessellateHair(OBJScene &scene)
  {
    for (int i=0;i<scene.hairsets.size();i++) 
      tessellateHair(scene,*scene.hairsets[i]);
    scene.hairsets.clear();
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
    if (hairFilename.str() != "") {
      loadHair(hairFilename,g_obj_scene,offset);
      if (tessellate_subdivisions > 0 && tessellate_strips > 0)
        tessellateHair(g_obj_scene);
    }

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
