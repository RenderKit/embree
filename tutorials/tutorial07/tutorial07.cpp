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
#include "tutorial/cy_hair_loader.h"
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
  static int tessellate_strips = -1;
  extern float g_reduce_hair_segment_error;

  /* scene */
  OBJScene g_obj_scene;
  static FileName objFilename = "";
  static FileName hairFilename = "";
  static FileName cy_hairFilename = "";
  static FileName outFilename = "";
  static int g_numFrames = 1;
  static int g_skipFrames = 0;
  static bool hairy_triangles = false;
  static float hairy_triangles_length = 1.0f;
  static float hairy_triangles_thickness = 0.1f;
  static int hairy_triangles_strands_per_triangle = 1;

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

      /* load hair model */
      else if (tag == "--cy_hair") {
        cy_hairFilename = path + cin->getFileName();
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
      else if (tag == "--tessellate-hair") {
        tessellate_subdivisions = cin->getInt();
        tessellate_strips = cin->getInt();
      }

      /* load hair model */
      else if (tag == "--hairy-triangles") {
        hairy_triangles_strands_per_triangle = cin->getInt();
        hairy_triangles_length = cin->getFloat();
        hairy_triangles_thickness = cin->getFloat();
        DBG_PRINT(hairy_triangles_length);
        DBG_PRINT(hairy_triangles_strands_per_triangle);
        DBG_PRINT(hairy_triangles_thickness);
        hairy_triangles = true;
      }

      /* reduce number of hair segments */
      else if (tag == "--reduce-hair-segment-error") {
        g_reduce_hair_segment_error = cin->getFloat();
      }

      /* output filename */
      else if (tag == "-o") {
        outFilename = cin->getFileName();
      }

      /* number of frames to render in benchmark mode */
      else if (tag == "-frames") {
        g_skipFrames = cin->getInt();
        g_numFrames  = cin->getInt();
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

  void addHairSegment(OBJScene::Mesh& mesh, const Vec3fa& p0, const Vec3fa& p1)
  {
    const size_t N = tessellate_strips;
    if (length(p1-p0) <= 1E-6f) return;
    const LinearSpace3fa xfm = frame(p1-p0);
    const int base = mesh.v.size();
    for (size_t i=0; i<N; i++) {
      const float a = 2.0f*float(pi)*float(i)/float(N);
      const float u = sinf(a), v = cosf(a);
      mesh.v.push_back(p0+u*p0.w*xfm.vx+v*p0.w*xfm.vy);
      mesh.v.push_back(p1+u*p1.w*xfm.vx+v*p1.w*xfm.vy);
    }
    for (size_t i=0; i<N; i++) {
      const int v0 = base + (2*i+0)%(2*N);
      const int v1 = base + (2*i+1)%(2*N);
      const int v2 = base + (2*i+2)%(2*N);
      const int v3 = base + (2*i+3)%(2*N);
      mesh.triangles.push_back(OBJScene::Triangle(v0,v1,v2,0));
      mesh.triangles.push_back(OBJScene::Triangle(v1,v3,v2,0));
    }
  }

  void tessellateHair(OBJScene::Mesh& mesh, 
                      const Vec3fa& p00,
                      const Vec3fa& p01,
                      const Vec3fa& p02,
                      const Vec3fa& p03,
                      const int depth)
  {
    if (depth > 0) 
    {
      const Vec3fa p10 = 0.5f*(p00+p01);
      const Vec3fa p11 = 0.5f*(p01+p02);
      const Vec3fa p12 = 0.5f*(p02+p03);
      const Vec3fa p20 = 0.5f*(p10+p11);
      const Vec3fa p21 = 0.5f*(p11+p12);
      const Vec3fa p30 = 0.5f*(p20+p21);
      tessellateHair(mesh,p00,p10,p20,p30,depth-1);
      tessellateHair(mesh,p30,p21,p12,p03,depth-1);
    } else {
      addHairSegment(mesh,p00,p03);
    }
  }

  void tessellateHair(OBJScene::Mesh& mesh, const OBJScene::HairSet& hairSet)
  {
    for (size_t i=0; i<hairSet.hairs.size(); i++) 
    {
      OBJScene::Hair hair = hairSet.hairs[i];
      const Vec3fa p00 = hairSet.v[hair.vertex+0];
      const Vec3fa p01 = hairSet.v[hair.vertex+1];
      const Vec3fa p02 = hairSet.v[hair.vertex+2];
      const Vec3fa p03 = hairSet.v[hair.vertex+3];
      tessellateHair(mesh,p00,p01,p02,p03,tessellate_subdivisions);
    }
  }

  void tessellateHair(OBJScene& scene)
  {
    for (int i=0; i<scene.hairsets.size(); i++) {
      OBJScene::Mesh* mesh = new OBJScene::Mesh;
      OBJScene::HairSet* hairset = scene.hairsets[i];
      tessellateHair(*mesh,*hairset);
      scene.meshes.push_back(mesh);
      delete hairset;
    }
    scene.hairsets.clear();
  }


  void generateHairOnTriangles(OBJScene& scene)
  {
    PING;
    OBJScene::HairSet* hairset = new OBJScene::HairSet;

    for (size_t m=0; m<scene.meshes.size(); m++) 
      for (size_t t=0; t<scene.meshes[m]->triangles.size(); t++) 
        {
          const Vec3fa &v0 = scene.meshes[m]->v[ scene.meshes[m]->triangles[t].v0 ];
          const Vec3fa &v1 = scene.meshes[m]->v[ scene.meshes[m]->triangles[t].v1 ];
          const Vec3fa &v2 = scene.meshes[m]->v[ scene.meshes[m]->triangles[t].v2 ];
          
          const Vec3fa edge0 = v1-v0;
          const Vec3fa edge1 = v2-v0;
          const Vec3fa normal = normalize(cross(edge0,edge1));
          LinearSpace3<Vec3fa> local_frame = frame(normal);

          const float length = hairy_triangles_length;
          const Vec3fa &dx = normalize(local_frame.vx);
          const Vec3fa &dy = normalize(local_frame.vy);
          const Vec3fa &dz = normalize(local_frame.vz) * length;

          const float thickness = hairy_triangles_thickness;

          for (size_t i=0;i<hairy_triangles_strands_per_triangle;i++)
            {
 
              float ru = drand48();
              float rv = drand48();
              float delta = drand48();
              const Vec3fa position = (1.0f - sqrtf(ru)) * v0 + (sqrtf(ru) * (1.0f - rv)) * v1 + (sqrtf(ru) * rv) * v2;




              const Vec3fa p0(   0, 0,0);
              const Vec3fa p1(0.25, delta,0);
              const Vec3fa p2(0.75,-delta,0);
              const Vec3fa p3(   1, 0,0);

              Vec3fa l0 = position + p0.x * dz + p0.y * dx + p0.z * dy;
              Vec3fa l1 = position + p1.x * dz + p1.y * dx + p1.z * dy;
              Vec3fa l2 = position + p2.x * dz + p2.y * dx + p2.z * dy;
              Vec3fa l3 = position + p3.x * dz + p3.y * dx + p3.z * dy;

          
              l0.w = thickness;
              l1.w = thickness;
              l2.w = thickness;
              l3.w = thickness;

              const unsigned int v_index = hairset->v.size();
              hairset->v.push_back(l0);
              hairset->v.push_back(l1);
              hairset->v.push_back(l2);
              hairset->v.push_back(l3);

              hairset->hairs.push_back( OBJScene::Hair(v_index,hairset->hairs.size()) );
            }
        }
    scene.hairsets.push_back(hairset);
  }

  void renderToFile(const FileName& fileName)
  {
    resize(g_width,g_height);
    AffineSpace3fa pixel2world = g_camera.pixel2world(g_width,g_height);

    for (size_t i=0; i<g_skipFrames; i++) 
      render(0.0f,
             pixel2world.l.vx,
             pixel2world.l.vy,
             pixel2world.l.vz,
             pixel2world.p);

    double dt = 0.0f;
    for (size_t i=0; i<g_numFrames; i++) 
    {
      double t0 = getSeconds();
      render(0.0f,
             pixel2world.l.vx,
             pixel2world.l.vy,
             pixel2world.l.vz,
             pixel2world.p);
      dt += getSeconds()-t0;
    }
    if (g_numFrames > 1) 
      std::cout << "BENCHMARK_RENDER " << double(g_numFrames)/dt << std::endl;
    
    void* ptr = map();
    Ref<Image> image = new Image4c(g_width, g_height, (Col4c*)ptr);
    storeImage(image, fileName);
    unmap();
  }

  /* main function in embree namespace */
  int main(int argc, char** argv) 
  {
    g_camera.from = Vec3fa(3.21034f,0.320831f,-0.162478f);
    g_camera.to   = Vec3fa(2.57003f,0.524887f, 0.163145f);

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
    if (objFilename.str() != "" && objFilename.str() != "none")
      loadOBJ(objFilename,g_obj_scene,offset);

    /* load hair */
    if (hairFilename.str() != "" && hairFilename.str() != "none") {
      loadHair(hairFilename,g_obj_scene,offset);
    }

   /* load cy_hair */
    if (cy_hairFilename.str() != "") {
      loadCYHair(cy_hairFilename,g_obj_scene,offset);
    }

    /* tessellate hair */
    if (tessellate_strips > 0) 
      tessellateHair(g_obj_scene);

    /* send model */
    if (objFilename.str() != "" || hairFilename.str() != "" || cy_hairFilename.str() != "")
      {
        if (hairy_triangles && objFilename.str() != "")
          generateHairOnTriangles(g_obj_scene);

        set_scene(&g_obj_scene);
      }

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
