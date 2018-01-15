// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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
#include "statistics.h"

/* include GLUT for display */
#if defined(__MACOSX__)
#  include <OpenGL/gl.h>
#  include <GLUT/glut.h>
#elif defined(__WIN32__)
#  include <windows.h>
#  define FREEGLUT_LIB_PRAGMAS 0
#  include <GL/gl.h>
#  include <GL/glut.h>
#else
#  include <GL/gl.h>
#  include <GL/glut.h>
#endif

/* include tutorial_device.h after GLUT, as otherwise we may get
 * warnings of redefined GLUT_KEY_F1, etc. */
#include "tutorial_device.h"
#include "../scenegraph/scenegraph.h"
#include "../scenegraph/geometry_creation.h"
#include "../scenegraph/obj_loader.h"
#include "../scenegraph/xml_loader.h"
#include "../image/image.h"

namespace embree
{
  extern "C"
  {
    float g_debug = 0.0f;
    Mode g_mode = MODE_NORMAL;
    ISPCScene* g_ispc_scene = nullptr;

    /* intensity scaling for traversal cost visualization */
    float scale = 1.0f / 1000000.0f;
    bool g_changed = false;

    int64_t get_tsc() { return read_tsc(); }

    unsigned int g_numThreads = 0;

    RTCIntersectFlags g_iflags_coherent = RTC_INTERSECT_COHERENT;
    RTCIntersectFlags g_iflags_incoherent = RTC_INTERSECT_INCOHERENT;

    RayStats* g_stats = nullptr;
  }

  extern "C" int g_instancing_mode;

  TutorialApplication* TutorialApplication::instance = nullptr;

  TutorialApplication::TutorialApplication (const std::string& tutorialName, int features)

    : Application(features),
      tutorialName(tutorialName),

      shader(SHADER_DEFAULT),

      width(512),
      height(512),
      pixels(nullptr),

      outputImageFilename(""),

      skipBenchmarkFrames(0),
      numBenchmarkFrames(0),
      numBenchmarkRepetitions(1),

      interactive(true),
      fullscreen(false),

      window_width(512),
      window_height(512),
      windowID(0),

      time0(getSeconds()),
      debug_int0(0),
      debug_int1(0),

      mouseMode(0),
      clickX(0), clickY(0),
      speed(1.0f),
      moveDelta(zero),
      command_line_camera(false),
      print_frame_rate(false),
      avg_render_time(64,1.0),
      avg_frame_time(64,1.0),
      avg_mrayps(64,1.0),
      print_camera(false),

      debug0(0),
      debug1(0),
      debug2(0),
      debug3(0),

      iflags_coherent(RTC_INTERSECT_COHERENT),
      iflags_incoherent(RTC_INTERSECT_INCOHERENT)
  {
    /* only a single instance of this class is supported */
    assert(instance == nullptr);
    instance = this;

    /* for best performance set FTZ and DAZ flags in MXCSR control and status register */
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    registerOption("c", [this] (Ref<ParseStream> cin, const FileName& path) {
        FileName file = path + cin->getFileName();
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }, "-c <filename>: parses command line option from <filename>");

    registerOption("o", [this] (Ref<ParseStream> cin, const FileName& path) {
        outputImageFilename = cin->getFileName();
        interactive = false;
      }, "-o <filename>: output image filename");

    /* camera settings */
    registerOption("vp", [this] (Ref<ParseStream> cin, const FileName& path) {
        camera.from = cin->getVec3fa();
        command_line_camera = true;
      }, "--vp <float> <float> <float>: camera position");

    registerOption("vi", [this] (Ref<ParseStream> cin, const FileName& path) {
        camera.to = cin->getVec3fa();
        command_line_camera = true;
      }, "--vi <float> <float> <float>: camera lookat position");

    registerOption("vd", [this] (Ref<ParseStream> cin, const FileName& path) {
        camera.to = camera.from + cin->getVec3fa();
        command_line_camera = true;
      }, "--vd <float> <float> <float>: camera direction vector");

    registerOption("vu", [this] (Ref<ParseStream> cin, const FileName& path) {
        camera.up = cin->getVec3fa();
        command_line_camera = true;
      }, "--vu <float> <float> <float>: camera up vector");

    registerOption("fov", [this] (Ref<ParseStream> cin, const FileName& path) {
        camera.fov = cin->getFloat();
        command_line_camera = true;
      }, "--fov <float>: vertical field of view");

    /* framebuffer settings */
    registerOption("size", [this] (Ref<ParseStream> cin, const FileName& path) {
        width = cin->getInt();
        height = cin->getInt();
      }, "--size <width> <height>: sets image size");

    registerOption("fullscreen", [this] (Ref<ParseStream> cin, const FileName& path) {
        fullscreen = true;
      }, "--fullscreen: starts in fullscreen mode");

    registerOption("benchmark", [this] (Ref<ParseStream> cin, const FileName& path) {
        skipBenchmarkFrames = cin->getInt();
        numBenchmarkFrames  = cin->getInt();
        if (cin->peek() != "" && cin->peek()[0] != '-')
          numBenchmarkRepetitions = cin->getInt();
        interactive = false;
        rtcore += ",benchmark=1,start_threads=1";
      }, "--benchmark <N> <M> <R>: enabled benchmark mode, builds scene, skips N frames, renders M frames, and repeats this R times");

    registerOption("nodisplay", [this] (Ref<ParseStream> cin, const FileName& path) {
        skipBenchmarkFrames = 0;
        numBenchmarkFrames  = 2048;
        interactive = false;
      }, "--nodisplay: enabled benchmark mode, continously renders frames");

    registerOption("print-frame-rate", [this] (Ref<ParseStream> cin, const FileName& path) {
        print_frame_rate = true;
      }, "--print-frame-rate: prints framerate for each frame on console");

     registerOption("print-camera", [this] (Ref<ParseStream> cin, const FileName& path) {
         print_camera = true;
      }, "--print-camera: prints camera for each frame on console");

     registerOption("debug0", [this] (Ref<ParseStream> cin, const FileName& path) {
         debug0 = cin->getInt();
       }, "--debug0: sets internal debugging value");

     registerOption("debug1", [this] (Ref<ParseStream> cin, const FileName& path) {
         debug1 = cin->getInt();
       }, "--debug1: sets internal debugging value");

     registerOption("debug2", [this] (Ref<ParseStream> cin, const FileName& path) {
         debug2 = cin->getInt();
       }, "--debug2: sets internal debugging value");

     registerOption("debug3", [this] (Ref<ParseStream> cin, const FileName& path) {
         debug3 = cin->getInt();
       }, "--debug3: sets internal debugging value");

    /* output filename */
    registerOption("shader", [this] (Ref<ParseStream> cin, const FileName& path) {
        std::string mode = cin->getString();
        if      (mode == "default" ) shader = SHADER_DEFAULT;
        else if (mode == "eyelight") shader = SHADER_EYELIGHT;
        else if (mode == "occlusion") shader = SHADER_OCCLUSION;
        else if (mode == "uv"      ) shader = SHADER_UV;
        else if (mode == "texcoords") shader = SHADER_TEXCOORDS;
        else if (mode == "texcoords-grid") shader = SHADER_TEXCOORDS_GRID;
        else if (mode == "Ng"      ) shader = SHADER_NG;
        else if (mode == "cycles"  ) { shader = SHADER_CYCLES; scale = cin->getFloat(); }
        else if (mode == "geomID"  ) shader = SHADER_GEOMID;
        else if (mode == "primID"  ) shader = SHADER_GEOMID_PRIMID;
        else if (mode == "ao"      ) shader = SHADER_AMBIENT_OCCLUSION;
        else throw std::runtime_error("invalid shader:" +mode);
      },
      "--shader <string>: sets shader to use at startup\n"
      "  default: default tutorial shader\n"
      "  eyelight: eyelight shading\n"
      "  occlusion: occlusion shading\n"
      "  uv: uv debug shader\n"
      "  texcoords: texture coordinate debug shader\n"
      "  texcoords-grid: grid texture debug shader\n"
      "  Ng: visualization of shading normal\n"
      "  cycles <float>: CPU cycle visualization\n"
      "  geomID: visualization of geometry ID\n"
      "  primID: visualization of geometry and primitive ID\n"
      "  ao: ambient occlusion shader");

    if (features & FEATURE_STREAM)
    {
      /* register parsing of stream mode */
      registerOption("mode", [] (Ref<ParseStream> cin, const FileName& path) {
          std::string mode = cin->getString();
          if      (mode == "normal") g_mode = MODE_NORMAL;
          else if (mode == "stream") g_mode = MODE_STREAM;
          else throw std::runtime_error("invalid mode:" +mode);
        },
        "--mode: sets rendering mode\n"
        "  normal  : normal mode\n"
        "  stream  : stream mode\n");
    }

    registerOption("coherent", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_iflags_coherent   = iflags_coherent   = RTC_INTERSECT_COHERENT;
        g_iflags_incoherent = iflags_incoherent = RTC_INTERSECT_COHERENT;
      }, "--coherent: force using RTC_INTERSECT_COHERENT hint when tracing rays");

    registerOption("incoherent", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_iflags_coherent   = iflags_coherent   = RTC_INTERSECT_INCOHERENT;
        g_iflags_incoherent = iflags_incoherent = RTC_INTERSECT_INCOHERENT;
      }, "--incoherent: force using RTC_INTERSECT_INCOHERENT hint when tracing rays");
  }

  TutorialApplication::~TutorialApplication()
  {
    g_ispc_scene = nullptr;
    device_cleanup();
    alignedFree(pixels);
    pixels = nullptr;
    width = 0;
    height = 0;
    alignedFree(g_stats);
    g_stats = nullptr;
  }

  SceneLoadingTutorialApplication::SceneLoadingTutorialApplication (const std::string& tutorialName, int features)

    : TutorialApplication(tutorialName, features),
      scene(new SceneGraph::GroupNode),
      convert_tris_to_quads(false),
      convert_tris_to_quads_prop(inf),
      convert_bezier_to_lines(false),
      convert_hair_to_curves(false),
      convert_bezier_to_bspline(false),
      convert_bspline_to_bezier(false),
      remove_mblur(false),
      remove_non_mblur(false),
      sceneFilename(""),
      instancing_mode(SceneGraph::INSTANCING_NONE),
      print_scene_cameras(false)
  {
    registerOption("i", [this] (Ref<ParseStream> cin, const FileName& path) {
        sceneFilename = path + cin->getFileName();
      }, "-i <filename>: parses scene from <filename>");

    registerOption("animlist", [this] (Ref<ParseStream> cin, const FileName& path) {
        FileName listFilename = path + cin->getFileName();

        std::ifstream listFile;
        listFile.open(listFilename.c_str());
        if (!listFile.is_open()) {
          THROW_RUNTIME_ERROR("cannot open " + listFilename.str());
        }
        else
        {
          while (!listFile.eof())
          {
            std::string line;
            listFile >> line;
            if (line != "")
              keyFramesFilenames.push_back(listFilename.path() + line);
          }
        }
      }, "-animlist <filename>: parses a sequence of .obj/.xml files listed in <filename> and adds them to the scene");

    registerOption("convert-triangles-to-quads", [this] (Ref<ParseStream> cin, const FileName& path) {
        convert_tris_to_quads = true;
        convert_tris_to_quads_prop = inf;
      }, "--convert-triangles-to-quads: converts all triangles to quads when loading");

    registerOption("convert-triangles-to-triangles-and-quads", [this] (Ref<ParseStream> cin, const FileName& path) {
        convert_tris_to_quads = true;
        convert_tris_to_quads_prop = 0.5f;
      }, "--convert-triangles-to-triangles-and-quads: converts to mixed triangle/quad scene");

    registerOption("convert-bezier-to-lines", [this] (Ref<ParseStream> cin, const FileName& path) {
        convert_bezier_to_lines = true;
      }, "--convert-bezier-to-lines: converts all bezier curves to line segments when loading");

    registerOption("convert-hair-to-curves", [this] (Ref<ParseStream> cin, const FileName& path) {
        convert_hair_to_curves = true;
      }, "--convert-hair-to-curves: converts all hair geometry to curves when loading");

    registerOption("convert-bezier-to-bspline", [this] (Ref<ParseStream> cin, const FileName& path) {
        convert_bezier_to_bspline = true;
      }, "--convert-bezier-to-bspline: converts all bezier curves to bsplines curves");

    registerOption("convert-bspline-to-bezier", [this] (Ref<ParseStream> cin, const FileName& path) {
        convert_bspline_to_bezier = true;
      }, "--convert-bspline-to-bezier: converts all bsplines curves to bezier curves");

    registerOption("remove-mblur", [this] (Ref<ParseStream> cin, const FileName& path) {
         remove_mblur = true;
      }, "--remove-mblur: removes all motion blur geometry");

    registerOption("remove-non-mblur", [this] (Ref<ParseStream> cin, const FileName& path) {
         remove_non_mblur = true;
      }, "--remove-non-mblur: removes all non-motion blur geometry");

    registerOption("instancing", [this] (Ref<ParseStream> cin, const FileName& path) {
        std::string mode = cin->getString();
        if      (mode == "none"    ) instancing_mode = SceneGraph::INSTANCING_NONE;
        else if (mode == "geometry") instancing_mode = SceneGraph::INSTANCING_GEOMETRY;
        else if (mode == "geometry_group") instancing_mode = SceneGraph::INSTANCING_GEOMETRY_GROUP;
        else if (mode == "scene_geometry") instancing_mode = SceneGraph::INSTANCING_SCENE_GEOMETRY;
        else if (mode == "scene_group"   ) instancing_mode = SceneGraph::INSTANCING_SCENE_GROUP;
        else throw std::runtime_error("unknown instancing mode: "+mode);
        g_instancing_mode = instancing_mode;
      }, "--instancing: set instancing mode\n"
      "  none: no instancing\n"
      "  geometry: instance individual geometries\n"
      "  geometry_group: instance geometry groups\n"
      "  scene_geometry: instance individual geometries as scenes\n"
      "  scene_group: instance geometry groups as scenes");

    registerOption("ambientlight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa L = cin->getVec3fa();
        scene->add(new SceneGraph::LightNode(new SceneGraph::AmbientLight(L)));
      }, "--ambientlight r g b: adds an ambient light with intensity rgb");
    registerOptionAlias("ambientlight","ambient");

    registerOption("pointlight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa P = cin->getVec3fa();
        const Vec3fa I = cin->getVec3fa();
        scene->add(new SceneGraph::LightNode(new SceneGraph::PointLight(P,I)));
      }, "--pointlight x y z r g b: adds a point light at position xyz with intensity rgb");

    registerOption("directionallight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa D = cin->getVec3fa();
        const Vec3fa E = cin->getVec3fa();
        scene->add(new SceneGraph::LightNode(new SceneGraph::DirectionalLight(D,E)));
      }, "--directionallight x y z r g b: adds a directional light with direction xyz and intensity rgb");
    registerOptionAlias("directionallight","dirlight");

    registerOption("distantlight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa D = cin->getVec3fa();
        const Vec3fa L = cin->getVec3fa();
        const float halfAngle = cin->getFloat();
        scene->add(new SceneGraph::LightNode(new SceneGraph::DistantLight(D,L,halfAngle)));
      }, "--distantlight x y z r g b a: adds a distant light with direction xyz, intensity rgb, and opening angle a");

    registerOption("triangle-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa p0 = cin->getVec3fa();
        const Vec3fa dx = cin->getVec3fa();
        const Vec3fa dy = cin->getVec3fa();
        const size_t width = cin->getInt();
        const size_t height = cin->getInt();
        scene->add(SceneGraph::createTrianglePlane(p0,dx,dy,width,height,new OBJMaterial));
      }, "--triangle-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z width height: adds a plane build of triangles originated at p0 and spanned by the vectors dx and dy with a tesselation width/height.");

    registerOption("quad-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa p0 = cin->getVec3fa();
        const Vec3fa dx = cin->getVec3fa();
        const Vec3fa dy = cin->getVec3fa();
        const size_t width = cin->getInt();
        const size_t height = cin->getInt();
        scene->add(SceneGraph::createQuadPlane(p0,dx,dy,width,height,new OBJMaterial));
      }, "--quad-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z width height: adds a plane build of quadrilaterals originated at p0 and spanned by the vectors dx and dy with a tesselation width/height.");

    registerOption("subdiv-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa p0 = cin->getVec3fa();
        const Vec3fa dx = cin->getVec3fa();
        const Vec3fa dy = cin->getVec3fa();
        const size_t width = cin->getInt();
        const size_t height = cin->getInt();
        const float tessellationRate = cin->getFloat();
        scene->add(SceneGraph::createSubdivPlane(p0,dx,dy,width,height,tessellationRate,new OBJMaterial));
      }, "--subdiv-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z width height tessellationRate: adds a plane build as a Catmull Clark subdivision surface originated at p0 and spanned by the vectors dx and dy. The plane consists of widt x height many patches, and each patch has the specified tesselation rate.");

    registerOption("hair-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa p0 = cin->getVec3fa();
        const Vec3fa dx = cin->getVec3fa();
        const Vec3fa dy = cin->getVec3fa();
        const float len = cin->getFloat();
        const float r = cin->getFloat();
        const size_t N = cin->getInt();
        scene->add(SceneGraph::createHairyPlane(0,p0,dx,dy,len,r,N,SceneGraph::HairSetNode::HAIR,new OBJMaterial));
      }, "--hair-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z length radius num: adds a hair plane originated at p0 and spanned by the vectors dx and dy. num hairs are generated with speficied length and radius.");

    registerOption("curve-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa p0 = cin->getVec3fa();
        const Vec3fa dx = cin->getVec3fa();
        const Vec3fa dy = cin->getVec3fa();
        const float len = cin->getFloat();
        const float r = cin->getFloat();
        const size_t N = cin->getInt();
        scene->add(SceneGraph::createHairyPlane(0,p0,dx,dy,len,r,N,SceneGraph::HairSetNode::CURVE,new OBJMaterial));
      }, "--curve-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z length radius: adds a plane build of bezier curves originated at p0 and spanned by the vectors dx and dy. num curves are generated with speficied length and radius.");

    registerOption("triangle-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa p = cin->getVec3fa();
        const float  r = cin->getFloat();
        const size_t numPhi = cin->getInt();
        scene->add(SceneGraph::createTriangleSphere(p,r,numPhi,new OBJMaterial));
      }, "--triangle-sphere p.x p.y p.z r numPhi: adds a sphere at position p with radius r and tesselation numPhi build of triangles.");

    registerOption("quad-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa p = cin->getVec3fa();
        const float  r = cin->getFloat();
        const size_t numPhi = cin->getInt();
        scene->add(SceneGraph::createQuadSphere(p,r,numPhi,new OBJMaterial));
      }, "--quad-sphere p.x p.y p.z r numPhi: adds a sphere at position p with radius r and tesselation numPhi build of quadrilaterals.");

    registerOption("quad-sphere-mblur", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa p = cin->getVec3fa();
        const Vec3fa dp = cin->getVec3fa();
        const float  r = cin->getFloat();
        const size_t numPhi = cin->getInt();
        Ref<SceneGraph::Node> mesh = SceneGraph::createQuadSphere(p,r,numPhi,new OBJMaterial);
        mesh->set_motion_vector(dp); 
        scene->add(mesh);
      }, "--quad-sphere-mb p.x p.y p.z d.x d.y d.z r numPhi : adds a motion blurred sphere build of quadrilaterals at position p, with motion vector d, radius r, and tesselation numPhi.");

    registerOption("subdiv-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa p = cin->getVec3fa();
        const float  r = cin->getFloat();
        const size_t numPhi = cin->getInt();
        const float tessellationRate = cin->getFloat();
        scene->add(SceneGraph::createSubdivSphere(p,r,numPhi,tessellationRate,new OBJMaterial));
      }, "--subdiv-sphere p.x p.y p.z r numPhi: adds a sphere at position p with radius r build of Catmull Clark subdivision surfaces. The sphere consists of numPhi x numPhi many patches and each path has the specified tessellation rate.");

    registerOption("cache", [this] (Ref<ParseStream> cin, const FileName& path) {
        subdiv_mode = ",subdiv_accel=bvh4.subdivpatch1cached";
        rtcore += subdiv_mode;
      }, "--cache: enabled cached subdiv mode");

    registerOption("pregenerate", [this] (Ref<ParseStream> cin, const FileName& path) {
        subdiv_mode = ",subdiv_accel=bvh4.grid.eager";
        rtcore += subdiv_mode;
      }, "--pregenerate: enabled pregenerate subdiv mode");

    registerOption("print-cameras", [this] (Ref<ParseStream> cin, const FileName& path) {
        print_scene_cameras = true;
      }, "--print-cameras: prints all camera names of the scene");

    registerOption("camera", [this] (Ref<ParseStream> cin, const FileName& path) {
        camera_name = cin->getString();
      }, "--camera: use camera with specified name");
  }

  void TutorialApplication::initRayStats()
  {
    if (!g_stats)
      g_stats = (RayStats*)alignedMalloc(TaskScheduler::threadCount() * sizeof(RayStats), sizeof(RayStats));

    for (size_t i = 0; i < TaskScheduler::threadCount(); i++)
      g_stats[i].numRays = 0;
  }

  int64_t TutorialApplication::getNumRays()
  {
    int64_t numRays = 0;
    for (size_t i = 0; i < TaskScheduler::threadCount(); i++)
      numRays += g_stats[i].numRays;
    return numRays;
  }

  void TutorialApplication::renderBenchmark()
  {
    IOStreamStateRestorer cout_state(std::cout);
    std::cout.setf(std::ios::fixed, std::ios::floatfield);
    std::cout.precision(4);

    resize(width,height);
    ISPCCamera ispccamera = camera.getISPCCamera(width,height);

    //Statistics stat;
    FilteredStatistics fpsStat(0.5f,0.0f);
    FilteredStatistics mraypsStat(0.5f,0.0f);
    for (size_t j=0; j<numBenchmarkRepetitions; j++)
    {
      size_t numTotalFrames = skipBenchmarkFrames + numBenchmarkFrames;
      for (size_t i=0; i<skipBenchmarkFrames; i++)
      {
        initRayStats();
        double t0 = getSeconds();
        device_render(pixels,width,height,0.0f,ispccamera);
        double t1 = getSeconds();
        std::cout << "frame [" << std::setw(3) << i << " / " << std::setw(3) << numTotalFrames << "]: " <<  std::setw(8) << 1.0/(t1-t0) << " fps (skipped)" << std::endl << std::flush;
      }

      for (size_t i=skipBenchmarkFrames; i<numTotalFrames; i++)
      {
        initRayStats();
        double t0 = getSeconds();
        device_render(pixels,width,height,0.0f,ispccamera);
        double t1 = getSeconds();

        float fps = float(1.0/(t1-t0));
        fpsStat.add(fps);

        float mrayps = float(double(getNumRays())/(1000000.0*(t1-t0)));
        mraypsStat.add(mrayps);

        if (numTotalFrames >= 1024 && (i % 64 == 0))
        {
          std::cout << "frame [" << std::setw(3) << i << " / " << std::setw(3) << numTotalFrames << "]: "
                    << std::setw(8) << fps << " fps, "
                    << "min = " << std::setw(8) << fpsStat.getMin() << " fps, "
                    << "avg = " << std::setw(8) << fpsStat.getAvg() << " fps, "
                    << "max = " << std::setw(8) << fpsStat.getMax() << " fps, "
                    << "sigma = " << std::setw(6) << fpsStat.getSigma() << " (" << 100.0f*fpsStat.getSigma()/fpsStat.getAvg() << "%)" << std::endl << std::flush;
        }
      }

      /* rebuild scene between repetitions */
      if (numBenchmarkRepetitions)
      {
        device_cleanup();
        device_init(rtcore.c_str());
        resize(width,height);
      }

      std::cout << "frame [" << std::setw(3) << skipBenchmarkFrames << " - " << std::setw(3) << numTotalFrames << "]: "
                << "              "
                << "min = " << std::setw(8) << fpsStat.getMin() << " fps, "
                << "avg = " << std::setw(8) << fpsStat.getAvg() << " fps, "
                << "max = " << std::setw(8) << fpsStat.getMax() << " fps, "
                << "sigma = " << std::setw(6) << fpsStat.getAvgSigma() << " (" << 100.0f*fpsStat.getAvgSigma()/fpsStat.getAvg() << "%)" << std::endl;
    }

    std::cout << "BENCHMARK_RENDER_MIN " << fpsStat.getMin() << std::endl;
    std::cout << "BENCHMARK_RENDER_AVG " << fpsStat.getAvg() << std::endl;
    std::cout << "BENCHMARK_RENDER_MAX " << fpsStat.getMax() << std::endl;
    std::cout << "BENCHMARK_RENDER_SIGMA " << fpsStat.getSigma() << std::endl;
    std::cout << "BENCHMARK_RENDER_AVG_SIGMA " << fpsStat.getAvgSigma() << std::endl;

#if defined(RAY_STATS)
    std::cout << "BENCHMARK_RENDER_MRAYPS_MIN " << mraypsStat.getMin() << std::endl;
    std::cout << "BENCHMARK_RENDER_MRAYPS_AVG " << mraypsStat.getAvg() << std::endl;
    std::cout << "BENCHMARK_RENDER_MRAYPS_MAX " << mraypsStat.getMax() << std::endl;
    std::cout << "BENCHMARK_RENDER_MRAYPS_SIGMA " << mraypsStat.getSigma() << std::endl;
    std::cout << "BENCHMARK_RENDER_MRAYPS_AVG_SIGMA " << mraypsStat.getAvgSigma() << std::endl;
#endif

    std::cout << std::flush;
  }

  void TutorialApplication::renderToFile(const FileName& fileName)
  {
    resize(width,height);
    ISPCCamera ispccamera = camera.getISPCCamera(width,height);
    initRayStats();
    device_render(pixels,width,height,0.0f,ispccamera);
    Ref<Image> image = new Image4uc(width, height, (Col4uc*)pixels);
    storeImage(image, fileName);
  }

  void TutorialApplication::set_parameter(size_t parm, ssize_t val) {
    rtcDeviceSetParameter1i(nullptr,(RTCParameter)parm,val);
  }

  void TutorialApplication::resize(unsigned width, unsigned height)
  {
    if (width == this->width && height == this->height && pixels)
      return;

    if (pixels) alignedFree(pixels);
    this->width = width;
    this->height = height;
    pixels = (unsigned*) alignedMalloc(width*height*sizeof(unsigned),64);
  }

  void TutorialApplication::set_scene (TutorialScene* in)
  {
    ispc_scene.reset(new ISPCScene(in));
    g_ispc_scene = ispc_scene.get();
  }

  void TutorialApplication::keyboardFunc(unsigned char key, int x, int y)
  {
    /* call tutorial keyboard handler */
    device_key_pressed(key);

    switch (key)
    {
    case 'w' : moveDelta.z = +1.0f; break;
    case 's' : moveDelta.z = -1.0f; break;
    case 'a' : moveDelta.x = -1.0f; break;
    case 'd' : moveDelta.x = +1.0f; break;

    case 'f' :
      if (fullscreen) {
        fullscreen = false;
        glutReshapeWindow(window_width,window_height);
      } else {
        fullscreen = true;
        window_width = width;
        window_height = height;
        glutFullScreen();
      }
      break;
    case 'c' : std::cout << camera.str() << std::endl; break;
    case '+' : g_debug=clamp(g_debug+0.01f); PRINT(g_debug); break;
    case '-' : g_debug=clamp(g_debug-0.01f); PRINT(g_debug); break;

    case ' ' : {
      Ref<Image> image = new Image4uc(width, height, (Col4uc*)pixels, true, "", true);
      storeImage(image, "screenshot.tga");
      break;
    }

    case '\033': case 'q': case 'Q':
      glutDestroyWindow(windowID);
#if defined(__MACOSX__) // FIXME: why only on MacOSX
      exit(1);
#endif
      break;
    }
  }

  void TutorialApplication::keyboardUpFunc(unsigned char key, int x, int y)
  {
    switch (key)
    {
    case 'w' : moveDelta.z = 0.0f; break;
    case 's' : moveDelta.z = 0.0f; break;
    case 'a' : moveDelta.x = 0.0f; break;
    case 'd' : moveDelta.x = 0.0f; break;
    }
  }

  void TutorialApplication::specialFunc(int key, int x, int y)
  {
    device_key_pressed(key);

    if (glutGetModifiers() == GLUT_ACTIVE_CTRL)
    {
      switch (key) {
      case GLUT_KEY_UP        : debug_int0++; set_parameter(1000000,debug_int0); PRINT(debug_int0); break;
      case GLUT_KEY_DOWN      : debug_int0--; set_parameter(1000000,debug_int0); PRINT(debug_int0); break;
      case GLUT_KEY_LEFT      : debug_int1--; set_parameter(1000001,debug_int1); PRINT(debug_int1); break;
      case GLUT_KEY_RIGHT     : debug_int1++; set_parameter(1000001,debug_int1); PRINT(debug_int1); break;
      }
    }
    else
    {
      switch (key) {
      case GLUT_KEY_LEFT      : camera.rotate(-0.02f,0.0f); break;
      case GLUT_KEY_RIGHT     : camera.rotate(+0.02f,0.0f); break;
      case GLUT_KEY_UP        : camera.move(0.0f,0.0f,+speed); break;
      case GLUT_KEY_DOWN      : camera.move(0.0f,0.0f,-speed); break;
      case GLUT_KEY_PAGE_UP   : speed *= 1.2f; break;
      case GLUT_KEY_PAGE_DOWN : speed /= 1.2f; break;
      }
    }
  }

  void TutorialApplication::clickFunc(int button, int state, int x, int y)
  {
    if (state == GLUT_UP)
    {
      mouseMode = 0;
    }
    else
    {
      if (button == GLUT_RIGHT_BUTTON)
      {
        ISPCCamera ispccamera = camera.getISPCCamera(width,height);
        Vec3fa p; bool hit = device_pick(float(x),float(y),ispccamera,p);

        if (hit) {
          Vec3fa delta = p - camera.to;
          Vec3fa right = normalize(ispccamera.xfm.l.vx);
          Vec3fa up    = normalize(ispccamera.xfm.l.vy);
          camera.to = p;
          camera.from += dot(delta,right)*right + dot(delta,up)*up;
        }
      }
      else
      {
        clickX = x; clickY = y;
        int modifiers = glutGetModifiers();
        if      (button == GLUT_LEFT_BUTTON && modifiers == GLUT_ACTIVE_SHIFT) mouseMode = 1;
        else if (button == GLUT_LEFT_BUTTON && modifiers == GLUT_ACTIVE_CTRL ) mouseMode = 3;
        else if (button == GLUT_LEFT_BUTTON) mouseMode = 4;
      }
    }
  }

  void TutorialApplication::motionFunc(int x, int y)
  {
    float dClickX = float(clickX - x), dClickY = float(clickY - y);
    clickX = x; clickY = y;

    switch (mouseMode) {
    case 1: camera.rotateOrbit(-0.005f*dClickX,0.005f*dClickY); break;
    case 2: break;
    case 3: camera.dolly(-dClickY); break;
    case 4: camera.rotate(-0.005f*dClickX,0.005f*dClickY); break;
    }
  }

  void TutorialApplication::displayFunc()
  {
    /* update camera */
    camera.move(moveDelta.x*speed, moveDelta.y*speed, moveDelta.z*speed);
    ISPCCamera ispccamera = camera.getISPCCamera(width,height,true);
     if (print_camera)
      std::cout << camera.str() << std::endl;

    /* render image using ISPC */
    initRayStats();
    double t0 = getSeconds();
    device_render(pixels,width,height,float(time0-t0),ispccamera);
    double dt0 = getSeconds()-t0;
    avg_render_time.add(dt0);
    double mrayps = double(getNumRays())/(1000000.0*dt0);
    avg_mrayps.add(mrayps);

    /* draw pixels to screen */
    glDrawPixels(width,height,GL_RGBA,GL_UNSIGNED_BYTE,pixels);

    if (fullscreen || !print_frame_rate)
    {
      glMatrixMode( GL_PROJECTION );
      glPushMatrix();
      glLoadIdentity();
      gluOrtho2D( 0, width, 0, height );
      glMatrixMode( GL_MODELVIEW );
      glPushMatrix();
      glLoadIdentity();

      /* print frame rate */
      glColor3f(1.0f, 0.25f, 0.0f);

      std::ostringstream stream;
      stream.setf(std::ios::fixed, std::ios::floatfield);
      stream.precision(2);

      stream << 1.0f/avg_render_time.get() << " fps";
      std::string str = stream.str();
      glRasterPos2i(6, height - 24);
      for (size_t i=0; i<str.size(); i++)
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, str[i]);

#if defined(RAY_STATS)
      stream.str("");
      stream << avg_mrayps.get() << " Mray/s";
      str = stream.str();
      glRasterPos2i(6, height - 52);
      for (size_t i=0; i<str.size(); i++)
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, str[i]);
#endif

      glRasterPos2i( 0, 0 );
      glPopMatrix();
      glMatrixMode( GL_PROJECTION );
      glPopMatrix();
      glMatrixMode( GL_MODELVIEW );
    }

    glutSwapBuffers();

    double dt1 = getSeconds()-t0;
    avg_frame_time.add(dt1);

    if (print_frame_rate)
    {
      std::ostringstream stream;
      stream.setf(std::ios::fixed, std::ios::floatfield);
      stream.precision(2);
      stream << "render: ";
      stream << 1.0f/dt0 << " fps, ";
      stream << dt0*1000.0f << " ms, ";
#if defined(RAY_STATS)
      stream << mrayps << " Mray/s, ";
#endif
      stream << "display: ";
      stream << 1.0f/dt1 << " fps, ";
      stream << dt1*1000.0f << " ms, ";
      stream << width << "x" << height << " pixels";
      std::cout << stream.str() << std::endl;
    } 
  }

  void TutorialApplication::reshapeFunc(int width, int height)
  {
    resize(width,height);
    glViewport(0, 0, width, height);
    this->width = width; this->height = height;
  }

  void TutorialApplication::idleFunc() {
    glutPostRedisplay();
  }

  void keyboardFunc(unsigned char key, int x, int y) {
    TutorialApplication::instance->keyboardFunc(key,x,y);
  }
  void keyboardUpFunc(unsigned char key, int x, int y) {
    TutorialApplication::instance->keyboardUpFunc(key,x,y);
  }
  void specialFunc(int key, int x, int y) {
    TutorialApplication::instance->specialFunc(key,x,y);
  }
  void clickFunc(int button, int state, int x, int y) {
    TutorialApplication::instance->clickFunc(button,state,x,y);
  }
  void motionFunc(int x, int y) {
    TutorialApplication::instance->motionFunc(x,y);
  }
  void displayFunc() {
    TutorialApplication::instance->displayFunc();
  }
  void reshapeFunc(int width, int height) {
    TutorialApplication::instance->reshapeFunc(width,height);
  }
  void idleFunc() {
    TutorialApplication::instance->idleFunc();
  }

  void TutorialApplication::run(int argc, char** argv)
  {
    /* set debug values */
    rtcDeviceSetParameter1i(nullptr,(RTCParameter) 1000000, debug0);
    rtcDeviceSetParameter1i(nullptr,(RTCParameter) 1000001, debug1);
    rtcDeviceSetParameter1i(nullptr,(RTCParameter) 1000002, debug2);
    rtcDeviceSetParameter1i(nullptr,(RTCParameter) 1000003, debug3);

    /* initialize ray tracing core */
    device_init(rtcore.c_str());

    /* set shader mode */
    switch (shader) {
    case SHADER_DEFAULT  : break;
    case SHADER_EYELIGHT : device_key_pressed(GLUT_KEY_F2); break;
    case SHADER_OCCLUSION: device_key_pressed(GLUT_KEY_F3); break;
    case SHADER_UV       : device_key_pressed(GLUT_KEY_F4); break;
    case SHADER_TEXCOORDS: device_key_pressed(GLUT_KEY_F8); break;
    case SHADER_TEXCOORDS_GRID: device_key_pressed(GLUT_KEY_F8); device_key_pressed(GLUT_KEY_F8); break;
    case SHADER_NG       : device_key_pressed(GLUT_KEY_F5); break;
    case SHADER_CYCLES   : device_key_pressed(GLUT_KEY_F9); break;
    case SHADER_GEOMID   : device_key_pressed(GLUT_KEY_F6); break;
    case SHADER_GEOMID_PRIMID: device_key_pressed(GLUT_KEY_F7); break;
    case SHADER_AMBIENT_OCCLUSION: device_key_pressed(GLUT_KEY_F11); break;
    };

    /* benchmark mode */
    if (numBenchmarkFrames)
    {
      renderBenchmark();
    }

    /* render to disk */
    if (outputImageFilename.str() != "")
      renderToFile(outputImageFilename);

    /* interactive mode */
    if (interactive)
    {
      resize(width,height);

      glutInit(&argc, argv);
      glutInitWindowSize((GLsizei)width, (GLsizei)height);
      glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
      glutInitWindowPosition(0, 0);
      windowID = glutCreateWindow(tutorialName.c_str());
      if (fullscreen) glutFullScreen();
      glutDisplayFunc(embree::displayFunc);
      glutIdleFunc(embree::idleFunc);
      glutKeyboardFunc(embree::keyboardFunc);
      glutKeyboardUpFunc(embree::keyboardUpFunc);
      glutSpecialFunc(embree::specialFunc);
      glutMouseFunc(embree::clickFunc);
      glutMotionFunc(embree::motionFunc);
      glutReshapeFunc(embree::reshapeFunc);
      glutMainLoop();
    }
  }

  int TutorialApplication::main(int argc, char** argv) try
  {
    /* parse command line options */
    parseCommandLine(argc,argv);

    /* callback */
    postParseCommandLine();

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

  int SceneLoadingTutorialApplication::main(int argc, char** argv) try
  {
    /* parse command line options */
    parseCommandLine(argc,argv);

    /* callback */
    try {
      postParseCommandLine();
    }
    catch (const std::exception& e) {
      std::cout << "Error: " << e.what() << std::endl;
    }

    /* load scene */
    if (sceneFilename != "")
    {
      if (toLowerCase(sceneFilename.ext()) == std::string("obj"))
        scene->add(loadOBJ(sceneFilename,subdiv_mode != ""));
      else if (sceneFilename.ext() != "")
        scene->add(SceneGraph::load(sceneFilename));
    }

    /* load key frames for animation */
    for (size_t i=0;i<keyFramesFilenames.size();i++)
    {
      std::cout << "Adding ["<< keyFramesFilenames[i] << "] to scene..." << std::flush;
      if (toLowerCase(keyFramesFilenames[i].ext()) == std::string("obj"))
        scene->add(loadOBJ(keyFramesFilenames[i],subdiv_mode != "",true));
      else if (keyFramesFilenames[i].ext() != "")
        scene->add(SceneGraph::load(keyFramesFilenames[i]));
      std::cout << "done" << std::endl << std::flush;
    }

    /* clear texture cache */
    Texture::clearTextureCache();

    /* perform removals */
    if (remove_mblur)     scene->remove_mblur(true);
    if (remove_non_mblur) scene->remove_mblur(false);

    /* perform conversions */
    if (convert_tris_to_quads    ) scene->triangles_to_quads(convert_tris_to_quads_prop);
    if (convert_bezier_to_lines  ) scene->bezier_to_lines();
    if (convert_hair_to_curves   ) scene->hair_to_curves();
    if (convert_bezier_to_bspline) scene->bezier_to_bspline();
    if (convert_bspline_to_bezier) scene->bspline_to_bezier();

    /* convert model */
    obj_scene.add(SceneGraph::flatten(scene,instancing_mode));
    scene = nullptr;

    /* print all cameras */
    if (print_scene_cameras) {
      obj_scene.print_camera_names();
      return 0;
    }

    /* use specified camera */
    if (camera_name != "") {
      Ref<SceneGraph::PerspectiveCameraNode> c = obj_scene.getCamera(camera_name);
      camera = Camera(c->from,c->to,c->up,c->fov);
    }

    /* otherwise use default camera */
    else if (!command_line_camera) {
      Ref<SceneGraph::PerspectiveCameraNode> c = obj_scene.getDefaultCamera();
      if (c) camera = Camera(c->from,c->to,c->up,c->fov);
    }

    /* send model */
    set_scene(&obj_scene);

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

  /* draws progress bar */
  static int progressWidth = 0;
  static std::atomic<size_t> progressDots(0);

  extern "C" void progressStart()
  {
    progressDots = 0;
    progressWidth = max(3,getTerminalWidth());
    std::cout << "[" << std::flush;
  }

  extern "C" bool progressMonitor(void* ptr, const double n)
  {
    size_t olddots = progressDots;
    size_t maxdots = progressWidth-2;
    size_t newdots = max(olddots,min(size_t(maxdots),size_t(n*double(maxdots))));
    if (progressDots.compare_exchange_strong(olddots,newdots))
      for (size_t i=olddots; i<newdots; i++) std::cout << "." << std::flush;
    return true;
  }

  extern "C" void progressEnd() {
    std::cout << "]" << std::endl;
  }
}
