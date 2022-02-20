// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "tutorial.h"
#include "scene.h"
#include "statistics.h"

#if defined(USE_GLFW)

/* include GL */
#if defined(__MACOSX__)
#  include <OpenGL/gl.h>
#elif defined(__WIN32__)
#  include <windows.h>
#  include <GL/gl.h>
#else
#  include <GL/gl.h>
#endif

#endif

#include "tutorial_device.h"
#include "../scenegraph/scenegraph.h"
#include "../scenegraph/geometry_creation.h"
#include "../scenegraph/obj_loader.h"
#include "../scenegraph/xml_loader.h"
#include "../image/image.h"

namespace embree
{
  /* access to debug shader render frame functions */
  typedef void (* renderFrameFunc)(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  renderFrameFunc renderFrame;
  
  extern "C" void renderFrameStandard(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  extern "C" void renderFrameEyeLight(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  extern "C" void renderFrameOcclusion(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  extern "C" void renderFrameUV      (int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  extern "C" void renderFrameNg      (int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  extern "C" void renderFrameGeomID  (int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  extern "C" void renderFrameGeomIDPrimID(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  extern "C" void renderFrameTexCoords(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  extern "C" void renderFrameCycles  (int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  extern "C" void renderFrameAmbientOcclusion(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  extern "C" void renderFrameDifferentials(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  
  extern "C"
  {
    RTCDevice g_device = nullptr;
    
    float g_debug = 0.0f;
    Mode g_mode = MODE_NORMAL;
    ISPCScene* g_ispc_scene = nullptr;

    /* intensity scaling for traversal cost visualization */
    float scale = 1.0f / 1000000.0f;
    bool g_changed = false;

    int64_t get_tsc() { return read_tsc(); }

    unsigned int g_numThreads = 0;

    RTCIntersectContextFlags g_iflags_coherent = RTC_INTERSECT_CONTEXT_FLAG_COHERENT;
    RTCIntersectContextFlags g_iflags_incoherent = RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT;

    int g_animation_mode = false;

    RayStats* g_stats = nullptr;

    unsigned int render_texcoords_mode = 0;

    int differentialMode = 0;
  }

  extern "C" int g_instancing_mode;

  /* error reporting function */
  void error_handler(void* userPtr, const RTCError code, const char* str)
  {
    if (code == RTC_ERROR_NONE)
      return;
    
    printf("Embree: ");
    switch (code) {
    case RTC_ERROR_UNKNOWN          : printf("RTC_ERROR_UNKNOWN"); break;
    case RTC_ERROR_INVALID_ARGUMENT : printf("RTC_ERROR_INVALID_ARGUMENT"); break;
    case RTC_ERROR_INVALID_OPERATION: printf("RTC_ERROR_INVALID_OPERATION"); break;
    case RTC_ERROR_OUT_OF_MEMORY    : printf("RTC_ERROR_OUT_OF_MEMORY"); break;
    case RTC_ERROR_UNSUPPORTED_CPU  : printf("RTC_ERROR_UNSUPPORTED_CPU"); break;
    case RTC_ERROR_CANCELLED        : printf("RTC_ERROR_CANCELLED"); break;
    default                         : printf("invalid error code"); break;
    }
    if (str) {
      printf(" (");
      while (*str) putchar(*str++);
      printf(")\n");
    }
    exit(1);
  }
  
  TutorialApplication* TutorialApplication::instance = nullptr;

  TutorialApplication::TutorialApplication (const std::string& tutorialName, int features)

    : Application(features),
      tutorialName(tutorialName),

      shader(SHADER_DEFAULT),

      width(512),
      height(512),
      pixels(nullptr),

      outputImageFilename(""),
      referenceImageFilename(""),
      referenceImageThreshold(32.0f),

      interactive(true),
      fullscreen(false),

      window_width(512),
      window_height(512),

      time0(getSeconds()),
      debug_int0(0),
      debug_int1(0),

      mouseMode(0),
      clickX(0.0), clickY(0.0),
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

      iflags_coherent(RTC_INTERSECT_CONTEXT_FLAG_COHERENT),
      iflags_incoherent(RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT)
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

    registerOption("compare", [this] (Ref<ParseStream> cin, const FileName& path) {
        referenceImageFilename = cin->getFileName();
        interactive = false;
      }, "--compare <filename>: reference image to compare against");

    registerOption("compare-threshold", [this] (Ref<ParseStream> cin, const FileName& path) {
        referenceImageThreshold = cin->getFloat();
      }, "--compare-threshold <float>: threshold in number of wrong pixels when image is considered wrong");

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

    registerOption("lefthanded", [this] (Ref<ParseStream> cin, const FileName& path) {
        camera.handedness = Camera::LEFT_HANDED;
      }, "--lefthanded: use left handed coordinates");

    registerOption("righthanded", [this] (Ref<ParseStream> cin, const FileName& path) {
        camera.handedness = Camera::RIGHT_HANDED;
      }, "--righthanded: use right handed coordinates");

    /* framebuffer settings */
    registerOption("size", [this] (Ref<ParseStream> cin, const FileName& path) {
        width = cin->getInt();
        height = cin->getInt();
      }, "--size <width> <height>: sets image size");

    registerOption("fullscreen", [this] (Ref<ParseStream> cin, const FileName& path) {
        fullscreen = true;
      }, "--fullscreen: starts in fullscreen mode");

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

     registerOption("time", [this] (Ref<ParseStream> cin, const FileName& path) {
         shader = SHADER_EYELIGHT;
         g_debug = cin->getFloat();
       }, "--time: sets time for motion blur");

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
        g_iflags_coherent   = iflags_coherent   = RTC_INTERSECT_CONTEXT_FLAG_COHERENT;
        g_iflags_incoherent = iflags_incoherent = RTC_INTERSECT_CONTEXT_FLAG_COHERENT;
      }, "--coherent: force using RTC_INTERSECT_CONTEXT_FLAG_COHERENT hint when tracing rays");

    registerOption("incoherent", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_iflags_coherent   = iflags_coherent   = RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT;
        g_iflags_incoherent = iflags_incoherent = RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT;
      }, "--incoherent: force using RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT hint when tracing rays");
  }

  TutorialApplication::~TutorialApplication()
  {
    g_ispc_scene = nullptr;
    ispc_scene = nullptr;
    device_cleanup();
    if (g_device) rtcReleaseDevice(g_device);
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
      convert_tris_to_quads_prop(inf),
      grid_resX(2),
      grid_resY(2),
      remove_mblur(false),
      remove_non_mblur(false),
      sceneFilename(),
      instancing_mode(SceneGraph::INSTANCING_NONE),
      print_scene_cameras(false)
  {
    registerOption("i", [this] (Ref<ParseStream> cin, const FileName& path) {
        sceneFilename.push_back(path + cin->getFileName());
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
        sgop.push_back(CONVERT_TRIANGLES_TO_QUADS);
        convert_tris_to_quads_prop = inf;
      }, "--convert-triangles-to-quads: converts all triangles to quads when loading");

    registerOption("convert-triangles-to-triangles-and-quads", [this] (Ref<ParseStream> cin, const FileName& path) {
        sgop.push_back(CONVERT_TRIANGLES_TO_QUADS);
        convert_tris_to_quads_prop = 0.5f;
      }, "--convert-triangles-to-triangles-and-quads: converts to mixed triangle/quad scene");

    registerOption("convert-bezier-to-lines", [this] (Ref<ParseStream> cin, const FileName& path) {
        sgop.push_back(CONVERT_BEZIER_TO_LINES);
      }, "--convert-bezier-to-lines: converts all bezier curves to line segments when loading");

    registerOption("convert-flat-to-round-curves", [this] (Ref<ParseStream> cin, const FileName& path) {
        sgop.push_back(CONVERT_FLAT_TO_ROUND_CURVES);
      }, "--convert-flat-to-round-curves: converts all flat curves to round curves");
    registerOptionAlias("convert-flat-to-round-curves","convert-hair-to-curves"); // for compatibility reasons

    registerOption("convert-round-to-flat-curves", [this] (Ref<ParseStream> cin, const FileName& path) {
        sgop.push_back(CONVERT_ROUND_TO_FLAT_CURVES);
      }, "--convert-round-to-flat-curves: converts all round curves to flat curves");
    
    registerOption("convert-bezier-to-bspline", [this] (Ref<ParseStream> cin, const FileName& path) {
        sgop.push_back(CONVERT_BEZIER_TO_BSPLINE);
      }, "--convert-bezier-to-bspline: converts all bezier curves to bsplines curves");

    registerOption("convert-bspline-to-bezier", [this] (Ref<ParseStream> cin, const FileName& path) {
        sgop.push_back(CONVERT_BSPLINE_TO_BEZIER);
      }, "--convert-bspline-to-bezier: converts all bsplines curves to bezier curves");

    registerOption("convert-bezier-to-hermite", [this] (Ref<ParseStream> cin, const FileName& path) {
        sgop.push_back(CONVERT_BEZIER_TO_HERMITE);
      }, "--convert-bezier-to-hermite: converts all bezier curves to hermite curves");

    registerOption("merge-triangles-to-grids", [this] (Ref<ParseStream> cin, const FileName& path) {
        sgop.push_back(CONVERT_TRIANGLES_TO_QUADS);
        sgop.push_back(MERGE_QUADS_TO_GRIDS);
      }, "--merge-triangles-to-grids: merges quads to grids");
    
    registerOption("merge-quads-to-grids", [this] (Ref<ParseStream> cin, const FileName& path) {
        sgop.push_back(MERGE_QUADS_TO_GRIDS);
      }, "--merge-quads-to-grids: merges quads to grids");

    registerOption("convert-quads-to-grids", [this] (Ref<ParseStream> cin, const FileName& path) {
        sgop.push_back(CONVERT_QUADS_TO_GRIDS);
      }, "--convert-quads-to-grids: converts all quads to grids");

    registerOption("convert-grids-to-quads", [this] (Ref<ParseStream> cin, const FileName& path) {
        sgop.push_back(CONVERT_GRIDS_TO_QUADS);
      }, "--convert-grids-to-quads: converts all grids to quads");

    registerOption("convert-triangles-to-grids", [this] (Ref<ParseStream> cin, const FileName& path) {
        sgop.push_back(CONVERT_TRIANGLES_TO_QUADS);
        sgop.push_back(CONVERT_QUADS_TO_GRIDS);
      }, "--convert-triangles-to-grids: converts all triangles to grids");

    registerOption("convert-triangles-to-grids-to-quads", [this] (Ref<ParseStream> cin, const FileName& path) {
        sgop.push_back(CONVERT_TRIANGLES_TO_QUADS);
        sgop.push_back(CONVERT_QUADS_TO_GRIDS);
        sgop.push_back(CONVERT_GRIDS_TO_QUADS);
      }, "--convert-triangles-to-grids-to-quads: converts all triangles to grids and then to quads");

    registerOption("grid-res", [this] (Ref<ParseStream> cin, const FileName& path) {
        grid_resX = min(max(cin->getInt(),2),0x7fff);
        grid_resY = min(max(cin->getInt(),2),0x7fff);        
      }, "--grid-res: sets tessellation resolution for the grid primitive");

    registerOption("convert-mblur-to-nonmblur", [this] (Ref<ParseStream> cin, const FileName& path) {
         sgop.push_back(CONVERT_MBLUR_TO_NONMBLUR);
      }, "--convert-mblur-to-nonmblur: converts all motion blur geometry to non-motion blur geometry");
    
    registerOption("remove-mblur", [this] (Ref<ParseStream> cin, const FileName& path) {
         remove_mblur = true;
      }, "--remove-mblur: removes all motion blur geometry");

    registerOption("remove-non-mblur", [this] (Ref<ParseStream> cin, const FileName& path) {
         remove_non_mblur = true;
      }, "--remove-non-mblur: removes all non-motion blur geometry");

    registerOption("instancing", [this] (Ref<ParseStream> cin, const FileName& path) {
        std::string mode = cin->getString();
        if      (mode == "none"    ) instancing_mode = SceneGraph::INSTANCING_NONE;
        else if (mode == "scene_geometry") instancing_mode = SceneGraph::INSTANCING_GEOMETRY; // for compatibility
        else if (mode == "scene_group"   ) instancing_mode = SceneGraph::INSTANCING_GROUP; // for compatibility
        else if (mode == "geometry") instancing_mode = SceneGraph::INSTANCING_GEOMETRY;
        else if (mode == "group"   ) instancing_mode = SceneGraph::INSTANCING_GROUP;
        else if (mode == "flattened") instancing_mode = SceneGraph::INSTANCING_FLATTENED;
        else if (mode == "multi_level") instancing_mode = SceneGraph::INSTANCING_MULTI_LEVEL;
        else throw std::runtime_error("unknown instancing mode: "+mode);
        g_instancing_mode = instancing_mode;
      }, "--instancing: set instancing mode\n"
      "  none: perform no instancing and flatten entire scene\n"
      "  geometry: instance individual geometries as scenes\n"
      "  group: instance geometry groups as scenes\n"
      "  multi_level: use multi-level instancing\n"
      "  flattened: assume flattened scene graph");

    registerOption("animation", [] (Ref<ParseStream> cin, const FileName& path) {
         g_animation_mode = true;
      }, "--animation: render animated geometries");

    registerOption("ambientlight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f L = cin->getVec3f();
        futures.push_back([=]() { scene->add(new SceneGraph::LightNodeImpl<SceneGraph::AmbientLight>(SceneGraph::AmbientLight(L))); });
      }, "--ambientlight r g b: adds an ambient light with intensity rgb");
    registerOptionAlias("ambientlight","ambient");

    registerOption("pointlight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f P = cin->getVec3f();
        const Vec3f I = cin->getVec3f();
        futures.push_back([=]() { scene->add(new SceneGraph::LightNodeImpl<SceneGraph::PointLight>(SceneGraph::PointLight(P,I))); });
      }, "--pointlight x y z r g b: adds a point light at position xyz with intensity rgb");

    registerOption("directionallight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f D = cin->getVec3f();
        const Vec3f E = cin->getVec3f();
        futures.push_back([=]() { scene->add(new SceneGraph::LightNodeImpl<SceneGraph::DirectionalLight>(SceneGraph::DirectionalLight(D,E))); });
      }, "--directionallight x y z r g b: adds a directional light with direction xyz and intensity rgb");
    registerOptionAlias("directionallight","dirlight");

    registerOption("distantlight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f D = cin->getVec3f();
        const Vec3f L = cin->getVec3f();
        const float halfAngle = cin->getFloat();
        futures.push_back([=]() { scene->add(new SceneGraph::LightNodeImpl<SceneGraph::DistantLight>(SceneGraph::DistantLight(D,L,halfAngle))); });
      }, "--distantlight x y z r g b a: adds a distant light with direction xyz, intensity rgb, and opening angle a");

    registerOption("triangle-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p0 = cin->getVec3f();
        const Vec3f dx = cin->getVec3f();
        const Vec3f dy = cin->getVec3f();
        const size_t width = cin->getInt();
        const size_t height = cin->getInt();
        futures.push_back([=]() { scene->add(SceneGraph::createTrianglePlane(p0,dx,dy,width,height,new OBJMaterial)); });
      }, "--triangle-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z width height: adds a plane build of triangles originated at p0 and spanned by the vectors dx and dy with a tessellation width/height.");

    registerOption("quad-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p0 = cin->getVec3f();
        const Vec3f dx = cin->getVec3f();
        const Vec3f dy = cin->getVec3f();
        const size_t width = cin->getInt();
        const size_t height = cin->getInt();
        futures.push_back([=]() { scene->add(SceneGraph::createQuadPlane(p0,dx,dy,width,height,new OBJMaterial)); });
      }, "--quad-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z width height: adds a plane build of quadrilaterals originated at p0 and spanned by the vectors dx and dy with a tessellation width/height.");

    registerOption("grid-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p0 = cin->getVec3f();
        const Vec3f dx = cin->getVec3f();
        const Vec3f dy = cin->getVec3f();
        const size_t width = cin->getInt();
        const size_t height = cin->getInt();
        futures.push_back([=]() { scene->add(SceneGraph::createGridPlane(p0,dx,dy,width,height,new OBJMaterial)); });
      }, "--grid-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z width height: adds a plane using a grid mesh build. The plane is originated at p0 and spanned by the vectors dx and dy with a tessellation width/height.");

    registerOption("subdiv-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p0 = cin->getVec3f();
        const Vec3f dx = cin->getVec3f();
        const Vec3f dy = cin->getVec3f();
        const size_t width = cin->getInt();
        const size_t height = cin->getInt();
        const float tessellationRate = cin->getFloat();
        futures.push_back([=]() { scene->add(SceneGraph::createSubdivPlane(p0,dx,dy,width,height,tessellationRate,new OBJMaterial)); });
      }, "--subdiv-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z width height tessellationRate: adds a plane build as a Catmull Clark subdivision surface originated at p0 and spanned by the vectors dx and dy. The plane consists of widt x height many patches, and each patch has the specified tessellation rate.");

    registerOption("hair-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p0 = cin->getVec3f();
        const Vec3f dx = cin->getVec3f();
        const Vec3f dy = cin->getVec3f();
        const float len = cin->getFloat();
        const float r = cin->getFloat();
        const size_t N = cin->getInt();
        futures.push_back([=]() { scene->add(SceneGraph::createHairyPlane(0,p0,dx,dy,len,r,N,SceneGraph::FLAT_CURVE,new OBJMaterial)); });
      }, "--hair-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z length radius num: adds a hair plane originated at p0 and spanned by the vectors dx and dy. num hairs are generated with specified length and radius.");

    registerOption("curve-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p0 = cin->getVec3f();
        const Vec3f dx = cin->getVec3f();
        const Vec3f dy = cin->getVec3f();
        const float len = cin->getFloat();
        const float r = cin->getFloat();
        const size_t N = cin->getInt();
        futures.push_back([=]() { scene->add(SceneGraph::createHairyPlane(0,p0,dx,dy,len,r,N,SceneGraph::ROUND_CURVE,new OBJMaterial)); });
      }, "--curve-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z length radius: adds a plane build of bezier curves originated at p0 and spanned by the vectors dx and dy. num curves are generated with specified length and radius.");

     registerOption("sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
         const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        futures.push_back([=]() { scene->add(SceneGraph::createSphere(p, r, new OBJMaterial)); });
      }, "--sphere p.x p.y p.z r: adds a sphere at position p with radius r");
     
    registerOption("triangle-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([=]() { scene->add(SceneGraph::createTriangleSphere(p,r,numPhi,new OBJMaterial)); });
      }, "--triangle-sphere p.x p.y p.z r numPhi: adds a sphere at position p with radius r and tessellation numPhi build of triangles.");

    registerOption("quad-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([=]() { scene->add(SceneGraph::createQuadSphere(p,r,numPhi,new OBJMaterial)); });
      }, "--quad-sphere p.x p.y p.z r numPhi: adds a sphere at position p with radius r and tessellation numPhi build of quadrilaterals.");

    registerOption("grid-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        const size_t N = cin->getInt();
        futures.push_back([=]() { scene->add(SceneGraph::createGridSphere(p,r,N,new OBJMaterial)); });
      }, "--grid-sphere p.x p.y p.z r N: adds a grid sphere at position p with radius r using a cube topology and N*N quads at each face.");

    registerOption("triangle-sphere-mblur", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const Vec3f dp = cin->getVec3f();
        const float  r = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([=]() {
          Ref<SceneGraph::Node> mesh = SceneGraph::createTriangleSphere(p,r,numPhi,new OBJMaterial);
          mesh->set_motion_vector(dp); 
          scene->add(mesh);
        });
      }, "--triangle-sphere-mblur p.x p.y p.z d.x d.y d.z r numPhi : adds a motion blurred sphere build of triangles at position p, with motion vector d, radius r, and tessellation numPhi.");

    registerOption("quad-sphere-mblur", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const Vec3f dp = cin->getVec3f();
        const float  r = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([=]() {
          Ref<SceneGraph::Node> mesh = SceneGraph::createQuadSphere(p,r,numPhi,new OBJMaterial);
          mesh->set_motion_vector(dp); 
          scene->add(mesh);
        });
      }, "--quad-sphere-mblur p.x p.y p.z d.x d.y d.z r numPhi : adds a motion blurred sphere build of quadrilaterals at position p, with motion vector d, radius r, and tessellation numPhi.");

    registerOption("subdiv-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        const size_t numPhi = cin->getInt();
        const float tessellationRate = cin->getFloat();
        futures.push_back([=]() { scene->add(SceneGraph::createSubdivSphere(p,r,numPhi,tessellationRate,new OBJMaterial)); });
      }, "--subdiv-sphere p.x p.y p.z r numPhi: adds a sphere at position p with radius r build of Catmull Clark subdivision surfaces. The sphere consists of numPhi x numPhi many patches and each path has the specified tessellation rate.");

    registerOption("point-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        const float pointR = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([=]() { scene->add(SceneGraph::createPointSphere(p, r, pointR, numPhi, SceneGraph::SPHERE, new OBJMaterial)); });
      }, "--point-sphere p.x p.y p.z r pointR numPhi: adds a sphere at position p with radius r and tessellation numPhi build of spheres.");

     registerOption("point-sphere-mblur", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const Vec3f dp = cin->getVec3f();
        const float  r = cin->getFloat();
        const float pointR = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([=]() { scene->add(SceneGraph::createPointSphere(p, r, pointR, numPhi, SceneGraph::SPHERE, new OBJMaterial)->set_motion_vector(dp)); });
      }, "--point-sphere p.x p.y p.z d.x d.y d.z r pointR numPhi: adds a sphere at position p, motion vector d, with radius r and tessellation numPhi build of spheres.");

    registerOption("disc-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        const float pointR = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([=]() { scene->add(SceneGraph::createPointSphere(p, r, pointR, numPhi, SceneGraph::DISC, new OBJMaterial)); });
      }, "--disc-sphere p.x p.y p.z r pointR numPhi: adds a sphere at position p with radius r and tessellation numPhi build of discs.");

    registerOption("oriented-disc-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        const float pointR = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([=]() { scene->add(SceneGraph::createPointSphere(p, r, pointR, numPhi, SceneGraph::ORIENTED_DISC, new OBJMaterial)); });
      }, "--oriented-disc-sphere p.x p.y p.z r pointR numPhi: adds a sphere at position p with radius r and tessellation numPhi build of oriented discs.");

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
      g_stats = (RayStats*)alignedMalloc(TaskScheduler::threadCount() * sizeof(RayStats), 64);

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

  void TutorialApplication::renderToFile(const FileName& fileName)
  {
    resize(width,height);
    ISPCCamera ispccamera = camera.getISPCCamera(width,height);
    initRayStats();
    render(pixels,width,height,0.0f,ispccamera);
    Ref<Image> image = new Image4uc(width, height, (Col4uc*)pixels);
    storeImage(image, fileName);
  }

  void TutorialApplication::compareToReferenceImage(const FileName& fileName)
  {
    resize(width,height);
    ISPCCamera ispccamera = camera.getISPCCamera(width,height);
    initRayStats();
    device_render(pixels,width,height,0.0f,ispccamera);
    renderFrame((int*)pixels,width,height,0.0f,ispccamera);
    Ref<Image> image = new Image4uc(width, height, (Col4uc*)pixels);
    Ref<Image> reference = loadImage(fileName);
    const double error = compareImages(image,reference);
    if (error > referenceImageThreshold) // error corresponds roughly to number of pixels that are completely off in color
      throw std::runtime_error("reference image differs by " + std::to_string(error));
  }

  void TutorialApplication::set_parameter(size_t parm, ssize_t val) {
    rtcSetDeviceProperty(nullptr,(RTCDeviceProperty)parm,val);
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
    ispc_scene.reset(new ISPCScene(g_device,in));
    g_ispc_scene = ispc_scene.get();
  }

  void errorFunc(int error, const char* description) {
    throw std::runtime_error(std::string("Error: ")+description);
  }

#if defined(USE_GLFW)
  
  void keyboardFunc(GLFWwindow* window, int key, int scancode, int action, int mods) {
    TutorialApplication::instance->keyboardFunc(window,key,scancode,action,mods);
  }
  void clickFunc(GLFWwindow* window, int button, int action, int mods) {
    TutorialApplication::instance->clickFunc(window,button,action,mods);
  }
  void motionFunc(GLFWwindow* window, double x, double y) {
    TutorialApplication::instance->motionFunc(window,x,y);
  }
  void displayFunc() {
    TutorialApplication::instance->displayFunc();
  }
  void reshapeFunc(GLFWwindow* window, int width, int height) {
    TutorialApplication::instance->reshapeFunc(window,width,height);
  }

  GLFWwindow* TutorialApplication::createFullScreenWindow()
  {
    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    glfwWindowHint(GLFW_RED_BITS,mode->redBits);
    glfwWindowHint(GLFW_GREEN_BITS,mode->greenBits);
    glfwWindowHint(GLFW_BLUE_BITS,mode->blueBits);
    glfwWindowHint(GLFW_REFRESH_RATE,mode->refreshRate);
    GLFWwindow* window = glfwCreateWindow(mode->width,mode->height,tutorialName.c_str(),monitor,nullptr);
    glfwSetKeyCallback(window,embree::keyboardFunc);
    glfwSetCursorPosCallback(window,embree::motionFunc);
    glfwSetMouseButtonCallback(window,embree::clickFunc);
    glfwSetCharCallback(window, ImGui_ImplGlfw_CharCallback);
    glfwSetScrollCallback(window, ImGui_ImplGlfw_ScrollCallback);
    glfwSetWindowSizeCallback(window,embree::reshapeFunc);
    resize(mode->width,mode->height);
    return window;
  }

  GLFWwindow* TutorialApplication::createStandardWindow(int width, int height)
  {
    GLFWwindow* window = glfwCreateWindow(width,height,tutorialName.c_str(),nullptr,nullptr);
    glfwSetKeyCallback(window,embree::keyboardFunc);
    glfwSetCursorPosCallback(window,embree::motionFunc);
    glfwSetMouseButtonCallback(window,embree::clickFunc);
    glfwSetCharCallback(window, ImGui_ImplGlfw_CharCallback);
    glfwSetScrollCallback(window, ImGui_ImplGlfw_ScrollCallback);
    glfwSetWindowSizeCallback(window,embree::reshapeFunc);
    resize(width,height);
    return window;
  }

  /* called when a key is pressed */
  void TutorialApplication::keypressed(int key)
  {
    if (key == GLFW_KEY_F1) {
      renderFrame = renderFrameStandard;
      g_changed = true;
    }
    else if (key == GLFW_KEY_F2) {
      renderFrame = renderFrameEyeLight;
      g_changed = true;
    }
    else if (key == GLFW_KEY_F3) {
      renderFrame = renderFrameOcclusion;
      g_changed = true;
    }
    else if (key == GLFW_KEY_F4) {
      renderFrame = renderFrameUV;
      g_changed = true;
    }
    else if (key == GLFW_KEY_F5) {
      renderFrame = renderFrameNg;
      g_changed = true;
    }
    else if (key == GLFW_KEY_F6) {
      renderFrame = renderFrameGeomID;
      g_changed = true;
    }
    else if (key == GLFW_KEY_F7) {
      renderFrame = renderFrameGeomIDPrimID;
      g_changed = true;
    }
    else if (key == GLFW_KEY_F8) {
      if (renderFrame == renderFrameTexCoords) render_texcoords_mode++;
      renderFrame = renderFrameTexCoords;
      g_changed = true;
    }
    else if (key == GLFW_KEY_F9) {
      if (renderFrame == renderFrameCycles) scale *= 2.0f;
      renderFrame = renderFrameCycles;
      g_changed = true;
    }
    else if (key == GLFW_KEY_F10) {
      if (renderFrame == renderFrameCycles) scale *= 0.5f;
      renderFrame = renderFrameCycles;
      g_changed = true;
    }
    else if (key == GLFW_KEY_F11) {
      renderFrame = renderFrameAmbientOcclusion;
      g_changed = true;
    }
    else if (key == GLFW_KEY_F12) {
      if (renderFrame == renderFrameDifferentials) {
        differentialMode = (differentialMode+1)%17;
      } else {
        renderFrame = renderFrameDifferentials;
        differentialMode = 0;
      }
      g_changed = true;
    }
  }

  void TutorialApplication::keyboardFunc(GLFWwindow* window_in, int key, int scancode, int action, int mods)
  {
    ImGui_ImplGlfw_KeyCallback(window_in,key,scancode,action,mods);
    if (ImGui::GetIO().WantCaptureKeyboard) return;
      
    if (action == GLFW_PRESS)
    {
      /* call tutorial keyboard handler */
      keypressed(key);

      if (mods & GLFW_MOD_CONTROL)
      {
        switch (key) {
        case GLFW_KEY_UP        : debug_int0++; set_parameter(1000000,debug_int0); PRINT(debug_int0); break;
        case GLFW_KEY_DOWN      : debug_int0--; set_parameter(1000000,debug_int0); PRINT(debug_int0); break;
        case GLFW_KEY_LEFT      : debug_int1--; set_parameter(1000001,debug_int1); PRINT(debug_int1); break;
        case GLFW_KEY_RIGHT     : debug_int1++; set_parameter(1000001,debug_int1); PRINT(debug_int1); break;
        }
      }
      else
      {
        switch (key) {
        case GLFW_KEY_LEFT      : camera.rotate(-0.02f,0.0f); break;
        case GLFW_KEY_RIGHT     : camera.rotate(+0.02f,0.0f); break;
        case GLFW_KEY_UP        : camera.move(0.0f,0.0f,+speed); break;
        case GLFW_KEY_DOWN      : camera.move(0.0f,0.0f,-speed); break;
        case GLFW_KEY_PAGE_UP   : speed *= 1.2f; break;
        case GLFW_KEY_PAGE_DOWN : speed /= 1.2f; break;

        case GLFW_KEY_W : moveDelta.z = +1.0f; break;
        case GLFW_KEY_S : moveDelta.z = -1.0f; break;
        case GLFW_KEY_A : moveDelta.x = -1.0f; break;
        case GLFW_KEY_D : moveDelta.x = +1.0f; break;
          
        case GLFW_KEY_F :
          glfwDestroyWindow(window);
          if (fullscreen) {
            width = window_width;
            height = window_height;
            window = createStandardWindow(width,height);
          }
          else {
            window_width = width;
            window_height = height;
            window = createFullScreenWindow();
          }
          glfwMakeContextCurrent(window);
          fullscreen = !fullscreen;
          break;
          
        case GLFW_KEY_C : std::cout << camera.str() << std::endl; break;
        case GLFW_KEY_HOME: g_debug=clamp(g_debug+0.01f); PRINT(g_debug); break;
        case GLFW_KEY_END : g_debug=clamp(g_debug-0.01f); PRINT(g_debug); break;
          
        case GLFW_KEY_SPACE: {
          Ref<Image> image = new Image4uc(width, height, (Col4uc*)pixels, true, "", true);
          storeImage(image, "screenshot.tga");
          break;
        }
          
        case GLFW_KEY_ESCAPE:
        case GLFW_KEY_Q: 
          glfwSetWindowShouldClose(window,1);
          break;
        }
      }
    }
    else if (action == GLFW_RELEASE)
    {
      switch (key)
      {
      case GLFW_KEY_W : moveDelta.z = 0.0f; break;
      case GLFW_KEY_S : moveDelta.z = 0.0f; break;
      case GLFW_KEY_A : moveDelta.x = 0.0f; break;
      case GLFW_KEY_D : moveDelta.x = 0.0f; break;
      }
    }
  }
    
  void TutorialApplication::clickFunc(GLFWwindow* window, int button, int action, int mods)
  {
    ImGui_ImplGlfw_MouseButtonCallback(window,button,action,mods);
    if (ImGui::GetIO().WantCaptureMouse) return;
  
    double x,y;
    glfwGetCursorPos(window,&x,&y);
    
    if (action == GLFW_RELEASE)
    {
      mouseMode = 0;
    }
    else if (action == GLFW_PRESS)
    {
      if (button == GLFW_MOUSE_BUTTON_RIGHT)
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
        if      (button == GLFW_MOUSE_BUTTON_LEFT && mods == GLFW_MOD_SHIFT) mouseMode = 1;
        else if (button == GLFW_MOUSE_BUTTON_LEFT && mods == GLFW_MOD_CONTROL ) mouseMode = 3;
        else if (button == GLFW_MOUSE_BUTTON_LEFT) mouseMode = 4;
      }
    }
  }

  void TutorialApplication::motionFunc(GLFWwindow* window, double x, double y)
  {
    if (ImGui::GetIO().WantCaptureMouse) return;
  
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
    double t0 = getSeconds();
    const float time = float(t0-time0);
    
    /* update camera */
    camera.move(moveDelta.x*speed, moveDelta.y*speed, moveDelta.z*speed);

    /* update animated camera */
    if (animated_camera)
      camera = Camera(animated_camera->get(time),camera.handedness);
    
    ISPCCamera ispccamera = camera.getISPCCamera(width,height);
     if (print_camera)
      std::cout << camera.str() << std::endl;

    /* render image using ISPC */
    initRayStats();
    render(pixels,width,height,time,ispccamera);
    double dt0 = getSeconds()-t0;
    avg_render_time.add(dt0);
    double mrayps = double(getNumRays())/(1000000.0*dt0);
    avg_mrayps.add(mrayps);

    /* draw pixels to screen */
    glRasterPos2i(-1,1);
    glPixelZoom(1.0f,-1.0f);
    glDrawPixels(width,height,GL_RGBA,GL_UNSIGNED_BYTE,pixels);

    ImGui_ImplGlfwGL2_NewFrame();
    
    ImGuiWindowFlags window_flags = 0;
    window_flags |= ImGuiWindowFlags_NoTitleBar;
    //window_flags |= ImGuiWindowFlags_NoScrollbar;
    //window_flags |= ImGuiWindowFlags_MenuBar;
    //window_flags |= ImGuiWindowFlags_NoMove;
    //window_flags |= ImGuiWindowFlags_NoResize;
    //window_flags |= ImGuiWindowFlags_NoCollapse;
    //window_flags |= ImGuiWindowFlags_NoNav;

    //ImGui::GetStyle().WindowBorderSize = 0.0f;
    //ImGui::SetNextWindowPos(ImVec2(width-200,0));
    //ImGui::SetNextWindowSize(ImVec2(200,height));
    ImGui::SetNextWindowBgAlpha(0.3f);
    ImGui::Begin("Embree", nullptr, window_flags);
    drawGUI();
    
    double avg_time = avg_render_time.get();
    double fps = avg_time != 0.0 ? 1.0f/avg_time : 0.0;
    ImGui::Text("%3.2f fps",fps);
#if defined(RAY_STATS)
    ImGui::Text("%3.2f Mray/s",avg_mrayps.get());
#endif
    ImGui::End();
     
    //ImGui::ShowDemoWindow();
        
    ImGui::Render();
    ImGui_ImplGlfwGL2_RenderDrawData(ImGui::GetDrawData());
    
    glfwSwapBuffers(window);

#ifdef __APPLE__
    // work around glfw issue #1334
    // https://github.com/glfw/glfw/issues/1334
    static bool macMoved = false;

    if (!macMoved) {
      int x, y;
      glfwGetWindowPos(window, &x, &y);
      glfwSetWindowPos(window, ++x, y);
      macMoved = true;
    }
#endif

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

  void TutorialApplication::reshapeFunc(GLFWwindow* window, int, int)
  {
    int width,height;
    glfwGetFramebufferSize(window, &width, &height);
    resize(width,height);
    glViewport(0, 0, width, height);
    this->width = width; this->height = height;
  }

   void TutorialApplication::renderInteractive()
   {
     window_width = width;
     window_height = height;
     glfwSetErrorCallback(errorFunc);
     glfwInit();
     glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,2);
     glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,0);
     
     if (fullscreen) window = createFullScreenWindow();
     else            window = createStandardWindow(width,height);
     
     glfwMakeContextCurrent(window);
     glfwSwapInterval(1);
     reshapeFunc(window,0,0);
     
     // Setup ImGui binding
     ImGui::CreateContext();
     ImGuiIO& io = ImGui::GetIO(); (void)io;
     //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
     ImGui_ImplGlfwGL2_Init(window, false);
     
     // Setup style
     ImGui::StyleColorsDark();
     //ImGui::StyleColorsClassic();
     
     // Load Fonts
     // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them. 
     // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple. 
     // - If the file cannot be loaded, the function will return NULL. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
     // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
     // - Read 'misc/fonts/README.txt' for more instructions and details.
     // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
     //io.Fonts->AddFontDefault();
     //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
     //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
     //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
     //io.Fonts->AddFontFromFileTTF("../../misc/fonts/ProggyTiny.ttf", 10.0f);
     //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());
     //IM_ASSERT(font != NULL);
     
     while (!glfwWindowShouldClose(window))
     {
       // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
       // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
       // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
       // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
       glfwPollEvents();
       
       displayFunc();
     }
     
     ImGui_ImplGlfwGL2_Shutdown();
     ImGui::DestroyContext();
     
     glfwDestroyWindow(window);
     glfwTerminate();
   }
  
#endif
  
  void TutorialApplication::render(unsigned* pixels, const unsigned width, const unsigned height, const float time, const ISPCCamera& camera) {
    device_render(pixels,width,height,time,camera);
    renderFrame((int*)pixels,width,height,time,camera);
  }
  
  void TutorialApplication::run(int argc, char** argv)
  {
    /* set debug values */
    rtcSetDeviceProperty(nullptr,(RTCDeviceProperty) 1000000, debug0);
    rtcSetDeviceProperty(nullptr,(RTCDeviceProperty) 1000001, debug1);
    rtcSetDeviceProperty(nullptr,(RTCDeviceProperty) 1000002, debug2);
    rtcSetDeviceProperty(nullptr,(RTCDeviceProperty) 1000003, debug3);

    /* initialize ray tracing core */
    renderFrame = renderFrameStandard;
    device_init(rtcore.c_str());

    /* set shader mode */
    switch (shader) {
    case SHADER_DEFAULT  : renderFrame = renderFrameStandard; break;
    case SHADER_EYELIGHT : renderFrame = renderFrameEyeLight; break;
    case SHADER_OCCLUSION: renderFrame = renderFrameOcclusion; break;
    case SHADER_UV       : renderFrame = renderFrameUV; break;
    case SHADER_TEXCOORDS: renderFrame = renderFrameTexCoords; render_texcoords_mode = 0; break;
    case SHADER_TEXCOORDS_GRID: renderFrame = renderFrameTexCoords; render_texcoords_mode = 1; break;
    case SHADER_NG       : renderFrame = renderFrameNg; break;
    case SHADER_CYCLES   : renderFrame = renderFrameCycles; break;
    case SHADER_GEOMID   : renderFrame = renderFrameGeomID; break;
    case SHADER_GEOMID_PRIMID: renderFrame = renderFrameGeomIDPrimID; break;
    case SHADER_AMBIENT_OCCLUSION: renderFrame = renderFrameAmbientOcclusion; break;
    };
    
    /* render to disk */
    if (outputImageFilename.str() != "")
      renderToFile(outputImageFilename);

    /* compare to reference image */
    if (referenceImageFilename.str() != "")
      compareToReferenceImage(referenceImageFilename);

#if defined(USE_GLFW)
    
    /* interactive mode */
    if (interactive)
      renderInteractive();
    
#else
    if (interactive) 
      std::cout << "GLFW is disabled, you can only render to disk using -o command line option." << std::endl;
#endif
  }

  int TutorialApplication::main(int argc, char** argv) try
  {
    /* parse command line options */
    parseCommandLine(argc,argv);

    /* callback */
    postParseCommandLine();

    /* create device */
    g_device = rtcNewDevice(rtcore.c_str());
    error_handler(nullptr,rtcGetDeviceError(g_device));

    /* set error handler */
    rtcSetDeviceErrorFunction(g_device,error_handler,nullptr);
  
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

    /* create device */
    g_device = rtcNewDevice(rtcore.c_str());
    error_handler(nullptr,rtcGetDeviceError(g_device));

    /* set error handler */
    rtcSetDeviceErrorFunction(g_device,error_handler,nullptr);
  
    log(1,"application start");

    /* execute postponed scene graph operations */
    for (auto& f : futures) f();
    
    /* load scene */
    if (sceneFilename.size())
    {
      for (auto& file : sceneFilename)
      {
        if (toLowerCase(file.ext()) == std::string("obj"))
          scene->add(loadOBJ(file,subdiv_mode != ""));
        else if (file.ext() != "")
          scene->add(SceneGraph::load(file));
      }
    }

    Application::instance->log(1,"loading scene done");

    /* load key frames for animation */
    for (size_t i=0; i<keyFramesFilenames.size(); i++)
    {
      if (verbosity >= 1) 
        std::cout << "Adding ["<< keyFramesFilenames[i] << "] to scene..." << std::flush;
      
      if (toLowerCase(keyFramesFilenames[i].ext()) == std::string("obj"))
        scene->add(loadOBJ(keyFramesFilenames[i],subdiv_mode != "",true));
      else if (keyFramesFilenames[i].ext() != "")
        scene->add(SceneGraph::load(keyFramesFilenames[i]));
      
      if (verbosity >= 1) 
        std::cout << " [DONE]" << std::endl << std::flush;
    }

    /* clear texture cache */
    Texture::clearTextureCache();

    /* perform removals */
    if (remove_mblur)     scene->remove_mblur(true);
    if (remove_non_mblur) scene->remove_mblur(false);

    /* perform conversions */
    if (sgop.size() && verbosity >= 1) {
      std::cout << std::endl;
      std::cout << "scene statistics (pre-convert):" << std::endl;
      SceneGraph::calculateStatistics(scene.dynamicCast<SceneGraph::Node>()).print();
      std::cout << std::endl;
    }

    /* perform scene graph conversions */
    for (auto& op : sgop)
    {
      switch (op) {
      case CONVERT_TRIANGLES_TO_QUADS   : scene->triangles_to_quads(convert_tris_to_quads_prop); break;
      case CONVERT_BEZIER_TO_LINES      : scene->bezier_to_lines(); break;
      case CONVERT_BEZIER_TO_BSPLINE    : scene->bezier_to_bspline(); break;
      case CONVERT_BSPLINE_TO_BEZIER    : scene->bspline_to_bezier(); break;
      case CONVERT_BEZIER_TO_HERMITE    : scene->bezier_to_hermite(); break;
      case CONVERT_FLAT_TO_ROUND_CURVES : scene->flat_to_round_curves(); break;
      case CONVERT_ROUND_TO_FLAT_CURVES : scene->round_to_flat_curves(); break;
      case MERGE_QUADS_TO_GRIDS         : scene->merge_quads_to_grids(); break;
      case CONVERT_QUADS_TO_GRIDS       : scene->quads_to_grids(grid_resX,grid_resY); break;
      case CONVERT_GRIDS_TO_QUADS       : scene->grids_to_quads(); break;
      case CONVERT_MBLUR_TO_NONMBLUR    : convert_mblur_to_nonmblur(scene.dynamicCast<SceneGraph::Node>()); break;
      default : throw std::runtime_error("unsupported scene graph operation");
      }
    }
    Application::instance->log(1,"converting scene done");

    if (verbosity >= 1) {
      std::cout << std::endl;
      std::cout << "scene statistics (pre-flattening):" << std::endl;
      SceneGraph::calculateStatistics(scene.dynamicCast<SceneGraph::Node>()).print();
      std::cout << std::endl;
    }
    
    Ref<SceneGraph::GroupNode> flattened_scene = SceneGraph::flatten(scene,instancing_mode);
    Application::instance->log(1,"flattening scene done");

    if (verbosity >= 1) {
      std::cout << std::endl;
      std::cout << "scene statistics (post-flattening):" << std::endl;
      SceneGraph::calculateStatistics(flattened_scene.dynamicCast<SceneGraph::Node>()).print();
      std::cout << std::endl;
    }

    /* convert model */
    obj_scene.add(flattened_scene);
    flattened_scene = nullptr;
    scene = nullptr;
    Application::instance->log(1,"populating tutorial scene done");
    
    /* print all cameras */
    if (print_scene_cameras) {
      obj_scene.print_camera_names();
      return 0;
    }

    /* use specified camera */
    if (camera_name != "") {
      auto cam = obj_scene.getCamera(camera_name);
      camera = Camera(cam->get(0),camera.handedness);
      if (cam->isAnimated()) animated_camera = cam;
    }

    /* otherwise use default camera */
    else if (!command_line_camera) {
      auto cam = obj_scene.getDefaultCamera();
      if (cam) {
        camera = Camera(cam->get(0),camera.handedness);
        if (cam->isAnimated()) animated_camera = cam;
      }
    }

    /* send model */
    set_scene(&obj_scene);
    Application::instance->log(1,"creating ISPC compatible scene done");

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
