// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "tutorial.h"
#include "scene.h"
#include "statistics.h"
#include <thread>

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

#if defined(EMBREE_SYCL_SUPPORT) && defined(EMBREE_SYCL_TUTORIAL)
#include "../sycl/util.h"
#endif

namespace embree
{
#if defined(EMBREE_SYCL_SUPPORT)
  sycl::context* global_gpu_context = nullptr;
  sycl::device* global_gpu_device = nullptr;
  sycl::queue* global_gpu_queue = nullptr;
#endif

  extern "C" void renderFrameStandard(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
    
  /* access to debug shader render frame functions */
  typedef void (* renderFrameFunc)(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  renderFrameFunc renderFrame = renderFrameStandard;
    
  extern "C"
  {
    RTCDevice g_device = nullptr;

    float g_debug = 0.0f;
    ISPCScene* g_ispc_scene = nullptr;

    /* intensity scaling for traversal cost visualization */
    float scale = 1.0f / 1000000.0f;
    bool g_changed = false;

    bool g_motion_blur = true;

#if !defined(__SYCL_DEVICE_ONLY__)
    int64_t get_tsc() { return read_tsc(); }
#endif

    unsigned int g_numThreads = 0;

    RTCRayQueryFlags g_iflags_coherent = RTC_RAY_QUERY_FLAG_COHERENT;
    RTCRayQueryFlags g_iflags_incoherent = RTC_RAY_QUERY_FLAG_INCOHERENT;

    int g_animation_mode = false;

    RayStats* g_stats = nullptr;
  }

  extern "C" int g_instancing_mode;

  /* error reporting function */
  void error_handler(void* userPtr, const RTCError code, const char* str)
  {
    if (code == RTC_ERROR_NONE)
      return;

    printf("Embree: %s", rtcGetErrorString(code));
    if (str) {
      printf(" (");
      while (*str) putchar(*str++);
      printf(")\n");
    } else {
      printf("\n");
    }
    exit(1);
  }

  TutorialApplication* TutorialApplication::instance = nullptr;

  TutorialApplication::TutorialApplication (const std::string& tutorialName, int features, int w, int h)

    : Application(features),
      tutorialName(tutorialName),

      width(w),
      height(h),
      pixels(nullptr),

      outputImageFilename(""),
      referenceImageFilename(""),
      referenceImageThreshold(35.0f),

      interactive(true),
      fullscreen(false),

      window_width(1024),
      window_height(1024),

      time0(getSeconds()),
      debug_int0(0),
      debug_int1(0),

      mouseMode(0),
      clickX(0.0), clickY(0.0),
      speed(1.0f),
      moveDelta(zero),

      animate(true),
      render_time(0.f),

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

      iflags_coherent(RTC_RAY_QUERY_FLAG_COHERENT),
      iflags_incoherent(RTC_RAY_QUERY_FLAG_INCOHERENT)
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

    registerOption("frames", [this] (Ref<ParseStream> cin, const FileName& path) {
        numFrames = cin->getInt();
      }, "--frames <int>: number of frames to render in compare or output mode");

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
         g_motion_blur = false;
         animate = false;
         render_time = cin->getFloat();
         g_debug = render_time;
       }, "--time: sets time for motion blur");

    registerOption("coherent", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_iflags_coherent   = iflags_coherent   = RTC_RAY_QUERY_FLAG_COHERENT;
        g_iflags_incoherent = iflags_incoherent = RTC_RAY_QUERY_FLAG_COHERENT;
      }, "--coherent: force using RTC_RAY_QUERY_FLAG_COHERENT hint when tracing rays");

    registerOption("incoherent", [this] (Ref<ParseStream> cin, const FileName& path) {
        g_iflags_coherent   = iflags_coherent   = RTC_RAY_QUERY_FLAG_INCOHERENT;
        g_iflags_incoherent = iflags_incoherent = RTC_RAY_QUERY_FLAG_INCOHERENT;
      }, "--incoherent: force using RTC_RAY_QUERY_FLAG_INCOHERENT hint when tracing rays");

#if defined(EMBREE_SYCL_TUTORIAL)
    registerOption("jit-cache", [this] (Ref<ParseStream> cin, const FileName& path) {
         jit_cache = cin->getInt();
       }, "--jit-cache <0/1>: enabled (1) or disables (0) JIT caching");
#endif
  }

  TutorialApplication::~TutorialApplication()
  {
    g_ispc_scene = nullptr;
    ispc_scene = nullptr;
    device_cleanup();
    if (g_device) rtcReleaseDevice(g_device);
    alignedUSMFree(pixels);
    pixels = nullptr;

#if defined(EMBREE_SYCL_SUPPORT)
    delete device; device = nullptr;
    delete queue; queue = nullptr;
    delete context; context = nullptr;
#endif
    
    width = 0;
    height = 0;
    alignedFree(g_stats);
    g_stats = nullptr;

#if defined(EMBREE_SYCL_SUPPORT)
    
    if (features & FEATURE_SYCL)
      disableUSMAllocTutorial();

#endif
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
        futures.push_back([this, L]() { scene->add(new SceneGraph::LightNodeImpl<SceneGraph::AmbientLight>(SceneGraph::AmbientLight(L))); });
      }, "--ambientlight r g b: adds an ambient light with intensity rgb");
    registerOptionAlias("ambientlight","ambient");

    registerOption("pointlight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f P = cin->getVec3f();
        const Vec3f I = cin->getVec3f();
        futures.push_back([this, P, I]() { scene->add(new SceneGraph::LightNodeImpl<SceneGraph::PointLight>(SceneGraph::PointLight(P,I))); });
      }, "--pointlight x y z r g b: adds a point light at position xyz with intensity rgb");

    registerOption("directionallight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f D = cin->getVec3f();
        const Vec3f E = cin->getVec3f();
        futures.push_back([this, D, E]() { scene->add(new SceneGraph::LightNodeImpl<SceneGraph::DirectionalLight>(SceneGraph::DirectionalLight(D,E))); });
      }, "--directionallight x y z r g b: adds a directional light with direction xyz and intensity rgb");
    registerOptionAlias("directionallight","dirlight");

    registerOption("distantlight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f D = cin->getVec3f();
        const Vec3f L = cin->getVec3f();
        const float halfAngle = cin->getFloat();
        futures.push_back([this, D, L, halfAngle]() { scene->add(new SceneGraph::LightNodeImpl<SceneGraph::DistantLight>(SceneGraph::DistantLight(D,L,halfAngle))); });
      }, "--distantlight x y z r g b a: adds a distant light with direction xyz, intensity rgb, and opening angle a");

    registerOption("triangle-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p0 = cin->getVec3f();
        const Vec3f dx = cin->getVec3f();
        const Vec3f dy = cin->getVec3f();
        const size_t width = cin->getInt();
        const size_t height = cin->getInt();
        futures.push_back([this, p0, dx, dy, width, height]() { scene->add(SceneGraph::createTrianglePlane(p0,dx,dy,width,height,new OBJMaterial)); });
      }, "--triangle-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z width height: adds a plane build of triangles originated at p0 and spanned by the vectors dx and dy with a tessellation width/height.");

    registerOption("quad-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p0 = cin->getVec3f();
        const Vec3f dx = cin->getVec3f();
        const Vec3f dy = cin->getVec3f();
        const size_t width = cin->getInt();
        const size_t height = cin->getInt();
        futures.push_back([this, p0, dx, dy, width, height]() { scene->add(SceneGraph::createQuadPlane(p0,dx,dy,width,height,new OBJMaterial)); });
      }, "--quad-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z width height: adds a plane build of quadrilaterals originated at p0 and spanned by the vectors dx and dy with a tessellation width/height.");

    registerOption("grid-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p0 = cin->getVec3f();
        const Vec3f dx = cin->getVec3f();
        const Vec3f dy = cin->getVec3f();
        const size_t width = cin->getInt();
        const size_t height = cin->getInt();
        futures.push_back([this, p0, dx, dy, width, height]() { scene->add(SceneGraph::createGridPlane(p0,dx,dy,width,height,new OBJMaterial)); });
      }, "--grid-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z width height: adds a plane using a grid mesh build. The plane is originated at p0 and spanned by the vectors dx and dy with a tessellation width/height.");

    registerOption("subdiv-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p0 = cin->getVec3f();
        const Vec3f dx = cin->getVec3f();
        const Vec3f dy = cin->getVec3f();
        const size_t width = cin->getInt();
        const size_t height = cin->getInt();
        const float tessellationRate = cin->getFloat();
        futures.push_back([this, p0, dx, dy, width, height, tessellationRate]() { scene->add(SceneGraph::createSubdivPlane(p0,dx,dy,width,height,tessellationRate,new OBJMaterial)); });
      }, "--subdiv-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z width height tessellationRate: adds a plane build as a Catmull Clark subdivision surface originated at p0 and spanned by the vectors dx and dy. The plane consists of widt x height many patches, and each patch has the specified tessellation rate.");

    registerOption("hair-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p0 = cin->getVec3f();
        const Vec3f dx = cin->getVec3f();
        const Vec3f dy = cin->getVec3f();
        const float len = cin->getFloat();
        const float r = cin->getFloat();
        const size_t N = cin->getInt();
        futures.push_back([this, p0, dx, dy, len, r, N]() { scene->add(SceneGraph::createHairyPlane(0,p0,dx,dy,len,r,N,SceneGraph::FLAT_CURVE,new OBJMaterial)); });
      }, "--hair-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z length radius num: adds a hair plane originated at p0 and spanned by the vectors dx and dy. num hairs are generated with specified length and radius.");

    registerOption("curve-plane", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p0 = cin->getVec3f();
        const Vec3f dx = cin->getVec3f();
        const Vec3f dy = cin->getVec3f();
        const float len = cin->getFloat();
        const float r = cin->getFloat();
        const size_t N = cin->getInt();
        futures.push_back([this, p0, dx, dy, len, r, N]() { scene->add(SceneGraph::createHairyPlane(0,p0,dx,dy,len,r,N,SceneGraph::ROUND_CURVE,new OBJMaterial)); });
      }, "--curve-plane p.x p.y p.z dx.x dx.y dx.z dy.x dy.y dy.z length radius: adds a plane build of bezier curves originated at p0 and spanned by the vectors dx and dy. num curves are generated with specified length and radius.");

     registerOption("sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
         const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        futures.push_back([this, p, r]() { scene->add(SceneGraph::createSphere(p, r, new OBJMaterial)); });
      }, "--sphere p.x p.y p.z r: adds a sphere at position p with radius r");
     
    registerOption("triangle-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([this, p, r, numPhi]() { scene->add(SceneGraph::createTriangleSphere(p,r,numPhi,new OBJMaterial)); });
      }, "--triangle-sphere p.x p.y p.z r numPhi: adds a sphere at position p with radius r and tessellation numPhi build of triangles.");

    registerOption("quad-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([this, p, r, numPhi]() { scene->add(SceneGraph::createQuadSphere(p,r,numPhi,new OBJMaterial)); });
      }, "--quad-sphere p.x p.y p.z r numPhi: adds a sphere at position p with radius r and tessellation numPhi build of quadrilaterals.");

    registerOption("grid-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        const size_t N = cin->getInt();
        futures.push_back([this, p, r, N]() { scene->add(SceneGraph::createGridSphere(p,r,N,new OBJMaterial)); });
      }, "--grid-sphere p.x p.y p.z r N: adds a grid sphere at position p with radius r using a cube topology and N*N quads at each face.");

    registerOption("triangle-sphere-mblur", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const Vec3f dp = cin->getVec3f();
        const float  r = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([this, p, dp, r, numPhi]() {
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
        futures.push_back([this, p, dp, r, numPhi]() {
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
        futures.push_back([this, p, r, numPhi, tessellationRate]() { scene->add(SceneGraph::createSubdivSphere(p,r,numPhi,tessellationRate,new OBJMaterial)); });
      }, "--subdiv-sphere p.x p.y p.z r numPhi: adds a sphere at position p with radius r build of Catmull Clark subdivision surfaces. The sphere consists of numPhi x numPhi many patches and each path has the specified tessellation rate.");

    registerOption("point-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        const float pointR = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([this, p, r, pointR, numPhi]() { scene->add(SceneGraph::createPointSphere(p, r, pointR, numPhi, SceneGraph::SPHERE, new OBJMaterial)); });
      }, "--point-sphere p.x p.y p.z r pointR numPhi: adds a sphere at position p with radius r and tessellation numPhi build of spheres.");

     registerOption("point-sphere-mblur", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const Vec3f dp = cin->getVec3f();
        const float  r = cin->getFloat();
        const float pointR = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([this, p, dp, r, pointR, numPhi]() { scene->add(SceneGraph::createPointSphere(p, r, pointR, numPhi, SceneGraph::SPHERE, new OBJMaterial)->set_motion_vector(dp)); });
      }, "--point-sphere p.x p.y p.z d.x d.y d.z r pointR numPhi: adds a sphere at position p, motion vector d, with radius r and tessellation numPhi build of spheres.");

    registerOption("disc-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        const float pointR = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([this, p, r, pointR, numPhi]() { scene->add(SceneGraph::createPointSphere(p, r, pointR, numPhi, SceneGraph::DISC, new OBJMaterial)); });
      }, "--disc-sphere p.x p.y p.z r pointR numPhi: adds a sphere at position p with radius r and tessellation numPhi build of discs.");

    registerOption("oriented-disc-sphere", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3f p = cin->getVec3f();
        const float  r = cin->getFloat();
        const float pointR = cin->getFloat();
        const size_t numPhi = cin->getInt();
        futures.push_back([this, p, r, pointR, numPhi]() { scene->add(SceneGraph::createPointSphere(p, r, pointR, numPhi, SceneGraph::ORIENTED_DISC, new OBJMaterial)); });
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
    
    for (unsigned int i=0; i<numFrames; i++)
      render(pixels,width,height,render_time,ispccamera);
    
    Ref<Image> image = new Image4uc(width, height, (Col4uc*)pixels);
    storeImage(image, fileName);
  }

  void TutorialApplication::compareToReferenceImage(const FileName& fileName)
  {
    resize(width,height);
    ISPCCamera ispccamera = camera.getISPCCamera(width,height);
    initRayStats();
    
    for (unsigned int i=0; i<numFrames; i++)
      render(pixels,width,height,render_time,ispccamera);

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

    this->width = width;
    this->height = height;
      
    if (pixels) alignedUSMFree(pixels);
    pixels = (unsigned*) alignedUSMMalloc(width*height*sizeof(unsigned),64,EmbreeUSMMode::DEVICE_READ_WRITE);
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

  void TutorialApplication::setCallbackFunctions(GLFWwindow* window)
  {
    glfwSetKeyCallback(window,embree::keyboardFunc);
    glfwSetCursorPosCallback(window,embree::motionFunc);
    glfwSetMouseButtonCallback(window,embree::clickFunc);
    glfwSetCharCallback(window, ImGui_ImplGlfw_CharCallback);
    glfwSetScrollCallback(window, ImGui_ImplGlfw_ScrollCallback);
    glfwSetWindowSizeCallback(window,embree::reshapeFunc);
    glfwSetWindowFocusCallback(window, ImGui_ImplGlfw_WindowFocusCallback);
    glfwSetCursorEnterCallback(window, ImGui_ImplGlfw_CursorEnterCallback);
    glfwSetMonitorCallback(ImGui_ImplGlfw_MonitorCallback);
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
    resize(mode->width,mode->height);
    return window;
  }

  GLFWwindow* TutorialApplication::createStandardWindow(int width, int height)
  {
    GLFWwindow* window = glfwCreateWindow(width,height,tutorialName.c_str(),nullptr,nullptr);
    resize(width,height);
    return window;
  }

  /* called when a key is pressed */
  void TutorialApplication::keypressed(int key)
  {
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
            setCallbackFunctions(window);
          }
          else {
            window_width = width;
            window_height = height;
            window = createFullScreenWindow();
            setCallbackFunctions(window);
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
      else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        printf("pixel pos (%d, %d)\n", (int)x, (int)y);
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
    ImGui_ImplGlfw_CursorPosCallback(window, x, y);
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
    const float time = g_motion_blur ? float(t0-time0) : (animate ? 0.5 * sinf(fabsf(float(t0-time0))) + 0.5 : render_time);
    
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
    if (ispccamera.render_time != 0.0) dt0 = ispccamera.render_time;
    avg_render_time.add(dt0);
    double mrayps = double(getNumRays())/(1000000.0*dt0);
    avg_mrayps.add(mrayps);

    /* draw pixels to screen */
    glRasterPos2i(-1,1);
    glPixelZoom(1.0f,-1.0f);
    glDrawPixels(width,height,GL_RGBA,GL_UNSIGNED_BYTE,pixels);

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    
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
    
    double render_dt = avg_render_time.get();
    double render_fps = render_dt != 0.0 ? 1.0f/render_dt : 0.0;
    ImGui::Text("Render: %3.2f fps",render_fps);

    double total_dt = avg_frame_time.get();
    double total_fps = total_dt != 0.0 ? 1.0f/total_dt : 0.0;
    ImGui::Text("Total: %3.2f fps",total_fps);

#if defined(RAY_STATS) && !defined(EMBREE_SYCL_TUTORIAL)
    ImGui::Text("%3.2f Mray/s",avg_mrayps.get());
#endif
    ImGui::End();
     
    //ImGui::ShowDemoWindow();
        
    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    
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
#if defined(RAY_STATS) && !defined(EMBREE_SYCL_TUTORIAL)
      stream << mrayps << " Mray/s, ";
#endif
      stream << "total: ";
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
    glfwSwapInterval(0);
    reshapeFunc(window,0,0);

    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();

    setCallbackFunctions(window);

    // Setup style
    ImGui::StyleColorsDark();

    while (!glfwWindowShouldClose(window))
    {
      glfwPollEvents();

      displayFunc();
    }

    // Cleanup
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
  }

#endif
  
  void TutorialApplication::render(unsigned* pixels, const unsigned width, const unsigned height, const float time, const ISPCCamera& camera)
  {
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
    device_init(rtcore.c_str());

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

  void TutorialApplication::create_device()
  {
#if defined(EMBREE_SYCL_SUPPORT) && defined(EMBREE_SYCL_TUTORIAL)

    /* create SYCL device */
    if (features & FEATURE_SYCL)
    {
      if (jit_cache)
      {
        /* enable SYCL JIT caching */
        FileName exe = getExecutableFileName();
        FileName cache_dir = exe.path() + FileName("cache");
        
#if defined(__WIN32__)
        _putenv_s("SYCL_CACHE_PERSISTENT","1");
        _putenv_s("SYCL_CACHE_DIR",cache_dir.c_str());
#else
        setenv("SYCL_CACHE_PERSISTENT","1",1);
        setenv("SYCL_CACHE_DIR",cache_dir.c_str(),1);
#endif
      }

      auto exception_handler = [](sycl::exception_list exceptions)
      {
        for (std::exception_ptr const &e : exceptions) {
          try {
            std::rethrow_exception(e);
          } catch (sycl::exception const &e) {
            std::cout << "ERROR: Caught asynchronous SYCL exception:\n"
                      << e.what() << std::endl;
            exit(1);
          }
        }
      };

      /* select device supported by Embree */
      try {
        device = new sycl::device(rtcSYCLDeviceSelector);
      } catch(std::exception& e) {
        std::cerr << "Caught exception creating sycl::device: " << e.what() << std::endl;
        printAllSYCLDevices();
        throw;
      }
      sycl::platform platform = device->get_platform();
      log(1, "Selected SYCL Platform: " + platform.get_info<sycl::info::platform::name>());
      log(1, "Selected SYCL Device: " + device->get_info<sycl::info::device::name>());

      context = new sycl::context(*device);
      queue = new sycl::queue(*context, *device, exception_handler, { sycl::property::queue::in_order(), sycl::property::queue::enable_profiling() });
      g_device = rtcNewSYCLDevice(*context,rtcore.c_str());
      error_handler(nullptr,rtcGetDeviceError(g_device),rtcGetDeviceLastErrorMessage(g_device));
      global_gpu_device = device;
      global_gpu_context = context;
      global_gpu_queue = queue;

      if (verbosity >= 1) {
        printAllSYCLDevices();
      }

      enableUSMAllocTutorial(global_gpu_context, global_gpu_device);
    }

    /* create standard device */
    else
#endif
      
    {
      g_device = rtcNewDevice(rtcore.c_str());
      error_handler(nullptr,rtcGetDeviceError(g_device));
    }
    
    /* set error handler */
    rtcSetDeviceErrorFunction(g_device,error_handler,nullptr);
  }
    
  int TutorialApplication::main(int argc, char** argv) try
  {
    /* parse command line options */
    parseCommandLine(argc,argv);

    /* callback */
    postParseCommandLine();

    /* create embree device */
    create_device();
  
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

    /* create embree device */
    create_device();
    
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
