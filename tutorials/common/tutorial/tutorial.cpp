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
#include "../transport/transport_host.h"

/* include GLUT for display */
#if defined(__MACOSX__)
#  include <OpenGL/gl.h>
#  include <GLUT/glut.h>
#elif defined(__WIN32__)
#  include <windows.h>
#  include <GL/gl.h>   
#  include <GL/glut.h>
#else
#  include <GL/gl.h>   
#  include <GL/glut.h>
#endif

/*#include <sys/time.h> // FIXME: remove
#include <sys/resource.h>
double process_time()
{
  struct rusage usage;
  getrusage(RUSAGE_THREAD,&usage);
  return double(usage.ru_utime.tv_sec) + double(usage.ru_utime.tv_usec)/1E6;
  }*/

extern "C" {
  float g_debug = 0.0f;
}

namespace embree
{
  std::string g_tutorialName;

  TutorialApplication* TutorialApplication::instance = nullptr; 

  TutorialApplication::TutorialApplication (const std::string& tutorialName)

    : tutorialName(tutorialName),

      rtcore(""),

      shader(SHADER_DEFAULT),

      width(512),
      height(512),

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
      speed(1.0f)
  {
    /* only a single instance of this class is supported */
    assert(instance == nullptr);
    instance = this;

    /* the coi device needs this name in a static variable */
    g_tutorialName = tutorialName;

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
    
    registerOption("o", [this] (Ref<ParseStream> cin, const FileName& path) {
        outputImageFilename = cin->getFileName();
        interactive = false;
      }, "-o: output image filename");
    
    /* camera settings */
    registerOption("vp", [this] (Ref<ParseStream> cin, const FileName& path) {
        camera.from = cin->getVec3fa();
      }, "--vp <float> <float> <float>: camera position");
    
    registerOption("vi", [this] (Ref<ParseStream> cin, const FileName& path) {
        camera.to = cin->getVec3fa();
      }, "--vi <float> <float> <float>: camera lookat position");
    
    registerOption("vd", [this] (Ref<ParseStream> cin, const FileName& path) {
        camera.to = camera.from + cin->getVec3fa();
      }, "--vd <float> <float> <float>: camera direction vector");
    
    registerOption("vu", [this] (Ref<ParseStream> cin, const FileName& path) {
        camera.up = cin->getVec3fa();
      }, "--vu <float> <float> <float>: camera up vector");
    
    registerOption("fov", [this] (Ref<ParseStream> cin, const FileName& path) {
        camera.fov = cin->getFloat();
      }, "--fov <float>: vertical field of view");
    
    /* framebuffer settings */
    registerOption("size", [this] (Ref<ParseStream> cin, const FileName& path) {
        width = cin->getInt();
        height = cin->getInt();
      }, "--size <width> <height>: image size");
    
    registerOption("fullscreen", [this] (Ref<ParseStream> cin, const FileName& path) {
        fullscreen = true;
      }, "--fullscreen: starts in fullscreen mode");
    
    registerOption("rtcore", [this] (Ref<ParseStream> cin, const FileName& path) {
        rtcore += "," + cin->getString();
      }, "--rtcore <string>: uses <string> to configure Embree device");
    
    registerOption("threads", [this] (Ref<ParseStream> cin, const FileName& path) {
        rtcore += ",threads=" + toString(cin->getInt());
      }, "--threads <int>: number of threads to use");
    
    registerOption("affinity", [this] (Ref<ParseStream> cin, const FileName& path) {
        rtcore += ",set_affinity=1";
      }, "--affinity: affinitize threads");
    
    registerOption("verbose", [this] (Ref<ParseStream> cin, const FileName& path) {
        rtcore += ",verbose=" + toString(cin->getInt());
      }, "--verbose <int>: sets verbosity level");
    
    registerOption("benchmark", [this] (Ref<ParseStream> cin, const FileName& path) {
        skipBenchmarkFrames = cin->getInt();
        numBenchmarkFrames  = cin->getInt();
        if (cin->peek() != "" && cin->peek()[0] != '-')
          numBenchmarkRepetitions = cin->getInt();
          
        interactive = false;
        rtcore += ",benchmark=1";
      }, "--benchmark <N> <M> <R>: enabled benchmark mode, builds scene, skips N frames, renders M frames, and repeats this R times");
    
    /* output filename */
    registerOption("shader", [this] (Ref<ParseStream> cin, const FileName& path) {
        std::string mode = cin->getString();
        if      (mode == "default" ) shader = SHADER_DEFAULT;
        else if (mode == "eyelight") shader = SHADER_EYELIGHT;
        else if (mode == "uv"      ) shader = SHADER_UV;
        else if (mode == "Ng"      ) shader = SHADER_NG;
        else if (mode == "geomID"  ) shader = SHADER_GEOMID;
        else if (mode == "primID"  ) shader = SHADER_GEOMID_PRIMID;
        else if (mode == "ao"      ) shader = SHADER_AMBIENT_OCCLUSION;
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
  }

  SceneLoadingTutorialApplication::SceneLoadingTutorialApplication (const std::string& tutorialName)

    : TutorialApplication(tutorialName),
      scene(new SceneGraph::GroupNode),
      convert_tris_to_quads(false),
      convert_bezier_to_lines(false),
      convert_hair_to_curves(false),
      sceneFilename(""),
      instancing_mode(0),
      subdiv_mode("")
  {
    registerOption("i", [this] (Ref<ParseStream> cin, const FileName& path) {
        sceneFilename = path + cin->getFileName();
      }, "-i <filename>: parses scene from <filename>");
    
    registerOption("convert-triangles-to-quads", [this] (Ref<ParseStream> cin, const FileName& path) {
        convert_tris_to_quads = true;
      }, "--convert-triangles-to-quads: converts all triangles to quads when loading");
    
    registerOption("convert-bezier-to-lines", [this] (Ref<ParseStream> cin, const FileName& path) {
        convert_bezier_to_lines = true;
      }, "--convert-bezier-to-lines: converts all bezier curves to line segments when loading");
    
    registerOption("convert-hair-to-curves", [this] (Ref<ParseStream> cin, const FileName& path) {
        convert_hair_to_curves = true;
      }, "--convert-hair-to-curves: converts all hair geometry to curves when loading");
    
    registerOption("instancing", [this] (Ref<ParseStream> cin, const FileName& path) {
        std::string mode = cin->getString();
        if      (mode == "none"    ) instancing_mode = TutorialScene::INSTANCING_NONE;
        //else if (mode == "geometry") instancing_mode = TutorialScene::INSTANCING_GEOMETRY;
        else if (mode == "scene_geometry") instancing_mode = TutorialScene::INSTANCING_SCENE_GEOMETRY;
        else if (mode == "scene_group"   ) instancing_mode = TutorialScene::INSTANCING_SCENE_GROUP;
        else throw std::runtime_error("unknown instancing mode: "+mode);
      }, "--instancing: set instancing mode\n"
      "  none: no instancing\n"
      "  geometry: instance individual geometries\n"
      "  scene_geometry: instance individual geometries as scenes\n"
      "  scene_group: instance geometry groups as scenes\n");
    
    registerOption("ambientlight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa L = cin->getVec3fa();
        scene->add(new SceneGraph::LightNode<AmbientLight>(AmbientLight(L)));
      }, "--ambientlight r g b: adds an ambient light with intensity rgb");
    registerOptionAlias("ambientlight","ambient");

    registerOption("pointlight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa P = cin->getVec3fa();
        const Vec3fa I = cin->getVec3fa();
        scene->add(new SceneGraph::LightNode<PointLight>(PointLight(P,I)));
      }, "--pointlight x y z r g b: adds a point light at position xyz with intensity rgb");
    
    registerOption("directionallight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa D = cin->getVec3fa();
        const Vec3fa E = cin->getVec3fa();
        scene->add(new SceneGraph::LightNode<DirectionalLight>(DirectionalLight(D,E)));
      }, "--directionallight x y z r g b: adds a directional light with direction xyz and intensity rgb");
    registerOptionAlias("directionallight","dirlight");
    
    registerOption("distantlight", [this] (Ref<ParseStream> cin, const FileName& path) {
        const Vec3fa D = cin->getVec3fa();
        const Vec3fa L = cin->getVec3fa();
        const float halfAngle = cin->getFloat();
        scene->add(new SceneGraph::LightNode<DistantLight>(DistantLight(D,L,halfAngle)));
      }, "--distantlight x y z r g b a: adds a distant light with direction xyz, intensity rgb, and opening angle a");

    registerOption("cache", [this] (Ref<ParseStream> cin, const FileName& path) {
        subdiv_mode = ",subdiv_accel=bvh4.subdivpatch1cached";
        rtcore += subdiv_mode;
      }, "--cache: enabled cached subdiv mode");
    
    registerOption("pregenerate", [this] (Ref<ParseStream> cin, const FileName& path) {
        subdiv_mode = ",subdiv_accel=bvh4.grid.eager";
        rtcore += subdiv_mode;
      }, "--pregenerate: enabled pregenerate subdiv mode");    
  }
  
  void TutorialApplication::registerOptionAlias(const std::string& name, const std::string& alternativeName) {
    commandLineOptionMap[alternativeName] = commandLineOptionMap[name];
  }
  
  void TutorialApplication::parseCommandLine(int argc, char** argv)
  {
    /* create stream for parsing */
    Ref<ParseStream> stream = new ParseStream(new CommandLineStream(argc, argv));
    
    /* parse command line */  
    parseCommandLine(stream, FileName());
    
    /* callback */
    postParseCommandLine();
  }
  
  void TutorialApplication::parseCommandLine(Ref<ParseStream> cin, const FileName& path)
  {
    while (true)
    {
      std::string tag = cin->getString();
      if (tag == "") return;
      std::string tag0 = tag;
      
      /* remove - or -- and lookup command line option */
      if (tag.find("-") == 0) 
      {
        tag = tag.substr(1);
        if (tag.find("-") == 0) tag = tag.substr(1);
        auto option = commandLineOptionMap.find(tag);
      
        /* process command line option */
        if (option != commandLineOptionMap.end()) {
          option->second->parse(cin,path);
          continue;
        }
      }
      
      /* handle unknown command line options */
      std::cerr << "unknown command line parameter: " << tag0 << " ";
      while (cin->peek() != "" && cin->peek()[0] != '-') std::cerr << cin->getString() << " ";
      std::cerr << std::endl;
    }
  }

  void TutorialApplication::printCommandLineHelp()
  {
    for (auto c : commandLineOptionList) {
      std::cout << c->description << std::endl;
    }
  }

  /* calculates min/avg/max and sigma */
  struct Statistics
  {
  public:
    Statistics() 
      : v(0.0f), v2(0.0f), vmin(pos_inf), vmax(neg_inf), N(0) {}

    void add(float a) 
    {
      v += a;
      v2 += a*a;
      vmin = min(vmin,a);
      vmax = max(vmax,a);
      N++;
    }

    float getSigma() const 
    {
      if (N == 0) return 0.0f;
      else return sqrt(max(0.0,v2/N - sqr(v/N)));
    }

    float getAvgSigma() const // standard deviation of average
    {
      if (N == 0) return 0.0f;
      else return getSigma()/sqrt(float(N));
    }

    float getMin() const { return vmin; }
    float getMax() const { return vmax; }
    float getAvg() const { return v/N; }

  private:
    double v;   // sum of all values
    double v2;  // sum of squared of all values
    float vmin; // min of all values
    float vmax; // max of all values
    size_t N;  // number of values
  };

  /* filters outlyers out */
  struct FilteredStatistics
  {
  public:
    FilteredStatistics(float fskip_small, float fskip_large) 
      : fskip_small(fskip_small), fskip_large(fskip_large) {}

    void add(float a) 
    {
      v.push_back(a);
      std::sort(v.begin(),v.end(),std::less<float>());
      size_t skip_small = floor(0.5f*fskip_small*v.size());
      size_t skip_large = floor(0.5f*fskip_large*v.size());

      new (&stat) Statistics;
      for (size_t i=skip_small; i<v.size()-skip_large; i++)
        stat.add(v[i]);
    }

    float getSigma() const { return stat.getSigma(); }
    float getAvgSigma() const { return stat.getAvgSigma(); }
    float getMin() const { return stat.getMin(); }
    float getMax() const { return stat.getMax(); }
    float getAvg() const { return stat.getAvg(); }

  private:
    float fskip_small; // fraction of small outlyers to filter out
    float fskip_large; // fraction of large outlyers to filter out
    std::vector<float> v;
    Statistics stat;
  };

  void TutorialApplication::renderBenchmark()
  {
    IOStreamStateRestorer cout_state(std::cout);
    std::cout.setf(std::ios::fixed, std::ios::floatfield);
    std::cout.precision(4);

    resize(width,height);
    ISPCCamera ispccamera = camera.getISPCCamera(width,height);

    //Statistics stat;
    FilteredStatistics stat(0.5f,0.0f);
    for (size_t j=0; j<numBenchmarkRepetitions; j++)
    {
      size_t numTotalFrames = skipBenchmarkFrames + numBenchmarkFrames;
      for (size_t i=0; i<skipBenchmarkFrames; i++) 
      {
        double t0 = getSeconds();
        render(0.0f,ispccamera);
        double t1 = getSeconds();
        std::cout << "frame [" << std::setw(3) << i << " / " << std::setw(3) << numTotalFrames << "]: " <<  std::setw(8) << 1.0/(t1-t0) << " fps (skipped)" << std::endl << std::flush;
        usleep(100000);
      }
      
      for (size_t i=skipBenchmarkFrames; i<numTotalFrames; i++) 
      {
        double t0 = getSeconds();
        render(0.0f,ispccamera);
        double t1 = getSeconds();

        float fr = 1.0f/(t1-t0);
        stat.add(fr);
        std::cout << "frame [" << std::setw(3) << i << " / " << std::setw(3) << numTotalFrames << "]: " 
                  << std::setw(8) << fr << " fps, " 
                  << "min = " << std::setw(8) << stat.getMin() << " fps, " 
                  << "avg = " << std::setw(8) << stat.getAvg() << " fps, "
                  << "max = " << std::setw(8) << stat.getMax() << " fps, "
                  << "sigma = " << std::setw(6) << stat.getSigma() << " (" << 100.0f*stat.getSigma()/stat.getAvg() << "%)" << std::endl << std::flush;
        usleep(100000);
      }

      /* rebuild scene between repetitions */
      if (numBenchmarkRepetitions)
      {
        cleanup();
        init(rtcore.c_str());
        resize(width,height);     
      }
 
      std::cout << "frame [" << std::setw(3) << skipBenchmarkFrames << " - " << std::setw(3) << numTotalFrames << "]: " 
                << "              " 
                << "min = " << std::setw(8) << stat.getMin() << " fps, " 
                << "avg = " << std::setw(8) << stat.getAvg() << " fps, "
                << "max = " << std::setw(8) << stat.getMax() << " fps, "
                << "sigma = " << std::setw(6) << stat.getAvgSigma() << " (" << 100.0f*stat.getAvgSigma()/stat.getAvg() << "%)" << std::endl;
    }

    std::cout << "BENCHMARK_RENDER_MIN " << stat.getMin() << std::endl;
    std::cout << "BENCHMARK_RENDER_AVG " << stat.getAvg() << std::endl;
    std::cout << "BENCHMARK_RENDER_MAX " << stat.getMax() << std::endl;
    std::cout << "BENCHMARK_RENDER_SIGMA " << stat.getSigma() << std::endl;
    std::cout << "BENCHMARK_RENDER_AVG_SIGMA " << stat.getAvgSigma() << std::endl << std::flush;
    usleep(100000);
  }

  void TutorialApplication::renderToFile(const FileName& fileName)
  {
    resize(width,height);
    ISPCCamera ispccamera = camera.getISPCCamera(width,height);
    render(0.0f,ispccamera);
    void* ptr = map();
    Ref<Image> image = new Image4uc(width, height, (Col4uc*)ptr);
    storeImage(image, fileName);
    unmap();
    cleanup();
  }

  void TutorialApplication::keyboardFunc(unsigned char key, int x, int y)
  {
    /* call tutorial keyboard handler */
    key_pressed(key);

    switch (key)
    {
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
    case 'c' : {
      std::cout.precision(10);
      std::cout << "-vp " << camera.from.x    << " " << camera.from.y    << " " << camera.from.z    << " " 
                << "-vi " << camera.to.x << " " << camera.to.y << " " << camera.to.z << " " 
                << "-vu " << camera.up.x     << " " << camera.up.y     << " " << camera.up.z     << " " 
                << "-fov " << camera.fov << std::endl;
      break;
    }
    case '+' : g_debug=clamp(g_debug+0.01f); PRINT(g_debug); break;
    case '-' : g_debug=clamp(g_debug-0.01f); PRINT(g_debug); break;

    case '\033': case 'q': case 'Q':
      cleanup();
      glutDestroyWindow(windowID);
#if defined(__MACOSX__)
      exit(1);
#endif
      break;
    }
  }

  void TutorialApplication::specialFunc(int key, int x, int y)
  {
    key_pressed(key);
    
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
      if (button == GLUT_LEFT_BUTTON && glutGetModifiers() == GLUT_ACTIVE_SHIFT) 
      {
        ISPCCamera ispccamera = camera.getISPCCamera(width,height);
        Vec3fa p; bool hit = pick(x,y,ispccamera,p);

        if (hit) {
          Vec3fa delta = p - camera.to;
          Vec3fa right = normalize(ispccamera.xfm.l.vx);
          Vec3fa up    = normalize(ispccamera.xfm.l.vy);
          camera.to = p;
          camera.from += dot(delta,right)*right + dot(delta,up)*up;
        }
      }
      else if (button == GLUT_LEFT_BUTTON && glutGetModifiers() == (GLUT_ACTIVE_CTRL | GLUT_ACTIVE_SHIFT)) 
      {
        ISPCCamera ispccamera = camera.getISPCCamera(width,height);
        Vec3fa p; bool hit = pick(x,y,ispccamera,p);
        if (hit) camera.to = p;
      }

    } else {
      clickX = x; clickY = y;
      int modifiers = glutGetModifiers();
      if      (button == GLUT_LEFT_BUTTON && modifiers == GLUT_ACTIVE_SHIFT) mouseMode = 1;
      else if (button == GLUT_MIDDLE_BUTTON) mouseMode = 2;
      else if (button == GLUT_RIGHT_BUTTON ) mouseMode = 3;
      else if (button == GLUT_LEFT_BUTTON && modifiers == GLUT_ACTIVE_CTRL ) mouseMode = 3;
      else if (button == GLUT_LEFT_BUTTON  ) mouseMode = 4;
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

  void TutorialApplication::displayFunc(void) 
  {
    ISPCCamera ispccamera = camera.getISPCCamera(width,height,true);
    
    /* render image using ISPC */
    double t0 = getSeconds();
    render(time0-t0,ispccamera);
    double dt0 = getSeconds()-t0;

    /* draw pixels to screen */
    int* pixels = map();
    glDrawPixels(width,height,GL_RGBA,GL_UNSIGNED_BYTE,pixels);
    
    if (fullscreen) 
    {
      glMatrixMode( GL_PROJECTION );
      glPushMatrix();
      glLoadIdentity();
      gluOrtho2D( 0, width, 0, height );
      glMatrixMode( GL_MODELVIEW );
      glPushMatrix();
      glLoadIdentity();
      
      /* print frame rate */
      std::ostringstream stream;
      stream.setf(std::ios::fixed, std::ios::floatfield);
      stream.precision(2);
      stream << 1.0f/dt0 << " fps";
      std::string str = stream.str();
      
      glRasterPos2i( width-str.size()*12, height - 24); 
      for ( int i = 0; i < str.size(); ++i )
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, str[i]);
      
      glRasterPos2i( 0, 0 ); 
      glPopMatrix();
      glMatrixMode( GL_PROJECTION );
      glPopMatrix();
      glMatrixMode( GL_MODELVIEW );
    }
    
    glutSwapBuffers();
    unmap();
    
    double dt1 = getSeconds()-t0;

    /* print frame rate */
    std::ostringstream stream;
    stream.setf(std::ios::fixed, std::ios::floatfield);
    stream.precision(2);
    stream << "render: ";
    stream << 1.0f/dt0 << " fps, ";
    stream << dt0*1000.0f << " ms, ";
    stream << "display: ";
    stream << 1.0f/dt1 << " fps, ";
    stream << dt1*1000.0f << " ms, ";
    stream << width << "x" << height << " pixels";
    std::cout << stream.str() << std::endl;
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
    /* initialize ray tracing core */
    init(rtcore.c_str());
    
    /* set shader mode */
    switch (shader) {
    case SHADER_DEFAULT : break;
    case SHADER_EYELIGHT: key_pressed(GLUT_KEY_F2); break;
    case SHADER_UV      : key_pressed(GLUT_KEY_F4); break;
    case SHADER_NG      : key_pressed(GLUT_KEY_F5); break;
    case SHADER_GEOMID  : key_pressed(GLUT_KEY_F6); break;
    case SHADER_GEOMID_PRIMID: key_pressed(GLUT_KEY_F7); break;
    case SHADER_AMBIENT_OCCLUSION: key_pressed(GLUT_KEY_F11); break;
    };
    
    /* benchmark mode */
    if (numBenchmarkFrames)
      renderBenchmark();
    
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

    /* load scene */
    if (toLowerCase(sceneFilename.ext()) == std::string("obj"))
      scene->add(loadOBJ(sceneFilename,subdiv_mode != ""));
    else if (sceneFilename.ext() != "")
      scene->add(SceneGraph::load(sceneFilename));

    /* convert triangles to quads */
    if (convert_tris_to_quads)
      scene->triangles_to_quads();
    
    /* convert bezier to lines */
    if (convert_bezier_to_lines)
      scene->bezier_to_lines();
    
    /* convert hair to curves */
    if (convert_hair_to_curves)
      scene->hair_to_curves();
    
    /* convert model */
    obj_scene.add(scene.dynamicCast<SceneGraph::Node>(),(TutorialScene::InstancingMode)instancing_mode); 
    scene = nullptr;

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
}
