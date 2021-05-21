// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../default.h"
#include "application.h"
#include "camera.h"
#include "scene.h"
#include "scene_device.h"

#if defined(USE_GLFW)

/* include GLFW for window management */
#include <GLFW/glfw3.h>

/* include ImGUI */
#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_glfw_gl2.h"

#endif

namespace embree
{
  /* functions provided by each tutorial */
  extern "C" void device_init(const char* cfg);
  extern "C" void device_resize(int width, int height);
  extern "C" void device_render(unsigned* pixels, const unsigned width, const unsigned height, const float time, const ISPCCamera& camera);
  extern "C" bool device_pick(const float x, const float y, const ISPCCamera& camera, Vec3fa& hitPos);
  //extern "C" void device_key_pressed (int key);
  extern "C" void device_cleanup();

  template<typename Ty>
    struct Averaged
  {
    Averaged (size_t N, double dt)
    : N(N), dt(dt) {}

    void add(double v)
    {
      values.push_front(std::make_pair(getSeconds(),v));
      if (values.size() > N) values.resize(N);
    }

    Ty get() const
    {
      if (values.size() == 0) return zero;
      double t_begin = values[0].first-dt;

      Ty sum(zero);
      size_t num(0);
      for (size_t i=0; i<values.size(); i++) {
        if (values[i].first >= t_begin) {
          sum += values[i].second;
          num++;
        }
      }
      if (num == 0) return 0;
      else return sum/Ty(num);
    }

    std::deque<std::pair<double,Ty>> values;
    size_t N;
    double dt;
  };

  class TutorialApplication : public Application
  {
  public:
    TutorialApplication (const std::string& tutorialName, const int features);
    virtual ~TutorialApplication();

  private:
    TutorialApplication (const TutorialApplication& other) DELETED; // do not implement
    TutorialApplication& operator= (const TutorialApplication& other) DELETED; // do not implement

  public:
    /* starts tutorial */
    void run(int argc, char** argv);

    /* virtual main function, contains all tutorial logic */
    virtual int main(int argc, char** argv);

    /* callback called after command line parsing finished */
    virtual void postParseCommandLine() {}

    /* render to file mode */
    void renderToFile(const FileName& fileName);

    /* compare rendering to reference image */
    void compareToReferenceImage(const FileName& fileName);

    /* passes parameters to the backend */
    void set_parameter(size_t parm, ssize_t val);

    /* resize framebuffer */
    void resize(unsigned width, unsigned height);

    /* set scene to use */
    void set_scene (TutorialScene* in);

#if defined(USE_GLFW)
    
  public:
    
    /* create a fullscreen window */
    GLFWwindow* createFullScreenWindow();

    /* create a standard window of specified size */
    GLFWwindow* createStandardWindow(int width, int height);

    /* interactive rendering using GLFW window */
    void renderInteractive();
 
    /* GLFW callback functions */
  public:
    virtual void keypressed(int key);
    virtual void keyboardFunc(GLFWwindow* window, int key, int scancode, int action, int mods);
    virtual void clickFunc(GLFWwindow* window, int button, int action, int mods);
    virtual void motionFunc(GLFWwindow* window, double x, double y);
    virtual void displayFunc();
    virtual void reshapeFunc(GLFWwindow* window, int width, int height);
    virtual void drawGUI() {};

  public:
    GLFWwindow* window = nullptr;

#endif
    
  public:
    virtual void render(unsigned* pixels, const unsigned width, const unsigned height, const float time, const ISPCCamera& camera);
  
  public:
    std::string tutorialName;

    /* render settings */
    Ref<SceneGraph::PerspectiveCameraNode> animated_camera;
    Camera camera;
    Shader shader;

    /* framebuffer settings */
    unsigned width;
    unsigned height;
    unsigned* pixels;

    /* image output settings */
    FileName outputImageFilename;
    FileName referenceImageFilename;
    float referenceImageThreshold; // threshold when we consider images to differ

    /* window settings */
    bool interactive;
    bool fullscreen;

    unsigned window_width;
    unsigned window_height;
  
    double time0;
    int debug_int0;
    int debug_int1;

    int mouseMode;
    double clickX, clickY;

    float speed;
    Vec3f moveDelta;

    bool command_line_camera;
    bool print_frame_rate;
    Averaged<double> avg_render_time;
    Averaged<double> avg_frame_time;
    Averaged<double> avg_mrayps;
    bool print_camera;

    int debug0;
    int debug1;
    int debug2;
    int debug3;

    RTCIntersectContextFlags iflags_coherent;
    RTCIntersectContextFlags iflags_incoherent;

    std::unique_ptr<ISPCScene> ispc_scene;
    
    /* ray statistics */
    void initRayStats();
    int64_t getNumRays();

  public:
    static TutorialApplication* instance;
  };

  class SceneLoadingTutorialApplication : public TutorialApplication
  {
  public:
    SceneLoadingTutorialApplication (const std::string& tutorialName, int features);

    virtual int main(int argc, char** argv);

    bool scene_empty_post_parse() const {
      return scene->size() == 0 && sceneFilename.size() == 0 && futures.size() == 0;
    }

  public:
    TutorialScene obj_scene;
    Ref<SceneGraph::GroupNode> scene;

    enum SceneGraphOperations
    {
      CONVERT_TRIANGLES_TO_QUADS,
      CONVERT_BEZIER_TO_LINES,
      CONVERT_BEZIER_TO_BSPLINE,
      CONVERT_BEZIER_TO_HERMITE,
      CONVERT_BSPLINE_TO_BEZIER,
      CONVERT_FLAT_TO_ROUND_CURVES,
      CONVERT_ROUND_TO_FLAT_CURVES,
      MERGE_QUADS_TO_GRIDS,
      CONVERT_QUADS_TO_GRIDS,
      CONVERT_GRIDS_TO_QUADS,
      CONVERT_MBLUR_TO_NONMBLUR,
    };
    std::vector<SceneGraphOperations> sgop;
    std::vector<std::function<void()>> futures; // future scene graph operations

    float convert_tris_to_quads_prop;
    unsigned grid_resX, grid_resY;
    bool remove_mblur;
    bool remove_non_mblur;
    std::vector<FileName> sceneFilename;
    std::vector<FileName> keyFramesFilenames;
    SceneGraph::InstancingMode instancing_mode;
    std::string subdiv_mode;
    bool print_scene_cameras;
    std::string camera_name;
  };
}
