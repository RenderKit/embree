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

#include "tutorial/tutorial.h"
#include "tutorial/obj_loader.h"
#include "tutorial/xml_loader.h"
#include "image/image.h"

extern "C" void toggleOpenSubdiv(unsigned char key, int x, int y);

//extern unsigned int g_subdivision_levels;

namespace embree 
{
  /* name of the tutorial */
  const char* tutorialName = "tutorial10";

  /* configuration */
  static std::string g_rtcore = "";
  static size_t g_numThreads = 0;
  static std::string g_subdiv_mode = "";

  /* output settings */
  static size_t g_width = 1024;
  static size_t g_height = 1024;
  static bool g_fullscreen = false;
  static FileName outFilename = "";
  static bool g_interactive = true;
  static bool g_loop_mode = false;
  static bool g_anim_mode = false;

  /* scene */
  OBJScene g_obj_scene;
  static FileName filename = "";

  static std::string getParameterString(Ref<ParseStream> &cin, std::string &term) {

    /*! Parameter name and options. */
    std::string parameter = term + " ";  while (cin->peek() != "" && cin->peek()[0] != '-') parameter += cin->getString();  return(parameter);

  }

  static void initEmbreeState(std::string configuration) {

    /*! Initialize Embree state. */
    init(configuration.c_str());

  }
  
  static void parseCommandLine(Ref<ParseStream> cin, const FileName &path) {

    for (std::string term = cin->getString() ; term != "" ; term = cin->getString()) {

      /*! Command line parameters from a file. */
      if (term == "-c") { FileName file = path + cin->getFileName();  parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path()); }

      /* load OBJ model*/
      else if (term == "-i") {
        filename = path + cin->getFileName();
      }

      /*! Camera field of view. */
      else if (term == "-fov") g_camera.fov = cin->getFloat();

      /*! Full screen mode. */
      else if (term == "-fullscreen") g_fullscreen = true;

      /* output filename */
	  else if (term == "-o") {
		  g_interactive = false;
		  outFilename = cin->getFileName();
	  }
      
      /*! Embree configuration. */
      else if (term == "-rtcore") g_rtcore = cin->getString();

      /*! Window size. */
      else if (term == "-size") { g_width = cin->getInt();  g_height = cin->getInt(); }

      /*! Thread count. */
      else if (term == "-threads") { g_numThreads = cin->getInt(); }

      /*! Camera view direction. */
      else if (term == "-vd") g_camera.to = g_camera.from + cin->getVec3fa();

      /*! Camera look point. */
      else if (term == "-vi") g_camera.to = cin->getVec3fa();

      /*! Camera position. */
      else if (term == "-vp") g_camera.from = cin->getVec3fa();

      /*! Camera up vector. */
      else if (term == "-vu") g_camera.up = cin->getVec3fa();

      else if (term == "-cache") 
	g_subdiv_mode = ",subdiv_accel=bvh4.subdivpatch1cached";

      else if (term == "-lazy") 
	g_subdiv_mode = ",subdiv_accel=bvh4.grid.lazy";

      else if (term == "-pregenerate") 
	g_subdiv_mode = ",subdiv_accel=bvh4.grid.eager";

      else if (term == "-loop") 
	g_loop_mode = true;

      else if (term == "-anim") 
	g_anim_mode = true;

      /*! Skip unknown command line parameters. */
      else std::cerr << "Unknown command line parameter: " << getParameterString(cin, term) << std::endl;

    }

  }

  void renderToFile(const FileName& fileName)
  {
    resize(g_width,g_height);
    if (g_anim_mode) g_camera.anim = true;

    do {
      double msec = getSeconds();
      AffineSpace3fa pixel2world = g_camera.pixel2world(g_width,g_height);

      render(0.0f,
	     pixel2world.l.vx,
	     pixel2world.l.vy,
	     pixel2world.l.vz,
	     pixel2world.p);
      msec = getSeconds() - msec;
      std::cout << "render time " << 1.0/msec << " fps" << std::endl;

    } while(g_loop_mode);
 
    void* ptr = map();
    Ref<Image> image = new Image4c(g_width, g_height, (Col4c*)ptr);
    storeImage(image, fileName);
    unmap();
    cleanup();
  }


  void addSubdivSphere (OBJScene &scene, const Vec3fa& pos)
  {
    OBJScene::SubdivMesh *subdiv_mesh = new OBJScene::SubdivMesh;
    scene.subdiv.push_back(subdiv_mesh);

    const float r = 4.0f;
    size_t numPhi = 400;

    size_t numTheta = 2*numPhi;
    subdiv_mesh->positions.resize( numTheta*(numPhi+1) );

    //avector<Vec3fa> vertices(numTheta*(numPhi+1));
    //std::vector<int> indices;
    //std::vector<int> faces;
    //std::vector<int> offsets;
    
    /* create sphere geometry */
    const float rcpNumTheta = rcp((float)numTheta);
    const float rcpNumPhi   = rcp((float)numPhi);
    for (int phi=0; phi<=numPhi; phi++)
    {
      for (int theta=0; theta<numTheta; theta++)
      {
	const float phif   = phi*float(pi)*rcpNumPhi;
	const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;
	Vec3fa& v = subdiv_mesh->positions[phi*numTheta+theta];
	Vec3fa P(pos.x + r*sin(phif)*sin(thetaf),
		 pos.y + r*cos(phif),
		 pos.z + r*sin(phif)*cos(thetaf));
	v.x = P.x;
	v.y = P.y;
	v.z = P.z;
      }
      if (phi == 0) continue;
      
      if (phi == 1)
      {
	// for (int theta=1; theta<=numTheta; theta++) 
	// {
	//   int p00 = numTheta-1;
	//   int p10 = phi*numTheta+theta-1;
	//   int p11 = phi*numTheta+theta%numTheta;
	//   offsets.push_back(indices.size());
	//   indices.push_back(p10); 
	//   indices.push_back(p00);
	//   indices.push_back(p11);
	//   faces.push_back(3);
	// }
      }
      else if (phi == numPhi)
      {
	// for (int theta=1; theta<=numTheta; theta++) 
	// {
	//   int p00 = (phi-1)*numTheta+theta-1;
	//   int p01 = (phi-1)*numTheta+theta%numTheta;
	//   int p10 = numPhi*numTheta;
	//   offsets.push_back(indices.size());
	//   indices.push_back(p10);
	//   indices.push_back(p00);
	//   indices.push_back(p01);
	//   faces.push_back(3);
	// }
      }
      else
      {
	for (int theta=1; theta<=numTheta; theta++) 
	{
	  int p00 = (phi-1)*numTheta+theta-1;
	  int p01 = (phi-1)*numTheta+theta%numTheta;
	  int p10 = phi*numTheta+theta-1;
	  int p11 = phi*numTheta+theta%numTheta;
	  //subdiv_mesh->offsets.push_back(indices.size());
	  subdiv_mesh->position_indices.push_back(p10);
	  subdiv_mesh->position_indices.push_back(p00);
	  subdiv_mesh->position_indices.push_back(p01);
	  subdiv_mesh->position_indices.push_back(p11);
	  subdiv_mesh->verticesPerFace.push_back(4);
	}
      }
    }
    
  }

  void main(int argc, char **argv) 
  {
#if defined(__USE_OPENSUBDIV__)
    mapKeyToFunction('t', toggleOpenSubdiv);
#endif
    std::cout << " === Possible cmd line options: -lazy, -pregenerate, -cache === " << std::endl;

    /*! Parse command line options. */  
    parseCommandLine(new ParseStream(new CommandLineStream(argc, argv)), FileName());

    /*! Set the thread count in the Embree configuration string. */
    if (g_numThreads) g_rtcore += ",threads=" + std::to_string((long long)g_numThreads);

    g_rtcore += g_subdiv_mode;

    /* load scene */
    if (strlwr(filename.ext()) == std::string("obj"))
      loadOBJ(filename,one,g_obj_scene,true);
      //loadOBJ(filename,one,g_obj_scene,false);
    else if (strlwr(filename.ext()) == std::string("xml"))
      loadXML(filename,one,g_obj_scene);
    else 
      {
	std::cout << "generating test sphere...." << std::endl;
	addSubdivSphere(g_obj_scene,g_camera.from);
	//THROW_RUNTIME_ERROR("invalid scene type: "+strlwr(filename.ext()));
      }

    /*! Initialize Embree state. */
    init(g_rtcore.c_str());

    /* send model */
    set_scene(&g_obj_scene);
        
    /* render to disk */
    if (outFilename.str() != "")
      renderToFile(outFilename);
    
    /* interactive mode */
    if (g_interactive) {
      initWindowState(argc,argv,tutorialName, g_width, g_height, g_fullscreen);
      
      enterWindowRunLoop(g_anim_mode);
    }

  }

}

int main(int argc, char** argv) {

  /*! Tutorial entry point. */
  try { embree::main(argc, argv);  return(0); }

  /*! Known exception. */
  catch (const std::exception& e) { std::cout << "Error: " << e.what() << std::endl;  return(1); }

  /*! Unknown exception. */
  catch (...) { std::cout << "Error: unknown exception caught." << std::endl;  return(1); }

}

