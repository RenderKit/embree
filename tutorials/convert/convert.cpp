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

#include "default.h"
#include "distribution2d.h"
#include "../common/tutorial/obj_loader.h"
#include "../common/tutorial/xml_loader.h"
#include "../common/image/image.h"

namespace embree
{
  /* name of the tutorial */
  const char* tutorialName = "convert";

  struct HeightField : public RefCount
  {
    HeightField (Ref<Image> texture, const AffineSpace3fa& space)
      : texture(texture), space(space) {}
    
    const AffineSpace3fa get(Vec2f p)
    {
      const size_t width  = texture->width;
      const size_t height = texture->height;
      const size_t x = clamp((size_t)(p.x*(width-1)),(size_t)0,width-1);
      const size_t y = clamp((size_t)(p.y*(height-1)),(size_t)0,height-1);
      const Color4 c = texture->get(x,y);
      return space*AffineSpace3fa::translate(Vec3fa(p.x,c.r,p.y));
    }
    
  private:
    Ref<Image> texture;
    AffineSpace3fa space;
  };

  struct Instantiator : public RefCount
  {
    Instantiator(const Ref<HeightField>& heightField,
                               const Ref<SceneGraph::Node>& object, const Ref<Image>& distribution, float minDistance, size_t N)
      : heightField(heightField), object(object), dist(nullptr), minDistance(minDistance), N(N)
    {
      /* create distribution */
      size_t width = distribution->width;
      size_t height = distribution->height;
      float* values = new float[width*height];
      for (size_t y=0; y<height; y++)
        for (size_t x=0; x<width; x++)
          values[y*width+x] = luminance(distribution->get(x,y));
      dist = new Distribution2D(values,width,height);
      delete values;
    }
        
    void instantiate(Ref<SceneGraph::GroupNode>& group) 
    {
      for (size_t i=0; i<N; i++) 
      {
        Vec2f r = Vec2f(random<float>(),random<float>());
        Vec2f p = dist->sample(r);
        p.x *= rcp(float(dist->width)); p.y *= rcp(float(dist->height));
        const AffineSpace3fa space = heightField->get(p);
        group->add(new SceneGraph::TransformNode(space,object));
      }
    }

  private:
    Ref<HeightField> heightField;
    Ref<SceneGraph::Node> object;
    Ref<Distribution2D> dist;
    float minDistance;
    size_t N;
  };

  /* scene */
  Ref<SceneGraph::GroupNode> g_scene = new SceneGraph::GroupNode;
  Ref<HeightField> g_height_field = nullptr;
  
  static void parseCommandLine(Ref<ParseStream> cin, const FileName& path)
  {
    while (true)
    {
      std::string tag = cin->getString();
      if (tag == "") return;

      /* parse command line parameters from a file */
      else if (tag == "-c") {
        FileName file = path + cin->getFileName();
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }

      /* load model */
      else if (tag == "-i") {
        g_scene->add(SceneGraph::load(path + cin->getFileName()));
      }

      /* load terrain */
      else if (tag == "-terrain") {
        Ref<Image> tex = loadImage(path + cin->getFileName());
        const Vec3fa lower = cin->getVec3fa();
        const Vec3fa upper = cin->getVec3fa();
        const AffineSpace3fa space = AffineSpace3fa::translate(-lower)*AffineSpace3fa::scale(upper-lower);
        g_height_field = new HeightField(tex,space);
      }

      /* instantiate model */
      else if (tag == "-instantiate") {
        Ref<SceneGraph::Node> object = SceneGraph::load(path + cin->getFileName());
        Ref<Image> distribution = loadImage(path + cin->getFileName());
        const float minDistance = cin->getFloat();
        const size_t N = cin->getInt();
        Ref<Instantiator> instantiator = new Instantiator(g_height_field,object,distribution,minDistance,N);
        instantiator->instantiate(g_scene);
      }

      /* output filename */
      else if (tag == "-o") {
        SceneGraph::store(g_scene.dynamicCast<SceneGraph::Node>(),path + cin->getFileName());
      }

      /* skip unknown command line parameter */
      else {
        std::cerr << "unknown command line parameter: " << tag << " ";
        while (cin->peek() != "" && cin->peek()[0] != '-') std::cerr << cin->getString() << " ";
        std::cerr << std::endl;
      }
    }
  }
  
  /* main function in embree namespace */
  int main(int argc, char** argv) 
  {
    /* create stream for parsing */
    Ref<ParseStream> stream = new ParseStream(new CommandLineStream(argc, argv));

    /* parse command line */  
    parseCommandLine(stream, FileName());

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
