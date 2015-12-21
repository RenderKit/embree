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
#include "../common/scenegraph/scenegraph.h"
#include "../common/image/image.h"

namespace embree
{
  /* name of the tutorial */
  const char* tutorialName = "convert";
  bool embedTextures = true;
  
  struct HeightField : public RefCount
  {
    ALIGNED_STRUCT;

    HeightField (Ref<Image> texture, const BBox3fa& bounds)
      : texture(texture), bounds(bounds) {}
    
    const Vec3fa at(const size_t x, const size_t y)
    {
      const size_t width  = texture->width;
      const size_t height = texture->height;
      const Color4 c = texture->get(x,y);
      const Vec2f p(x/float(width-1),y/float(height-1));
      const float px = p.x*(bounds.upper.x-bounds.lower.x) + bounds.lower.x;
      const float py = c.r*(bounds.upper.y-bounds.lower.y) + bounds.lower.y;
      const float pz = p.y*(bounds.upper.z-bounds.lower.z) + bounds.lower.z;
      return Vec3fa(px,py,pz);
    }

    const AffineSpace3fa get(Vec2f p)
    {
      const size_t width  = texture->width;
      const size_t height = texture->height;
      const size_t x = clamp((size_t)(p.x*(width-1)),(size_t)0,width-1);
      const size_t y = clamp((size_t)(p.y*(height-1)),(size_t)0,height-1);
      const Color4 c = texture->get(x,y);
      const float px = p.x*(bounds.upper.x-bounds.lower.x) + bounds.lower.x;
      const float py = c.r*(bounds.upper.y-bounds.lower.y) + bounds.lower.y;
      const float pz = p.y*(bounds.upper.z-bounds.lower.z) + bounds.lower.z;
      return AffineSpace3fa::translate(Vec3fa(px,py,pz));
    }

    Ref<SceneGraph::Node> geometry()
    {
      OBJMaterial material(1.0f,Vec3fa(1.0f),Vec3fa(0.0f),1.0f);
      Ref<SceneGraph::MaterialNode> mnode = new SceneGraph::MaterialNode((Material&)material);
      Ref<SceneGraph::TriangleMeshNode> mesh = new SceneGraph::TriangleMeshNode(mnode);

      const size_t width = texture->width;
      const size_t height = texture->height;
      
      mesh->v.resize(height*width);
      for (size_t y=0; y<height; y++) 
        for (size_t x=0; x<width; x++) 
          mesh->v[y*width+x] = at(x,y);

      mesh->triangles.resize(2*(height-1)*(width-1));
      for (size_t y=0; y<height-1; y++) {
        for (size_t x=0; x<width-1; x++) {
          const size_t p00 = (y+0)*width+(x+0);
          const size_t p01 = (y+0)*width+(x+1);
          const size_t p10 = (y+1)*width+(x+0);
          const size_t p11 = (y+1)*width+(x+1);
          const size_t tri = y*(width-1)+x;
          mesh->triangles[2*tri+0] = SceneGraph::TriangleMeshNode::Triangle(p00,p01,p10);
          mesh->triangles[2*tri+1] = SceneGraph::TriangleMeshNode::Triangle(p01,p11,p10);
        }
      }
      return mesh.dynamicCast<SceneGraph::Node>();
    }
    
  private:
    Ref<Image> texture;
    BBox3fa bounds;
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
      delete[] values;
    }
        
    void instantiate(Ref<SceneGraph::GroupNode>& group) 
    {
      for (size_t i=0; i<N; i++) 
      {
        Vec2f r = Vec2f(random<float>(),random<float>());
        Vec2f p = dist->sample(r);
        p.x *= rcp(float(dist->width)); p.y *= rcp(float(dist->height));
        float angle = 2.0f*float(pi)*random<float>();
        const AffineSpace3fa space = heightField->get(p)*AffineSpace3fa::rotate(Vec3fa(0,1,0),angle);
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

      /* convert triangles to quads */
      else if (tag == "-convert-triangles-to-quads") {
        g_scene->triangles_to_quads();
      }

      /* convert bezier to lines */
      else if (tag == "-convert-bezier-to-lines") {
        g_scene->bezier_to_lines();
      }

      /* load terrain */
      else if (tag == "-terrain") 
      {
        Ref<Image> tex = loadImage(path + cin->getFileName());
        const Vec3fa lower = cin->getVec3fa();
        const Vec3fa upper = cin->getVec3fa();
        g_height_field = new HeightField(tex,BBox3fa(lower,upper));
        g_scene->add(g_height_field->geometry());
      }

      /* distribute model */
      else if (tag == "-distribute") {
        Ref<SceneGraph::Node> object = SceneGraph::load(path + cin->getFileName());
        Ref<Image> distribution = loadImage(path + cin->getFileName());
        const float minDistance = cin->getFloat();
        const size_t N = cin->getInt();
        Ref<Instantiator> instantiator = new Instantiator(g_height_field,object,distribution,minDistance,N);
        instantiator->instantiate(g_scene);
      }

      /* instantiate model a single time */
      else if (tag == "-instantiate") {
        Ref<SceneGraph::Node> object = SceneGraph::load(path + cin->getFileName());
        const float px = cin->getFloat();
        const float py = cin->getFloat();
        const Vec2f p(px,py);
        const float angle = cin->getFloat()/180.0f*float(pi);
        const AffineSpace3fa space = g_height_field->get(p)*AffineSpace3fa::rotate(Vec3fa(0,1,0),angle);
        g_scene->add(new SceneGraph::TransformNode(space,object));
      }

      /* enable texture embedding */
      else if (tag == "-embed-textures") {
        embedTextures = true;
      }

      /* enable texture referencing */
      else if (tag == "-reference-textures") {
        embedTextures = false;
      }

      /* output filename */
      else if (tag == "-o") {
        SceneGraph::store(g_scene.dynamicCast<SceneGraph::Node>(),path + cin->getFileName(),embedTextures);
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
