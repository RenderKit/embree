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

#include "default.h"
#include "distribution2d.h"
#include "../common/scenegraph/scenegraph.h"
#include "../common/image/image.h"

namespace embree
{
  extern "C" {
    RTCDevice g_device = nullptr;
  }
  
  /* name of the tutorial */
  bool embedTextures = true;
  bool referenceMaterials = false;
  bool referenceObjects = true;
  float centerScale = 0.0f;
  Vec3fa centerTranslate(0.0f,0.0f,0.0f);

  struct HeightField : public RefCount
  {
    ALIGNED_STRUCT_(16);

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
      Ref<SceneGraph::MaterialNode> mnode = new OBJMaterial(1.0f,Vec3fa(1.0f),Vec3fa(0.0f),1.0f);
      Ref<SceneGraph::TriangleMeshNode> mesh = new SceneGraph::TriangleMeshNode(mnode,1);

      const size_t width = texture->width;
      const size_t height = texture->height;
      
      mesh->positions[0].resize(height*width);
      for (size_t y=0; y<height; y++) 
        for (size_t x=0; x<width; x++) 
          mesh->positions[0][y*width+x] = at(x,y);

      mesh->triangles.resize(2*(height-1)*(width-1));
      for (size_t y=0; y<height-1; y++) {
        for (size_t x=0; x<width-1; x++) {
          const size_t p00 = (y+0)*width+(x+0);
          const size_t p01 = (y+0)*width+(x+1);
          const size_t p10 = (y+1)*width+(x+0);
          const size_t p11 = (y+1)*width+(x+1);
          const size_t tri = y*(width-1)+x;
          mesh->triangles[2*tri+0] = SceneGraph::TriangleMeshNode::Triangle(unsigned(p00),unsigned(p01),unsigned(p10));
          mesh->triangles[2*tri+1] = SceneGraph::TriangleMeshNode::Triangle(unsigned(p01),unsigned(p11),unsigned(p10));
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
      : heightField(heightField), object(object), /*minDistance(minDistance),*/ N(N)
    {
      /* create distribution */
      size_t width = distribution->width;
      size_t height = distribution->height;
      std::vector<float> values(width*height);
      for (size_t y=0; y<height; y++)
        for (size_t x=0; x<width; x++)
          values[y*width+x] = luminance(distribution->get(x,y));
      dist = std::make_shared<Distribution2D>(values.data(),width,height);
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
    std::shared_ptr<Distribution2D> dist;
    //float MAYBE_UNUSED minDistance;
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
        Ref<SceneGraph::Node> object = SceneGraph::load(path + cin->getFileName());
        if (centerScale != 0.0f)
        {
          BBox3fa bb = object->bounds();
          Vec3fa center = bb.center();
          const AffineSpace3fa space = AffineSpace3fa::translate(centerTranslate)*AffineSpace3fa::scale(Vec3fa(centerScale))*AffineSpace3fa::translate(-center);
          g_scene->add(new SceneGraph::TransformNode(space,object));
        }
        else
          g_scene->add(object);
      }

      /* used to generate some test scene automatically */
      else if (tag == "-special-barbarian-instantiate") {
        for (int x=0; x<10; x++) {
          for (int y=0; y<10; y++) {
            const char* model = "barbarian_mblur.xml";
            if (random<int>()%4 == 0) model = "barbarian_msmblur_translate.xml";
            else if (random<int>()%32 == 0) model = "barbarian_msmblur_rotate0_5.xml";
            else if (random<int>()%32 == 0) model = "barbarian_msmblur.xml";
            printf("<Transform><AffineSpace translate=\"%7.2f %7.2f %7.2f\"/><extern src=\"%s\"/></Transform>\n",float(x*400)+200.0f*drand48(),0.0f,float(y*400)+200.0f*drand48(),model);
          }
        }
      }

      /* convert triangles to quads */
      else if (tag == "-convert-triangles-to-quads") {
        g_scene->triangles_to_quads();
      }

      /* convert to subdivs */
      else if (tag == "-convert-to-subdivs") {
        g_scene->triangles_to_quads();
        g_scene->quads_to_subdivs();
      }

      /* convert bezier to lines */
      else if (tag == "-convert-bezier-to-lines") {
        g_scene->bezier_to_lines();
      }

      /* convert bezier to bspline curves */
      else if (tag == "-convert-bezier-to-bspline") {
        g_scene->bezier_to_bspline();
      }

      /* convert bspline to bezier curves */
      else if (tag == "-convert-bspline-to-bezier") {
        g_scene->bspline_to_bezier();
      }

      /* flatten scene */
      else if (tag == "-flatten-group") {
        g_scene = SceneGraph::flatten(g_scene,SceneGraph::INSTANCING_GROUP);
      }

      /* flatten scene */
      else if (tag == "-flatten-geometry") {
        g_scene = SceneGraph::flatten(g_scene,SceneGraph::INSTANCING_GEOMETRY);
      }

      /* flatten scene */
      else if (tag == "-flatten") {
        g_scene = SceneGraph::flatten(g_scene,SceneGraph::INSTANCING_NONE);
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
        const FileName objectFile = cin->getFileName();
        Ref<SceneGraph::Node> object = SceneGraph::load(path + objectFile);
        if (referenceObjects) object->fileName = objectFile;
        Ref<Image> distribution = loadImage(path + cin->getFileName());
        const float minDistance = cin->getFloat();
        const size_t N = cin->getInt();
        Ref<Instantiator> instantiator = new Instantiator(g_height_field,object,distribution,minDistance,N);
        instantiator->instantiate(g_scene);
      }

      /* instantiate model a single time */
      else if (tag == "-instantiate") {
        const FileName objectFile = cin->getFileName();
        Ref<SceneGraph::Node> object = SceneGraph::load(path + objectFile);
        if (referenceObjects) object->fileName = objectFile;
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

      /* enable material referencing */
      else if (tag == "-reference-materials") {
        referenceMaterials = true;
      }

      /* enable object embedding */
      else if (tag == "-embed-objects") {
        referenceObjects = false;
      }

      /* enable object referencing */
      else if (tag == "-reference-objects") {
        referenceObjects = true;
      }

      else if (tag == "-centerScaleTranslate") {
        centerScale       = cin->getFloat();
        centerTranslate.x = cin->getFloat();
        centerTranslate.y = cin->getFloat();
        centerTranslate.z = cin->getFloat();
      }

      /* output filename */
      else if (tag == "-o") {
        SceneGraph::store(g_scene.dynamicCast<SceneGraph::Node>(),path + cin->getFileName(),embedTextures,referenceMaterials);
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
    g_device = rtcNewDevice(nullptr);
      
    /* create stream for parsing */
    Ref<ParseStream> stream = new ParseStream(new CommandLineStream(argc, argv));

    /* parse command line */  
    parseCommandLine(stream, FileName());

    rtcReleaseDevice(g_device); g_device = nullptr;
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
