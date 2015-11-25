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

#include "xml_loader.h"
#include "xml_parser.h"
#include "obj_loader.h"

namespace embree
{
  struct Variant
  {
    ALIGNED_CLASS
  public:

    /*! Determines which kind of value is stored in the variant. */
    enum Type {
      EMPTY,     /*!< variant is empty                 */
      BOOL1,     /*!< variant stores bool value        */
      BOOL2,     /*!< variant stores bool2 value       */
      BOOL3,     /*!< variant stores bool3 value       */
      BOOL4,     /*!< variant stores bool4 value       */
      INT1,      /*!< variant stores int value         */
      INT2,      /*!< variant stores int2 value        */
      INT3,      /*!< variant stores int3 value        */
      INT4,      /*!< variant stores int4 value        */
      FLOAT1,    /*!< variant stores float value       */
      FLOAT2,    /*!< variant stores float2 value      */
      FLOAT3,    /*!< variant stores float3 value      */
      FLOAT4,    /*!< variant stores float4 value      */
      STRING,    /*!< variant stores string value      */
      TEXTURE,   /*!< variant stores texture value     */
    };

    /*! Constructs an empty variant object. */
    Variant ( ) : type(EMPTY) { }

    /*! Constructs a variant object holding a bool value. */
    Variant (bool b0                           ) : type(BOOL1) { b[0] = b0; }

    /*! Constructs a variant object holding a bool2 value. */
    Variant (bool b0, bool b1                  ) : type(BOOL2) { b[0] = b0; b[1] = b1; }

    /*! Constructs a variant object holding a bool3 value. */
    Variant (bool b0, bool b1, bool b2         ) : type(BOOL3) { b[0] = b0; b[1] = b1; b[2] = b2; }

    /*! Constructs a variant object holding a bool4 value. */
    Variant (bool b0, bool b1, bool b2, bool b3) : type(BOOL4) { b[0] = b0; b[1] = b1; b[2] = b2; b[3] = b3; }

    /*! Constructs a variant object holding an int value. */
    Variant (int i0) : type(INT1) { i[0] = i0; }

    /*! Constructs a variant object holding an int2 value. */
    Variant (Vec2i v) : type(INT2) { i[0] = v.x; i[1] = v.y; }

    /*! Constructs a variant object holding an int3 value. */
    Variant (Vec3i v) : type(INT3) { i[0] = v.x; i[1] = v.y; i[2] = v.z; }

    /*! Constructs a variant object holding an int4 value. */
    Variant (Vec4i v) : type(INT4) { i[0] = v.x; i[1] = v.y; i[2] = v.z; i[3] = v.w; }

    /*! Constructs a variant object holding a float value. */
    Variant (float f0) : type(FLOAT1) { f[0] = f0; }

    /*! Constructs a variant object holding a float2 value. */
    Variant (Vec2f v) : type(FLOAT2) { f[0] = v.x; f[1] = v.y; }

    /*! Constructs a variant object holding a float3 value. */
    Variant (Vec3f v) : type(FLOAT3) { f[0] = v.x; f[1] = v.y; f[2] = v.z; }

    /*! Constructs a variant object holding a float4 value. */
    Variant (Vec4f v) : type(FLOAT4) { f[0] = v.x; f[1] = v.y; f[2] = v.z; f[3] = v.w; }

    /*! Constructs a variant object holding a string value. */
    Variant (const char* str) : type(STRING), str(str) {}

    /*! Constructs a variant object holding a string value. */
    Variant (const std::string& str) : type(STRING), str(str) {}

    /*! Constructs a variant object holding a texture value. */
    Variant (const Texture* tex) : type(TEXTURE) { texture = tex; }

    /*! Extracts a boolean from the variant type. */
    bool  getBool () const { return b[0]; }

    /*! Extracts an integer from the variant type. */
    int   getInt  () const { return i[0]; }

    /*! Extracts a float from the variant type. */
    float getFloat() const { return f[0]; }

    /*! Extracts a Vec2f from the variant type. */
    Vec2f getVec2f() const { return Vec2f(f[0],f[1]); }

    /*! Extracts a Vec3f from the variant type. */
    Vec3f getVec3f() const { return Vec3f(f[0],f[1],f[2]); }

    /*! Extracts a Vec3fa from the variant type. */
    Vec3f getVec3fa() const { return Vec3fa(f[0],f[1],f[2]); }

    /*! Extracts a string from the variant type. */
    std::string getString() const { return str;   }

    /*! Extracts a texture from the variant type. */
    const Texture* getTexture() const { return texture;   }

    operator bool() const {
      return type != EMPTY;
    }

  public:
    Type type;            //!< Type of the data contained in the variant.
    union {
      bool b[4];          //!< Storage for single bool,bool2,bool3, and bool4 values.
      int i[4];           //!< Storage for single int,int2,int3, and int4 values.
      float f[12];         //!< Storage for single float,float2,float3, float4, and AffineSpace3f values.
      const Texture* texture;
    };
    std::string str;      //!< Storage for string values.
  };

  /*! Parameter container. Implements parameter container as a mapping
   *  from a string to variant values. This container is used to pass
   *  parameters for constructing objects from the API to the
   *  constructors of that objects. All the extraction functions
   *  return a default values in case the parameter is not found. */
  class Parms
  {
  public:

    /*! clears the parameter container */
    void clear() {
      m.clear();
    }

    /*! Extracts a named boolean out of the container. */
    bool getBool(const char* name, bool def = false) const {
      std::map<std::string,Variant>::const_iterator i = m.find(name);
      if (i == m.end() || (*i).second.type != Variant::BOOL1) return def;
      return (*i).second.getBool();
    }

    /*! Extracts a named integer out of the container. */
    int getInt(const char* name, int def = zero) const {
      std::map<std::string,Variant>::const_iterator i = m.find(name);
      if (i == m.end() || (*i).second.type != Variant::INT1) return def;
      return (*i).second.getInt();
    }

    /*! Extracts a named float out of the container. */
    float getFloat(const char* name, float def = zero) const {
      std::map<std::string,Variant>::const_iterator i = m.find(name);
      if (i == m.end() || (*i).second.type != Variant::FLOAT1) return def;
      return (*i).second.getFloat();
    }

    /*! Extracts a named Vec2f out of the container. */
    Vec2f getVec2f(const char* name, const Vec2f& def = zero) const {
      std::map<std::string,Variant>::const_iterator i = m.find(name);
      if (i == m.end() || (*i).second.type != Variant::FLOAT2) return def;
      return (*i).second.getVec2f();
    }

    /*! Extracts a named Vec3f out of the container. */
    Vec3f getVec3f(const char* name, const Vec3f& def = zero) const {
      std::map<std::string,Variant>::const_iterator i = m.find(name);
      if (i == m.end() || (*i).second.type != Variant::FLOAT3) return def;
      return (*i).second.getVec3f();
    }

    /*! Extracts a named Vec3f out of the container. */
    Vec3fa getVec3fa(const char* name, const Vec3fa& def = zero) const {
      std::map<std::string,Variant>::const_iterator i = m.find(name);
      if (i == m.end() || (*i).second.type != Variant::FLOAT3) return def;
      return (*i).second.getVec3fa();
    }

    /*! Extracts a named string out of the container. */
    std::string getString(const char* name, std::string def = "") const {
      std::map<std::string,Variant>::const_iterator i = m.find(name);
      if (i == m.end() || (*i).second.type != Variant::STRING) return def;
      return (*i).second.getString();
    }

    /*! Extracts a named texture out of the container. */
    const Texture* getTexture(const char* name) const {
      std::map<std::string,Variant>::const_iterator i = m.find(name);
      if (i == m.end() || (*i).second.type != Variant::TEXTURE) return nullptr;
      return (*i).second.getTexture();
    }

    /*! Adds a new named element to the container. */
    void add(const std::string& name, Variant data) {
      m[name] = data;
    }

  private:

    /*! Implementation of the container as an STL map. */
    std::map<std::string,Variant> m;
  };

  class XMLLoader
  {
  public:

    static Ref<SceneGraph::Node> load(const FileName& fileName, const AffineSpace3fa& space);
    XMLLoader(const FileName& fileName, const AffineSpace3fa& space);
   ~XMLLoader();

  public:
    Texture* loadTextureParm(const Ref<XML>& xml);
    Parms loadMaterialParms(const Ref<XML>& parms);
    Ref<SceneGraph::MaterialNode> addMaterial(const std::string& type, const Parms& parms);

  public:
    Ref<SceneGraph::Node> loadPointLight(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadSpotLight(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadDirectionalLight(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadDistantLight(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadAmbientLight(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadTriangleLight(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadQuadLight(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadHDRILight(const Ref<XML>& xml);
    
    Ref<SceneGraph::Node> loadTriangleMesh(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadQuadMesh(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadSubdivMesh(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadLineSegments(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadHairSet(const Ref<XML>& xml);

  private:
    Ref<SceneGraph::MaterialNode> loadMaterial(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadTransformNode(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadTransform2Node(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadAnimation2Node(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadGroupNode(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadNode(const Ref<XML>& xml);

  private:
    Ref<SceneGraph::MaterialNode> loadBGFMaterial(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadBGFMesh(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadBGFTransformNode(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadBGFGroupNode(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadBGFNode(const Ref<XML>& xml);

  private:
    template<typename T> T load(const Ref<XML>& xml) { assert(false); return T(zero); }
    template<typename T> T load(const Ref<XML>& xml, const T& opt) { assert(false); return T(zero); }
    char* loadBinary(const Ref<XML>& xml, size_t eltSize, size_t& size);

    std::vector<float> loadFloatArray(const Ref<XML>& xml);
    std::vector<Vec2f> loadVec2fArray(const Ref<XML>& xml);
    std::vector<Vec3f> loadVec3fArray(const Ref<XML>& xml);
    std::vector<Vec3fa> loadVec4fArray(const Ref<XML>& xml);
    std::vector<int>   loadIntArray(const Ref<XML>& xml);
    std::vector<Vec2i> loadVec2iArray(const Ref<XML>& xml);
    std::vector<Vec3i> loadVec3iArray(const Ref<XML>& xml);
    std::vector<Vec4i> loadVec4iArray(const Ref<XML>& xml);

  private:
    FileName path;         //!< path to XML file
    FILE* binFile;         //!< .bin file for reading binary data
    FileName binFileName;  //!< name of the .bin file

  private:
    std::map<std::string,Ref<SceneGraph::MaterialNode> > materialMap;     //!< named materials
    std::map<Ref<XML>, Ref<SceneGraph::MaterialNode> > materialCache;     //!< map for detecting repeated materials
    std::map<std::string,Ref<SceneGraph::Node> > sceneMap; 
    std::map<std::string,Texture* > textureMap; 

  private:
    size_t currentNodeID;
    std::map<size_t, Ref<SceneGraph::Node> > id2node;
    std::map<size_t, Ref<SceneGraph::MaterialNode> > id2material;

  public:
    Ref<SceneGraph::Node> root;
  };

  //////////////////////////////////////////////////////////////////////////////
  //// Loading standard types from an XML node
  //////////////////////////////////////////////////////////////////////////////

  template<> std::string XMLLoader::load<std::string>(const Ref<XML>& xml) {
    if (xml->body.size() < 1) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong string body");
    return xml->body[0].String();
  }

  template<> bool XMLLoader::load<bool>(const Ref<XML>& xml, const bool& opt) {
    if (xml == null) return opt;
    if (xml->body.size() != 1) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong bool body");
    return xml->body[0].Int() != 0;
  }

  template<> int XMLLoader::load<int>(const Ref<XML>& xml) {
    if (xml->body.size() != 1) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong int body");
    return xml->body[0].Int();
  }

  template<> Vec2i XMLLoader::load<Vec2i>(const Ref<XML>& xml) {
    if (xml->body.size() != 2) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong int2 body");
    return Vec2i(xml->body[0].Int(),xml->body[1].Int());
  }

  template<> Vec3i XMLLoader::load<Vec3i>(const Ref<XML>& xml) {
    if (xml->body.size() != 3) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong int3 body");
    return Vec3i(xml->body[0].Int(),xml->body[1].Int(),xml->body[2].Int());
  }

  template<> Vec4i XMLLoader::load<Vec4i>(const Ref<XML>& xml) {
    if (xml->body.size() != 4) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong int4 body");
    return Vec4i(xml->body[0].Int(),xml->body[1].Int(),xml->body[2].Int(),xml->body[3].Int());
  }

  template<> float XMLLoader::load<float>(const Ref<XML>& xml) {
    if (xml->body.size() != 1) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong float body");
    return xml->body[0].Float();
  }

  template<> float XMLLoader::load<float>(const Ref<XML>& xml, const float& opt) {
    if (xml == null) return opt;
    if (xml->body.size() != 1) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong float body");
    return xml->body[0].Float();
  }

  template<> Vec2f XMLLoader::load<Vec2f>(const Ref<XML>& xml) {
    if (xml->body.size() != 2) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong float2 body");
    return Vec2f(xml->body[0].Float(),xml->body[1].Float());
  }

  template<> Vec3f XMLLoader::load<Vec3f>(const Ref<XML>& xml) {
    if (xml->body.size() != 3) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong float3 body");
    return Vec3f(xml->body[0].Float(),xml->body[1].Float(),xml->body[2].Float());
  }

  template<> Vec3fa XMLLoader::load<Vec3fa>(const Ref<XML>& xml) {
    if (xml->body.size() != 3) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong float3 body");
    return Vec3fa(xml->body[0].Float(),xml->body[1].Float(),xml->body[2].Float());
  }

  template<> Vec3fa XMLLoader::load<Vec3fa>(const Ref<XML>& xml, const Vec3fa& opt) {
    if (xml == null) return opt;
    if (xml->body.size() != 3) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong float3 body");
    return Vec3fa(xml->body[0].Float(),xml->body[1].Float(),xml->body[2].Float());
  }

  template<> Vec4f XMLLoader::load<Vec4f>(const Ref<XML>& xml) {
    if (xml->body.size() != 4) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong float4 body");
    return Vec4f(xml->body[0].Float(),xml->body[1].Float(),xml->body[2].Float(),xml->body[3].Float());
  }

  template<> AffineSpace3fa XMLLoader::load<AffineSpace3fa>(const Ref<XML>& xml) 
  {
    if (xml->parm("translate") != "") {
      float x,y,z; sscanf(xml->parm("translate").c_str(),"%f %f %f",&x,&y,&z);
      return AffineSpace3fa::translate(Vec3f(x,y,z));
    } else if (xml->parm("scale") != "") {
      float x,y,z; sscanf(xml->parm("scale").c_str(),"%f %f %f",&x,&y,&z);
      return AffineSpace3fa::scale(Vec3f(x,y,z));
    } else if (xml->parm("rotate_x") != "") {
      float degrees; sscanf(xml->parm("rotate_x").c_str(),"%f",&degrees);
      return AffineSpace3fa::rotate(Vec3f(1,0,0),deg2rad(degrees));
    } else if (xml->parm("rotate_y") != "") {
      float degrees; sscanf(xml->parm("rotate_y").c_str(),"%f",&degrees);
      return AffineSpace3fa::rotate(Vec3f(0,1,0),deg2rad(degrees));
    } else if (xml->parm("rotate_z") != "") {
      float degrees; sscanf(xml->parm("rotate_z").c_str(),"%f",&degrees);
      return AffineSpace3fa::rotate(Vec3f(0,0,1),deg2rad(degrees));
    } else if (xml->parm("rotate") != "" && xml->parm("axis") != "") {
      float degrees; sscanf(xml->parm("rotate").c_str(),"%f",&degrees);
      float x,y,z; sscanf(xml->parm("axis").c_str(),"%f %f %f",&x,&y,&z);
      return AffineSpace3fa::rotate(Vec3f(x,y,z),deg2rad(degrees));
    } else {
      if (xml->body.size() != 12) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong AffineSpace body");
      return AffineSpace3fa(LinearSpace3fa(xml->body[0].Float(),xml->body[1].Float(),xml->body[ 2].Float(),
                                           xml->body[4].Float(),xml->body[5].Float(),xml->body[ 6].Float(),
                                           xml->body[8].Float(),xml->body[9].Float(),xml->body[10].Float()),
                            Vec3fa(xml->body[3].Float(),xml->body[7].Float(),xml->body[11].Float()));
    }
  }

  char* XMLLoader::loadBinary(const Ref<XML>& xml, size_t eltSize, size_t& size)
  {
    if (!binFile) 
      THROW_RUNTIME_ERROR("cannot open file "+binFileName.str()+" for reading");

    size_t ofs = atol(xml->parm("ofs").c_str());
    fseek(binFile,long(ofs),SEEK_SET);

    size = atol(xml->parm("size").c_str());
    if (size == 0) size = atol(xml->parm("num").c_str()); // version for BGF format
    char* data = (char*) alignedMalloc(size*eltSize);

    if (size != fread(data, eltSize, size, binFile)) 
      THROW_RUNTIME_ERROR("error reading from binary file: "+binFileName.str());

    return data;
  }

  std::vector<float> XMLLoader::loadFloatArray(const Ref<XML>& xml)
  {
    /*! do not fail of array does not exist */
    if (!xml) { return std::vector<float>(); }

    size_t size = 0;
    float* data = nullptr;
    if (xml->parm("ofs") != "") {
      data = (float*)loadBinary(xml,sizeof(float),size);
    } else {
      size_t elts = xml->body.size();
      size = elts;
      data = (float*) alignedMalloc(size*sizeof(float));
      for (size_t i=0; i<size; i++) 
        data[i] = xml->body[i].Float();
    }
    std::vector<float> res;
    for (size_t i=0; i<size; i++) res.push_back(data[i]);
    alignedFree(data);
    return res;
  }

  std::vector<Vec2f> XMLLoader::loadVec2fArray(const Ref<XML>& xml)
  {
    /*! do not fail of array does not exist */
    if (!xml) { return std::vector<Vec2f>(); }

    size_t size = 0;
    Vec2f* data = nullptr;
    if (xml->parm("ofs") != "") {
      data = (Vec2f*)loadBinary(xml,2*sizeof(float),size);
    } else {
      size_t elts = xml->body.size();
      if (elts % 2 != 0) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong vector<float2> body");
      size = elts/2;
      data = (Vec2f*) alignedMalloc(size*sizeof(Vec2f));
      for (size_t i=0; i<size; i++) 
        data[i] = Vec2f(xml->body[2*i+0].Float(),xml->body[2*i+1].Float());
    }
    std::vector<Vec2f> res;
    for (size_t i=0; i<size; i++) res.push_back(data[i]);
    alignedFree(data);
    return res;
  }

  std::vector<Vec3f> XMLLoader::loadVec3fArray(const Ref<XML>& xml)
  {
    /*! do not fail of array does not exist */
    if (!xml) { return std::vector<Vec3f>(); }

    size_t size = 0;
    Vec3f* data = nullptr;
    if (xml->parm("ofs") != "") {
      data = (Vec3f*) loadBinary(xml,3*sizeof(float),size);
    }
    else {
      size_t elts = xml->body.size();
      if (elts % 3 != 0) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong vector<float3> body");
      size = elts/3;
      data = (Vec3f*) alignedMalloc(size*sizeof(Vec3f));
      for (size_t i=0; i<size; i++) 
        data[i] = Vec3f(xml->body[3*i+0].Float(),xml->body[3*i+1].Float(),xml->body[3*i+2].Float());
    }
    std::vector<Vec3f> res;
    for (size_t i=0; i<size; i++) res.push_back(data[i]);
    alignedFree(data);
    return res;
  }

  std::vector<Vec3fa> XMLLoader::loadVec4fArray(const Ref<XML>& xml)
  {
    /*! do not fail of array does not exist */
    if (!xml) { return std::vector<Vec3fa>(); }

    size_t size = 0;
    Vec3fa* data = nullptr;
    if (xml->parm("ofs") != "") {
      data = (Vec3fa*) loadBinary(xml,4*sizeof(float),size);
    }
    else {
      size_t elts = xml->body.size();
      if (elts % 4 != 0) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong vector<float4> body");
      size = elts/4;
      data = (Vec3fa*) alignedMalloc(size*sizeof(Vec3fa));
      for (size_t i=0; i<size; i++) 
        data[i] = Vec3fa(xml->body[4*i+0].Float(),xml->body[4*i+1].Float(),xml->body[4*i+2].Float(),xml->body[4*i+3].Float());
    }
    std::vector<Vec3fa> res;
    for (size_t i=0; i<size; i++) res.push_back(data[i]);
    alignedFree(data);
    return res;
  }

  std::vector<int> XMLLoader::loadIntArray(const Ref<XML>& xml)
  {
    /*! do not fail of array does not exist */
    if (!xml) { return std::vector<int>(); }

    size_t size = 0;
    int* data = nullptr;
    if (xml->parm("ofs") != "") {
      data = (int*)loadBinary(xml,sizeof(int),size);
    } else {
      size_t elts = xml->body.size();
      size = elts;
      data = (int*) alignedMalloc(size*sizeof(int));
      for (size_t i=0; i<size; i++) 
        data[i] = xml->body[i].Int();
    }
    std::vector<int> res;
    for (size_t i=0; i<size; i++) res.push_back(data[i]);
    alignedFree(data);
    return res;
  }

  std::vector<Vec2i> XMLLoader::loadVec2iArray(const Ref<XML>& xml)
  {
    /*! do not fail of array does not exist */
    if (!xml) { return std::vector<Vec2i>(); }
    
    size_t size = 0;
    Vec2i* data = nullptr;
    if (xml->parm("ofs") != "") {
      data = (Vec2i*) loadBinary(xml,2*sizeof(int),size);
    }
    else {
      size_t elts = xml->body.size();
      if (elts % 2 != 0) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong vector<int2> body");
      size = elts/2;
      data = (Vec2i*) alignedMalloc(size*sizeof(Vec2i));
      for (size_t i=0; i<size; i++) 
        data[i] = Vec2i(xml->body[2*i+0].Int(),xml->body[2*i+1].Int());
    }
    std::vector<Vec2i> res;
    for (size_t i=0; i<size; i++) res.push_back(data[i]);
    alignedFree(data);
    return res;
  }

  std::vector<Vec3i> XMLLoader::loadVec3iArray(const Ref<XML>& xml)
  {
    /*! do not fail of array does not exist */
    if (!xml) { return std::vector<Vec3i>(); }
    
    size_t size = 0;
    Vec3i* data = nullptr;
    if (xml->parm("ofs") != "") {
      data = (Vec3i*) loadBinary(xml,3*sizeof(int),size);
    }
    else {
      size_t elts = xml->body.size();
      if (elts % 3 != 0) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong vector<int3> body");
      size = elts/3;
      data = (Vec3i*) alignedMalloc(size*sizeof(Vec3i));
      for (size_t i=0; i<size; i++) 
        data[i] = Vec3i(xml->body[3*i+0].Int(),xml->body[3*i+1].Int(),xml->body[3*i+2].Int());
    }
    std::vector<Vec3i> res;
    for (size_t i=0; i<size; i++) res.push_back(data[i]);
    alignedFree(data);
    return res;
  }

  std::vector<Vec4i> XMLLoader::loadVec4iArray(const Ref<XML>& xml)
  {
    /*! do not fail of array does not exist */
    if (!xml) { return std::vector<Vec4i>(); }
    
    size_t size = 0;
    Vec4i* data = nullptr;
    if (xml->parm("ofs") != "") {
      data = (Vec4i*) loadBinary(xml,4*sizeof(int),size);
    }
    else {
      size_t elts = xml->body.size();
      if (elts % 4 != 0) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong vector<int4> body");
      size = elts/4;
      data = (Vec4i*) alignedMalloc(size*sizeof(Vec4i));
      for (size_t i=0; i<size; i++) 
        data[i] = Vec4i(xml->body[4*i+0].Int(),xml->body[4*i+1].Int(),xml->body[4*i+2].Int(),xml->body[4*i+3].Int());
    }
    std::vector<Vec4i> res;
    for (size_t i=0; i<size; i++) res.push_back(data[i]);
    alignedFree(data);
    return res;
  }

  //////////////////////////////////////////////////////////////////////////////
  //// Loading of objects from XML file
  //////////////////////////////////////////////////////////////////////////////

  Ref<SceneGraph::Node> XMLLoader::loadPointLight(const Ref<XML>& xml) 
  {
    const AffineSpace3fa space = load<AffineSpace3fa>(xml->child("AffineSpace"));
    const Vec3fa I = load<Vec3f>(xml->child("I"));
    const Vec3fa P = Vec3fa(zero);
    return new SceneGraph::TransformNode(space, new SceneGraph::LightNode<PointLight>(PointLight(P,I)));
  }

  Ref<SceneGraph::Node> XMLLoader::loadSpotLight(const Ref<XML>& xml) 
  {
    const AffineSpace3fa space = load<AffineSpace3fa>(xml->child("AffineSpace"));
    const Vec3fa I = load<Vec3fa>(xml->child("I"));
    const Vec3fa P = Vec3fa(zero);
    const Vec3fa D = Vec3fa(0,0,1);
    const float angleMin = load<float>(xml->child("angleMin"));
    const float angleMax = load<float>(xml->child("angleMax"));
    return new SceneGraph::TransformNode(space, new SceneGraph::LightNode<SpotLight>(SpotLight(P,D,I,angleMin,angleMax)));
  }

  Ref<SceneGraph::Node> XMLLoader::loadDirectionalLight(const Ref<XML>& xml) 
  {
    const AffineSpace3fa space = load<AffineSpace3fa>(xml->child("AffineSpace"));
    const Vec3fa E = load<Vec3fa>(xml->child("E"));
    const Vec3fa D = Vec3fa(0,0,1);
    return new SceneGraph::TransformNode(space, new SceneGraph::LightNode<DirectionalLight>(DirectionalLight(D,E)));
  }

  Ref<SceneGraph::Node> XMLLoader::loadDistantLight(const Ref<XML>& xml) 
  {
    const AffineSpace3fa space = load<AffineSpace3fa>(xml->child("AffineSpace"));
    const Vec3fa L = load<Vec3fa>(xml->child("L"));
    const Vec3fa D = Vec3fa(0,0,1);
    const float halfAngle = load<float>(xml->child("halfAngle"));
    return new SceneGraph::TransformNode(space, new SceneGraph::LightNode<DistantLight>(DistantLight(D,L,halfAngle)));
  }

  Ref<SceneGraph::Node> XMLLoader::loadAmbientLight(const Ref<XML>& xml) 
  {
    const Vec3fa L = load<Vec3fa>(xml->child("L"));
    return new SceneGraph::LightNode<AmbientLight>(AmbientLight(L));
  }

  Ref<SceneGraph::Node> XMLLoader::loadTriangleLight(const Ref<XML>& xml) 
  {
    const AffineSpace3fa space = load<AffineSpace3fa>(xml->child("AffineSpace"));
    const Vec3fa L = load<Vec3fa>(xml->child("L"));
    const Vec3fa v0 = xfmPoint(space, Vec3fa(1, 0, 0));
    const Vec3fa v1 = xfmPoint(space, Vec3fa(0, 1, 0));
    const Vec3fa v2 = xfmPoint(space, Vec3fa(0, 0, 0));
    return new SceneGraph::LightNode<TriangleLight>(TriangleLight(v0,v1,v2,L));
  }

  Ref<SceneGraph::Node> XMLLoader::loadQuadLight(const Ref<XML>& xml) 
  {
    const AffineSpace3fa space = load<AffineSpace3fa>(xml->child("AffineSpace"));
    const Vec3fa L = load<Vec3fa>(xml->child("L"));
    const Vec3fa v0 = xfmPoint(space, Vec3fa(0, 0, 0));
    const Vec3fa v1 = xfmPoint(space, Vec3fa(0, 1, 0));
    const Vec3fa v2 = xfmPoint(space, Vec3fa(1, 1, 0));
    const Vec3fa v3 = xfmPoint(space, Vec3fa(1, 0, 0));
    return new SceneGraph::LightNode<QuadLight>(QuadLight(v0,v1,v2,v3,L));
  }

  Texture* XMLLoader::loadTextureParm(const Ref<XML>& xml)
  {
    const std::string id = xml->parm("id");
    if (id != "" && textureMap.find(id) != textureMap.end())
      return textureMap[id];

    Texture* texture = nullptr;
    const FileName src = xml->parm("src");

    /*! load texture from file */
    if (src.str() != "") {
      texture = Texture::load(path+src);
    }

    /*! load texture from binary file */
    else {
      const size_t width  = stoi(xml->parm("width"));
      const size_t height = stoi(xml->parm("height"));
      const Texture::Format format = Texture::string_to_format(xml->parm("format"));
      const size_t bytesPerTexel = Texture::getFormatBytesPerTexel(format);
      texture = new Texture(width,height,format);
      if (width*height != fread(texture->data, bytesPerTexel, width*height, binFile)) 
        THROW_RUNTIME_ERROR("error reading from binary file: "+binFileName.str());
    }
    
    if (id != "") textureMap[id] = texture;
    return texture;
  }

  Parms XMLLoader::loadMaterialParms(const Ref<XML>& parms)
  {
    Parms material;
    for (size_t i=0; i<parms->children.size(); i++) 
    {
      Ref<XML> entry = parms->children[i];
      std::string name = entry->parm("name");
      if      (entry->name == "int"    ) { material.add(name, load<int>  (entry)); }
      else if (entry->name == "int2"   ) { material.add(name, load<Vec2i>(entry)); }
      else if (entry->name == "int3"   ) { material.add(name, load<Vec3i>(entry)); }
      else if (entry->name == "int4"   ) { material.add(name, load<Vec4i>(entry)); }
      else if (entry->name == "float"  ) { material.add(name, load<float>(entry)); }
      else if (entry->name == "float2" ) { material.add(name, load<Vec2f>(entry)); }
      else if (entry->name == "float3" ) { material.add(name, load<Vec3f>(entry)); }
      else if (entry->name == "float4" ) { material.add(name, load<Vec4f>(entry)); }
      else if (entry->name == "texture3d") { material.add(name, loadTextureParm(entry)); }
      else if (entry->name == "param") {
        const std::string type = entry->parm("type");
        if      (type ==  "int"   ) { material.add(name, load<int>  (entry)); }
        else if (type == "int2"   ) { material.add(name, load<Vec2i>(entry)); }
        else if (type == "int3"   ) { material.add(name, load<Vec3i>(entry)); }
        else if (type == "int4"   ) { material.add(name, load<Vec4i>(entry)); }
        else if (type == "float"  ) { material.add(name, load<float>(entry)); }
        else if (type == "float2" ) { material.add(name, load<Vec2f>(entry)); }
        else if (type == "float3" ) { material.add(name, load<Vec3f>(entry)); }
        else if (type == "float4" ) { material.add(name, load<Vec4f>(entry)); }
        else THROW_RUNTIME_ERROR(entry->loc.str()+": invalid param type: "+type);
      }
    }
    return material;
  }

  Ref<SceneGraph::MaterialNode> XMLLoader::loadMaterial(const Ref<XML>& xml) 
  {
    const std::string id = xml->parm("id");
    if (id != "" && materialMap.find(id) != materialMap.end())
      return materialMap[id];

    Ref<XML> parameters = xml->child("parameters");
    if (materialCache.find(parameters) != materialCache.end()) {
      return materialMap[id] = materialCache[parameters];
    }

    std::string type = load<std::string>(xml->child("code")).c_str();
    Parms parms = loadMaterialParms(parameters);
    Ref<SceneGraph::MaterialNode> material = addMaterial(type,parms);
    materialCache[parameters] = material;
    return materialMap[id] = material;
  }

  Ref<SceneGraph::MaterialNode> XMLLoader::addMaterial(const std::string& type, const Parms& parms) 
  {
    Material material;
    if (type == "Matte")
    {
      const Vec3fa reflectance = parms.getVec3fa("reflectance",one);
      new (&material) MatteMaterial(reflectance);
    }
    else if (type == "Mirror")
    {
      const Vec3fa reflectance = parms.getVec3fa("reflectance",one);
      new (&material) MirrorMaterial(reflectance);
    }
    else if (type == "OBJ") 
    {
      const Texture* map_d = parms.getTexture("map_d");  
      const float d = parms.getFloat("d", 1.0f);
      const Texture* map_Kd = parms.getTexture("map_Kd");  
      const Vec3fa Kd = parms.getVec3fa("Kd", one);
      const Texture* map_Ks = parms.getTexture("map_Ks");  
      const Vec3fa Ks = parms.getVec3fa("Ks", zero);
      const Texture* map_Ns = parms.getTexture("map_Ns");  
      const float Ns = parms.getFloat("Ns", 10.0f);
      const Texture* map_Bump = parms.getTexture("map_Bump");
      new (&material) OBJMaterial(d,map_d,Kd,map_Kd,Ks,map_Ks,Ns,map_Ns,map_Bump);
    }
    else if (type == "OBJMaterial")  // for BGF file format
    {
      //map_d = parms.getTexture("map_d");  
      const float d = parms.getFloat("d", 1.0f);
      //map_Kd = parms.getTexture("map_kd");  
      const Vec3fa Kd = parms.getVec3fa("kd", one);
      //map_Ks = parms.getTexture("map_ks");  
      const Vec3fa Ks = parms.getVec3fa("ks", zero);
      //map_Ns = parms.getTexture("map_ns");  
      const float Ns = parms.getFloat("ns", 10.0f);
      //map_Bump = parms.getTexture("map_Bump");
      new (&material) OBJMaterial(d,Kd,Ks,Ns);
    }
    else if (type == "ThinDielectric" || type == "ThinGlass")
    {
      const Vec3fa transmission = parms.getVec3fa("transmission",one);
      const float eta          = parms.getFloat("eta",1.4f);
      const float thickness    = parms.getFloat("thickness",0.1f);
      new (&material) ThinDielectricMaterial(transmission,eta,thickness);
    }
    else if (type == "Plastic")
    {
      const Vec3fa pigmentColor = parms.getVec3fa("pigmentColor",one);
      const float eta          = parms.getFloat("eta",1.4f);
      const float roughness    = parms.getFloat("roughness",0.01f);
      new (&material) MetallicPaintMaterial(pigmentColor,pigmentColor,roughness,eta);
    }
    else if (type == "Metal")
    {
      const Vec3fa reflectance  = parms.getVec3fa("reflectance",one);
      const Vec3fa eta          = parms.getVec3fa("eta",Vec3fa(1.4f));
      const Vec3fa k            = parms.getVec3fa("k",Vec3fa(0.0f));
      const float roughness     = parms.getFloat("roughness",0.01f);
      if (roughness == 0.0f)
        new (&material) MetalMaterial(reflectance,eta,k);
      else 
        new (&material) MetalMaterial(reflectance,eta,k,roughness);
    }
    else if (type == "Velvet")
    {
      const Vec3fa reflectance = parms.getVec3fa("reflectance",one);
      const float backScattering = parms.getFloat("backScattering",zero);
      const Vec3fa horizonScatteringColor = parms.getVec3fa("horizonScatteringColor",one);
      const float horizonScatteringFallOff = parms.getFloat("horizonScatteringFallOff",zero);
      new (&material) VelvetMaterial(reflectance,backScattering,horizonScatteringColor,horizonScatteringFallOff);
    }
    else if (type == "Dielectric")
    {
      const Vec3fa transmissionOutside = parms.getVec3fa("transmissionOutside",one);
      const Vec3fa transmissionInside  = parms.getVec3fa("transmission",one);
      const float etaOutside = parms.getFloat("etaOutside",1.0f);
      const float etaInside  = parms.getFloat("etaInside",1.4f);
      new (&material) DielectricMaterial(transmissionOutside,transmissionInside,etaOutside,etaInside);
    }
    else if (type == "MetallicPaint")
    {
      const Vec3fa shadeColor    = parms.getVec3fa("shadeColor",one);
      const Vec3fa glitterColor  = parms.getVec3fa("glitterColor",zero);
      const float glitterSpread = parms.getFloat("glitterSpread",1.0f);
      const float eta           = parms.getFloat("eta",1.4f);
      new (&material) MetallicPaintMaterial(shadeColor,glitterColor,glitterSpread,eta);
    }
    else if (type == "Hair")
    {
      const Vec3fa Kr = parms.getVec3fa("Kr",one);
      const Vec3fa Kt = parms.getVec3fa("Kt",zero);
      const float nx = parms.getFloat("nx",20.0f);
      const float ny = parms.getFloat("ny",2.0f);
      new (&material) HairMaterial(Kr,Kt,nx,ny);
    }
    else {
      std::cout << "Warning: unsupported material " << type << std::endl;
      new (&material) OBJMaterial(1.0f,Vec3fa(0.5f),Vec3fa(0.0f),0.0f);
    }
    return new SceneGraph::MaterialNode(material);
  }

  Ref<SceneGraph::Node> XMLLoader::loadTriangleMesh(const Ref<XML>& xml) 
  {
    Ref<SceneGraph::MaterialNode> material = loadMaterial(xml->child("material"));
    std::vector<Vec3f> positions = loadVec3fArray(xml->childOpt("positions"));
    std::vector<Vec3f> positions2= loadVec3fArray(xml->childOpt("positions2"));
    std::vector<Vec3f> normals   = loadVec3fArray(xml->childOpt("normals"  ));
    std::vector<Vec2f> texcoords = loadVec2fArray(xml->childOpt("texcoords"));
    std::vector<Vec3i> triangles = loadVec3iArray(xml->childOpt("triangles"));

    SceneGraph::TriangleMeshNode* mesh = new SceneGraph::TriangleMeshNode(material);
    for (size_t i=0; i<positions.size(); i++) mesh->v.push_back(positions[i]);
    for (size_t i=0; i<positions2.size();i++) mesh->v2.push_back(positions2[i]);
    for (size_t i=0; i<normals.size();   i++) mesh->vn.push_back(normals[i]);
    for (size_t i=0; i<texcoords.size(); i++) mesh->vt.push_back(texcoords[i]);
    for (size_t i=0; i<triangles.size(); i++) mesh->triangles.push_back(SceneGraph::TriangleMeshNode::Triangle(triangles[i].x,triangles[i].y,triangles[i].z));
    mesh->verify();
    return mesh;
  }

  Ref<SceneGraph::Node> XMLLoader::loadQuadMesh(const Ref<XML>& xml) 
  {
    Ref<SceneGraph::MaterialNode> material = loadMaterial(xml->child("material"));
    std::vector<Vec3f> positions = loadVec3fArray(xml->childOpt("positions"));
    std::vector<Vec3f> positions2= loadVec3fArray(xml->childOpt("positions2"));
    std::vector<Vec3f> normals   = loadVec3fArray(xml->childOpt("normals"  ));
    std::vector<Vec2f> texcoords = loadVec2fArray(xml->childOpt("texcoords"));
    std::vector<Vec4i> indices   = loadVec4iArray(xml->childOpt("indices"));

    SceneGraph::QuadMeshNode* mesh = new SceneGraph::QuadMeshNode(material);
    for (size_t i=0; i<positions.size(); i++) mesh->v.push_back(positions[i]);
    for (size_t i=0; i<positions2.size();i++) mesh->v2.push_back(positions2[i]);
    for (size_t i=0; i<normals.size();   i++) mesh->vn.push_back(normals[i]);
    for (size_t i=0; i<texcoords.size(); i++) mesh->vt.push_back(texcoords[i]);
    for (size_t i=0; i<indices.size();   i++) mesh->quads.push_back(SceneGraph::QuadMeshNode::Quad(indices[i].x,indices[i].y,indices[i].z,indices[i].w));
    mesh->verify();
    return mesh;
  }

  Ref<SceneGraph::Node> XMLLoader::loadSubdivMesh(const Ref<XML>& xml) 
  {
    Ref<SceneGraph::MaterialNode> material = loadMaterial(xml->child("material"));

    SceneGraph::SubdivMeshNode* mesh = new SceneGraph::SubdivMeshNode(material);
    std::vector<Vec3f> positions = loadVec3fArray(xml->childOpt("positions"));
    for (size_t i=0; i<positions.size(); i++) mesh->positions.push_back(positions[i]);
    std::vector<Vec3f> positions2 = loadVec3fArray(xml->childOpt("positions2"));
    for (size_t i=0; i<positions2.size(); i++) mesh->positions2.push_back(positions2[i]);
    std::vector<Vec3f> normals = loadVec3fArray(xml->childOpt("normals"));
    for (size_t i=0; i<normals.size(); i++) mesh->normals.push_back(normals[i]);
    mesh->texcoords = loadVec2fArray(xml->childOpt("texcoords"));
    mesh->position_indices = loadIntArray(xml->childOpt("position_indices"));
    mesh->normal_indices   = loadIntArray(xml->childOpt("normal_indices"));
    mesh->texcoord_indices = loadIntArray(xml->childOpt("texcoord_indices"));
    mesh->verticesPerFace  = loadIntArray(xml->childOpt("faces"));
    mesh->holes            = loadIntArray(xml->childOpt("holes"));
    mesh->edge_creases     = loadVec2iArray(xml->childOpt("edge_creases"));
    mesh->edge_crease_weights = loadFloatArray(xml->childOpt("edge_crease_weights"));
    mesh->vertex_creases      = loadIntArray(xml->childOpt("vertex_creases"));
    mesh->vertex_crease_weights = loadFloatArray(xml->childOpt("vertex_crease_weights"));
    mesh->verify();
    return mesh;
  }

  Ref<SceneGraph::Node> XMLLoader::loadLineSegments(const Ref<XML>& xml) 
  {
    Ref<SceneGraph::MaterialNode> material = loadMaterial(xml->child("material"));
    std::vector<Vec3fa> positions  = loadVec4fArray(xml->childOpt("positions"));
    std::vector<Vec3fa> positions2 = loadVec4fArray(xml->childOpt("positions2"));
    std::vector<int>    indices    = loadIntArray(xml->childOpt("indices"));

    SceneGraph::LineSegmentsNode* mesh = new SceneGraph::LineSegmentsNode(material);
    mesh->v .resize(positions .size()); for (size_t i=0; i<positions .size(); i++) mesh->v [i] = positions [i];
    mesh->v2.resize(positions2.size()); for (size_t i=0; i<positions2.size(); i++) mesh->v2[i] = positions2[i];
    mesh->indices.resize(indices.size()); for (size_t i=0; i<indices.size(); i++) mesh->indices[i] = indices[i];
    mesh->verify();
    return mesh;
  }

  Ref<SceneGraph::Node> XMLLoader::loadHairSet(const Ref<XML>& xml) 
  {
    Ref<SceneGraph::MaterialNode> material = loadMaterial(xml->child("material"));
    std::vector<Vec3fa> positions  = loadVec4fArray(xml->childOpt("positions"));
    std::vector<Vec3fa> positions2 = loadVec4fArray(xml->childOpt("positions2"));
    std::vector<Vec2i> indices     = loadVec2iArray(xml->childOpt("indices"));

    SceneGraph::HairSetNode* hair = new SceneGraph::HairSetNode(material);
    hair->v .resize(positions .size()); for (size_t i=0; i<positions .size(); i++) hair->v [i] = positions [i];
    hair->v2.resize(positions2.size()); for (size_t i=0; i<positions2.size(); i++) hair->v2[i] = positions2[i];
    hair->hairs.resize(indices.size()); for (size_t i=0; i<indices.size(); i++) hair->hairs[i] = SceneGraph::HairSetNode::Hair(indices[i].x,indices[i].y);
    hair->verify();
    return hair;
  }

  Ref<SceneGraph::Node> XMLLoader::loadTransformNode(const Ref<XML>& xml) 
  {
    AffineSpace3fa space = load<AffineSpace3fa>(xml->children[0]);

    Ref<SceneGraph::GroupNode> group = new SceneGraph::GroupNode;
    for (size_t i=1; i<xml->children.size(); i++)
      group->add(loadNode(xml->children[i]));

    return new SceneGraph::TransformNode(space,group.cast<SceneGraph::Node>());
  }

  Ref<SceneGraph::Node> XMLLoader::loadTransform2Node(const Ref<XML>& xml) 
  {
    AffineSpace3fa space0 = load<AffineSpace3fa>(xml->children[0]);
    AffineSpace3fa space1 = load<AffineSpace3fa>(xml->children[1]);

    Ref<SceneGraph::GroupNode> group = new SceneGraph::GroupNode;
    for (size_t i=2; i<xml->children.size(); i++) 
      group->add(loadNode(xml->children[i]));

    return new SceneGraph::TransformNode(space0,space1,group.cast<SceneGraph::Node>());
  }

  Ref<SceneGraph::Node> XMLLoader::loadAnimation2Node(const Ref<XML>& xml) 
  {
    if (xml->children.size() != 2) THROW_RUNTIME_ERROR("invalid Animation2 node");
    Ref<SceneGraph::Node> node0 = loadNode(xml->children[0]);
    Ref<SceneGraph::Node> node1 = loadNode(xml->children[1]);
    SceneGraph::set_motion_blur(node0,node1);
    return node0;
  }

  Ref<SceneGraph::Node> XMLLoader::loadGroupNode(const Ref<XML>& xml) 
  {
    Ref<SceneGraph::GroupNode> group = new SceneGraph::GroupNode;
    for (size_t i=0; i<xml->children.size(); i++)
      group->add(loadNode(xml->children[i]));
    return group.cast<SceneGraph::Node>();
  }
  
  Ref<SceneGraph::Node> XMLLoader::loadNode(const Ref<XML>& xml)
  {
    if (xml->name == "assign") 
    {
      if (xml->parm("type") == "material") 
      {
        Ref<SceneGraph::MaterialNode> material = loadMaterial(xml->child(0));
        materialMap[xml->parm("id")] = material;
        return nullptr;
      }
      else if (xml->parm("type") == "scene")
      {
        sceneMap[xml->parm("id")] = loadNode(xml->child(0));
        return nullptr;
      }
      else 
        THROW_RUNTIME_ERROR(xml->loc.str()+": unknown type: "+xml->parm("type"));
    }
    else 
    {
      const std::string id = xml->parm("id");
      if      (xml->name == "extern"          ) return sceneMap[id] = SceneGraph::load(path + xml->parm("src"));
      else if (xml->name == "obj"             ) {
        const bool subdiv_mode = xml->parm("subdiv") == "1";
        return sceneMap[id] = loadOBJ(path + xml->parm("src"),subdiv_mode);
      }
      else if (xml->name == "ref"             ) return sceneMap[id] = sceneMap[xml->parm("id")];
      else if (xml->name == "PointLight"      ) return sceneMap[id] = loadPointLight      (xml);
      else if (xml->name == "SpotLight"       ) return sceneMap[id] = loadSpotLight       (xml);
      else if (xml->name == "DirectionalLight") return sceneMap[id] = loadDirectionalLight(xml);
      else if (xml->name == "DistantLight"    ) return sceneMap[id] = loadDistantLight    (xml);
      else if (xml->name == "AmbientLight"    ) return sceneMap[id] = loadAmbientLight    (xml);
      else if (xml->name == "TriangleLight"   ) return sceneMap[id] = loadTriangleLight   (xml);
      else if (xml->name == "QuadLight"       ) return sceneMap[id] = loadQuadLight       (xml);
      else if (xml->name == "TriangleMesh"    ) return sceneMap[id] = loadTriangleMesh    (xml);
      else if (xml->name == "QuadMesh"        ) return sceneMap[id] = loadQuadMesh        (xml);
      else if (xml->name == "SubdivisionMesh" ) return sceneMap[id] = loadSubdivMesh      (xml);
      else if (xml->name == "LineSegments"    ) return sceneMap[id] = loadLineSegments    (xml);
      else if (xml->name == "Hair"            ) return sceneMap[id] = loadHairSet         (xml);
      else if (xml->name == "Group"           ) return sceneMap[id] = loadGroupNode       (xml);
      else if (xml->name == "Transform"       ) return sceneMap[id] = loadTransformNode   (xml);
      else if (xml->name == "Transform2"      ) return sceneMap[id] = loadTransform2Node  (xml);
      else if (xml->name == "Animation2"      ) return sceneMap[id] = loadAnimation2Node  (xml);

      else THROW_RUNTIME_ERROR(xml->loc.str()+": unknown tag: "+xml->name);
    }

    return nullptr;
  }

  /*******************************************************************************************/

  Ref<SceneGraph::MaterialNode> XMLLoader::loadBGFMaterial(const Ref<XML>& xml) 
  {
    std::string type = xml->parm("type");
    std::string name = xml->parm("name");
    Parms parms = loadMaterialParms(xml);
    return addMaterial(type,parms);
  }

  Ref<SceneGraph::Node> XMLLoader::loadBGFMesh(const Ref<XML>& xml) 
  {
    size_t matid = xml->child("materiallist")->body[0].Int();
    Ref<SceneGraph::MaterialNode> material = id2material.at(matid);
    std::vector<Vec3f> positions = loadVec3fArray(xml->childOpt("vertex"));
    std::vector<Vec3f> normals   = loadVec3fArray(xml->childOpt("normal"));
    std::vector<Vec2f> texcoords = loadVec2fArray(xml->childOpt("texcoord"));
    std::vector<Vec4i> triangles = loadVec4iArray(xml->childOpt("prim"));

    SceneGraph::TriangleMeshNode* mesh = new SceneGraph::TriangleMeshNode(material);
    for (size_t i=0; i<positions.size(); i++) mesh->v.push_back(positions[i]);
    for (size_t i=0; i<normals.size();   i++) mesh->vn.push_back(normals[i]);
    for (size_t i=0; i<texcoords.size(); i++) mesh->vt.push_back(texcoords[i]);
    for (size_t i=0; i<triangles.size(); i++) mesh->triangles.push_back(SceneGraph::TriangleMeshNode::Triangle(triangles[i].x,triangles[i].y,triangles[i].z));
    return mesh;
  }

  Ref<SceneGraph::Node> XMLLoader::loadBGFTransformNode(const Ref<XML>& xml) 
  {
    const size_t child = atoi(xml->parm("child").c_str()); 
    if (xml->body.size() != 12) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong AffineSpace body");
    const AffineSpace3fa space(LinearSpace3fa(Vec3fa(xml->body[0].Float(),xml->body[1].Float(),xml->body[2].Float()),
                                              Vec3fa(xml->body[3].Float(),xml->body[4].Float(),xml->body[5].Float()),
                                              Vec3fa(xml->body[6].Float(),xml->body[7].Float(),xml->body[8].Float())),
                               Vec3fa(xml->body[9].Float(),xml->body[10].Float(),xml->body[11].Float()));
    return new SceneGraph::TransformNode(space,id2node.at(child));
  }

  Ref<SceneGraph::Node> XMLLoader::loadBGFGroupNode(const Ref<XML>& xml) 
  {
    const size_t N  = atoi(xml->parm("numChildren").c_str());
    if (xml->body.size() != N) 
      THROW_RUNTIME_ERROR(xml->loc.str()+": invalid group node");

    Ref<SceneGraph::GroupNode> group = new SceneGraph::GroupNode(N);
    for (size_t i=0; i<N; i++) 
    {
      const size_t id = xml->body[i].Int();
      group->set(i,id2node.at(id));
    }
    return group.cast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> XMLLoader::loadBGFNode(const Ref<XML>& xml)
  {
    //const size_t id = atoi(xml->parm("id").c_str());
    const size_t id = currentNodeID++;

    if      (xml->name == "Mesh"     ) return id2node[id] = loadBGFMesh(xml);
    else if (xml->name == "Group"    ) return id2node[id] = loadBGFGroupNode(xml);
    else if (xml->name == "Transform") return id2node[id] = loadBGFTransformNode(xml);
    else if (xml->name == "Material" ) 
    {
      Ref<SceneGraph::MaterialNode> material = loadBGFMaterial(xml); 
      id2material[id] = material;
      return material.cast<SceneGraph::Node>();
    }
    else if (xml->name == "Texture2D") {
      // textures not implemented yet
      return new SceneGraph::GroupNode();
    }
    else THROW_RUNTIME_ERROR(xml->loc.str()+": unknown tag: "+xml->name);
  }

  /*******************************************************************************************/
  

  Ref<SceneGraph::Node> XMLLoader::load(const FileName& fileName, const AffineSpace3fa& space) {
    XMLLoader loader(fileName,space); return loader.root;
  }

  XMLLoader::XMLLoader(const FileName& fileName, const AffineSpace3fa& space) : binFile(nullptr), currentNodeID(0)
  {
    path = fileName.path();
    binFileName = fileName.setExt(".bin");
    binFile = fopen(binFileName.c_str(),"rb");
    if (!binFile) {
      binFileName = fileName.addExt(".bin");
      binFile = fopen(binFileName.c_str(),"rb");
    }

    Ref<XML> xml = parseXML(fileName);
    if (xml->name == "scene") 
    {
      Ref<SceneGraph::GroupNode> group = new SceneGraph::GroupNode;
      for (size_t i=0; i<xml->children.size(); i++) { 
        group->add(loadNode(xml->children[i]));
      }
      root = group.cast<SceneGraph::Node>();
    }
    else if (xml->name == "BGFscene") 
    {
      Ref<SceneGraph::Node> last = nullptr;
      for (size_t i=0; i<xml->children.size(); i++) { 
        root = loadBGFNode(xml->children[i]);
      }
    }
    else 
      THROW_RUNTIME_ERROR(xml->loc.str()+": invalid scene tag");

    if (space == AffineSpace3fa(one)) 
      return;
    
    root = new SceneGraph::TransformNode(space,root);
  }

  XMLLoader::~XMLLoader() {
    if (binFile) fclose(binFile);
  }

  /*! read from disk */
  Ref<SceneGraph::Node> loadXML(const FileName& fileName, const AffineSpace3fa& space) {
    return XMLLoader::load(fileName,space);
  }
}
