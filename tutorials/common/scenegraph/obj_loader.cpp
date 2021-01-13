// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "obj_loader.h"
#include "texture.h"

namespace embree
{
  /*! Three-index vertex, indexing start at 0, -1 means invalid vertex. */
  struct Vertex {
    unsigned int v, vt, vn;
    Vertex() {};
    Vertex(unsigned int v) : v(v), vt(v), vn(v) {};
    Vertex(unsigned int v, unsigned int vt, unsigned int vn) : v(v), vt(vt), vn(vn) {};
  };

  struct Crease {
    float w;
    unsigned int a, b;
    Crease() : w(0), a(-1), b(-1) {};
    Crease(float w, unsigned int a, unsigned int b) : w(w), a(a), b(b) {};
  };

  static inline bool operator < ( const Vertex& a, const Vertex& b ) {
    if (a.v  != b.v)  return a.v  < b.v;
    if (a.vn != b.vn) return a.vn < b.vn;
    if (a.vt != b.vt) return a.vt < b.vt;
    return false;
  }

  /*! Fill space at the end of the token with 0s. */
  static inline const char* trimEnd(const char* token) 
  {
    if (*token == 0) return token;
    char* pe = (char*) token;
    while (*pe) pe++;
    pe--;
    while ((*pe == ' ' || *pe == '\t' || *pe == '\r') && pe >= token) *pe-- = 0;
    return token;
  }

  /*! Determine if character is a separator. */
  static inline bool isSep(const char c) {
    return (c == ' ') || (c == '\t');
  }

  /*! Parse separator. */
  static inline const char* parseSep(const char*& token) {
    size_t sep = strspn(token, " \t");
    if (!sep) THROW_RUNTIME_ERROR("separator expected");
    return token+=sep;
  }

  /*! Parse optional separator. */
  static inline const char* parseSepOpt(const char*& token) {
    return token+=strspn(token, " \t");
  }

  /*! Read float from a string. */
  static inline float getFloat(const char*& token) {
    token += strspn(token, " \t");
    float n = (float)atof(token);
    token += strcspn(token, " \t\r");
    return n;
  }

  /*! Read int from a string. */
  static inline int getInt(const char*& token) {
    token += strspn(token, " \t");
    int n = atoi(token);
    token += strcspn(token, " \t\r");
    return n;
  }

  /*! Read Vec2f from a string. */
  static inline Vec2f getVec2f(const char*& token) {
    float x = getFloat(token);
    float y = getFloat(token);
    return Vec2f(x,y);
  }

  /*! Read Vec3f from a string. */
  static inline Vec3f getVec3f(const char*& token) {
    float x = getFloat(token);
    token += strspn(token, " \t");
    if (*token == 0) return Vec3f(x);
    float y = getFloat(token);
    float z = getFloat(token);
    return Vec3f(x,y,z);
  }

  /*! Read Vec3fa from a string. */
  static inline Vec3fa getVec3fa(const char*& token) {
    float x = getFloat(token);
    token += strspn(token, " \t");
    if (*token == 0) return Vec3f(x);
    float y = getFloat(token);
    float z = getFloat(token);
    return Vec3fa(x,y,z);
  }

  class OBJLoader
  {
  public:

    /*! Constructor. */
    OBJLoader(const FileName& fileName, const bool subdivMode, const bool combineIntoSingleObject);
 
    /*! output model */
    Ref<SceneGraph::GroupNode> group;
  
  private:

    /*! file to load */
    FileName path;
  
    /*! load only quads and ignore triangles */
    bool subdivMode;

    /*! Geometry buffer. */
    avector<Vec3fa> v;
    avector<Vec3fa> vn;
    std::vector<Vec2f> vt;
    std::vector<Crease> ec;

    std::vector<std::vector<Vertex> > curGroup;
    std::vector<avector<Vec3ff> > curGroupHair;

    /*! Material handling. */
    std::string curMaterialName;
    Ref<SceneGraph::MaterialNode> curMaterial;
    std::map<std::string, Ref<SceneGraph::MaterialNode> > material;
    std::map<std::string, std::shared_ptr<Texture>> textureMap; 

  private:
    void loadMTL(const FileName& fileName);
    unsigned int fix_v (int index);
    unsigned int fix_vt(int index);
    unsigned int fix_vn(int index);
    void flushFaceGroup();
    void flushTriGroup();
    void flushHairGroup();
    Vertex getUInt3(const char*& token);
    uint32_t getVertex(std::map<Vertex,uint32_t>& vertexMap, Ref<SceneGraph::TriangleMeshNode> mesh, const Vertex& i);
    std::shared_ptr<Texture> loadTexture(const FileName& fname);
  };

  OBJLoader::OBJLoader(const FileName &fileName, const bool subdivMode, const bool combineIntoSingleObject) 
    : group(new SceneGraph::GroupNode), path(fileName.path()), subdivMode(subdivMode)
  {
    /* open file */
    std::ifstream cin;
    cin.open(fileName.c_str());
    if (!cin.is_open()) {
      THROW_RUNTIME_ERROR("cannot open " + fileName.str());
      return;
    }

    /* generate default material */
    Ref<SceneGraph::MaterialNode> defaultMaterial = new OBJMaterial("default");
    curMaterialName = "default";
    curMaterial = defaultMaterial;

    while (cin.peek() != -1)
    {
      /* load next multiline */
      std::string line; std::getline(cin,line);
      while (!line.empty() && line[line.size()-1] == '\\') {
	line[line.size()-1] = ' ';
	std::string next_line; std::getline(cin,next_line);
	if (next_line.empty()) break;
	line += next_line;
      }
      const char* token = trimEnd(line.c_str() + strspn(line.c_str(), " \t"));
      if (token[0] == 0) continue;

      /*! parse position */
      if (token[0] == 'v' && isSep(token[1])) { 
        v.push_back(getVec3f(token += 2)); continue;
      }

      /* parse normal */
      if (token[0] == 'v' && token[1] == 'n' && isSep(token[2])) { 
        vn.push_back(getVec3f(token += 3)); 
        continue; 
      }

      /* parse texcoord */
      if (token[0] == 'v' && token[1] == 't' && isSep(token[2])) { vt.push_back(getVec2f(token += 3)); continue; }

      /*! parse face */
      if (token[0] == 'f' && isSep(token[1]))
      {
        parseSep(token += 1);

        std::vector<Vertex> face;
        while (token[0]) {
	  Vertex vtx = getUInt3(token);
          face.push_back(vtx);
          parseSepOpt(token);
        }
        curGroup.push_back(face);
        continue;
      }

      /*! parse corona hair */
      if (!strncmp(token,"hair",4) && isSep(token[4]))
      {
        parseSep(token += 4);
        bool plane = !strncmp(token,"plane",5) && isSep(token[5]);
        if (plane) {
          parseSep(token += 5);
        }
        else if (!strncmp(token,"cylinder",8) && isSep(token[8])) {
          parseSep(token += 8);
        }
        else continue;

        unsigned int N = getInt(token);
        avector<Vec3ff> hair;
        for (unsigned int i=0; i<3*N+1; i++) {
          hair.push_back((Vec3ff)getVec3fa(token));
        }
        
        for (unsigned int i=0; i<N+1; i++)
        {
          float r = getFloat(token);
          MAYBE_UNUSED float t = (float)getInt(token);
          if (i != 0) hair[3*i-1].w = r;
          hair[3*i+0].w = r;
          if (i != N) hair[3*i+1].w = r;
        }
        curGroupHair.push_back(hair);
      }
      
      /*! parse edge crease */
      if (token[0] == 'e' && token[1] == 'c' && isSep(token[2]))
      {
	parseSep(token += 2);
	float w = getFloat(token);
	parseSepOpt(token);
	unsigned int a = fix_v(getInt(token));
	parseSepOpt(token);
	unsigned int b = fix_v(getInt(token));
	parseSepOpt(token);
	ec.push_back(Crease(w, a, b));
	continue;
      }

      /*! use material */
      if (!strncmp(token, "usemtl", 6) && isSep(token[6]))
      {
        if (!combineIntoSingleObject) flushFaceGroup();
        std::string name(parseSep(token += 6));
        if (material.find(name) == material.end()) {
          curMaterial = defaultMaterial;
          curMaterialName = "default";
        }
        else {
          curMaterial = material[name];
          curMaterialName = name;
        }
        continue;
      }

      /* load material library */
      if (!strncmp(token, "mtllib", 6) && isSep(token[6])) {
        loadMTL(path + std::string(parseSep(token += 6)));
        continue;
      }

      // ignore unknown stuff
    }
    flushFaceGroup();

    cin.close();
  }

  struct ExtObjMaterial
  {
  public:

    enum Type { NONE, MATTE, GLASS, METAL, METALLIC_PAINT };

    ExtObjMaterial ()
      : type(NONE), illum(0), d(1.f), Ns(1.f), Ni(1.f), Ka(0.f), Kd(1.f), Ks(0.f), Kt(1.0f),
        roughness(0.0f), coat_eta(1.0f), coat_roughness(0.0f), bump(0.0f), eta(1.4f), k(3.0f) {}

    Ref<SceneGraph::MaterialNode> select() const 
    {
      std::shared_ptr<Texture> nulltex;
      if (type == NONE) {
        return new OBJMaterial(d,map_d,Kd,map_Kd,Ks,map_Ks,Ns,map_Ns,map_Displ);
      } else if (type == MATTE) {
        if (coat_eta != 1.0f) return new MetallicPaintMaterial (Kd,zero,0.0f,eta.x);
        else                  return new OBJMaterial(1.0f,nulltex,Kd,map_Kd,Ks,nulltex,1.0f/(1E-6f+roughness),nulltex,nulltex);
      } else if (type == GLASS) {
        return new ThinDielectricMaterial(Vec3fa(1.0f),eta.x,0.1f);
      } else if (type == METAL) {
        if (roughness == 0.0f) {
          if (eta == Vec3fa(1.0f) && k == Vec3fa(0.0f)) return new MirrorMaterial(Kd);
          else                                          return new MetalMaterial(Kd,eta,k);
        }
        else return new MetalMaterial(Kd,eta,k,roughness);
      } else if (type == METALLIC_PAINT) {
        return new MetallicPaintMaterial (Kd,Ks,0.0f,coat_eta);
      }
      return new MatteMaterial(Vec3fa(0.5f));
    }

  public:
    Type type;

    int illum;             /*< illumination model */
    float d;               /*< dissolve factor, 1=opaque, 0=transparent */
    float Ns;              /*< specular exponent */
    float Ni;              /*< optical density for the surface (index of refraction) */

    Vec3fa Ka;              /*< ambient reflectivity */
    Vec3fa Kd;              /*< diffuse reflectivity */
    Vec3fa Ks;              /*< specular reflectivity */
    Vec3fa Kt;              /*< transmission filter */

    std::shared_ptr<Texture> map_d;            /*< d texture */
    std::shared_ptr<Texture> map_Kd;           /*< Kd texture */
    std::shared_ptr<Texture> map_Ks;           /*< Ks texture */
    std::shared_ptr<Texture> map_Ns;           /*< Ns texture */
    std::shared_ptr<Texture> map_Displ;        /*< Displ texture */

    float roughness;
    std::shared_ptr<Texture> roughnessMap;
    float coat_eta;
    float coat_roughness;
    std::shared_ptr<Texture> coat_roughnessMap;
    float bump;
    Vec3f eta;
    Vec3f k;
  };

  std::shared_ptr<Texture> OBJLoader::loadTexture(const FileName& fname)
  {
    if (textureMap.find(fname.str()) != textureMap.end())
      return textureMap[fname.str()];
    
    return std::shared_ptr<Texture>(Texture::load(path+fname));
  }

  /* load material file */
  void OBJLoader::loadMTL(const FileName &fileName)
  {
    std::ifstream cin;
    cin.open(fileName.c_str());
    if (!cin.is_open()) {
      std::cerr << "cannot open " << fileName.str() << std::endl;
      return;
    }

    std::string name;
    ExtObjMaterial cur;

    while (cin.peek() != -1)
    {
      /* load next multiline */
      std::string line; std::getline(cin,line);
      while (!line.empty() && line[line.size()-1] == '\\') {
	line[line.size()-1] = ' ';
	std::string next_line; std::getline(cin,next_line);
	if (next_line.empty()) break;
	line += next_line;
      }
      const char* token = trimEnd(line.c_str() + strspn(line.c_str(), " \t"));

      if (token[0] == 0  ) continue; // ignore empty lines
      if (token[0] == '#') continue; // ignore comments

      if (!strncmp(token, "newmtl", 6)) 
      {
        if (name != "") {
          material[name] = cur.select();
          material[name]->name = name;
        }
        
        parseSep(token+=6);
        name = token;
        cur = ExtObjMaterial();
        continue;
      }
      if (name == "") THROW_RUNTIME_ERROR("invalid material file: newmtl expected first");
      
      try 
      {
        if (!strncmp(token, "illum", 5)) { parseSep(token += 5);  continue; }
        
        if (!strncmp(token, "d",  1)) { parseSep(token += 1);  cur.d  = getFloat(token); continue; }
        if (!strncmp(token, "Ns", 2)) { parseSep(token += 2);  cur.Ns = getFloat(token); continue; }
        if (!strncmp(token, "Ni", 2)) { parseSep(token += 2);  cur.Ni = getFloat(token); continue; }
        
        if (!strncmp(token, "map_d", 5) || !strncmp(token, "d_map", 5)) {
          parseSep(token += 5);
          cur.map_d = loadTexture(FileName(token));
          continue;
        }
        if (!strncmp(token, "map_Ka", 6) || !strncmp(token, "Ka_map", 6)) { continue; }
        if (!strncmp(token, "map_Kd", 6) || !strncmp(token, "Kd_map", 6)) {
          parseSep(token += 6);
          cur.map_Kd = loadTexture(FileName(token));
          continue;
        }
        
        if (!strncmp(token, "map_Ks", 6) || !strncmp(token, "Ks_map", 6)) { continue; }
        if (!strncmp(token, "map_Tf", 6) || !strncmp(token, "Tf_map", 6)) { continue; }
        if (!strncmp(token, "map_Displ", 9) || !strncmp(token, "Displ_map", 9)) {
          parseSep(token += 9);
          cur.map_Displ = loadTexture(FileName(token));
          continue;
        }
        
        if (!strncmp(token, "Ka", 2)) { parseSep(token += 2);  cur.Ka = getVec3f(token); continue; }
        if (!strncmp(token, "Kd", 2)) { parseSep(token += 2);  cur.Kd = getVec3f(token); continue; }
        if (!strncmp(token, "Ks", 2)) { parseSep(token += 2);  cur.Ks = getVec3f(token); continue; }
        if (!strncmp(token, "Tf", 2)) { parseSep(token += 2);  cur.Kt = getVec3f(token); continue; }
        
        /* extended OBJ */
        if (!strncmp(token, "type", 4)) {
          parseSep(token += 4); std::string type = token;
          if      (type == "matte") cur.type = ExtObjMaterial::MATTE;
          else if (type == "glass") cur.type = ExtObjMaterial::GLASS;
          else if (type == "metal") cur.type = ExtObjMaterial::METAL;
          else if (type == "metallicPaint") cur.type = ExtObjMaterial::METALLIC_PAINT;
        }
        if (!strncmp(token, "roughnessMap",     12)) { parseSep(token += 12); cur.roughnessMap = loadTexture(FileName(token)); }
        if (!strncmp(token, "roughness",         9)) { parseSep(token +=  9); cur.roughness = getFloat(token); }
        if (!strncmp(token, "colorMap",          8)) { parseSep(token +=  8); cur.map_Kd  = loadTexture(FileName(token)); }
        if (!strncmp(token, "color",             5)) { parseSep(token +=  5); cur.Kd      = getVec3f(token); }
        if (!strncmp(token, "coat.color",       10)) { parseSep(token += 10); cur.Kd      = getVec3f(token); }
        if (!strncmp(token, "coat.eta",          8)) { parseSep(token +=  8); cur.coat_eta  = getFloat(token); }
        if (!strncmp(token, "coat.roughnessMap",17)) { parseSep(token += 17); cur.coat_roughnessMap   = loadTexture(FileName(token)); }
        if (!strncmp(token, "coat.roughness",   14)) { parseSep(token += 14); cur.coat_roughness = getFloat(token); }
        if (!strncmp(token, "bumpMap",           7)) { parseSep(token +=  7); cur.map_Displ = loadTexture(FileName(token)); }
        if (!strncmp(token, "bump",              4)) { parseSep(token +=  4); cur.bump   = getFloat(token); }
        if (!strncmp(token, "eta",               3)) { parseSep(token +=  3); cur.eta = getVec3f(token); }
        if (!strncmp(token, "k",                 1)) { parseSep(token +=  1); cur.k = getVec3f(token); }
      } 
      catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
      }
    }

    if (name != "") {
      material[name] = cur.select();
      material[name]->name = name;
    }

    cin.close();
  }

  /*! handles relative indices and starts indexing from 0 */
  unsigned int OBJLoader::fix_v (int index) { return (index > 0 ? index - 1 : (index == 0 ? 0 : (int) v .size() + index)); }
  unsigned int OBJLoader::fix_vt(int index) { return (index > 0 ? index - 1 : (index == 0 ? 0 : (int) vt.size() + index)); }
  unsigned int OBJLoader::fix_vn(int index) { return (index > 0 ? index - 1 : (index == 0 ? 0 : (int) vn.size() + index)); }

  /*! Parse differently formated triplets like: n0, n0/n1/n2, n0//n2, n0/n1.          */
  /*! All indices are converted to C-style (from 0). Missing entries are assigned -1. */
  Vertex OBJLoader::getUInt3(const char*& token)
  {
    Vertex v(-1);
    v.v = fix_v(atoi(token));
    token += strcspn(token, "/ \t\r");
    if (token[0] != '/') return(v);
    token++;

    // it is i//n
    if (token[0] == '/') {
      token++;
      v.vn = fix_vn(atoi(token));
      token += strcspn(token, " \t\r");
      return(v);
    }

    // it is i/t/n or i/t
    v.vt = fix_vt(atoi(token));
    token += strcspn(token, "/ \t\r");
    if (token[0] != '/') return(v);
    token++;

    // it is i/t/n
    v.vn = fix_vn(atoi(token));
    token += strcspn(token, " \t\r");
    return(v);
  }

  uint32_t OBJLoader::getVertex(std::map<Vertex,uint32_t>& vertexMap, Ref<SceneGraph::TriangleMeshNode> mesh, const Vertex& i)
  {
    const std::map<Vertex, uint32_t>::iterator& entry = vertexMap.find(i);
    if (entry != vertexMap.end()) return(entry->second);
    
    if (i.v >= v.size()) std::cout << "WARNING: corrupted OBJ file" << std::endl;
    else mesh->positions[0].push_back(v[i.v]);
      
    if (i.vn != -1) {
      while (mesh->normals[0].size() < mesh->positions[0].size()) mesh->normals[0].push_back(zero); // some vertices might not had a normal

      if (i.vn >= vn.size()) std::cout << "WARNING: corrupted OBJ file" << std::endl;
      else mesh->normals[0][mesh->positions[0].size()-1] = vn[i.vn];
    }
    if (i.vt != -1) {
      while (mesh->texcoords.size() < mesh->positions[0].size()) mesh->texcoords.push_back(zero); // some vertices might not had a texture coordinate

      if (i.vt >= vt.size()) std::cout << "WARNING: corrupted OBJ file" << std::endl;
      else mesh->texcoords[mesh->positions[0].size()-1] = vt[i.vt];
    }
    return (vertexMap[i] = (unsigned int)(mesh->positions[0].size()) - 1);
  }

  void OBJLoader::flushFaceGroup()
  {
    flushTriGroup();
    flushHairGroup();
  }

  /*! end current facegroup and append to mesh */
  void OBJLoader::flushTriGroup()
  {
    if (curGroup.empty()) return;

    if (subdivMode)
    {
      Ref<SceneGraph::SubdivMeshNode> mesh = new SceneGraph::SubdivMeshNode(curMaterial,BBox1f(0,1),1);
      mesh->normals.resize(1);
      group->add(mesh.cast<SceneGraph::Node>());

      for (size_t i=0; i<v.size();  i++) mesh->positions[0].push_back(v[i]);
      for (size_t i=0; i<vn.size(); i++) mesh->normals[0].push_back(vn[i]);
      for (size_t i=0; i<vt.size(); i++) mesh->texcoords.push_back(vt[i]);
      
      for (size_t i=0; i<ec.size(); ++i) {
        assert(((size_t)ec[i].a < v.size()) && ((size_t)ec[i].b < v.size()));
        mesh->edge_creases.push_back(Vec2i(ec[i].a, ec[i].b));
        mesh->edge_crease_weights.push_back(ec[i].w);
      }
      
      for (size_t j=0; j<curGroup.size(); j++)
      {
        const std::vector<Vertex>& face = curGroup[j];
        mesh->verticesPerFace.push_back(int(face.size()));
        for (size_t i=0; i<face.size(); i++)
          mesh->position_indices.push_back(face[i].v);
      }
      if (mesh->normals[0].size() == 0)
        mesh->normals.clear();
      mesh->verify();
    }
    else
    {
      Ref<SceneGraph::TriangleMeshNode> mesh = new SceneGraph::TriangleMeshNode(curMaterial,BBox1f(0,1),1);
      mesh->normals.resize(1);
      group->add(mesh.cast<SceneGraph::Node>());
      // merge three indices into one
      std::map<Vertex, uint32_t> vertexMap;
      for (size_t j=0; j<curGroup.size(); j++)
      {
        /* iterate over all faces */
        const std::vector<Vertex>& face = curGroup[j];
        
        /* triangulate the face with a triangle fan */
        Vertex i0 = face[0], i1 = Vertex(-1), i2 = face[1];
        for (size_t k=2; k < face.size(); k++) 
        {
          i1 = i2; i2 = face[k];
          uint32_t v0,v1,v2;
          v0 = getVertex(vertexMap, mesh, i0);
          v1 = getVertex(vertexMap, mesh, i1);
          v2 = getVertex(vertexMap, mesh, i2);
          assert(v0 < mesh->numVertices());
          assert(v1 < mesh->numVertices());
          assert(v2 < mesh->numVertices());
          mesh->triangles.push_back(SceneGraph::TriangleMeshNode::Triangle(v0,v1,v2));
        }
      }
      /* there may be vertices without normals or texture coordinates, thus we have to make these arrays the same size here */
      if (mesh->normals[0].size()) while (mesh->normals[0].size() < mesh->numVertices()) mesh->normals[0].push_back(zero);
      if (mesh->texcoords.size()) while (mesh->texcoords.size() < mesh->numVertices()) mesh->texcoords.push_back(zero);

      if (mesh->normals[0].size() == 0)
        mesh->normals.clear();
      mesh->verify();
    }
    
    curGroup.clear();
    ec.clear();
  }

   void OBJLoader::flushHairGroup()
   {
     if (curGroupHair.empty()) return;

     avector<Vec3ff> vertices;
     std::vector<SceneGraph::HairSetNode::Hair> curves;
     
     for (size_t i=0; i<curGroupHair.size(); i++) {
       for (size_t j=0; j<curGroupHair[i].size(); j++) {
         if (j%3 == 0) curves.push_back(SceneGraph::HairSetNode::Hair((unsigned int)vertices.size(),(unsigned int)i));
         vertices.push_back(curGroupHair[i][j]);
       }
     }
       
     Ref<SceneGraph::HairSetNode> mesh = new SceneGraph::HairSetNode(vertices,curves,curMaterial,RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE);
     group->add(mesh.cast<SceneGraph::Node>());
     mesh->verify();
     curGroupHair.clear();
   }
   
  Ref<SceneGraph::Node> loadOBJ(const FileName& fileName, const bool subdivMode, const bool combineIntoSingleObject) {
    OBJLoader loader(fileName,subdivMode,combineIntoSingleObject); 
    return loader.group.cast<SceneGraph::Node>();
  }
}

