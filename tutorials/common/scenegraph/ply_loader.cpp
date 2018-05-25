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

#include "ply_loader.h"
#include <list>

namespace embree
{
  namespace SceneGraph
  {
    /*! PLY type */
    struct Type {
      enum Tag { PTY_CHAR, PTY_UCHAR, PTY_SHORT, PTY_USHORT, PTY_INT, PTY_UINT, PTY_FLOAT, PTY_DOUBLE, PTY_LIST, PTY_NONE } ty, index, data;
      Type() : ty(PTY_NONE), index(PTY_NONE), data(PTY_NONE) {}
      Type(Tag ty) : ty(ty), index(PTY_NONE), data(PTY_NONE) {}
      Type(Tag ty, Tag index, Tag data) : ty(ty), index(index), data(data) {}
    };

    /*! an element stored in the PLY file, such as vertex, face, etc. */
    struct Element {
      std::string name;
      size_t size;                             /// number of data items of the element
      std::vector<std::string> properties;     /// list of all properties of the element (e.g. x, y, z) (not strictly necessary)
      std::map<std::string,Type> type;         /// mapping of property name to type
      std::map<std::string,std::vector<float> > data;                /// data array properties (all represented as floats)
      std::map<std::string,std::vector<std::vector<size_t> > > list; /// list properties (integer lists supported only)
    };

    /*! mesh structure that reflects the PLY file format */
    struct Mesh {
      std::vector<std::string> order;           /// order of all elements in file (not strictly necessary)
      std::map<std::string,Element> elements;   /// all elements of the file, e.g. vertex, face, ...
    };

    /* returns the size of a type in bytes */
    size_t sizeOfType(Type::Tag ty) 
    {
      switch (ty) {
      case Type::PTY_CHAR : return 1;
      case Type::PTY_UCHAR : return 1;
      case Type::PTY_SHORT : return 2;
      case Type::PTY_USHORT : return 2;
      case Type::PTY_INT : return 4;
      case Type::PTY_UINT : return 4;
      case Type::PTY_FLOAT : return 4;
      case Type::PTY_DOUBLE : return 8;
      default : throw std::runtime_error("invalid type");
      }
    }
    
    /* compute the type of a string */
    Type::Tag typeTagOfString(const std::string& ty) {
      if (ty == "char") return Type::PTY_CHAR;
      if (ty == "int8") return Type::PTY_CHAR;
      if (ty == "uchar") return Type::PTY_UCHAR;
      if (ty == "uint8") return Type::PTY_UCHAR;
      if (ty == "short") return Type::PTY_SHORT;
      if (ty == "int16") return Type::PTY_SHORT;
      if (ty == "ushort") return Type::PTY_USHORT;
      if (ty == "uint16") return Type::PTY_USHORT;
      if (ty == "int") return Type::PTY_INT;
      if (ty == "int32") return Type::PTY_INT;
      if (ty == "uint") return Type::PTY_UINT;
      if (ty == "uint32") return Type::PTY_UINT;
      if (ty == "float") return Type::PTY_FLOAT;
      if (ty == "float32") return Type::PTY_FLOAT;
      if (ty == "double") return Type::PTY_DOUBLE;
      throw std::runtime_error("invalid type " + ty);
      return Type::PTY_NONE;
    }
    
    /* compute the type of a string */
    std::string stringOfTypeTag(Type::Tag ty) {
      if (ty == Type::PTY_CHAR) return "char";
      if (ty == Type::PTY_UCHAR) return "uchar";
      if (ty == Type::PTY_SHORT) return "short";
      if (ty == Type::PTY_USHORT) return "ushort";
      if (ty == Type::PTY_INT) return "int";
      if (ty == Type::PTY_UINT) return "uint";
      if (ty == Type::PTY_FLOAT) return "float";
      if (ty == Type::PTY_DOUBLE) return "double";
      if (ty == Type::PTY_LIST) return "list";
      throw std::runtime_error("invalid type");
      return "";
    }
    
    /* compute the type of a string */
    std::string stringOfType(Type ty) {
      if (ty.ty == Type::PTY_LIST) return "list " + stringOfTypeTag(ty.index) + " " + stringOfTypeTag(ty.data);
      else return stringOfTypeTag(ty.ty);
    }

    /* PLY parser class */
    struct PlyParser
    {
      std::fstream fs;
      Mesh mesh;
      Ref<SceneGraph::Node> scene;

      /* storage format of data in file */
      enum Format { ASCII, BINARY_BIG_ENDIAN, BINARY_LITTLE_ENDIAN } format;

      /* constructor parses the input stream */
      PlyParser(const FileName& fileName) : format(ASCII)
      {
        /* open file */
        fs.open (fileName.c_str(), std::fstream::in | std::fstream::binary);
        if (!fs.is_open()) throw std::runtime_error("cannot open file : " + fileName.str());

        /* check for file signature */
        std::string signature; getline(fs,signature);
        if (signature != "ply") throw std::runtime_error("invalid PLY file signature: " + signature);
        
        /* read header */
        std::list<std::string> header;
        while (true) {
          std::string line; getline(fs,line);
          if (line == "end_header") break;
          if (line.find_first_of('#') == 0) continue;
          if (line == "") continue;
          header.push_back(line);
        }

        /* parse header */
        parseHeader(header);

        /* now parse all elements */
        for (std::vector<std::string>::iterator i = mesh.order.begin(); i!=mesh.order.end(); i++)
          parseElementData(mesh.elements[*i]);

        /* create triangle mesh */
        scene = import();
      }

      /* parse the PLY header */
      void parseHeader(std::list<std::string>& header) 
      {
        while (!header.empty())
        {          
          std::stringstream line(header.front()); 
          header.pop_front();
          
          std::string tag; line >> tag;
          
          /* ignore comments */
          if (tag == "comment") {
          }
          
          /* parse format */
          else if (tag == "format")  
          {
            std::string fmt; line >> fmt;
            if (fmt == "ascii") format = ASCII;
            else if (fmt == "binary_big_endian") format = BINARY_BIG_ENDIAN;
            else if (fmt == "binary_little_endian") format = BINARY_LITTLE_ENDIAN;
            else throw std::runtime_error("invalid PLY file format: " + fmt);
            std::string  version; line >> version;
            if (version != "1.0") throw std::runtime_error("invalid PLY file version: " + version);
          }

          /* parse end of header tag */
          else if (tag == "end_header")
            break;
          
          /* parse elements */
          else if (tag == "element") parseElementDescr(line,header);
      
          /* report unknown tags */
          else throw std::runtime_error("unknown tag in PLY file: " + tag);
        }
      }

      /* parses a PLY element description */
      void parseElementDescr(std::stringstream& cin, std::list<std::string>& header) 
      {
        Element elt;
        std::string name; cin >> name;
        size_t num; cin >> num;
        mesh.order.push_back(name);
        elt.name = name;
        elt.size = num;
  
        /* parse all properties */
        while (!header.empty())
        {
          std::stringstream line(header.front()); 
          std::string tag; line >> tag;
          if (tag != "property") break;
          header.pop_front();
          
          Type ty = parseType(line);
          std::string name; line >> name;
          elt.type[name] = ty; 
          elt.properties.push_back(name);
        }

        mesh.elements[name] = elt;
      }
      
      /* parses a PLY type */
      Type parseType(std::stringstream& cin) 
      {
        std::string ty; cin >> ty;
        if (ty == "list") {
          std::string ty0; cin >> ty0;
          std::string ty1; cin >> ty1;
          return Type(Type::PTY_LIST,typeTagOfString(ty0),typeTagOfString(ty1));
        } else return Type(typeTagOfString(ty));
      }

      /* parses data of a PLY element */
      void parseElementData(Element& elt) 
      {
        /* allocate data for all properties */
        for (std::vector<std::string>::iterator i=elt.properties.begin(); i!=elt.properties.end(); i++) {
          if (elt.type[*i].ty == Type::PTY_LIST) elt.list[*i] = std::vector<std::vector<size_t> >();
          else elt.data[*i] = std::vector<float>();
        }
        
        /* parse all elements */
        for (size_t e=0; e<elt.size; e++)
        {
          /* load all properties of the element */
          for (std::vector<std::string>::iterator i=elt.properties.begin(); i!=elt.properties.end(); i++) {
            Type ty = elt.type[*i];
            if (ty.ty == Type::PTY_LIST) loadPropertyList(elt.list[*i],ty.index,ty.data);
            else loadPropertyData(elt.data[*i],ty.ty);
          }
        }
      }

      /* load bytes from file and take care of little and big endian encoding */
      void readBytes(void* dst, int num) {
        if (format == BINARY_LITTLE_ENDIAN) fs.read((char*)dst,num);
        else if (format == BINARY_BIG_ENDIAN) for (int i=0; i<num; i++) fs.read((char*)dst+num-i-1,1);
        else throw std::runtime_error("internal error on PLY loader");
      }
      
      int            read_ascii_int  () { int i;   fs >> i; return i; }
      float          read_ascii_float() { float f; fs >> f; return f; }
      signed char    read_char  () { if (format == ASCII) return read_ascii_int();   signed char r = 0;    readBytes(&r,1); return r; }
      unsigned char  read_uchar () { if (format == ASCII) return read_ascii_int();   unsigned char r = 0;  readBytes(&r,1); return r; }
      signed short   read_short () { if (format == ASCII) return read_ascii_int();   signed short r = 0;   readBytes(&r,2); return r; }
      unsigned short read_ushort() { if (format == ASCII) return read_ascii_int();   unsigned short r = 0; readBytes(&r,2); return r; }
      signed int     read_int   () { if (format == ASCII) return read_ascii_int();   signed int r = 0;     readBytes(&r,4); return r; }
      unsigned int   read_uint  () { if (format == ASCII) return read_ascii_int();   unsigned int r = 0;   readBytes(&r,4); return r; }
      float          read_float () { if (format == ASCII) return read_ascii_float(); float r = 0;          readBytes(&r,4); return r; }
      double         read_double() { if (format == ASCII) return read_ascii_float(); double r = 0;         readBytes(&r,8); return r; }

      /* load an integer type */
      size_t loadInteger(Type::Tag ty)
      {
        switch (ty) {
        case Type::PTY_CHAR   : return read_char(); break;
        case Type::PTY_UCHAR  : return read_uchar(); break;
        case Type::PTY_SHORT  : return read_short(); break;
        case Type::PTY_USHORT : return read_ushort(); break;
        case Type::PTY_INT    : return read_int(); break;
        case Type::PTY_UINT   : return read_uint(); break;
        default : throw std::runtime_error("invalid type"); return 0;
        }
      }
        
      /* load a list */
      void loadPropertyList(std::vector<std::vector<size_t> >& vec,Type::Tag index_ty,Type::Tag data_ty)
      {
        std::vector<size_t> lst;
        size_t num = loadInteger(index_ty);
        for (size_t i=0; i<num; i++) lst.push_back(loadInteger(data_ty));
        vec.push_back(lst);
      }

      /* load a data element */
      void loadPropertyData(std::vector<float>& vec, Type::Tag ty) 
      {
        switch (ty) {
        case Type::PTY_CHAR   : vec.push_back(float(read_char()));   break;
        case Type::PTY_UCHAR  : vec.push_back(float(read_uchar()));  break;
        case Type::PTY_SHORT  : vec.push_back(float(read_short()));  break;
        case Type::PTY_USHORT : vec.push_back(float(read_ushort())); break;
        case Type::PTY_INT    : vec.push_back(float(read_int()));    break;
        case Type::PTY_UINT   : vec.push_back(float(read_uint()));   break;
        case Type::PTY_FLOAT  : vec.push_back(float(read_float()));  break;
        case Type::PTY_DOUBLE : vec.push_back(float(read_double())); break;
        default : throw std::runtime_error("invalid type");
        }
      }

      Ref<SceneGraph::Node> import()
      {
        Ref<SceneGraph::MaterialNode> material = new OBJMaterial;
        Ref<SceneGraph::TriangleMeshNode> mesh_o = new SceneGraph::TriangleMeshNode(material,BBox1f(0,1),1);
        
        /* convert all vertices */
        const Element& vertices = mesh.elements.at("vertex");
        const std::vector<float>& posx = vertices.data.at("x");
        const std::vector<float>& posy = vertices.data.at("y");
        const std::vector<float>& posz = vertices.data.at("z");
        
        mesh_o->positions[0].resize(vertices.size);
        for (size_t i=0; i<vertices.size; i++) {
          mesh_o->positions[0][i].x = posx[i];
          mesh_o->positions[0][i].y = posy[i];
          mesh_o->positions[0][i].z = posz[i];
        }

        /* convert all faces */
        const Element& faces = mesh.elements.at("face");
        const std::vector<std::vector<size_t> >& polygons = faces.list.at("vertex_indices");
        
        for (size_t j=0; j<polygons.size(); j++)
        {
          const std::vector<size_t>& face = polygons[j];
          if (face.size() < 3) continue;
          
          /* triangulate the face with a triangle fan */
          size_t i0 = face[0], i1 = 0, i2 = face[1];
          for (size_t k=2; k<face.size(); k++) {
            i1 = i2; i2 = face[k];
            mesh_o->triangles.push_back(SceneGraph::TriangleMeshNode::Triangle((unsigned int)i0, (unsigned int)i1, (unsigned int)i2));
          }
        }
        return mesh_o.dynamicCast<SceneGraph::Node>();
      }
    };
    
    Ref<Node> loadPLY(const FileName& fileName) {
      return PlyParser(fileName).scene;
    }
  }
}

