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

#pragma once

#include "materials.h"
#include "lights.h"
#include "../../../include/embree2/rtcore.h"

namespace embree
{  
  namespace SceneGraph
  {
    struct Node;
    struct MaterialNode;

    Ref<Node> load(const FileName& fname);
    void store(Ref<Node> root, const FileName& fname, bool embedTextures);
    void set_motion_blur(Ref<Node> node0, Ref<Node> node1);
    void set_motion_vector(Ref<Node> node, const Vec3fa& dP);
    void resize_randomly(Ref<Node> node, const size_t N);
    Ref<Node> convert_triangles_to_quads(Ref<Node> node);
    Ref<Node> convert_quads_to_subdivs(Ref<Node> node);
    Ref<Node> convert_bezier_to_lines(Ref<Node> node);
    Ref<Node> convert_hair_to_curves(Ref<Node> node);

    Ref<Node> createTrianglePlane (const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy, size_t width, size_t height, Ref<MaterialNode> material = nullptr);
    Ref<Node> createQuadPlane     (const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy, size_t width, size_t height, Ref<MaterialNode> material = nullptr);
    Ref<Node> createSubdivPlane   (const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy, size_t width, size_t height, float tessellationRate, Ref<MaterialNode> material = nullptr);
    Ref<Node> createTriangleSphere(const Vec3fa& center, const float radius, size_t numPhi, Ref<MaterialNode> material = nullptr);
    Ref<Node> createQuadSphere    (const Vec3fa& center, const float radius, size_t numPhi, Ref<MaterialNode> material = nullptr);
    Ref<Node> createSubdivSphere  (const Vec3fa& center, const float radius, size_t numPhi, float tessellationRate, Ref<MaterialNode> material = nullptr);
    Ref<Node> createSphereShapedHair(const Vec3fa& center, const float radius, Ref<MaterialNode> material = nullptr);
  
    Ref<Node> createHairyPlane    (const Vec3fa& pos, const Vec3fa& dx, const Vec3fa& dy, const float len, const float r, size_t numHairs, bool hair, Ref<MaterialNode> material = nullptr);

    Ref<Node> createGarbageTriangleMesh (size_t numTriangles, bool mblur, Ref<MaterialNode> material = nullptr);
    Ref<Node> createGarbageQuadMesh (size_t numQuads, bool mblur, Ref<MaterialNode> material = nullptr);
    Ref<Node> createGarbageHair (size_t numHairs, bool mblur, Ref<MaterialNode> material = nullptr);
    Ref<Node> createGarbageLineSegments (size_t numLineSegments, bool mblur, Ref<MaterialNode> material = nullptr);
    Ref<Node> createGarbageSubdivMesh (size_t numFaces, bool mblur, Ref<MaterialNode> material = nullptr);

    struct Node : public RefCount
    {
      Node (bool closed = false)
        : indegree(0), closed(closed) {}

       /* resets indegree and closed parameters */
      void reset()
      {
        std::set<Ref<Node>> done;
        resetNode(done);
      }

      /* sets material */
      virtual void setMaterial(Ref<MaterialNode> material) {};

      /* resets indegree and closed parameters */
      virtual void resetNode(std::set<Ref<Node>>& done);

      /* calculates the number of parent nodes pointing to this node */
      virtual void calculateInDegree();

      /* calculates for each node if its subtree is closed, indegrees have to be calculated first */
      virtual bool calculateClosed();

      /* checks if the node is closed */
      __forceinline bool isClosed() const { return closed; }

      /* calculates bounding box of node */
      virtual BBox3fa bounds() const {
        return empty;
      }

    protected:
      size_t indegree;   // number of nodes pointing to us
      bool closed;       // determines if the subtree may represent an instance
    };
    
    struct TransformNode : public Node
    {
      ALIGNED_STRUCT;

      TransformNode (const AffineSpace3fa& xfm, const Ref<Node>& child)
        : xfm0(xfm), xfm1(xfm), child(child) {}

      TransformNode (const AffineSpace3fa& xfm0, const AffineSpace3fa& xfm1, const Ref<Node>& child)
        : xfm0(xfm0), xfm1(xfm1), child(child) {}

      virtual void setMaterial(Ref<MaterialNode> material) {
        child->setMaterial(material);
      }

      virtual void resetNode(std::set<Ref<Node>>& done);
      virtual void calculateInDegree();
      virtual bool calculateClosed();
      
      virtual BBox3fa bounds() 
      {
        const BBox3fa cbounds = child->bounds();
        const BBox3fa b0 = xfmBounds(xfm0,cbounds);
        const BBox3fa b1 = xfmBounds(xfm1,cbounds);
        return merge(b0,b1);
      }

    public:
      AffineSpace3fa xfm0;
      AffineSpace3fa xfm1;
      Ref<Node> child;
    };
    
    struct GroupNode : public Node
    { 
      GroupNode (const size_t N = 0) { 
        children.resize(N); 
      }

      size_t size() const {
        return children.size();
      }
      
      void add(const Ref<Node>& node) {
        if (node) children.push_back(node);
      }
      
      void set(const size_t i, const Ref<Node>& node) {
        children[i] = node;
      }

      virtual BBox3fa bounds() const
      {
        BBox3fa b = empty;
        for (auto c : children)
          b.extend(c->bounds());
        return b;
      }

      void triangles_to_quads()
      {
        for (size_t i=0; i<children.size(); i++)
          children[i] = convert_triangles_to_quads(children[i]);
      }

      void quads_to_subdivs()
      {
        for (size_t i=0; i<children.size(); i++)
          children[i] = convert_quads_to_subdivs(children[i]);
      }
      
      void bezier_to_lines()
      {
        for (size_t i=0; i<children.size(); i++)
          children[i] = convert_bezier_to_lines(children[i]);
      }

      void hair_to_curves()
      {
        for (size_t i=0; i<children.size(); i++)
          children[i] = convert_hair_to_curves(children[i]);
      }

      virtual void setMaterial(Ref<MaterialNode> material) {
        for (auto child : children) child->setMaterial(material);
      }

      virtual void resetNode(std::set<Ref<Node>>& done);
      virtual void calculateInDegree();
      virtual bool calculateClosed();
      
    public:
      std::vector<Ref<Node> > children;
    };
    
    template<typename Light>
      struct LightNode : public Node
    {
      ALIGNED_STRUCT;
      
      LightNode (const Light& light)
        : light(light) {}
      
      Light light;
    };
    
    struct MaterialNode : public Node
    {
      ALIGNED_STRUCT;

    MaterialNode(const Material& material)
      : material(material) {}
      
      Material material;
    };
    
    /*! Mesh. */
    struct TriangleMeshNode : public Node
    {
      typedef Vec3fa Vertex;

      struct Triangle 
      {
      public:
        Triangle() {}
        Triangle (int v0, int v1, int v2) 
        : v0(v0), v1(v1), v2(v2) {}
      public:
        int v0, v1, v2;
      };
      
    public:
      TriangleMeshNode (Ref<MaterialNode> material) 
        : Node(true), material(material) {}
      
      virtual void setMaterial(Ref<MaterialNode> material) {
        this->material = material;
      }

      virtual BBox3fa bounds() const
      {
        BBox3fa b = empty;
        for (auto x : v ) b.extend(x);
        for (auto x : v2) b.extend(x);
        return b;
      }

      void verify() const;

    public:
      avector<Vertex> v;
      avector<Vertex> v2;
      avector<Vertex> vn;
      std::vector<Vec2f> vt;
      std::vector<Triangle> triangles;
      Ref<MaterialNode> material;
    };

    /*! Mesh. */
    struct QuadMeshNode : public Node
    {
      typedef Vec3fa Vertex;

      struct Quad
      {
      public:
        Quad() {}
        Quad (int v0, int v1, int v2, int v3) 
        : v0(v0), v1(v1), v2(v2), v3(v3) {}
      public:
        int v0, v1, v2, v3;
      };
      
    public:
      QuadMeshNode (Ref<MaterialNode> material) 
        : Node(true), material(material) {}
      
      virtual void setMaterial(Ref<MaterialNode> material) {
        this->material = material;
      }

      virtual BBox3fa bounds() const
      {
        BBox3fa b = empty;
        for (auto x : v ) b.extend(x);
        for (auto x : v2) b.extend(x);
        return b;
      }

      void verify() const;

    public:
      avector<Vertex> v;
      avector<Vertex> v2;
      avector<Vertex> vn;
      std::vector<Vec2f> vt;
      std::vector<Quad> quads;
      Ref<MaterialNode> material;
    };

    /*! Subdivision Mesh. */
    struct SubdivMeshNode : public Node
    {
      typedef Vec3fa Vertex;

      SubdivMeshNode (Ref<MaterialNode> material) 
        : Node(true), material(material), boundaryMode(RTC_BOUNDARY_EDGE_ONLY), tessellationRate(2.0f) {}

      virtual void setMaterial(Ref<MaterialNode> material) {
        this->material = material;
      }

      virtual BBox3fa bounds() const
      {
        BBox3fa b = empty;
        for (auto x : positions ) b.extend(x);
        for (auto x : positions2) b.extend(x);
        return b;
      }

      void verify() const;

    public:
      avector<Vertex> positions;            //!< vertex positions
      avector<Vertex> positions2;           //!< vertex positions for 2nd timestep
      avector<Vertex> normals;              //!< face vertex normals
      std::vector<Vec2f> texcoords;             //!< face texture coordinates
      std::vector<int> position_indices;        //!< position indices for all faces
      std::vector<int> normal_indices;          //!< normal indices for all faces
      std::vector<int> texcoord_indices;        //!< texcoord indices for all faces
      std::vector<int> verticesPerFace;         //!< number of indices of each face
      std::vector<int> holes;                   //!< face ID of holes
      std::vector<Vec2i> edge_creases;          //!< index pairs for edge crease 
      std::vector<float> edge_crease_weights;   //!< weight for each edge crease
      std::vector<int> vertex_creases;          //!< indices of vertex creases
      std::vector<float> vertex_crease_weights; //!< weight for each vertex crease
      Ref<MaterialNode> material;
      RTCBoundaryMode boundaryMode;
      float tessellationRate;
    };

    /*! Line Segments */
    struct LineSegmentsNode : public Node
    {
      typedef Vec3fa Vertex;

    public:
      LineSegmentsNode (Ref<MaterialNode> material)
        : Node(true), material(material) {}
      
      virtual void setMaterial(Ref<MaterialNode> material) {
        this->material = material;
      }

      virtual BBox3fa bounds() const
      {
        BBox3fa b = empty;
        for (auto x : v ) b.extend(x);
        for (auto x : v2) b.extend(x);
        return b;
      }

      void verify() const;

    public:
      avector<Vertex> v;        //!< control points (x,y,z,r)
      avector<Vertex> v2;       //!< control points (x,y,z,r)
      std::vector<int> indices; //!< list of line segments
      Ref<MaterialNode> material;
    };

    /*! Hair Set. */
    struct HairSetNode : public Node
    {
      typedef Vec3fa Vertex;

      struct Hair
      {
      public:
        Hair () {}
        Hair (int vertex, int id)
        : vertex(vertex), id(id) {}

      public:
        int vertex,id;  //!< index of first control point and hair ID
      };
      
    public:
      HairSetNode (bool hair, Ref<MaterialNode> material)
        : Node(true), hair(hair), material(material) {}

      virtual void setMaterial(Ref<MaterialNode> material) {
        this->material = material;
      }

      virtual BBox3fa bounds() const
      {
        BBox3fa b = empty;
        for (auto x : v ) b.extend(x);
        for (auto x : v2) b.extend(x);
        return b;
      }

      void verify() const;

    public:
      bool hair;                //!< true is this is hair geometry, false if this are curves
      avector<Vertex> v;        //!< hair control points (x,y,z,r)
      avector<Vertex> v2;       //!< hair control points (x,y,z,r)
      std::vector<Hair> hairs;  //!< list of hairs
      Ref<MaterialNode> material;
    };
  };
}
