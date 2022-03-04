// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lights.h"
#include "../../../include/embree3/rtcore.h"
RTC_NAMESPACE_USE
#include "../math/random_sampler.h"

namespace embree
{  
  struct Material;
  
  namespace SceneGraph
  {
    struct Node;
    struct MaterialNode;
    struct Transformations;
    struct TriangleMeshNode;
    struct QuadMeshNode;
    struct GridMeshNode;

    Ref<Node> load(const FileName& fname, bool singleObject = false);
    void store(Ref<Node> root, const FileName& fname, bool embedTextures, bool referenceMaterials, bool binaryFormat);
    void extend_animation(Ref<Node> node0, Ref<Node> node1);
    void optimize_animation(Ref<Node> node0);
    void set_motion_vector(Ref<Node> node, const Vec3fa& dP);
    void set_motion_vector(Ref<Node> node, const avector<Vec3fa>& motion_vector);
    void set_time_range(Ref<SceneGraph::Node> node, const BBox1f& time_range);
    void resize_randomly(RandomSampler& sampler, Ref<Node> node, const size_t N);
    Ref<Node> convert_triangles_to_quads(Ref<Node> node, float prop);
    Ref<Node> convert_triangles_to_quads( Ref<TriangleMeshNode> tmesh);
    Ref<Node> convert_quads_to_subdivs(Ref<Node> node);
    Ref<Node> my_merge_quads_to_grids(Ref<SceneGraph::Node> node);
    Ref<Node> convert_bezier_to_lines(Ref<Node> node);
    Ref<Node> convert_bezier_to_bspline(Ref<Node> node);
    Ref<Node> convert_bezier_to_hermite(Ref<Node> node);
    Ref<Node> convert_bspline_to_bezier(Ref<Node> node);
    Ref<Node> convert_flat_to_round_curves(Ref<Node> node);
    Ref<Node> convert_round_to_flat_curves(Ref<Node> node);
    Ref<Node> convert_quads_to_grids( Ref<QuadMeshNode> qmesh,  const unsigned resX, const unsigned resY );
    Ref<Node> convert_quads_to_grids( Ref<Node> node, const unsigned resX, const unsigned resY );
    Ref<Node> convert_grids_to_quads( Ref<GridMeshNode> gmesh);
    Ref<Node> convert_grids_to_quads( Ref<Node> node);

    Ref<Node> remove_mblur(Ref<Node> node, bool mblur);
    void convert_mblur_to_nonmblur(Ref<Node> node);

    extern void (*opaque_geometry_destruction)(void*);

    struct Statistics
    {
      Statistics ()
      : numTriangleMeshes(0), numTriangles(0), numTriangleBytes(0),
        numQuadMeshes(0),     numQuads(0),     numQuadBytes(0),
        numSubdivMeshes(0),   numPatches(0),   numSubdivBytes(0),
        numCurveSets(0),      numCurves(0),    numCurveBytes(0),
        numGridMeshNodes(0),  numGrids(0),     numGridBytes(0),
        numPointSets(0),      numPoints(0),    numPointBytes(0),
        numTransformNodes(0),
        numTransformedObjects(0),
        numLights(0),
        numCameras(0),
        numMaterials(0) {}

      void print();
      
      size_t numTriangleMeshes;
      size_t numTriangles;
      size_t numTriangleBytes;
      
      size_t numQuadMeshes;
      size_t numQuads;
      size_t numQuadBytes;
      
      size_t numSubdivMeshes;
      size_t numPatches;
      size_t numSubdivBytes;
      
      size_t numCurveSets;
      size_t numCurves;
      size_t numCurveBytes;
      
      size_t numGridMeshNodes;
      size_t numGrids;
      size_t numGridBytes;
      
      size_t numPointSets;
      size_t numPoints;
      size_t numPointBytes;

      size_t numTransformNodes;
      size_t numTransformedObjects;
      
      size_t numLights;
      size_t numCameras;
      size_t numMaterials;
    };
    
    struct Node : public RefCount
    {
      Node (bool closed = false)
        : indegree(0), closed(closed), hasLightOrCamera(false), id(-1), geometry(nullptr) {}

      Node (const std::string& name) 
        : name(name), indegree(0), closed(false), id(-1), geometry(nullptr) {}

      ~Node() {
        if (opaque_geometry_destruction)
          opaque_geometry_destruction(geometry);
      }

      /* prints scenegraph */
      virtual void print(std::ostream& cout, int depth = 0) = 0;

      /* sets material */
      virtual void setMaterial(Ref<MaterialNode> material) {};

      /* calculates the number of parent nodes pointing to this node */
      virtual void calculateInDegree();

      /* calculates for each node if its subtree is closed, indegrees have to be calculated first */
      virtual bool calculateClosed(bool group_instancing);

      /* resets the number of parent nodes pointing to this node */
      virtual void resetInDegree();

      /* calculates statistics */
      virtual void calculateStatistics(Statistics& stat);

      /* checks if the node is closed */
      __forceinline bool isClosed() const { return closed; }

      /* calculates bounding box of node */
      virtual BBox3fa bounds() const {
        return empty;
      }

      /* calculates linear bounding box of node */
      virtual LBBox3fa lbounds() const {
        return empty;
      }

      /* calculates number of primitives */
      virtual size_t numPrimitives() const {
        return 0;
      }

      Ref<Node> set_motion_vector(const Vec3fa& dP) {
        SceneGraph::set_motion_vector(this,dP); return this;
      }

      Ref<Node> set_motion_vector(const avector<Vec3fa>& motion_vector) {
        SceneGraph::set_motion_vector(this,motion_vector); return this;
      }

    public:
      std::string fileName; // when set to some filename the exporter references this file
      std::string name;     // name of this node
      size_t indegree;      // number of nodes pointing to us
      bool closed;          // determines if the subtree may represent an instance
      bool hasLightOrCamera;
      unsigned int id;
      void* geometry;
    };

    struct Transformations
    {
      __forceinline Transformations() {}

      __forceinline Transformations(OneTy)
        : time_range(0.0f,1.0f)
      {
        spaces.push_back(one);
      }

      __forceinline Transformations( const BBox1f& time_range, size_t N ) 
        : time_range(time_range), spaces(N) {}
      
      __forceinline Transformations(const AffineSpace3fa& space)
        : time_range(0.0f,1.0f)
      {
        spaces.push_back(space);
      }

      __forceinline Transformations(const AffineSpace3fa& space0, const AffineSpace3fa& space1)
        : time_range(0.0f,1.0f)
      {
        spaces.push_back(space0);
        spaces.push_back(space1);
      }

      __forceinline Transformations(const avector<AffineSpace3ff>& spaces)
        : time_range(0.0f,1.0f), spaces(spaces) { assert(spaces.size()); }

      __forceinline size_t size() const {
        return spaces.size();
      }

      __forceinline       AffineSpace3ff& operator[] ( const size_t i )       { return spaces[i]; }
      __forceinline const AffineSpace3ff& operator[] ( const size_t i ) const { return spaces[i]; }

      BBox3fa bounds ( const BBox3fa& cbounds ) const 
      {
        BBox3fa r = empty;
        for (size_t i=0; i<spaces.size(); i++)
          r.extend(xfmBounds(spaces[i],cbounds));
        return r;
      }

      LBBox3fa lbounds ( const LBBox3fa& cbounds ) const 
      {
        assert(spaces.size());
        if (spaces.size() == 1) 
        {
          return LBBox3fa(xfmBounds(spaces[0],cbounds.bounds0),
                          xfmBounds(spaces[0],cbounds.bounds1));
        }
        else
        {
          avector<BBox3fa> bounds(spaces.size());
          for (size_t i=0; i<spaces.size(); i++) {
            const float f = float(i)/float(spaces.size()-1);
            bounds[i] = xfmBounds(spaces[i],cbounds.interpolate(f));
          }
          return LBBox3fa(bounds);
        }
      }

      void add (const Transformations& other) {
        for (size_t i=0; i<other.size(); i++) spaces.push_back(other[i]);
      }

      static __forceinline bool isIdentity(AffineSpace3ff const& M, bool q)
      {
        if (M.l.vx.x      != 1.f) return false;
        if (M.l.vx.y      != 0.f) return false;
        if (M.l.vx.z      != 0.f) return false;
        if (q && M.l.vx.w != 0.f) return false;

        if (M.l.vy.x      != 0.f) return false;
        if (M.l.vy.y      != 1.f) return false;
        if (M.l.vy.z      != 0.f) return false;
        if (q && M.l.vy.w != 0.f) return false;

        if (M.l.vz.x      != 0.f) return false;
        if (M.l.vz.y      != 0.f) return false;
        if (M.l.vz.z      != 1.f) return false;
        if (q && M.l.vz.w != 0.f) return false;

        if (M.p.x         != 0.f) return false;
        if (M.p.y         != 0.f) return false;
        if (M.p.z         != 0.f) return false;
        if (q && M.p.w    != 1.f) return false;

        return true;
      }

      static __forceinline AffineSpace3ff mul(AffineSpace3ff const& M0, AffineSpace3ff const& M1, bool q0, bool q1, bool& q)
      {
        q = false;
        if (isIdentity(M0, q0)) { q = q1; return M1; }
        if (isIdentity(M1, q1)) { q = q0; return M0; }

        // simple case non of the transformations is a quaternion
        if (q0 == false && q1 == false)
        {
          return M0 * M1;
        }
        else if (q0 == true && q1 == true)
        {
          std::cout << "warning: cannot multiply two quaternion decompositions. will convert to regular transforms and multiply" << std::endl;
          return quaternionDecompositionToAffineSpace(M0) * quaternionDecompositionToAffineSpace(M1);
        }
        else if (q0 == true && q1 == false)
        {
          AffineSpace3fa S; Quaternion3f Q; Vec3fa T;
          quaternionDecomposition(M0, T, Q, S);
          S = S * AffineSpace3fa(M1);
          if (S.l.vx.y != 0.f || S.l.vx.z != 0 || S.l.vy.z != 0)
            std::cout << "warning: cannot multiply quaternion and general transformation matrix. will ignore lower diagonal" << std::endl;
          q = true;
          return quaternionDecomposition(T, Q, S);
        }
        else {
          if (M0.l.vx.y != 0.f || M0.l.vx.z != 0 || M0.l.vy.z != 0 || M0.l.vy.x != 0.f || M0.l.vz.x != 0 || M0.l.vz.y != 0)
            std::cout << "warning: cannot multiply general transformation matrix and quaternion. will only consider translation and diagonal as scale factors" << std::endl;
          AffineSpace3ff M = M1;
          M.l.vx.y += M0.p.x;
          M.l.vx.z += M0.p.y;
          M.l.vy.z += M0.p.z;
          M.l.vx.x *= M0.l.vx.x;
          M.l.vy.y *= M0.l.vy.y;
          M.l.vz.z *= M0.l.vz.z;
          q = true;
          return M;
        }
      }

      friend __forceinline Transformations operator* ( const Transformations& a, const Transformations& b )
      {
        if (a.size() == 1)
        {
          Transformations c(intersect(a.time_range,b.time_range),b.size());
          for (size_t i=0; i<b.size(); i++) c[i] = mul(a[0], b[i], a.quaternion, b.quaternion, c.quaternion);
          return c;
        } 
        else if (b.size() == 1)
        {
          Transformations c(intersect(a.time_range,b.time_range),a.size());
          c.quaternion = a.quaternion || b.quaternion;
          for (size_t i=0; i<a.size(); i++) c[i] = mul(a[i], b[0], a.quaternion, b.quaternion, c.quaternion);
          return c;
        }
        else if (a.size() == b.size())
        {
          Transformations c(intersect(a.time_range,b.time_range),a.size());
          c.quaternion = a.quaternion || b.quaternion;
          for (size_t i=0; i<a.size(); i++) c[i] = mul(a[i], b[i], a.quaternion, b.quaternion, c.quaternion);
          return c;
        }
        else
          THROW_RUNTIME_ERROR("number of transformations does not match");
      }

      AffineSpace3ff interpolate (const float gtime) const
      {
        assert(time_range.lower == 0.0f && time_range.upper == 1.0f);
        if (spaces.size() == 1) return spaces[0];

        /* calculate time segment itime and fractional time ftime */
        const int time_segments = int(spaces.size()-1);
        const float time = gtime*float(time_segments);
        const int itime = clamp(int(floor(time)),0,time_segments-1);
        const float ftime = time - float(itime);
        return lerp(spaces[itime+0],spaces[itime+1],ftime);
      }

    public:
      BBox1f time_range;
      avector<AffineSpace3ff> spaces;
      bool quaternion = false;
    };

    template<typename Vertex>
       std::vector<avector<Vertex>> transformMSMBlurBuffer(const std::vector<avector<Vertex>>& positions_in, const Transformations& spaces)
    {
      std::vector<avector<Vertex>> positions_out;
      const size_t num_time_steps = positions_in.size(); assert(num_time_steps);
      const size_t num_vertices = positions_in[0].size();

      /* if we have only one set of vertices, use transformation to generate more vertex sets */
      if (num_time_steps == 1)
      {
        for (size_t i=0; i<spaces.size(); i++) 
        {
          avector<Vertex> verts(num_vertices);
          for (size_t j=0; j<num_vertices; j++) {
            verts[j] = xfmPoint(spaces[i],positions_in[0][j]);
            verts[j].w = positions_in[0][j].w;
          }
          positions_out.push_back(std::move(verts));
        }
      } 
      /* otherwise transform all vertex sets with interpolated transformation */
      else
      {
        for (size_t t=0; t<num_time_steps; t++) 
        {
          float time = num_time_steps > 1 ? float(t)/float(num_time_steps-1) : 0.0f;
          const AffineSpace3ff space = spaces.interpolate(time);
          avector<Vertex> verts(num_vertices);
          for (size_t i=0; i<num_vertices; i++) {
            verts[i] = xfmPoint (space,positions_in[t][i]);
            verts[i].w = positions_in[t][i].w;
          }
          positions_out.push_back(std::move(verts));
        }
      }
      return positions_out;
    }

    inline std::vector<avector<Vec3fa>> transformMSMBlurVec3faBuffer(const std::vector<avector<Vec3fa>>& positions_in, const Transformations& spaces)
    {
      std::vector<avector<Vec3fa>> positions_out;
      const size_t num_time_steps = positions_in.size(); assert(num_time_steps);
      const size_t num_vertices = positions_in[0].size();

      /* if we have only one set of vertices, use transformation to generate more vertex sets */
      if (num_time_steps == 1)
      {
        for (size_t i=0; i<spaces.size(); i++) 
        {
          avector<Vec3fa> verts(num_vertices);
          for (size_t j=0; j<num_vertices; j++) {
            verts[j] = xfmPoint((AffineSpace3fa)spaces[i],positions_in[0][j]);
          }
          positions_out.push_back(std::move(verts));
        }
      } 
      /* otherwise transform all vertex sets with interpolated transformation */
      else
      {
        for (size_t t=0; t<num_time_steps; t++) 
        {
          float time = num_time_steps > 1 ? float(t)/float(num_time_steps-1) : 0.0f;
          const AffineSpace3ff space = spaces.interpolate(time);
          avector<Vec3fa> verts(num_vertices);
          for (size_t i=0; i<num_vertices; i++) {
            verts[i] = xfmPoint ((AffineSpace3fa)space,positions_in[t][i]);
          }
          positions_out.push_back(std::move(verts));
        }
      }
      return positions_out;
    }

    template<typename Vertex>
       std::vector<avector<Vertex>> transformMSMBlurVectorBuffer(const std::vector<avector<Vertex>>& vectors_in, const Transformations& spaces)
    {
      if (vectors_in.size() == 0)
        return vectors_in;
      
      std::vector<avector<Vertex>> vectors_out;
      const size_t num_time_steps = vectors_in.size();
      const size_t num_vertices = vectors_in[0].size();

      /* if we have only one set of vertices, use transformation to generate more vertex sets */
      if (num_time_steps == 1)
      {
        for (size_t i=0; i<spaces.size(); i++) 
        {
          avector<Vertex> vecs(num_vertices);
          for (size_t j=0; j<num_vertices; j++) {
            vecs[j] = xfmVector(spaces[i],vectors_in[0][j]);
            vecs[j].w = vectors_in[0][j].w;
          }
          vectors_out.push_back(std::move(vecs));
        }
      } 
      /* otherwise transform all vertex sets with interpolated transformation */
      else
      {
        for (size_t t=0; t<num_time_steps; t++) 
        {
          float time = num_time_steps > 1 ? float(t)/float(num_time_steps-1) : 0.0f;
          const AffineSpace3ff space = spaces.interpolate(time);
          avector<Vertex> vecs(num_vertices);
          for (size_t i=0; i<num_vertices; i++) {
            vecs[i] = xfmVector (space,vectors_in[t][i]);
            vecs[i].w = vectors_in[t][i].w;
          }
          vectors_out.push_back(std::move(vecs));
        }
      }
      return vectors_out;
    }

    template<typename Vertex>
       std::vector<avector<Vertex>> transformMSMBlurVectorVec3faBuffer(const std::vector<avector<Vertex>>& vectors_in, const Transformations& spaces)
    {
      if (vectors_in.size() == 0)
        return vectors_in;
      
      std::vector<avector<Vertex>> vectors_out;
      const size_t num_time_steps = vectors_in.size();
      const size_t num_vertices = vectors_in[0].size();

      /* if we have only one set of vertices, use transformation to generate more vertex sets */
      if (num_time_steps == 1)
      {
        for (size_t i=0; i<spaces.size(); i++) 
        {
          avector<Vertex> vecs(num_vertices);
          for (size_t j=0; j<num_vertices; j++) {
            vecs[j] = xfmVector((AffineSpace3fa)spaces[i],vectors_in[0][j]);
          }
          vectors_out.push_back(std::move(vecs));
        }
      } 
      /* otherwise transform all vertex sets with interpolated transformation */
      else
      {
        for (size_t t=0; t<num_time_steps; t++) 
        {
          float time = num_time_steps > 1 ? float(t)/float(num_time_steps-1) : 0.0f;
          const AffineSpace3ff space = spaces.interpolate(time);
          avector<Vertex> vecs(num_vertices);
          for (size_t i=0; i<num_vertices; i++) {
            vecs[i] = xfmVector ((AffineSpace3fa)space,vectors_in[t][i]);
          }
          vectors_out.push_back(std::move(vecs));
        }
      }
      return vectors_out;
    }

    template<typename Vertex>
       std::vector<avector<Vertex>> transformMSMBlurNormalBuffer(const std::vector<avector<Vertex>>& normals_in, const Transformations& spaces)
    {
      if (normals_in.size() == 0)
        return normals_in;
      
      std::vector<avector<Vertex>> normals_out;
      const size_t num_time_steps = normals_in.size();
      const size_t num_vertices = normals_in[0].size();

      /* if we have only one set of vertices, use transformation to generate more vertex sets */
      if (num_time_steps == 1)
      {
        for (size_t i=0; i<spaces.size(); i++) 
        {
          avector<Vertex> norms(num_vertices);
          for (size_t j=0; j<num_vertices; j++) {
            norms[j] = xfmNormal((AffineSpace3fa)spaces[i],normals_in[0][j]);
          }
          normals_out.push_back(std::move(norms));
        }
      } 
      /* otherwise transform all vertex sets with interpolated transformation */
      else
      {
        for (size_t t=0; t<num_time_steps; t++) 
        {
          float time = num_time_steps > 1 ? float(t)/float(num_time_steps-1) : 0.0f;
          const AffineSpace3ff space = spaces.interpolate(time);
          avector<Vertex> norms(num_vertices);
          for (size_t i=0; i<num_vertices; i++) {
            norms[i] = xfmNormal ((AffineSpace3fa)space,normals_in[t][i]);
          }
          normals_out.push_back(std::move(norms));
        }
      }
      return normals_out;
    }

    struct PerspectiveCameraData
    {
      PerspectiveCameraData()
        : from(1,0,0), to(0,0,0), up(0,1,0), fov(30) {}
      
      PerspectiveCameraData (const Vec3fa& from, const Vec3fa& to, const Vec3fa& up, const float fov)
        : from(from), to(to), up(up), fov(fov) {}

      PerspectiveCameraData (const PerspectiveCameraData& other, const AffineSpace3fa& space)
        : from(xfmPoint(space,other.from)), to(xfmPoint(space,other.to)), up(xfmVector(space,other.up)), fov(other.fov) {}

      friend PerspectiveCameraData lerp(const PerspectiveCameraData& a, const PerspectiveCameraData& b, const float t)
      {
        const Vec3fa from = embree::lerp(a.from, b.from, t);
        const Vec3fa to   = embree::lerp(a.to  , b.to  , t);
        const Vec3fa up   = embree::lerp(a.up  , b.up  , t);
        const float  fov  = embree::lerp(a.fov , b.fov , t);
        return PerspectiveCameraData(from,to,up,fov);
      }

    public:
      Vec3fa from;   //!< position of camera
      Vec3fa to;     //!< look at point
      Vec3fa up;     //!< up vector
      float fov;     //!< vertical field of view
    };

    struct PerspectiveCameraNode : public Node
    {
      ALIGNED_STRUCT_(16);

      PerspectiveCameraNode (std::string name = "")
        : Node(name) {}
      
      PerspectiveCameraNode (const Vec3fa& from, const Vec3fa& to, const Vec3fa& up, const float fov)
        : data(from, to, up, fov) {}

      PerspectiveCameraNode (const Ref<PerspectiveCameraNode>& other, const AffineSpace3fa& space, const std::string& id = "")
        : Node(id), data(other->data,space) {}

      virtual bool isAnimated() const {
        return false;
      }

      virtual PerspectiveCameraData get(float time) const {
        return data;
      }
      
      virtual void print(std::ostream& cout, int depth);
      
      virtual void calculateStatistics(Statistics& stat);
      virtual bool calculateClosed(bool group_instancing);
            
    public:
      PerspectiveCameraData data;
    };

    struct AnimatedPerspectiveCameraNode : public PerspectiveCameraNode
    {
      AnimatedPerspectiveCameraNode (std::vector<Ref<PerspectiveCameraNode>>&& cameras, BBox1f time_range, const std::string& id = "")
        : time_range(time_range), cameras(cameras) {}

      AnimatedPerspectiveCameraNode (const Ref<AnimatedPerspectiveCameraNode>& other, const AffineSpace3fa& space, const std::string& id)
        : PerspectiveCameraNode(id), time_range(other->time_range)
      {
        cameras.resize(other->size());
        for (size_t i=0; i<other->size(); i++)
          cameras[i] = new PerspectiveCameraNode(other->cameras[i],space);
      }

      virtual bool isAnimated() const {
        return true;
      }

      virtual PerspectiveCameraData get(float time) const
      {
        time = frac((time-time_range.lower)/time_range.size());
        time = (cameras.size()-1)*time;
        int   itime = (int)floor(time);
        itime = min(max(itime,0),(int)cameras.size()-2);
        float ftime = time - (float)itime;
        return lerp(cameras[itime+0]->get(time), cameras[itime+1]->get(time), ftime);
      }
      
      virtual void print(std::ostream& cout, int depth);

      virtual void calculateStatistics(Statistics& stat);
      virtual bool calculateClosed(bool group_instancing);

      size_t size() const { return cameras.size(); }

    public:
      BBox1f time_range;
      std::vector<Ref<PerspectiveCameraNode>> cameras;
    };

    struct TransformNode : public Node
    {
      ALIGNED_STRUCT_(16);

      TransformNode (const AffineSpace3fa& xfm, const Ref<Node>& child)
        : spaces((AffineSpace3ff)xfm), child(child) {}

      TransformNode (const AffineSpace3fa& xfm0, const AffineSpace3fa& xfm1, const Ref<Node>& child)
        : spaces((AffineSpace3ff)xfm0,(AffineSpace3ff)xfm1), child(child) {}

      TransformNode (const avector<AffineSpace3ff>& spaces, const Ref<Node>& child)
        : spaces(spaces), child(child) {}

      TransformNode(const Transformations& spaces, const Ref<Node>& child)
        : spaces(spaces), child(child) {}

      virtual void setMaterial(Ref<MaterialNode> material) {
        child->setMaterial(material);
      }

      virtual void print(std::ostream& cout, int depth);
      
      virtual void calculateStatistics(Statistics& stat);
      virtual void calculateInDegree();
      virtual bool calculateClosed(bool group_instancing);
      virtual void resetInDegree();
      
      virtual BBox3fa bounds() const {
        return spaces.bounds(child->bounds());
      }

      virtual LBBox3fa lbounds() const {
        return spaces.lbounds(child->lbounds());
      }

      virtual size_t numPrimitives() const {
        return child->numPrimitives();
      }

      virtual AffineSpace3ff get(float time) const
      {
        if (spaces.size() <= 1) return spaces[0];

        int numTimeSteps = spaces.size();
        
        BBox1f time_range = spaces.time_range;
        time = frac((time-time_range.lower)/time_range.size());
        time = (numTimeSteps-1)*time;
        int   itime = (int)floor(time);
        itime = min(max(itime,0),(int)numTimeSteps-2);
        float ftime = time - (float)itime;
    
        const AffineSpace3ff xfm0 = spaces[itime+0];
        const AffineSpace3ff xfm1 = spaces[itime+1];
        const AffineSpace3ff xfm  = lerp(xfm0,xfm1,ftime);
        return xfm;
      }

    public:
      Transformations spaces;
      Ref<Node> child;
    };
    
    struct GroupNode : public Node
    { 
      GroupNode (const size_t N = 0) { 
        children.resize(N); 
      }

      GroupNode (std::vector<Ref<Node>>& children)
        : children(children) {}

      size_t size() const {
        return children.size();
      }
      
      void add(const Ref<Node>& node) {
        if (node) children.push_back(node);
      }
      
      void set(const size_t i, const Ref<Node>& node) {
        children[i] = node;
      }
 
      Ref<Node> child ( size_t i ) const {
        return children[i];
      }

      virtual BBox3fa bounds() const
      {
        BBox3fa b = empty;
        for (auto c : children) b.extend(c->bounds());
        return b;
      }

      virtual LBBox3fa lbounds() const
      {
        LBBox3fa b = empty;
        for (auto c : children) b.extend(c->lbounds());
        return b;
      }

      virtual size_t numPrimitives() const 
      {
        size_t n = 0;
        for (auto child : children) n += child->numPrimitives();
        return n;
      }

      void triangles_to_quads(float prop = inf)
      {
        for (size_t i=0; i<children.size(); i++)
          children[i] = convert_triangles_to_quads(children[i],prop);
      }

      void quads_to_grids(unsigned int resX, unsigned int resY)
      {
        for (size_t i=0; i<children.size(); i++)
          children[i] = convert_quads_to_grids(children[i],resX, resY);
      }

      void grids_to_quads()
      {
        for (size_t i=0; i<children.size(); i++)
          children[i] = convert_grids_to_quads(children[i]);
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

      void flat_to_round_curves()
      {
        for (size_t i=0; i<children.size(); i++)
          children[i] = convert_flat_to_round_curves(children[i]);
      }

      void round_to_flat_curves()
      {
        for (size_t i=0; i<children.size(); i++)
          children[i] = convert_round_to_flat_curves(children[i]);
      }

      void bezier_to_bspline()
      {
        for (size_t i=0; i<children.size(); i++)
          children[i] = convert_bezier_to_bspline(children[i]);
      }

      void bezier_to_hermite()
      {
        for (size_t i=0; i<children.size(); i++)
          children[i] = convert_bezier_to_hermite(children[i]);
      }

      void bspline_to_bezier()
      {
        for (size_t i=0; i<children.size(); i++)
          children[i] = convert_bspline_to_bezier(children[i]);
      }

      void merge_quads_to_grids()
      {
        for (size_t i=0; i<children.size(); i++)
          children[i] = my_merge_quads_to_grids(children[i]);
      }

      void remove_mblur(bool mblur)
      {
        for (size_t i=0; i<children.size(); i++)
          SceneGraph::remove_mblur(children[i], mblur);
      }

      virtual void setMaterial(Ref<MaterialNode> material) {
        for (auto& child : children) child->setMaterial(material);
      }

      virtual void print(std::ostream& cout, int depth);
      
      virtual void calculateStatistics(Statistics& stat);
      virtual void calculateInDegree();
      virtual bool calculateClosed(bool group_instancing);
      virtual void resetInDegree();
      
    public:
      std::vector<Ref<Node> > children;
    };

    struct LightNode : public Node
    {
      virtual void print(std::ostream& cout, int depth);
      virtual void calculateStatistics(Statistics& stat);
      virtual bool calculateClosed(bool group_instancing);

      virtual LightType getType() const = 0;
      virtual Ref<LightNode> transform(const AffineSpace3fa& space) const = 0;
      virtual Ref<LightNode> lerp(const Ref<LightNode>& light1_in, float f) const = 0;
      virtual Ref<LightNode> get(float time) const = 0;
    };

    template<typename Light>
    struct LightNodeImpl : public LightNode
    {
      ALIGNED_STRUCT_(16);
      
      LightNodeImpl (const Light& light)
        : light(light) {}

      virtual LightType getType() const {
        return light.getType();
      }
      
      virtual Ref<LightNode> transform(const AffineSpace3fa& space) const {
        return new LightNodeImpl(light.transform(space));
      }

      virtual Ref<LightNode> get(float time) const {
        return (LightNode*) this;
      }

      virtual Ref<LightNode> lerp(const Ref<LightNode>& light1_in, float f) const
      {
        const Ref<LightNodeImpl<Light>> light1 = light1_in.dynamicCast<LightNodeImpl<Light>>();
        assert(light1);
        return new LightNodeImpl(Light::lerp(light,light1->light,f));
      }
      
      Light light;
    };

    struct AnimatedLightNode : public LightNode
    {
      AnimatedLightNode (const std::vector<Ref<LightNode>>&& lights, BBox1f time_range)
        : lights(lights), time_range(time_range) {}

      virtual LightType getType() const {
        return lights[0]->getType();
      }
      
      virtual Ref<LightNode> transform(const AffineSpace3fa& space) const
      {
        std::vector<Ref<LightNode>> xfm_lights(lights.size());
        for (size_t i=0; i<lights.size(); i++)
          xfm_lights[i] = lights[i]->transform(space);
        return new AnimatedLightNode(std::move(xfm_lights), time_range);
      }

      virtual Ref<LightNode> get(float time) const
      {
        time = frac((time-time_range.lower)/time_range.size());
        time = (lights.size()-1)*time;
        int   itime = (int)floor(time);
        itime = min(max(itime,0),(int)lights.size()-2);
        float ftime = time - (float)itime;
        Ref<LightNode> light0 = lights[itime+0]->get(time);
        Ref<LightNode> light1 = lights[itime+1]->get(time);
        return light0->lerp(light1,ftime);
      }

      virtual Ref<LightNode> lerp(const Ref<LightNode>& light1_in, float f) const {
        assert(false); return nullptr;
      }
      
    public: 
      std::vector<Ref<LightNode>> lights;
      BBox1f time_range;
    };

    
    struct MaterialNode : public Node
    {
      ALIGNED_STRUCT_(16);

      MaterialNode(const std::string& name = "")
        : Node(name) {}

      virtual Material* material() = 0;

      virtual void print(std::ostream& cout, int depth);
      virtual void calculateStatistics(Statistics& stat);
    };

    /*! Mesh. */
    struct TriangleMeshNode : public Node
    {
      typedef Vec3fa Vertex;

      struct Triangle 
      {
      public:
        Triangle() {}
        Triangle (unsigned v0, unsigned v1, unsigned v2) 
        : v0(v0), v1(v1), v2(v2) {}
      public:
        unsigned v0, v1, v2;
      };
      
    public:
      TriangleMeshNode (const avector<Vertex>& positions_in, 
                        const avector<Vertex>& normals_in, 
                        const std::vector<Vec2f>& texcoords,
                        const std::vector<Triangle>& triangles,
                        Ref<MaterialNode> material) 
        : Node(true), time_range(0.0f,1.0f), texcoords(texcoords), triangles(triangles), material(material) 
      {
        positions.push_back(positions_in);
        normals.push_back(normals_in);
      }

      TriangleMeshNode (Ref<MaterialNode> material, const BBox1f time_range = BBox1f(0,1), size_t numTimeSteps = 0) 
        : Node(true), time_range(time_range), material(material) 
      {
        for (size_t i=0; i<numTimeSteps; i++)
          positions.push_back(avector<Vertex>());
      }

      TriangleMeshNode (Ref<SceneGraph::TriangleMeshNode> imesh, const Transformations& spaces)
        : Node(true),
          time_range(imesh->time_range),
          positions(transformMSMBlurVec3faBuffer(imesh->positions,spaces)),
          normals(transformMSMBlurNormalBuffer(imesh->normals,spaces)),
          texcoords(imesh->texcoords), triangles(imesh->triangles), material(imesh->material) {}
      
      virtual void setMaterial(Ref<MaterialNode> material) {
        this->material = material;
      }

      virtual BBox3fa bounds() const
      {
        BBox3fa b = empty;
        for (const auto& p : positions)
          for (auto x : p)
            b.extend(x);
        return b;
      }

      virtual LBBox3fa lbounds() const
      {
        avector<BBox3fa> bboxes(positions.size());
        for (size_t t=0; t<positions.size(); t++) {
          BBox3fa b = empty;
          for (auto x : positions[t]) b.extend(x);
          bboxes[t] = b;
        }
        return LBBox3fa(bboxes);
      }

      virtual size_t numPrimitives() const {
        return triangles.size();
      }

      size_t numVertices() const {
        assert(positions.size());
        return positions[0].size();
      }

      size_t numTimeSteps() const {
        return positions.size();
      }

      size_t numBytes() const {
        return numPrimitives()*sizeof(Triangle) + numVertices()*numTimeSteps()*sizeof(Vertex);
      }

      void verify() const;

      virtual void print(std::ostream& cout, int depth);
      virtual void calculateStatistics(Statistics& stat);
      virtual void calculateInDegree();
      virtual void resetInDegree();
      
    public:
      BBox1f time_range;
      std::vector<avector<Vertex>> positions;
      std::vector<avector<Vertex>> normals;
      std::vector<Vec2f> texcoords;
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
        Quad (unsigned int v0, unsigned int v1, unsigned int v2, unsigned int v3) 
        : v0(v0), v1(v1), v2(v2), v3(v3) {}
      public:
        unsigned int v0, v1, v2, v3;
      };
      
    public:
      QuadMeshNode (Ref<MaterialNode> material, const BBox1f time_range = BBox1f(0,1), size_t numTimeSteps = 0 ) 
        : Node(true), time_range(time_range), material(material) 
      {
        for (size_t i=0; i<numTimeSteps; i++)
          positions.push_back(avector<Vertex>());
      }

      QuadMeshNode (Ref<SceneGraph::QuadMeshNode> imesh, const Transformations& spaces)
        : Node(true),
          time_range(imesh->time_range),
          positions(transformMSMBlurVec3faBuffer(imesh->positions,spaces)),
          normals(transformMSMBlurNormalBuffer(imesh->normals,spaces)),
          texcoords(imesh->texcoords), quads(imesh->quads), material(imesh->material) {}
   
      virtual void setMaterial(Ref<MaterialNode> material) {
        this->material = material;
      }

      virtual BBox3fa bounds() const
      {
        BBox3fa b = empty;
        for (const auto& p : positions)
          for (auto x : p)
            b.extend(x);
        return b;
      }

      virtual LBBox3fa lbounds() const
      {
        avector<BBox3fa> bboxes(positions.size());
        for (size_t t=0; t<positions.size(); t++) {
          BBox3fa b = empty;
          for (auto x : positions[t]) b.extend(x);
          bboxes[t] = b;
        }
        return LBBox3fa(bboxes);
      }
      
      virtual size_t numPrimitives() const {
        return quads.size();
      }

      size_t numVertices() const {
        assert(positions.size());
        return positions[0].size();
      }

      size_t numTimeSteps() const {
        return positions.size();
      }

      size_t numBytes() const {
        return numPrimitives()*sizeof(Quad) + numVertices()*numTimeSteps()*sizeof(Vertex);
      }
      
      void verify() const;

      virtual void print(std::ostream& cout, int depth);
      virtual void calculateStatistics(Statistics& stat);
      virtual void calculateInDegree();
      virtual void resetInDegree();
            
    public:
      BBox1f time_range;
      std::vector<avector<Vertex>> positions;
      std::vector<avector<Vertex>> normals;
      std::vector<Vec2f> texcoords;
      std::vector<Quad> quads;
      Ref<MaterialNode> material;
    };

    /*! Subdivision Mesh. */
    struct SubdivMeshNode : public Node
    {
      typedef Vec3fa Vertex;

      SubdivMeshNode (Ref<MaterialNode> material, const BBox1f time_range = BBox1f(0,1), size_t numTimeSteps = 0) 
        : Node(true),
          time_range(time_range),
          position_subdiv_mode(RTC_SUBDIVISION_MODE_SMOOTH_BOUNDARY), 
          normal_subdiv_mode(RTC_SUBDIVISION_MODE_SMOOTH_BOUNDARY),
          texcoord_subdiv_mode(RTC_SUBDIVISION_MODE_SMOOTH_BOUNDARY),
          material(material), tessellationRate(2.0f) 
      {
        for (size_t i=0; i<numTimeSteps; i++)
          positions.push_back(avector<Vertex>());
        zero_pad_arrays();
      }

      SubdivMeshNode (Ref<SceneGraph::SubdivMeshNode> imesh, const Transformations& spaces)
        : Node(true),
        time_range(imesh->time_range),
        positions(transformMSMBlurVec3faBuffer(imesh->positions,spaces)),
        normals(transformMSMBlurNormalBuffer(imesh->normals,spaces)),
        texcoords(imesh->texcoords),
        position_indices(imesh->position_indices),
        normal_indices(imesh->normal_indices),
        texcoord_indices(imesh->texcoord_indices),
        position_subdiv_mode(imesh->position_subdiv_mode), 
        normal_subdiv_mode(imesh->normal_subdiv_mode),
        texcoord_subdiv_mode(imesh->texcoord_subdiv_mode),
        verticesPerFace(imesh->verticesPerFace),
        holes(imesh->holes),
        edge_creases(imesh->edge_creases),
        edge_crease_weights(imesh->edge_crease_weights),
        vertex_creases(imesh->vertex_creases),
        vertex_crease_weights(imesh->vertex_crease_weights),
        material(imesh->material), 
        tessellationRate(imesh->tessellationRate)
      {
        zero_pad_arrays();
      }

      void zero_pad_arrays()
      {
        if (texcoords.size()) { // zero pad to 16 bytes
          texcoords.reserve(texcoords.size()+1);
          texcoords.data()[texcoords.size()] = zero;
        }
      }
      
      virtual void setMaterial(Ref<MaterialNode> material) {
        this->material = material;
      }

      virtual BBox3fa bounds() const
      {
        BBox3fa b = empty;
        for (const auto& p : positions)
          for (auto x : p)
            b.extend(x);
        return b;
      }

      virtual LBBox3fa lbounds() const
      {
        avector<BBox3fa> bboxes(positions.size());
        for (size_t t=0; t<positions.size(); t++) {
          BBox3fa b = empty;
          for (auto x : positions[t]) b.extend(x);
          bboxes[t] = b;
        }
        return LBBox3fa(bboxes);
      }

      virtual size_t numPrimitives() const {
        return verticesPerFace.size();
      }

      size_t numPositions() const {
        assert(positions.size());
        return positions[0].size();
      }

      size_t numNormals() const {
        if (normals.size()) return normals[0].size();
        else return 0;
      }

      size_t numEdges() const {
        return position_indices.size();
      }

      size_t numTimeSteps() const {
        return positions.size();
      }

      size_t numBytes() const {
        return numPrimitives()*sizeof(unsigned) + numEdges()*sizeof(unsigned) + numPositions()*numTimeSteps()*sizeof(Vertex);
      }
      
      void verify() const;

      virtual void print(std::ostream& cout, int depth);
      virtual void calculateStatistics(Statistics& stat);
      virtual void calculateInDegree();
      virtual void resetInDegree();

    public:
      BBox1f time_range;                      //!< geometry time range for motion blur
      std::vector<avector<Vertex>> positions; //!< vertex positions for multiple timesteps
      std::vector<avector<Vertex>> normals;    //!< vertex normals
      std::vector<Vec2f> texcoords;             //!< face texture coordinates
      std::vector<unsigned> position_indices;        //!< position indices for all faces
      std::vector<unsigned> normal_indices;          //!< normal indices for all faces
      std::vector<unsigned> texcoord_indices;        //!< texcoord indices for all faces
      RTCSubdivisionMode position_subdiv_mode;  
      RTCSubdivisionMode normal_subdiv_mode;
      RTCSubdivisionMode texcoord_subdiv_mode;
      std::vector<unsigned> verticesPerFace;         //!< number of indices of each face
      std::vector<unsigned> holes;                   //!< face ID of holes
      std::vector<Vec2i> edge_creases;          //!< index pairs for edge crease 
      std::vector<float> edge_crease_weights;   //!< weight for each edge crease
      std::vector<unsigned> vertex_creases;          //!< indices of vertex creases
      std::vector<float> vertex_crease_weights; //!< weight for each vertex crease
      Ref<MaterialNode> material;
      float tessellationRate;
    };

    /*! Hair Set. */
    struct HairSetNode : public Node
    {
      typedef Vec3ff Vertex;

      struct Hair
      {
      public:
        Hair () {}
        Hair (unsigned vertex, unsigned id) 
        : vertex(vertex), id(id) {}
      public:
        unsigned vertex, id;  //!< index of first control point and hair ID
      };
      
    public:
      HairSetNode (RTCGeometryType type, Ref<MaterialNode> material, const BBox1f time_range = BBox1f(0,1), size_t numTimeSteps = 0)
        : Node(true), time_range(time_range), type(type), material(material), tessellation_rate(4)
      {
        for (size_t i=0; i<numTimeSteps; i++)
          positions.push_back(avector<Vertex>());
      }

      HairSetNode (const avector<Vertex>& positions_in, const std::vector<Hair>& hairs, Ref<MaterialNode> material, RTCGeometryType type)
        : Node(true), time_range(0.0f,1.0f), type(type), hairs(hairs), material(material), tessellation_rate(4)
      {
        positions.push_back(positions_in);
      }
   
      HairSetNode (Ref<SceneGraph::HairSetNode> imesh, const Transformations& spaces)
        : Node(true),
        time_range(imesh->time_range),
        type(imesh->type),
        positions(transformMSMBlurBuffer(imesh->positions,spaces)),
        normals(transformMSMBlurNormalBuffer(imesh->normals,spaces)),
        tangents(transformMSMBlurVectorBuffer(imesh->tangents,spaces)),
        dnormals(transformMSMBlurVectorVec3faBuffer(imesh->dnormals,spaces)),
        hairs(imesh->hairs), flags(imesh->flags), material(imesh->material), tessellation_rate(imesh->tessellation_rate) {}

      virtual void setMaterial(Ref<MaterialNode> material) {
        this->material = material;
      }

      virtual BBox3fa bounds() const
      {
        BBox3fa b = empty;
        for (const auto& p : positions)
          for (auto x : p)
            b.extend(x);
        return b;
      }

      virtual LBBox3fa lbounds() const
      {
        avector<BBox3fa> bboxes(positions.size());
        for (size_t t=0; t<positions.size(); t++) {
          BBox3fa b = empty;
          for (auto x : positions[t]) b.extend(x);
          bboxes[t] = b;
        }
        return LBBox3fa(bboxes);
      }

      virtual size_t numPrimitives() const {
        return hairs.size();
      }

      size_t numVertices() const {
        assert(positions.size());
        return positions[0].size();
      }

      size_t numTimeSteps() const {
        return positions.size();
      }

      size_t numBytes() const {
        return numPrimitives()*sizeof(Hair) + numVertices()*numTimeSteps()*sizeof(Vertex);
      }

      void convert_bezier_to_bspline();
      void convert_bspline_to_bezier();
      void convert_bezier_to_hermite();
      void compact_vertices();

      void verify() const;

      virtual void print(std::ostream& cout, int depth);
      virtual void calculateStatistics(Statistics& stat);
      virtual void calculateInDegree();
      virtual void resetInDegree();

    public:
      BBox1f time_range;                      //!< geometry time range for motion blur
      RTCGeometryType type;                   //!< type of curve
      std::vector<avector<Vertex>> positions; //!< hair control points (x,y,z,r) for multiple timesteps
      std::vector<avector<Vec3fa>> normals;   //!< hair control normals (nx,ny,nz) for multiple timesteps
      std::vector<avector<Vertex>> tangents;  //!< hair control tangents (tx,ty,tz,tr) for multiple timesteps
      std::vector<avector<Vec3fa>> dnormals;  //!< hair control normal derivatives (nx,ny,nz) for multiple timesteps
      std::vector<Hair> hairs;                //!< list of hairs
      std::vector<unsigned char> flags;       //!< left, right end cap flags

      Ref<MaterialNode> material;
      unsigned tessellation_rate;
    };

    /*! Point Set. */
    struct PointSetNode : public Node
    {
      typedef Vec3ff Vertex;

    public:
      PointSetNode (RTCGeometryType type, Ref<MaterialNode> material, const BBox1f time_range = BBox1f(0,1), size_t numTimeSteps = 0)
        : Node(true), time_range(time_range), type(type), material(material)
      {
        for (size_t i=0; i<numTimeSteps; i++)
          positions.push_back(avector<Vertex>());
      }

      PointSetNode (const avector<Vertex>& positions_in, Ref<MaterialNode> material, RTCGeometryType type)
        : Node(true), time_range(0.0f, 1.0f), type(type), material(material)
      {
        positions.push_back(positions_in);
      }

      PointSetNode (Ref<SceneGraph::PointSetNode> imesh, const Transformations& spaces)
        : Node(true), time_range(imesh->time_range), type(imesh->type), positions(transformMSMBlurBuffer(imesh->positions,spaces)),
          normals(transformMSMBlurNormalBuffer(imesh->normals,spaces)),
        material(imesh->material) {}

      virtual void setMaterial(Ref<MaterialNode> material) {
        this->material = material;
      }

      virtual BBox3fa bounds() const
      {
        BBox3fa b = empty;
        for (const auto& p : positions)
          for (auto x : p)
            b.extend(x);
        return b;
      }

      virtual LBBox3fa lbounds() const
      {
        avector<BBox3fa> bboxes(positions.size());
        for (size_t t=0; t<positions.size(); t++) {
          BBox3fa b = empty;
          for (auto x : positions[t])
            b.extend(x);
          bboxes[t] = b;
        }
        return LBBox3fa(bboxes);
      }

      virtual size_t numPrimitives() const {
        return numVertices();
      }

      size_t numVertices() const {
        assert(positions.size());
        return positions[0].size();
      }

      size_t numTimeSteps() const {
        return positions.size();
      }

      size_t numBytes() const {
        return numVertices()*numTimeSteps()*sizeof(Vertex);
      }

      void verify() const;

      virtual void print(std::ostream& cout, int depth);
      virtual void calculateStatistics(Statistics& stat);
      virtual void calculateInDegree();
      virtual void resetInDegree();

    public:
      BBox1f time_range;                      //!< geometry time range for motion blur
      RTCGeometryType type;                   //!< type of point
      std::vector<avector<Vertex>> positions; //!< point control points (x,y,z,r) for multiple timesteps
      std::vector<avector<Vec3fa>> normals;   //!< point control normals (nx,ny,nz) for multiple timesteps (oriented only)

      Ref<MaterialNode> material;
    };

    struct GridMeshNode : public Node
    {
      typedef Vec3fa Vertex;

      static const unsigned int GRID_RES_MAX = 0x7FFF;

      struct Grid
      {
      public:
        Grid() {}
        Grid (unsigned int startVtx, unsigned int lineStride, unsigned int resX_in, unsigned int resY_in)
         : startVtx(startVtx), lineStride(lineStride), resX(resX_in), resY(resY_in)
        {
          assert(resX_in <= GRID_RES_MAX);
          assert(resY_in <= GRID_RES_MAX);
        }
      public:
        unsigned int startVtx;
        unsigned int lineStride;
        unsigned short resX,resY;
      };
      
    public:
      GridMeshNode (Ref<MaterialNode> material, const BBox1f time_range = BBox1f(0,1), size_t numTimeSteps = 0) 
        : Node(true), time_range(time_range), material(material) 
      {
        for (size_t i=0; i<numTimeSteps; i++)
          positions.push_back(avector<Vertex>());
      }

      GridMeshNode (Ref<SceneGraph::GridMeshNode> imesh, const Transformations& spaces)
        : Node(true),
         time_range(imesh->time_range),
          positions(transformMSMBlurVec3faBuffer(imesh->positions,spaces)),
        grids(imesh->grids), material(imesh->material) {}
   
      virtual void setMaterial(Ref<MaterialNode> material) {
        this->material = material;
      }

      virtual BBox3fa bounds() const
      {
        BBox3fa b = empty;
        for (const auto& p : positions)
          for (auto x : p)
            b.extend(x);
        return b;
      }

      virtual LBBox3fa lbounds() const
      {
        avector<BBox3fa> bboxes(positions.size());
        for (size_t t=0; t<positions.size(); t++) {
          BBox3fa b = empty;
          for (auto x : positions[t]) b.extend(x);
          bboxes[t] = b;
        }
        return LBBox3fa(bboxes);
      }
      
      virtual size_t numPrimitives() const {
        return grids.size();
      }

      size_t numVertices() const {
        assert(positions.size());
        return positions[0].size();
      }

      size_t numTimeSteps() const {
        return positions.size();
      }

      size_t numBytes() const {
        return numPrimitives()*sizeof(Grid) + numVertices()*numTimeSteps()*sizeof(Vertex);
      }

      void verify() const;

      virtual void print(std::ostream& cout, int depth);
      virtual void calculateStatistics(Statistics& stat);
      virtual void calculateInDegree();
      virtual void resetInDegree();

    public:
      BBox1f time_range;
      std::vector<avector<Vertex>> positions; 
      std::vector<Grid> grids;
      Ref<MaterialNode> material;
    };

    
    enum InstancingMode { INSTANCING_NONE, INSTANCING_GEOMETRY, INSTANCING_GROUP, INSTANCING_FLATTENED, INSTANCING_MULTI_LEVEL };
    Ref<Node> flatten(Ref<Node> node, InstancingMode mode);
    Ref<GroupNode> flatten(Ref<GroupNode> node, InstancingMode mode);

    Statistics calculateStatistics(Ref<Node> node);

    enum CurveSubtype
    {
      ROUND_CURVE,
      FLAT_CURVE
    };

    enum PointSubtype
    {
      SPHERE,
      DISC,
      ORIENTED_DISC
    };
  }
}

#include "materials.h"

