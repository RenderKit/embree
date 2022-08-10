
#include "../rthwif_production.h"
#include "../rthwif_builder.h"

#include "../../../include/embree4/rtcore.h"

#include <vector>
#include <map>
#include <iostream>

sycl::device device;
sycl::context context;
void* dispatchGlobalsPtr = nullptr;

#if defined(__SYCL_DEVICE_ONLY__)
#define CONSTANT __attribute__((opencl_constant))
#else
#define CONSTANT
#endif

#define sycl_printf0(format, ...) {               \
    static const CONSTANT char fmt[] = format;               \
    cl::sycl::ext::oneapi::experimental::printf(fmt, __VA_ARGS__ );   \
  }

#define sycl_print_str(format) {               \
    static const CONSTANT char fmt[] = format;               \
    cl::sycl::ext::oneapi::experimental::printf(fmt);   \
  }

void sycl_print_float(float x)
{
  int i = (int)sycl::trunc(x);
  int f = (int)sycl::trunc(1000.0f*(x-i));
  sycl_printf0("%i.%i",i,f);
}

#define sycl_printf_float(str, x) \
  {                               \
    sycl_print_str(str);          \
    sycl_print_float(x);          \
    sycl_print_str("\n");         \
  }

#define sycl_printf_float3(str, v)                              \
{                                                               \
  sycl_print_str(str);                                          \
  sycl_print_float(v.x());                                      \
  sycl_print_str(" ");                                          \
  sycl_print_float(v.y());                                      \
  sycl_print_str(" ");                                          \
  sycl_print_float(v.z());                                      \
  sycl_print_str("\n");                                         \
}

struct RandomSampler {
  unsigned int s;
};

unsigned int MurmurHash3_mix(unsigned int hash, unsigned int k)
{
  const unsigned int c1 = 0xcc9e2d51;
  const unsigned int c2 = 0x1b873593;
  const unsigned int r1 = 15;
  const unsigned int r2 = 13;
  const unsigned int m = 5;
  const unsigned int n = 0xe6546b64;

  k *= c1;
  k = (k << r1) | (k >> (32 - r1));
  k *= c2;

  hash ^= k;
  hash = ((hash << r2) | (hash >> (32 - r2))) * m + n;

  return hash;
}

unsigned int MurmurHash3_finalize(unsigned int hash)
{
  hash ^= hash >> 16;
  hash *= 0x85ebca6b;
  hash ^= hash >> 13;
  hash *= 0xc2b2ae35;
  hash ^= hash >> 16;
  return hash;
}

unsigned int LCG_next(unsigned int value)
{
  const unsigned int m = 1664525;
  const unsigned int n = 1013904223;
  return value * m + n;
}

void RandomSampler_init(RandomSampler& self, int id)
{
  unsigned int hash = 0;
  hash = MurmurHash3_mix(hash, id);
  hash = MurmurHash3_finalize(hash);
  self.s = hash;
}

int RandomSampler_getInt(RandomSampler& self) {
  self.s = LCG_next(self.s); return self.s >> 1;
}

unsigned int RandomSampler_getUInt(RandomSampler& self) {
  self.s = LCG_next(self.s); return self.s;
}

float RandomSampler_getFloat(RandomSampler& self) {
  return (float)RandomSampler_getInt(self) * 4.656612873077392578125e-10f;
}

RandomSampler rng;

enum class InstancingType
{
  NONE,
  SW_INSTANCING,
  HW_INSTANCING
};

enum class TestType
{
  TRIANGLES_COMMITTED_HIT,           // triangles
  TRIANGLES_POTENTIAL_HIT,           // triangles + filter + check potential hit
  TRIANGLES_ANYHIT_SHADER_COMMIT,    // triangles + filter + commit
  TRIANGLES_ANYHIT_SHADER_REJECT,    // triangles + filter + reject
  PROCEDURALS_COMMITTED_HIT,         // procedural triangles
};

// triangles_hw_instancing: triangles + hw instancing
// triangles_sw_instancing: triangles + sw instancing
// procedural_triangles: procedural triangles + commit/reject

struct TestInput
{
  sycl::float3 org;
  sycl::float3 dir;
  float tnear;
  float tfar;
  uint32_t mask;
  uint32_t flags;
};

enum TestHitType {
  TEST_COMMITTED_HIT,
  TEST_POTENTIAL_HIT,
  TEST_MISS
};

struct TestOutput
{
  // Ray data at level 0
  sycl::float3 ray0_org;
  sycl::float3 ray0_dir;
  float ray0_tnear;
  uint32_t ray0_mask;
  uint32_t ray0_flags;
  
  // Ray data at hit bvh_level
  sycl::float3 rayN_org;
  sycl::float3 rayN_dir;
  float rayN_tnear;
  uint32_t rayN_mask;
  uint32_t rayN_flags;
  
  // Hit data
  TestHitType hit_type;
  uint32_t bvh_level;
  uint32_t hit_candidate;
  float t;
  float u;
  float v;
  bool front_face;
  uint32_t geomID;
  uint32_t primID;
  uint32_t instID;
  // FIXME: what about instanceID?
  sycl::float3 v0;
  sycl::float3 v1;
  sycl::float3 v2;

  float4x3_INTEL world_to_object;
  float4x3_INTEL object_to_world;
};

std::ostream& operator<<(std::ostream& out, const sycl::float3& v) {
    return out << "(" << v.x() << "," << v.y() << "," << v.z()  << ")";
}

void compareTestOutput(uint32_t tid, uint32_t& errors, const TestOutput& test, const TestOutput& expected)
{
#define COMPARE(member)                 \
  if (test.member != expected.member) { \
    if (errors < 16)                                                    \
      std::cout << "test" << tid << " " #member " mismatch: output " << test.member << " != expected " << expected.member << std::endl; \
    errors++;                                                           \
  }
#define COMPARE1(member,eps)               \
  if (fabs(test.member-expected.member) > eps) {                              \
    if (errors < 16)                                                    \
      std::cout << "test" << tid << " " #member " mismatch: output " << test.member << " != expected " << expected.member << std::endl; \
    errors++;                                                           \
  }
#define COMPARE3(member,eps) {                                          \
    const bool x = fabs(test.member.x()-expected.member.x()) > eps;     \
    const bool y = fabs(test.member.y()-expected.member.y()) > eps;     \
    const bool z = fabs(test.member.z()-expected.member.z()) > eps;     \
    if (x || y || z) {                                                  \
      if (errors < 16)                                                  \
        std::cout << "test" << tid << " " #member " mismatch: output " << test.member << " != expected " << expected.member << std::endl; \
      errors++;                                                         \
    }                                                                   \
  }

  float eps = 1E-5;

  COMPARE3(ray0_org,0);
  COMPARE3(ray0_dir,0);
  COMPARE1(ray0_tnear,0);
  COMPARE(ray0_mask);
  COMPARE(ray0_flags);
  COMPARE3(rayN_org,0);
  COMPARE3(rayN_dir,0);
  COMPARE1(rayN_tnear,0);
  COMPARE(rayN_mask);
  COMPARE(rayN_flags);
  COMPARE(hit_type);
  COMPARE(bvh_level);
  COMPARE(hit_candidate);
  COMPARE1(t,eps);
  COMPARE1(u,eps);
  COMPARE1(v,eps);
  COMPARE(front_face);
  COMPARE(geomID);
  COMPARE(primID);
  COMPARE(instID);
  COMPARE3(v0,eps);
  COMPARE3(v1,eps);
  COMPARE3(v2,eps);
}

struct LinearSpace3f
{
  /*! matrix construction from column vectors */
  LinearSpace3f(const sycl::float3& vx, const sycl::float3& vy, const sycl::float3& vz)
    : vx(vx), vy(vy), vz(vz) {}
  
  /*! matrix construction from row mayor data */
  LinearSpace3f(const float m00, const float m01, const float m02,
                const float m10, const float m11, const float m12,
                const float m20, const float m21, const float m22)
    : vx(m00,m10,m20), vy(m01,m11,m21), vz(m02,m12,m22) {}
  
  /*! compute the determinant of the matrix */
  const float det() const { return sycl::dot(vx,sycl::cross(vy,vz)); }
  
  /*! compute adjoint matrix */
  const LinearSpace3f adjoint() const { return LinearSpace3f(sycl::cross(vy,vz),sycl::cross(vz,vx),sycl::cross(vx,vy)).transposed(); }
  
  /*! compute inverse matrix */
  const LinearSpace3f inverse() const
  {
    const float d = det();
    const LinearSpace3f a = adjoint();
    return { a.vx/d, a.vy/d, a.vz/d };
  }

  /*! compute transposed matrix */
  const LinearSpace3f transposed() const { return LinearSpace3f(vx.x(),vx.y(),vx.z(),vy.x(),vy.y(),vy.z(),vz.x(),vz.y(),vz.z()); }
  
  sycl::float3 vx,vy,vz;
};

sycl::float3 xfmPoint (const LinearSpace3f& m, const sycl::float3& p) {
  return p.x()*m.vx + p.y()*m.vy + p.z()*m.vz;
}

struct Transform
{
  Transform ()
    : vx(1,0,0), vy(0,1,0), vz(0,0,1), p(0,0,0) {}

  Transform ( sycl::float3 vx, sycl::float3 vy, sycl::float3 vz, sycl::float3 p )
    : vx(vx), vy(vy), vz(vz), p(p) {}

  Transform ( float4x3_INTEL xfm )
    : vx(xfm.vx), vy(xfm.vy), vz(xfm.vz), p(xfm.p) {}

  operator float4x3_INTEL () {
    return { vx, vy, vz, p };
  }
  
  sycl::float3 vx,vy,vz,p;
};

std::ostream& operator<<(std::ostream& out, const Transform& t) {
  return out << " Transform {" << t.vx << ", " << t.vy << ", " << t.vz << ", " << t.p  << "}";
}

sycl::float3 xfmPoint (const Transform& m, const sycl::float3& p) {
  return p.x()*m.vx + p.y()*m.vy + p.z()*m.vz + m.p;
}

sycl::float3 xfmVector (const Transform& m, const sycl::float3& v) {
  return v.x()*m.vx + v.y()*m.vy + v.z()*m.vz;
}

Transform operator* (const Transform& a, const Transform& b) {
  return Transform(xfmVector(a,b.vx),xfmVector(a,b.vy),xfmVector(a,b.vz),xfmPoint(a,b.p));
}

Transform rcp( const Transform& a )
{
  const LinearSpace3f l = { a.vx, a.vy, a.vz };
  const LinearSpace3f il = l.inverse();
  return Transform(il.vx, il.vy, il.vz, -xfmPoint(il,a.p));
}

struct Bounds3f
{
  void extend( sycl::float3 p ) {
    lower = sycl::min(lower,p);
    upper = sycl::max(upper,p);
  }

  static Bounds3f empty() {
    return { sycl::float3(INFINITY), sycl::float3(-INFINITY) };
  }
  
  sycl::float3 lower;
  sycl::float3 upper;
};

std::ostream& operator<<(std::ostream& out, const Bounds3f& b) {
  return out << "Bounds3f {" << b.lower << "," << b.upper  << "}";
}

const Bounds3f xfmBounds(const Transform& m, const Bounds3f& b) 
{ 
  Bounds3f dst = Bounds3f::empty();
  const sycl::float3 p0(b.lower.x(),b.lower.y(),b.lower.z()); dst.extend(xfmPoint(m,p0));
  const sycl::float3 p1(b.lower.x(),b.lower.y(),b.upper.z()); dst.extend(xfmPoint(m,p1));
  const sycl::float3 p2(b.lower.x(),b.upper.y(),b.lower.z()); dst.extend(xfmPoint(m,p2));
  const sycl::float3 p3(b.lower.x(),b.upper.y(),b.upper.z()); dst.extend(xfmPoint(m,p3));
  const sycl::float3 p4(b.upper.x(),b.lower.y(),b.lower.z()); dst.extend(xfmPoint(m,p4));
  const sycl::float3 p5(b.upper.x(),b.lower.y(),b.upper.z()); dst.extend(xfmPoint(m,p5));
  const sycl::float3 p6(b.upper.x(),b.upper.y(),b.lower.z()); dst.extend(xfmPoint(m,p6));
  const sycl::float3 p7(b.upper.x(),b.upper.y(),b.upper.z()); dst.extend(xfmPoint(m,p7));
  return dst;
}

struct Triangle
{
  Triangle()
    : v0(0,0,0), v1(0,0,0), v2(0,0,0), index(0) {}
  
  Triangle (sycl::float3 v0, sycl::float3 v1, sycl::float3 v2, uint32_t index)
    : v0(v0), v1(v1), v2(v2), index(index) {}

  sycl::float3 sample(float u, float v) const {
    return (1.0f-u-v)*v0 + u*v1 + v*v2;
  }

  sycl::float3 center() const {
    return (v0+v1+v2)/3.0f;
  }

  Bounds3f bounds() const
  {
    const sycl::float3 lower = sycl::min(v0,sycl::min(v1,v2));
    const sycl::float3 upper = sycl::max(v0,sycl::max(v1,v2));
    return { lower, upper };
  }

  const Triangle transform( Transform xfm ) const {
    return Triangle(xfmPoint(xfm,v0), xfmPoint(xfm,v1), xfmPoint(xfm,v2), index);
  }

  sycl::float3 v0;
  sycl::float3 v1;
  sycl::float3 v2;
  uint32_t index;
};

struct less_float3 {
  bool operator() ( const sycl::float3& a, const sycl::float3& b ) const {
    if (a.x() != b.x()) return a.x()  < b.x();
    if (a.y() != b.y()) return a.y()  < b.y();
    if (a.z() != b.z()) return a.z()  < b.z();
    return false;
  }
};

std::ostream& operator<<(std::ostream& out, const Triangle& tri) {
  return out << "Triangle {" << tri.v0 << "," << tri.v1 << "," << tri.v2  << "}";
}

struct Hit
{
  Transform local_to_world;
  Triangle triangle;
  uint32_t instID = -1;
  uint32_t geomID = -1;
  uint32_t primID = -1;
};

struct Geometry
{
  enum Type {
    TRIANGLE_MESH,
    INSTANCE
  };

  Geometry (Type type)
    : type(type) {}

  virtual RTHWIF_GEOMETRY_DESC getDesc() = 0;

  virtual void buildAccel(RTCDevice rtcdevice, sycl::device& device, sycl::context& context) {
  };

  virtual void buildTriMap(Transform local_to_world, std::vector<uint32_t> id_stack, std::vector<Hit>& tri_map) = 0;

  Type type;
};

struct TriangleMesh : public Geometry
{
public:

  TriangleMesh (RTHWIF_GEOMETRY_FLAGS gflags = RTHWIF_GEOMETRY_FLAG_OPAQUE, bool procedural = false)
    : Geometry(Type::TRIANGLE_MESH),
      gflags(gflags), procedural(procedural),
      triangles_alloc(context,device), triangles(0,triangles_alloc),
      vertices_alloc(context,device), vertices(0,vertices_alloc) {}

  virtual ~TriangleMesh() {}

  void* operator new(size_t size) {
    return sycl::aligned_alloc(64,size,device,context,sycl::usm::alloc::shared);
  }
  void operator delete(void* ptr) {
    sycl::free(ptr,context);
  }

  size_t size() const {
    return triangles.size();
  }

  static RTHWIF_AABB getBoundsCallback (const uint32_t primID, void* geomUserPtr, void* userPtr)
  {
    const TriangleMesh* mesh = (TriangleMesh*) geomUserPtr;
    const Bounds3f bounds = mesh->getBounds(primID);
    
    RTHWIF_AABB r;
    r.lower.x = bounds.lower.x();
    r.lower.y = bounds.lower.y();
    r.lower.z = bounds.lower.z();
    r.upper.x = bounds.upper.x();
    r.upper.y = bounds.upper.y();
    r.upper.z = bounds.upper.z();
    return r;
  }
  
  virtual RTHWIF_GEOMETRY_DESC getDesc() override
  {
    if (procedural)
    {
      RTHWIF_GEOMETRY_AABBS_DESC out;
      memset(&out,0,sizeof(out));
      out.GeometryType = RTHWIF_GEOMETRY_TYPE_PROCEDURALS;
      out.GeometryFlags = gflags;
      out.GeometryMask = 0xFF;
      out.AABBCount = triangles.size();
      out.AABBs = TriangleMesh::getBoundsCallback;
      out.userPtr = this;

      RTHWIF_GEOMETRY_DESC desc;
      desc.AABBs = out;
      return desc;
    }
    else
    {
      RTHWIF_GEOMETRY_TRIANGLES_DESC out;
      memset(&out,0,sizeof(out));
      out.GeometryType = RTHWIF_GEOMETRY_TYPE_TRIANGLES;
      out.GeometryFlags = gflags;
      out.GeometryMask = 0xFF;
      out.IndexBuffer = (RTHWIF_UINT3*) triangles.data();
      out.TriangleCount = triangles.size();
      out.TriangleStride = sizeof(sycl::int3);
      out.VertexBuffer = (RTHWIF_FLOAT3*) vertices.data();
      out.VertexCount = vertices.size();
      out.VertexStride = sizeof(sycl::float3);
      
      RTHWIF_GEOMETRY_DESC desc;
      desc.Triangles = out;
      return desc;
    }
  }

  Triangle getTriangle( const uint32_t primID ) const
  {
    const sycl::float3 v0 = vertices[triangles[primID].x()];
    const sycl::float3 v1 = vertices[triangles[primID].y()];
    const sycl::float3 v2 = vertices[triangles[primID].z()];
    const uint32_t index = indices[primID];
    return Triangle(v0,v1,v2,index);
  }

  Bounds3f getBounds( const uint32_t primID ) const {
    return getTriangle(primID).bounds();
  }

  uint32_t addVertex( const sycl::float3& v )
  {
    auto e = vertex_map.find(v);
    if (e != vertex_map.end()) return e->second;
    vertices.push_back(v);
    vertex_map[v] = vertices.size()-1;
    return vertices.size()-1;
  }

  void addTriangle( const Triangle& tri )
  {
    const uint32_t v0 = addVertex(tri.v0);
    const uint32_t v1 = addVertex(tri.v1);
    const uint32_t v2 = addVertex(tri.v2);
    triangles.push_back(sycl::int3(v0,v1,v2));
    indices.push_back(tri.index);
  }

  void split(const sycl::float3 P, const sycl::float3 N, std::shared_ptr<TriangleMesh>& mesh0, std::shared_ptr<TriangleMesh>& mesh1)
  {
    mesh0 = std::shared_ptr<TriangleMesh>(new TriangleMesh(gflags,procedural));
    mesh1 = std::shared_ptr<TriangleMesh>(new TriangleMesh(gflags,procedural));
    
    for (uint32_t primID=0; primID<(uint32_t) size(); primID++)
    {
      const Triangle tri = getTriangle(primID);
      if (sycl::dot(tri.center()-P,N) < 0.0f) mesh0->addTriangle(tri);
      else                                    mesh1->addTriangle(tri);
    }
  }

  virtual void buildTriMap(Transform local_to_world, std::vector<uint32_t> id_stack, std::vector<Hit>& tri_map) override
  {
    uint32_t instID = -1;
    uint32_t geomID = -1;
    
    if (id_stack.size()) {
      geomID = id_stack.back();
      id_stack.pop_back();
    }
    
    if (id_stack.size()) {
      instID = id_stack.back();
      id_stack.pop_back();
    }
    assert(id_stack.size() == 0);
    
    for (uint32_t primID=0; primID<triangles.size(); primID++)
    {
      const Triangle tri = getTriangle(primID);
      const Triangle tri1 = tri.transform(local_to_world);
      assert(tri_map[tri.index].geomID == -1);
      tri_map[tri.index].primID = primID;
      tri_map[tri.index].geomID = geomID;
      tri_map[tri.index].instID = instID;
      tri_map[tri.index].triangle = tri1;
      tri_map[tri.index].local_to_world = local_to_world;
    }
  }
  
public:
  RTHWIF_GEOMETRY_FLAGS gflags = RTHWIF_GEOMETRY_FLAG_OPAQUE;
  bool procedural = false;
  
  std::vector<uint32_t> indices;

  typedef sycl::usm_allocator<sycl::int3, sycl::usm::alloc::shared> triangles_alloc_ty;
  triangles_alloc_ty triangles_alloc;
  std::vector<sycl::int3, triangles_alloc_ty> triangles;

  typedef sycl::usm_allocator<sycl::float3, sycl::usm::alloc::shared> vertices_alloc_ty;
  vertices_alloc_ty vertices_alloc;
  std::vector<sycl::float3, vertices_alloc_ty> vertices;
  
  std::map<sycl::float3,uint32_t,less_float3> vertex_map;
};

template<typename Scene>
struct InstanceGeometryT : public Geometry
{
  InstanceGeometryT(Transform& local2world, std::shared_ptr<Scene> scene, bool procedural)
    : Geometry(Type::INSTANCE), procedural(procedural), local2world(local2world), scene(scene) {}

  virtual ~InstanceGeometryT() {}

  void* operator new(size_t size) {
    return sycl::aligned_alloc(64,size,device,context,sycl::usm::alloc::shared);
  }
  void operator delete(void* ptr) {
    sycl::free(ptr,context);
  }

  static RTHWIF_AABB getBoundsCallback (const uint32_t primID, void* geomUserPtr, void* userPtr)
  {
    const InstanceGeometryT* inst = (InstanceGeometryT*) geomUserPtr;
    const Bounds3f scene_bounds = inst->scene->getBounds();
    const Bounds3f bounds = xfmBounds(inst->local2world, scene_bounds);
    
    RTHWIF_AABB r;
    r.lower.x = bounds.lower.x();
    r.lower.y = bounds.lower.y();
    r.lower.z = bounds.lower.z();
    r.upper.x = bounds.upper.x();
    r.upper.y = bounds.upper.y();
    r.upper.z = bounds.upper.z();
    return r;
  }

  virtual RTHWIF_GEOMETRY_DESC getDesc() override
  {
    if (procedural)
    {
      RTHWIF_GEOMETRY_AABBS_DESC out;
      memset(&out,0,sizeof(out));
      out.GeometryType = RTHWIF_GEOMETRY_TYPE_PROCEDURALS;
      out.GeometryFlags = RTHWIF_GEOMETRY_FLAG_NONE;
      out.GeometryMask = 0xFF;
      out.AABBCount = 1;
      out.AABBs = InstanceGeometryT::getBoundsCallback;
      out.userPtr = this;

      RTHWIF_GEOMETRY_DESC desc;
      desc.AABBs = out;
      return desc;
    }
    else
    {
      RTHWIF_GEOMETRY_INSTANCE_DESC out;
      memset(&out,0,sizeof(out));
      out.GeometryType = RTHWIF_GEOMETRY_TYPE_INSTANCES;
      out.InstanceFlags = RTHWIF_INSTANCE_FLAG_NONE;
      out.GeometryMask = 0xFF;
      out.InstanceID = 0;
      out.Transform.vx.x = local2world.vx.x();
      out.Transform.vx.y = local2world.vx.y();
      out.Transform.vx.z = local2world.vx.z();
      out.Transform.vy.x = local2world.vy.x();
      out.Transform.vy.y = local2world.vy.y();
      out.Transform.vy.z = local2world.vy.z();
      out.Transform.vz.x = local2world.vz.x();
      out.Transform.vz.y = local2world.vz.y();
      out.Transform.vz.z = local2world.vz.z();
      out.Transform.p.x  = local2world.p.x();
      out.Transform.p.y  = local2world.p.y();
      out.Transform.p.z  = local2world.p.z();
      out.Accel = scene->getAccel();
      
      RTHWIF_GEOMETRY_DESC desc;
      desc.Instances = out;
      return desc;
    }
  }

  virtual void buildAccel(RTCDevice rtcdevice, sycl::device& device, sycl::context& context) override {
    scene->buildAccel(rtcdevice,device,context);
  }

  virtual void buildTriMap(Transform local_to_world_in, std::vector<uint32_t> id_stack, std::vector<Hit>& tri_map) override {
    if (procedural) id_stack.back() = -1;
    scene->buildTriMap(local_to_world_in * local2world, id_stack, tri_map);
  }

  bool procedural;
  Transform local2world;
  std::shared_ptr<Scene> scene;
};

std::shared_ptr<TriangleMesh> createTrianglePlane (const sycl::float3& p0, const sycl::float3& dx, const sycl::float3& dy, size_t width, size_t height)
{
  std::shared_ptr<TriangleMesh> mesh(new TriangleMesh);
  mesh->indices.resize(2*width*height);
  mesh->triangles.resize(2*width*height);
  mesh->vertices.resize((width+1)*(height+1));
  
  for (size_t y=0; y<=height; y++) {
    for (size_t x=0; x<=width; x++) {
      sycl::float3 p = p0+float(x)/float(width)*dx+float(y)/float(height)*dy;
      size_t i = y*(width+1)+x;
      mesh->vertices[i] = p;
    }
  }
  for (size_t y=0; y<height; y++) {
    for (size_t x=0; x<width; x++) {
      size_t i = 2*y*width+2*x;
      size_t p00 = (y+0)*(width+1)+(x+0);
      size_t p01 = (y+0)*(width+1)+(x+1);
      size_t p10 = (y+1)*(width+1)+(x+0);
      size_t p11 = (y+1)*(width+1)+(x+1);
      mesh->triangles[i+0] = sycl::int3((int)p00,(int)p01,(int)p10);
      mesh->triangles[i+1] = sycl::int3((int)p11,(int)p10,(int)p01);
      mesh->indices[i+0] = i+0;
      mesh->indices[i+1] = i+1;
    }
  }
  return mesh;
}

struct Scene
{
  typedef InstanceGeometryT<Scene> InstanceGeometry;
  
  Scene()
    : geometries_alloc(context,device), geometries(0,geometries_alloc), bounds(Bounds3f::empty()), accel(nullptr) {}
      
  Scene(uint32_t width, uint32_t height, bool opaque, bool procedural)
    : geometries_alloc(context,device), geometries(0,geometries_alloc), bounds(Bounds3f::empty()), accel(nullptr) 
  {
    std::shared_ptr<TriangleMesh> plane = createTrianglePlane(sycl::float3(0,0,0), sycl::float3(width,0,0), sycl::float3(0,height,0), width, height);
    plane->gflags = opaque ? RTHWIF_GEOMETRY_FLAG_OPAQUE : RTHWIF_GEOMETRY_FLAG_NONE;
    plane->procedural = procedural;
    geometries.push_back(plane);
  }

  void* operator new(size_t size) {
    return sycl::aligned_alloc(64,size,device,context,sycl::usm::alloc::shared);
  }

  void operator delete(void* ptr) {
    sycl::free(ptr,context);
  }

  void splitIntoGeometries(uint32_t numGeometries)
  {
    for (uint32_t i=0; i<numGeometries-1; i++)
    {
      if (std::shared_ptr<TriangleMesh> mesh = std::dynamic_pointer_cast<TriangleMesh>(geometries[i]))
      {
        const Triangle tri = mesh->getTriangle(RandomSampler_getUInt(rng)%mesh->size());
        const float u = 2.0f*M_PI*RandomSampler_getFloat(rng);
        const sycl::float3 P = tri.center();
        const sycl::float3 N(cosf(u),sinf(u),0.0f);
        
        std::shared_ptr<TriangleMesh> mesh0, mesh1;
        mesh->split(P,N,mesh0,mesh1);
        geometries[i] = std::dynamic_pointer_cast<Geometry>(mesh0);
        geometries.push_back(std::dynamic_pointer_cast<Geometry>(mesh1));
      }
    }
    assert(geometries.size() == (size_t) numGeometries);
  }

  void createInstances(uint32_t blockSize, bool procedural)
  {
    std::vector<std::shared_ptr<Geometry>, geometries_alloc_ty> instances(0,geometries_alloc);
    
    for (uint32_t i=0; i<geometries.size(); i+=blockSize)
    {
      const uint32_t begin = i;
      const uint32_t end   = std::min((uint32_t)geometries.size(),i+blockSize);

      std::shared_ptr<Scene> scene(new Scene);
      for (size_t j=begin; j<end; j++)
        scene->geometries.push_back(geometries[j]);

      Transform local2world;
      local2world.vx = sycl::float3(1,0,0);
      local2world.vy = sycl::float3(0,1,0);
      local2world.vz = sycl::float3(0,0,1);
      local2world.p  = sycl::float3(0,0,0);
      
      //std::shared_ptr<InstanceGeometry> instance = std::make_shared<InstanceGeometry>(local2world,scene,procedural);
      std::shared_ptr<InstanceGeometry> instance(new InstanceGeometry(local2world,scene,procedural));
      instances.push_back(instance);
    }

    geometries = instances;
  }

  void buildAccel(RTCDevice rtcdevice, sycl::device& device, sycl::context& context)
  {
    /* fill geometry descriptor buffer */
    std::vector<RTHWIF_GEOMETRY_DESC> desc(size());
    std::vector<const RTHWIF_GEOMETRY_DESC*> geom(size());
    for (size_t geomID=0; geomID<size(); geomID++)
    {
      geometries[geomID]->buildAccel(rtcdevice,device,context);
      desc[geomID] = geometries[geomID]->getDesc();
      geom[geomID] = (const RTHWIF_GEOMETRY_DESC*) &desc[geomID];
    }

    /* estimate accel size */
    RTHWIF_AABB bounds;
    RTHWIF_BUILD_ACCEL_ARGS args;
    memset(&args,0,sizeof(args));
    args.bytes = sizeof(args);
    args.device = nullptr;
    args.embree_device = (void*) rtcdevice;
    args.dispatchGlobalsPtr = dispatchGlobalsPtr;
    args.geometries = (const RTHWIF_GEOMETRY_DESC**) geom.data();
    args.numGeometries = geom.size();
    args.accel = nullptr;
    args.numBytes = 0;
    args.quality = RTHWIF_BUILD_QUALITY_MEDIUM;
    args.flags = RTHWIF_BUILD_FLAG_NONE;
    args.bounds = &bounds;
    args.userPtr = nullptr;
    
    RTHWIF_ACCEL_SIZE size;
    memset(&size,0,sizeof(RTHWIF_ACCEL_SIZE));
    size.bytes = sizeof(RTHWIF_ACCEL_SIZE);
    RTHWIF_ERROR err = rthwifGetAccelSize(args,size);
    if (err != RTHWIF_ERROR_NONE)
      throw std::runtime_error("BVH size estimate failed");
    
    accel = nullptr;
    for (size_t bytes = size.expectedBytes; bytes < size.worstCaseBytes; bytes*=1.2)
    {
      /* allocate BVH data */
      if (accel) sycl::free(accel,context);
      accel = sycl::aligned_alloc(RTHWIF_BVH_ALIGNMENT,bytes,device,context,sycl::usm::alloc::shared);
      memset(accel,0,bytes); // FIXME: not required
      
      /* build accel */
      args.numGeometries = geom.size();
      args.accel = accel;
      args.numBytes = bytes;
      err = rthwifBuildAccel(args);
      
      if (err != RTHWIF_ERROR_OUT_OF_MEMORY)
        break;
    }
    
    if (err != RTHWIF_ERROR_NONE)
      throw std::runtime_error("build error");

    this->bounds.lower.x() = bounds.lower.x;
    this->bounds.lower.y() = bounds.lower.y;
    this->bounds.lower.z() = bounds.lower.z;
    this->bounds.upper.x() = bounds.upper.x;
    this->bounds.upper.y() = bounds.upper.y;
    this->bounds.upper.z() = bounds.upper.z;
  }
  
  void buildTriMap(Transform local_to_world, std::vector<uint32_t> id_stack, std::vector<Hit>& tri_map)
  {    
    for (uint32_t geomID=0; geomID<geometries.size(); geomID++)
    {
      id_stack.push_back(geomID);
      geometries[geomID]->buildTriMap(local_to_world,id_stack,tri_map);
      id_stack.pop_back();
    }
  }
  
  size_t size() const {
    return geometries.size();
  }

  Bounds3f getBounds() {
    return bounds;
  }
  
  void* getAccel() {
    return accel;
  }

  std::shared_ptr<Geometry> operator[] ( size_t i ) { return geometries[i]; }

  typedef sycl::usm_allocator<std::shared_ptr<Geometry>, sycl::usm::alloc::shared> geometries_alloc_ty;
  geometries_alloc_ty geometries_alloc;
  std::vector<std::shared_ptr<Geometry>, geometries_alloc_ty> geometries;

  Bounds3f bounds;
  void* accel;
};

void exception_handler(sycl::exception_list exceptions)
{
  for (std::exception_ptr const& e : exceptions) {
    try {
      std::rethrow_exception(e);
    } catch(sycl::exception const& e) {
      std::cout << "Caught asynchronous SYCL exception: " << e.what() << std::endl;
    }
  }
};

void render(uint32_t i, const TestInput& in, TestOutput& out, rtas_t* accel)
{
  /* setup ray */
  RayDescINTEL ray;
  ray.O = in.org;
  ray.D = in.dir;
  ray.tmin = in.tnear;
  ray.tmax = in.tfar;
  ray.mask = in.mask;
  ray.flags = in.flags;
  
  /* trace ray */
  rayquery_t query = intel_ray_query_init(0,ray,accel,0);
  intel_ray_query_start_traversal(query);
  intel_sync_ray_query(query);
  
  /* return ray data of level 0 */
  out.ray0_org = intel_get_ray_origin(query,0);
  out.ray0_dir = intel_get_ray_direction(query,0);
  out.ray0_tnear = intel_get_ray_tnear(query,0);
  out.ray0_mask = intel_get_ray_mask(query,0);
  out.ray0_flags = intel_get_ray_flags(query,0);
  
  /* clear ray data of level N */
  out.rayN_org = sycl::float3(0,0,0);
  out.rayN_dir = sycl::float3(0,0,0);
  out.rayN_tnear = 0.0f;
  out.rayN_mask = 0;
  out.rayN_flags = 0;

  /* potential hit */
  if (!intel_is_traversal_done(query))
  {
    out.hit_type = TEST_POTENTIAL_HIT;
    out.bvh_level = intel_get_hit_bvh_level( query, POTENTIAL_HIT );
    out.hit_candidate = intel_get_hit_candidate( query, POTENTIAL_HIT );
    out.t = intel_get_hit_distance(query, POTENTIAL_HIT);
    out.u = intel_get_hit_barys(query, POTENTIAL_HIT).x();
    out.v = intel_get_hit_barys(query, POTENTIAL_HIT).y();
    out.front_face = intel_hit_is_front_face( query, POTENTIAL_HIT );
    out.instID = intel_get_hit_instanceID( query, POTENTIAL_HIT );
    out.geomID = intel_get_hit_geomID( query, POTENTIAL_HIT );
    if (i%2) out.primID = intel_get_hit_primID_triangle( query, POTENTIAL_HIT );
    else     out.primID = intel_get_hit_primID         ( query, POTENTIAL_HIT );
    sycl::float3 vertex_out[3];
    intel_get_hit_triangle_verts(query, vertex_out, POTENTIAL_HIT);
    out.v0 = vertex_out[0];
    out.v1 = vertex_out[1];
    out.v2 = vertex_out[2];

    /* return ray data at current level */
    uint32_t bvh_level = intel_get_hit_bvh_level( query, POTENTIAL_HIT );
    out.rayN_org = intel_get_ray_origin(query,bvh_level);
    out.rayN_dir = intel_get_ray_direction(query,bvh_level);
    out.rayN_tnear = intel_get_ray_tnear(query,bvh_level);
    out.rayN_mask = intel_get_ray_mask(query,bvh_level);
    out.rayN_flags = intel_get_ray_flags(query,bvh_level);

    /* return instance transformations */
    out.world_to_object = intel_get_hit_world_to_object(query,POTENTIAL_HIT);
    out.object_to_world = intel_get_hit_object_to_world(query,POTENTIAL_HIT);
  }

  /* committed hit */
  else if (intel_has_committed_hit(query))
  {
    out.hit_type = TEST_COMMITTED_HIT;
    out.bvh_level = intel_get_hit_bvh_level( query, COMMITTED_HIT );
    out.hit_candidate = intel_get_hit_candidate( query, COMMITTED_HIT );
    out.t = intel_get_hit_distance(query, COMMITTED_HIT);
    out.u = intel_get_hit_barys(query, COMMITTED_HIT).x();
    out.v = intel_get_hit_barys(query, COMMITTED_HIT).y();
    out.front_face = intel_hit_is_front_face( query, COMMITTED_HIT );
    out.instID = intel_get_hit_instanceID( query, COMMITTED_HIT );
    out.geomID = intel_get_hit_geomID( query, COMMITTED_HIT );
    if (i%2) out.primID = intel_get_hit_primID_triangle( query, COMMITTED_HIT );
    else     out.primID = intel_get_hit_primID         ( query, COMMITTED_HIT );
    sycl::float3 vertex_out[3];
    intel_get_hit_triangle_verts(query, vertex_out, COMMITTED_HIT);
    out.v0 = vertex_out[0];
    out.v1 = vertex_out[1];
    out.v2 = vertex_out[2];

    /* return instance transformations */
    out.world_to_object = intel_get_hit_world_to_object(query,COMMITTED_HIT);
    out.object_to_world = intel_get_hit_object_to_world(query,COMMITTED_HIT);
  }

  /* miss */
  else {
    out.hit_type = TEST_MISS;
  }
}

void render_loop(uint32_t i, const TestInput& in, TestOutput& out, size_t scene_in, rtas_t* accel, TestType test)
{
  /* setup ray */
  RayDescINTEL ray;
  ray.O = in.org;
  ray.D = in.dir;
  ray.tmin = in.tnear;
  ray.tmax = in.tfar;
  ray.mask = in.mask;
  ray.flags = in.flags;
  
  /* trace ray */
  rayquery_t query = intel_ray_query_init(0,ray,accel,0);
  intel_ray_query_start_traversal(query);
  intel_sync_ray_query(query);
  
  /* return ray data of level 0 */
  out.ray0_org = intel_get_ray_origin(query,0);
  out.ray0_dir = intel_get_ray_direction(query,0);
  out.ray0_tnear = intel_get_ray_tnear(query,0);
  out.ray0_mask = intel_get_ray_mask(query,0);
  out.ray0_flags = intel_get_ray_flags(query,0);
  
  /* clear ray data of level N */
  out.rayN_org = sycl::float3(0,0,0);
  out.rayN_dir = sycl::float3(0,0,0);
  out.rayN_tnear = 0.0f;
  out.rayN_mask = 0;
  out.rayN_flags = 0;

  Scene* scenes[2];
  scenes[0] = (Scene*) scene_in;
  scenes[1] = nullptr;

  /* traversal loop */
  while (!intel_is_traversal_done(query))
  {
    const CandidateType candidate = intel_get_hit_candidate(query, POTENTIAL_HIT);

    if (candidate == TRIANGLE)
    {
      if (test == TestType::TRIANGLES_POTENTIAL_HIT)
      {
        out.hit_type = TEST_POTENTIAL_HIT;
        out.bvh_level = intel_get_hit_bvh_level( query, POTENTIAL_HIT );
        out.hit_candidate = intel_get_hit_candidate( query, POTENTIAL_HIT );
        out.t = intel_get_hit_distance(query, POTENTIAL_HIT);
        out.u = intel_get_hit_barys(query, POTENTIAL_HIT).x();
        out.v = intel_get_hit_barys(query, POTENTIAL_HIT).y();
        out.front_face = intel_hit_is_front_face( query, POTENTIAL_HIT );
        out.instID = intel_get_hit_instanceID( query, POTENTIAL_HIT );
        out.geomID = intel_get_hit_geomID( query, POTENTIAL_HIT );
        if (i%2) out.primID = intel_get_hit_primID_triangle( query, POTENTIAL_HIT );
        else     out.primID = intel_get_hit_primID         ( query, POTENTIAL_HIT );
        sycl::float3 vertex_out[3];
        intel_get_hit_triangle_verts(query, vertex_out, POTENTIAL_HIT);
        out.v0 = vertex_out[0];
        out.v1 = vertex_out[1];
        out.v2 = vertex_out[2];
        
        /* return ray data at current level */
        uint32_t bvh_level = intel_get_hit_bvh_level( query, POTENTIAL_HIT );
        out.rayN_org = intel_get_ray_origin(query,bvh_level);
        out.rayN_dir = intel_get_ray_direction(query,bvh_level);
        out.rayN_tnear = intel_get_ray_tnear(query,bvh_level);
        out.rayN_mask = intel_get_ray_mask(query,bvh_level);
        out.rayN_flags = intel_get_ray_flags(query,bvh_level);
        return;
      }
    
      if (test == TestType::TRIANGLES_ANYHIT_SHADER_COMMIT)
        intel_ray_query_commit_potential_hit(query);
    }

    else if (candidate == PROCEDURAL)
    {
      const uint32_t bvh_level = intel_get_hit_bvh_level( query, POTENTIAL_HIT );
      
      const uint32_t instID = intel_get_hit_instanceID( query, POTENTIAL_HIT );
      const uint32_t geomID = intel_get_hit_geomID( query, POTENTIAL_HIT );
      const uint32_t primID = intel_get_hit_primID( query, POTENTIAL_HIT );

      Geometry* geom = nullptr;
      if (instID != -1) {
        Scene::InstanceGeometry* instance = (Scene::InstanceGeometry*) scenes[0]->geometries[instID].get();
        geom = instance->scene->geometries[geomID].get();
      } else {
        geom = scenes[bvh_level]->geometries[geomID].get();
      }

      if (geom->type == Geometry::TRIANGLE_MESH)
      {
        const TriangleMesh* mesh = (TriangleMesh*) geom;

        const sycl::int3 tri = mesh->triangles[primID];
        const sycl::float3 tri_v0 = mesh->vertices[tri.x()];
        const sycl::float3 tri_v1 = mesh->vertices[tri.y()];
        const sycl::float3 tri_v2 = mesh->vertices[tri.z()];

        /* calculate vertices relative to ray origin */
        const sycl::float3 O = intel_get_ray_origin(query,bvh_level);
        const sycl::float3 D = intel_get_ray_direction(query,bvh_level);
        const float tnear = intel_get_ray_tnear(query,bvh_level);
        const float tfar = intel_get_hit_distance(query, COMMITTED_HIT);
        const sycl::float3 v0 = tri_v0-O;
        const sycl::float3 v1 = tri_v1-O;
        const sycl::float3 v2 = tri_v2-O;
        
        /* calculate triangle edges */
        const sycl::float3 e0 = v2-v0;
        const sycl::float3 e1 = v0-v1;
        const sycl::float3 e2 = v1-v2;
        
        /* perform edge tests */
        const float U = sycl::dot(cross(e0,v2+v0),D);
        const float V = sycl::dot(cross(e1,v0+v1),D);
        const float W = sycl::dot(cross(e2,v1+v2),D);
        const float UVW = U+V+W;
        bool valid = (std::min(U,std::min(V,W)) >= -0.0f) | (std::max(U,std::max(V,W)) <= 0.0f);
        
        /* calculate geometry normal and denominator */
        const sycl::float3 Ng = sycl::cross(e2,e1);
        const float den = 2.0f*(dot(Ng,D));
        
        /* perform depth test */
        const float T = 2.0f*dot(v0,Ng);
        const float t = T/den;
        const float u = U/UVW;
        const float v = V/UVW;
        valid &= tnear <= t & t <= tfar;
        valid &= den != 0.0f;

        /* commit hit */
        if (valid)
          intel_ray_query_commit_potential_hit(query,t,sycl::float2(u,v));
      }
      else if (geom->type == Geometry::INSTANCE)
      {
        const Scene::InstanceGeometry* inst = (Scene::InstanceGeometry*) geom;
        const Transform local2world = inst->local2world;
        const Transform world2local = rcp(local2world);
        
        /* load ray */
        const uint32_t bvh_level = intel_get_hit_bvh_level( query, POTENTIAL_HIT );
        const sycl::float3 O = intel_get_ray_origin(query,bvh_level);
        const sycl::float3 D = intel_get_ray_direction(query,bvh_level);

        /* transform ray */
        const sycl::float3 O1 = xfmPoint(world2local, O);
        const sycl::float3 D1 = xfmVector(world2local, D);

        scenes[bvh_level+1] = inst->scene.get();
        rtas_t* inst_accel = (rtas_t*) inst->scene->getAccel();

        /* continue traversal */
        RayDescINTEL ray;
        ray.O = O1;
        ray.D = D1;
        ray.tmin = intel_get_ray_tnear(query,bvh_level);
        ray.tmax = 0.0f; // unused
        ray.mask = intel_get_ray_mask(query,bvh_level);
        ray.flags = intel_get_ray_flags(query,bvh_level);
        intel_ray_query_forward_ray(query, bvh_level+1, ray, inst_accel, 0);
      }
    }
    
    intel_ray_query_start_traversal(query);
    intel_sync_ray_query(query);
  }

  /* committed hit */
  if (intel_has_committed_hit(query))
  {
    out.hit_type = TEST_COMMITTED_HIT;
    out.bvh_level = intel_get_hit_bvh_level( query, COMMITTED_HIT );
    out.hit_candidate = intel_get_hit_candidate( query, COMMITTED_HIT );
    out.t = intel_get_hit_distance(query, COMMITTED_HIT);
    out.u = intel_get_hit_barys(query, COMMITTED_HIT).x();
    out.v = intel_get_hit_barys(query, COMMITTED_HIT).y();
    out.front_face = intel_hit_is_front_face( query, COMMITTED_HIT );
    out.instID = intel_get_hit_instanceID( query, COMMITTED_HIT );
    out.geomID = intel_get_hit_geomID( query, COMMITTED_HIT );
    out.primID = intel_get_hit_primID( query, COMMITTED_HIT );

    out.v0 = sycl::float3(0,0,0);
    out.v1 = sycl::float3(0,0,0);
    out.v2 = sycl::float3(0,0,0);
    if (intel_get_hit_candidate( query, COMMITTED_HIT ) == TRIANGLE)
    {
      sycl::float3 vertex_out[3];
      intel_get_hit_triangle_verts(query, vertex_out, COMMITTED_HIT);
      out.v0 = vertex_out[0];
      out.v1 = vertex_out[1];
      out.v2 = vertex_out[2];
    }
  }

  /* miss */
  else {
    out.hit_type = TEST_MISS;
  }
}

static const int width = 128;
static const int height = 128;
static const size_t numTests = 2*width*height;

uint32_t executeTest(sycl::device& device, sycl::queue& queue, sycl::context& context, InstancingType inst, TestType test)
{
  bool opaque = true;
  bool procedural = false;
  switch (test) {
  case TestType::TRIANGLES_COMMITTED_HIT       : opaque = true;  procedural=false; break;
  case TestType::TRIANGLES_POTENTIAL_HIT       : opaque = false; procedural=false; break;
  case TestType::TRIANGLES_ANYHIT_SHADER_COMMIT: opaque = false; procedural=false; break;
  case TestType::TRIANGLES_ANYHIT_SHADER_REJECT: opaque = false; procedural=false; break;
  case TestType::PROCEDURALS_COMMITTED_HIT     : opaque = false; procedural=true;  break;
  };

  uint32_t levels = 1;
  if (inst != InstancingType::NONE) levels = 2;

  RTCDevice rtcdevice = rtcNewSYCLDevice(&context, &queue, nullptr); // FIXME: remove

  //std::shared_ptr<Scene> scene = std::make_shared<Scene>(width,height,opaque,procedural);
  std::shared_ptr<Scene> scene(new Scene(width,height,opaque,procedural));
  scene->splitIntoGeometries(16);
  if (inst != InstancingType::NONE)
    scene->createInstances(3, inst == InstancingType::SW_INSTANCING);
  scene->buildAccel(rtcdevice,device,context);

  std::vector<Hit> tri_map;
  tri_map.resize(2*width*height);
  std::vector<uint32_t> id_stack;
  Transform local_to_world;
  scene->buildTriMap(local_to_world,id_stack,tri_map);
 
  TestInput* in = (TestInput*) sycl::aligned_alloc(64,numTests*sizeof(TestInput),device,context,sycl::usm::alloc::shared);
  memset(in, 0, numTests*sizeof(TestInput));

  TestOutput* out_test = (TestOutput*) sycl::aligned_alloc(64,numTests*sizeof(TestOutput),device,context,sycl::usm::alloc::shared);
  memset(out_test, 0, numTests*sizeof(TestOutput));

  TestOutput* out_expected = (TestOutput*) sycl::aligned_alloc(64,numTests*sizeof(TestOutput),device,context,sycl::usm::alloc::shared);
  memset(out_expected, 0, numTests*sizeof(TestOutput));

  TestHitType hit_type = TEST_MISS;
  switch (test) {
  case TestType::TRIANGLES_COMMITTED_HIT: hit_type = TEST_COMMITTED_HIT; break;
  case TestType::TRIANGLES_POTENTIAL_HIT: hit_type = TEST_POTENTIAL_HIT; break;
  case TestType::TRIANGLES_ANYHIT_SHADER_COMMIT: hit_type = TEST_COMMITTED_HIT; break;
  case TestType::TRIANGLES_ANYHIT_SHADER_REJECT: hit_type = TEST_MISS; break;
  case TestType::PROCEDURALS_COMMITTED_HIT: hit_type = TEST_COMMITTED_HIT; break;
  };

  for (size_t y=0; y<height; y++)
  {
    for (size_t x=0; x<width; x++)
    {
      for (size_t i=0; i<2; i++)
      {
        size_t tid = 2*(y*width+x)+i;
        assert(tid < numTests);

        Hit hit = tri_map[tid];
        sycl::float3 p = hit.triangle.sample(0.1f,0.6f);
        
        in[tid].org = p + sycl::float3(0,0,-1);
        in[tid].dir = sycl::float3(0,0,1);
        in[tid].tnear = 0.0f;
        in[tid].tfar = 10000.0f;
        in[tid].mask = 0xFF;
        in[tid].flags = NONE;

        // Ray data at level 0
        out_expected[tid].ray0_org = in[tid].org;
        out_expected[tid].ray0_dir = in[tid].dir;
        out_expected[tid].ray0_tnear = in[tid].tnear;
        out_expected[tid].ray0_mask = in[tid].mask;
        out_expected[tid].ray0_flags = in[tid].flags;
        
        // Ray data at hit bvh_level
        switch (test) {
        default: break;
        case TestType::TRIANGLES_POTENTIAL_HIT:
          out_expected[tid].rayN_org = in[tid].org;
          out_expected[tid].rayN_dir = in[tid].dir;
          out_expected[tid].rayN_tnear = in[tid].tnear;
          out_expected[tid].rayN_mask = in[tid].mask;
          out_expected[tid].rayN_flags = in[tid].flags;
          break;
        }
                 
        // Hit data
        out_expected[tid].hit_type = hit_type;
        switch (test) {
        default: break;
        case TestType::TRIANGLES_COMMITTED_HIT:
        case TestType::TRIANGLES_POTENTIAL_HIT:
        case TestType::TRIANGLES_ANYHIT_SHADER_COMMIT:
          out_expected[tid].bvh_level = levels-1;
          out_expected[tid].hit_candidate = TRIANGLE;
          out_expected[tid].t = 1.0f;
          out_expected[tid].u = 0.1f;
          out_expected[tid].v = 0.6f;
          out_expected[tid].front_face = 0;
          out_expected[tid].geomID = hit.geomID;
          out_expected[tid].primID = hit.primID;
          out_expected[tid].instID = hit.instID;
          out_expected[tid].v0 = hit.triangle.v0;
          out_expected[tid].v1 = hit.triangle.v1;
          out_expected[tid].v2 = hit.triangle.v2;
          out_expected[tid].world_to_object = rcp(Transform(hit.local_to_world));
          out_expected[tid].object_to_world = hit.local_to_world;
          break;
        case TestType::PROCEDURALS_COMMITTED_HIT:
          out_expected[tid].bvh_level = levels-1;
          out_expected[tid].hit_candidate = PROCEDURAL;
          out_expected[tid].t = 1.0f;
          out_expected[tid].u = 0.1f;
          out_expected[tid].v = 0.6f;
          out_expected[tid].front_face = 0;
          out_expected[tid].geomID = hit.geomID;
          out_expected[tid].primID = hit.primID;
          out_expected[tid].instID = hit.instID;
          out_expected[tid].v0 = sycl::float3(0,0,0);
          out_expected[tid].v1 = sycl::float3(0,0,0);
          out_expected[tid].v2 = sycl::float3(0,0,0);
          break;
        }
      }
    }
  }

  /* execute test */
  void* accel = scene->getAccel();
  size_t scene_ptr = (size_t) scene.get();

  if (inst != InstancingType::SW_INSTANCING &&
      (test == TestType::TRIANGLES_COMMITTED_HIT || test == TestType::TRIANGLES_POTENTIAL_HIT))
  {
    queue.submit([&](sycl::handler& cgh) {
                   const sycl::range<1> range(numTests);
                   cgh.parallel_for(range, [=](sycl::item<1> item) {
                                             const uint i = item.get_id(0);
                                             render(i,in[i],out_test[i],(rtas_t*)accel);
                                           });
                 });
    queue.wait_and_throw();
  }
  else
  {
    queue.submit([&](sycl::handler& cgh) {
                   const sycl::range<1> range(numTests);
                   cgh.parallel_for(range, [=](sycl::item<1> item) {
                                             const uint i = item.get_id(0);
                                             render_loop(i,in[i],out_test[i],scene_ptr,(rtas_t*)accel,test);
                                           });
                 });
    queue.wait_and_throw();
  }
    
  /* verify result */
  uint32_t numErrors = 0;
  for (size_t tid=0; tid<numTests; tid++)
    compareTestOutput(tid,numErrors,out_test[tid],out_expected[tid]);

  return numErrors;
}

enum Flags : uint32_t {
  FLAGS_NONE,
  DEPTH_TEST_LESS_EQUAL = 1 << 0  // when set we use <= for depth test, otherwise <
};

struct DispatchGlobals
{
  uint64_t rtMemBasePtr;               // base address of the allocated stack memory
  uint64_t callStackHandlerKSP;             // this is the KSP of the continuation handler that is invoked by BTD when the read KSP is 0
  uint32_t asyncStackSize;             // async-RT stack size in 64 byte blocks
  uint32_t numDSSRTStacks : 16;        // number of stacks per DSS
  uint32_t syncRayQueryCount : 4;      // number of ray queries in the sync-RT stack: 0-15 mapped to: 1-16
  unsigned _reserved_mbz : 12;
  uint32_t maxBVHLevels;               // the maximal number of supported instancing levels (0->8, 1->1, 2->2, ...)
  Flags flags;                         // per context control flags
};

void* allocDispatchGlobals(sycl::device device, sycl::context context)
{
  size_t maxBVHLevels = RTC_MAX_INSTANCE_LEVEL_COUNT+1;
  
  size_t rtstack_bytes = (64+maxBVHLevels*(64+32)+63)&-64;
  size_t num_rtstacks = 1<<17; // this is sufficiently large also for PVC
  size_t dispatchGlobalSize = 128+num_rtstacks*rtstack_bytes;
  
  //dispatchGlobalsPtr = this->malloc(dispatchGlobalSize, 64);
  void* dispatchGlobalsPtr = sycl::aligned_alloc(64,dispatchGlobalSize,device,context,sycl::usm::alloc::shared);
  memset(dispatchGlobalsPtr, 0, dispatchGlobalSize);
  
  if (((size_t)dispatchGlobalsPtr & 0xFFFF000000000000ull) != 0)
    throw std::runtime_error("internal error in RTStack allocation");
  
  DispatchGlobals* dg = (DispatchGlobals*) dispatchGlobalsPtr;
  dg->rtMemBasePtr = (uint64_t) dispatchGlobalsPtr + dispatchGlobalSize;
  dg->callStackHandlerKSP = 0;
  dg->asyncStackSize = 0;
  dg->numDSSRTStacks = 0;
  dg->syncRayQueryCount = 0;
  dg->_reserved_mbz = 0;
  dg->maxBVHLevels = maxBVHLevels;
  dg->flags = DEPTH_TEST_LESS_EQUAL;
  
  return dispatchGlobalsPtr;
}

int main(int argc, char* argv[])
{
  TestType test = TestType::TRIANGLES_COMMITTED_HIT;
  InstancingType inst = InstancingType::NONE;

  /* command line parsing */
  if (argc == 1) {
    std::cout << "ERROR: no test specified" << std::endl;
    return 1;
  }

  /* parse all command line options */
  for (size_t i=1; i<argc; i++)
  {
    if (strcmp(argv[i], "--triangles-committed-hit") == 0) {
    test = TestType::TRIANGLES_COMMITTED_HIT;
    }
    else if (strcmp(argv[i], "--triangles-potential-hit") == 0) {
      test = TestType::TRIANGLES_POTENTIAL_HIT;
    }
    else if (strcmp(argv[i], "--triangles-anyhit-shader-commit") == 0) {
      test = TestType::TRIANGLES_ANYHIT_SHADER_COMMIT;
    }
    else if (strcmp(argv[i], "--triangles-anyhit-shader-reject") == 0) {
      test = TestType::TRIANGLES_ANYHIT_SHADER_REJECT;
    }
    else if (strcmp(argv[i], "--procedurals-committed-hit") == 0) {
      test = TestType::PROCEDURALS_COMMITTED_HIT;
    }
    else if (strcmp(argv[i], "--no-instancing") == 0) {
      inst = InstancingType::NONE;
    }
    else if (strcmp(argv[i], "--hw-instancing") == 0) {
      inst = InstancingType::HW_INSTANCING;
    }
    else if (strcmp(argv[i], "--sw-instancing") == 0) {
      inst = InstancingType::SW_INSTANCING;
    }
    else {
      std::cout << "ERROR: invalid command line option " << argv[i] << std::endl;
      return 1;
    }
  }

  /* initialize SYCL device */
  device = sycl::device(sycl::gpu_selector());
  sycl::queue queue = sycl::queue(device,exception_handler);
  context = queue.get_context();

  dispatchGlobalsPtr = allocDispatchGlobals(device,context);

  /* execute test */
  RandomSampler_init(rng,0x56FE238A);
  uint32_t numErrors = executeTest(device,queue,context,inst,test);

  sycl::free(dispatchGlobalsPtr, context);
  
  return numErrors ? 1 : 0;
}
