
#include "../rthwif_production.h"
#include "../rthwif_builder.h"

#include "../../../include/embree4/rtcore.h"

#include <vector>
#include <iostream>
#include <iomanip>

// triangles_committed_hit: triangles
// quads_committed_hit: quads
// triangles_potential_hit: triangles + filter + check potential hit
// triangles_anyhit_shader: triangles + filter + commit/reject 
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
};

std::ostream& operator<<(std::ostream& out, const sycl::float3& v) {
    return out << "(" << v.x() << "," << v.y() << "," << v.z()  << ")";
}

uint32_t compareTestOutput(uint32_t tid, const TestOutput& test, const TestOutput& expected)
{
#define COMPARE(member)                 \
  if (test.member != expected.member) { \
    errors++; std::cout << "test" << tid << " " #member " mismatch " << test.member << " != " << expected.member << std::endl; \
  }
#define COMPARE1(member,eps)               \
  if (fabs(test.member-expected.member) > eps) {                              \
    errors++; std::cout << "test" << tid << " " #member " mismatch " << test.member << " != " << expected.member << std::endl; \
  }
#define COMPARE3(member,eps) {                                          \
    const bool x = fabs(test.member.x()-expected.member.x()) > eps;     \
    const bool y = fabs(test.member.y()-expected.member.y()) > eps;     \
    const bool z = fabs(test.member.z()-expected.member.z()) > eps;     \
    if (x || y || z) {                                                  \
      errors++; std::cout << "test" << tid << " " #member " mismatch " << test.member << " != " << expected.member << std::endl; \
    }                                                                   \
  }

  float eps = 1E-5;

  uint32_t errors = 0;
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

  return errors;
}

void render(uint32_t i, const TestInput& in, TestOutput& out, rtas_t* accel)
{
#if 1
  /* setup ray */
  RayDescINTEL ray;
  ray.O = in.org;
  ray.D = in.dir;
  ray.tmin = in.tnear;
  ray.tmax = in.tfar;
  ray.mask = in.mask;
  ray.flags = in.flags;
#else

  /* fixed camera */
  unsigned int x = 10;
  unsigned int y = 10;
  sycl::float3 vx(-1, -0, -0);
  sycl::float3 vy(-0, -1, -0);
  sycl::float3 vz(32, 32, 95.6379f);
  sycl::float3 p(278, 273, -800);
  
  /* compute primary ray */
  RayDescINTEL ray;
  ray.O = p;
  ray.D = float(x)*vx/8.0f + float(y)*vy/8.0f + vz;;
  ray.tmin = 0.0f;
  ray.tmax = INFINITY;
  ray.mask = 0xFF;
  ray.flags = 0;
#endif
  
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
  
  if (intel_has_committed_hit(query))
  {
    /* return committed hit data */
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

    /* return ray data at current level */
    uint32_t bvh_level = intel_get_hit_bvh_level( query, COMMITTED_HIT );
    out.rayN_org = intel_get_ray_origin(query,bvh_level);
    out.rayN_dir = intel_get_ray_direction(query,bvh_level);
    out.rayN_tnear = intel_get_ray_tnear(query,bvh_level);
    out.rayN_mask = intel_get_ray_mask(query,bvh_level);
    out.rayN_flags = intel_get_ray_flags(query,bvh_level);
  }
}

struct Triangle
{
  Triangle (sycl::float3 v0, sycl::float3 v1, sycl::float3 v2, uint32_t geomID, uint32_t primID)
    : v0(v0), v1(v1), v2(v2), geomID(geomID), primID(primID) {}

  sycl::float3 sample(float u, float v) {
    return (1.0f-u-v)*v0 + u*v1 + v*v2;
  }

  sycl::float3 v0,v1,v2;
  uint32_t geomID;
  uint32_t primID;
};

struct TriangleMesh
{
public:
  RTHWIF_GEOMETRY_TRIANGLES_DESC getDesc()
  {
    RTHWIF_GEOMETRY_TRIANGLES_DESC out;
    memset(&out,0,sizeof(out));
    out.GeometryType = RTHWIF_GEOMETRY_TYPE_TRIANGLES;
    out.GeometryFlags = RTHWIF_GEOMETRY_FLAG_OPAQUE;
    out.GeometryMask = 0xFF;
    out.IndexBuffer = (RTHWIF_UINT3*) triangles.data();
    out.TriangleCount = triangles.size();
    out.TriangleStride = sizeof(sycl::int3);
    out.VertexBuffer = (RTHWIF_FLOAT3*) vertices.data();
    out.VertexCount = vertices.size();
    out.VertexStride = sizeof(sycl::float3);
    return out;
  }

  Triangle getTriangle( uint32_t geomID, uint32_t primID ) const
  {
    const sycl::float3 v0 = vertices[triangles[primID].x()];
    const sycl::float3 v1 = vertices[triangles[primID].y()];
    const sycl::float3 v2 = vertices[triangles[primID].z()];
    return Triangle(v0,v1,v2,geomID,primID);
  }
  
public:
  std::vector<sycl::int3> triangles;
  std::vector<sycl::float3> vertices;
};

TriangleMesh createTrianglePlane (const sycl::float3& p0, const sycl::float3& dx, const sycl::float3& dy, size_t width, size_t height)
{
  TriangleMesh mesh;
  mesh.vertices.resize((width+1)*(height+1));
  mesh.triangles.resize(2*width*height);
  
  for (size_t y=0; y<=height; y++) {
    for (size_t x=0; x<=width; x++) {
      sycl::float3 p = p0+float(x)/float(width)*dx+float(y)/float(height)*dy;
      //if (x%2 == 0) p += sycl::float3(0,0,0.01f);
      //else p -= sycl::float3(0,0,0.01f);
      size_t i = y*(width+1)+x;
      mesh.vertices[i] = p;
    }
  }
  for (size_t y=0; y<height; y++) {
    for (size_t x=0; x<width; x++) {
      size_t i = 2*y*width+2*x;
      size_t p00 = (y+0)*(width+1)+(x+0);
      size_t p01 = (y+0)*(width+1)+(x+1);
      size_t p10 = (y+1)*(width+1)+(x+0);
      size_t p11 = (y+1)*(width+1)+(x+1);
      mesh.triangles[i+0] = sycl::int3((int)p00,(int)p01,(int)p10);
      mesh.triangles[i+1] = sycl::int3((int)p11,(int)p10,(int)p01);
    }
  }
  return mesh;
}

struct Scene
{
  Scene(uint32_t width, uint32_t height)
    : width(width), height(height)
  {
    TriangleMesh mesh = createTrianglePlane(sycl::float3(0,0,0), sycl::float3(width,0,0), sycl::float3(0,height,0), width, height);
    geometries.push_back(mesh);
  }
  
  size_t size() const {
    return geometries.size();
  }

  TriangleMesh& operator[] ( size_t i ) { return geometries[i]; }

  Triangle getTriangle( uint32_t x, uint32_t y, uint32_t id)
  {
    uint32_t geomID = 0;
    uint32_t primID = 2*(y*width+x)+id;
    TriangleMesh& mesh = geometries[geomID];
    return mesh.getTriangle(geomID,primID);
  }

  uint32_t width;
  uint32_t height;
  std::vector<TriangleMesh> geometries;
};

void* buildAccel( RTCDevice rtcdevice, sycl::device& device, sycl::context& context, Scene& scene)
{
  /* fill geometry descriptor buffer */
  std::vector<RTHWIF_GEOMETRY_TRIANGLES_DESC> desc(scene.size());
  std::vector<const RTHWIF_GEOMETRY_DESC*> geom(scene.size());
  for (size_t geomID=0; geomID<scene.size(); geomID++) {
    desc[geomID] = scene[geomID].getDesc();
    geom[geomID] = (const RTHWIF_GEOMETRY_DESC*) &desc[geomID];
  }

  /* estimate accel size */
  RTHWIF_AABB bounds;
  RTHWIF_BUILD_ACCEL_ARGS args;
  memset(&args,0,sizeof(args));
  args.bytes = sizeof(args);
  args.device = nullptr;
  args.embree_device = (void*) rtcdevice;
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

  void* accel = nullptr;
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

    if (err == RTHWIF_ERROR_OUT_OF_MEMORY)
      continue;
  }
  
  if (err != RTHWIF_ERROR_NONE)
    throw std::runtime_error("build error");

  return accel;
}

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

int main(int argc, char* argv[])
{
  static const int width = 16;
  static const int height = 16;
  
  sycl::device device = sycl::device(sycl::gpu_selector());
  sycl::queue queue = sycl::queue(device,exception_handler);
  sycl::context context = queue.get_context();

  RTCDevice rtcdevice = rtcNewSYCLDevice(&context, &queue, nullptr); // FIXME: remove

  Scene scene(width,height);

  void* accel = buildAccel( rtcdevice, device, context, scene);

  size_t numTests = 2*width*height;

  TestInput* in = (TestInput*) sycl::aligned_alloc(64,numTests*sizeof(TestInput),device,context,sycl::usm::alloc::shared);
  memset(in, 0, numTests*sizeof(TestInput));

  TestOutput* out_test = (TestOutput*) sycl::aligned_alloc(64,numTests*sizeof(TestOutput),device,context,sycl::usm::alloc::shared);
  memset(out_test, 0, numTests*sizeof(TestOutput));

  TestOutput* out_expected = (TestOutput*) sycl::aligned_alloc(64,numTests*sizeof(TestOutput),device,context,sycl::usm::alloc::shared);
  memset(out_expected, 0, numTests*sizeof(TestOutput));

  for (size_t y=0; y<height; y++)
  {
    for (size_t x=0; x<width; x++)
    {
      for (size_t i=0; i<2; i++)
      {
        size_t tid = 2*(y*width+x)+i;
        assert(tid < numTests);

        Triangle tri = scene.getTriangle(x,y,i);
        sycl::float3 p = tri.sample(0.1f,0.6f);
        
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
        out_expected[tid].rayN_org = in[tid].org;
        out_expected[tid].rayN_dir = in[tid].dir;
        out_expected[tid].rayN_tnear = in[tid].tnear;
        out_expected[tid].rayN_mask = in[tid].mask;
        out_expected[tid].rayN_flags = in[tid].flags;
          
        // Hit data
        out_expected[tid].bvh_level = 0;
        out_expected[tid].hit_candidate = TRIANGLE;
        out_expected[tid].t = 1.0f;
        out_expected[tid].u = 0.1f;
        out_expected[tid].v = 0.6f;
        out_expected[tid].front_face = 0;
        out_expected[tid].geomID = tri.geomID;
        out_expected[tid].primID = tri.primID;
        out_expected[tid].instID = -1;
        out_expected[tid].v0 = tri.v0;
        out_expected[tid].v1 = tri.v1;
        out_expected[tid].v2 = tri.v2;
      }
    }
  }

  /* execute test */
  queue.submit([&](sycl::handler& cgh) {
                 const sycl::range<1> range(numTests);
                 cgh.parallel_for(range, [=](sycl::item<1> item) {
                                              const uint i = item.get_id(0);
                                              render(i,in[i],out_test[i],(rtas_t*)accel);
                                            });
               });
  queue.wait_and_throw();
  
  /* verify result */
  uint32_t numErrors = 0;
  for (size_t tid=0; tid<numTests; tid++)
    numErrors += compareTestOutput(tid,out_test[tid],out_expected[tid]);
    
  return numErrors ? 1 : 0;
}
