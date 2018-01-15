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

#define _CRT_SECURE_NO_WARNINGS

#include "verify.h"
#include "../common/scenegraph/scenegraph.h"
#include "../common/scenegraph/geometry_creation.h"
#include "../../common/algorithms/parallel_for.h"
#include <regex>
#include <stack>

#define random  use_random_function_of_test // do use random_int() and random_float() from Test class
#define drand48 use_random_function_of_test // do use random_int() and random_float() from Test class

#define DEFAULT_STACK_SIZE 4*1024*1024
#define TEXT_ALIGN 85
 
namespace embree
{
  /* error reporting function */
  void errorHandler(void* userPtr, const RTCError code, const char* str = nullptr)
  {
    if (code == RTC_NO_ERROR)
      return;
    
    std::string descr = str ? ": " + std::string(str) : "";
    switch (code) {
    case RTC_UNKNOWN_ERROR    : throw std::runtime_error("RTC_UNKNOWN_ERROR"+descr);
    case RTC_INVALID_ARGUMENT : throw std::runtime_error("RTC_INVALID_ARGUMENT"+descr); break;
    case RTC_INVALID_OPERATION: throw std::runtime_error("RTC_INVALID_OPERATION"+descr); break;
    case RTC_OUT_OF_MEMORY    : throw std::runtime_error("RTC_OUT_OF_MEMORY"+descr); break;
    case RTC_UNSUPPORTED_CPU  : throw std::runtime_error("RTC_UNSUPPORTED_CPU"+descr); break;
    case RTC_CANCELLED        : throw std::runtime_error("RTC_CANCELLED"+descr); break;
    default                   : throw std::runtime_error("invalid error code"+descr); break;
    }
  }

  bool hasISA(const int isa) 
  {
    int cpu_features = getCPUFeatures();
    return (cpu_features & isa) == isa;
  }

  bool regex_match(std::string str, std::string regex)
  {
#if (defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1600)) // works around __ZTVNSt3__123__match_any_but_newlineIcEE link error
    return str == regex; 
#elif defined(__GNUC__) && !defined(__clang__) && (__GNUC__ <= 4) && (__GNUC_MINOR__ < 8) // workaround for older gcc version
    return str == regex; 
#else
    std::smatch match; std::regex regexpr(regex);
    return std::regex_match(str, match, regexpr);
#endif
  }

  void AssertNoError(RTCDevice device) 
  {
    RTCError error = rtcDeviceGetError(device);
    if (error != RTC_NO_ERROR) 
      throw std::runtime_error("Error occured: "+string_of(error));
  }

  void AssertAnyError(RTCDevice device)
  {
    RTCError error = rtcDeviceGetError(device);
    if (error == RTC_NO_ERROR) 
      throw std::runtime_error("Any error expected");
  }

  void AssertError(RTCDevice device, RTCError expectedError)
  {
    RTCError error = rtcDeviceGetError(device);
    if (error != expectedError) 
      throw std::runtime_error("Error "+string_of(expectedError)+" expected");
  }

  typedef SceneGraph::TriangleMeshNode::Triangle Triangle;

  void addRandomSubdivFeatures(RandomSampler& sampler, Ref<SceneGraph::SubdivMeshNode> mesh, size_t numEdgeCreases, size_t numVertexCreases, size_t numHoles)
  {
    std::vector<unsigned> offsets;
    std::vector<unsigned>& faces = mesh->verticesPerFace;
    std::vector<unsigned>& indices = mesh->position_indices;
    for (size_t i=0, j=0; i<mesh->verticesPerFace.size(); i++) {
      offsets.push_back(unsigned(j)); j+=mesh->verticesPerFace[i];
    } 
    
    for (size_t i=0; i<numEdgeCreases; i++) 
    {
      if (faces.size()) {
        int f = RandomSampler_getInt(sampler) % faces.size();
        int n = faces[f];
        int e = RandomSampler_getInt(sampler) % n;
        mesh->edge_creases.push_back(Vec2i(indices[offsets[f]+(e+0)%n],indices[offsets[f]+(e+1)%n]));
      } else {
        mesh->edge_creases.push_back(Vec2i(0,0));
      }
      mesh->edge_crease_weights.push_back(10.0f*RandomSampler_getFloat(sampler));
    }
    
    for (size_t i=0; i<numVertexCreases; i++) 
    {
      if (faces.size()) {
        size_t f = RandomSampler_getInt(sampler) % faces.size();
        size_t e = RandomSampler_getInt(sampler) % faces[f];
        mesh->vertex_creases.push_back(indices[offsets[f] + e]);
        mesh->vertex_crease_weights.push_back(10.0f*RandomSampler_getFloat(sampler));
      }
    }
    
    for (size_t i=0; i<numHoles; i++) {
      mesh->holes.push_back(RandomSampler_getInt(sampler) % faces.size());
    }
  }    

  struct Sphere
  {
    ALIGNED_CLASS;
  public:
    Sphere () : pos(zero), r(zero) {}
    Sphere (const Vec3fa& pos, float r) : pos(pos), r(r) {}
    __forceinline BBox3fa bounds() const { return BBox3fa(pos-Vec3fa(r),pos+Vec3fa(r)); }
  public:
    Vec3fa pos;
    float r;
  };

  void BoundsFunc(Sphere* sphere, size_t index, BBox3fa* bounds_o)
  {
    bounds_o->lower.x = sphere->pos.x-sphere->r;
    bounds_o->lower.y = sphere->pos.y-sphere->r;
    bounds_o->lower.z = sphere->pos.z-sphere->r;
    bounds_o->upper.x = sphere->pos.x+sphere->r;
    bounds_o->upper.y = sphere->pos.y+sphere->r;
    bounds_o->upper.z = sphere->pos.z+sphere->r;
  }

  void IntersectFuncN(const int* valid,
                      void* ptr,
                      const RTCIntersectContext* context,
                      RTCRayN* rays,
                      size_t N,
                      size_t item)
  {
  }

  struct VerifyScene : public RefCount
  {
    VerifyScene (const RTCDeviceRef& device, RTCSceneFlags sflags, RTCAlgorithmFlags aflags)
      : device(device), scene(rtcDeviceNewScene(device,sflags,aflags)) {}

    operator RTCScene() const {
      return scene;
    }
    
    unsigned addGeometry(const RTCGeometryFlags gflag, const Ref<SceneGraph::Node>& node)
    {
      nodes.push_back(node);
      
      if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) 
      {
        unsigned geomID = rtcNewTriangleMesh (scene, gflag, mesh->triangles.size(), mesh->numVertices(), mesh->numTimeSteps());
        AssertNoError(device);
        rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER ,mesh->triangles.data(),0,sizeof(SceneGraph::TriangleMeshNode::Triangle));
        for (size_t t=0; t<mesh->numTimeSteps(); t++)
          rtcSetBuffer(scene,geomID,RTCBufferType(RTC_VERTEX_BUFFER0+t),mesh->positions[t].data(),0,sizeof(SceneGraph::TriangleMeshNode::Vertex));
        AssertNoError(device);
        return geomID;
      }
      else if (Ref<SceneGraph::QuadMeshNode> mesh = node.dynamicCast<SceneGraph::QuadMeshNode>())
      {
        unsigned geomID = rtcNewQuadMesh (scene, gflag, mesh->quads.size(), mesh->numVertices(), mesh->numTimeSteps());
        AssertNoError(device);
        rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER ,mesh->quads.data(),0,sizeof(SceneGraph::QuadMeshNode::Quad  ));
        for (size_t t=0; t<mesh->numTimeSteps(); t++)
          rtcSetBuffer(scene,geomID,RTCBufferType(RTC_VERTEX_BUFFER0+t),mesh->positions[t].data(),0,sizeof(SceneGraph::QuadMeshNode::Vertex));
        AssertNoError(device);
        return geomID;
      } 
      else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>())
      {
        unsigned geomID = rtcNewSubdivisionMesh (scene, gflag, 
                                                 mesh->verticesPerFace.size(), mesh->position_indices.size(), mesh->numPositions(), 
                                                 mesh->edge_creases.size(), mesh->vertex_creases.size(), mesh->holes.size(), mesh->numTimeSteps());
        AssertNoError(device);
        rtcSetBuffer(scene,geomID,RTC_FACE_BUFFER  ,mesh->verticesPerFace.data(), 0,sizeof(int));
        rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER ,mesh->position_indices.data(),0,sizeof(int));
        for (size_t t=0; t<mesh->numTimeSteps(); t++)
          rtcSetBuffer(scene,geomID,RTCBufferType(RTC_VERTEX_BUFFER0+t),mesh->positions[t].data(),0,sizeof(SceneGraph::SubdivMeshNode::Vertex));
        if (mesh->edge_creases.size()) rtcSetBuffer(scene,geomID,RTC_EDGE_CREASE_INDEX_BUFFER,mesh->edge_creases.data(),0,2*sizeof(int));
        if (mesh->edge_crease_weights.size()) rtcSetBuffer(scene,geomID,RTC_EDGE_CREASE_WEIGHT_BUFFER,mesh->edge_crease_weights.data(),0,sizeof(float));
        if (mesh->vertex_creases.size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_CREASE_INDEX_BUFFER,mesh->vertex_creases.data(),0,sizeof(int));
        if (mesh->vertex_crease_weights.size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_CREASE_WEIGHT_BUFFER,mesh->vertex_crease_weights.data(),0,sizeof(float));
        if (mesh->holes.size()) rtcSetBuffer(scene,geomID,RTC_HOLE_BUFFER,mesh->holes.data(),0,sizeof(int));
        rtcSetTessellationRate(scene,geomID,mesh->tessellationRate);
        rtcSetSubdivisionMode(scene,geomID,0,mesh->position_subdiv_mode);
        AssertNoError(device);
        return geomID;
      }
      else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>())
      {
        unsigned int geomID = 0;
        switch (mesh->basis) {
        case SceneGraph::HairSetNode::BEZIER : geomID = rtcNewBezierHairGeometry (scene, gflag, (unsigned int)mesh->numPrimitives() , (unsigned int)mesh->numVertices(), (unsigned int)mesh->numTimeSteps()); break;
        case SceneGraph::HairSetNode::BSPLINE: geomID = rtcNewBSplineHairGeometry (scene, gflag, (unsigned int)mesh->numPrimitives(), (unsigned int)mesh->numVertices(), (unsigned int)mesh->numTimeSteps()); break;
        default: assert(false);
        }
        AssertNoError(device);
        for (size_t t=0; t<mesh->numTimeSteps(); t++)
          rtcSetBuffer(scene,geomID,RTCBufferType(RTC_VERTEX_BUFFER0+t),mesh->positions[t].data(),0,sizeof(SceneGraph::HairSetNode::Vertex));
        rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,mesh->hairs.data(),0,sizeof(SceneGraph::HairSetNode::Hair));
        AssertNoError(device);
        return geomID;
      } 
      else if (Ref<SceneGraph::LineSegmentsNode> mesh = node.dynamicCast<SceneGraph::LineSegmentsNode>())
      {
        unsigned int geomID = rtcNewLineSegments (scene, gflag, mesh->indices.size(), mesh->numVertices(), mesh->numTimeSteps());
        AssertNoError(device);
        for (size_t t=0; t<mesh->numTimeSteps(); t++)
          rtcSetBuffer(scene,geomID,RTCBufferType(RTC_VERTEX_BUFFER0+t),mesh->positions[t].data(),0,sizeof(SceneGraph::LineSegmentsNode::Vertex));
        rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,mesh->indices.data(),0,sizeof(int));
        AssertNoError(device);
        return geomID;
      }
      else {
        THROW_RUNTIME_ERROR("unknown node type");
      }
      return 0;
    }

    std::pair<unsigned,Ref<SceneGraph::Node>> addGeometry2(const RTCGeometryFlags gflag, const Ref<SceneGraph::Node>& node) {
      return std::make_pair(addGeometry(gflag,node),node);
    }
    
    std::pair<unsigned,Ref<SceneGraph::Node>> addPlane (RandomSampler& sampler, const RTCGeometryFlags gflag, size_t num, const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy) {
      return addGeometry2(gflag,SceneGraph::createTrianglePlane(p0,dx,dy,num,num));
    }
    
    std::pair<unsigned,Ref<SceneGraph::Node>> addSubdivPlane (RandomSampler& sampler, RTCGeometryFlags gflag, size_t num, const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy) {
      return addGeometry2(gflag,SceneGraph::createSubdivPlane(p0,dx,dy,num,num,2.0f));
    }
    
    std::pair<unsigned,Ref<SceneGraph::Node>> addSphere (RandomSampler& sampler, RTCGeometryFlags gflag, const Vec3fa& pos, const float r, size_t numPhi, size_t maxTriangles = -1, const avector<Vec3fa>& motion_vector = avector<Vec3fa>())
    {
      Ref<SceneGraph::Node> node = SceneGraph::createTriangleSphere(pos,r,numPhi);
      if (motion_vector.size()) SceneGraph::set_motion_vector(node,motion_vector);
      if (maxTriangles != size_t(-1)) SceneGraph::resize_randomly(sampler,node,maxTriangles);
      return addGeometry2(gflag,node);
    }

    std::pair<unsigned,Ref<SceneGraph::Node>> addQuadSphere (RandomSampler& sampler, RTCGeometryFlags gflag, const Vec3fa& pos, const float r, size_t numPhi, size_t maxQuads = -1, const avector<Vec3fa>& motion_vector = avector<Vec3fa>())
    {
      Ref<SceneGraph::Node> node = SceneGraph::createQuadSphere(pos,r,numPhi);
      if (motion_vector.size()) SceneGraph::set_motion_vector(node,motion_vector);
      if (maxQuads != size_t(-1)) SceneGraph::resize_randomly(sampler,node,maxQuads);
      return addGeometry2(gflag,node);
    }

    std::pair<unsigned,Ref<SceneGraph::Node>> addSubdivSphere (RandomSampler& sampler, RTCGeometryFlags gflag, const Vec3fa& pos, const float r, size_t numPhi, float level, size_t maxFaces = -1, const avector<Vec3fa>& motion_vector = avector<Vec3fa>())
    {
      Ref<SceneGraph::Node> node = SceneGraph::createSubdivSphere(pos,r,numPhi,level);
      if (motion_vector.size()) SceneGraph::set_motion_vector(node,motion_vector);
      if (maxFaces != size_t(-1)) SceneGraph::resize_randomly(sampler,node,maxFaces);
      addRandomSubdivFeatures(sampler,node.dynamicCast<SceneGraph::SubdivMeshNode>(),10,10,0);
      return addGeometry2(gflag,node);
    }
    
    std::pair<unsigned,Ref<SceneGraph::Node>> addSphereHair (RandomSampler& sampler, RTCGeometryFlags gflag, const Vec3fa& center, const float radius, const avector<Vec3fa>& motion_vector = avector<Vec3fa>())
    {
      Ref<SceneGraph::Node> node = SceneGraph::createSphereShapedHair(center,radius);
      if (motion_vector.size()) SceneGraph::set_motion_vector(node,motion_vector);
      return addGeometry2(gflag,node);
    }
    
    std::pair<unsigned,Ref<SceneGraph::Node>> addHair (RandomSampler& sampler, RTCGeometryFlags gflag, const Vec3fa& pos, const float scale, const float r, size_t numHairs = 1, const avector<Vec3fa>& motion_vector = avector<Vec3fa>())
    {
      Ref<SceneGraph::Node> node = SceneGraph::createHairyPlane(RandomSampler_getInt(sampler),pos,Vec3fa(1,0,0),Vec3fa(0,0,1),scale,r,numHairs,SceneGraph::HairSetNode::HAIR);
      if (motion_vector.size()) SceneGraph::set_motion_vector(node,motion_vector);
      return addGeometry2(gflag,node);
    }

    std::pair<unsigned,Ref<SceneGraph::Node>> addUserGeometryEmpty (RandomSampler& sampler, RTCGeometryFlags gflag, Sphere* sphere)
    {
      unsigned geom = rtcNewUserGeometry3 (scene,gflag,1,1);
      rtcSetBoundsFunction(scene,geom,(RTCBoundsFunc)BoundsFunc);
      rtcSetUserData(scene,geom,sphere);
      rtcSetIntersectFunctionN(scene,geom,IntersectFuncN);
      rtcSetOccludedFunctionN(scene,geom,IntersectFuncN);
      return std::make_pair(geom,Ref<SceneGraph::Node>(nullptr));
    }

    void resizeRandomly (std::pair<unsigned,Ref<SceneGraph::Node>> geom, RandomSampler& sampler)
    {
      if (Ref<SceneGraph::TriangleMeshNode> mesh = geom.second.dynamicCast<SceneGraph::TriangleMeshNode>())
      {
        rtcSetBuffer2(scene,geom.first,RTC_INDEX_BUFFER ,mesh->triangles.data(),0,sizeof(SceneGraph::TriangleMeshNode::Triangle), RandomSampler_getInt(sampler) % (mesh->triangles.size()+1));
      }
      else if (Ref<SceneGraph::QuadMeshNode> mesh = geom.second.dynamicCast<SceneGraph::QuadMeshNode>())
      {
        rtcSetBuffer2(scene,geom.first,RTC_INDEX_BUFFER ,mesh->quads.data(),0,sizeof(SceneGraph::QuadMeshNode::Quad), RandomSampler_getInt(sampler) % (mesh->quads.size()+1));
      } 
      else if (Ref<SceneGraph::SubdivMeshNode> mesh = geom.second.dynamicCast<SceneGraph::SubdivMeshNode>())
      {
        rtcSetBuffer2(scene,geom.first,RTC_FACE_BUFFER  ,mesh->verticesPerFace.data(), 0,sizeof(int), RandomSampler_getInt(sampler) % (mesh->verticesPerFace.size()+1));
      }
      else if (Ref<SceneGraph::HairSetNode> mesh = geom.second.dynamicCast<SceneGraph::HairSetNode>())
      {
        rtcSetBuffer2(scene,geom.first,RTC_INDEX_BUFFER,mesh->hairs.data(),0,sizeof(SceneGraph::HairSetNode::Hair), RandomSampler_getInt(sampler) % (mesh->hairs.size()+1));
      } 
      else if (Ref<SceneGraph::LineSegmentsNode> mesh = geom.second.dynamicCast<SceneGraph::LineSegmentsNode>())
      {
        rtcSetBuffer2(scene,geom.first,RTC_INDEX_BUFFER,mesh->indices.data(),0,sizeof(int), RandomSampler_getInt(sampler) % (mesh->indices.size()+1));
      }
    }

  public:
    const RTCDeviceRef& device;
    RTCSceneRef scene;
    std::vector<Ref<SceneGraph::Node>> nodes;
  };

  VerifyApplication::TestReturnValue VerifyApplication::Test::execute(VerifyApplication* state, bool silent)
  {
    if (!isEnabled())
      return SKIPPED;
    
    if (!silent) 
      std::cout << std::setw(TEXT_ALIGN) << name << " ..." << std::flush;
      
    TestReturnValue v = SKIPPED;
    try {
      v = run(state,silent);
    } catch (...) {
      v = FAILED;
    }
    TestReturnValue ev = ty != TEST_SHOULD_FAIL ? PASSED : FAILED;
    bool passed = v == ev || v == SKIPPED;

    if (silent) 
    {
      Lock<MutexSys> lock(state->mutex);
      if (v != SKIPPED) {
        if      (passed       ) std::cout << state->green ("+") << std::flush;
        else if (ignoreFailure) std::cout << state->yellow("!") << std::flush;
        else                    std::cout << state->red   ("-") << std::flush;
      }
    } 
    else
    {
      if      (v == SKIPPED ) std::cout << state->green (" [SKIPPED]") << std::endl << std::flush;
      else if (passed       ) std::cout << state->green (" [PASSED]" ) << std::endl << std::flush;
      else if (ignoreFailure) std::cout << state->yellow(" [FAILED] (ignored)") << std::endl << std::flush;
      else                    std::cout << state->red   (" [FAILED]" ) << std::endl << std::flush;
    }

    /* update passed/failed counters */
    state->numPassedTests += passed;
    if (ignoreFailure) state->numFailedAndIgnoredTests += !passed;
    else               state->numFailedTests           += !passed;
    
    /* do ignore failures for some specific tests */
    if (ignoreFailure) 
      passed = true;
    //assert(passed);
    return passed ? PASSED : FAILED;
  }

  double VerifyApplication::Benchmark::readDatabase(VerifyApplication* state)
  {
    std::fstream db;
    FileName base = state->database+FileName(name);
    db.open(base.addExt(".txt"), std::fstream::in);
    double start_value = higher_is_better ? double(neg_inf) : double(pos_inf);
    if (!db.is_open()) return start_value;

    double bestAvg = start_value;
    while (true) 
    {
      std::string line; std::getline(db,line);
      if (db.eof()) break;
      if (line == "") { bestAvg = start_value; continue; }
      std::stringstream linestream(line); 
      std::string hash; linestream >> hash;
      double avg; linestream >> avg;
      if (higher_is_better) {
        if (avg > bestAvg) bestAvg = avg;
      } else {
        if (avg < bestAvg) bestAvg = avg;
      }
    }
    db.close();
    return bestAvg;
  }

  void VerifyApplication::Benchmark::updateDatabase(VerifyApplication* state, Statistics stat, double bestAvg)
  {
    /* load git hash from file */
    std::fstream hashFile;
    std::string hash = "unknown";
    hashFile.open(FileName::executableFolder()+FileName("hash"), std::fstream::in);
    if (hashFile.is_open()) {
      hashFile >> hash;
      hashFile.close();
    }

    /* update database */
    std::fstream db;
    FileName base = state->database+FileName(name);
    db.open(base.addExt(".txt"), std::fstream::out | std::fstream::app);
    db << hash << " " << stat.getAvg() << " " << bestAvg << std::endl;
    db.close();
  }

  void VerifyApplication::Benchmark::plotDatabase(VerifyApplication* state)
  {
    std::fstream plot;
    FileName base = state->database+FileName(name);
    plot.open(base.addExt(".plot"), std::fstream::out | std::fstream::trunc);
    plot << "set terminal png size 2048,600 enhanced" << std::endl;
    plot << "set output \"" << base.addExt(".png") << "\"" << std::endl;
    plot << "set key inside right top vertical Right noreverse enhanced autotitles box linetype -1 linewidth 1.000" << std::endl;
    plot << "set samples 50, 50" << std::endl;
    //plot << "set title \"" << name << "\"" << std::endl; 
    //plot << "set xlabel \"" << name << "\""<< std::endl;
    plot << "set xtics axis rotate by 90" << std::endl;
    plot << "set ylabel \"" << unit << "\"" << std::endl;
    plot << "set yrange [0:]" << std::endl;
    plot << "plot \"" << FileName(name).addExt(".txt") << "\" using :2:xtic(1) title \"" << name << "\" with lines, \\" << std::endl; 
    plot << "     \"" << FileName(name).addExt(".txt") << "\" using :3         title \"best\" with lines" << std::endl;
    plot << std::endl;
    plot.close();
  }

  Statistics VerifyApplication::Benchmark::benchmark_loop(VerifyApplication* state)
  {
    //sleepSeconds(0.1);
    const size_t skipBenchmarkFrames = 1;
    const size_t numBenchmarkFrames = 8;
    FilteredStatistics stat(0.5f,0.0f);
    size_t numTotalFrames = skipBenchmarkFrames + numBenchmarkFrames;
    for (size_t i=0; i<skipBenchmarkFrames; i++) {
      benchmark(state);
    }  
    for (size_t i=skipBenchmarkFrames; i<numTotalFrames; i++) {
      stat.add(benchmark(state));
    }
    return stat.getStatistics();
  }

  VerifyApplication::TestReturnValue VerifyApplication::Benchmark::execute(VerifyApplication* state, bool silent) try
  {
    if (!isEnabled())
      return SKIPPED;

    if (!setup(state)) {
      cleanup(state);
      return SKIPPED;
    }
    
    /* print benchmark name */
    std::cout << std::setw(TEXT_ALIGN) << name << ": " << std::flush;
   
    /* read current best from database */
    double avgdb = 0.0f;
    if (state->database != "")
      avgdb = readDatabase(state);

    /* execute benchmark */
    Statistics curStat;
    Statistics bestStat;
    bool passed = false;
    
    /* retry if benchmark failed */
    //static size_t numRetries = 0;
    size_t i=0;
    for (; i<max_attempts && !passed; i++)
    {
      if (i != 0) {
        cleanup(state);
        //if (numRetries++ > 1000) break;
        //std::cout << state->yellow(" [RETRY]" ) << " (" << 100.0f*(curStat.getAvg()-avgdb)/avgdb << "%)" << std::flush;
        setup(state);
      }

      try {
        curStat = benchmark_loop(state);
        if (i == 0) bestStat = curStat;
        if (higher_is_better) {
          if (curStat.getAvg() > bestStat.getAvg()) bestStat = curStat;
        } else {
          if (curStat.getAvg() < bestStat.getAvg()) bestStat = curStat;
        }
      } catch (TestReturnValue v) {
        return v;
      }

      /* check against database to see if test passed */
      if (state->database != "") {
        if (higher_is_better)
          passed = !(curStat.getAvg()-avgdb < -state->benchmark_tolerance*avgdb); // !(a < b) on purpose for nan case
        else
          passed = !(curStat.getAvg()-avgdb > +state->benchmark_tolerance*avgdb); // !(a > b) on purpose for nan case
      }
      else
        passed = true;
    }

    if (state->database == "" || avgdb == double(neg_inf))
      avgdb = bestStat.getAvg();

    /* update database */
    if (state->database != "" && state->update_database)
      updateDatabase(state,bestStat,avgdb);
      
    /* print test result */
    std::cout << std::setw(8) << std::setprecision(3) << std::fixed << bestStat.getAvg() << " " << unit << " (+/-" << 100.0f*bestStat.getAvgSigma()/bestStat.getAvg() << "%)";
    if (passed) std::cout << state->green(" [PASSED]" ) << " (" << 100.0f*(bestStat.getAvg()-avgdb)/avgdb << "%) (" << i << " attempts)" << std::endl << std::flush;
    else        std::cout << state->red  (" [FAILED]" ) << " (" << 100.0f*(bestStat.getAvg()-avgdb)/avgdb << "%) (" << i << " attempts)" << std::endl << std::flush;
    if (state->database != "")
      plotDatabase(state);

    /* print dart measurement */
    if (state->cdash) 
    {
      //std::cout << "<DartMeasurement name=\"" + name + ".avg\" type=\"numeric/float\">" << bestStat.getAvg() << "</DartMeasurement>" << std::endl;
      //std::cout << "<DartMeasurement name=\"" + name + ".sigma\" type=\"numeric/float\">" << bestStat.getAvgSigma() << "</DartMeasurement>" << std::endl;

      /* send plot only when test failed */
      if (!passed)
      {
        FileName base = state->database+FileName(name);
        std::string command = std::string("cd ")+state->database.str()+std::string(" && gnuplot ") + FileName(name).addExt(".plot").str();
        if (system(command.c_str()) == 0)
          std::cout << "<DartMeasurementFile name=\"" << name << "\" type=\"image/png\">" << base.addExt(".png") << "</DartMeasurementFile>" << std::endl;
      }
    }   

    sleepSeconds(0.1);
    cleanup(state);

    /* update passed/failed counters */
    state->numPassedTests += passed;
    state->numFailedTests += !passed;
    return passed ? PASSED : FAILED;
  }
  catch (const std::exception& e)
  {
    std::cout << std::setw(TEXT_ALIGN) << name << ": " << std::flush;
    std::cout << state->red(" [FAILED] ") << "(" << e.what() << ")" << std::endl << std::flush;
    state->numFailedTests++;
    return VerifyApplication::FAILED;
  }
  catch (...)
  {
    std::cout << std::setw(TEXT_ALIGN) << name << ": " << std::flush;
    std::cout << state->red(" [FAILED] ") << " (unknown error)" << std::endl << std::flush;
    state->numFailedTests++;
    return VerifyApplication::FAILED;
  }

  VerifyApplication::TestReturnValue VerifyApplication::TestGroup::execute(VerifyApplication* state, bool silent_in)
  {
    if (!isEnabled())
      return SKIPPED;

    bool leaftest = state->flatten && !silent_in && silent;
    bool nextsilent = silent_in || (state->flatten && silent);
    if (leaftest) 
      std::cout << std::setw(TEXT_ALIGN) << name << " ..." << std::flush;
    
    std::atomic<int> passed(true);

#if defined(__WIN32__) && !defined(__X86_64__)
	/* deactivating parallel test execution on win32 platforms due to out-of-memory exceptions */
	parallel = false;
#endif

    if (state->parallel && parallel && leaftest) 
    {
      parallel_for(tests.size(),[&] (size_t i) {
          passed.fetch_and(tests[i]->execute(state,nextsilent) != FAILED);
        });
    } 
    else {
      for (auto& test : tests)
        passed.fetch_and(test->execute(state,nextsilent) != FAILED);
    }

    if (leaftest) {
      if (passed) std::cout << state->green(" [PASSED]") << std::endl << std::flush;
      else        std::cout << state->red  (" [FAILED]") << std::endl << std::flush;
    }

    return passed ? PASSED : FAILED;
  }

  RTCAlgorithmFlags aflags = (RTCAlgorithmFlags) (RTC_INTERSECT1 | RTC_INTERSECT4 | RTC_INTERSECT8 | RTC_INTERSECT16);
  RTCAlgorithmFlags aflags_all = (RTCAlgorithmFlags) (RTC_INTERSECT1 | RTC_INTERSECT4 | RTC_INTERSECT8 | RTC_INTERSECT16 | RTC_INTERSECT_STREAM);

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  struct InitExitTest : public VerifyApplication::Test
  {
    InitExitTest (std::string name)
      : VerifyApplication::Test(name,0,VerifyApplication::TEST_SHOULD_PASS) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      DISABLE_DEPRECATED_WARNING;
      rtcInit("verbose=1");
      errorHandler(nullptr,rtcGetError());
      rtcExit();
      ENABLE_DEPRECATED_WARNING;
      return VerifyApplication::PASSED;
    }
  };

  struct EmbreeInternalTest : public VerifyApplication::Test
  {
    EmbreeInternalTest (std::string name, size_t testID)
      : VerifyApplication::Test(name,0,VerifyApplication::TEST_SHOULD_PASS), testID(testID) {}
  
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      RTCDeviceRef device = rtcNewDevice(state->rtcore.c_str());
      AssertNoError(device);
      return (VerifyApplication::TestReturnValue)rtcDeviceGetParameter1i(device,(RTCParameter)(3000000+testID));
    }

    size_t testID;
  };

  struct os_shrink_test : public VerifyApplication::Test
  {
    struct Allocation 
    {
      Allocation (size_t bytes) 
        : ptr(os_malloc(bytes,hugepages)), bytes(bytes) {}

      ~Allocation() {
        free();
      }

      void free() {
        if (!ptr) return;
        os_free(ptr,bytes,hugepages);
        ptr = nullptr;
      }

      size_t shrink() {
        if (!ptr) return 0;
        bytes = os_shrink(ptr,bytes/2,bytes,hugepages);
        return bytes;
      }

    private:
      void* ptr;
      size_t bytes;
      bool hugepages;
    };

    os_shrink_test ()
      : VerifyApplication::Test("os_shrink_test",ISA,VerifyApplication::TEST_SHOULD_PASS,false) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::vector<std::unique_ptr<Allocation>> allocations;
      allocations.reserve(1024*1024);
      for (size_t i=0; i<1024*1024; i++) 
      {
        std::unique_ptr<Allocation> alloc(new Allocation(2*4096));
        if (allocations.size() > 1) allocations.back()->shrink();
        allocations.push_back(std::move(alloc)); 
        
      }
      allocations.clear();
      return VerifyApplication::PASSED;
    }
  };

  struct MultipleDevicesTest : public VerifyApplication::Test
  {
    MultipleDevicesTest (std::string name, int isa)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDevice device1 = rtcNewDevice(cfg.c_str());
      AssertNoError(device1);
      RTCDevice device2 = rtcNewDevice(cfg.c_str());
      AssertNoError(device2);
      RTCDevice device3 = rtcNewDevice(cfg.c_str());
      AssertNoError(device3);
      rtcDeleteDevice(device1);
      rtcDeleteDevice(device3);
      rtcDeleteDevice(device2);
      return VerifyApplication::PASSED;
    }
  };

  struct FlagsTest : public VerifyApplication::Test
  {
    RTCSceneFlags sceneFlags;
    RTCGeometryFlags geomFlags;

    FlagsTest (std::string name, int isa, VerifyApplication::TestType type, RTCSceneFlags sceneFlags, RTCGeometryFlags geomFlags)
      : VerifyApplication::Test(name,isa,type), sceneFlags(sceneFlags), geomFlags(geomFlags) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      RTCSceneRef scene = rtcDeviceNewScene(device,sceneFlags,aflags);
      AssertNoError(device);
      rtcNewTriangleMesh (scene, geomFlags, 0, 0);
      AssertNoError(device);
      rtcNewBezierHairGeometry (scene, geomFlags, 0, 0);
      AssertNoError(device);
      rtcCommit (scene);
      AssertNoError(device);
      return VerifyApplication::PASSED;
    }
  };
  
  struct UnmappedBeforeCommitTest : public VerifyApplication::Test
  {
    UnmappedBeforeCommitTest (std::string name, int isa)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      VerifyScene scene(device,RTC_SCENE_STATIC,aflags);
      AssertNoError(device);
      unsigned geom0 = scene.addSphere(sampler,RTC_GEOMETRY_STATIC,zero,1.0f,50).first;
      unsigned geom1 = scene.addSphere(sampler,RTC_GEOMETRY_STATIC,zero,1.0f,50).first;
      AssertNoError(device);
      rtcMapBuffer(scene,geom0,RTC_INDEX_BUFFER);
      rtcMapBuffer(scene,geom1,RTC_VERTEX_BUFFER);
      AssertNoError(device);
      rtcCommit (scene);
      AssertError(device,RTC_INVALID_OPERATION); // error, buffers still mapped
      return VerifyApplication::PASSED;
    }
  };

  struct GetBoundsTest : public VerifyApplication::Test
  {
    GeometryType gtype;

    GetBoundsTest (std::string name, int isa, GeometryType gtype)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), gtype(gtype) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      VerifyScene scene(device,RTC_SCENE_STATIC,RTC_INTERSECT1);
      AssertNoError(device);

      Ref<SceneGraph::Node> node = nullptr;
      switch (gtype) {
      case TRIANGLE_MESH   : node = SceneGraph::createTriangleSphere(zero,1.0f,50); break;
      case TRIANGLE_MESH_MB: node = SceneGraph::createTriangleSphere(zero,1.0f,5)->set_motion_vector(Vec3fa(1.0f)); break;
      case QUAD_MESH       : node = SceneGraph::createQuadSphere(zero,1.0f,50); break;
      case QUAD_MESH_MB    : node = SceneGraph::createQuadSphere(zero,1.0f,50)->set_motion_vector(Vec3fa(1.0f)); break;
        //case SUBDIV_MESH:
        //case SUBDIV_MESH_MB:
      default: return VerifyApplication::SKIPPED;
      }

      BBox3fa bounds0 = node->bounds();
      scene.addGeometry(RTC_GEOMETRY_STATIC,node);
      AssertNoError(device);
      rtcCommit (scene);
      AssertNoError(device);
      BBox3fa bounds1;
      rtcGetBounds(scene,(RTCBounds&)bounds1);
      AssertNoError(device);
      return (VerifyApplication::TestReturnValue)(bounds0 == bounds1);
    }
  };
  
  struct GetLinearBoundsTest : public VerifyApplication::Test
  {
    GeometryType gtype;

    GetLinearBoundsTest (std::string name, int isa, GeometryType gtype)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), gtype(gtype) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      VerifyScene scene(device,RTC_SCENE_STATIC,RTC_INTERSECT1);
      AssertNoError(device);

      Ref<SceneGraph::Node> node = nullptr;
      switch (gtype) {
      case TRIANGLE_MESH   : node = SceneGraph::createTriangleSphere(zero,1.0f,50); break;
      case TRIANGLE_MESH_MB: node = SceneGraph::createTriangleSphere(zero,1.0f,50)->set_motion_vector(Vec3fa(1.0f)); break;
      case QUAD_MESH       : node = SceneGraph::createQuadSphere(zero,1.0f,50); break;
      case QUAD_MESH_MB    : node = SceneGraph::createQuadSphere(zero,1.0f,50)->set_motion_vector(Vec3fa(1.0f)); break;
        //case SUBDIV_MESH:
        //case SUBDIV_MESH_MB:
      default: return VerifyApplication::SKIPPED;
      }

      LBBox3fa bounds0 = node->lbounds();
      scene.addGeometry(RTC_GEOMETRY_STATIC,node);
      AssertNoError(device);
      rtcCommit (scene);
      AssertNoError(device);
      BBox3fa bbox[2];
      rtcGetLinearBounds(scene,(RTCBounds*)bbox);
      LBBox3fa bounds1(bbox[0],bbox[1]);
      AssertNoError(device);
      return (VerifyApplication::TestReturnValue)(bounds0 == bounds1);
    }
  };

  struct GetUserDataTest : public VerifyApplication::Test
  {
    GetUserDataTest (std::string name, int isa)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_STATIC,RTC_INTERSECT1);
      AssertNoError(device);

      unsigned geom0 = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      rtcSetUserData(scene,geom0,(void*)1);
      unsigned geom1 = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 0, 0, 2);
      rtcSetUserData(scene,geom1,(void*)2);
      unsigned geom2 = rtcNewQuadMesh (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      rtcSetUserData(scene,geom2,(void*)3);
      unsigned geom3 = rtcNewQuadMesh (scene, RTC_GEOMETRY_STATIC, 0, 0, 2);
      rtcSetUserData(scene,geom3,(void*)4);
      unsigned geom4 = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC, 0, 0, 0, 0, 0, 0, 1);
      rtcSetUserData(scene,geom4,(void*)5);
      unsigned geom5 = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC, 0, 0, 0, 0, 0, 0, 2);
      rtcSetUserData(scene,geom5,(void*)6);
      unsigned geom6 = rtcNewBezierHairGeometry (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      rtcSetUserData(scene,geom6,(void*)7);
      unsigned geom7 = rtcNewBezierHairGeometry (scene, RTC_GEOMETRY_STATIC, 0, 0, 2);
      rtcSetUserData(scene,geom7,(void*)8);
      unsigned geom8 = rtcNewBezierCurveGeometry (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      rtcSetUserData(scene,geom8,(void*)9);
      unsigned geom9 = rtcNewBezierCurveGeometry (scene, RTC_GEOMETRY_STATIC, 0, 0, 2);
      rtcSetUserData(scene,geom9,(void*)10);
       unsigned geom10 = rtcNewLineSegments (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      rtcSetUserData(scene,geom10,(void*)11);
      unsigned geom11 = rtcNewLineSegments (scene, RTC_GEOMETRY_STATIC, 0, 0, 2);
      rtcSetUserData(scene,geom11,(void*)12);
      unsigned geom12 = rtcNewUserGeometry2(scene,0,1);
      rtcSetUserData(scene,geom12,(void*)13);
      unsigned geom13 = rtcNewUserGeometry2(scene,0,2);
      rtcSetUserData(scene,geom13,(void*)14);
      rtcCommit (scene);
      AssertNoError(device);
      
      if ((size_t)rtcGetUserData(scene,geom0 ) !=  1) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom1 ) !=  2) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom2 ) !=  3) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom3 ) !=  4) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom4 ) !=  5) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom5 ) !=  6) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom6 ) !=  7) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom7 ) !=  8) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom8 ) !=  9) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom9 ) != 10) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom10) != 11) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom11) != 12) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom12) != 13) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom13) != 14) return VerifyApplication::FAILED;
      AssertNoError(device);

      return VerifyApplication::PASSED;
    }
  };

  struct BufferStrideTest : public VerifyApplication::Test
  {
    GeometryType gtype;

    BufferStrideTest (std::string name, int isa, GeometryType gtype)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), gtype(gtype) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_STATIC,aflags);
      AssertNoError(device);

      unsigned geomID = RTC_INVALID_GEOMETRY_ID;
      switch (gtype) {
      case TRIANGLE_MESH    : geomID = rtcNewTriangleMesh   (scene, RTC_GEOMETRY_STATIC, 16, 16, 1); break;
      case TRIANGLE_MESH_MB : geomID = rtcNewTriangleMesh   (scene, RTC_GEOMETRY_STATIC, 16, 16, 2); break;
      case QUAD_MESH        : geomID = rtcNewQuadMesh       (scene, RTC_GEOMETRY_STATIC, 16, 16, 1); break;
      case QUAD_MESH_MB     : geomID = rtcNewQuadMesh       (scene, RTC_GEOMETRY_STATIC, 16, 16, 2); break;
      case SUBDIV_MESH      : geomID = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC,  0, 16, 16, 0,0,0, 1); break;
      case SUBDIV_MESH_MB   : geomID = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC,  0, 16, 16, 0,0,0, 2); break;
      case HAIR_GEOMETRY    : geomID = rtcNewBezierHairGeometry(scene, RTC_GEOMETRY_STATIC, 16, 16, 1); break;
      case HAIR_GEOMETRY_MB : geomID = rtcNewBezierHairGeometry(scene, RTC_GEOMETRY_STATIC, 16, 16, 2); break;
      case CURVE_GEOMETRY   : geomID = rtcNewBezierCurveGeometry   (scene, RTC_GEOMETRY_STATIC, 16, 16, 1); break;
      case CURVE_GEOMETRY_MB: geomID = rtcNewBezierCurveGeometry   (scene, RTC_GEOMETRY_STATIC, 16, 16, 2); break;
      default               : throw std::runtime_error("unknown geometry type: "+to_string(gtype));
      }
      AssertNoError(device);

      avector<char> indexBuffer(8+16*6*sizeof(int));
      avector<char> vertexBuffer(12+16*9*sizeof(float)+4);
      
      rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,indexBuffer.data(),1,3*sizeof(int));
      AssertError(device,RTC_INVALID_OPERATION);
      rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER,vertexBuffer.data(),1,3*sizeof(float));
      AssertError(device,RTC_INVALID_OPERATION);

      rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,indexBuffer.data(),0,3*sizeof(int)+3);
      AssertError(device,RTC_INVALID_OPERATION);
      rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER,vertexBuffer.data(),0,3*sizeof(float)+3);
      AssertError(device,RTC_INVALID_OPERATION);
      
      rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,indexBuffer.data(),0,3*sizeof(int));
      AssertNoError(device);
      rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER,vertexBuffer.data(),0,3*sizeof(float));
      AssertNoError(device);
      
      rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,indexBuffer.data(),8,6*sizeof(int));
      AssertNoError(device);
      rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER,vertexBuffer.data(),12,9*sizeof(float));
      AssertNoError(device);
      
      rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,indexBuffer.data(),0,3*sizeof(int));
      AssertNoError(device);
      
      rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER,vertexBuffer.data(),0,4*sizeof(float));
      AssertNoError(device);
      
      return VerifyApplication::PASSED;
    }
  };

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  struct EmptySceneTest : public VerifyApplication::Test
  {
    EmptySceneTest (std::string name, int isa, RTCSceneFlags sflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,aflags);
      AssertNoError(device);
      rtcCommit (scene);
      AssertNoError(device);
      return VerifyApplication::PASSED;
    }

  public:
    RTCSceneFlags sflags;
  };

  struct EmptyGeometryTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;

    EmptyGeometryTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,aflags);
      rtcNewTriangleMesh (scene,gflags,0,0,1);
      rtcNewTriangleMesh (scene,gflags,0,0,2);
      rtcNewQuadMesh (scene,gflags,0,0,1);
      rtcNewQuadMesh (scene,gflags,0,0,2);
      rtcNewSubdivisionMesh (scene,gflags,0,0,0,0,0,0,1);
      rtcNewSubdivisionMesh (scene,gflags,0,0,0,0,0,0,2);
      rtcNewBezierHairGeometry (scene,gflags,0,0,1);
      rtcNewBezierHairGeometry (scene,gflags,0,0,2);
      rtcNewBezierCurveGeometry (scene,gflags,0,0,1);
      rtcNewBezierCurveGeometry (scene,gflags,0,0,2);
      rtcNewUserGeometry2 (scene,0,1);
      rtcNewUserGeometry2 (scene,0,2);
      rtcCommit (scene);
      AssertNoError(device);
      return VerifyApplication::PASSED;
    }
  };

  struct ManyBuildTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags; 
    
    ManyBuildTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS,false), sflags(sflags), gflags(gflags) {}
    
    VerifyApplication::TestReturnValue run (VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      rtcDeviceSetErrorFunction2(device,errorHandler,nullptr);
      
      std::vector<Vec3f> p;
      p.reserve(4);
      p.push_back(Vec3f(0.0f, 0.0f, 0.0f));
      p.push_back(Vec3f(1.0f, 0.0f, 0.0f));
      p.push_back(Vec3f(0.0f, 0.5f, 1.0f));
      
      std::vector<uint32_t> indices;
      indices.push_back(0);
      indices.push_back(1);
      indices.push_back(2);
      
      const size_t numTriangles = 1;
      const size_t numVertices = 3;
      
      for (size_t i = 0; i < 1024*1024; ++i)
      {
        //if (i%100 == 0) PRINT(i);
        rtcDeviceSetParameter1i(nullptr,(RTCParameter) 1000000, i);
        
        RTCScene scene = rtcDeviceNewScene(device, sflags, RTC_INTERSECT1);
        
        unsigned geomID = rtcNewTriangleMesh(scene, gflags, numTriangles, numVertices, 1);
        rtcSetBuffer2(scene, geomID, RTC_VERTEX_BUFFER, p.data(), 0, 3 * sizeof(float), numVertices);
        rtcSetBuffer2(scene, geomID, RTC_INDEX_BUFFER, indices.data(), 0, 3 * sizeof(uint32_t), numTriangles);
        
        rtcCommit(scene);
      }
      AssertNoError(device);

      return VerifyApplication::PASSED;
    }
  };

  struct BuildTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags; 

    BuildTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}
    
    VerifyApplication::TestReturnValue run (VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      VerifyScene scene(device,sflags,aflags);

      const Vec3fa center = zero;
      const float radius = 1.0f;
      const Vec3fa dx(1,0,0);
      const Vec3fa dy(0,1,0);
      scene.addGeometry(gflags,SceneGraph::createTriangleSphere(center,radius,50));
      scene.addGeometry(gflags,SceneGraph::createTriangleSphere(center,radius,50)->set_motion_vector(random_motion_vector(1.0f)));
      scene.addGeometry(gflags,SceneGraph::createQuadSphere(center,radius,50));
      scene.addGeometry(gflags,SceneGraph::createQuadSphere(center,radius,50)->set_motion_vector(random_motion_vector(1.0f)));
      scene.addGeometry(gflags,SceneGraph::createSubdivSphere(center,radius,8,20));
      scene.addGeometry(gflags,SceneGraph::createSubdivSphere(center,radius,8,20)->set_motion_vector(random_motion_vector(1.0f)));
      scene.addGeometry(gflags,SceneGraph::createHairyPlane(RandomSampler_getInt(sampler),center,dx,dy,0.1f,0.01f,100,SceneGraph::HairSetNode::HAIR));
      scene.addGeometry(gflags,SceneGraph::createHairyPlane(RandomSampler_getInt(sampler),center,dx,dy,0.1f,0.01f,100,SceneGraph::HairSetNode::HAIR)->set_motion_vector(random_motion_vector(1.0f)));
      rtcCommit (scene);
      AssertNoError(device);

      if ((sflags & RTC_SCENE_DYNAMIC) == 0) 
      {
        for (unsigned int i=0; i<8; i++) {
          rtcDisable(scene,i);
          AssertAnyError(device);
        }
      }
      AssertNoError(device);

      return VerifyApplication::PASSED;
    }
  };

  struct OverlappingGeometryTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags; 
    size_t N;
    
    OverlappingGeometryTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, size_t N)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags), N(N) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      VerifyScene scene(device,sflags,aflags);
      AssertNoError(device);

      const Vec3fa p (0,0,0);
      const Vec3fa dx(1,0,0);
      const Vec3fa dy(0,1,0);
      
      Ref<SceneGraph::TriangleMeshNode> trimesh = SceneGraph::createTrianglePlane(p,dx,dy,1,1).dynamicCast<SceneGraph::TriangleMeshNode>();
      for (size_t i=0; i<N; i++) trimesh->triangles.push_back(trimesh->triangles.back());
      scene.addGeometry(gflags,trimesh.dynamicCast<SceneGraph::Node>());

      Ref<SceneGraph::TriangleMeshNode> trimesh2 = SceneGraph::createTrianglePlane(p,dx,dy,1,1)->set_motion_vector(random_motion_vector(1.0f)).dynamicCast<SceneGraph::TriangleMeshNode>();
      for (size_t i=0; i<N; i++) trimesh2->triangles.push_back(trimesh2->triangles.back());
      scene.addGeometry(gflags,trimesh2.dynamicCast<SceneGraph::Node>());

      Ref<SceneGraph::QuadMeshNode> quadmesh = SceneGraph::createQuadPlane(p,dx,dy,1,1).dynamicCast<SceneGraph::QuadMeshNode>();
      for (size_t i=0; i<N; i++) quadmesh->quads.push_back(quadmesh->quads.back());
      scene.addGeometry(gflags,quadmesh.dynamicCast<SceneGraph::Node>());

      Ref<SceneGraph::QuadMeshNode> quadmesh2 = SceneGraph::createQuadPlane(p,dx,dy,1,1)->set_motion_vector(random_motion_vector(1.0f)).dynamicCast<SceneGraph::QuadMeshNode>();
      for (size_t i=0; i<N; i++) quadmesh2->quads.push_back(quadmesh2->quads.back());
      scene.addGeometry(gflags,quadmesh2.dynamicCast<SceneGraph::Node>());

      Ref<SceneGraph::SubdivMeshNode> subdivmesh = new SceneGraph::SubdivMeshNode(nullptr,1);
      for (unsigned i=0; i<unsigned(N); i++) {
        subdivmesh->verticesPerFace.push_back(4);
        subdivmesh->position_indices.push_back(4*i+0);
        subdivmesh->position_indices.push_back(4*i+1);
        subdivmesh->position_indices.push_back(4*i+2);
        subdivmesh->position_indices.push_back(4*i+3);
        subdivmesh->positions[0].push_back(Vec3fa(0,0,0));
        subdivmesh->positions[0].push_back(Vec3fa(0,1,0));
        subdivmesh->positions[0].push_back(Vec3fa(1,1,0));
        subdivmesh->positions[0].push_back(Vec3fa(1,0,0));
      }
      scene.addGeometry(gflags,subdivmesh.dynamicCast<SceneGraph::Node>());

      Ref<SceneGraph::SubdivMeshNode> subdivmesh2 = new SceneGraph::SubdivMeshNode(nullptr,1);
      for (unsigned i=0; i<unsigned(N); i++) {
        subdivmesh2->verticesPerFace.push_back(4);
        subdivmesh2->position_indices.push_back(4*i+0);
        subdivmesh2->position_indices.push_back(4*i+1);
        subdivmesh2->position_indices.push_back(4*i+2);
        subdivmesh2->position_indices.push_back(4*i+3);
        subdivmesh2->positions[0].push_back(Vec3fa(0,0,0));
        subdivmesh2->positions[0].push_back(Vec3fa(0,1,0));
        subdivmesh2->positions[0].push_back(Vec3fa(1,1,0));
        subdivmesh2->positions[0].push_back(Vec3fa(1,0,0));
      }
      subdivmesh2->set_motion_vector(random_motion_vector(1.0f));
      scene.addGeometry(gflags,subdivmesh2.dynamicCast<SceneGraph::Node>());
      
      Ref<SceneGraph::HairSetNode> hairgeom = SceneGraph::createHairyPlane(RandomSampler_getInt(sampler),p,dx,dy,0.2f,0.01f,1,SceneGraph::HairSetNode::HAIR).dynamicCast<SceneGraph::HairSetNode>();
      for (size_t i=0; i<N; i++) hairgeom->hairs.push_back(hairgeom->hairs.back());
      scene.addGeometry(gflags,hairgeom.dynamicCast<SceneGraph::Node>());

      Ref<SceneGraph::HairSetNode> hairgeom2 = SceneGraph::createHairyPlane(RandomSampler_getInt(sampler),p,dx,dy,0.2f,0.01f,1,SceneGraph::HairSetNode::HAIR)->set_motion_vector(random_motion_vector(1.0f)).dynamicCast<SceneGraph::HairSetNode>();
      for (size_t i=0; i<N; i++) hairgeom2->hairs.push_back(hairgeom2->hairs.back());
      scene.addGeometry(gflags,hairgeom2.dynamicCast<SceneGraph::Node>());

      Ref<SceneGraph::Node> curvegeom = SceneGraph::convert_hair_to_curves(hairgeom.dynamicCast<SceneGraph::Node>());
      scene.addGeometry(gflags,curvegeom);

      Ref<SceneGraph::Node> curvegeom2 = SceneGraph::convert_hair_to_curves(hairgeom2.dynamicCast<SceneGraph::Node>());
      scene.addGeometry(gflags,curvegeom2);

      Ref<SceneGraph::Node> linegeom = SceneGraph::convert_bezier_to_lines(hairgeom.dynamicCast<SceneGraph::Node>());
      scene.addGeometry(gflags,linegeom);

      Ref<SceneGraph::Node> linegeom2 = SceneGraph::convert_bezier_to_lines(hairgeom2.dynamicCast<SceneGraph::Node>());
      scene.addGeometry(gflags,linegeom2);
      
      rtcCommit (scene);
      AssertNoError(device);
      return VerifyApplication::PASSED;
    }
  };

  static std::atomic<ssize_t> memory_consumption_bytes_used(0);

  struct MemoryConsumptionTest : public VerifyApplication::Test
  {
    GeometryType gtype;
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    
    MemoryConsumptionTest (std::string name, int isa, GeometryType gtype, RTCSceneFlags sflags, RTCGeometryFlags gflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), gtype(gtype), sflags(sflags), gflags(gflags) {}

    static bool memoryMonitor(void* userPtr, const ssize_t bytes, const bool /*post*/)
    {
      memory_consumption_bytes_used += bytes;
      return true;
    }

    double expected_size_helper(VerifyApplication* state, size_t NN)
    {
      bool avx = (isa & AVX) == AVX;
      bool avx2 = (isa & AVX2) == AVX2;
      switch (gtype)
      {
      case TRIANGLE_MESH: switch (sflags) {
        case RTC_SCENE_STATIC : return avx ?  70.0f*NN :  63.0f*NN; // triangle4
        case RTC_SCENE_ROBUST : return avx ?  70.0f*NN :  63.0f*NN; // triangle4v
        case RTC_SCENE_COMPACT: return avx ?  35.0f*NN :  35.0f*NN; // triangle4i
        case RTC_SCENE_DYNAMIC: return avx ? 131.0f*NN : 117.0f*NN; // triangle4
        default: return inf;
        }
      case TRIANGLE_MESH_MB: switch (sflags) {
        case RTC_SCENE_STATIC : return avx2 ? 55.0f*NN : 45.0f*NN; // triangle4imb
        case RTC_SCENE_ROBUST : return avx2 ? 55.0f*NN : 45.0f*NN; // triangle4imb
        case RTC_SCENE_COMPACT: return avx2 ? 45.0f*NN : 45.0f*NN; // triangle4imb
        default: return inf;
        }
        
      case QUAD_MESH: switch (sflags) {
        case RTC_SCENE_STATIC : return avx ? 85.0f*NN : 79.0f*NN; // quad4v
        case RTC_SCENE_ROBUST : return avx ? 85.0f*NN : 79.0f*NN; // quad4v
        case RTC_SCENE_COMPACT: return avx ? 41.0f*NN : 41.0f*NN; // quad4i
        default: return inf;
        }
      case QUAD_MESH_MB: switch (sflags) {
        case RTC_SCENE_STATIC : return avx ? 68.0f*NN : 53.0f*NN; // quad4imb
        case RTC_SCENE_ROBUST : return avx ? 68.0f*NN : 53.0f*NN; // quad4imb
        case RTC_SCENE_COMPACT: return avx ? 53.0f*NN : 53.0f*NN; // quad4imb
        default: return inf;
        }
      
      case HAIR_GEOMETRY: switch (sflags) {
        case RTC_SCENE_STATIC : return avx2 ?  222.0f*NN : 165.0f*NN; // bezier1v
        case RTC_SCENE_ROBUST : return avx2 ?  222.0f*NN : 165.0f*NN; // bezier1v
        case RTC_SCENE_COMPACT: return avx2 ?  105.0f*NN : 105.0f*NN; // bezier1i
        default: return inf;
        }

      case HAIR_GEOMETRY_MB: switch (sflags) {
        case RTC_SCENE_STATIC : return avx2 ?  386.0f*NN : 190.0f*NN; // bezier1i // FIXME: 386 are very loose bounds
        case RTC_SCENE_ROBUST : return avx2 ?  386.0f*NN : 190.0f*NN; // bezier1i // FIXME: 386 are very loose bounds 
        case RTC_SCENE_COMPACT: return avx2 ?  190.0f*NN : 190.0f*NN; // bezier1i
        default: return inf;
        }
      
      case LINE_GEOMETRY: switch (sflags) {
        case RTC_SCENE_STATIC : return avx ? 32.0f*NN : 26.0f*NN; // line4i
        case RTC_SCENE_ROBUST : return avx ? 32.0f*NN : 26.0f*NN; // line4i
        case RTC_SCENE_COMPACT: return avx ? 26.0f*NN : 26.0f*NN; // line4i
        default: return inf;
        }

      case LINE_GEOMETRY_MB: switch (sflags) {
        case RTC_SCENE_STATIC : return avx ? 45.0f*NN : 36.0f*NN; // line4i
        case RTC_SCENE_ROBUST : return avx ? 45.0f*NN : 36.0f*NN; // line4i
        case RTC_SCENE_COMPACT: return avx ? 36.0f*NN : 36.0f*NN; // line4i
        default: return inf;
        }
      
      default: return inf;
      }
    }

    double expected_size(VerifyApplication* state, size_t NN)
    {
      double bytes_expected = expected_size_helper(state,NN);
      bool use_single_mode = false; //bytes_expected < 100000;
      double mainBlockSize = clamp(bytes_expected/20,1024.0,double(2*1024*1024-64));
      double threadLocalBlockSize = clamp(bytes_expected/20,double(1024),double(PAGE_SIZE));
      double expected = bytes_expected + ceil(bytes_expected/mainBlockSize)*128 + ceil(bytes_expected/threadLocalBlockSize)*128 + mainBlockSize;
      if (use_single_mode == false) expected += mainBlockSize;
      return expected;
    }

    std::pair<ssize_t,ssize_t> run_build(VerifyApplication* state, size_t N, unsigned numThreads)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa) + ",threads="+std::to_string((long long)numThreads);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      memory_consumption_bytes_used = 0;
      rtcDeviceSetMemoryMonitorFunction2(device,memoryMonitor,nullptr);
      VerifyScene scene(device,sflags,aflags);
      AssertNoError(device);
      
      int numPhi = (size_t) ceilf(sqrtf(N/4.0f));
      ssize_t NN = 0;

      float plen = sqrtf(N/100000.0f);
      Vec3fa planeX = plen*normalize(Vec3fa(1,1,0));
      Vec3fa planeY = plen*normalize(Vec3fa(0,1,1));
      
      Ref<SceneGraph::Node> mesh;
      int i = 0;
      switch (gtype) {
      case TRIANGLE_MESH:    
      case TRIANGLE_MESH_MB: mesh = SceneGraph::createTriangleSphere(zero,float(i+1),numPhi); break;
      case QUAD_MESH:        
      case QUAD_MESH_MB:     mesh = SceneGraph::createQuadSphere(zero,float(i+1),numPhi); break;
      case SUBDIV_MESH:      
      case SUBDIV_MESH_MB:   mesh = SceneGraph::createSubdivSphere(zero,float(i+1),8,float(numPhi)/8.0f); break;
      case HAIR_GEOMETRY:    
      case HAIR_GEOMETRY_MB: mesh = SceneGraph::createHairyPlane(i,Vec3fa(float(i)),planeX,planeY,0.01f,0.00001f,4*numPhi*numPhi,SceneGraph::HairSetNode::HAIR); break;
      case LINE_GEOMETRY:    
      case LINE_GEOMETRY_MB: mesh = SceneGraph::createHairyPlane(i,Vec3fa(float(i)),planeX,planeY,0.01f,0.00001f,4*numPhi*numPhi/3,SceneGraph::HairSetNode::HAIR); break;
      default:               throw std::runtime_error("invalid geometry for benchmark");
      }
      
      switch (gtype) {
      case LINE_GEOMETRY:    
      case LINE_GEOMETRY_MB: mesh = SceneGraph::convert_bezier_to_lines(mesh); break;
      default: break;
      }
      
      switch (gtype) {
      case TRIANGLE_MESH_MB: 
      case QUAD_MESH_MB:     
      case SUBDIV_MESH_MB:   
      case HAIR_GEOMETRY_MB: 
      case LINE_GEOMETRY_MB: mesh = mesh->set_motion_vector(random_motion_vector2(0.0001f)); break;
      default: break;
      }
      NN = mesh->numPrimitives(); 
      scene.addGeometry(gflags,mesh);
      rtcCommit (scene);
      AssertNoError(device);
      
      return std::make_pair(NN,memory_consumption_bytes_used.load());
    }
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      VerifyApplication::TestReturnValue ret = VerifyApplication::PASSED;

      size_t maxN = 0;
      switch (gtype) {
      case LINE_GEOMETRY:    
      case LINE_GEOMETRY_MB: 
      case HAIR_GEOMETRY:
      case HAIR_GEOMETRY_MB: maxN = 250000; break;
      default: maxN = 1000000; break;
      }
      
      for (size_t N=128; N<maxN; N = (size_t)((float)N * 1.5f)) 
      {
        auto bytes_one_thread  = run_build(state,N,1);
        auto bytes_all_threads = run_build(state,N,0);
        double bytes_expected = expected_size(state,bytes_one_thread.first);
        double expected_to_single = double(bytes_one_thread.second)/double(bytes_expected);
        double single_to_threaded = double(bytes_all_threads.second)/double(bytes_one_thread.second);
     
        const bool failed0 = expected_to_single > 1.0f;
        const bool failed1 = sflags == RTC_SCENE_DYNAMIC ? single_to_threaded > 1.30f : single_to_threaded > 1.12f;

        if (failed0 || failed1) {
          std::cout << state->red ("-") << std::flush;
          ret = VerifyApplication::FAILED;
        } else {
          std::cout << state->green ("+") << std::flush;
        }

        if (failed0 || failed1)
        {
          double num_primitives = (double)bytes_one_thread.first;
          std::cout << "N = " << num_primitives << ", n = " << ceilf(sqrtf(N/4.0f)) << ", "
            "expected = " << bytes_expected/num_primitives << " B, " << 
            "1 thread = " << bytes_one_thread.second/num_primitives << " B (" << 100.0f*expected_to_single << " %)" << (failed0 ? state->red(" [FAILED]") : "") << ", " << 
            "all_threads = " << bytes_all_threads.second/num_primitives << " B (" << 100.0f*single_to_threaded << " %)" << (failed1 ? state->red(" [FAILED]") : "") << std::endl;
        }
      }
      return ret;
    }
  };
    
  struct NewDeleteGeometryTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;

    NewDeleteGeometryTest (std::string name, int isa, RTCSceneFlags sflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      VerifyScene scene(device,sflags,aflags_all);
      AssertNoError(device);
      int geom[128];
      for (size_t i=0; i<128; i++) geom[i] = -1;
      Sphere spheres[128];
      memset(spheres,0,sizeof(spheres));
      
      for (size_t i=0; i<size_t(50*state->intensity); i++) 
      {
        for (size_t j=0; j<10; j++) {
          int index = random_int()%128;
          Vec3fa pos = 100.0f*random_Vec3fa();
          if (geom[index] == -1) {
            switch (random_int()%9) {
            case 0: geom[index] = scene.addSphere(sampler,RTC_GEOMETRY_STATIC,pos,2.0f,10).first; break;
            case 1: geom[index] = scene.addSphere(sampler,RTC_GEOMETRY_STATIC,pos,2.0f,10,-1,random_motion_vector(1.0f)).first; break;
            case 2: geom[index] = scene.addQuadSphere(sampler,RTC_GEOMETRY_STATIC,pos,2.0f,10).first; break;
            case 3: geom[index] = scene.addQuadSphere(sampler,RTC_GEOMETRY_STATIC,pos,2.0f,10,-1,random_motion_vector(1.0f)).first; break;
            case 4: geom[index] = scene.addHair  (sampler,RTC_GEOMETRY_STATIC,pos,1.0f,2.0f,10).first; break;
            case 5: geom[index] = scene.addHair  (sampler,RTC_GEOMETRY_STATIC,pos,1.0f,2.0f,10,random_motion_vector(1.0f)).first; break;
            case 6: geom[index] = scene.addSubdivSphere(sampler,RTC_GEOMETRY_STATIC,pos,2.0f,4,4).first; break;
            case 7: geom[index] = scene.addSubdivSphere(sampler,RTC_GEOMETRY_STATIC,pos,2.0f,4,4,-1,random_motion_vector(1.0f)).first; break;
            case 8: 
              spheres[index] = Sphere(pos,2.0f);
              geom[index] = scene.addUserGeometryEmpty(sampler,RTC_GEOMETRY_STATIC,&spheres[index]).first; break;
            }
            AssertNoError(device);
          }
          else { 
            rtcDeleteGeometry(scene,geom[index]);     
            AssertNoError(device);
            geom[index] = -1; 
          }
        }
        rtcCommit(scene);
        AssertNoError(device);
        rtcCommit(scene);
        AssertNoError(device);
        if (i%2 == 0) std::cout << "." << std::flush;
      }
      
      /* now delete all geometries */
      for (size_t i=0; i<128; i++) 
        if (geom[i] != -1) rtcDeleteGeometry(scene,geom[i]);
      rtcCommit(scene);
      AssertNoError(device);

      rtcCommit (scene);
      AssertNoError(device);

      return VerifyApplication::PASSED;
    }
  };

  struct UserGeometryIDTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;

    UserGeometryIDTest (std::string name, int isa, RTCSceneFlags sflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      VerifyScene scene(device,sflags,aflags_all);
      AssertNoError(device);

      int geom[128];
      for (size_t i=0; i<128; i++) geom[i] = -1;
      
      for (size_t i=0; i<size_t(50*state->intensity); i++) 
      {
        for (size_t j=0; j<10; j++) 
        {
          int index = random_int()%128;
          if (geom[index] == -1) {
            if (random_bool()) {
              unsigned int geomID = rtcNewTriangleMesh2(scene,RTC_GEOMETRY_STATIC,0,0,1,index);
              geom[geomID] = geomID;
              AssertNoError(device);
            } else {
              unsigned int geomID = rtcNewTriangleMesh(scene,RTC_GEOMETRY_STATIC,0,0,1);
              geom[geomID] = geomID;
              AssertNoError(device);
            }
          } else {
             if (random_bool()) {
               rtcDeleteGeometry(scene,geom[index]);     
               AssertNoError(device);
               geom[index] = -1; 
             }
          }
        }
        rtcCommit(scene);
        AssertNoError(device);
      }
      return VerifyApplication::PASSED;
    }
  };

  struct EnableDisableGeometryTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;

    EnableDisableGeometryTest (std::string name, int isa, RTCSceneFlags sflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      VerifyScene scene(device,sflags,aflags);
      AssertNoError(device);
      unsigned geom0 = scene.addSphere      (sampler,RTC_GEOMETRY_STATIC,Vec3fa(-1,0,-1),1.0f,50).first;
      unsigned geom1 = scene.addQuadSphere  (sampler,RTC_GEOMETRY_STATIC,Vec3fa(-1,0,+1),1.0f,50).first;
      unsigned geom2 = scene.addSubdivSphere(sampler,RTC_GEOMETRY_STATIC,Vec3fa(+1,0,-1),1.0f,5,4).first;
      unsigned geom3 = scene.addHair        (sampler,RTC_GEOMETRY_STATIC,Vec3fa(+1,0,+1),1.0f,1.0f,1).first;
      AssertNoError(device);
      
      for (size_t i=0; i<16; i++) 
      {
        bool enabled0 = i & 1, enabled1 = i & 2, enabled2 = i & 4, enabled3 = i & 8;
        if (enabled0) rtcEnable(scene,geom0); else rtcDisable(scene,geom0); AssertNoError(device);
        if (enabled1) rtcEnable(scene,geom1); else rtcDisable(scene,geom1); AssertNoError(device);
        if (enabled2) rtcEnable(scene,geom2); else rtcDisable(scene,geom2); AssertNoError(device);
        if (enabled3) rtcEnable(scene,geom3); else rtcDisable(scene,geom3); AssertNoError(device);
        rtcCommit (scene);
        AssertNoError(device);
        {
          RTCRay ray0 = makeRay(Vec3fa(-1,10,-1),Vec3fa(0,-1,0));
          RTCRay ray1 = makeRay(Vec3fa(-1,10,+1),Vec3fa(0,-1,0)); 
          RTCRay ray2 = makeRay(Vec3fa(+1,10,-1),Vec3fa(0,-1,0)); 
          RTCRay ray3 = makeRay(Vec3fa(+1,10,+1),Vec3fa(0,-1,0)); 
          rtcIntersect(scene,ray0);
          rtcIntersect(scene,ray1);
          rtcIntersect(scene,ray2);
          rtcIntersect(scene,ray3);
          bool ok0 = enabled0 ? ray0.geomID == 0 : ray0.geomID == RTC_INVALID_GEOMETRY_ID;
          bool ok1 = enabled1 ? ray1.geomID == 1 : ray1.geomID == RTC_INVALID_GEOMETRY_ID;
          bool ok2 = enabled2 ? ray2.geomID == 2 : ray2.geomID == RTC_INVALID_GEOMETRY_ID;
          bool ok3 = enabled3 ? ray3.geomID == 3 : ray3.geomID == RTC_INVALID_GEOMETRY_ID;
          if (!ok0 || !ok1 || !ok2 || !ok3) return VerifyApplication::FAILED;
        }
      }
      AssertNoError(device);

      return VerifyApplication::PASSED;
    }
  };
  
  struct UpdateTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;

    UpdateTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}
    
    static void move_mesh(const VerifyScene& scene, unsigned mesh, size_t numVertices, Vec3fa& pos) 
    {
      Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
      for (size_t i=0; i<numVertices; i++) vertices[i] += Vec3fa(pos);
      rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER);
      rtcUpdate(scene,mesh);
    }

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      if (!supportsIntersectMode(device,imode))
        return VerifyApplication::SKIPPED;
      
      VerifyScene scene(device,sflags,to_aflags(imode));
      AssertNoError(device);
      size_t numPhi = 10;
      size_t numVertices = 2*numPhi*(numPhi+1);
      Vec3fa pos0 = Vec3fa(-10,0,-10);
      Vec3fa pos1 = Vec3fa(-10,0,+10);
      Vec3fa pos2 = Vec3fa(+10,0,-10);
      Vec3fa pos3 = Vec3fa(+10,0,+10);
      unsigned geom0 = scene.addSphere      (sampler,gflags,pos0,1.0f,numPhi).first;
      unsigned geom1 = scene.addQuadSphere  (sampler,gflags,pos1,1.0f,numPhi).first;
      unsigned geom2 = scene.addSubdivSphere(sampler,gflags,pos2,1.0f,numPhi,4).first;
      unsigned geom3 = scene.addSphereHair  (sampler,gflags,pos3,1.0f).first;
      AssertNoError(device);
      
      for (size_t i=0; i<16; i++) 
      {
        bool move0 = i & 1, move1 = i & 2, move2 = i & 4, move3 = i & 8;
        Vec3fa ds(2,0.1f,2);
        if (move0) { move_mesh(scene,geom0,numVertices,ds); pos0 += ds; }
        if (move1) { move_mesh(scene,geom1,numVertices,ds); pos1 += ds; }
        if (move2) { move_mesh(scene,geom2,numVertices,ds); pos2 += ds; }
        if (move3) { move_mesh(scene,geom3,4,ds); pos3 += ds; }
        rtcCommit (scene);
        AssertNoError(device);

        RTCRay ray0 = makeRay(pos0+Vec3fa(0,10,0),Vec3fa(0,-1,0)); // hits geomID == 0
        RTCRay ray1 = makeRay(pos1+Vec3fa(0,10,0),Vec3fa(0,-1,0)); // hits geomID == 1
        RTCRay ray2 = makeRay(pos2+Vec3fa(0,10,0),Vec3fa(0,-1,0)); // hits geomID == 2
        RTCRay ray3 = makeRay(pos3+Vec3fa(0,10,0),Vec3fa(0,-1,0)); // hits geomID == 3
        RTCRay testRays[4] = { ray0, ray1, ray2, ray3 };

        const size_t maxRays = 100;
        RTCRay rays[maxRays];
        for (size_t numRays=1; numRays<maxRays; numRays++) {
          for (size_t i=0; i<numRays; i++) rays[i] = testRays[i%4];
          IntersectWithMode(imode,ivariant,scene,rays,numRays);
          for (size_t i=0; i<numRays; i++) if (rays[i].geomID == RTC_INVALID_GEOMETRY_ID) return VerifyApplication::FAILED;
        }
      }
      AssertNoError(device);

      return VerifyApplication::PASSED;
    }
  };

  struct GarbageGeometryTest : public VerifyApplication::Test
  {
    GarbageGeometryTest (std::string name, int isa)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + "isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));

      for (size_t i=0; i<size_t(1000*state->intensity); i++) 
      {
        RandomSampler_init(sampler,int(i*23565));
        if (i%20 == 0) std::cout << "." << std::flush;
        
        RTCSceneFlags sflag = getSceneFlag(i); 
        VerifyScene scene(device,sflag,aflags);
        AssertNoError(device);
        
        for (size_t j=0; j<20; j++) 
        {
          size_t numPrimitives = random_int()%256;
          switch (random_int()%8) {
          case 0: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageTriangleMesh(random_int(),numPrimitives,false)); break;
          case 1: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageTriangleMesh(random_int(),numPrimitives,true )); break;
          case 2: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageQuadMesh(random_int(),numPrimitives,false)); break;
          case 3: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageQuadMesh(random_int(),numPrimitives,true )); break;
          case 4: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageHair(random_int(),numPrimitives,false)); break;
          case 5: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageHair(random_int(),numPrimitives,true )); break;
          case 6: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageLineSegments(random_int(),numPrimitives,false)); break;
          case 7: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageLineSegments(random_int(),numPrimitives,true )); break;
            //case 8: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageSubdivMesh(random_int(),numPrimitives,false)); break; // FIXME: not working yet
            //case 9: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageSubdivMesh(random_int(),numPrimitives,true )); break;
          }
          AssertNoError(device);
        }
        
        rtcCommit(scene);
        AssertNoError(device);
      }
      AssertNoError(device);

      return VerifyApplication::PASSED;
    }
  };

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  const size_t num_interpolation_vertices = 16;
  const size_t num_interpolation_quad_faces = 9;
  const size_t num_interpolation_triangle_faces = 18;

  __aligned(16) float interpolation_vertices[num_interpolation_vertices*3] = {
    -1.0f, -1.0f, 0.0f,
    0.0f, -1.0f, 0.0f, 
    +1.0f, -1.0f, 0.0f,
    +2.0f, -1.0f, 0.0f,
    
    -1.0f,  0.0f, 0.0f,
    0.0f,  0.0f, 0.0f,
    +1.0f,  0.0f, 0.0f,
    +2.0f,  0.0f, 0.0f,

    -1.0f, +1.0f, 0.0f,
    0.0f, +1.0f, 0.0f,
    +1.0f, +1.0f, 0.0f,
    +2.0f, +1.0f, 0.0f,

    -1.0f, +2.0f, 0.0f,
    0.0f, +2.0f, 0.0f,
    +1.0f, +2.0f, 0.0f,
    +2.0f, +2.0f, 0.0f,
  };

  __aligned(16) int interpolation_quad_indices[num_interpolation_quad_faces*4] = {
    0, 1, 5, 4,
    1, 2, 6, 5,
    2, 3, 7, 6,
    4, 5, 9, 8, 
    5, 6, 10, 9,
    6, 7, 11, 10,
    8, 9, 13, 12,
    9, 10, 14, 13,
    10, 11, 15, 14
  };

  __aligned(16) int interpolation_triangle_indices[num_interpolation_triangle_faces*3] = {
    0, 1, 5,  0, 5, 4,
    1, 2, 6,  1, 6, 5,
    2, 3, 7,  2, 7, 6,
    4, 5, 9,  4, 9, 8, 
    5, 6, 10,  5, 10, 9,
    6, 7, 11,  6, 11, 10,
    8, 9, 13,   8, 13, 12,
    9, 10, 14,   9, 14, 13,
    10, 11, 15,  10, 15, 14
  };

  __aligned(16) int interpolation_quad_faces[num_interpolation_quad_faces] = {
    4, 4, 4, 4, 4, 4, 4, 4, 4
  };

  __aligned(16) float interpolation_vertex_crease_weights[2] = {
    inf, inf
  };

  __aligned(16) unsigned int interpolation_vertex_crease_indices[2] = {
    12, 15
  };

  __aligned(16) float interpolation_edge_crease_weights[3] = {
    inf, inf, inf
  };

  __aligned(16) unsigned int interpolation_edge_crease_indices[6] = 
  {
    8,9, 9,10, 10,11
  };

  struct InterpolateSubdivTest : public VerifyApplication::Test
  {
    size_t N;
    
    InterpolateSubdivTest (std::string name, int isa, size_t N)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), N(N) {}

    bool checkInterpolation2D(const RTCSceneRef& scene, int geomID, int primID, float u, float v, int v0, RTCBufferType buffer, float* data, size_t N, size_t N_total)
    {
      assert(N < 256);
      bool passed = true;
      float P[256], dPdu[256], dPdv[256];
      rtcInterpolate(scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,N);
      
      for (size_t i=0; i<N; i++) {
        float p0 = (1.0f/6.0f)*(1.0f*data[(v0-4-1)*N_total+i] + 4.0f*data[(v0-4+0)*N_total+i] + 1.0f*data[(v0-4+1)*N_total+i]);
        float p1 = (1.0f/6.0f)*(1.0f*data[(v0+0-1)*N_total+i] + 4.0f*data[(v0+0+0)*N_total+i] + 1.0f*data[(v0+0+1)*N_total+i]);
        float p2 = (1.0f/6.0f)*(1.0f*data[(v0+4-1)*N_total+i] + 4.0f*data[(v0+4+0)*N_total+i] + 1.0f*data[(v0+4+1)*N_total+i]);
        float p = (1.0f/6.0f)*(1.0f*p0+4.0f*p1+1.0f*p2);
        passed &= fabsf(p-P[i]) < 1E-4f;
      }
      return passed;
    }
    
    bool checkInterpolation1D(const RTCSceneRef& scene, int geomID, int primID, float u, float v, int v0, int v1, int v2, RTCBufferType buffer, float* data, size_t N, size_t N_total)
    {
      assert(N < 256);
      bool passed = true;
      float P[256], dPdu[256], dPdv[256];
      rtcInterpolate(scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,N);
      
      for (size_t i=0; i<N; i++) {
        float v = (1.0f/6.0f)*(1.0f*data[v0*N_total+i] + 4.0f*data[v1*N_total+i] + 1.0f*data[v2*N_total+i]);
        passed &= fabsf(v-P[i]) < 0.001f;
      }
      return passed;
    }
    
    bool checkInterpolationSharpVertex(const RTCSceneRef& scene, int geomID, int primID, float u, float v, int v0, RTCBufferType buffer, float* data, size_t N, size_t N_total)
    {
      assert(N < 256);
      bool passed = true;
      float P[256], dPdu[256], dPdv[256];
      rtcInterpolate(scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,N);
      
      for (size_t i=0; i<N; i++) {
        float v = data[v0*N_total+i];
        passed &= fabs(v-P[i]) < 1E-3f;
      }
      return passed;
    }
    
    bool checkSubdivInterpolation(const RTCDeviceRef& device, const RTCSceneRef& scene, int geomID, RTCBufferType buffer, float* vertices0, size_t N, size_t N_total)
    {
      rtcSetSubdivisionMode(scene,geomID,0,RTC_SUBDIV_SMOOTH_BOUNDARY);
      AssertNoError(device);
      rtcDisable(scene,geomID);
      AssertNoError(device);
      rtcCommit(scene);
      AssertNoError(device);
      bool passed = true;
      passed &= checkInterpolation1D(scene,geomID,0,0.0f,0.0f,4,0,1,buffer,vertices0,N,N_total);
      passed &= checkInterpolation1D(scene,geomID,2,1.0f,0.0f,2,3,7,buffer,vertices0,N,N_total);
      
      passed &= checkInterpolation2D(scene,geomID,3,1.0f,0.0f,5,buffer,vertices0,N,N_total);
      passed &= checkInterpolation2D(scene,geomID,1,1.0f,1.0f,6,buffer,vertices0,N,N_total);
      
      //passed &= checkInterpolation1D(scene,geomID,3,1.0f,1.0f,8,9,10,buffer,vertices0,N,N_total);
      //passed &= checkInterpolation1D(scene,geomID,7,1.0f,0.0f,9,10,11,buffer,vertices0,N,N_total);
      
      passed &= checkInterpolationSharpVertex(scene,geomID,6,0.0f,1.0f,12,buffer,vertices0,N,N_total);
      passed &= checkInterpolationSharpVertex(scene,geomID,8,1.0f,1.0f,15,buffer,vertices0,N,N_total);
      
      rtcSetSubdivisionMode(scene,geomID,0,RTC_SUBDIV_PIN_CORNERS);
      AssertNoError(device);
      rtcCommit(scene);
      AssertNoError(device);
      
      passed &= checkInterpolationSharpVertex(scene,geomID,0,0.0f,0.0f,0,buffer,vertices0,N,N_total);
      passed &= checkInterpolationSharpVertex(scene,geomID,2,1.0f,0.0f,3,buffer,vertices0,N,N_total);
      return passed;
    }
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      size_t M = num_interpolation_vertices*N+16; // padds the arrays with some valid data
      
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_DYNAMIC,RTC_INTERPOLATE);
      AssertNoError(device);
      unsigned int geomID = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC, num_interpolation_quad_faces, num_interpolation_quad_faces*4, num_interpolation_vertices, 3, 2, 0, 1);
      AssertNoError(device);
      
      rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  interpolation_quad_indices , 0, sizeof(unsigned int));
      rtcSetBuffer(scene, geomID, RTC_FACE_BUFFER,   interpolation_quad_faces,    0, sizeof(unsigned int));
      rtcSetBuffer(scene, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,   interpolation_edge_crease_indices,  0, 2*sizeof(unsigned int));
      rtcSetBuffer(scene, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,  interpolation_edge_crease_weights,  0, sizeof(float));
      rtcSetBuffer(scene, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER, interpolation_vertex_crease_indices,0, sizeof(unsigned int));
      rtcSetBuffer(scene, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER,interpolation_vertex_crease_weights,0, sizeof(float));
      AssertNoError(device);
      
      std::vector<float> vertices0(M);
      for (size_t i=0; i<M; i++) vertices0[i] = random_float();
      rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER0, vertices0.data(), 0, N*sizeof(float));
      AssertNoError(device);
      
      /*std::vector<float> vertices1(M);
        for (size_t i=0; i<M; i++) vertices1[i] = random_float();
        rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER1, vertices1.data(), 0, N*sizeof(float));
        AssertNoError(device);*/
      
      std::vector<float> user_vertices0(M);
      for (size_t i=0; i<M; i++) user_vertices0[i] = random_float();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER0, user_vertices0.data(), 0, N*sizeof(float));
      AssertNoError(device);
      
      std::vector<float> user_vertices1(M);
      for (size_t i=0; i<M; i++) user_vertices1[i] = random_float();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER1, user_vertices1.data(), 0, N*sizeof(float));
      AssertNoError(device);
      
      bool passed = true;
      passed &= checkSubdivInterpolation(device,scene,geomID,RTC_VERTEX_BUFFER0,vertices0.data(),N,N);
      //passed &= checkSubdivInterpolation(device,scene,geomID,RTC_VERTEX_BUFFER1,vertices1.data(),N,N);
      passed &= checkSubdivInterpolation(device,scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0.data(),N,N);
      passed &= checkSubdivInterpolation(device,scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1.data(),N,N);
      
      passed &= checkSubdivInterpolation(device,scene,geomID,RTC_VERTEX_BUFFER0,vertices0.data(),1,N);
      //passed &= checkSubdivInterpolation(device,scene,geomID,RTC_VERTEX_BUFFER1,vertices1.data(),1,N);
      passed &= checkSubdivInterpolation(device,scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0.data(),1,N);
      passed &= checkSubdivInterpolation(device,scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1.data(),1,N);
      
      AssertNoError(device);

      return (VerifyApplication::TestReturnValue) passed;
    }
  };

  struct InterpolateTrianglesTest : public VerifyApplication::Test
  {
    size_t N;
    
    InterpolateTrianglesTest (std::string name, int isa, size_t N)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), N(N) {}
    
    bool checkTriangleInterpolation(const RTCSceneRef& scene, int geomID, int primID, float u, float v, int v0, int v1, int v2, RTCBufferType buffer, float* data, size_t N, size_t N_total)
    {
      assert(N<256);
      bool passed = true;
      float P[256], dPdu[256], dPdv[256];
      rtcInterpolate(scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,N);
      
      for (size_t i=0; i<N; i++) {
        float p0 = data[v0*N_total+i];
        float p1 = data[v1*N_total+i];
        float p2 = data[v2*N_total+i];
        float p = (1.0f-u-v)*p0 + u*p1 + v*p2;
        passed &= fabs(p-P[i]) < 1E-4f;
      }
      return passed;
    }
    
    bool checkTriangleInterpolation(const RTCSceneRef& scene, int geomID, RTCBufferType buffer, float* vertices0, size_t N, size_t N_total)
    {
      bool passed = true;
      passed &= checkTriangleInterpolation(scene,geomID,0,0.0f,0.0f,0,1,5,buffer,vertices0,N,N_total);
      passed &= checkTriangleInterpolation(scene,geomID,0,0.5f,0.5f,0,1,5,buffer,vertices0,N,N_total);
      passed &= checkTriangleInterpolation(scene,geomID,17,0.0f,0.0f,10,15,14,buffer,vertices0,N,N_total);
      passed &= checkTriangleInterpolation(scene,geomID,17,0.5f,0.5f,10,15,14,buffer,vertices0,N,N_total);
      return passed;
    }

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));

      size_t M = num_interpolation_vertices*N+16; // padds the arrays with some valid data
      
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_DYNAMIC,RTC_INTERPOLATE);
      AssertNoError(device);
      unsigned int geomID = rtcNewTriangleMesh(scene, RTC_GEOMETRY_STATIC, num_interpolation_triangle_faces, num_interpolation_vertices, 1);
      AssertNoError(device);
      
      rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  interpolation_triangle_indices , 0, 3*sizeof(unsigned int));
      AssertNoError(device);
      
      std::vector<float> vertices0(M);
      for (size_t i=0; i<M; i++) vertices0[i] = random_float();
      rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER0, vertices0.data(), 0, N*sizeof(float));
      AssertNoError(device);
      
      /*std::vector<float> vertices1(M);
        for (size_t i=0; i<M; i++) vertices1[i] = random_float();
        rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER1, vertices1.data(), 0, N*sizeof(float));
        AssertNoError(device);*/
      
      std::vector<float> user_vertices0(M);
      for (size_t i=0; i<M; i++) user_vertices0[i] = random_float();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER0, user_vertices0.data(), 0, N*sizeof(float));
      AssertNoError(device);
      
      std::vector<float> user_vertices1(M);
      for (size_t i=0; i<M; i++) user_vertices1[i] = random_float();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER1, user_vertices1.data(), 0, N*sizeof(float));
      AssertNoError(device);
      
      rtcDisable(scene,geomID);
      AssertNoError(device);
      rtcCommit(scene);
      AssertNoError(device);
      
      bool passed = true;
      passed &= checkTriangleInterpolation(scene,geomID,RTC_VERTEX_BUFFER0,vertices0.data(),N,N);
      //passed &= checkTriangleInterpolation(scene,geomID,RTC_VERTEX_BUFFER1,vertices1.data(),N,N);
      passed &= checkTriangleInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0.data(),N,N);
      passed &= checkTriangleInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1.data(),N,N);
      
      passed &= checkTriangleInterpolation(scene,geomID,RTC_VERTEX_BUFFER0,vertices0.data(),1,N);
      //passed &= checkTriangleInterpolation(scene,geomID,RTC_VERTEX_BUFFER1,vertices1.data(),1,N);
      passed &= checkTriangleInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0.data(),1,N);
      passed &= checkTriangleInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1.data(),1,N);
      
      AssertNoError(device);

      return (VerifyApplication::TestReturnValue) passed;
    }
  };
  
  const size_t num_interpolation_hair_vertices = 13;
  const size_t num_interpolation_hairs = 4;

  __aligned(16) int interpolation_hair_indices[num_interpolation_hairs] = {
    0, 3, 6, 9
  };

  struct InterpolateHairTest : public VerifyApplication::Test
  {
    size_t N;
    
    InterpolateHairTest (std::string name, int isa, size_t N)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), N(N) {}
    
    bool checkHairInterpolation(const RTCSceneRef& scene, int geomID, int primID, float u, float v, int v0, RTCBufferType buffer, float* data, size_t N, size_t N_total)
    {
      assert(N<256);
      bool passed = true;
      float P[256], dPdu[256], dPdv[256];
      rtcInterpolate(scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,N);
      
      for (size_t i=0; i<N; i++) {
        const float p00 = data[(v0+0)*N_total+i];
        const float p01 = data[(v0+1)*N_total+i];
        const float p02 = data[(v0+2)*N_total+i];
        const float p03 = data[(v0+3)*N_total+i];
        const float t0 = 1.0f - u, t1 = u;
        const float p10 = p00 * t0 + p01 * t1;
        const float p11 = p01 * t0 + p02 * t1;
        const float p12 = p02 * t0 + p03 * t1;
        const float p20 = p10 * t0 + p11 * t1;
        const float p21 = p11 * t0 + p12 * t1;
        const float p30 = p20 * t0 + p21 * t1;
        passed &= fabs(p30-P[i]) < 1E-4f;
      }
      return passed;
    }
    
    bool checkHairInterpolation(const RTCSceneRef& scene, int geomID, RTCBufferType buffer, float* vertices0, size_t N, size_t N_total)
    {
      bool passed = true;
      passed &= checkHairInterpolation(scene,geomID,0,0.0f,0.0f,0,buffer,vertices0,N,N_total);
      passed &= checkHairInterpolation(scene,geomID,1,0.5f,0.0f,3,buffer,vertices0,N,N_total);
      passed &= checkHairInterpolation(scene,geomID,2,0.0f,0.0f,6,buffer,vertices0,N,N_total);
      passed &= checkHairInterpolation(scene,geomID,3,0.2f,0.0f,9,buffer,vertices0,N,N_total);
      return passed;
    }
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));

      size_t M = num_interpolation_vertices*N+16; // padds the arrays with some valid data
      
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_DYNAMIC,RTC_INTERPOLATE);
      AssertNoError(device);
      unsigned int geomID = rtcNewBezierHairGeometry(scene, RTC_GEOMETRY_STATIC, num_interpolation_hairs, num_interpolation_hair_vertices, 1);
      AssertNoError(device);
      
      rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  interpolation_hair_indices , 0, sizeof(unsigned int));
      AssertNoError(device);
      
      std::vector<float> vertices0(M);
      for (size_t i=0; i<M; i++) vertices0[i] = random_float();
      rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER0, vertices0.data(), 0, N*sizeof(float));
      AssertNoError(device);
      
      /*std::vector<float> vertices1(M);
        for (size_t i=0; i<M; i++) vertices1[i] = random_float();
        rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER1, vertices1.data(), 0, N*sizeof(float));
        AssertNoError(device);*/
      
      std::vector<float> user_vertices0(M);
      for (size_t i=0; i<M; i++) user_vertices0[i] = random_float();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER0, user_vertices0.data(), 0, N*sizeof(float));
      AssertNoError(device);
      
      std::vector<float> user_vertices1(M);
      for (size_t i=0; i<M; i++) user_vertices1[i] = random_float();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER1, user_vertices1.data(), 0, N*sizeof(float));
      AssertNoError(device);
      
      rtcDisable(scene,geomID);
      AssertNoError(device);
      rtcCommit(scene);
      AssertNoError(device);
      
      bool passed = true;
      passed &= checkHairInterpolation(scene,geomID,RTC_VERTEX_BUFFER0,vertices0.data(),N,N);
      //passed &= checkHairInterpolation(scene,geomID,RTC_VERTEX_BUFFER1,vertices1.data(),N,N);
      passed &= checkHairInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0.data(),N,N);
      passed &= checkHairInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1.data(),N,N);
      
      passed &= checkHairInterpolation(scene,geomID,RTC_VERTEX_BUFFER0,vertices0.data(),1,N);
      //passed &= checkHairInterpolation(scene,geomID,RTC_VERTEX_BUFFER1,vertices1.data(),1,N);
      passed &= checkHairInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0.data(),1,N);
      passed &= checkHairInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1.data(),1,N);
      
      AssertNoError(device);

      return (VerifyApplication::TestReturnValue) passed;
    }
  };

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  struct TriangleHitTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags; 
    RTCGeometryFlags gflags; 

    TriangleHitTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}

    inline Vec3fa uniformSampleTriangle(const Vec3fa &a, const Vec3fa &b, const Vec3fa &c, float &u, float& v)
    {
      const float su = sqrtf(u);
      v *= su;
      u = 1.0f-su;
      if (u < 0.001f || v < 0.001f || (u+v) > 0.999f) { u = 0.333f; v = 0.333f; } // avoid hitting edges
      return c + u * (a-c) + v * (b-c);
    }

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      if (!supportsIntersectMode(device,imode))
        return VerifyApplication::SKIPPED;
     
      Vec3f vertices[4] = {
        Vec3f(0.0f,0.0f,0.0f),
        Vec3f(1.0f,0.0f,0.0f),
        Vec3f(0.0f,1.0f,0.0f),
        Vec3f(zero) // dummy vertex for 16 byte padding
      };
      Triangle triangles[1] = {
        Triangle(0,1,2)
      };
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,to_aflags(imode));
      int geomID = rtcNewTriangleMesh (scene, gflags, 1, 3);
      rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER, vertices , 0, sizeof(Vec3f));
      rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER , triangles, 0, sizeof(Triangle));
      rtcCommit (scene);
      AssertNoError(device);

      float u[256], v[256];
      RTCRay rays[256];
      for (size_t i=0; i<256; i++)
      {
        u[i] = random_float(); 
        v[i] = random_float();
        Vec3fa from(0.0f,0.0f,-1.0f);
        Vec3fa to = uniformSampleTriangle(vertices[1],vertices[2],vertices[0],u[i],v[i]);
        rays[i] = makeRay(from,to-from);
      }
      IntersectWithMode(imode,ivariant,scene,rays,256);

      for (size_t i=0; i<256; i++)
      {
        if (rays[i].geomID != 0) return VerifyApplication::FAILED;
        
        if (ivariant & VARIANT_OCCLUDED) continue;

        if (rays[i].primID != 0) return VerifyApplication::FAILED;
        if (abs(rays[i].u - u[i]) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        if (abs(rays[i].v - v[i]) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        if (abs(rays[i].tfar - 1.0f) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        const Vec3fa org(rays[i].org[0],rays[i].org[1],rays[i].org[2]);
        const Vec3fa dir(rays[i].dir[0],rays[i].dir[1],rays[i].dir[2]);
        const Vec3fa ht  = org + rays[i].tfar*dir;
        const Vec3fa huv = vertices[0] + rays[i].u*(vertices[1]-vertices[0]) + rays[i].v*(vertices[2]-vertices[0]);
        if (reduce_max(abs(ht-huv)) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        const Vec3fa Ng = Vec3fa(rays[i].Ng[0],rays[i].Ng[1],rays[i].Ng[2]);
        if (reduce_max(abs(Ng - Vec3fa(0.0f,0.0f,-1.0f))) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
      }
      AssertNoError(device);

      return VerifyApplication::PASSED;
    }
  };

  struct QuadHitTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags; 
    RTCGeometryFlags gflags; 

    QuadHitTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      if (!supportsIntersectMode(device,imode))
        return VerifyApplication::SKIPPED;
     
      Vec3f vertices[5] = {
        Vec3f(0.0f,0.0f,0.0f),
        Vec3f(1.0f,0.0f,0.0f),
        Vec3f(1.0f,1.0f,0.0f),
        Vec3f(0.0f,1.0f,0.0f),
        Vec3f(zero) // dummy vertex for 16 byte padding
      };
      int quads[4] = {
        0,1,2,3
      };
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,to_aflags(imode));
      int geomID = rtcNewQuadMesh (scene, gflags, 1, 4);
      rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER, vertices , 0, sizeof(Vec3f));
      rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER , quads, 0, 4*sizeof(int));
      rtcCommit (scene);
      AssertNoError(device);

      float u[256], v[256];
      RTCRay rays[256];
      for (size_t i=0; i<256; i++)
      {
        u[i] = random_float(); 
        v[i] = random_float();
        if (u[i] < 0.001f || v[i] < 0.001f || u[i] > 0.999f || v[i] > 0.999f) { u[i] = 0.333f; v[i] = 0.333f; } // avoid hitting edges
        Vec3fa from(0.0f,0.0f,-1.0f);
        Vec3fa to = vertices[0] + u[i]*(vertices[1]-vertices[0]) + v[i]*(vertices[3]-vertices[0]);
        rays[i] = makeRay(from,to-from);
      }
      IntersectWithMode(imode,ivariant,scene,rays,256);

      for (size_t i=0; i<256; i++)
      {
        if (rays[i].geomID != 0) return VerifyApplication::FAILED;
        if (ivariant & VARIANT_OCCLUDED) continue;
        if (rays[i].primID != 0) return VerifyApplication::FAILED;
        if (abs(rays[i].u - u[i]) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        if (abs(rays[i].v - v[i]) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        if (abs(rays[i].tfar - 1.0f) > 16.0f*float(ulp)) return VerifyApplication::FAILED;

        const Vec3fa org(rays[i].org[0],rays[i].org[1],rays[i].org[2]);
        const Vec3fa dir(rays[i].dir[0],rays[i].dir[1],rays[i].dir[2]);
        const Vec3fa ht  = org + rays[i].tfar*dir;
        const Vec3fa huv = vertices[0] + rays[i].u*(vertices[1]-vertices[0]) + rays[i].v*(vertices[3]-vertices[0]);
        if (reduce_max(abs(ht-huv)) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        const Vec3fa Ng = Vec3fa(rays[i].Ng[0],rays[i].Ng[1],rays[i].Ng[2]);
        if (reduce_max(abs(Ng - Vec3fa(0.0f,0.0f,-1.0f))) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
      }
      AssertNoError(device);
      
      return VerifyApplication::PASSED;
    }
  };
  
  struct RayMasksTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags; 
    RTCGeometryFlags gflags; 

    RayMasksTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      if (!supportsIntersectMode(device,imode))
        return VerifyApplication::SKIPPED;

      bool passed = true;
      Vec3fa pos0 = Vec3fa(-10,0,-10);
      Vec3fa pos1 = Vec3fa(-10,0,+10);
      Vec3fa pos2 = Vec3fa(+10,0,-10);
      Vec3fa pos3 = Vec3fa(+10,0,+10);
      
      VerifyScene scene(device,sflags,to_aflags(imode));
      unsigned int geom0 = scene.addSphere      (sampler,gflags,pos0,1.0f,50).first;
      unsigned int geom1 = scene.addQuadSphere  (sampler,gflags,pos1,1.0f,50).first; 
      unsigned int geom2 = scene.addSubdivSphere(sampler,gflags,pos2,1.0f,5,4).first;
      unsigned int geom3 = scene.addHair        (sampler,gflags,pos3,1.0f,1.0f,1).first;
      rtcSetMask(scene,geom0,1);
      rtcSetMask(scene,geom1,2);
      rtcSetMask(scene,geom2,4);
      rtcSetMask(scene,geom3,8);
      rtcCommit (scene);
      AssertNoError(device);
      
      for (unsigned i=0; i<16; i++) 
      {
        unsigned mask0 = i;
        unsigned mask1 = i+1;
        unsigned mask2 = i+2;
        unsigned mask3 = i+3;
        unsigned masks[4] = { mask0, mask1, mask2, mask3 };
        RTCRay ray0 = makeRay(pos0+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray0.mask = mask0;
        RTCRay ray1 = makeRay(pos1+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray1.mask = mask1;
        RTCRay ray2 = makeRay(pos2+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray2.mask = mask2;
        RTCRay ray3 = makeRay(pos3+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray3.mask = mask3;
        RTCRay rays[4] = { ray0, ray1, ray2, ray3 };
        IntersectWithMode(imode,ivariant,scene,rays,4);
        for (size_t j=0; j<4; j++)
          passed &= masks[j] & (1<<j) ? rays[j].geomID != RTC_INVALID_GEOMETRY_ID : rays[j].geomID == RTC_INVALID_GEOMETRY_ID;
      }
      AssertNoError(device);

      return (VerifyApplication::TestReturnValue) passed;
    }
  };

  struct BackfaceCullingTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    GeometryType gtype;

    BackfaceCullingTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, GeometryType gtype, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags), gtype(gtype) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      if (!supportsIntersectMode(device,imode))
        return VerifyApplication::SKIPPED;
       
      /* create triangle that is front facing for a right handed 
         coordinate system if looking along the z direction */
      VerifyScene scene(device,sflags,to_aflags(imode));
      AssertNoError(device);
      const Vec3fa p0 = Vec3fa(0.0f);
      const Vec3fa dx = Vec3fa(1.0f,0.0f,0.0f);
      const Vec3fa dy = Vec3fa(0.0f,1.0f,0.0f);
      switch (gtype) {
      case TRIANGLE_MESH:    scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createTrianglePlane(p0,dx,dy,1,1)); break;
      case TRIANGLE_MESH_MB: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createTrianglePlane(p0,dx,dy,1,1)->set_motion_vector(random_motion_vector(1.0f))); break;
      case QUAD_MESH:        scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createQuadPlane(p0,dx,dy,1,1)); break;
      case QUAD_MESH_MB:     scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createQuadPlane(p0,dx,dy,1,1)->set_motion_vector(random_motion_vector(1.0f))); break;
      case SUBDIV_MESH:      scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createSubdivPlane(p0,dx,dy,1,1,4.0f)); break;
      case SUBDIV_MESH_MB:   scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createSubdivPlane(p0,dx,dy,1,1,4.0f)->set_motion_vector(random_motion_vector(1.0f))); break;
      default:               throw std::runtime_error("unsupported geometry type: "+to_string(gtype)); 
      }
      
      AssertNoError(device);
      rtcCommit (scene);
      AssertNoError(device);

      const size_t numRays = 1000;
      RTCRay rays[numRays];
      bool passed = true;

      for (size_t i=0; i<numRays; i++) {
        const float rx = random_float();
        const float ry = random_float();
        if (i%2) rays[i] = makeRay(Vec3fa(rx,ry,+1),Vec3fa(0,0,-1)); 
        else     rays[i] = makeRay(Vec3fa(rx,ry,-1),Vec3fa(0,0,+1)); 
      }
      
      IntersectWithMode(imode,ivariant,scene,rays,numRays);
      
      for (size_t i=0; i<numRays; i++) 
      {
        Vec3fa dir(rays[i].dir[0],rays[i].dir[1],rays[i].dir[2]);
        Vec3fa Ng (rays[i].Ng[0], rays[i].Ng[1], rays[i].Ng[2]);
        if (i%2) passed &= rays[i].geomID == RTC_INVALID_GEOMETRY_ID;
        else {
          passed &= rays[i].geomID == 0;
          if (ivariant & VARIANT_INTERSECT)
            passed &= dot(dir,Ng) < 0.0f;
        }
      }
      AssertNoError(device);

      return (VerifyApplication::TestReturnValue) passed;
    }
  };

  struct IntersectionFilterTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    bool subdiv;

    IntersectionFilterTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, bool subdiv, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags), subdiv(subdiv) {}
    
    static void intersectionFilter1(void* userGeomPtr, RTCRay& ray) 
    {
      if ((size_t)userGeomPtr != 123) 
        return;
      
      if (ray.primID & 2)
        ray.geomID = RTC_INVALID_GEOMETRY_ID;
    }
    
    static void intersectionFilter4(const void* valid_i, void* userGeomPtr, RTCRay4& ray) 
    {
      if ((size_t)userGeomPtr != 123) 
        return;
      
      int* valid = (int*)valid_i;
      for (size_t i=0; i<4; i++)
        if (valid[i] == -1)
          if (ray.primID[i] & 2) 
            ray.geomID[i] = RTC_INVALID_GEOMETRY_ID;
    }
    
    static void intersectionFilter8(const void* valid_i, void* userGeomPtr, RTCRay8& ray) 
    {
      if ((size_t)userGeomPtr != 123) 
        return;
      
      int* valid = (int*)valid_i;
      for (size_t i=0; i<8; i++)
        if (valid[i] == -1)
          if (ray.primID[i] & 2) 
            ray.geomID[i] = RTC_INVALID_GEOMETRY_ID;
    }
    
    static void intersectionFilter16(const void* valid_i, void* userGeomPtr, RTCRay16& ray) 
    {
      if ((size_t)userGeomPtr != 123) 
        return;
      
      int* valid = (int*)valid_i;
      for (size_t i=0; i<16; i++)
	if (valid[i] == -1)
	  if (ray.primID[i] & 2) 
	    ray.geomID[i] = RTC_INVALID_GEOMETRY_ID;
    }
    
    static void intersectionFilterN(int* valid,
                                    void* userGeomPtr,
                                    const RTCIntersectContext* context,
                                    RTCRayN* ray,
                                    const RTCHitN* potentialHit,
                                    const size_t N)
    {
      if ((size_t)userGeomPtr != 123) 
        return;

      for (size_t i=0; i<N; i++)
      {
	if (valid[i] != -1) continue;

        /* reject hit */
        if (RTCHitN_primID(potentialHit,N,i) & 2) {
          valid[i] = 0;
        }

        /* accept hit */
        else {
          RTCRayN_instID(ray,N,i) = RTCHitN_instID(potentialHit,N,i);
          RTCRayN_geomID(ray,N,i) = RTCHitN_geomID(potentialHit,N,i);
          RTCRayN_primID(ray,N,i) = RTCHitN_primID(potentialHit,N,i);
          
          RTCRayN_u(ray,N,i) = RTCHitN_u(potentialHit,N,i);
          RTCRayN_v(ray,N,i) = RTCHitN_v(potentialHit,N,i);
          RTCRayN_tfar(ray,N,i) = RTCHitN_t(potentialHit,N,i);
          
          RTCRayN_Ng_x(ray,N,i) = RTCHitN_Ng_x(potentialHit,N,i);
          RTCRayN_Ng_y(ray,N,i) = RTCHitN_Ng_y(potentialHit,N,i);
          RTCRayN_Ng_z(ray,N,i) = RTCHitN_Ng_z(potentialHit,N,i);
        }
      }
    }

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      if (!supportsIntersectMode(device,imode))
        return VerifyApplication::SKIPPED;

      VerifyScene scene(device,sflags,to_aflags(imode));
      Vec3fa p0(-0.75f,-0.25f,-10.0f), dx(4,0,0), dy(0,4,0);
      int geom0 = 0;
      if (subdiv) geom0 = scene.addSubdivPlane (sampler,gflags, 4, p0, dx, dy).first;
      else        geom0 = scene.addPlane       (sampler,gflags, 4, p0, dx, dy).first;
      rtcSetUserData(scene,geom0,(void*)123);
      
      if (imode == MODE_INTERSECT1 ) {
        rtcSetIntersectionFilterFunction(scene,geom0,intersectionFilter1);
        rtcSetOcclusionFilterFunction   (scene,geom0,intersectionFilter1);
      }
      else if (imode == MODE_INTERSECT4 ) {
        rtcSetIntersectionFilterFunction4(scene,geom0,intersectionFilter4);
        rtcSetOcclusionFilterFunction4   (scene,geom0,intersectionFilter4);
      }
      else if (imode == MODE_INTERSECT8 ) {
        rtcSetIntersectionFilterFunction8(scene,geom0,intersectionFilter8);
        rtcSetOcclusionFilterFunction8   (scene,geom0,intersectionFilter8);
      }
      else if (imode == MODE_INTERSECT16) {
        rtcSetIntersectionFilterFunction16(scene,geom0,intersectionFilter16);
        rtcSetOcclusionFilterFunction16   (scene,geom0,intersectionFilter16);
      }
      else {
        rtcSetIntersectionFilterFunctionN (scene,geom0,intersectionFilterN);
        rtcSetOcclusionFilterFunctionN (scene,geom0,intersectionFilterN);
      }
      rtcCommit (scene);
      AssertNoError(device);
      
      RTCRay rays[16];
      for (unsigned int iy=0; iy<4; iy++) 
      {
        for (unsigned int ix=0; ix<4; ix++) 
        {
          unsigned int primID = iy*4+ix;
          if (!subdiv) primID *= 2;
          rays[iy*4+ix] = makeRay(Vec3fa(float(ix),float(iy),0.0f),Vec3fa(0,0,-1));
        }
      }
      IntersectWithMode(imode,ivariant,scene,rays,16);
      
      bool passed = true;
      for (unsigned int iy=0; iy<4; iy++) 
      {
        for (unsigned int ix=0; ix<4; ix++) 
        {
          unsigned int primID = iy*4+ix;
          if (!subdiv) primID *= 2;
          RTCRay& ray = rays[iy*4+ix];
          bool ok = (primID & 2) ? (ray.geomID == RTC_INVALID_GEOMETRY_ID) : (ray.geomID == 0);
          if (!ok) passed = false;
        }
      }
      AssertNoError(device);

      return (VerifyApplication::TestReturnValue) passed;
    }
  };
    
  struct InactiveRaysTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;

    static const size_t N = 10;
    static const size_t maxStreamSize = 100;
    
    InactiveRaysTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}
   
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      if (!supportsIntersectMode(device,imode))
        return VerifyApplication::SKIPPED;

      Vec3fa pos = zero;
      VerifyScene scene(device,sflags,to_aflags(imode));
      scene.addSphere(sampler,RTC_GEOMETRY_STATIC,pos,2.0f,50); // FIXME: use different geometries too
      rtcCommit (scene);
      AssertNoError(device);

      RTCRay invalid_ray; clearRay(invalid_ray);
      invalid_ray.tnear = pos_inf;
      invalid_ray.tfar  = neg_inf;
      invalid_ray = invalid_ray;
      
      size_t numFailures = 0;
      for (size_t i=0; i<size_t(N*state->intensity); i++) 
      {
        for (size_t M=1; M<maxStreamSize; M++)
        {
          bool valid[maxStreamSize];
          __aligned(16) RTCRay rays[maxStreamSize];
          for (size_t j=0; j<M; j++) 
          {
            if (rand()%2) {
              valid[j] = true;
              Vec3fa org = 2.0f*random_Vec3fa()-Vec3fa(1.0f);
              Vec3fa dir = 2.0f*random_Vec3fa()-Vec3fa(1.0f);
              rays[j] = makeRay(pos+org,dir); 
            } else {
              valid[j] = false;
              rays[j] = invalid_ray;
            }
          }
          IntersectWithMode(imode,ivariant,scene,rays,M);
          for (size_t j=0; j<M; j++) {
            if (valid[j]) continue;
            numFailures += neq_ray_special(rays[j],invalid_ray);
          }
        }
      }
      AssertNoError(device);
      return (VerifyApplication::TestReturnValue) (numFailures == 0);
    }
  };

  struct WatertightTest : public VerifyApplication::IntersectTest
  {
    ALIGNED_STRUCT;
    RTCSceneFlags sflags;
    std::string model;
    Vec3fa pos;
    static const size_t N = 10;
    static const size_t maxStreamSize = 100;
    
    WatertightTest (std::string name, int isa, RTCSceneFlags sflags, IntersectMode imode, std::string model, const Vec3fa& pos)
      : VerifyApplication::IntersectTest(name,isa,imode,VARIANT_INTERSECT,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), model(model), pos(pos) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      if (!supportsIntersectMode(device,imode))
        return VerifyApplication::SKIPPED;

      avector<Vec3fa> motion_vector;
      motion_vector.push_back(Vec3fa(0.0f));
      motion_vector.push_back(Vec3fa(0.0f));

      VerifyScene scene(device,sflags,to_aflags(imode));
      size_t size = state->intensity < 1.0f ? 50 : 500;
      if      (model == "sphere.triangles") scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createTriangleSphere(pos,2.0f,size));
      else if (model == "sphere.quads"    ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createQuadSphere    (pos,2.0f,size));
      else if (model == "sphere.subdiv"   ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createSubdivSphere  (pos,2.0f,4,64));
      else if (model == "plane.triangles" ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createTrianglePlane (Vec3fa(pos.x,-6.0f,-6.0f),Vec3fa(0.0f,0.0f,12.0f),Vec3fa(0.0f,12.0f,0.0f),size,size));
      else if (model == "plane.quads"     ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createQuadPlane     (Vec3fa(pos.x,-6.0f,-6.0f),Vec3fa(0.0f,0.0f,12.0f),Vec3fa(0.0f,12.0f,0.0f),size,size));
      else if (model == "plane.subdiv"    ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createSubdivPlane   (Vec3fa(pos.x,-6.0f,-6.0f),Vec3fa(0.0f,0.0f,12.0f),Vec3fa(0.0f,12.0f,0.0f),size,size,2));
      else if (model == "sphere.triangles_mb") scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createTriangleSphere(pos,2.0f,size)->set_motion_vector(motion_vector));
      else if (model == "sphere.quads_mb"    ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createQuadSphere    (pos,2.0f,size)->set_motion_vector(motion_vector));
      else if (model == "plane.triangles_mb" ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createTrianglePlane (Vec3fa(pos.x,-6.0f,-6.0f),Vec3fa(0.0f,0.0f,12.0f),Vec3fa(0.0f,12.0f,0.0f),size,size)->set_motion_vector(motion_vector));
      else if (model == "plane.quads_mb"     ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createQuadPlane     (Vec3fa(pos.x,-6.0f,-6.0f),Vec3fa(0.0f,0.0f,12.0f),Vec3fa(0.0f,12.0f,0.0f),size,size)->set_motion_vector(motion_vector));
      bool plane = model.compare(0,5,"plane") == 0;
      rtcCommit (scene);
      AssertNoError(device);
      
      size_t numTests = 0;
      size_t numFailures = 0;
      for (auto ivariant : state->intersectVariants)
      for (size_t i=0; i<size_t(N*state->intensity); i++) 
      {
        for (size_t M=1; M<maxStreamSize; M++)
        {
          __aligned(16) RTCRay rays[maxStreamSize];
          for (size_t j=0; j<M; j++) 
          {
            if (plane) {
              Vec3fa dir = 2.0f*random_Vec3fa() - Vec3fa(1.0f); dir.x = 1.0f;
              rays[j] = makeRay(Vec3fa(pos.x-3.0f,0.0f,0.0f),dir); 
            } else {
              Vec3fa org = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
              Vec3fa dir = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
              rays[j] = makeRay(pos+org,dir); 
            }
          }
          IntersectWithMode(imode,ivariant,scene,rays,M);
          for (size_t j=0; j<M; j++) {
            numTests++;
            numFailures += rays[j].geomID == RTC_INVALID_GEOMETRY_ID;            
          }
        }
      }
      AssertNoError(device);

      double failRate = double(numFailures) / double(numTests);
      bool failed = failRate > 0.00002;
      if (!silent) { printf(" (%f%%)", 100.0f*failRate); fflush(stdout); }
      return (VerifyApplication::TestReturnValue)(!failed);
    }
  };

  struct SmallTriangleHitTest : public VerifyApplication::IntersectTest
  {
    ALIGNED_STRUCT;
    RTCSceneFlags sflags;
    Vec3fa pos;
    float radius;
    static const size_t N = 10000;
    static const size_t maxStreamSize = 100;
    
    SmallTriangleHitTest (std::string name, int isa, RTCSceneFlags sflags, IntersectMode imode, const Vec3fa& pos, const float radius)
      : VerifyApplication::IntersectTest(name,isa,imode,VARIANT_INTERSECT,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), pos(pos), radius(radius) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      if (!supportsIntersectMode(device,imode))
        return VerifyApplication::SKIPPED;

      VerifyScene scene(device,sflags,to_aflags(imode));
      LinearSpace3fa space(Vec3fa(1.0f,0.0f,0.0f),Vec3fa(0.0f,1.0f,0.0f),Vec3fa(0.0f,0.0f,1.0f));
      space *= LinearSpace3fa::rotate(Vec3fa(4.0f,7.0f,-1.0f),4.34f);
      const Vec3fa dx = 100.0f*normalize(space.vx);
      const Vec3fa dy = 100.0f*normalize(space.vy);
      const Vec3fa p = pos-0.5f*(dx+dy);
      Ref<SceneGraph::TriangleMeshNode> plane = SceneGraph::createTrianglePlane (p,dx,dy,100,100).dynamicCast<SceneGraph::TriangleMeshNode>();
      scene.addGeometry(RTC_GEOMETRY_STATIC,plane.dynamicCast<SceneGraph::Node>());
      rtcCommit (scene);
      AssertNoError(device);
      
      size_t numTests = 0;
      size_t numFailures = 0;
      //for (auto ivariant : state->intersectVariants)
      IntersectVariant ivariant = VARIANT_INTERSECT_INCOHERENT;
      size_t numRays = size_t(N*state->intensity);
      for (size_t i=0; i<numRays; i+=maxStreamSize) 
      {
        size_t M = min(maxStreamSize,numRays-i);
        unsigned int primIDs[maxStreamSize];
        __aligned(16) RTCRay rays[maxStreamSize];
        for (size_t j=0; j<M; j++) 
        {
          Vec3fa org = pos + radius*(2.0f*random_Vec3fa() - Vec3fa(1.0f));
          unsigned int primID = random_int() % plane->triangles.size();
          Vec3fa v0 = plane->positions[0][plane->triangles[primID].v0];
          Vec3fa v1 = plane->positions[0][plane->triangles[primID].v1];
          Vec3fa v2 = plane->positions[0][plane->triangles[primID].v2];
          Vec3fa c = (v0+v1+v2)/3.0f;
          primIDs[j] = primID;
          rays[j] = makeRay(org,c-org); 
        }
        IntersectWithMode(imode,ivariant,scene,rays,M);
        for (size_t j=0; j<M; j++) {
          Vec3fa dir(rays[j].dir[0],rays[j].dir[1],rays[j].dir[2]);
          //if (abs(dot(normalize(dir),space.vz)) < 0.9f) continue;
          numTests++;
          numFailures += rays[j].primID != primIDs[j];
        }
      }
      AssertNoError(device);

      double failRate = double(numFailures) / double(numTests);
      bool failed = failRate > 0.00002;
      if (!silent) { printf(" (%f%%)", 100.0f*failRate); fflush(stdout); }
      return (VerifyApplication::TestReturnValue)(!failed);
    }
  };

  struct RayAlignmentTest : public VerifyApplication::IntersectTest
  {
    ALIGNED_STRUCT;
    RTCSceneFlags sflags;
    std::string model;
    static const size_t N = 10;
    static const size_t maxStreamSize = 100;
    
    RayAlignmentTest (std::string name, int isa, RTCSceneFlags sflags, IntersectMode imode, std::string model)
      : VerifyApplication::IntersectTest(name,isa,imode,VARIANT_INTERSECT,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), model(model) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      if (!supportsIntersectMode(device,imode))
        return VerifyApplication::SKIPPED;

      VerifyScene scene(device,sflags,to_aflags(imode));
      if      (model == "sphere.triangles") scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createTriangleSphere(zero,2.0f,50));
      else if (model == "sphere.quads"    ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createQuadSphere    (zero,2.0f,50));
      else if (model == "sphere.subdiv"   ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createSubdivSphere  (zero,2.0f,4,4));
      // FIXME: test more geometry types
      rtcCommit (scene);
      AssertNoError(device);
      
      for (auto ivariant : state->intersectVariants)
      for (size_t i=0; i<size_t(N*state->intensity); i++) 
      {
        for (size_t M=1; M<maxStreamSize; M++)
        {
          size_t alignment = alignment_of(imode);
          __aligned(64) char data[maxStreamSize*sizeof(RTCRay)]; 
          RTCRay* rays = (RTCRay*) &data[alignment];
          for (size_t j=0; j<M; j++) 
          {
            Vec3fa org = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
            Vec3fa dir = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
            rays[j] = makeRay(org,dir); 
          }
          IntersectWithMode(imode,ivariant,scene,rays,M);
        }
      }
      AssertNoError(device);

      return VerifyApplication::PASSED;
    }
  };
  
  struct NaNTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    
    NaNTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags)  {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      if (!supportsIntersectMode(device,imode))
        return VerifyApplication::SKIPPED;

      bool ok = false;
      for (size_t i=0; i<10 && !ok; i++)
      {
        const size_t numRays = 1000;
        RTCRay rays[numRays];
        VerifyScene scene(device,sflags,to_aflags(imode));
        scene.addSphere(sampler,gflags,zero,2.0f,100);
        scene.addHair  (sampler,gflags,zero,1.0f,1.0f,100);
        rtcCommit (scene);
        
        double c0 = getSeconds();
        for (size_t i=0; i<numRays; i++) {
          Vec3fa org = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          Vec3fa dir = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          rays[i] = makeRay(org,dir); 
        }
        IntersectWithMode(imode,ivariant,scene,rays,numRays);
        
        double c1 = getSeconds();
        for (size_t i=0; i<numRays; i++) {
          Vec3fa org = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          Vec3fa dir = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          rays[i] = makeRay(org+Vec3fa(nan),dir); 
        }
        IntersectWithMode(imode,ivariant,scene,rays,numRays);
        
        double c2 = getSeconds();
        for (size_t i=0; i<numRays; i++) {
          Vec3fa org = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          Vec3fa dir = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          rays[i] = makeRay(org+Vec3fa(nan),dir+Vec3fa(nan)); 
        }
        IntersectWithMode(imode,ivariant,scene,rays,numRays);
        
        double c3 = getSeconds();
        for (size_t i=0; i<numRays; i++) {
          Vec3fa org = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          Vec3fa dir = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          rays[i] = makeRay(org,dir,nan,nan); 
        }
        IntersectWithMode(imode,ivariant,scene,rays,numRays);
        
        double c4 = getSeconds();
        double d1 = c1-c0;
        double d2 = c2-c1;
        double d3 = c3-c2;
        double d4 = c4-c3;
        AssertNoError(device);        
        
        ok = (d2 < 2.5*d1) && (d3 < 2.5*d1) && (d4 < 2.5*d1);
        double f = max(d2/d1,d3/d1,d4/d1);
        if (!silent) { printf(" (%3.2fx)",f); fflush(stdout); }
      }
      return (VerifyApplication::TestReturnValue) ok;
    }
  };
    
  struct InfTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    
    InfTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}
   
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      if (!supportsIntersectMode(device,imode))
        return VerifyApplication::SKIPPED;

      const size_t numRays = 1000;
      RTCRay rays[numRays];
      VerifyScene scene(device,sflags,to_aflags(imode));
      scene.addSphere(sampler,gflags,zero,2.0f,100);
      scene.addHair  (sampler,gflags,zero,1.0f,1.0f,100);
      rtcCommit (scene);
      AssertNoError(device);

      bool ok = false;
      for (size_t i=0; i<10 && !ok; i++)
      {
        double c0 = getSeconds();
        for (size_t i=0; i<numRays; i++) {
          Vec3fa org = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          Vec3fa dir = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          rays[i] = makeRay(org,dir); 
        }
        IntersectWithMode(imode,ivariant,scene,rays,numRays);
        
        double c1 = getSeconds();
        for (size_t i=0; i<numRays; i++) {
          Vec3fa org = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          Vec3fa dir = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          rays[i] = makeRay(org+Vec3fa(inf),dir); 
        }
        IntersectWithMode(imode,ivariant,scene,rays,numRays);
        
        double c2 = getSeconds();
        for (size_t i=0; i<numRays; i++) {
          Vec3fa org = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          Vec3fa dir = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          rays[i] = makeRay(org,dir+Vec3fa(inf)); 
        }
        IntersectWithMode(imode,ivariant,scene,rays,numRays);
        
        double c3 = getSeconds();
        for (size_t i=0; i<numRays; i++) {
          Vec3fa org = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          Vec3fa dir = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          rays[i] = makeRay(org+Vec3fa(inf),dir+Vec3fa(inf)); 
        }
        IntersectWithMode(imode,ivariant,scene,rays,numRays);
        
        double c4 = getSeconds();
        for (size_t i=0; i<numRays; i++) {
          Vec3fa org = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          Vec3fa dir = 2.0f*random_Vec3fa() - Vec3fa(1.0f);
          rays[i] = makeRay(org,dir,-0.0f,inf); 
        }
        IntersectWithMode(imode,ivariant,scene,rays,numRays);
        
        double c5 = getSeconds();      
        double d1 = c1-c0;
        double d2 = c2-c1;
        double d3 = c3-c2;
        double d4 = c4-c3;
        double d5 = c5-c4;
        AssertNoError(device);
        
        ok = (d2 < 2.5*d1) && (d3 < 2.5*d1) && (d4 < 2.5*d1) && (d5 < 2.5*d1);
        double f = max(d2/d1,d3/d1,d4/d1,d5/d1);
        if (!silent) { printf(" (%3.2fx)",f); fflush(stdout); }
      }
      return (VerifyApplication::TestReturnValue) ok;
    }
  };

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  void shootRandomRays (int hash, std::vector<IntersectMode>& intersectModes, std::vector<IntersectVariant>& intersectVariants, const VerifyScene& scene)
  {
    RandomSampler sampler;
    RandomSampler_init(sampler,hash);

    const size_t numRays = 100;
    for (auto imode : intersectModes)
    {
      for (auto ivariant : intersectVariants)
      {
        if (!has_variant(imode,ivariant)) continue;

        RTCRay rays[numRays];
        for (size_t i=0; i<numRays; i++) {
          Vec3fa org = 2.0f*RandomSampler_get3D(sampler) - Vec3fa(1.0f);
          Vec3fa dir = 2.0f*RandomSampler_get3D(sampler) - Vec3fa(1.0f);
          rays[i] = makeRay(org,dir); 
        }
        IntersectWithMode(imode,ivariant,scene,rays,numRays);
      }
    }
  }

  static bool build_join_test = false;

  struct RegressionTask
  {
    RegressionTask (VerifyApplication::Test* test, unsigned int sceneIndex, unsigned int sceneCount, unsigned int threadCount, bool cancelBuild)
      : sceneIndex(sceneIndex), sceneCount(sceneCount), test(test), scene(nullptr), numActiveThreads(0), cancelBuild(cancelBuild), errorCounter(0) 
    { 
      barrier.init(threadCount); 
      RandomSampler_init(sampler,0);
    }

    unsigned int sceneIndex;
    unsigned int sceneCount;
    VerifyApplication::Test* test;
    VerifyApplication* state;
    Ref<VerifyScene> scene;
    BarrierSys barrier;
    volatile unsigned int numActiveThreads;
    bool cancelBuild;
    std::atomic<size_t> errorCounter;
    RandomSampler sampler;
  };

  struct ThreadRegressionTask
  {
    ThreadRegressionTask (unsigned int threadIndex, unsigned int threadCount,
                          VerifyApplication* state, RTCDeviceRef& device, std::vector<IntersectMode>& intersectModes, RegressionTask* task)
      : threadIndex(threadIndex), threadCount(threadCount), state(state), device(device), intersectModes(intersectModes), task(task) {}

    unsigned int threadIndex;
    unsigned int threadCount;
    VerifyApplication* state;
    RTCDeviceRef& device;
    std::vector<IntersectMode>& intersectModes;
    RegressionTask* task;
  };

  size_t monitorProgressBreak = 0;
  std::atomic<size_t> monitorProgressInvokations(0);
  std::atomic<size_t> monitorProgressBreakExecuted(0);

  bool monitorProgressFunction(void* ptr, double dn) 
  {
    size_t n = monitorProgressInvokations++;
    if (n == monitorProgressBreak) {
      monitorProgressBreakExecuted++;
      return false;
    }
    return true;
  }

  void rtcore_regression_static_thread(void* ptr)
  {
    ThreadRegressionTask* thread = (ThreadRegressionTask*) ptr;
    RegressionTask* task = thread->task;
    if (thread->threadIndex > 0) 
    {
      for (unsigned int i=0; i<task->sceneCount; i++) 
      {
	task->barrier.wait();
	if (thread->threadIndex < task->numActiveThreads) 
	{
          if (build_join_test) rtcCommitJoin(*task->scene);
          else                 {
            rtcCommitThread(*task->scene,thread->threadIndex,task->numActiveThreads);
            rtcCommitThread(*task->scene,thread->threadIndex,task->numActiveThreads);
          }
	  //if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
          if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
            task->errorCounter++;
          }
          else {
            shootRandomRays(i,thread->intersectModes,thread->state->intersectVariants,*task->scene);
          }
	}
        task->barrier.wait();
      }
      delete thread; thread = nullptr;
      return;
    }

    if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
    bool hasError = false;

    for (unsigned int i=0; i<task->sceneCount; i++) 
    {
      RandomSampler_init(task->sampler,task->sceneIndex*13565+i*3242);
      if (i%5 == 0) std::cout << "." << std::flush;

      RTCSceneFlags sflag = getSceneFlag(i); 
      task->scene = new VerifyScene(thread->device,sflag,aflags_all);
      if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
      if (task->cancelBuild) rtcSetProgressMonitorFunction(*task->scene,monitorProgressFunction,nullptr);
      std::vector<std::unique_ptr<Sphere>> spheres;
      
      for (unsigned int j=0; j<10; j++) 
      {
        Vec3fa pos = 100.0f*RandomSampler_get3D(task->sampler);
	int type = RandomSampler_getInt(task->sampler)%9;
        switch (RandomSampler_getInt(task->sampler)%16) {
        case 0: pos = Vec3fa(nan); break;
        case 1: pos = Vec3fa(inf); break;
        case 2: pos = Vec3fa(1E30f); break;
        default: break;
        };
	size_t numPhi = RandomSampler_getInt(task->sampler)%50;
	if (type == 2) numPhi = RandomSampler_getInt(task->sampler)%10;
        size_t numTriangles = 2*2*numPhi*(numPhi-1);
	numTriangles = RandomSampler_getInt(task->sampler)%(numTriangles+1);
        switch (type) {
        case 0: task->scene->addSphere(task->sampler,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles); break;
	case 1: task->scene->addSphere(task->sampler,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles,task->test->random_motion_vector(1.0f)); break;
        case 2: task->scene->addQuadSphere(task->sampler,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles); break;
	case 3: task->scene->addQuadSphere(task->sampler,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles,task->test->random_motion_vector(1.0f)); break;
	case 4: task->scene->addSubdivSphere(task->sampler,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,4,numTriangles); break;
        case 5: task->scene->addSubdivSphere(task->sampler,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,4,numTriangles,task->test->random_motion_vector(1.0f)); break;
	case 6: task->scene->addHair  (task->sampler,RTC_GEOMETRY_STATIC,pos,1.0f,2.0f,numTriangles); break;
	case 7: task->scene->addHair  (task->sampler,RTC_GEOMETRY_STATIC,pos,1.0f,2.0f,numTriangles,task->test->random_motion_vector(1.0f)); break; 

        case 8: {
	  std::unique_ptr<Sphere> sphere(new Sphere(pos,2.0f));  
	  task->scene->addUserGeometryEmpty(task->sampler,RTC_GEOMETRY_STATIC,sphere.get());
          spheres.push_back(std::move(sphere));
          break;
        }
	}
        //if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
        if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
          task->errorCounter++;
          hasError = true;
          break;
        }
      }
      
      if (thread->threadCount) {
	task->numActiveThreads = max(unsigned(1),RandomSampler_getInt(task->sampler) % thread->threadCount);
	task->barrier.wait();
        if (build_join_test) rtcCommitJoin(*task->scene);
        else                 {
          rtcCommitThread(*task->scene,thread->threadIndex,task->numActiveThreads);
          rtcCommitThread(*task->scene,thread->threadIndex,task->numActiveThreads);          
        }
      } else {
        if (!hasError) {
          rtcCommit(*task->scene);
        }
      }
      //if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;

      if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
        task->errorCounter++;
      }
      else {
        if (!hasError) {
          shootRandomRays(i,thread->intersectModes,thread->state->intersectVariants,*task->scene);
        }
      }

      if (thread->threadCount) 
	task->barrier.wait();

      task->scene = nullptr;
      if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;
    }

    delete thread; thread = nullptr;
    return;
  }

  void rtcore_regression_dynamic_thread(void* ptr)
  {
    ThreadRegressionTask* thread = (ThreadRegressionTask*) ptr;
    RegressionTask* task = thread->task;
    if (thread->threadIndex > 0) 
    {
      for (unsigned int i=0; i<task->sceneCount; i++) 
      {
	task->barrier.wait();
	if (thread->threadIndex < task->numActiveThreads) 
	{
          if (build_join_test) rtcCommitJoin(*task->scene);
          else	               rtcCommitThread(*task->scene,thread->threadIndex,task->numActiveThreads);
	  //if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
          if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
            task->errorCounter++;
          }
          else {
            shootRandomRays(i,thread->intersectModes,thread->state->intersectVariants,*task->scene);
          }
	}
	task->barrier.wait();
      }
      delete thread; thread = nullptr;
      return;
    }
    task->scene = new VerifyScene(thread->device,RTC_SCENE_DYNAMIC,aflags_all);
    if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
    if (task->cancelBuild) rtcSetProgressMonitorFunction(*task->scene,monitorProgressFunction,nullptr);
    const size_t numSlots = 20;
    const size_t numIterations = 2*numSlots;
    std::pair<int,Ref<SceneGraph::Node>> geom[numSlots];
    int types[numSlots];
    Sphere spheres[numSlots];
    size_t numVertices[numSlots];
    for (size_t i=0; i<numSlots; i++)  {
      geom[i].first = -1;
      geom[i].second = nullptr;
      types[i] = 0;
      numVertices[i] = 0;
    }

    bool hasError = false;

    for (unsigned int i=0; i<task->sceneCount; i++) 
    {
      srand(task->sceneIndex*23565+i*2242);
      if (i%20 == 0) std::cout << "." << std::flush;

      for (unsigned int j=0; j<numIterations; j++) 
      {
        int index = RandomSampler_getInt(task->sampler)%numSlots;
        if (geom[index].first == -1) 
        {
          int type = RandomSampler_getInt(task->sampler)%21;
          Vec3fa pos = 100.0f*RandomSampler_get3D(task->sampler);
          switch (RandomSampler_getInt(task->sampler)%16) {
          case 0: pos = Vec3fa(nan); break;
          case 1: pos = Vec3fa(inf); break;
          case 2: pos = Vec3fa(1E30f); break;
          default: break;
          };
          size_t numPhi = RandomSampler_getInt(task->sampler)%100;
	  if (type >= 12 || type <= 17) numPhi = RandomSampler_getInt(task->sampler)%10;
#if defined(__WIN32__)          
          numPhi = RandomSampler_getInt(task->sampler) % 4;
#endif

          size_t numTriangles = 2*2*numPhi*(numPhi-1);
          numTriangles = RandomSampler_getInt(task->sampler)%(numTriangles+1);
          types[index] = type;
          numVertices[index] = 2*numPhi*(numPhi+1);
          switch (type) {
          case 0: geom[index] = task->scene->addSphere(task->sampler,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles); break;
          case 1: geom[index] = task->scene->addSphere(task->sampler,RTC_GEOMETRY_DEFORMABLE,pos,2.0f,numPhi,numTriangles); break;
          case 2: geom[index] = task->scene->addSphere(task->sampler,RTC_GEOMETRY_DYNAMIC,pos,2.0f,numPhi,numTriangles); break;

          case 3: geom[index] = task->scene->addSphere(task->sampler,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles,task->test->random_motion_vector(1.0f)); break;
          case 4: geom[index] = task->scene->addSphere(task->sampler,RTC_GEOMETRY_DEFORMABLE,pos,2.0f,numPhi,numTriangles,task->test->random_motion_vector(1.0f)); break;
          case 5: geom[index] = task->scene->addSphere(task->sampler,RTC_GEOMETRY_DYNAMIC,pos,2.0f,numPhi,numTriangles,task->test->random_motion_vector(1.0f)); break;

          case 6: geom[index] = task->scene->addQuadSphere(task->sampler,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles); break;
          case 7: geom[index] = task->scene->addQuadSphere(task->sampler,RTC_GEOMETRY_DEFORMABLE,pos,2.0f,numPhi,numTriangles); break;
          case 8: geom[index] = task->scene->addQuadSphere(task->sampler,RTC_GEOMETRY_DYNAMIC,pos,2.0f,numPhi,numTriangles); break;

          case 9: geom[index] = task->scene->addQuadSphere(task->sampler,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles,task->test->random_motion_vector(1.0f)); break;
          case 10: geom[index] = task->scene->addQuadSphere(task->sampler,RTC_GEOMETRY_DEFORMABLE,pos,2.0f,numPhi,numTriangles,task->test->random_motion_vector(1.0f)); break;
          case 11: geom[index] = task->scene->addQuadSphere(task->sampler,RTC_GEOMETRY_DYNAMIC,pos,2.0f,numPhi,numTriangles,task->test->random_motion_vector(1.0f)); break;

          case 12: geom[index] = task->scene->addSubdivSphere(task->sampler,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,4,numTriangles); break;
	  case 13: geom[index] = task->scene->addSubdivSphere(task->sampler,RTC_GEOMETRY_DEFORMABLE,pos,2.0f,numPhi,4,numTriangles); break;
	  case 14: geom[index] = task->scene->addSubdivSphere(task->sampler,RTC_GEOMETRY_DYNAMIC,pos,2.0f,numPhi,4,numTriangles); break;

          case 15: geom[index] = task->scene->addSubdivSphere(task->sampler,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,4,numTriangles,task->test->random_motion_vector(1.0f)); break;
	  case 16: geom[index] = task->scene->addSubdivSphere(task->sampler,RTC_GEOMETRY_DEFORMABLE,pos,2.0f,numPhi,4,numTriangles,task->test->random_motion_vector(1.0f)); break;
	  case 17: geom[index] = task->scene->addSubdivSphere(task->sampler,RTC_GEOMETRY_DYNAMIC,pos,2.0f,numPhi,4,numTriangles,task->test->random_motion_vector(1.0f)); break;

          case 18: spheres[index] = Sphere(pos,2.0f); geom[index] = task->scene->addUserGeometryEmpty(task->sampler,RTC_GEOMETRY_STATIC,&spheres[index]); break;
          case 19: spheres[index] = Sphere(pos,2.0f); geom[index] = task->scene->addUserGeometryEmpty(task->sampler,RTC_GEOMETRY_DEFORMABLE,&spheres[index]); break;
          case 20: spheres[index] = Sphere(pos,2.0f); geom[index] = task->scene->addUserGeometryEmpty(task->sampler,RTC_GEOMETRY_DYNAMIC,&spheres[index]); break;
          }; 
	  //if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
          if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
            task->errorCounter++;
            hasError = true;
            break;
          }
        }
        else 
        {
          switch (types[index]) {
          case 18:
          case 19:
          case 20:
          {
            rtcDeleteGeometry(*task->scene,geom[index].first);     
	    if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
            geom[index].first = -1; 
            geom[index].second = nullptr; 
            break;
          }
          case 0:
          case 1: 
          case 2:
          case 3:
          case 4: 
          case 5:
          case 6:
	  case 7: 
          case 8:
          case 9:
          case 10:
          case 11:
          case 12:
          case 13:
          case 14:
          case 15:
          case 16:
          case 17:
          {
            int op = RandomSampler_getInt(task->sampler)%3;
            switch (op) {
            case 0: {
              rtcDeleteGeometry(*task->scene,geom[index].first);     
	      if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
              geom[index].first = -1; 
              geom[index].second = nullptr; 
              break;
            }
            case 1: {
              Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(*task->scene,geom[index].first,RTC_VERTEX_BUFFER);
              if (vertices) { 
                for (size_t i=0; i<numVertices[index]; i++) vertices[i] += Vec3fa(0.1f);
              }
              rtcUnmapBuffer(*task->scene,geom[index].first,RTC_VERTEX_BUFFER);
              switch (types[index])
              {
              case 4: case 5: case 10: case 11:
                Vec3fa* vertices = (Vec3fa*)rtcMapBuffer(*task->scene, geom[index].first, RTC_VERTEX_BUFFER1);
                if (vertices) {
                  for (size_t i = 0; i < numVertices[index]; i++) vertices[i] += Vec3fa(0.1f);
                }
                rtcUnmapBuffer(*task->scene, geom[index].first, RTC_VERTEX_BUFFER1);
              }
              break;
            }
            case 2: {
              switch (types[index])
              {
              case 2:
              case 5:
              case 8:
              case 11:
              case 14:
                task->scene->resizeRandomly(geom[index],task->sampler);
                break;
              }
            }
            }
            break;
          }
          }
        }
        
        /* entirely delete all objects at the end */
        /*if (j == numIterations-1) {
          for (size_t i=0; i<numSlots; i++) {
            if (geom[i].first != -1) {
              rtcDeleteGeometry(*task->scene,geom[i].first);
              if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
              geom[i].first = -1;
              geom[i].second = nullptr;
            }
          }
          }*/
      }

      if (thread->threadCount) {
	task->numActiveThreads = max(unsigned(1),RandomSampler_getInt(task->sampler) % thread->threadCount);
	task->barrier.wait();
        if (build_join_test) rtcCommitJoin(*task->scene);
        else                 rtcCommitThread(*task->scene,thread->threadIndex,task->numActiveThreads);
      } else {
        if (!hasError) 
          rtcCommit(*task->scene);
      }
      //if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;

      if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR)
        task->errorCounter++;
      else
        if (!hasError)
          shootRandomRays(i,thread->intersectModes,thread->state->intersectVariants,*task->scene);

      if (thread->threadCount) 
	task->barrier.wait();
    }

    task->scene = nullptr;
    if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;

    delete thread; thread = nullptr;
    return;
  }

  struct IntensiveRegressionTest : public VerifyApplication::Test
  {
    thread_func func;
    int mode;
    float intensity;
    std::vector<IntersectMode> intersectModes;
    std::vector<thread_t> threads;
    
    IntensiveRegressionTest (std::string name, int isa, thread_func func, int mode, float intensity)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), func(func), mode(mode), intensity(intensity) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));

      /* only test supported intersect modes */
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT1)) intersectModes.push_back(MODE_INTERSECT1);
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT4)) intersectModes.push_back(MODE_INTERSECT4);
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT8)) intersectModes.push_back(MODE_INTERSECT8);
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT16)) intersectModes.push_back(MODE_INTERSECT16);
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM)) {
        intersectModes.push_back(MODE_INTERSECT1M);
        intersectModes.push_back(MODE_INTERSECT1Mp);
        intersectModes.push_back(MODE_INTERSECTNM1);
        intersectModes.push_back(MODE_INTERSECTNM3);
        intersectModes.push_back(MODE_INTERSECTNM4);
        intersectModes.push_back(MODE_INTERSECTNM8);
        intersectModes.push_back(MODE_INTERSECTNM16);
        intersectModes.push_back(MODE_INTERSECTNp);
      }

      size_t errorCounter = 0;
      unsigned int sceneIndex = 0;
      while (sceneIndex < size_t(intensity*state->intensity)) 
      {
        if (mode)
        {
          build_join_test = (mode == 2);
          unsigned numThreads = getNumberOfLogicalThreads();
          
          std::vector<RegressionTask*> tasks;
          while (numThreads) 
          {
            unsigned int N = max(unsigned(1),random_int()%numThreads); numThreads -= N;
            RegressionTask* task = new RegressionTask(this,sceneIndex++,5,N,false);
            tasks.push_back(task);
            
            for (unsigned int i=0; i<N; i++) 
              threads.push_back(createThread(func,new ThreadRegressionTask(i,N,state,device,intersectModes,task),DEFAULT_STACK_SIZE,numThreads+i));
          }
          
          for (size_t i=0; i<threads.size(); i++)
            join(threads[i]);
          for (size_t i=0; i<tasks.size(); i++) {
            errorCounter += tasks[i]->errorCounter;
            delete tasks[i];
          }
          
          threads.clear();
        }
        else
        {
          RegressionTask task(this,sceneIndex++,5,0,false);
          func(new ThreadRegressionTask(0,0,state,device,intersectModes,&task));
        }	
      }
      return (VerifyApplication::TestReturnValue) (errorCounter == 0);
    }
  };

  size_t monitorMemoryBreak = 0;
  std::atomic<size_t> monitorMemoryBytesUsed(0);
  std::atomic<size_t> monitorMemoryInvokations(0);
  std::atomic<size_t> monitorMemoryBreakExecuted(0);

  bool monitorMemoryFunction(void* userPtr, ssize_t bytes, bool post) 
  {
    monitorMemoryBytesUsed += bytes;
    if (bytes > 0) {
      size_t n = monitorMemoryInvokations++;
      if (n == monitorMemoryBreak) {
        monitorMemoryBreakExecuted++;
        if (!post) monitorMemoryBytesUsed -= bytes;
        return false;
      }
    }
    return true;
  }

  struct MemoryMonitorTest : public VerifyApplication::Test
  {
    thread_func func;
    float intensity;
    std::vector<IntersectMode> intersectModes;
    
    MemoryMonitorTest (std::string name, int isa, thread_func func, float intensity)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), func(func), intensity(intensity) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      
      /* only test supported intersect modes */
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT1)) intersectModes.push_back(MODE_INTERSECT1);
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT4)) intersectModes.push_back(MODE_INTERSECT4);
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT8)) intersectModes.push_back(MODE_INTERSECT8);
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT16)) intersectModes.push_back(MODE_INTERSECT16);
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM)) {
        intersectModes.push_back(MODE_INTERSECT1M);
        intersectModes.push_back(MODE_INTERSECT1Mp);
        intersectModes.push_back(MODE_INTERSECTNM1);
        intersectModes.push_back(MODE_INTERSECTNM3);
        intersectModes.push_back(MODE_INTERSECTNM4);
        intersectModes.push_back(MODE_INTERSECTNM8);
        intersectModes.push_back(MODE_INTERSECTNM16);
        intersectModes.push_back(MODE_INTERSECTNp);
      }
      
      rtcDeviceSetMemoryMonitorFunction2(device,monitorMemoryFunction,nullptr);
      
      unsigned int sceneIndex = 0;
      while (sceneIndex < size_t(intensity*state->intensity)) 
      {
        monitorMemoryBreak = std::numeric_limits<size_t>::max();
        monitorMemoryBytesUsed = 0;
        monitorMemoryInvokations = 0;
        monitorMemoryBreakExecuted = 0;
        monitorProgressBreak = std::numeric_limits<size_t>::max();
        monitorProgressInvokations = 0;
        monitorProgressBreakExecuted = 0;
        RegressionTask task1(this,sceneIndex,1,0,true);
        func(new ThreadRegressionTask(0,0,state,device,intersectModes,&task1));
        if (monitorMemoryBytesUsed) 
          return VerifyApplication::FAILED;

        monitorMemoryBreak = size_t(float(monitorMemoryInvokations) * random_float());
        monitorMemoryBytesUsed = 0;
        monitorMemoryInvokations = 0;
        monitorMemoryBreakExecuted = 0;
        monitorProgressBreak = size_t(float(monitorProgressInvokations) * 2.0f * random_float());
        monitorProgressInvokations = 0;
        monitorProgressBreakExecuted = 0;
        RegressionTask task2(this,sceneIndex,1,0,true);
        func(new ThreadRegressionTask(0,0,state,device,intersectModes,&task2));
        if (monitorMemoryBytesUsed) 
          return VerifyApplication::FAILED;
        if (monitorMemoryBreakExecuted && task2.errorCounter != 1) 
          return VerifyApplication::FAILED;
        if (monitorProgressBreakExecuted && task2.errorCounter != 1) 
          return VerifyApplication::FAILED;
        if (!monitorMemoryBreakExecuted && !monitorProgressBreakExecuted && task2.errorCounter != 0) 
          return VerifyApplication::FAILED;
        sceneIndex++;
      }
      rtcDeviceSetMemoryMonitorFunction2(device,nullptr,nullptr);
      AssertNoError(device);
      return VerifyApplication::PASSED;
    }
  };

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  struct SimpleBenchmark : public VerifyApplication::Benchmark
  {
    SimpleBenchmark (std::string name, int isa)
      : VerifyApplication::Benchmark(name,isa,"1/s",true,10) {}
    
    float benchmark(VerifyApplication* state)
    {
      double t0 = getSeconds();
      for (volatile size_t i=0; i<10000000; i++);
      double t1 = getSeconds();
      return 1.0f/float(t1-t0);
    }
  };
  
  struct ParallelIntersectBenchmark : public VerifyApplication::Benchmark
  {
    unsigned int N, dN;
    
    ParallelIntersectBenchmark (std::string name, int isa, unsigned int N, unsigned int dN)
      : VerifyApplication::Benchmark(name,isa,"Mrps",true,10), N(N), dN(dN) {}

    bool setup(VerifyApplication* state) 
    {
      return true;
    }

    virtual void render_block(size_t i, size_t n) = 0;

    float benchmark(VerifyApplication* state)
    {
      double t0 = getSeconds();
      parallel_for((size_t)N/dN, [&](size_t i) {
          render_block(i*dN, dN);
      });
      double t1 = getSeconds();
      return 1E-6f * (float)(N)/float(t1-t0);
    }

    virtual void cleanup(VerifyApplication* state) 
    {
    }
  };

  struct CoherentRaysBenchmark : public ParallelIntersectBenchmark
  {
    GeometryType gtype;
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    IntersectMode imode;
    IntersectVariant ivariant;
    size_t numPhi;
    RTCDeviceRef device;
    Ref<VerifyScene> scene;
	static const size_t tileSizeX = 32;
	static const size_t tileSizeY = 32;
	static const size_t width = 4096;
	static const size_t height = 4096;
	static const size_t numTilesX = width / tileSizeX;
	static const size_t numTilesY = height / tileSizeY;
    
    CoherentRaysBenchmark (std::string name, int isa, GeometryType gtype, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant, size_t numPhi)
      : ParallelIntersectBenchmark(name,isa,numTilesX*numTilesY,1), gtype(gtype), sflags(sflags), gflags(gflags), imode(imode), ivariant(ivariant), numPhi(numPhi) {}
    
    size_t setNumPrimitives(size_t N) 
    { 
      numPhi = size_t(ceilf(sqrtf(N/4.0f)));
      return 4*numPhi*numPhi;
    }

    bool setup(VerifyApplication* state) 
    {
      if (!ParallelIntersectBenchmark::setup(state))
        return false;

      std::string cfg = state->rtcore + ",start_threads=1,set_affinity=1,isa="+stringOfISA(isa);
      device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      rtcDeviceSetErrorFunction2(device,errorHandler,nullptr);
      if (!supportsIntersectMode(device,imode))
        return false;

      scene = new VerifyScene(device,sflags,aflags_all);
      switch (gtype) {
      case TRIANGLE_MESH:    scene->addGeometry(gflags,SceneGraph::createTriangleSphere(zero,one,numPhi)); break;
      case TRIANGLE_MESH_MB: scene->addGeometry(gflags,SceneGraph::createTriangleSphere(zero,one,numPhi)->set_motion_vector(random_motion_vector2(0.01f))); break;
      case QUAD_MESH:        scene->addGeometry(gflags,SceneGraph::createQuadSphere(zero,one,numPhi)); break;
      case QUAD_MESH_MB:     scene->addGeometry(gflags,SceneGraph::createQuadSphere(zero,one,numPhi)->set_motion_vector(random_motion_vector2(0.01f))); break;
      case SUBDIV_MESH:      scene->addGeometry(gflags,SceneGraph::createSubdivSphere(zero,one,8,float(numPhi)/8.0f)); break;
      case SUBDIV_MESH_MB:   scene->addGeometry(gflags,SceneGraph::createSubdivSphere(zero,one,8,float(numPhi)/8.0f)->set_motion_vector(random_motion_vector2(0.01f))); break;
      default:               throw std::runtime_error("invalid geometry for benchmark");
      }
      rtcCommit (*scene);
      AssertNoError(device);
            
      return true;
    }

    void render_block(size_t i, size_t)
    {
      float rcpWidth = 1.0f/width;
      float rcpHeight = 1.0/height;
      const size_t tileY = i / numTilesX;
	  const size_t tileX = i - tileY * numTilesX;
	  const size_t x0 = tileX * tileSizeX;
	  const size_t x1 = min(x0 + tileSizeX, width);
	  const size_t y0 = tileY * tileSizeY;
	  const size_t y1 = min(y0 + tileSizeY, height);
      
      RTCIntersectContext context;
      context.flags = ((ivariant & VARIANT_COHERENT_INCOHERENT_MASK) == VARIANT_COHERENT) ? RTC_INTERSECT_COHERENT :  RTC_INTERSECT_INCOHERENT;
      context.userRayExt = nullptr;

      switch (imode) 
      {
      case MODE_INTERSECT1: 
      {
        for (size_t y=y0; y<y1; y++) {
          for (size_t x=x0; x<x1; x++) {
            RTCRay ray = fastMakeRay(zero,Vec3f(float(x)*rcpWidth,1,float(y)*rcpHeight));
            switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
            case VARIANT_INTERSECT: rtcIntersect(*scene,ray); break;
            case VARIANT_OCCLUDED : rtcOccluded (*scene,ray); break;
            }
          }
        }
        break;
      }
      case MODE_INTERSECT4: 
      {
        for (size_t y=y0; y<y1; y+=2) {
          for (size_t x=x0; x<x1; x+=2) {
            RTCRay4 ray4; 
            for (size_t dy=0; dy<2; dy++) {
              for (size_t dx=0; dx<2; dx++) {
                setRay(ray4,2*dy+dx,fastMakeRay(zero,Vec3f(float(x+dx)*rcpWidth,1,float(y+dy)*rcpHeight)));
              }
            }
            __aligned(16) int valid4[4] = { -1,-1,-1,-1 };
            switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
            case VARIANT_INTERSECT: rtcIntersect4(valid4,*scene,ray4); break;
            case VARIANT_OCCLUDED : rtcOccluded4 (valid4,*scene,ray4); break;
            }
          }
        }
        break;
      }
      case MODE_INTERSECT8: 
      {
        for (size_t y=y0; y<y1; y+=4) {
          for (size_t x=x0; x<x1; x+=2) {
            RTCRay8 ray8; 
            for (size_t dy=0; dy<4; dy++) {
              for (size_t dx=0; dx<2; dx++) {
                setRay(ray8,2*dy+dx,fastMakeRay(zero,Vec3f(float(x+dx)*rcpWidth,1,float(y+dy)*rcpHeight)));
              }
            }
            __aligned(32) int valid8[8] = { -1,-1,-1,-1,-1,-1,-1,-1 };
            switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
            case VARIANT_INTERSECT: rtcIntersect8(valid8,*scene,ray8); break;
            case VARIANT_OCCLUDED : rtcOccluded8 (valid8,*scene,ray8); break;
            }
          }
        }
        break;
      }
      case MODE_INTERSECT16: 
      {
        for (size_t y=y0; y<y1; y+=4) {
          for (size_t x=x0; x<x1; x+=4) {
            RTCRay16 ray16; 
            for (size_t dy=0; dy<4; dy++) {
              for (size_t dx=0; dx<4; dx++) {
                setRay(ray16,4*dy+dx,fastMakeRay(zero,Vec3f(float(x+dx)*rcpWidth,1,float(y+dy)*rcpHeight)));
              }
            }
            __aligned(64) int valid16[16] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
            switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
            case VARIANT_INTERSECT: rtcIntersect16(valid16,*scene,ray16); break;
            case VARIANT_OCCLUDED : rtcOccluded16 (valid16,*scene,ray16); break;
            }
          }
        }
        break;
      }
      case MODE_INTERSECT1M: 
      {
        for (size_t y=y0; y<y1; y+=16) {
          for (size_t x=x0; x<x1; x+=16) {
            RTCRay rays[256];
            for (size_t dy=0; dy<16; dy++) {
              for (size_t dx=0; dx<16; dx++) {
                rays[dy*16+dx] = fastMakeRay(zero,Vec3f(float(x+dx)*rcpWidth,1,float(y+dy)*rcpHeight));
              }
            }
            switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
            case VARIANT_INTERSECT: rtcIntersect1M(*scene,&context,rays,256,sizeof(RTCRay)); break;
            case VARIANT_OCCLUDED : rtcOccluded1M (*scene,&context,rays,256,sizeof(RTCRay)); break;
            }
          }
        }
        break;
      }
      default: break;
      }
    }

    float benchmark(VerifyApplication* state) {
      return tileSizeX*tileSizeY*ParallelIntersectBenchmark::benchmark(state);
    }

    virtual void cleanup(VerifyApplication* state) 
    {
      AssertNoError(device);
      scene = nullptr;
      device = nullptr;
      ParallelIntersectBenchmark::cleanup(state);
    }
  };

  struct IncoherentRaysBenchmark : public ParallelIntersectBenchmark
  {
    GeometryType gtype;
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    IntersectMode imode;
    IntersectVariant ivariant;
    size_t numPhi;
    RTCDeviceRef device;
    Ref<VerifyScene> scene;
    static const size_t numRays = 16*1024*1024;
    static const size_t deltaRays = 1024;
    
    IncoherentRaysBenchmark (std::string name, int isa, GeometryType gtype, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant, size_t numPhi)
      : ParallelIntersectBenchmark(name,isa,numRays,deltaRays), gtype(gtype), sflags(sflags), gflags(gflags), imode(imode), ivariant(ivariant), numPhi(numPhi), device(nullptr)  {}

    size_t setNumPrimitives(size_t N) 
    { 
      numPhi = size_t(ceilf(sqrtf(N/4.0f)));
      return 4*numPhi*numPhi;
    }

    bool setup(VerifyApplication* state) 
    {
      if (!ParallelIntersectBenchmark::setup(state))
        return false;

      std::string cfg = state->rtcore + ",start_threads=1,set_affinity=1,isa="+stringOfISA(isa);
      device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      rtcDeviceSetErrorFunction2(device,errorHandler,nullptr);
      if (!supportsIntersectMode(device,imode))
        return false;

      scene = new VerifyScene(device,sflags,aflags_all);
      switch (gtype) {
      case TRIANGLE_MESH:    scene->addGeometry(gflags,SceneGraph::createTriangleSphere(zero,one,numPhi)); break;
      case TRIANGLE_MESH_MB: scene->addGeometry(gflags,SceneGraph::createTriangleSphere(zero,one,numPhi)->set_motion_vector(random_motion_vector2(0.01f))); break;
      case QUAD_MESH:        scene->addGeometry(gflags,SceneGraph::createQuadSphere(zero,one,numPhi)); break;
      case QUAD_MESH_MB:     scene->addGeometry(gflags,SceneGraph::createQuadSphere(zero,one,numPhi)->set_motion_vector(random_motion_vector2(0.01f))); break;
      case SUBDIV_MESH:      scene->addGeometry(gflags,SceneGraph::createSubdivSphere(zero,one,8,float(numPhi)/8.0f)); break;
      case SUBDIV_MESH_MB:   scene->addGeometry(gflags,SceneGraph::createSubdivSphere(zero,one,8,float(numPhi)/8.0f)->set_motion_vector(random_motion_vector2(0.01f))); break;
      default:               throw std::runtime_error("invalid geometry for benchmark");
      }
      rtcCommit (*scene);
      AssertNoError(device);      

      return true;
    }

    void render_block(size_t i, size_t dn)
    {
      RTCIntersectContext context;
      context.flags = ((ivariant & VARIANT_COHERENT_INCOHERENT_MASK) == VARIANT_COHERENT) ? RTC_INTERSECT_COHERENT :  RTC_INTERSECT_INCOHERENT;
      context.userRayExt = nullptr;

      RandomSampler sampler;
      RandomSampler_init(sampler, (int)i);

      switch (imode) 
      {
      case MODE_INTERSECT1: 
      {
        for (size_t j=0; j<dn; j++) {
          RTCRay ray; 
          fastMakeRay(ray,zero,sampler);
          switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
          case VARIANT_INTERSECT: rtcIntersect(*scene,ray); break;
          case VARIANT_OCCLUDED : rtcOccluded (*scene,ray); break;
          }
        }
        break;
      }
      case MODE_INTERSECT4: 
      {
        for (size_t j=0; j<dn; j+=4) {
          RTCRay4 ray4;
          for (size_t k=0; k<4; k++) {
            setRay(ray4,k,fastMakeRay(zero,sampler));
          }
          __aligned(16) int valid4[4] = { -1,-1,-1,-1 };
          switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
          case VARIANT_INTERSECT: rtcIntersect4(valid4,*scene,ray4); break;
          case VARIANT_OCCLUDED : rtcOccluded4 (valid4,*scene,ray4); break;
          }
        }
        break;
      }
      case MODE_INTERSECT8: 
      {
        for (size_t j=0; j<dn; j+=8) {
          RTCRay8 ray8;
          for (size_t k=0; k<8; k++) {
            setRay(ray8,k,fastMakeRay(zero,sampler));
          }
          __aligned(32) int valid8[8] = { -1,-1,-1,-1,-1,-1,-1,-1 };
          switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
          case VARIANT_INTERSECT: rtcIntersect8(valid8,*scene,ray8); break;
          case VARIANT_OCCLUDED : rtcOccluded8 (valid8,*scene,ray8); break;
          }
        }
        break;
      }
      case MODE_INTERSECT16: 
      {
        for (size_t j=0; j<dn; j+=16) {
          RTCRay16 ray16;
          for (size_t k=0; k<16; k++) {
            setRay(ray16,k,fastMakeRay(zero,sampler));
          }
          __aligned(64) int valid16[16] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
          switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
          case VARIANT_INTERSECT: rtcIntersect16(valid16,*scene,ray16); break;
          case VARIANT_OCCLUDED : rtcOccluded16 (valid16,*scene,ray16); break;
          }
        }
        break;
      }
      case MODE_INTERSECT1M: 
      {
        for (size_t j=0; j<dn; j+=128) {
          RTCRay rays[128];
          for (size_t k=0; k<128; k++) fastMakeRay(rays[k],zero,sampler);
          switch (ivariant & VARIANT_INTERSECT_OCCLUDED_MASK) {
          case VARIANT_INTERSECT: rtcIntersect1M(*scene,&context,rays,128,sizeof(RTCRay)); break;
          case VARIANT_OCCLUDED : rtcOccluded1M (*scene,&context,rays,128,sizeof(RTCRay)); break;
          }
        }
        break;
      }
      default: break;
      }
    }

    virtual void cleanup(VerifyApplication* state) 
    {
      AssertNoError(device);
      scene = nullptr;
      device = nullptr;
      ParallelIntersectBenchmark::cleanup(state);
    }
  };

  static std::atomic<ssize_t> create_geometry_bytes_used(0);

  struct CreateGeometryBenchmark : public VerifyApplication::Benchmark
  {
    GeometryType gtype;
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    size_t numPhi;
    size_t numMeshes;
    bool update;
    bool dobenchmark; // true = measure build performance, false = measure memory consumption
    size_t numPrimitives;
    RTCDeviceRef device;
    Ref<VerifyScene> scene;
    std::vector<Ref<SceneGraph::Node>> geometries;
    
    CreateGeometryBenchmark (std::string name, int isa, GeometryType gtype, RTCSceneFlags sflags, RTCGeometryFlags gflags, size_t numPhi, size_t numMeshes, bool update, bool dobenchmark)
      : VerifyApplication::Benchmark(name,isa,dobenchmark ? "Mprims/s" : "MB",dobenchmark,dobenchmark?10:1), gtype(gtype), sflags(sflags), gflags(gflags), 
        numPhi(numPhi), numMeshes(numMeshes), update(update), dobenchmark(dobenchmark),
        numPrimitives(0), device(nullptr), scene(nullptr) {}

    size_t setNumPrimitives(size_t N) 
    { 
      numPhi = (size_t) ceilf(sqrtf(N/4.0f));
      return 4*numPhi*numPhi;
    }

    void create_scene()
    {
      scene = new VerifyScene(device,sflags,aflags_all);

      numPrimitives = 0;
      for (size_t i=0; i<numMeshes; i++)
      {
        numPrimitives += geometries[i]->numPrimitives();

        switch (gtype) { 
        case TRIANGLE_MESH:    
        case QUAD_MESH:        
        case SUBDIV_MESH:      
        case HAIR_GEOMETRY:
        case LINE_GEOMETRY:
        case TRIANGLE_MESH_MB: 
        case QUAD_MESH_MB:     
        case SUBDIV_MESH_MB:   
        case HAIR_GEOMETRY_MB:
        case LINE_GEOMETRY_MB:
          scene->addGeometry(gflags,geometries[i]); 
          break;
        default: 
          throw std::runtime_error("invalid geometry for benchmark");
        }
      }
    }

    static bool memoryMonitor(void* userPtr, const ssize_t bytes, const bool /*post*/)
    {
      create_geometry_bytes_used += bytes;
      return true;
    }

    bool setup(VerifyApplication* state) 
    {
      std::string cfg = "start_threads=1,set_affinity=1,isa="+stringOfISA(isa) + ",threads=" + std::to_string((long long)numThreads)+","+state->rtcore;
      device = rtcNewDevice(cfg.c_str());
      errorHandler(nullptr,rtcDeviceGetError(device));
      rtcDeviceSetErrorFunction2(device,errorHandler,nullptr);
      if (!dobenchmark) rtcDeviceSetMemoryMonitorFunction2(device,memoryMonitor,nullptr);

      for (unsigned int i=0; i<numMeshes; i++)
      {
        switch (gtype) {
        case TRIANGLE_MESH:    
        case TRIANGLE_MESH_MB: geometries.push_back(SceneGraph::createTriangleSphere(zero,float(i+1),numPhi)); break;
        case QUAD_MESH:        
        case QUAD_MESH_MB:     geometries.push_back(SceneGraph::createQuadSphere(zero,float(i+1),numPhi)); break;
        case SUBDIV_MESH:      
        case SUBDIV_MESH_MB:   geometries.push_back(SceneGraph::createSubdivSphere(zero,float(i+1),8,float(numPhi)/8.0f)); break;
        case HAIR_GEOMETRY:    
        case HAIR_GEOMETRY_MB: geometries.push_back(SceneGraph::createHairyPlane(i,Vec3fa(float(i)),Vec3fa(1,0,0),Vec3fa(0,1,0),0.01f,0.00001f,4*numPhi*numPhi,SceneGraph::HairSetNode::HAIR)); break;
        case LINE_GEOMETRY: 
        case LINE_GEOMETRY_MB: geometries.push_back(SceneGraph::createHairyPlane(i,Vec3fa(float(i)),Vec3fa(1,0,0),Vec3fa(0,1,0),0.01f,0.00001f,4*numPhi*numPhi/3,SceneGraph::HairSetNode::HAIR)); break;
        default:               throw std::runtime_error("invalid geometry for benchmark");
        }

        switch (gtype) {
        case LINE_GEOMETRY:    
        case LINE_GEOMETRY_MB: geometries.back() = SceneGraph::convert_bezier_to_lines(geometries.back()); break;
        default: break;
        }

        switch (gtype) {
        case TRIANGLE_MESH_MB: 
        case QUAD_MESH_MB:     
        case SUBDIV_MESH_MB:   
        case HAIR_GEOMETRY_MB: 
        case LINE_GEOMETRY_MB: geometries.back() = geometries.back()->set_motion_vector(random_motion_vector2(0.0001f)); break;
        default: break;
        }
      }

      if (update)
        create_scene();

      return true;
    }

    float benchmark(VerifyApplication* state)
    {
      if (!update) create_scene();
      
      double t0 = getSeconds();
      
      if (update)
        for (unsigned int i=0; i<numMeshes; i++) 
          rtcUpdate(*scene,i);
      
      create_geometry_bytes_used = 0;
      rtcCommit (*scene);
      AssertNoError(device);
      
      double t1 = getSeconds();

      if (dobenchmark)
        return 1E-6f*float(numPrimitives)/float(t1-t0);
      else
        return 1E-6f*create_geometry_bytes_used;
    }
      
    virtual void cleanup(VerifyApplication* state) 
    {
      scene = nullptr;
      device = nullptr;
      geometries.clear();
    }
  };

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  VerifyApplication::VerifyApplication ()
    : Application(Application::FEATURE_RTCORE), 
      intensity(1.0f), 
      numPassedTests(0), numFailedTests(0), numFailedAndIgnoredTests(0),
      tests(new TestGroup("",false,false)), 
      device(nullptr),
      user_specified_tests(false), flatten(true), parallel(true), cdash(false), 
      database(""), update_database(false), benchmark_tolerance(0.05f),
      usecolors(true)
  {
    rtcore = ""; // do not start threads nor set affinty for normal tests 
    device = rtcNewDevice(rtcore.c_str());

#if defined(__WIN32__)
    usecolors = false;
#endif

    GeometryType gtypes[] = { TRIANGLE_MESH, TRIANGLE_MESH_MB, QUAD_MESH, QUAD_MESH_MB, SUBDIV_MESH, SUBDIV_MESH_MB };
    GeometryType gtypes_all[] = { TRIANGLE_MESH, TRIANGLE_MESH_MB, QUAD_MESH, QUAD_MESH_MB, SUBDIV_MESH, SUBDIV_MESH_MB, 
                                  HAIR_GEOMETRY, HAIR_GEOMETRY_MB, CURVE_GEOMETRY, CURVE_GEOMETRY_MB, LINE_GEOMETRY, LINE_GEOMETRY_MB };

    /* create list of all ISAs to test */
#if defined(EMBREE_TARGET_SSE2)
    if (hasISA(SSE2)) isas.push_back(SSE2);
#endif
#if defined(EMBREE_TARGET_SSE42)
    if (hasISA(SSE42)) isas.push_back(SSE42);
#endif
#if defined(EMBREE_TARGET_AVX)
    if (hasISA(AVX)) isas.push_back(AVX);
#endif
#if defined(EMBREE_TARGET_AVX2)
    if (hasISA(AVX2)) isas.push_back(AVX2);
#endif
#if defined(EMBREE_TARGET_AVX512KNL)
    if (hasISA(AVX512KNL)) isas.push_back(AVX512KNL);
#endif
#if defined(EMBREE_TARGET_AVX512SKX)
    if (hasISA(AVX512SKX)) isas.push_back(AVX512SKX);
#endif
    
    /* create list of all intersect modes to test */
    intersectModes.push_back(MODE_INTERSECT1);
    intersectModes.push_back(MODE_INTERSECT4);
    intersectModes.push_back(MODE_INTERSECT8);
    intersectModes.push_back(MODE_INTERSECT16);
    intersectModes.push_back(MODE_INTERSECT1M);
    intersectModes.push_back(MODE_INTERSECT1Mp);
    intersectModes.push_back(MODE_INTERSECTNM1);
    intersectModes.push_back(MODE_INTERSECTNM3);
    intersectModes.push_back(MODE_INTERSECTNM4);
    intersectModes.push_back(MODE_INTERSECTNM8);
    intersectModes.push_back(MODE_INTERSECTNM16);
    intersectModes.push_back(MODE_INTERSECTNp);
    
    /* create a list of all intersect variants for each intersect mode */
    intersectVariants.push_back(VARIANT_INTERSECT_COHERENT);
    intersectVariants.push_back(VARIANT_OCCLUDED_COHERENT);
    intersectVariants.push_back(VARIANT_INTERSECT_INCOHERENT);
    intersectVariants.push_back(VARIANT_OCCLUDED_INCOHERENT);
    intersectVariants.push_back(VARIANT_INTERSECT_OCCLUDED_COHERENT);

    /* create list of all scene flags to test */
    sceneFlags.push_back(RTC_SCENE_STATIC);
    sceneFlags.push_back(RTC_SCENE_STATIC | RTC_SCENE_ROBUST);
    sceneFlags.push_back(RTC_SCENE_STATIC | RTC_SCENE_COMPACT);
    sceneFlags.push_back(RTC_SCENE_STATIC | RTC_SCENE_COMPACT | RTC_SCENE_ROBUST);
    sceneFlags.push_back(RTC_SCENE_STATIC | RTC_SCENE_HIGH_QUALITY);
    sceneFlags.push_back(RTC_SCENE_DYNAMIC);
    sceneFlags.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_ROBUST);
    sceneFlags.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_COMPACT);
    sceneFlags.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_COMPACT | RTC_SCENE_ROBUST);

    sceneFlagsRobust.push_back(RTC_SCENE_STATIC  | RTC_SCENE_ROBUST);
    sceneFlagsRobust.push_back(RTC_SCENE_STATIC  | RTC_SCENE_ROBUST | RTC_SCENE_COMPACT);
    sceneFlagsRobust.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_ROBUST);
    sceneFlagsRobust.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_ROBUST | RTC_SCENE_COMPACT);

    sceneFlagsDynamic.push_back(RTC_SCENE_DYNAMIC);
    sceneFlagsDynamic.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_ROBUST);
    sceneFlagsDynamic.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_COMPACT);
    sceneFlagsDynamic.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_COMPACT | RTC_SCENE_ROBUST);

    /**************************************************************************/
    /*                      Smaller API Tests                                 */
    /**************************************************************************/

    std::stack<Ref<TestGroup>> groups;
    groups.push(tests.dynamicCast<TestGroup>());
    auto push = [&] (Ref<TestGroup> group) {
      groups.top()->add(group.dynamicCast<Test>());
      groups.push(group);
    };

    groups.top()->add(new InitExitTest("init_exit"));
    
    /* add Embree internal tests */
    for (size_t i=2000000; i<3000000; i++) {
      const char* testName = (const char*) rtcDeviceGetParameter1i(device,(RTCParameter)i);
      if (testName == nullptr) break;
      groups.top()->add(new EmbreeInternalTest(testName,i-2000000));
    }
    groups.top()->add(new os_shrink_test());

    for (auto isa : isas)
    {
      push(new TestGroup(stringOfISA(isa),false,false));
      
      groups.top()->add(new MultipleDevicesTest("multiple_devices",isa));

      push(new TestGroup("flags",true,true));
      groups.top()->add(new FlagsTest("static_static"     ,isa,VerifyApplication::TEST_SHOULD_PASS, RTC_SCENE_STATIC, RTC_GEOMETRY_STATIC));
      groups.top()->add(new FlagsTest("static_deformable" ,isa,VerifyApplication::TEST_SHOULD_FAIL, RTC_SCENE_STATIC, RTC_GEOMETRY_DEFORMABLE));
      groups.top()->add(new FlagsTest("static_dynamic"    ,isa,VerifyApplication::TEST_SHOULD_FAIL, RTC_SCENE_STATIC, RTC_GEOMETRY_DYNAMIC));
      groups.top()->add(new FlagsTest("dynamic_static"    ,isa,VerifyApplication::TEST_SHOULD_PASS, RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC));
      groups.top()->add(new FlagsTest("dynamic_deformable",isa,VerifyApplication::TEST_SHOULD_PASS, RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DEFORMABLE));
      groups.top()->add(new FlagsTest("dynamic_dynamic"   ,isa,VerifyApplication::TEST_SHOULD_PASS, RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DYNAMIC));    
      groups.pop();

      groups.top()->add(new UnmappedBeforeCommitTest("unmapped_before_commit",isa));

      push(new TestGroup("get_bounds",true,true));
      for (auto gtype : gtypes_all)
        groups.top()->add(new GetBoundsTest(to_string(gtype),isa,gtype));
      groups.pop();

      push(new TestGroup("get_linear_bounds",true,true));
      for (auto gtype : gtypes_all)
        groups.top()->add(new GetLinearBoundsTest(to_string(gtype),isa,gtype));
      groups.pop();
      
      groups.top()->add(new GetUserDataTest("get_user_data",isa));

      push(new TestGroup("buffer_stride",true,true));
      for (auto gtype : gtypes)
        groups.top()->add(new BufferStrideTest(to_string(gtype),isa,gtype));
      groups.pop();
      
      /**************************************************************************/
      /*                        Builder Tests                                   */
      /**************************************************************************/
      
      push(new TestGroup("empty_scene",true,true));
      for (auto sflags : sceneFlags) 
        groups.top()->add(new EmptySceneTest(to_string(sflags),isa,sflags));
      groups.pop();
      
      push(new TestGroup("empty_geometry",true,true));
      for (auto sflags : sceneFlags) 
        groups.top()->add(new EmptyGeometryTest(to_string(sflags),isa,sflags,RTC_GEOMETRY_STATIC));
      groups.pop();
      
      push(new TestGroup("many_build",false,false,false));
      for (auto sflags : sceneFlags) 
        groups.top()->add(new ManyBuildTest(to_string(sflags),isa,sflags,RTC_GEOMETRY_STATIC));
      groups.pop();

      push(new TestGroup("build",true,true));
      for (auto sflags : sceneFlags) 
        groups.top()->add(new BuildTest(to_string(sflags),isa,sflags,RTC_GEOMETRY_STATIC));
      groups.pop();
      
      push(new TestGroup("overlapping_primitives",true,true));
      for (auto sflags : sceneFlags)
        groups.top()->add(new OverlappingGeometryTest(to_string(sflags),isa,sflags,RTC_GEOMETRY_STATIC,clamp(int(intensity*10000),1000,100000)));
      groups.pop();

      push(new TestGroup("new_delete_geometry",true,true));
      for (auto sflags : sceneFlagsDynamic) 
        groups.top()->add(new NewDeleteGeometryTest(to_string(sflags),isa,sflags));
      groups.pop();

      push(new TestGroup("user_geometry_id",true,true));
      for (auto sflags : sceneFlagsDynamic) 
        groups.top()->add(new UserGeometryIDTest(to_string(sflags),isa,sflags));
      groups.pop();
      
      push(new TestGroup("enable_disable_geometry",true,true));
      for (auto sflags : sceneFlagsDynamic) 
        groups.top()->add(new EnableDisableGeometryTest(to_string(sflags),isa,sflags));
      groups.pop();
      
      push(new TestGroup("update",true,true));
      for (auto sflags : sceneFlagsDynamic) {
        for (auto imode : intersectModes) {
          for (auto ivariant : intersectVariants) {
            if (has_variant(imode,ivariant)) {
              groups.top()->add(new UpdateTest("deformable."+to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_DEFORMABLE,imode,ivariant));
              groups.top()->add(new UpdateTest("dynamic."+to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_DYNAMIC,imode,ivariant));
            }
          }
        }
      }
      groups.pop();

      groups.top()->add(new GarbageGeometryTest("build_garbage_geom",isa));

      GeometryType gtypes_memory[] = { TRIANGLE_MESH, TRIANGLE_MESH_MB, QUAD_MESH, QUAD_MESH_MB, HAIR_GEOMETRY, HAIR_GEOMETRY_MB, LINE_GEOMETRY, LINE_GEOMETRY_MB };
      std::vector<std::pair<RTCSceneFlags,RTCGeometryFlags>> sflags_gflags_memory;
      sflags_gflags_memory.push_back(std::make_pair(RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC));
      sflags_gflags_memory.push_back(std::make_pair(RTC_SCENE_STATIC | RTC_SCENE_COMPACT,RTC_GEOMETRY_STATIC));
      sflags_gflags_memory.push_back(std::make_pair(RTC_SCENE_STATIC | RTC_SCENE_ROBUST,RTC_GEOMETRY_STATIC));
      sflags_gflags_memory.push_back(std::make_pair(RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DYNAMIC));

      push(new TestGroup("memory_consumption",false,false));

      for (auto gtype : gtypes_memory)
        for (auto sflags : sflags_gflags_memory)
          groups.top()->add(new MemoryConsumptionTest(to_string(gtype)+"."+to_string(sflags.first,sflags.second),isa,gtype,sflags.first,sflags.second));

      groups.pop();

      /**************************************************************************/
      /*                     Interpolation Tests                                */
      /**************************************************************************/
      
      push(new TestGroup("interpolate",false,false));
      int interpolateTests[] = { 4,5,8,11,12,15 };

      push(new TestGroup("triangles",true,true));
      for (auto s : interpolateTests) 
        groups.top()->add(new InterpolateTrianglesTest(std::to_string((long long)(s)),isa,s));
      groups.pop();

      push(new TestGroup("subdiv",true,true));
      for (auto s : interpolateTests)
        groups.top()->add(new InterpolateSubdivTest(std::to_string((long long)(s)),isa,s));
      groups.pop();
        
      push(new TestGroup("hair",true,true));
      for (auto s : interpolateTests) 
        groups.top()->add(new InterpolateHairTest(std::to_string((long long)(s)),isa,s));
      groups.pop();

      groups.pop();
      
      /**************************************************************************/
      /*                      Intersection Tests                                */
      /**************************************************************************/

      push(new TestGroup("triangle_hit",true,true));
      for (auto sflags : sceneFlags) 
        for (auto imode : intersectModes) 
          for (auto ivariant : intersectVariants)
            if (has_variant(imode,ivariant))
                groups.top()->add(new TriangleHitTest(to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
      groups.pop();
      
      push(new TestGroup("quad_hit",true,true));
      for (auto sflags : sceneFlags) 
        for (auto imode : intersectModes) 
          for (auto ivariant : intersectVariants)
            if (has_variant(imode,ivariant))
                groups.top()->add(new QuadHitTest(to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
      groups.pop();

      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_RAY_MASK)) 
      {
        push(new TestGroup("ray_masks",true,true));
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              if (has_variant(imode,ivariant))
                  groups.top()->add(new RayMasksTest(to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
        groups.pop();
      }
      
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_BACKFACE_CULLING)) 
      {
        push(new TestGroup("backface_culling",true,true));
        for (auto gtype : gtypes)
          for (auto sflags : sceneFlags) 
            for (auto imode : intersectModes) 
              for (auto ivariant : intersectVariants)
                if (has_variant(imode,ivariant))
                    groups.top()->add(new BackfaceCullingTest(to_string(gtype,sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,gtype,imode,ivariant));
        groups.pop();
      }
      
      push(new TestGroup("intersection_filter",true,true));
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECTION_FILTER)) 
      {
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              if (has_variant(imode,ivariant))
                  groups.top()->add(new IntersectionFilterTest("triangles."+to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,false,imode,ivariant));
        
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              if (has_variant(imode,ivariant))
                  groups.top()->add(new IntersectionFilterTest("subdiv."+to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,true,imode,ivariant));
      }
      groups.pop();
      
      push(new TestGroup("inactive_rays",true,true));
      for (auto sflags : sceneFlags) 
        for (auto imode : intersectModes) 
          for (auto ivariant : intersectVariants)
            if (has_variant(imode,ivariant))
                if (imode != MODE_INTERSECT1) // INTERSECT1 does not support disabled rays
                  groups.top()->add(new InactiveRaysTest(to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
      groups.pop();
      
      push(new TestGroup("watertight_triangles",true,true)); {
        std::string watertightModels [] = {"sphere.triangles", "plane.triangles"};
        const Vec3fa watertight_pos = Vec3fa(148376.0f,1234.0f,-223423.0f);
        for (auto sflags : sceneFlagsRobust) 
          for (auto imode : intersectModes) 
            for (std::string model : watertightModels) 
              groups.top()->add(new WatertightTest(to_string(sflags,imode)+"."+model,isa,sflags,imode,model,watertight_pos));
        groups.pop();
      }

      push(new TestGroup("watertight_triangles_mb",true,true)); {
        std::string watertightModels [] = {"sphere.triangles_mb", "plane.triangles_mb"};
        const Vec3fa watertight_pos = Vec3fa(148376.0f,1234.0f,-223423.0f);
        for (auto sflags : sceneFlagsRobust) 
          for (auto imode : intersectModes) 
            for (std::string model : watertightModels) 
              groups.top()->add(new WatertightTest(to_string(sflags,imode)+"."+model,isa,sflags,imode,model,watertight_pos));
        groups.pop();
      }

      push(new TestGroup("watertight_quads",true,true)); {
        std::string watertightModels [] = {"sphere.quads", "plane.quads"};
        const Vec3fa watertight_pos = Vec3fa(148376.0f,1234.0f,-223423.0f);
        for (auto sflags : sceneFlagsRobust) 
          for (auto imode : intersectModes) 
            for (std::string model : watertightModels) 
              groups.top()->add(new WatertightTest(to_string(sflags,imode)+"."+model,isa,sflags,imode,model,watertight_pos));
        groups.pop();
      }

      push(new TestGroup("watertight_quads_mb",true,true)); {
        std::string watertightModels [] = {"sphere.quads_mb", "plane.quads_mb"};
        const Vec3fa watertight_pos = Vec3fa(148376.0f,1234.0f,-223423.0f);
        for (auto sflags : sceneFlagsRobust) 
          for (auto imode : intersectModes) 
            for (std::string model : watertightModels) 
              groups.top()->add(new WatertightTest(to_string(sflags,imode)+"."+model,isa,sflags,imode,model,watertight_pos));
        groups.pop();
      }

      push(new TestGroup("watertight_subdiv",true,true)); {
        std::string watertightModels [] = { "sphere.subdiv", "plane.subdiv"};
        const Vec3fa watertight_pos = Vec3fa(148376.0f,1234.0f,-223423.0f);
        for (auto sflags : sceneFlagsRobust) 
          for (auto imode : intersectModes) 
            for (std::string model : watertightModels) 
              groups.top()->add(new WatertightTest(to_string(sflags,imode)+"."+model,isa,sflags,imode,model,watertight_pos));
        groups.pop();
      }
      
      /*push(new TestGroup("small_triangle_hit_test",true,true)); {
        const Vec3fa pos = Vec3fa(0.0f,0.0f,0.0f);
        const float radius = 1000000.0f;
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            groups.top()->add(new SmallTriangleHitTest(to_string(sflags,imode),isa,sflags,imode,pos,radius));
        groups.pop();
        }*/
      
      push(new TestGroup("ray_alignment_test",true,true)); {
        std::string watertightModels [] = {"sphere.triangles", "sphere.quads", "sphere.subdiv" };
        for (auto sflags : sceneFlagsRobust) 
          for (auto imode : intersectModes) 
            for (std::string model : watertightModels) 
              groups.top()->add(new RayAlignmentTest(to_string(sflags,imode)+"."+model,isa,sflags,imode,model));
        groups.pop();
      }

      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_IGNORE_INVALID_RAYS))
      {
        push(new TestGroup("nan_test",true,false));
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              if (has_variant(imode,ivariant))
                  groups.top()->add(new NaNTest(to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
        groups.pop();
        
        push(new TestGroup("inf_test",true,false));
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              if (has_variant(imode,ivariant))
                  groups.top()->add(new InfTest(to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
        groups.pop();
      }
    
      /**************************************************************************/
      /*                  Randomized Stress Testing                             */
      /**************************************************************************/
      
      groups.top()->add(new IntensiveRegressionTest("regression_static",isa,rtcore_regression_static_thread,0,30));
      groups.top()->add(new IntensiveRegressionTest("regression_dynamic",isa,rtcore_regression_dynamic_thread,0,30));

      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_COMMIT_THREAD)) {
	groups.top()->add(new IntensiveRegressionTest("regression_static_user_threads", isa,rtcore_regression_static_thread,1,30));
	groups.top()->add(new IntensiveRegressionTest("regression_dynamic_user_threads",isa,rtcore_regression_dynamic_thread,1,30));
      }

      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_COMMIT_JOIN)) {
	groups.top()->add(new IntensiveRegressionTest("regression_static_build_join", isa,rtcore_regression_static_thread,2,30));
	groups.top()->add(new IntensiveRegressionTest("regression_dynamic_build_join",isa,rtcore_regression_dynamic_thread,2,30));
      }

      groups.top()->add(new MemoryMonitorTest("regression_static_memory_monitor", isa,rtcore_regression_static_thread,30));
      groups.top()->add(new MemoryMonitorTest("regression_dynamic_memory_monitor",isa,rtcore_regression_dynamic_thread,30));

      /**************************************************************************/
      /*                           Benchmarks                                   */
      /**************************************************************************/
      
      push(new TestGroup("benchmarks",false,false));

      std::vector<std::pair<RTCSceneFlags,RTCGeometryFlags>> benchmark_sflags_gflags;
      benchmark_sflags_gflags.push_back(std::make_pair(RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC));
      benchmark_sflags_gflags.push_back(std::make_pair(RTC_SCENE_STATIC | RTC_SCENE_ROBUST,RTC_GEOMETRY_STATIC));
      benchmark_sflags_gflags.push_back(std::make_pair(RTC_SCENE_STATIC | RTC_SCENE_COMPACT,RTC_GEOMETRY_STATIC));
      benchmark_sflags_gflags.push_back(std::make_pair(RTC_SCENE_STATIC | RTC_SCENE_COMPACT | RTC_SCENE_ROBUST,RTC_GEOMETRY_STATIC));
      benchmark_sflags_gflags.push_back(std::make_pair(RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DYNAMIC));

      std::vector<std::pair<IntersectMode,IntersectVariant>> benchmark_imodes_ivariants;
      benchmark_imodes_ivariants.push_back(std::make_pair(MODE_INTERSECT1,VARIANT_INTERSECT));
      benchmark_imodes_ivariants.push_back(std::make_pair(MODE_INTERSECT1,VARIANT_OCCLUDED));
      benchmark_imodes_ivariants.push_back(std::make_pair(MODE_INTERSECT4,VARIANT_INTERSECT));
      benchmark_imodes_ivariants.push_back(std::make_pair(MODE_INTERSECT4,VARIANT_OCCLUDED));
      benchmark_imodes_ivariants.push_back(std::make_pair(MODE_INTERSECT8,VARIANT_INTERSECT));
      benchmark_imodes_ivariants.push_back(std::make_pair(MODE_INTERSECT8,VARIANT_OCCLUDED));
      benchmark_imodes_ivariants.push_back(std::make_pair(MODE_INTERSECT16,VARIANT_INTERSECT));
      benchmark_imodes_ivariants.push_back(std::make_pair(MODE_INTERSECT16,VARIANT_OCCLUDED));
      benchmark_imodes_ivariants.push_back(std::make_pair(MODE_INTERSECT1M,VARIANT_INTERSECT_INCOHERENT));
      benchmark_imodes_ivariants.push_back(std::make_pair(MODE_INTERSECT1M,VARIANT_OCCLUDED_INCOHERENT));

      GeometryType benchmark_gtypes[] = { 
        TRIANGLE_MESH, 
        TRIANGLE_MESH_MB, 
        QUAD_MESH, 
        QUAD_MESH_MB, 
        SUBDIV_MESH, 
        //SUBDIV_MESH_MB  // FIXME: not supported yet
        // FIXME: use more geometry types
      };

      groups.top()->add(new SimpleBenchmark("simple",isa));
      
      for (auto gtype : benchmark_gtypes)
        for (auto sflags : benchmark_sflags_gflags) 
          for (auto imode : benchmark_imodes_ivariants)
            groups.top()->add(new CoherentRaysBenchmark("coherent."+to_string(gtype)+"_1000k."+to_string(sflags.first,imode.first,imode.second),
                                                        isa,gtype,sflags.first,sflags.second,imode.first,imode.second,501));

      for (auto gtype : benchmark_gtypes)
        for (auto sflags : benchmark_sflags_gflags) 
          for (auto imode : benchmark_imodes_ivariants)
            groups.top()->add(new IncoherentRaysBenchmark("incoherent."+to_string(gtype)+"_1000k."+to_string(sflags.first,imode.first,imode.second),
                                                          isa,gtype,sflags.first,sflags.second,imode.first,imode.second,501));

      std::vector<std::pair<RTCSceneFlags,RTCGeometryFlags>> benchmark_create_sflags_gflags;
      benchmark_create_sflags_gflags.push_back(std::make_pair(RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC));
      //benchmark_create_sflags_gflags.push_back(std::make_pair(RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC));
      //benchmark_create_sflags_gflags.push_back(std::make_pair(RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DEFORMABLE));
      benchmark_create_sflags_gflags.push_back(std::make_pair(RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DYNAMIC));

      GeometryType benchmark_create_gtypes[] = { 
        TRIANGLE_MESH, 
        TRIANGLE_MESH_MB, 
        QUAD_MESH, 
        QUAD_MESH_MB, 
        HAIR_GEOMETRY,
        HAIR_GEOMETRY_MB,
        LINE_GEOMETRY,
        LINE_GEOMETRY_MB
      };

      std::vector<std::tuple<const char*,int,int>> num_primitives;
      num_primitives.push_back(std::make_tuple("120",6,1));
      num_primitives.push_back(std::make_tuple("1k" ,17,1));
      num_primitives.push_back(std::make_tuple("10k",51,1));
      num_primitives.push_back(std::make_tuple("100k",159,1));
      num_primitives.push_back(std::make_tuple("10000k_1",801,1));
      num_primitives.push_back(std::make_tuple("1000k_1",501,1));
      num_primitives.push_back(std::make_tuple("100k_10",159,10));
      num_primitives.push_back(std::make_tuple("10k_100",51,100));
      num_primitives.push_back(std::make_tuple("1k_1000",17,1000));

      for (auto gtype : benchmark_create_gtypes)
        for (auto sflags : benchmark_create_sflags_gflags)
          for (auto num_prims : num_primitives)
            groups.top()->add(new CreateGeometryBenchmark("create."+to_string(gtype)+"_"+std::get<0>(num_prims)+"."+to_string(sflags.first,sflags.second),
                                                          isa,gtype,sflags.first,sflags.second,std::get<1>(num_prims),std::get<2>(num_prims),false,true));

      std::vector<std::pair<RTCSceneFlags,RTCGeometryFlags>> benchmark_update_sflags_gflags;
      benchmark_update_sflags_gflags.push_back(std::make_pair(RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC));
      benchmark_update_sflags_gflags.push_back(std::make_pair(RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DEFORMABLE));
      benchmark_update_sflags_gflags.push_back(std::make_pair(RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DYNAMIC));

      for (auto gtype : benchmark_create_gtypes)
        for (auto sflags : benchmark_update_sflags_gflags)
          for (auto num_prims : num_primitives)
            groups.top()->add(new CreateGeometryBenchmark("update."+to_string(gtype)+"_"+std::get<0>(num_prims)+"."+to_string(sflags.first,sflags.second),
                                                          isa,gtype,sflags.first,sflags.second,std::get<1>(num_prims),std::get<2>(num_prims),true,true));

      groups.pop(); // benchmarks

      /**************************************************************************/
      /*                        Memory Consumption                              */
      /**************************************************************************/
   
      push(new TestGroup("embree_reported_memory",false,false));

      for (auto gtype : benchmark_create_gtypes)
        for (auto sflags : benchmark_create_sflags_gflags)
          for (auto num_prims : num_primitives)
            groups.top()->add(new CreateGeometryBenchmark(to_string(gtype)+"_"+std::get<0>(num_prims)+"."+to_string(sflags.first,sflags.second),
                                                          isa,gtype,sflags.first,sflags.second,std::get<1>(num_prims),std::get<2>(num_prims),false,false));

      groups.pop(); // embree_reported_memory

      groups.pop(); // isa
    }

    prefix_test_names(tests);

    /* ignore failure of some tests that are known to fail */
    map_tests(tests, [&] (Ref<Test> test) { 
        if (test->name.find("watertight_subdiv") != std::string::npos) test->ignoreFailure = true;
      });
    
    /**************************************************************************/
    /*                     Command Line Parsing                               */
    /**************************************************************************/
  
    registerOption("run", [this] (Ref<ParseStream> cin, const FileName& path) {
        if (!user_specified_tests) enable_disable_all_tests(tests,false);
        user_specified_tests = true;
        std::string regex = cin->getString();
        if (!enable_disable_some_tests(tests,regex,true)) {
          std::cout << "no tests matched regular expression " << regex << std::endl;
          exit(1);
        }
      }, "--run <regexpr>: Runs all tests whose name match the regular expression. If no test matches the application fails.");

    registerOption("enable", [this] (Ref<ParseStream> cin, const FileName& path) {
        if (!user_specified_tests) enable_disable_all_tests(tests,false);
        user_specified_tests = true;
        std::string regex = cin->getString();
        enable_disable_some_tests(tests,regex,true);
      }, "--enable <regexpr>: Enables all tests whose name matches the regular expression.");

    registerOption("skip", [this] (Ref<ParseStream> cin, const FileName& path) {
        user_specified_tests = true;
        std::string regex = cin->getString();
        enable_disable_some_tests(tests,regex,false);
      }, "--skip <regexpr>: Skips all tests whose name matches the regular expression.");
    registerOptionAlias("skip","disable");

    registerOption("skip-before", [this] (Ref<ParseStream> cin, const FileName& path) {
        user_specified_tests = true;
        bool found = false;
        std::string regex = cin->getString();
        map_tests(tests, [&] (Ref<Test> test) {
            if (regex_match(test->name,regex)) found = true;
            test->enabled &= found;
          });
      }, "--skip-before <regexpr>: Skips all tests before the first test matching the regular expression.");
    registerOptionAlias("skip-before","disable-before");

    registerOption("flatten", [this] (Ref<ParseStream> cin, const FileName& path) {
        flatten = false;
      }, "--flatten: shows all leaf test names when executing tests");
    
    registerOption("sequential", [this] (Ref<ParseStream> cin, const FileName& path) {
        parallel = false;
      }, "--sequential: execute all tests sequentially");

    registerOption("parallel", [this] (Ref<ParseStream> cin, const FileName& path) {
        parallel = true;
      }, "--parallel: parallelized test execution (default)");

    registerOption("colors", [this] (Ref<ParseStream> cin, const FileName& path) {
        usecolors = true;
      }, "--colors: do use shell colors");

    registerOption("no-colors", [this] (Ref<ParseStream> cin, const FileName& path) {
        usecolors = false;
      }, "--no-colors: do not use shell colors");

    registerOption("cdash", [this] (Ref<ParseStream> cin, const FileName& path) {
        cdash = true;
      }, "--cdash: prints cdash measurements");

    registerOption("database", [this] (Ref<ParseStream> cin, const FileName& path) {
        database = cin->getString();
        update_database = true;
      }, "--database <folder>: location to folder containing the measurement database");

    registerOption("compare", [this] (Ref<ParseStream> cin, const FileName& path) {
        database = cin->getString();
        update_database = false;
      }, "--compare <folder>: compares with database, but does not update database");

    registerOption("benchmark-tolerance", [this] (Ref<ParseStream> cin, const FileName& path) {
        benchmark_tolerance = cin->getFloat();
      }, "--benchmark-tolerance: maximal relative slowdown to let a test pass");
    registerOptionAlias("benchmark-tolerance","tolerance");

    registerOption("print-tests", [this] (Ref<ParseStream> cin, const FileName& path) {
        print_tests(tests,0);
        exit(1);
      }, "--print-tests: prints all enabled test names");
    registerOptionAlias("print-tests","print-names");

    registerOption("print-ctests", [this] (Ref<ParseStream> cin, const FileName& path) {
        print_ctests(tests,0);
        exit(1);
      }, "--print-ctests: prints all test in a form to add to CMakeLists.txt");
    
    registerOption("intensity", [this] (Ref<ParseStream> cin, const FileName& path) {
        intensity = cin->getFloat();
      }, "--intensity <float>: intensity of testing to perform");

    registerOption("plot-over-primitives", [this] (Ref<ParseStream> cin, const FileName& path) {
        std::vector<Ref<Benchmark>> benchmarks;
        FileName outFileName = parse_benchmark_list(cin,benchmarks);
        plot(benchmarks,outFileName,"#primitives",1000,1100000,1.2f,0,[&] (Ref<Benchmark> benchmark, size_t& N) {
            N = benchmark->setNumPrimitives(N);
            benchmark->setup(this);
            Statistics stat = benchmark->benchmark_loop(this);
            benchmark->cleanup(this);
            return stat;
          });
        exit(1);
      }, "--plot-over-primitives <benchmark0> <benchmark1> ... <outfile>: Plots performance over number of primitives to outfile for the specified benchmarks.");

    registerOption("plot-over-threads", [this] (Ref<ParseStream> cin, const FileName& path) {
        std::vector<Ref<Benchmark>> benchmarks;
        FileName outFileName = parse_benchmark_list(cin,benchmarks);
        plot(benchmarks,outFileName,"#threads",2,getNumberOfLogicalThreads(),1.0f,2,[&] (Ref<Benchmark> benchmark, size_t N) {
            benchmark->setNumThreads(N);
            benchmark->setup(this);
            Statistics stat = benchmark->benchmark_loop(this);
            benchmark->cleanup(this);
            return stat;
          });
        exit(1);
      }, "--plot-over-primitives <benchmark0> <benchmark1> ... <outfile>: Plots performance over number of primitives to outfile for the specified benchmarks.");

    /* the internal tasking system need the device to be present to allow parallel test execution */
#if !defined(TASKING_INTERNAL)
    device = nullptr;
#endif
  }

  void VerifyApplication::prefix_test_names(Ref<Test> test, std::string prefix)
  {
    if (Ref<TestGroup> group = test.dynamicCast<TestGroup>()) 
      for (auto& t : group->tests) 
        prefix_test_names(t,group->extend_prefix(prefix));

    test->name = prefix + test->name;
    name2test[test->name] = test;
  }

  FileName VerifyApplication::parse_benchmark_list(Ref<ParseStream> cin, std::vector<Ref<Benchmark>>& benchmarks)
  {
    std::vector<std::string> regexpr;
    while (cin->peek() != "" && cin->peek()[0] != '-') 
      regexpr.push_back(cin->getString());
    
    if (regexpr.size() == 0) throw std::runtime_error("no output file specified");
    FileName outFileName = regexpr.back();
    regexpr.pop_back();
    
    for (size_t i=0; i<regexpr.size(); i++) 
    {
      map_tests(tests,[&] (Ref<Test> test) {
          if (!regex_match(test->name,regexpr[i])) return;
          Ref<Benchmark> benchmark = test.dynamicCast<Benchmark>();
          if (!benchmark) return;
          benchmarks.push_back(benchmark);
        });
    }
    return outFileName;
  }

  bool VerifyApplication::update_tests(Ref<Test> test)
  {
    if (Ref<TestGroup> group = test.dynamicCast<TestGroup>()) 
    {
      bool any_enabled = false;
      for (auto t : group->tests) any_enabled |= update_tests(t);
      return test->enabled = any_enabled;
    } 
    else 
      return test->enabled;
  }

  void VerifyApplication::print_tests(Ref<Test> test, size_t depth)
  {
    if (!test->isEnabled()) return;

    if (Ref<TestGroup> group = test.dynamicCast<TestGroup>()) 
    {
      std::cout << std::string(2*depth,' ') << group->name << (group->name != "" ? " " : "") << "{" << std::endl;
      for (auto& t : group->tests) print_tests(t,depth+1);
      std::cout << std::string(2*depth,' ') << "}" << std::endl;
    } else {
      std::cout << std::string(2*depth,' ') << test->name << std::endl;
    }
  }

  void VerifyApplication::print_ctests(Ref<Test> test, size_t depth)
  {
    if (!test->isEnabled()) return;

    if (Ref<TestGroup> group = test.dynamicCast<TestGroup>()) 
    {
      for (auto& t : group->tests) print_ctests(t,depth+1);
    } else {
      std::cout << "ADD_TEST(NAME " << test->name << " COMMAND verify --no-colors --cdash-measurements --run " << test->name << std::endl;
    }
  }

  template<typename Function> 
  void VerifyApplication::map_tests(Ref<Test> test, const Function& f)
  {
    if (Ref<TestGroup> group = test.dynamicCast<TestGroup>()) {
      for (auto& t : group->tests) map_tests(t,f);
    } else {
      f(test);
    }
  }

  void VerifyApplication::enable_disable_all_tests(Ref<Test> test, bool enabled)
  {
    map_tests(test, [&] (Ref<Test> test) { test->enabled = enabled; });
    update_tests(test);
  }

  size_t VerifyApplication::enable_disable_some_tests(Ref<Test> test, std::string regex, bool enabled)
  {
    size_t N = 0;
    map_tests(test, [&] (Ref<Test> test) { 
        if (regex_match(test->name,regex)) {
          test->enabled = enabled;
          N++;
        }
      });
   update_tests(test);
   return N;
  }

  template<typename Closure>
  void VerifyApplication::plot(std::vector<Ref<Benchmark>> benchmarks, const FileName outFileName, std::string xlabel, size_t startN, size_t endN, float f, size_t dn, const Closure& test)
  {
    std::fstream plot;
    plot.open(outFileName, std::fstream::out | std::fstream::trunc);
    plot << "set key inside right top vertical Right noreverse enhanced autotitles box linetype -1 linewidth 1.000" << std::endl;
    plot << "set samples 50, 50" << std::endl;
    plot << "set title \"" << outFileName.name() << "\"" << std::endl; 
    plot << "set xlabel \"" + xlabel + "\"" << std::endl;
    if (f != 1.0f) plot << "set logscale x" << std::endl;
    if (benchmarks.size())
      plot << "set ylabel \"" << benchmarks[0]->unit << "\"" << std::endl;
    plot << "set yrange [0:]" << std::endl;

    plot << "plot \\" << std::endl;
    for (size_t i=0; i<benchmarks.size(); i++) {
      plot << "\"" << outFileName.name() << "." << benchmarks[i]->name << ".txt\" using 1:2 title \"" << benchmarks[i]->name << "\" with lines";
      if (i != benchmarks.size()-1) plot << ",\\";
      plot << std::endl;      
    }
    plot << std::endl;
    plot.close();
    
    for (auto benchmark : benchmarks) 
    {
      std::fstream data;
      data.open(outFileName.name()+"."+benchmark->name+".txt", std::fstream::out | std::fstream::trunc);
      std::cout << benchmark->name << std::endl;
      for (size_t i=startN; i<=endN; i=size_t(i*f)+dn) 
      {
        size_t N = i;
        Statistics stat = test(benchmark,N);
        data << " " << N << " " << stat.getAvg() << std::endl;
        std::cout<< " " << N << " " << stat.getAvg() << std::endl;
      }
      data.close();
    }
  }

  int VerifyApplication::main(int argc, char** argv) try
  {
    /* for best performance set FTZ and DAZ flags in MXCSR control and status register */
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    /* parse command line options */
    parseCommandLine(argc,argv);

    /* run all enabled tests */
    tests->execute(this,false);

    /* print result */
    std::cout << std::endl;
    std::cout << std::setw(TEXT_ALIGN) << "Tests passed" << ": " << numPassedTests << std::endl; 
    std::cout << std::setw(TEXT_ALIGN) << "Tests failed" << ": " << numFailedTests << std::endl; 
    std::cout << std::setw(TEXT_ALIGN) << "Tests failed and ignored" << ": " << numFailedAndIgnoredTests << std::endl; 
    std::cout << std::endl;

    return (int)numFailedTests;
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

int main(int argc, char** argv)
{
  embree::VerifyApplication app;
  return app.main(argc,argv);
}
