// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "nanite_geometry_device.h"

#if defined(USE_GLFW)

/* include GLFW for window management */
#include <GLFW/glfw3.h>

/* include ImGUI */
#include "../common/imgui/imgui.h"
#include "../common/imgui/imgui_impl_glfw_gl2.h"

#endif

namespace embree {

  template<typename Ty>
    struct Averaged
  {
    Averaged (size_t N, double dt)
    : N(N), dt(dt) {}

    void add(double v)
    {
      values.push_front(std::make_pair(getSeconds(),v));
      if (values.size() > N) values.resize(N);
    }

    Ty get() const
    {
      if (values.size() == 0) return zero;
      double t_begin = values[0].first-dt;

      Ty sum(zero);
      size_t num(0);
      for (size_t i=0; i<values.size(); i++) {
        if (values[i].first >= t_begin) {
          sum += values[i].second;
          num++;
        }
      }
      if (num == 0) return 0;
      else return sum/Ty(num);
    }

    std::deque<std::pair<double,Ty>> values;
    size_t N;
    double dt;
  };


  
#define FEATURE_MASK                            \
  RTC_FEATURE_FLAG_TRIANGLE |                   \
  RTC_FEATURE_FLAG_INSTANCE
  
  RTCScene g_scene  = nullptr;
  TutorialData data;


  extern "C" RenderMode user_rendering_mode = RENDER_PRIMARY;
  extern "C" uint user_spp = 1;

  Averaged<double> avg_bvh_build_time(64,1.0);
  Averaged<double> avg_lod_selection_crack_fixing_time(64,1.0);
  


  // =========================================================================================================================================================
  // =========================================================================================================================================================
  // =========================================================================================================================================================

  template<typename T>
  static __forceinline uint as_uint(T t)
  {
    return __builtin_bit_cast(uint,t);
  }

  template<typename T>
  static __forceinline float as_float(T t)
  {
    return __builtin_bit_cast(float,t);
  }


  enum {
    NO_BORDER     = 0,
    TOP_BORDER    = 1 << 0,
    BOTTOM_BORDER = 1 << 1,
    LEFT_BORDER   = 1 << 2,
    RIGHT_BORDER  = 1 << 3,
    FULL_BORDER   = TOP_BORDER|BOTTOM_BORDER|LEFT_BORDER|RIGHT_BORDER,
    CRACK_FIXED_BORDER = 1 << 31    
  };
  
  struct __aligned(32) LCGQuadNode
  {
    RTCLossyCompressedGrid grid;
    uint flags;
    uint lod_level;
    uint ID;
    uint localID;
    Vec2f u_range;
    Vec2f v_range;    

    __forceinline bool hasBorder() const { return flags & FULL_BORDER; }
    __forceinline Vec3f getVertex(const uint x, const uint y) { return Vec3f(grid.vertex[y][x][0],grid.vertex[y][x][1],grid.vertex[y][x][2]); }
    
  }; 

  __forceinline BBox3f getGridBounds(const RTCLossyCompressedGrid &grid)
  {
    BBox3f bounds(empty);
    for (uint y=0;y<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES;y++)
      for (uint x=0;x<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES;x++)
      {
        Vec3f v(grid.vertex[y][x][0],grid.vertex[y][x][1],grid.vertex[y][x][2]);
        bounds.extend(v);
      }
    return bounds;
  }

  
  static const uint LOD_LEVELS = 3;
  static const uint NUM_TOTAL_QUAD_NODES_PER_RTC_LCG = (1-(1<<(2*LOD_LEVELS)))/(1-4);

  struct LODEdgeLevel {
    uchar top,right,bottom,left; // top,right,bottom,left
    __forceinline LODEdgeLevel() {}
    __forceinline LODEdgeLevel(const uchar top, const uchar right, const uchar bottom, const uchar left) : top(top),right(right),bottom(bottom),left(left) {}
    __forceinline uint level() const { return max(max(top,right),max(bottom,left)); }

    __forceinline bool needsCrackFixing() const {
      const uint l = level();
      if ( l != top || l != right || l != bottom || l != left) return true;
      return false;
    }
  };

  __forceinline Vec2f projectVertexToPlane(const Vec3f &p, const Vec3f &vx, const Vec3f &vy, const Vec3f &vz, const uint width, const uint height)
  {
    const Vec3f vn = cross(vx,vy);    
    const float distance = (float)dot(vn,vz) / (float)dot(vn,p);
    Vec3f pip = p * distance;
    if (distance < 0.0f)
      pip = vz;
    float a = dot((pip-vz),vx);
    float b = dot((pip-vz),vy);
    a = min(max(a,0.0f),(float)width);
    b = min(max(b,0.0f),(float)height);    
    return Vec2f(a,b);
  }
  
  __forceinline LODEdgeLevel getLODEdgeLevels(LCGQuadNode &current,const ISPCCamera& camera, const uint width, const uint height)
  {
    const Vec3f v0 = current.getVertex(0,0);
    const Vec3f v1 = current.getVertex(RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1,0);
    const Vec3f v2 = current.getVertex(RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1,RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1);
    const Vec3f v3 = current.getVertex(0,RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1);

    const Vec3f vx = camera.xfm.l.vx;
    const Vec3f vy = camera.xfm.l.vy;
    const Vec3f vz = camera.xfm.l.vz;
    const Vec3f org = camera.xfm.p;

    const Vec2f p0 = projectVertexToPlane(v0-org,vx,vy,vz,width,height);
    const Vec2f p1 = projectVertexToPlane(v1-org,vx,vy,vz,width,height);
    const Vec2f p2 = projectVertexToPlane(v2-org,vx,vy,vz,width,height);
    const Vec2f p3 = projectVertexToPlane(v3-org,vx,vy,vz,width,height);

    const float f = 1.0/8.0f;
    const float d0 = length(p1-p0) * f;
    const float d1 = length(p2-p1) * f;
    const float d2 = length(p3-p2) * f;
    const float d3 = length(p0-p3) * f;

    
    int i0 = (int)floorf(d0 / RTC_LOSSY_COMPRESSED_GRID_QUAD_RES);
    int i1 = (int)floorf(d1 / RTC_LOSSY_COMPRESSED_GRID_QUAD_RES);
    int i2 = (int)floorf(d2 / RTC_LOSSY_COMPRESSED_GRID_QUAD_RES);
    int i3 = (int)floorf(d3 / RTC_LOSSY_COMPRESSED_GRID_QUAD_RES);
    
    i0 = min(max(0,i0),(int)LOD_LEVELS-1);
    i1 = min(max(0,i1),(int)LOD_LEVELS-1);
    i2 = min(max(0,i2),(int)LOD_LEVELS-1);
    i3 = min(max(0,i3),(int)LOD_LEVELS-1);

#if 0
    i0 = 2;
    i1 = 2;
    i2 = 2;
    i3 = 2;
#endif
    
    LODEdgeLevel lod_levels(i0,i1,i2,i3);
    return lod_levels;
  }

    __forceinline uint getCrackFixingBorderMask(const LCGQuadNode &current, const LODEdgeLevel &lodEdgeLevel, const uint gridLODLevel)
    {
      uint mask = 0;
      if ((current.flags & TOP_BORDER   ) && lodEdgeLevel.top    != gridLODLevel) mask |= TOP_BORDER;
      if ((current.flags & BOTTOM_BORDER) && lodEdgeLevel.bottom != gridLODLevel) mask |= BOTTOM_BORDER;
      if ((current.flags & RIGHT_BORDER ) && lodEdgeLevel.right  != gridLODLevel) mask |= RIGHT_BORDER;
      if ((current.flags & LEFT_BORDER  ) && lodEdgeLevel.left   != gridLODLevel) mask |= LEFT_BORDER;
      return mask;
    }
  
  __forceinline void fixCracks(LCGQuadNode &current, const uint borderMask, const LODEdgeLevel &lodEdgeLevel, const uint gridLODLevel)
  {
    if (borderMask & TOP_BORDER)
    {
      current.flags |= CRACK_FIXED_BORDER;
      const uint diff = gridLODLevel - lodEdgeLevel.top;
      for (uint i=1;i<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1;i++)
      {
        const uint index = (i>>diff)<<diff;
        current.grid.vertex[0][i][0] = current.grid.vertex[0][index][0];
        current.grid.vertex[0][i][1] = current.grid.vertex[0][index][1];
        current.grid.vertex[0][i][2] = current.grid.vertex[0][index][2];        
      }
    }

    if (borderMask & BOTTOM_BORDER)
    {
      current.flags |= CRACK_FIXED_BORDER;      
      const uint diff = gridLODLevel - lodEdgeLevel.bottom;
      for (uint i=1;i<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1;i++)
      {
        const uint index = (i>>diff)<<diff;
        current.grid.vertex[8][i][0] = current.grid.vertex[8][index][0];
        current.grid.vertex[8][i][1] = current.grid.vertex[8][index][1];
        current.grid.vertex[8][i][2] = current.grid.vertex[8][index][2];        
      }
    }
    
    if (borderMask & RIGHT_BORDER)
    {
      current.flags |= CRACK_FIXED_BORDER;      
      const uint diff = gridLODLevel - lodEdgeLevel.right;
      for (uint i=1;i<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1;i++)
      {
        const uint index = (i>>diff)<<diff;
        current.grid.vertex[i][8][0] = current.grid.vertex[index][8][0];
        current.grid.vertex[i][8][1] = current.grid.vertex[index][8][1];
        current.grid.vertex[i][8][2] = current.grid.vertex[index][8][2];        
      }
    }

    if (borderMask & LEFT_BORDER)
    {
      current.flags |= CRACK_FIXED_BORDER;      
      const uint diff = gridLODLevel - lodEdgeLevel.left;
      for (uint i=1;i<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1;i++)
      {
        const uint index = (i>>diff)<<diff;
        current.grid.vertex[i][0][0] = current.grid.vertex[index][0][0];
        current.grid.vertex[i][0][1] = current.grid.vertex[index][0][1];
        current.grid.vertex[i][0][2] = current.grid.vertex[index][0][2];        
      }
    }        
  }
  
  
  struct __aligned(64) LCG_LODQuadTree_Grid {    
    uint numQuadTrees;
    uint numTotalQuadNodes;
    uint numMaxResQuadNodes;

    LCGQuadNode *quadTrees;
    
    uint numLCGPtrs;
    void **lcg_ptrs;

    uint numCrackFixQuadNodes;
    LCGQuadNode *crackFixQuadNodes;
    
    RTCGeometry geometry;
    uint geomID;

    Texture* map_Kd;
    
    static __forceinline uint getQuadTreeChildIndex(const uint index)
    {
      return 4*index+1;
    }
  };

  LCG_LODQuadTree_Grid *global_grid;
  
  inline Vec3fa getVertex(const uint x, const uint y, const Vec3fa *const vtx, const uint grid_resX, const uint grid_resY)
  {
    const uint px = min(x,grid_resX-1);
    const uint py = min(y,grid_resY-1);    
    return vtx[py*grid_resX + px];
  }

  inline Vec3fa getAvgVertex(const uint x, const uint y, const Vec3fa *const vtx, const uint grid_resX, const uint grid_resY)
  {
    Vec3fa v(0.0f);
    for (uint j=y-1;j<=y+1;j++)    
      for (uint i=x-1;i<=x+1;i++)
        v+= getVertex(i,j,vtx,grid_resX,grid_resY);
    return v * 1.0f/9.0f;
  }
      
  void createQuadNode(LCGQuadNode &current, LCGQuadNode *nodes, uint &index, const uint start_x, const uint start_y, const uint step, const Vec3fa *const vtx, const uint grid_resX, const uint grid_resY, const uint border_flags, const uint ID, const uint lod_level, const Vec2f u_range, Vec2f v_range)
  {
    if (step == 0) return;
      
    for (int y=0;y<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES;y++)
      for (int x=0;x<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES;x++)
      {
        const uint px = start_x+x*step;
        const uint py = start_y+y*step;
        Vec3fa v;
        //if (x==0 || y==0 || x==RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1 || y==RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1)          
          v = getVertex(px,py,vtx,grid_resX,grid_resY);
          //else
          //v = getAvgVertex(px,py,vtx,grid_resX,grid_resY);
        current.grid.vertex[y][x][0] = v.x;
        current.grid.vertex[y][x][1] = v.y;
        current.grid.vertex[y][x][2] = v.z;          
      }
    
    current.flags     = border_flags;
    current.lod_level = lod_level;
    current.ID        = ID;
    current.localID   = index;
    current.u_range   = u_range;
    current.v_range   = v_range;    

    const uint new_step = step>>1;
    const uint new_res = RTC_LOSSY_COMPRESSED_GRID_QUAD_RES*new_step;

    const float u_center = (u_range.x + u_range.y)/2.0f;
    const float v_center = (v_range.x + v_range.y)/2.0f;
    
    if (new_step)
    {    
      const uint new_index = index;
      index += 4;
      
      createQuadNode(nodes[new_index+0],nodes,index,start_x + 0*new_res,start_y + 0*new_res,new_step,vtx,grid_resX,grid_resY,border_flags & (LEFT_BORDER|TOP_BORDER),ID,lod_level+1,Vec2f(u_range.x,u_center),Vec2f(v_range.x,v_center));
      createQuadNode(nodes[new_index+1],nodes,index,start_x + 1*new_res,start_y + 0*new_res,new_step,vtx,grid_resX,grid_resY,border_flags & (RIGHT_BORDER|TOP_BORDER),ID,lod_level+1,Vec2f(u_center,u_range.y),Vec2f(v_range.x,v_center));
      createQuadNode(nodes[new_index+2],nodes,index,start_x + 0*new_res,start_y + 1*new_res,new_step,vtx,grid_resX,grid_resY,border_flags & (LEFT_BORDER|BOTTOM_BORDER),ID,lod_level+1,Vec2f(u_range.x,u_center),Vec2f(v_center,v_range.y));
      createQuadNode(nodes[new_index+3],nodes,index,start_x + 1*new_res,start_y + 1*new_res,new_step,vtx,grid_resX,grid_resY,border_flags & (RIGHT_BORDER|BOTTOM_BORDER),ID,lod_level+1,Vec2f(u_center,u_range.y),Vec2f(v_center,v_range.y));
    }
  }

  struct LCGBP
  {
    static const uint SIGN_BIT      = 1;
    static const uint EXPONENT_BITS = 4;    
    static const uint MANTISSA_BITS = 5;
    static const uint RP_BITS       = SIGN_BIT + EXPONENT_BITS + MANTISSA_BITS;
    static const uint EXPONENT_MASK = ((uint)1 << EXPONENT_BITS)-1;    
    static const uint MANTISSA_MASK = ((uint)1 << MANTISSA_BITS)-1;
    static const uint RP_MASK       = ((uint)1 << RP_BITS)-1;

    static const uint FLOAT_EXPONENT_BIAS = 126; // 127-1 due to zigzag encoding
    static const uint FLOAT_MANTISSA_BITS = 23;
    

    static const uint GRID_RES_VERTEX = 33;
    static const uint GRID_RES_QUAD   = GRID_RES_VERTEX-1;

    const Vec3f v0,v1,v2,v3;
    uint lc_offsets[GRID_RES_VERTEX*GRID_RES_VERTEX];

    static const uint zigzagEncode(const int i)
    {
      return (i >> 31) ^ (i << 1);
    }
    
    static const int zigzagDecode(const uint i)
    {
      return (i >> 1) ^ -(i & 1);
    }
        
    static __forceinline uint encodeFloat(const float diff) 
    {
      int exponent; uint mantissa = as_uint(frexp(diff, &exponent));
      /*--- clamp exponent---*/
      exponent = max(exponent, ((int)(-1) << (EXPONENT_BITS-1))+1 );
      exponent = min(exponent, ((int)( 1) << (EXPONENT_BITS-1))   );      
      uint exp = zigzagEncode(exponent);
      /*--- round mantissa---*/      
      mantissa += (uint)1 << (FLOAT_MANTISSA_BITS-MANTISSA_BITS-1);
      /*--- clamp mantissa---*/            
      mantissa >>= FLOAT_MANTISSA_BITS-MANTISSA_BITS;
      mantissa &= MANTISSA_MASK;
      const uint sign = diff < 0.0f ? ((uint)1<<(RP_BITS-1)) : 0;
      return sign | (exp << MANTISSA_BITS) | mantissa;      
    }

    static __forceinline float decodeFloat(const uint input)
    {
      const uint sign = (input >> (RP_BITS-1)) << 31;
      const uint exp  = ( ((uint)zigzagDecode((input >> MANTISSA_BITS) & EXPONENT_MASK)+FLOAT_EXPONENT_BIAS)<<FLOAT_MANTISSA_BITS);
      const uint mant = ((input & MANTISSA_MASK) << (FLOAT_MANTISSA_BITS-MANTISSA_BITS));
      const uint output = input != 0 ? (sign|exp|mant) : 0;
      return as_float(output);
    }

    __forceinline Vec3f evalBilinearPatch(const float u, const float v)
    {
      return lerp(lerp(v0,v1,u),lerp(v3,v2,u),v);
    }
    
    
    __forceinline uint encode(const Vec3f &p, const uint x, const uint y, const uint gridResX, const uint gridResY)
    {
      const float u = (float)x / (gridResX-1);
      const float v = (float)y / (gridResY-1);      
      const Vec3f bp_p = evalBilinearPatch(u,v);
      const Vec3f diff = p - bp_p;
      const uint rp_x = encodeFloat(diff.x);
      const uint rp_y = encodeFloat(diff.y);
      const uint rp_z = encodeFloat(diff.z);
      return rp_x | (rp_y << (1*RP_BITS)) | (rp_z << (2*RP_BITS));
    }

    __forceinline Vec3f decode(const uint input, const uint x, const uint y, const uint gridResX, const uint gridResY)
    {
      const float u = (float)x / (gridResX-1);
      const float v = (float)y / (gridResY-1);      
      const Vec3f bp_p = evalBilinearPatch(u,v);
      const float px = bp_p.x + decodeFloat((input>>0*RP_BITS) & RP_MASK);
      const float py = bp_p.y + decodeFloat((input>>1*RP_BITS) & RP_MASK);
      const float pz = bp_p.z + decodeFloat((input>>2*RP_BITS) & RP_MASK);
      return Vec3f(px,py,pz);
    }

    __forceinline Vec3f decode(const uint x, const uint y)
    {
      return decode(lc_offsets[y*GRID_RES_VERTEX+x],x,y,GRID_RES_VERTEX,GRID_RES_VERTEX);
    }
    
    __forceinline Vec3fa getVertex(const uint x, const uint y, const Vec3fa *const vtx, const uint grid_resX, const uint grid_resY)
    {
      const uint px = min(x,grid_resX-1);
      const uint py = min(y,grid_resY-1);    
      return vtx[py*grid_resX + px];
    }

    __forceinline void encode(const uint start_x, const uint start_y, const Vec3fa *const vtx, const uint gridResX, const uint gridResY)
    {
      uint index = 0;
      for (int y=0;y<GRID_RES_VERTEX;y++)
        for (int x=0;x<GRID_RES_VERTEX;x++)
        {
          const Vec3f vertex = getVertex(start_x+x,start_y+y,vtx,gridResX,gridResY);
          lc_offsets[index++] = encode(vertex,x,y,GRID_RES_VERTEX,GRID_RES_VERTEX);
        }      
    }
    
    __forceinline LCGBP() {}
    __forceinline LCGBP(const Vec3f v0,const Vec3f v1,const Vec3f v2,const Vec3f v3) : v0(v0),v1(v1),v2(v2),v3(v3) {}
    
  };

  
  
  void convertISPCGridMesh(ISPCGridMesh* grid, RTCScene scene, ISPCOBJMaterial *material)
  {
    Vec3fa *vtx = grid->positions[0];
    PRINT(material);
    PRINT(grid->numVertices);
    PRINT(grid->numGrids);
    PRINT(sizeof(LCGQuadNode));    
    PRINT( NUM_TOTAL_QUAD_NODES_PER_RTC_LCG );
    PRINT(material->map_Kd);
                
    const uint InitialSubGridRes = (1 << (LOD_LEVELS-1)) * (RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1);
    PRINT( InitialSubGridRes );
    
    uint numSubGrids = 0;
    uint numQuadNodes = 0;
    for (uint i=0;i<grid->numGrids;i++)
    {
      PRINT3(i,grid->grids[i].resX,grid->grids[i].resY);      
      const uint grid_resX = grid->grids[i].resX;
      const uint grid_resY = grid->grids[i].resY;
      const uint numInitialSubGrids = ((grid_resX-1) / InitialSubGridRes) * ((grid_resY-1) / InitialSubGridRes);
      PRINT(numInitialSubGrids);
      numSubGrids  += numInitialSubGrids;
      numQuadNodes += numInitialSubGrids * NUM_TOTAL_QUAD_NODES_PER_RTC_LCG;
    }
  
    PRINT(numSubGrids);

    global_grid = (LCG_LODQuadTree_Grid*)alignedUSMMalloc(sizeof(LCG_LODQuadTree_Grid),64);

    global_grid->numQuadTrees = numSubGrids;
    global_grid->numTotalQuadNodes = numQuadNodes;
    global_grid->numMaxResQuadNodes = (1<<(2*(LOD_LEVELS-1))) * numSubGrids;
    PRINT2( (1<<(2*(LOD_LEVELS-1))), global_grid->numMaxResQuadNodes );
    const size_t sizeQuadTrees = sizeof(LCGQuadNode)*global_grid->numTotalQuadNodes;
    global_grid->quadTrees = (LCGQuadNode*)alignedUSMMalloc(sizeQuadTrees,64);
    const size_t sizeLCGPtrs = sizeof(void*)*global_grid->numMaxResQuadNodes;
    global_grid->lcg_ptrs   = (void**)alignedUSMMalloc(sizeLCGPtrs,64,EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE);
    const size_t sizeCrackFixQuadNodes = sizeof(LCGQuadNode)*global_grid->numMaxResQuadNodes;
    global_grid->crackFixQuadNodes = (LCGQuadNode*)alignedUSMMalloc(sizeCrackFixQuadNodes,64,EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE); // FIXME: only borders at highest resolution

    
    PRINT3(sizeQuadTrees,sizeLCGPtrs,sizeCrackFixQuadNodes);
    
    float max_error = 0.0f;
    double avg_error = 0.0;
    uint num_error = 0;
    BBox3f bounds(empty);
    
    uint index = 0;
    for (uint i=0;i<grid->numGrids;i++)
    {
      const uint grid_resX = grid->grids[i].resX;
      const uint grid_resY = grid->grids[i].resY;
    
      for (int start_y=0;start_y+InitialSubGridRes<grid_resY;start_y+=InitialSubGridRes)
        for (int start_x=0;start_x+InitialSubGridRes<grid_resX;start_x+=InitialSubGridRes)
        {
          LCGQuadNode *current = &global_grid->quadTrees[index*NUM_TOTAL_QUAD_NODES_PER_RTC_LCG];
          uint local_index = 1;
          Vec2f u_range((float)start_x/(grid_resX-1),(float)(start_x+InitialSubGridRes)/(grid_resX-1));
          Vec2f v_range((float)start_y/(grid_resY-1),(float)(start_y+InitialSubGridRes)/(grid_resY-1));
          createQuadNode(current[0],current,local_index,start_x,start_y,(1<<(LOD_LEVELS-1)),vtx,grid_resX,grid_resY,FULL_BORDER,index,0,u_range,v_range);          
          if (local_index != NUM_TOTAL_QUAD_NODES_PER_RTC_LCG)
          {
            PRINT2(local_index,NUM_TOTAL_QUAD_NODES_PER_RTC_LCG);
            FATAL("NUM_TOTAL_QUAD_NODES_PER_RTC_LCG");
          }          
          index++;

#if 1
          LCGBP lcgbp(getVertex(start_x,start_y,vtx,grid_resX,grid_resY),
                      getVertex(start_x+LCGBP::GRID_RES_VERTEX-1,start_y,vtx,grid_resX,grid_resY),
                      getVertex(start_x+LCGBP::GRID_RES_VERTEX-1,start_y+LCGBP::GRID_RES_VERTEX-1,vtx,grid_resX,grid_resY),
                      getVertex(start_x,start_y+LCGBP::GRID_RES_VERTEX-1,vtx,grid_resX,grid_resY));
          lcgbp.encode(start_x,start_y,vtx,grid_resX,grid_resY);
                       
          for (int y=0;y<InitialSubGridRes;y++)
            for (int x=0;x<InitialSubGridRes;x++)
            {          
              const Vec3f org_v  = getVertex(start_x+x,start_y+y,vtx,grid_resX,grid_resY);
              bounds.extend(org_v);
              const uint encoded = lcgbp.encode(org_v,x,y,33,33);
              const Vec3f new_v  = lcgbp.decode(encoded,x,y,33,33);

              const Vec3f new_v2  = lcgbp.decode(x,y);
              if (new_v != new_v2)
              {
                PRINT2(new_v,new_v2);
                exit(0);
              }
              const float error = length(new_v-org_v);
              avg_error += (double)error;
              max_error = max(max_error,error);
              num_error++;
            }
          // ===============
#endif                
        }
    }

    PRINT2(bounds,length(bounds.size()));
    
    if (index > numSubGrids)
      FATAL("numSubGrids");    

    PRINT2((float)(avg_error / num_error),max_error);
    
    // PRINT(max_error);
    // exit(0);
    
    global_grid->geometry = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_LOSSY_COMPRESSED_GEOMETRY);
    rtcCommitGeometry(global_grid->geometry);
    global_grid->geomID = rtcAttachGeometry(scene,global_grid->geometry);
    //rtcReleaseGeometry(geom);

    global_grid->map_Kd = (Texture*)material->map_Kd;
    
  }



  __forceinline Vec3fa getTexel3f(const Texture* texture, float s, float t)
  {
    int iu = (int)floor(s * (float)(texture->width));
    int iv = (int)floor(t * (float)(texture->height));    
    const int offset = (iv * texture->width + iu) * 4;
    unsigned char * txt = (unsigned char*)texture->data;
    const unsigned char  r = txt[offset+0];
    const unsigned char  g = txt[offset+1];
    const unsigned char  b = txt[offset+2];
    return Vec3fa(  (float)r * 1.0f/255.0f, (float)g * 1.0f/255.0f, (float)b * 1.0f/255.0f );
  }
  
  extern "C" ISPCScene* g_ispc_scene;

/* called by the C++ code for initialization */
  extern "C" void device_init (char* cfg)
  {
    TutorialData_Constructor(&data);
    /* create scene */
    data.g_scene = g_scene = rtcNewScene(g_device);
    rtcSetSceneBuildQuality(data.g_scene,RTC_BUILD_QUALITY_LOW);
    rtcSetSceneFlags(data.g_scene,RTC_SCENE_FLAG_DYNAMIC);

    PRINT(g_ispc_scene->numGeometries);
    PRINT(g_ispc_scene->numMaterials);
    
    for (unsigned int geomID=0; geomID<g_ispc_scene->numGeometries; geomID++)
    {
      ISPCGeometry* geometry = g_ispc_scene->geometries[geomID];
      if (geometry->type == GRID_MESH)
        convertISPCGridMesh((ISPCGridMesh*)geometry,data.g_scene, (ISPCOBJMaterial*)g_ispc_scene->materials[geomID]);
    }  
  
    /* update scene */
    rtcCommitScene (data.g_scene);  
  }


  Vec3fa randomColor(const int ID)
  {
    int r = ((ID+13)*17*23) & 255;
    int g = ((ID+15)*11*13) & 255;
    int b = ((ID+17)* 7*19) & 255;
    const float oneOver255f = 1.f/255.f;
    return Vec3fa(r*oneOver255f,g*oneOver255f,b*oneOver255f);
  }

/* task that renders a single screen tile */
  Vec3fa renderPixelPrimary(const TutorialData& data, float x, float y, const ISPCCamera& camera, const unsigned int width, const unsigned int height, LCG_LODQuadTree_Grid *grid)
  {
    RTCIntersectArguments args;
    rtcInitIntersectArguments(&args);
    args.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
  
    /* initialize ray */
    Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

    /* intersect ray with scene */
    rtcIntersect1(data.g_scene,RTCRayHit_(ray),&args);

    Vec3f color(1.0f,1.0f,1.0f);    
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
      color = Vec3fa(0.0f);
    else
      color = Vec3fa( abs(dot(ray.dir,normalize(ray.Ng))) );
    return color;
  }

  Vec3fa renderPixelDebug(const TutorialData& data, float x, float y, const ISPCCamera& camera, const unsigned int width, const unsigned int height, LCG_LODQuadTree_Grid *grid, const RenderMode mode)
  {
    RTCIntersectArguments args;
    rtcInitIntersectArguments(&args);
    args.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
  
    /* initialize ray */
    Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

    /* intersect ray with scene */
    rtcIntersect1(data.g_scene,RTCRayHit_(ray),&args);

    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);

    const uint localID = ray.primID & (((uint)1<<RTC_LOSSY_COMPRESSED_GRID_LOCAL_ID_SHIFT)-1);
    const uint primID = ray.primID >> RTC_LOSSY_COMPRESSED_GRID_LOCAL_ID_SHIFT;
    
    Vec3f color(1.0f,1.0f,1.0f);
    
    if (mode == RENDER_DEBUG_QUADS)
    {
      const float LINE_THRESHOLD = 0.1f;
      if (ray.u <= LINE_THRESHOLD ||
          ray.v <= LINE_THRESHOLD ||
          ray.u + ray.v <= LINE_THRESHOLD)
        color = Vec3fa(1.0f,0.0f,0.0f);      
    }
    else if (mode == RENDER_DEBUG_SUBGRIDS)
    {
      const uint gridID = ((LCGQuadNode*)grid->lcg_ptrs[primID])->ID;          
      const uint subgridID = ((LCGQuadNode*)grid->lcg_ptrs[primID])->localID;    
      color = randomColor(gridID*NUM_TOTAL_QUAD_NODES_PER_RTC_LCG+subgridID);   
    }    
    else if (mode == RENDER_DEBUG_GRIDS)
    {
      const uint ID = ((LCGQuadNode*)grid->lcg_ptrs[primID])->ID;    
      color = randomColor(ID);   
    }
    else if (mode == RENDER_DEBUG_LOD)
    {
      const uint level = ((LCGQuadNode*)grid->lcg_ptrs[primID])->lod_level;
      if (level == 0)
        color = Vec3fa(0,0,1);
      else if (level == 1)
        color = Vec3fa(0,1,0);
      else if (level == 2)
        color = Vec3fa(1,0,0);            
    }
    else if (mode == RENDER_DEBUG_CRACK_FIXING)
    {
      const uint cracks_fixed = ((LCGQuadNode*)grid->lcg_ptrs[primID])->flags & CRACK_FIXED_BORDER;
      if (cracks_fixed)
        color = Vec3fa(1,0,1);      
    }
    else if (mode == RENDER_DEBUG_UV)
    {
      const uint flip_uv = localID & 1;
      const uint localQuadID = localID>>1;
      const uint local_y = localQuadID /  RTC_LOSSY_COMPRESSED_GRID_QUAD_RES;
      const uint local_x = localQuadID %  RTC_LOSSY_COMPRESSED_GRID_QUAD_RES;
      
      const LCGQuadNode *current = ((LCGQuadNode*)grid->lcg_ptrs[primID]);      
      const float u = flip_uv ? 1-ray.u : ray.u;
      const float v = flip_uv ? 1-ray.v : ray.v;
      const float u_size = (current->u_range.y - current->u_range.x) * (1.0f / RTC_LOSSY_COMPRESSED_GRID_QUAD_RES);
      const float v_size = (current->v_range.y - current->v_range.x) * (1.0f / RTC_LOSSY_COMPRESSED_GRID_QUAD_RES);
      const float u_start = current->u_range.x + u_size * (float)local_x;
      const float v_start = current->v_range.x + v_size * (float)local_y;      
      const float fu = u_start + u * u_size;
      const float fv = v_start + v * v_size;

      color = getTexel3f(grid->map_Kd,1-fu,fv);
      //color = Vec3fa(fu,fv,1.0f-fu-fv);
    }
    return Vec3fa(abs(dot(ray.dir,normalize(ray.Ng)))) * color;
  }
  

  void renderPixelStandard(const TutorialData& data,
                           int x, int y, 
                           int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera,
                           LCG_LODQuadTree_Grid *grid,
                           const RenderMode mode,
                           const uint spp)
  {
    RandomSampler sampler;

    Vec3fa color(0.0f);
    const float inv_spp = 1.0f / (float)spp;
    
    for (uint i=0;i<spp;i++)
    {
      RandomSampler_init(sampler, x, y, i);      
      float fx = x + RandomSampler_get1D(sampler);
      float fy = y + RandomSampler_get1D(sampler);
    
      /* calculate pixel color */
      if (mode == RENDER_PRIMARY)
        color += renderPixelPrimary(data, (float)fx,(float)fy,camera, width, height, grid);
      else
        color += renderPixelDebug(data, (float)fx,(float)fy,camera, width, height, grid,mode);
    }
    color *= inv_spp;
    
    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }

  extern "C" void renderFrameStandard (int* pixels,
                                       const unsigned int width,
                                       const unsigned int height,
                                       const float time,
                                       const ISPCCamera& camera)
  {
    /* render all pixels */
#if defined(EMBREE_SYCL_TUTORIAL)
    RenderMode rendering_mode = user_rendering_mode;    
    LCG_LODQuadTree_Grid *grid = global_grid;
    uint spp = user_spp;
    TutorialData ldata = data;
    sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh){
                                                   const sycl::nd_range<2> nd_range = make_nd_range(height,width);
                                                   cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) {
                                                                               const unsigned int x = item.get_global_id(1); if (x >= width ) return;
                                                                               const unsigned int y = item.get_global_id(0); if (y >= height) return;
                                                                               renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,grid,rendering_mode,spp);
                                                                             });
                                                 });
    global_gpu_queue->wait_and_throw();

    const auto t0 = event.template get_profiling_info<sycl::info::event_profiling::command_start>();
    const auto t1 = event.template get_profiling_info<sycl::info::event_profiling::command_end>();
    const double dt = (t1-t0)*1E-9;
    ((ISPCCamera*)&camera)->render_time = dt;  
#endif
  }

  __forceinline size_t alignTo(const uint size, const uint alignment)
  {
    return ((size+alignment-1)/alignment)*alignment;
  }

  __forceinline void waitOnQueueAndCatchException(sycl::queue &gpu_queue)
  {
    try {
      gpu_queue.wait_and_throw();
    } catch (sycl::exception const& e) {
      std::cout << "Caught synchronous SYCL exception:\n"
                << e.what() << std::endl;
      FATAL("SYCL Exception");     
    }      
  }

  __forceinline void waitOnEventAndCatchException(sycl::event &event)
  {
    try {
      event.wait_and_throw();
    } catch (sycl::exception const& e) {
      std::cout << "Caught synchronous SYCL exception:\n"
                << e.what() << std::endl;
      FATAL("SYCL Exception");     
    }      
  }

  __forceinline float getDeviceExecutionTiming(sycl::event &queue_event)
  {
    const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
    const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
    return (float)((t1-t0)*1E-6);      
  }

  template<typename T>
  static __forceinline uint atomic_add_global(T *dest, const T count=1)
  {
    sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> counter(*dest);        
    return counter.fetch_add(count);      
  }


  extern "C" void device_gui()
  {
    const uint trisPerSubGrid = RTC_LOSSY_COMPRESSED_GRID_QUAD_RES*RTC_LOSSY_COMPRESSED_GRID_QUAD_RES*2;
    ImGui::Text("SPP: %d",user_spp);    
    ImGui::Text("BVH Build Time: %4.4f ms",avg_bvh_build_time.get());
    ImGui::Text("numQuadTrees: %d ",global_grid->numQuadTrees);
    ImGui::Text("numTotalQuadNodes: %d ",global_grid->numTotalQuadNodes);
    ImGui::Text("numMaxResQuadNodes: %d => %d Triangles",global_grid->numMaxResQuadNodes,global_grid->numMaxResQuadNodes*trisPerSubGrid);    
    ImGui::Text("numLCGPtrs: %d => %d Triangles",global_grid->numLCGPtrs,global_grid->numLCGPtrs*trisPerSubGrid);
    ImGui::Text("numCrackFixQuadNodes: %d => %d Triangles",global_grid->numCrackFixQuadNodes,global_grid->numCrackFixQuadNodes*trisPerSubGrid);    
  }
  
  
/* called by the C++ code to render */
  extern "C" void device_render (int* pixels,
                                 const unsigned int width,
                                 const unsigned int height,
                                 const float time,
                                 const ISPCCamera& camera)
  {
#if defined(EMBREE_SYCL_TUTORIAL)

    
    LCG_LODQuadTree_Grid *grid = global_grid;

    // PRINT(grid);
    // PRINT(grid->quadTrees);
    // PRINT(grid->lcg_ptrs);
    // PRINT(grid->crackFixQuadNodes);
    
    const uint numQuadTrees = grid->numQuadTrees;
    
    sycl::event init_event =  global_gpu_queue->submit([&](sycl::handler &cgh) {
                                                         cgh.single_task([=]() {
                                                                           grid->numCrackFixQuadNodes = 0;
                                                                           grid->numLCGPtrs = 0;
                                                                         });
                                                       });
    
    const uint wgSize = 64;
    const sycl::nd_range<1> nd_range1(alignTo(numQuadTrees,wgSize),sycl::range<1>(wgSize));          
    
    sycl::event compute_lod_event = global_gpu_queue->submit([=](sycl::handler& cgh){
                                                               cgh.depends_on(init_event);                                                   
                                                               cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) {
                                                                                           const uint i = item.get_global_id(0);
                                                                                           if (i < numQuadTrees)
                                                                                           {
                                                                                             LCGQuadNode *const root = &grid->quadTrees[i*NUM_TOTAL_QUAD_NODES_PER_RTC_LCG];
                                                                                             const LODEdgeLevel lod_edge_levels = getLODEdgeLevels(*root,camera,width,height);
                                                                                             //PRINT5(i,lod_edge_levels.top,lod_edge_levels.right,lod_edge_levels.bottom,lod_edge_levels.left);
                                                                                             const uint lod_level = lod_edge_levels.level();
                                                                                             const uint crackFixingBorderMask = getCrackFixingBorderMask(*root,lod_edge_levels,lod_level);
                                                                                             const uint numSubGrids = 1<<(2*lod_level);
                                                                                             const uint offset = ((1<<(2*lod_level))-1)/(4-1);
                                                                                             const uint numActiveQuads = atomic_add_global(&grid->numLCGPtrs,numSubGrids);
                                                                                             
                                                                                             if (crackFixingBorderMask)
                                                                                             {
                                                                                               for (uint j=0;j<numSubGrids;j++)
                                                                                               {
                                                                                                 LCGQuadNode *subgrid_root = &root[offset+j];
                                                                                                 const uint subgrid_crackFixingBorderMask = subgrid_root->flags & crackFixingBorderMask;
                                                                                                 if (subgrid_crackFixingBorderMask)
                                                                                                 {
                                                                                                   const uint numCrackFixQuadNodes = atomic_add_global(&grid->numCrackFixQuadNodes,(uint)1);
                                                                                                   subgrid_root = &grid->crackFixQuadNodes[numCrackFixQuadNodes];
                                                                                                   *subgrid_root = root[offset+j];
                                                                                                   fixCracks(*subgrid_root, subgrid_crackFixingBorderMask, lod_edge_levels, lod_level);
                                                                                                 }
                                                                                                 grid->lcg_ptrs[numActiveQuads+j] = &subgrid_root->grid;
                                                                                               }          
                                                                                             }
                                                                                             else        
                                                                                               for (uint j=0;j<numSubGrids;j++)
                                                                                                 grid->lcg_ptrs[numActiveQuads+j] = &root[offset+j].grid;
                                                                                           }
                                                                                           
                                                                                         });
                                                             });
    waitOnEventAndCatchException(compute_lod_event);

    //PRINT(getDeviceExecutionTiming(compute_lod_event));
    //waitOnQueueAndCatchException(*global_gpu_queue);
    //PRINT2(grid->numLCGPtrs,grid->numCrackFixQuadNodes);
    double t0 = getSeconds();

    
    rtcSetGeometryUserData(grid->geometry,grid->lcg_ptrs);
    rtcSetLossyCompressedGeometryPrimitiveCount(grid->geometry,grid->numLCGPtrs);
    rtcCommitGeometry(grid->geometry);
    
    /* commit changes to scene */
    rtcCommitScene (data.g_scene);

    double dt0 = (getSeconds()-t0)*1000.0;
    avg_bvh_build_time.add(dt0);
#endif
  }

/* called by the C++ code for cleanup */
  extern "C" void device_cleanup ()
  {
    TutorialData_Destructor(&data);
  }

} // namespace embree
