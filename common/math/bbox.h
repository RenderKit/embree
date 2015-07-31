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

#pragma once

#include "vec2.h"
#include "vec3.h"

namespace embree
{
  template<typename T>
  struct BBox
  {
    T lower, upper;

    ////////////////////////////////////////////////////////////////////////////////
    /// Construction
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline BBox           ( )                   { }
    __forceinline BBox           ( const BBox& other ) { lower = other.lower; upper = other.upper; }
    __forceinline BBox& operator=( const BBox& other ) { lower = other.lower; upper = other.upper; return *this; }

    __forceinline BBox ( const T& v                     ) : lower(v), upper(v) {}
    __forceinline BBox ( const T& lower, const T& upper ) : lower(lower), upper(upper) {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Extending Bounds
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline const BBox& extend(const BBox& other) { lower = min(lower,other.lower); upper = max(upper,other.upper); return *this; }
    __forceinline const BBox& extend(const T   & other) { lower = min(lower,other      ); upper = max(upper,other      ); return *this; }

    __forceinline void extend_atomic(const BBox& other) { 
      atomic_min_f32(&lower.x,other.lower.x);
      atomic_min_f32(&lower.y,other.lower.y);
      atomic_min_f32(&lower.z,other.lower.z);
      atomic_max_f32(&upper.x,other.upper.x);
      atomic_max_f32(&upper.y,other.upper.y);
      atomic_max_f32(&upper.z,other.upper.z);
    }

    /*! tests if box is empty */
    __forceinline bool empty() const { for (int i=0; i<T::N; i++) if (lower[i] > upper[i]) return true; return false; }

    /*! computes the size of the box */
    __forceinline T size() const { return upper - lower; }

    /*! computes the center of the box */
    __forceinline T center() const { return 0.5f*(lower+upper); }

    /*! computes twice the center of the box */
    __forceinline T center2() const { return lower+upper; }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline BBox( EmptyTy ) : lower(pos_inf), upper(neg_inf) {}
    __forceinline BBox( FullTy  ) : lower(neg_inf), upper(pos_inf) {}
    __forceinline BBox( FalseTy ) : lower(pos_inf), upper(neg_inf) {}
    __forceinline BBox( TrueTy  ) : lower(neg_inf), upper(pos_inf) {}
    __forceinline BBox( NegInfTy ): lower(pos_inf), upper(neg_inf) {}
    __forceinline BBox( PosInfTy ): lower(neg_inf), upper(pos_inf) {}
  };

#if defined(__SSE__)
  template<> __forceinline bool BBox<Vec3fa>::empty() const {
    return !all(le_mask(lower,upper));
  }
#endif

  /*! tests if box is finite */
  __forceinline bool isvalid( const BBox<Vec3fa>& v ) {
    return all(gt_mask(v.lower,Vec3fa_t(-FLT_LARGE)) & lt_mask(v.upper,Vec3fa_t(+FLT_LARGE)));
  }

  /*! tests if box has finite entries */
  __forceinline bool is_finite( const BBox<Vec3fa>& b) {
    return is_finite(b.lower) && is_finite(b.upper);
  }

  /*! test if point contained in box */
  __forceinline bool inside ( const BBox<Vec3fa>& b, const Vec3fa& p ) { return all(ge_mask(p,b.lower) & le_mask(p,b.upper)); }

  /*! computes the center of the box */
  template<typename T> __forceinline const T center (const BBox<T>& box) { return T(.5f)*(box.lower + box.upper); }
  template<typename T> __forceinline const T center2(const BBox<T>& box) { return box.lower + box.upper; }

  /*! computes the volume of a bounding box */
  __forceinline float volume    ( const BBox<Vec3fa>& b ) { return reduce_mul(b.size()); }
  __forceinline float safeVolume( const BBox<Vec3fa>& b ) { if (b.empty()) return 0.0f; else return volume(b); }

  /*! computes the volume of a bounding box */
  __forceinline float volume( const BBox<Vec3f>& b )  { return reduce_mul(b.size()); }

  /*! computes the surface area of a bounding box */
  template<typename T> __forceinline const T area( const BBox<Vec2<T> >& b ) { const Vec2<T> d = b.size(); return d.x*d.y; }

  template<typename T> __forceinline const T halfArea( const BBox<Vec3<T> >& b ) { return halfArea(b.size()); }
  template<typename T> __forceinline const T     area( const BBox<Vec3<T> >& b ) { return 2.0f*halfArea(b); }

  __forceinline float halfArea( const BBox<Vec3fa>& b ) { return halfArea(b.size()); }
  __forceinline float     area( const BBox<Vec3fa>& b ) { return 2.0f*halfArea(b); }

  template<typename Vec> __forceinline float safeArea( const BBox<Vec>& b ) { if (b.empty()) return 0.0f; else return area(b); }

  /*! merges bounding boxes and points */
  template<typename T> __forceinline const BBox<T> merge( const BBox<T>& a, const       T& b ) { return BBox<T>(min(a.lower, b    ), max(a.upper, b    )); }
  template<typename T> __forceinline const BBox<T> merge( const       T& a, const BBox<T>& b ) { return BBox<T>(min(a    , b.lower), max(a    , b.upper)); }
  template<typename T> __forceinline const BBox<T> merge( const BBox<T>& a, const BBox<T>& b ) { return BBox<T>(min(a.lower, b.lower), max(a.upper, b.upper)); }

  /*! Merges three boxes. */
  template<typename T> __forceinline const BBox<T> merge( const BBox<T>& a, const BBox<T>& b, const BBox<T>& c ) { return merge(a,merge(b,c)); }

  /*! Merges four boxes. */
  template<typename T> __forceinline BBox<T> merge(const BBox<T>& a, const BBox<T>& b, const BBox<T>& c, const BBox<T>& d) {
    return merge(merge(a,b),merge(c,d));
  }

  /*! Comparison Operators */
  template<typename T> __forceinline bool operator==( const BBox<T>& a, const BBox<T>& b ) { return a.lower == b.lower && a.upper == b.upper; }
  template<typename T> __forceinline bool operator!=( const BBox<T>& a, const BBox<T>& b ) { return a.lower != b.lower || a.upper != b.upper; }

  /*! scaling */
  template<typename T> __forceinline BBox<T> operator *( const float& a, const BBox<T>& b ) { return BBox<T>(a*b.lower,a*b.upper); }
  template<typename T> __forceinline BBox<T> operator *( const     T& a, const BBox<T>& b ) { return BBox<T>(a*b.lower,a*b.upper); }

  /*! translations */
  template<typename T> __forceinline BBox<T> operator +( const BBox<T>& a, const BBox<T>& b ) { return BBox<T>(a.lower+b.lower,a.upper+b.upper); }
  template<typename T> __forceinline BBox<T> operator -( const BBox<T>& a, const BBox<T>& b ) { return BBox<T>(a.lower-b.lower,a.upper-b.upper); }
  template<typename T> __forceinline BBox<T> operator +( const BBox<T>& a, const      T & b ) { return BBox<T>(a.lower+b      ,a.upper+b      ); }
  template<typename T> __forceinline BBox<T> operator -( const BBox<T>& a, const      T & b ) { return BBox<T>(a.lower-b      ,a.upper-b      ); }

  /*! extension */
  template<typename T> __forceinline BBox<T> enlarge(const BBox<T>& a, const T& b) { return BBox<T>(a.lower-b, a.upper+b); }

  /*! intersect bounding boxes */
  template<typename T> __forceinline const BBox<T> intersect( const BBox<T>& a, const BBox<T>& b ) { return BBox<T>(max(a.lower, b.lower), min(a.upper, b.upper)); }
  template<typename T> __forceinline const BBox<T> intersect( const BBox<T>& a, const BBox<T>& b, const BBox<T>& c ) { return intersect(a,intersect(b,c)); }

  /*! tests if bounding boxes (and points) are disjoint (empty intersection) */
  template<typename T> __inline bool disjoint( const BBox<T>& a, const BBox<T>& b ) { return intersect(a,b).empty(); }
  template<typename T> __inline bool disjoint( const BBox<T>& a, const       T& b ) { return disjoint(a,BBox<T>(b)); }
  template<typename T> __inline bool disjoint( const       T& a, const BBox<T>& b ) { return disjoint(BBox<T>(a),b); }

  /*! tests if bounding boxes (and points) are conjoint (non-empty intersection) */
  template<typename T> __inline bool conjoint( const BBox<T>& a, const BBox<T>& b ) { return !intersect(a,b).empty(); }
  template<typename T> __inline bool conjoint( const BBox<T>& a, const       T& b ) { return conjoint(a,BBox<T>(b)); }
  template<typename T> __inline bool conjoint( const       T& a, const BBox<T>& b ) { return conjoint(BBox<T>(a),b); }

  /*! subset relation */
  template<typename T> __inline bool subset( const BBox<T>& a, const BBox<T>& b )
  { 
    for ( size_t i = 0; i < T::N; i++ ) if ( a.lower[i] < b.lower[i] ) return false;
    for ( size_t i = 0; i < T::N; i++ ) if ( a.upper[i] > b.upper[i] ) return false;
    return true; 
  }

  /*! output operator */
  template<typename T> __forceinline std::ostream& operator<<(std::ostream& cout, const BBox<T>& box) {
    return cout << "[" << box.lower << "; " << box.upper << "]";
  }

  /*! default template instantiations */
  typedef BBox<Vec2f> BBox2f;
  typedef BBox<Vec3f> BBox3f;
  typedef BBox<Vec3fa> BBox3fa;
}
