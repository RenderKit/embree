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

namespace embree
{
   /*! 16-wide MIC bool type. */
  class bool16
  {
  public:
    __mmask v;
    
    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline bool16 () {};
    __forceinline bool16 (const bool16 &t) { v = t.v; };
    __forceinline bool16& operator=(const bool16 &f) { v = f.v; return *this; };

    __forceinline bool16 (const __mmask &t) { v = t; };
    __forceinline operator __mmask () const { return v; };
    
    __forceinline bool16(bool b) { v = b ? 0xFFFF : 0x0000; };
    __forceinline bool16(int t ) { v = (__mmask)t; };
    __forceinline bool16(unsigned int t ) { v = (__mmask)t; };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline bool16( FalseTy ) : v(0x0000) {}
    __forceinline bool16( TrueTy  ) : v(0xffff) {}

    static unsigned int shift1[32];
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////
  
   __forceinline bool16 operator!(const bool16 &a) { return _mm512_knot(a); }
  
   ////////////////////////////////////////////////////////////////////////////////
   /// Binary Operators
   ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline bool16 operator&(const bool16 &a, const bool16 &b) { return _mm512_kand(a,b); };
  __forceinline bool16 operator|(const bool16 &a, const bool16 &b) { return _mm512_kor(a,b); };
  __forceinline bool16 operator^(const bool16 &a, const bool16 &b) { return _mm512_kxor(a,b); };

    __forceinline bool16 andn(const bool16 &a, const bool16 &b) { return _mm512_kandn(b,a); }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline const bool16 operator &=( bool16& a, const bool16& b ) { return a = a & b; }
  __forceinline const bool16 operator |=( bool16& a, const bool16& b ) { return a = a | b; }
  __forceinline const bool16 operator ^=( bool16& a, const bool16& b ) { return a = a ^ b; }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline const bool16 operator !=( const bool16& a, const bool16& b ) { return _mm512_kxor(a, b); }
  __forceinline const bool16 operator ==( const bool16& a, const bool16& b ) { return _mm512_kxnor(a, b); }
  
  __forceinline bool16 select (const bool16 &s, const bool16 &a, const bool16 &b) {
    return _mm512_kor(_mm512_kand(s,a),_mm512_kandn(s,b));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reduction Operations
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline int all(const bool16 &a)  { return  _mm512_kortestc(a,a) != 0; }
  __forceinline int any(const bool16 &a)  { return  _mm512_kortestz(a,a) == 0; }
  __forceinline int none(const bool16 &a) { return  _mm512_kortestz(a,a) != 0; }

  __forceinline int all       ( const bool16& valid, const bool16& b ) { return all(!valid | b); }
  __forceinline int any       ( const bool16& valid, const bool16& b ) { return any( valid & b); }
  __forceinline int none      ( const bool16& valid, const bool16& b ) { return none(valid & b); }
  
  __forceinline size_t movemask( const bool16& a ) { return _mm512_kmov(a); }
  __forceinline size_t popcnt  ( const bool16& a ) { return _mm_countbits_32(a.v); }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Convertion Operations
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline unsigned int toInt (const bool16 &a) { return _mm512_mask2int(a); };
  __forceinline bool16        toMask(const int &a)   { return _mm512_int2mask(a); };

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  inline std::ostream& operator<<(std::ostream& cout, const bool16& a) 
  {
    cout << "<";
    for (size_t i=0; i<16; i++) {
      if ((a.v >> i) & 1) cout << "1"; else cout << "0";
    }
    return cout << ">";
  }
}
