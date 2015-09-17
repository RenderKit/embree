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

#include <cstddef>
#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <cstring>
#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
/// detect platform
////////////////////////////////////////////////////////////////////////////////

/* detect 32 or 64 platform */
#if defined(__x86_64__) || defined(__ia64__) || defined(_M_X64)
#define __X86_64__
#endif

/* detect Linux platform */
#if defined(linux) || defined(__linux__) || defined(__LINUX__)
#  if !defined(__LINUX__)
#     define __LINUX__
#  endif
#  if !defined(__UNIX__)
#     define __UNIX__
#  endif
#endif

/* detect FreeBSD platform */
#if defined(__FreeBSD__) || defined(__FREEBSD__)
#  if !defined(__FREEBSD__)
#     define __FREEBSD__
#  endif
#  if !defined(__UNIX__)
#     define __UNIX__
#  endif
#endif

/* detect Windows 95/98/NT/2000/XP/Vista/7 platform */
#if (defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)) && !defined(__CYGWIN__)
#  if !defined(__WIN32__)
#     define __WIN32__
#  endif
#endif

/* detect Cygwin platform */
#if defined(__CYGWIN__)
#  if !defined(__UNIX__)
#     define __UNIX__
#  endif
#endif

/* detect MAC OS X platform */
#if defined(__APPLE__) || defined(MACOSX) || defined(__MACOSX__)
#  if !defined(__MACOSX__)
#     define __MACOSX__
#  endif
#  if !defined(__UNIX__)
#     define __UNIX__
#  endif
#endif

/* try to detect other Unix systems */
#if defined(__unix__) || defined (unix) || defined(__unix) || defined(_unix)
#  if !defined(__UNIX__)
#     define __UNIX__
#  endif
#endif

#if defined (_DEBUG)
#define DEBUG
#endif

////////////////////////////////////////////////////////////////////////////////
/// ISA configuration
////////////////////////////////////////////////////////////////////////////////

#if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
  #define __SSE__
  #define __SSE2__
#endif

#if defined(__WIN32__) 
#if defined(CONFIG_SSE41)
  #if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
    #define __SSE3__
    #define __SSSE3__
    #define __SSE4_1__
  #endif
#endif
#if defined(CONFIG_SSE42)
  #if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
    #define __SSE3__
    #define __SSSE3__
    #define __SSE4_1__
    #define __SSE4_2__
  #endif
#endif
#if defined(CONFIG_AVX)
  #if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
    #define __SSE3__
    #define __SSSE3__
    #define __SSE4_1__
    #define __SSE4_2__
    #if !defined(__AVX__)
      #define __AVX__
    #endif
  #endif
#endif
#if defined(CONFIG_AVX2)
  #if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
    #define __SSE3__
    #define __SSSE3__
    #define __SSE4_1__
    #define __SSE4_2__
    #if !defined(__AVX__)
      #define __AVX__
    #endif
    #if !defined(__AVX2__)
      #define __AVX2__
    #endif
  #endif
#endif
#if defined(CONFIG_AVX512)
  #if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
    #define __SSE3__
    #define __SSSE3__
    #define __SSE4_1__
    #define __SSE4_2__
    #if !defined(__AVX__)
      #define __AVX__
    #endif
    #if !defined(__AVX2__)
      #define __AVX2__
    #endif
    #if !defined(__AVX512F__)
      #define __AVX512F__
    #endif
  #endif
#endif

#endif

////////////////////////////////////////////////////////////////////////////////
/// Makros
////////////////////////////////////////////////////////////////////////////////

#ifdef __WIN32__
#define __dllexport __declspec(dllexport)
#define __dllimport __declspec(dllimport)
#else
#define __dllexport __attribute__ ((visibility ("default")))
#define __dllimport 
#endif

#ifdef __WIN32__
#undef __noinline
#define __noinline             __declspec(noinline)
//#define __forceinline          __forceinline
//#define __restrict             __restrict
#define __thread               __declspec(thread)
#define __aligned(...)           __declspec(align(__VA_ARGS__))
//#define __FUNCTION__           __FUNCTION__
#define debugbreak()           __debugbreak()

#else
#undef __noinline
#undef __forceinline
#define __noinline             __attribute__((noinline))
#define __forceinline          inline __attribute__((always_inline))
//#define __restrict             __restrict
//#define __thread               __thread
#define __aligned(...)           __attribute__((aligned(__VA_ARGS__)))
#define __FUNCTION__           __PRETTY_FUNCTION__
#define debugbreak()           asm ("int $3")
#endif

#ifdef __GNUC__
  #define MAYBE_UNUSED __attribute__((used))
#else
  #define MAYBE_UNUSED
#endif

#if defined(_MSC_VER)
#define __restrict__ // FIXME: should be supported on ICC?
#endif

#if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
#define   likely(expr) (expr)
#define unlikely(expr) (expr)
#else
#define   likely(expr) __builtin_expect((bool)(expr),true )
#define unlikely(expr) __builtin_expect((bool)(expr),false)
#endif

////////////////////////////////////////////////////////////////////////////////
/// Error handling and debugging
////////////////////////////////////////////////////////////////////////////////

/* debug printing macros */
#define STRING(x) #x
#define TOSTRING(x) STRING(x)
#define PING std::cout << __FILE__ << " (" << __LINE__ << "): " << __FUNCTION__ << std::endl
#define PRINT(x) std::cout << STRING(x) << " = " << (x) << std::endl
#define PRINT2(x,y) std::cout << STRING(x) << " = " << (x) << ", " << STRING(y) << " = " << (y) << std::endl
#define PRINT3(x,y,z) std::cout << STRING(x) << " = " << (x) << ", " << STRING(y) << " = " << (y) << ", " << STRING(z) << " = " << (z) << std::endl
#define PRINT4(x,y,z,w) std::cout << STRING(x) << " = " << (x) << ", " << STRING(y) << " = " << (y) << ", " << STRING(z) << " = " << (z) << ", " << STRING(w) << " = " << (w) << std::endl

#define THROW_RUNTIME_ERROR(str) \
  throw std::runtime_error(std::string(__FILE__) + " (" + std::to_string((long long)__LINE__) + "): " + std::string(str));

#if defined(__MIC__)
#define FATAL(x) { std::cout << "FATAL error in " << __FUNCTION__ << " : " << x << std::endl << std::flush; exit(1); }
#else
#define FATAL(x) THROW_RUNTIME_ERROR(x)
#endif

#define NOT_IMPLEMENTED FATAL("Not implemented")

////////////////////////////////////////////////////////////////////////////////
/// Basic Types
////////////////////////////////////////////////////////////////////////////////

/* windows does not have ssize_t */
#if defined(__WIN32__)
#if defined(__X86_64__)
typedef int64_t ssize_t;
#else
typedef int32_t ssize_t;
#endif
#endif

////////////////////////////////////////////////////////////////////////////////
/// Disable some compiler warnings
////////////////////////////////////////////////////////////////////////////////

#if defined(__INTEL_COMPILER)
#pragma warning(disable:265 ) // floating-point operation result is out of range
#pragma warning(disable:383 ) // value copied to temporary, reference to temporary used
#pragma warning(disable:869 ) // parameter was never referenced
#pragma warning(disable:981 ) // operands are evaluated in unspecified order
#pragma warning(disable:1418) // external function definition with no prior declaration
#pragma warning(disable:1419) // external declaration in primary source file
#pragma warning(disable:1572) // floating-point equality and inequality comparisons are unreliable
#pragma warning(disable:94  ) // the size of an array must be greater than zero
#pragma warning(disable:1599) // declaration hides parameter
#pragma warning(disable:424 ) // extra ";" ignored
#pragma warning(disable:2196) // routine is both "inline" and "noinline"
#pragma warning(disable:177 ) // label was declared but never referenced
#pragma warning(disable:114 ) // function was referenced but not defined
#endif

#if defined(_MSC_VER)
#pragma warning(disable:4200) // nonstandard extension used : zero-sized array in struct/union
#pragma warning(disable:4800) // forcing value to bool 'true' or 'false' (performance warning)
#pragma warning(disable:4267) // '=' : conversion from 'size_t' to 'unsigned long', possible loss of data
#pragma warning(disable:4244) // 'argument' : conversion from 'ssize_t' to 'unsigned int', possible loss of data
#pragma warning(disable:4355) // 'this' : used in base member initializer list
#pragma warning(disable:4996) // 'std::copy': Function call with parameters that may be unsafe 
#pragma warning(disable:391 ) // '<=' : signed / unsigned mismatch
#pragma warning(disable:4018) // '<' : signed / unsigned mismatch
#pragma warning(disable:4305) // 'initializing' : truncation from 'double' to 'float'
#pragma warning(disable:4068) // unknown pragma
#pragma warning(disable:4146) // : unary minus operator applied to unsigned type, result still unsigned
#endif

////////////////////////////////////////////////////////////////////////////////
/// Some macros for static profiling
////////////////////////////////////////////////////////////////////////////////

#if defined (__GNUC__) 
#define IACA_SSC_MARK( MARK_ID )						\
__asm__ __volatile__ (									\
					  "\n\t  movl $"#MARK_ID", %%ebx"	\
					  "\n\t  .byte 0x64, 0x67, 0x90"	\
					  : : : "memory" );

#define IACA_UD_BYTES __asm__ __volatile__ ("\n\t .byte 0x0F, 0x0B");

#else
#define IACA_UD_BYTES {__asm _emit 0x0F \
	__asm _emit 0x0B}

#define IACA_SSC_MARK(x) {__asm  mov ebx, x\
	__asm  _emit 0x64 \
	__asm  _emit 0x67 \
	__asm  _emit 0x90 }

#define IACA_VC64_START __writegsbyte(111, 111);
#define IACA_VC64_END   __writegsbyte(222, 222);

#endif

#define IACA_START {IACA_UD_BYTES \
					IACA_SSC_MARK(111)}
#define IACA_END {IACA_SSC_MARK(222) \
					IACA_UD_BYTES}
