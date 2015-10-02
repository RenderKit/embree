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

#include "heuristic_binning.h"
#include "../../algorithms/IndexSort.h"
#include "../../common/scene.h"
#include "presplit.h"
#include <immintrin.h>

#ifdef _WIN32
#define ALIGNED(n) __declspec(align(n))
#else
#define ALIGNED(n) __attribute__((aligned(n)))
#endif

#define _MM256_TRANSPOSE4_PS(row0, row1, row2, row3) \
do {\
	__m256 tmp3, tmp2, tmp1, tmp0;\
	tmp0 = _mm256_unpacklo_ps((row0), (row1));\
	tmp2 = _mm256_unpacklo_ps((row2), (row3));\
	tmp1 = _mm256_unpackhi_ps((row0), (row1));\
	tmp3 = _mm256_unpackhi_ps((row2), (row3));\
	(row0) = _mm256_movelh_ps(tmp0, tmp2);\
	(row1) = _mm256_movehl_ps(tmp2, tmp0);\
	(row2) = _mm256_movelh_ps(tmp1, tmp3);\
	(row3) = _mm256_movehl_ps(tmp3, tmp1);\
} while (0)

inline __m256 combine(__m128 a, __m128 b) {
	return _mm256_insertf128_ps(_mm256_castps128_ps256(a), b, 1);
}

inline __m256 _mm256_movelh_ps(__m256 a, __m256 b) {
	return _mm256_shuffle_ps(a, b, _MM_SHUFFLE(1, 0, 1, 0));
}

inline __m256 _mm256_movehl_ps(__m256 a, __m256 b) {
	return _mm256_shuffle_ps(b, a, _MM_SHUFFLE(3, 2, 3, 2));
}

inline float surfaceArea(__m256 bounds) {
	__m128 d = _mm_add_ps(_mm256_castps256_ps128(bounds), _mm256_extractf128_ps(bounds, 1));
	d = _mm_mul_ps(_mm_shuffle_ps(d, d, _MM_SHUFFLE(3,1,0,0)), _mm_shuffle_ps(d, d, _MM_SHUFFLE(3,2,2,1)));
	return _mm_cvtss_f32(_mm_add_ss(_mm_add_ss(d, _mm_shuffle_ps(d, d, _MM_SHUFFLE(2,2,2,2))), _mm_shuffle_ps(d, d, _MM_SHUFFLE(1,1,1,1))));
}

namespace embree
{
  namespace isa
  {


	  struct FastSweepData {
		  unsigned* __restrict indices[3];

		  float* __restrict midX;
		  float* __restrict midY;
		  float* __restrict midZ;

		  float* __restrict accumulatedArea;
		  unsigned* __restrict temporaryIndices;
		  unsigned char* __restrict leftPartition;
		  PrimRef* __restrict gatheredRefs;
		  unsigned offset;
			bool isAllocated;

			FastSweepData() {
				isAllocated = false;
			}

		  FastSweepData(unsigned size, size_t maxLeafSize) {
				fastSweepDataAllocate(size, maxLeafSize);
		  }

		  void setOffset(unsigned offset) {
			  this->offset = offset;
		  }

			void fastSweepDataAllocate(unsigned size, size_t maxLeafSize) {
				indices[0] = static_cast<unsigned*>(_mm_malloc(4 * size * sizeof(unsigned) + 256, 64));
			  indices[1] = static_cast<unsigned*>(_mm_malloc(4 * size * sizeof(unsigned) + 256, 64));
			  indices[2] = static_cast<unsigned*>(_mm_malloc(4 * size * sizeof(unsigned) + 256, 64));

			  midX = static_cast<float*>(_mm_malloc(size*sizeof(float) + 256, 64));
			  midY = static_cast<float*>(_mm_malloc(size*sizeof(float) + 256, 64));
			  midZ = static_cast<float*>(_mm_malloc(size*sizeof(float) + 256, 64));

			  accumulatedArea = static_cast<float*>(_mm_malloc(size * sizeof(float), 64)); //new float[size];
			  temporaryIndices = new unsigned[size];
			  leftPartition = new unsigned char[size];
			  gatheredRefs = static_cast<PrimRef*>(_mm_malloc(maxLeafSize * 2 * sizeof(PrimRef), 64));
				isAllocated = true;
			}

		  ~FastSweepData() {
			  _mm_free(indices[0]);
			  _mm_free(indices[1]);
			  _mm_free(indices[2]);

			  _mm_free(midX);
			  _mm_free(midY);
			  _mm_free(midZ);

			  _mm_free(accumulatedArea);
				_mm_free(gatheredRefs);
			  delete [] temporaryIndices;
			  delete [] leftPartition;
		  }
	  };

	  /*! stores all information to perform some split */
	  struct BonsaiSplit
	  {
		  /*! construct an invalid split by default */
		  __forceinline BonsaiSplit()
		  : dim(-1), pos(0.f), minimalLeafFound(false) {}

		  /*! tests if this split is valid */
		  __forceinline bool valid() const { return dim != -1; }

		  /*! sets leafFound to true */
		  __forceinline void leafFound() { minimalLeafFound = true; }

		  /*! */
		  __forceinline bool isLeaf() const { return minimalLeafFound; }

		  /*! stream output */
		  friend std::ostream& operator<<(std::ostream& cout, const BonsaiSplit& split) {
			  return cout << "BinSplit { dim = " << split.dim << ", pos = " << split.pos << "}";
		  }

	  public:
		  int dim;                  //!< split dimension
		  float pos;                  //!< split plane at dim dimension
		  int index;				//!< split index
		  bool minimalLeafFound;	//!< true if a local SAH minimum has been found and size is less than maxLeafSize
	  };

    /*! Performs standard object binning */
    template<typename PrimRef>
      struct HeuristicBonsai
      {
		  typedef BonsaiSplit Split;
        typedef range<size_t> Set;

        static const size_t PARALLEL_THRESHOLD = 10000;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 4096;
        static const size_t PARALLEL_PARITION_BLOCK_SIZE = 64;

				volatile int newIndex = 0;

		FastSweepData** __restrict ThreadLocal;

        __forceinline HeuristicBonsai ()
          : prims(nullptr) {}

        /*! remember prim array */
        __forceinline HeuristicBonsai (PrimRef* prims, Scene* scene, int splitIndex)
          : prims(prims), scene(scene), splitIndex(splitIndex) {}

		  /*! array partitioning */
		  void split(const Split& split, const PrimInfo& pinfo, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
		  {
						if (likely(pinfo.size() < PARALLEL_THRESHOLD))
					  	sequential_split(split,set,left,lset,right,rset);
							//sequential_partition(split,set,left,lset,right,rset);
						else
			 				  parallel_split(split,set,left,lset,right,rset);
		  }

			void sequential_partition(const Split& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) {
				const size_t begin = set.begin();
			  const size_t end   = set.end();
				CentGeomBBox3fa local_left(empty);
				CentGeomBBox3fa local_right(empty);
			  const float splitPos = split.pos;
			  const unsigned int splitDim = split.dim;

				int i = begin;
				int j = end - 1;

				while (prims[i].center2()[splitDim] <= splitPos) {
					local_left.extend(prims[i].bounds());
					++i;
				}

				while (prims[j].center2()[splitDim] > splitPos) {
					local_right.extend(prims[j].bounds());
					--j;
				}

				while (i < j) {

					xchg(prims[i], prims[j]);
					local_left.extend(prims[i].bounds());

					local_right.extend(prims[j].bounds());
					++i;
					--j;

					while (prims[i].center2()[splitDim] <= splitPos) {
						local_left.extend(prims[i].bounds());
						++i;
					}

					while (prims[j].center2()[splitDim] > splitPos) {
						local_right.extend(prims[j].bounds());
						--j;
					}
				}

				size_t center = i;
				new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds);
			  new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds);
			  new (&lset) range<size_t>(begin,center);
			  new (&rset) range<size_t>(center,end);
			}
		  /*! array partitioning */
		  void sequential_split(const Split& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
		  {
			  /*
			  if (!split.valid()) {
				  deterministic_order(set);
				  return splitFallback(set,left,lset,right,rset);
			  }
			  */

			  const size_t begin = set.begin();
			  const size_t end   = set.end();
			  CentGeomBBox3fa local_left(empty);
			  CentGeomBBox3fa local_right(empty);
			  const float splitPos = split.pos;
			  const unsigned int splitDim = split.dim;
			  const unsigned int splitDimMask = (unsigned int)1 << splitDim; // FIXME: also use in unaligned and spatial binner

			  size_t center = serial_partitioning(prims,begin,end,local_left,local_right,
												  [&] (const PrimRef& ref) { return ref.center2()[splitDim] < splitPos; },
												  [] (CentGeomBBox3fa& pinfo,const PrimRef& ref) { pinfo.extend(ref.bounds()); });

			  new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds);
			  new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds);
			  new (&lset) range<size_t>(begin,center);
			  new (&rset) range<size_t>(center,end);
			  assert(area(left.geomBounds) >= 0.0f);
			  assert(area(right.geomBounds) >= 0.0f);
		  }

		  /*! array partitioning */
		  void parallel_split(const Split& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
		  {
			  /*
			  if (!split.valid()) {
				  deterministic_order(set);
				  return splitFallback(set,left,lset,right,rset);
			  }
			  */

			  const size_t begin = set.begin();
			  const size_t end   = set.end();
			  left.reset();
			  right.reset();
			  PrimInfo init; init.reset();
			  const float splitPos = split.pos;
			  const unsigned int splitDim = split.dim;

			  const size_t mid = parallel_in_place_partitioning<PARALLEL_PARITION_BLOCK_SIZE,PrimRef,PrimInfo>
			  (&prims[begin],end-begin,init,left,right,
			   [&] (const PrimRef &ref) { return ref.center2()[splitDim] < splitPos; },
			   [] (PrimInfo &pinfo,const PrimRef &ref) { pinfo.add(ref.bounds()); },
			   [] (PrimInfo &pinfo0,const PrimInfo &pinfo1) { pinfo0.merge(pinfo1); });

			  const size_t center = begin+mid;
			  left.begin  = begin;  left.end  = center; // FIXME: remove?
			  right.begin = center; right.end = end;

			  new (&lset) range<size_t>(begin,center);
			  new (&rset) range<size_t>(center,end);
		  }

		  void sortIndices(FastSweepData& data, const int count, unsigned offset) {

			  __m128 neg = _mm_set_ps(0.0f, -0.0f, -0.0f, -0.f); // shouldn't it be (-0, -0, -0, 0) ?


			  int i = 0;
			  for (; i < count - 3; i += 4) {

				// crashes if prims[(i + 0 + offset)].center2() are used

				//  __m128 mid0 = prims[(i + 0 + offset)].center2();

				__m128 mid0 = _mm_add_ps(prims[(i + 0 + offset)].lower, prims[(i + 0 + offset)].upper);
				  prims[(i + 0 + offset)].lower = _mm_xor_ps(prims[(i + 0 + offset)].lower, neg);
				  //__m128 mid1 = prims[(i + 1 + data.offset)].center2();
					__m128 mid1 = _mm_add_ps(prims[(i + 1 + offset)].lower, prims[(i + 1 + offset)].upper);
				  prims[(i + 1 + offset)].lower = _mm_xor_ps(prims[(i + 1 + offset)].lower, neg);
				  //__m128 mid2 = prims[(i + 2 + data.offset)].center2();
					__m128 mid2 = _mm_add_ps(prims[(i + 2 + offset)].lower, prims[(i + 2 + offset)].upper);
				  prims[(i + 2 + offset)].lower = _mm_xor_ps(prims[(i + 2 + offset)].lower, neg);
				  //__m128 mid3 = prims[(i + 3 + data.offset)].center2();
					__m128 mid3 = _mm_add_ps(prims[(i + 3 + offset)].lower, prims[(i + 3 + offset)].upper);
				  prims[(i + 3 + offset)].lower = _mm_xor_ps(prims[(i + 3 + offset)].lower, neg);


				  _MM_TRANSPOSE4_PS(mid0, mid1, mid2, mid3);
				  _mm_store_ps(data.midX + i, mid0);
				  _mm_store_ps(data.midY + i, mid1);
				  _mm_store_ps(data.midZ + i, mid2);
//#define STD_SORT

#ifdef STD_SORT
					data.indices[0][i+0] = data.indices[1][i+0] = data.indices[2][i+0] = i + offset + 0;
					data.indices[0][i+1] = data.indices[1][i+1] = data.indices[2][i+1] = i + offset + 1;
					data.indices[0][i+2] = data.indices[1][i+2] = data.indices[2][i+2] = i + offset + 2;
					data.indices[0][i+3] = data.indices[1][i+3] = data.indices[2][i+3] = i + offset + 3;
#endif
			  }

			  for (; i < count; i++) {

				  //Vec3fa mid = prims[(i + offset)].center2();
					__m128 mid = _mm_add_ps(prims[(i + offset)].lower, prims[(i + offset)].upper);
				  prims[(i + offset)].lower = _mm_xor_ps(prims[(i + offset)].lower, neg);
					ALIGNED(32) float midArray[4];
					_mm_store_ps(midArray, mid);
				  data.midX[i] = midArray[0];
				  data.midY[i] = midArray[1];
				  data.midZ[i] = midArray[2];
#ifdef STD_SORT
					data.indices[0][i+0] = data.indices[1][i+0] = data.indices[2][i+0] = i + offset + 0;
#endif
			  }
				float* mmx = data.midX;
				float* mmy = data.midY;
				float* mmz = data.midZ;

#ifdef STD_SORT
				std::sort(data.indices[0], data.indices[0] + count, [mmx, offset](unsigned& a, unsigned& b) { return mmx[a-offset] < mmx[b-offset]; });
				std::sort(data.indices[1], data.indices[1] + count, [mmy, offset](unsigned& a, unsigned& b) { return mmy[a-offset] < mmy[b-offset]; });
				std::sort(data.indices[2], data.indices[2] + count, [mmz, offset](unsigned& a, unsigned& b) { return mmz[a-offset] < mmz[b-offset]; });
#else
				sortedIndicesFromFloatsX3(data.midX, data.midY, data.midZ, data.indices[0], data.indices[1], data.indices[2], count, offset);
#endif
				for (int i = 0; i < count-1; i++) {
					assert(mmx[data.indices[0][i]-offset] <= mmx[data.indices[0][i+1]-offset]);
					assert(mmy[data.indices[1][i]-offset] <= mmy[data.indices[1][i+1]-offset]);
					assert(mmz[data.indices[2][i]-offset] <= mmz[data.indices[2][i+1]-offset]);
				}
			  }

			__forceinline __m256 computeBounds(const float* __restrict leafBounds, const unsigned* __restrict remap, unsigned first, unsigned last) {
				__m256 bounds0 = _mm256_load_ps(leafBounds + remap[first]*8);
				__m256 bounds1 = bounds0;
				__m256 bounds2 = bounds0;
				__m256 bounds3 = bounds0;

				unsigned i = first;

				for (; i+7 < last; i += 8) {
					unsigned i0 = remap[i+0];
					unsigned i1 = remap[i+1];
					unsigned i2 = remap[i+2];
					unsigned i3 = remap[i+3];

					__m256 a0 = _mm256_load_ps(leafBounds + i0*8);
					__m256 a1 = _mm256_load_ps(leafBounds + i1*8);
					__m256 a2 = _mm256_load_ps(leafBounds + i2*8);
					__m256 a3 = _mm256_load_ps(leafBounds + i3*8);

					unsigned i4 = remap[i+4];
					unsigned i5 = remap[i+5];
					unsigned i6 = remap[i+6];
					unsigned i7 = remap[i+7];

					__m256 a4 = _mm256_load_ps(leafBounds + i4*8);
					__m256 a5 = _mm256_load_ps(leafBounds + i5*8);
					__m256 a6 = _mm256_load_ps(leafBounds + i6*8);
					__m256 a7 = _mm256_load_ps(leafBounds + i7*8);

					a1 = _mm256_max_ps(a1, a0);
					a3 = _mm256_max_ps(a3, a2);
					a5 = _mm256_max_ps(a5, a4);
					a7 = _mm256_max_ps(a7, a6);

					bounds0 = _mm256_max_ps(bounds0, a1);
					bounds1 = _mm256_max_ps(bounds1, a3);
					bounds2 = _mm256_max_ps(bounds2, a5);
					bounds3 = _mm256_max_ps(bounds3, a7);
				}

				for (; i < last; ++i)
					bounds0 = _mm256_max_ps(bounds0, _mm256_load_ps(leafBounds + remap[i]*8));

				return _mm256_max_ps(_mm256_max_ps(bounds0, bounds1), _mm256_max_ps(bounds2, bounds3));
			}

			inline BBox3fa avxBoundsToEmbreeBounds(__m256 bounds) {
				  __m128 neg = _mm_set_ps(0.0f, -0.0f, -0.0f, -0.f);

				Vec3fa lower(_mm_xor_ps(_mm256_extractf128_ps(bounds, 0), neg));
				Vec3fa upper(_mm256_extractf128_ps(bounds, 1));

				return BBox3fa(lower, upper);
			}

		  bool fastSweep(FastSweepData& data, PrimInfo& current, Split& csplit, Set& cprims, PrimInfo& left, Set& lprims, Split& lsplit, PrimInfo& right, Set& rprims, Split& rsplit, const size_t maxLeafSize, unsigned offset) {

				int block_shift = 2;
				const int mult = 1;//1000000;
			  const float traversalCost = 0.5f;
			  const float intersectionCost = 1.0f;

			  unsigned first = cprims.begin();
			  unsigned last = cprims.end();

				 float* __restrict__ leafBounds = reinterpret_cast<float*>(prims);

				/*
				if (depth == 0) {
					__m256 cb = computeBounds(leafBounds, data.indices[0], first, last);
					current.geomBounds = avxBoundsToEmbreeBounds(cb);
					float divPsa = intersectionCost / surfaceArea(cb);

				}
				*/
			  float divPsa = intersectionCost / surfaceArea(combine(_mm_xor_ps(current.geomBounds.lower, _mm_set_ps(-0.0f, -0.0f, -0.0f, -0.f)), current.geomBounds.upper));

			  float bestSah = std::numeric_limits<float>::infinity();

			  unsigned bestDim = 0xffffffff;
			  unsigned pivot = 0xffffffff;

			  __m256 bestBounds;
				//std::cout << "first: " << first << " pivot: " << pivot << " last: " << last << std::endl;
			  if (divPsa != std::numeric_limits<float>::infinity()) {

				  for (unsigned dim = 0; dim < 3; ++dim) {
					  unsigned* __restrict sortedIndices = data.indices[dim];
					  float* __restrict accumulatedArea = data.accumulatedArea;

					  /* left to right accumulate sweep */
					  __m256 bounds = _mm256_load_ps(leafBounds + sortedIndices[first]*8);

					  {
						  int i = (int)first;

						  for(; (int)i < (int)last - 8; i+=8) {

							  __m256 bb0 = _mm256_load_ps(leafBounds + sortedIndices[i+0]*8);
							  __m256 bb1 = _mm256_load_ps(leafBounds + sortedIndices[i+1]*8);
							  __m256 bb2 = _mm256_load_ps(leafBounds + sortedIndices[i+2]*8);
							  __m256 bb3 = _mm256_load_ps(leafBounds + sortedIndices[i+3]*8);

							  bb0 = _mm256_max_ps(bb0, bounds);

							  __m256 bb4 = _mm256_load_ps(leafBounds + sortedIndices[i+4]*8);
							  __m256 bb5 = _mm256_load_ps(leafBounds + sortedIndices[i+5]*8);
							  __m256 bb6 = _mm256_load_ps(leafBounds + sortedIndices[i+6]*8);
							  __m256 bb7 = _mm256_load_ps(leafBounds + sortedIndices[i+7]*8);


							  bb1 = _mm256_max_ps(bb1, bb0);
							  bb3 = _mm256_max_ps(bb3, bb2);
							  bb5 = _mm256_max_ps(bb5, bb4);
							  bb7 = _mm256_max_ps(bb7, bb6);
							  bb3 = _mm256_max_ps(bb3, bb1);
							  bb7 = _mm256_max_ps(bb7, bb5);
							  bb7 = _mm256_max_ps(bb7, bb3);
							  bb5 = _mm256_max_ps(bb5, bb3);
							  bb4 = _mm256_max_ps(bb4, bb3);
							  bb2 = _mm256_max_ps(bb2, bb1);
							  bb6 = _mm256_max_ps(bb6, bb5);

							  bounds = bb7;

							   __m256 d0 = _mm256_add_ps(_mm256_permute2f128_ps(bb0, bb4, (0) | ((2) << 4)),
							   _mm256_permute2f128_ps(bb0, bb4, (1) | ((3) << 4)));
							   __m256 d1 = _mm256_add_ps(_mm256_permute2f128_ps(bb1, bb5, (0) | ((2) << 4)),
							   _mm256_permute2f128_ps(bb1, bb5, (1) | ((3) << 4)));
							   __m256 d2 = _mm256_add_ps(_mm256_permute2f128_ps(bb2, bb6, (0) | ((2) << 4)),
							   _mm256_permute2f128_ps(bb2, bb6, (1) | ((3) << 4)));
							   __m256 d3 = _mm256_add_ps(_mm256_permute2f128_ps(bb3, bb7, (0) | ((2) << 4)),
							   _mm256_permute2f128_ps(bb3, bb7, (1) | ((3) << 4)));


							  _MM256_TRANSPOSE4_PS(d0, d1, d2, d3);

							  __m256 area = _mm256_fmadd_ps(d0, d1, _mm256_fmadd_ps(d0, d2, _mm256_mul_ps(d1, d2)));

							  _mm256_storeu_ps(accumulatedArea + i, area);

						  }
							for (; i < last-1; ++i) {
								bounds = _mm256_max_ps(bounds, _mm256_load_ps(leafBounds + sortedIndices[i]*8));
								accumulatedArea[i] = surfaceArea(bounds);
							}
					  }

						/* right to left sah cost computation sweep */
						bounds = _mm256_load_ps(leafBounds + sortedIndices[last-1]*8);
						{
							__m256 bb[8];
							int i = (int)last - 1;

							for (; i > (int)first+7; i -= 8) {

								bb[0] = _mm256_load_ps(leafBounds + sortedIndices[i-0]*8);
								bb[1] = _mm256_load_ps(leafBounds + sortedIndices[i-1]*8);
								bb[2] = _mm256_load_ps(leafBounds + sortedIndices[i-2]*8);
								bb[3] = _mm256_load_ps(leafBounds + sortedIndices[i-3]*8);

								bb[0] = _mm256_max_ps(bb[0], bounds);

								bb[4] = _mm256_load_ps(leafBounds + sortedIndices[i-4]*8);
								bb[5] = _mm256_load_ps(leafBounds + sortedIndices[i-5]*8);
								bb[6] = _mm256_load_ps(leafBounds + sortedIndices[i-6]*8);
								bb[7] = _mm256_load_ps(leafBounds + sortedIndices[i-7]*8);


								bb[1] = _mm256_max_ps(bb[1], bb[0]);
								bb[3] = _mm256_max_ps(bb[3], bb[2]);
								bb[5] = _mm256_max_ps(bb[5], bb[4]);
								bb[7] = _mm256_max_ps(bb[7], bb[6]);

								bb[3] = _mm256_max_ps(bb[3], bb[1]);

								__m256 lsa = _mm256_loadu_ps(accumulatedArea + (i - 8));

								bb[7] = _mm256_max_ps(bb[7], bb[5]);

								lsa = _mm256_shuffle_ps(lsa, lsa, _MM_SHUFFLE(0,1,2,3));
								lsa = _mm256_permute2f128_ps(lsa, lsa, (1) | ((0) << 4));

								bb[7] = _mm256_max_ps(bb[7], bb[3]);
								bb[5] = _mm256_max_ps(bb[5], bb[3]);
								bb[4] = _mm256_max_ps(bb[4], bb[3]);
								bb[2] = _mm256_max_ps(bb[2], bb[1]);
								bb[6] = _mm256_max_ps(bb[6], bb[5]);

								bounds = bb[7];
								__m256 d0 = _mm256_add_ps(_mm256_permute2f128_ps(bb[0], bb[4], (0) | ((2) << 4)),
														  _mm256_permute2f128_ps(bb[0], bb[4], (1) | ((3) << 4)));
								__m256 d1 = _mm256_add_ps(_mm256_permute2f128_ps(bb[1], bb[5], (0) | ((2) << 4)),
														  _mm256_permute2f128_ps(bb[1], bb[5], (1) | ((3) << 4)));
								__m256 d2 = _mm256_add_ps(_mm256_permute2f128_ps(bb[2], bb[6], (0) | ((2) << 4)),
														  _mm256_permute2f128_ps(bb[2], bb[6], (1) | ((3) << 4)));
								__m256 d3 = _mm256_add_ps(_mm256_permute2f128_ps(bb[3], bb[7], (0) | ((2) << 4)),
														  _mm256_permute2f128_ps(bb[3], bb[7], (1) | ((3) << 4)));


								_MM256_TRANSPOSE4_PS(d0, d1, d2, d3);

								__m256 rsa = _mm256_fmadd_ps(d0, d1, _mm256_fmadd_ps(d0, d2, _mm256_mul_ps(d1, d2)));
								__m256i leftIndices;
								__m256i rightIndices;
								if (last - first <= maxLeafSize*mult) {
								int lbi[8];
								int rbi[8];
								for (int bi = 0; bi < 8; bi++) {
										lbi[bi] = ((size_t((i - bi) - first)+(size_t(1)<<block_shift)-1) >> block_shift);
										rbi[bi] = ((size_t(last - (i - bi))+(size_t(1)<<block_shift)-1) >> block_shift);
								}
								//std::cout << "first: " << first << " pivot: " << pivot << " last: " << last << std::endl;
								leftIndices = _mm256_setr_epi32(lbi[0], lbi[1], lbi[2], lbi[3], lbi[4], lbi[5], lbi[6], lbi[7]);
								rightIndices = _mm256_setr_epi32(rbi[0], rbi[1], rbi[2], rbi[3], rbi[4], rbi[5], rbi[6], rbi[7]);
								}
/*
								__m256 sah = _mm256_add_ps(_mm256_mul_ps(lsa, leftIndices),
															_mm256_mul_ps(rsa, rightIndices));
*/
								else {
									__m256i indices = _mm256_sub_epi32(_mm256_set1_epi32(i), _mm256_setr_epi32(0, 1, 2, 3, 4, 5, 6, 7));
									leftIndices = _mm256_sub_epi32(indices, _mm256_set1_epi32(first));
									rightIndices = _mm256_sub_epi32(_mm256_set1_epi32(last), indices);
								}

/*
								__m256 sah = _mm256_fmadd_ps(_mm256_set1_ps(intersectionCost * divPsa),
															_mm256_fmadd_ps(lsa, _mm256_cvtepi32_ps(leftIndices),
															_mm256_mul_ps(rsa, _mm256_cvtepi32_ps(rightIndices))), _mm256_set1_ps(traversalCost));
*/
															__m256 sah = _mm256_add_ps(_mm256_mul_ps(lsa, _mm256_cvtepi32_ps(leftIndices)),
																						_mm256_mul_ps(rsa, _mm256_cvtepi32_ps(rightIndices)));

				if (_mm256_movemask_ps(_mm256_cmp_ps(sah, _mm256_set1_ps(bestSah), _CMP_LT_OQ))) {
					__m256 minSah = _mm256_min_ps(sah, _mm256_shuffle_ps(sah, sah, _MM_SHUFFLE(2,3,0,1)));
					minSah = _mm256_min_ps(minSah, _mm256_shuffle_ps(minSah, minSah, _MM_SHUFFLE(1,0,3,2)));
					minSah = _mm256_min_ps(minSah, _mm256_permute2f128_ps(minSah, minSah, (1) << ((0) << 4)));

					unsigned mask = _mm256_movemask_ps(_mm256_cmp_ps(minSah, sah, _CMP_EQ_OQ));

					ALIGNED(32) float sahArray[8];
					_mm256_store_ps(sahArray, sah);

					unsigned index = __builtin_ctz(mask);

					bestBounds = bb[index];
					bestSah = sahArray[index];
					bestDim = dim;
					pivot = i-(int)index;
				}
			}

			for (; i > (int)first; --i) {
				bounds = _mm256_max_ps(bounds, _mm256_load_ps(leafBounds + sortedIndices[i]*8));

				float lsa = accumulatedArea[i - 1];
				float rsa = surfaceArea(bounds);

				if (last - first <= maxLeafSize*mult) {
					lsa *= float((size_t(i-first)+(size_t(1)<<block_shift)-1) >> block_shift);
					rsa *= float((size_t(last-i)+(size_t(1)<<block_shift)-1) >> block_shift);
				}
				else {
					lsa *= ((float)(i - (int)first));	// Left child.
					rsa *= ((float)((int)last - i));	// Right child.
				}

				//std::cout << lcount << ", " << rcount << std::endl;
				float sah =  //traversalCost + intersectionCost * divPsa * // Constant cost.
				//(lsa * ((float)(i - (int)first)) +	// Left child.
				//rsa * ((float)((int)last - i)));	// Right child.
				 lsa +
				 rsa;

				if (sah < bestSah) {
					bestBounds = bounds;
					bestSah = sah;
					bestDim = dim;
					pivot = i;
				}
			}
					  }
				  }
			  }
			  else {
				  divPsa = 1.f;
				  bestDim = 0;
			  }
				//std::cout << "first: " << first << " pivot: " << pivot << " last: " << last << std::endl;
				float leafBlocks = (last - first);//float((size_t(last-first)+(size_t(1)<<block_shift)-1) >> block_shift);
				if (last - first <= maxLeafSize*mult)
					leafBlocks = float((size_t(last-first)+(size_t(1)<<block_shift)-1) >> block_shift);
				//float blockBestSah = surfaceArea(bestBounds) * leafBlocks;
			  //if (bestSah > leafBlocks * intersectionCost) {
					if (traversalCost + intersectionCost * divPsa * bestSah > leafBlocks * intersectionCost) {
				  if (last - first > maxLeafSize) {

						//if (bestSah == std::numeric_limits<float>::infinity()) {
					  	pivot = (first + last) >> 1;

							bestBounds = computeBounds(leafBounds, data.indices[bestDim], pivot, last);
						//}

				  }

				  else {
					  csplit.leafFound();
					  return true;
				  }
			  }

			  lprims = Set(first, pivot);
			  rprims = Set(pivot, last);


			  partition(data, bestDim, first, last, pivot, offset);

				BBox3fa finalBounds = avxBoundsToEmbreeBounds(bestBounds);
			  right = PrimInfo(pivot, last, finalBounds, finalBounds);

				bestBounds = computeBounds(leafBounds, data.indices[bestDim], first, pivot);
				finalBounds = avxBoundsToEmbreeBounds(bestBounds);

			  left = PrimInfo(first, pivot, finalBounds, finalBounds);

			  return false;
		  }

		  void partition(FastSweepData& data, unsigned axis, unsigned first, unsigned last, unsigned pivot, unsigned offset) const {
			  unsigned* __restrict ref = data.indices[axis];
			  unsigned char* __restrict leftPartition = data.leftPartition;

			  for (unsigned i = first; i < pivot; ++i)
				  leftPartition[ref[i] - offset] = 1;

			  for (unsigned i = pivot; i < last; ++i)
				  leftPartition[ref[i] - offset] = 0;

			  partitionAccordingToRef(data, (axis+1)%3, first, last, offset);
			  partitionAccordingToRef(data, (axis+2)%3, first, last, offset);
		  }

		  void partitionAccordingToRef(FastSweepData& data, unsigned axis, unsigned first, unsigned last, unsigned offset) const {

			  unsigned* __restrict temporaryIndices = data.temporaryIndices;
			  unsigned char* __restrict leftPartition = data.leftPartition;

			  unsigned* __restrict indices = data.indices[axis];

			  unsigned* source = indices + first;
			  unsigned* end = indices + last;

			  unsigned* left = source;
			  unsigned* right = temporaryIndices + first;

			  while (source < end) {
				  unsigned index = *source;
				  unsigned l = leftPartition[index - offset];

				  *left = index;
				  *right = index;

				  left += l;
				  right += 1-l;

				  ++source;
			  }
			  std::copy(temporaryIndices + first, right, left);
		  }

		  void gatherReferences(FastSweepData& data, const Set& current) {
			  PrimRef* localPrims = data.gatheredRefs;
			  for (int i = 0; i < current.size(); i++) {
				  localPrims[i] = prims[data.indices[0][i + current.begin()]];
			  }
		}

      private:
        PrimRef* prims;
				Scene* scene;
				volatile int splitIndex; // not currently used
      };
  }
}
