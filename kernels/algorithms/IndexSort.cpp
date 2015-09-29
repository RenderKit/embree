//
//  IndexSort.cpp
//  RayAccelerator
//
//  Created by Rasmus Barringer on 2014-09-03.
//  Copyright (c) 2014 Rasmus Barringer. All rights reserved.
//

#include "IndexSort.h"
#include <immintrin.h>
#include <algorithm>

void sortedIndicesFromFloats(float* __restrict values, unsigned* __restrict indices, unsigned count) {
	long long* keys = reinterpret_cast<long long*>(indices);

	__m256i i0 = _mm256_setr_epi32(0, 1, 2, 3, 4, 5, 6, 7);
	__m256i i1 = _mm256_setr_epi32(8, 9, 10, 11, 12, 13, 14, 15);
	__m256i i2 = _mm256_setr_epi32(16, 17, 18, 19, 20, 21, 22, 23);
	__m256i i3 = _mm256_setr_epi32(24, 25, 26, 27, 28, 29, 30, 31);

	__m256i increment = _mm256_set1_epi32(32);
	__m256i signMask = _mm256_set1_epi32(0x80000000);

	unsigned i = 0;

	for (; i+31 < count; i += 32) {
		const __m256i* input = reinterpret_cast<const __m256i*>(values + i);
		__m256i* output = reinterpret_cast<__m256i*>(keys + i);

		__m256i value0 = _mm256_load_si256(input + 0);
		__m256i value1 = _mm256_load_si256(input + 1);
		__m256i value2 = _mm256_load_si256(input + 2);
		__m256i value3 = _mm256_load_si256(input + 3);

		__m256i absValue0 = _mm256_andnot_si256(signMask, value0);
		__m256i absValue1 = _mm256_andnot_si256(signMask, value1);
		__m256i absValue2 = _mm256_andnot_si256(signMask, value2);
		__m256i absValue3 = _mm256_andnot_si256(signMask, value3);

		__m256i encodedValue0 = _mm256_sign_epi32(absValue0, value0);
		__m256i encodedValue1 = _mm256_sign_epi32(absValue1, value1);
		__m256i encodedValue2 = _mm256_sign_epi32(absValue2, value2);
		__m256i encodedValue3 = _mm256_sign_epi32(absValue3, value3);

		__m256i key00 = _mm256_unpacklo_epi32(i0, encodedValue0);
		__m256i key01 = _mm256_unpackhi_epi32(i0, encodedValue0);
		__m256i key10 = _mm256_unpacklo_epi32(i1, encodedValue1);
		__m256i key11 = _mm256_unpackhi_epi32(i1, encodedValue1);
		__m256i key20 = _mm256_unpacklo_epi32(i2, encodedValue2);
		__m256i key21 = _mm256_unpackhi_epi32(i2, encodedValue2);
		__m256i key30 = _mm256_unpacklo_epi32(i3, encodedValue3);
		__m256i key31 = _mm256_unpackhi_epi32(i3, encodedValue3);

		_mm256_store_si256(output + 0, key00);
		_mm256_store_si256(output + 1, key01);
		_mm256_store_si256(output + 2, key10);
		_mm256_store_si256(output + 3, key11);
		_mm256_store_si256(output + 4, key20);
		_mm256_store_si256(output + 5, key21);
		_mm256_store_si256(output + 6, key30);
		_mm256_store_si256(output + 7, key31);

		i0 = _mm256_add_epi32(i0, increment);
		i1 = _mm256_add_epi32(i1, increment);
		i2 = _mm256_add_epi32(i2, increment);
		i3 = _mm256_add_epi32(i3, increment);
	}

	for (; i < count; ++i) {
		int input = *reinterpret_cast<const int*>(values + i);
		int absInput = input & 0x7fffffff;

		if (input < 0)
			absInput = -absInput;

		keys[i] = ((long long)absInput << 32) | i;
	}

	std::sort(keys, keys + count);

	for (unsigned i = 0; i < count; i += 32) {
		const float* input = reinterpret_cast<const float*>(keys + i);
		float* output = reinterpret_cast<float*>(indices + i);

		__m256 value0 = _mm256_load_ps(input + 0);
		__m256 value1 = _mm256_load_ps(input + 8);
		__m256 value2 = _mm256_load_ps(input + 16);
		__m256 value3 = _mm256_load_ps(input + 24);
		__m256 value4 = _mm256_load_ps(input + 32);
		__m256 value5 = _mm256_load_ps(input + 40);
		__m256 value6 = _mm256_load_ps(input + 48);
		__m256 value7 = _mm256_load_ps(input + 56);

		__m256 i0 = _mm256_shuffle_ps(value0, value1, _MM_SHUFFLE(2,0,2,0));
		__m256 i1 = _mm256_shuffle_ps(value2, value3, _MM_SHUFFLE(2,0,2,0));
		__m256 i2 = _mm256_shuffle_ps(value4, value5, _MM_SHUFFLE(2,0,2,0));
		__m256 i3 = _mm256_shuffle_ps(value6, value7, _MM_SHUFFLE(2,0,2,0));

		i0 = _mm256_castpd_ps(_mm256_permute4x64_pd(_mm256_castps_pd(i0), (0) | ((2) << 2) | ((1) << 4) | ((3) << 6)));
		i1 = _mm256_castpd_ps(_mm256_permute4x64_pd(_mm256_castps_pd(i1), (0) | ((2) << 2) | ((1) << 4) | ((3) << 6)));
		i2 = _mm256_castpd_ps(_mm256_permute4x64_pd(_mm256_castps_pd(i2), (0) | ((2) << 2) | ((1) << 4) | ((3) << 6)));
		i3 = _mm256_castpd_ps(_mm256_permute4x64_pd(_mm256_castps_pd(i3), (0) | ((2) << 2) | ((1) << 4) | ((3) << 6)));

		_mm256_store_ps(output + 0, i0);
		_mm256_store_ps(output + 8, i1);
		_mm256_store_ps(output + 16, i2);
		_mm256_store_ps(output + 24, i3);
	}
}

inline void radixSortUint64High32X3(unsigned long long* __restrict dataA, unsigned long long* __restrict dataB, unsigned long long* __restrict dataC,
									unsigned long long* __restrict tempA, unsigned long long* __restrict tempB, unsigned long long* __restrict tempC, unsigned count) {
	unsigned bucketsA0[257];
	unsigned bucketsA1[257];
	unsigned bucketsA2[257];
	unsigned bucketsA3[257];

	unsigned bucketsB0[257];
	unsigned bucketsB1[257];
	unsigned bucketsB2[257];
	unsigned bucketsB3[257];

	unsigned bucketsC0[257];
	unsigned bucketsC1[257];
	unsigned bucketsC2[257];
	unsigned bucketsC3[257];

	for (unsigned i = 0; i < 257; ++i) {
		bucketsA0[i] = 0;
		bucketsA1[i] = 0;
		bucketsA2[i] = 0;
		bucketsA3[i] = 0;

		bucketsB0[i] = 0;
		bucketsB1[i] = 0;
		bucketsB2[i] = 0;
		bucketsB3[i] = 0;

		bucketsC0[i] = 0;
		bucketsC1[i] = 0;
		bucketsC2[i] = 0;
		bucketsC3[i] = 0;
	}

	unsigned* histogramA0 = bucketsA0+1;
	unsigned* histogramA1 = bucketsA1+1;
	unsigned* histogramA2 = bucketsA2+1;
	unsigned* histogramA3 = bucketsA3+1;

	unsigned* histogramB0 = bucketsB0+1;
	unsigned* histogramB1 = bucketsB1+1;
	unsigned* histogramB2 = bucketsB2+1;
	unsigned* histogramB3 = bucketsB3+1;

	unsigned* histogramC0 = bucketsC0+1;
	unsigned* histogramC1 = bucketsC1+1;
	unsigned* histogramC2 = bucketsC2+1;
	unsigned* histogramC3 = bucketsC3+1;

	for (unsigned i = 0; i < count; ++i) {
		unsigned long long da = dataA[i];
		unsigned long long db = dataB[i];
		unsigned long long dc = dataC[i];

		++histogramA0[(da >> (4 << 3)) & 0xff];
		++histogramA1[(da >> (5 << 3)) & 0xff];
		++histogramA2[(da >> (6 << 3)) & 0xff];
		++histogramA3[(da >> (7 << 3)) & 0xff];

		++histogramB0[(db >> (4 << 3)) & 0xff];
		++histogramB1[(db >> (5 << 3)) & 0xff];
		++histogramB2[(db >> (6 << 3)) & 0xff];
		++histogramB3[(db >> (7 << 3)) & 0xff];

		++histogramC0[(dc >> (4 << 3)) & 0xff];
		++histogramC1[(dc >> (5 << 3)) & 0xff];
		++histogramC2[(dc >> (6 << 3)) & 0xff];
		++histogramC3[(dc >> (7 << 3)) & 0xff];
	}

	for (unsigned i = 1; i < 256; ++i) {
		bucketsA0[i] += bucketsA0[i-1];
		bucketsA1[i] += bucketsA1[i-1];
		bucketsA2[i] += bucketsA2[i-1];
		bucketsA3[i] += bucketsA3[i-1];

		bucketsB0[i] += bucketsB0[i-1];
		bucketsB1[i] += bucketsB1[i-1];
		bucketsB2[i] += bucketsB2[i-1];
		bucketsB3[i] += bucketsB3[i-1];

		bucketsC0[i] += bucketsC0[i-1];
		bucketsC1[i] += bucketsC1[i-1];
		bucketsC2[i] += bucketsC2[i-1];
		bucketsC3[i] += bucketsC3[i-1];
	}

	for (unsigned i = 0; i < count; ++i) {
		unsigned long long da = dataA[i];
		unsigned long long db = dataB[i];
		unsigned long long dc = dataC[i];

		unsigned indexA = bucketsA0[(da >> (4 << 3)) & 0xff]++;
		unsigned indexB = bucketsB0[(db >> (4 << 3)) & 0xff]++;
		unsigned indexC = bucketsC0[(dc >> (4 << 3)) & 0xff]++;

		tempA[indexA] = da;
		tempB[indexB] = db;
		tempC[indexC] = dc;
	}

	for (unsigned i = 0; i < count; ++i) {
		unsigned long long da = tempA[i];
		unsigned long long db = tempB[i];
		unsigned long long dc = tempC[i];

		unsigned indexA = bucketsA1[(da >> (5 << 3)) & 0xff]++;
		unsigned indexB = bucketsB1[(db >> (5 << 3)) & 0xff]++;
		unsigned indexC = bucketsC1[(dc >> (5 << 3)) & 0xff]++;

		dataA[indexA] = da;
		dataB[indexB] = db;
		dataC[indexC] = dc;
	}

	for (unsigned i = 0; i < count; ++i) {
		unsigned long long da = dataA[i];
		unsigned long long db = dataB[i];
		unsigned long long dc = dataC[i];

		unsigned indexA = bucketsA2[(da >> (6 << 3)) & 0xff]++;
		unsigned indexB = bucketsB2[(db >> (6 << 3)) & 0xff]++;
		unsigned indexC = bucketsC2[(dc >> (6 << 3)) & 0xff]++;

		tempA[indexA] = da;
		tempB[indexB] = db;
		tempC[indexC] = dc;
	}

	for (unsigned i = 0; i < count; ++i) {
		unsigned long long da = tempA[i];
		unsigned long long db = tempB[i];
		unsigned long long dc = tempC[i];

		unsigned indexA = bucketsA3[(da >> (7 << 3)) & 0xff]++;
		unsigned indexB = bucketsB3[(db >> (7 << 3)) & 0xff]++;
		unsigned indexC = bucketsC3[(dc >> (7 << 3)) & 0xff]++;

		dataA[indexA] = da;
		dataB[indexB] = db;
		dataC[indexC] = dc;
	}
}

inline void packFloatsAndIndicesInUint64(float* __restrict values, unsigned long long* __restrict keys, unsigned count, int offset) {

	__m256i offset256 = _mm256_set1_epi32(offset);
	__m256i i0 = _mm256_add_epi32(_mm256_setr_epi32(0, 1, 2, 3, 4, 5, 6, 7), offset256);
	__m256i i1 = _mm256_add_epi32(_mm256_setr_epi32(8, 9, 10, 11, 12, 13, 14, 15), offset256);
	__m256i i2 = _mm256_add_epi32(_mm256_setr_epi32(16, 17, 18, 19, 20, 21, 22, 23), offset256);
	__m256i i3 = _mm256_add_epi32(_mm256_setr_epi32(24, 25, 26, 27, 28, 29, 30, 31), offset256);

	__m256i increment = _mm256_set1_epi32(32);
	__m256 signMask = _mm256_castps_si256(_mm256_set1_epi32(0x80000000));
	__m256 fullMask = _mm256_castps_si256(_mm256_set1_epi32(0xffffffff));

	unsigned i = 0;

	for (; i+31 < count; i += 32) {
		const float* input = reinterpret_cast<const float*>(values + i);
		float* output = reinterpret_cast<float*>(keys + i);

		__m256 value0 = _mm256_load_ps(input + 0*8);
		__m256 value1 = _mm256_load_ps(input + 1*8);
		__m256 value2 = _mm256_load_ps(input + 2*8);
		__m256 value3 = _mm256_load_ps(input + 3*8);

		__m256 mask0 = _mm256_blendv_ps(signMask, fullMask, value0);
		__m256 mask1 = _mm256_blendv_ps(signMask, fullMask, value1);
		__m256 mask2 = _mm256_blendv_ps(signMask, fullMask, value2);
		__m256 mask3 = _mm256_blendv_ps(signMask, fullMask, value3);

		__m256 encodedValue0 = _mm256_xor_ps(mask0, value0);
		__m256 encodedValue1 = _mm256_xor_ps(mask1, value1);
		__m256 encodedValue2 = _mm256_xor_ps(mask2, value2);
		__m256 encodedValue3 = _mm256_xor_ps(mask3, value3);

		__m256 key00 = _mm256_unpacklo_ps(i0, encodedValue0);
		__m256 key01 = _mm256_unpackhi_ps(i0, encodedValue0);
		__m256 key10 = _mm256_unpacklo_ps(i1, encodedValue1);
		__m256 key11 = _mm256_unpackhi_ps(i1, encodedValue1);
		__m256 key20 = _mm256_unpacklo_ps(i2, encodedValue2);
		__m256 key21 = _mm256_unpackhi_ps(i2, encodedValue2);
		__m256 key30 = _mm256_unpacklo_ps(i3, encodedValue3);
		__m256 key31 = _mm256_unpackhi_ps(i3, encodedValue3);

		_mm256_store_ps(output + 0*8, key00);
		_mm256_store_ps(output + 1*8, key01);
		_mm256_store_ps(output + 2*8, key10);
		_mm256_store_ps(output + 3*8, key11);
		_mm256_store_ps(output + 4*8, key20);
		_mm256_store_ps(output + 5*8, key21);
		_mm256_store_ps(output + 6*8, key30);
		_mm256_store_ps(output + 7*8, key31);

		i0 = _mm256_add_epi32(i0, increment);
		i1 = _mm256_add_epi32(i1, increment);
		i2 = _mm256_add_epi32(i2, increment);
		i3 = _mm256_add_epi32(i3, increment);
	}

	for (; i < count; ++i) {
		int input = *reinterpret_cast<const int*>(values + i);

		input ^= input < 0 ? 0xffffffff : 0x80000000;

		keys[i] = ((unsigned long long)input << 32) | (i + offset);
	}
}

inline void inPlaceUnpackIndicesFromLowUin64ToUint32(unsigned long long* keys, unsigned count) {
	for (unsigned i = 0; i < count; i += 32) {
		const float* input = reinterpret_cast<const float*>(keys + i);
		float* output = reinterpret_cast<float*>(keys) + i;

		__m256 value0 = _mm256_load_ps(input + 0);
		__m256 value1 = _mm256_load_ps(input + 8);
		__m256 value2 = _mm256_load_ps(input + 16);
		__m256 value3 = _mm256_load_ps(input + 24);
		__m256 value4 = _mm256_load_ps(input + 32);
		__m256 value5 = _mm256_load_ps(input + 40);
		__m256 value6 = _mm256_load_ps(input + 48);
		__m256 value7 = _mm256_load_ps(input + 56);

		__m256 i0 = _mm256_shuffle_ps(value0, value1, _MM_SHUFFLE(2,0,2,0));
		__m256 i1 = _mm256_shuffle_ps(value2, value3, _MM_SHUFFLE(2,0,2,0));
		__m256 i2 = _mm256_shuffle_ps(value4, value5, _MM_SHUFFLE(2,0,2,0));
		__m256 i3 = _mm256_shuffle_ps(value6, value7, _MM_SHUFFLE(2,0,2,0));

		i0 = _mm256_permute4x64_pd(i0, (0) | ((2) << 2) | ((1) << 4) | ((3) << 6));
		i1 = _mm256_permute4x64_pd(i1, (0) | ((2) << 2) | ((1) << 4) | ((3) << 6));
		i2 = _mm256_permute4x64_pd(i2, (0) | ((2) << 2) | ((1) << 4) | ((3) << 6));
		i3 = _mm256_permute4x64_pd(i3, (0) | ((2) << 2) | ((1) << 4) | ((3) << 6));

		_mm256_store_ps(output + 0, i0);
		_mm256_store_ps(output + 8, i1);
		_mm256_store_ps(output + 16, i2);
		_mm256_store_ps(output + 24, i3);
	}
}

void sortedIndicesFromFloatsX3(float* __restrict valuesA, float* __restrict valuesB, float* __restrict valuesC, unsigned* __restrict indicesA, unsigned* __restrict indicesB, unsigned* __restrict indicesC, unsigned count, int offset) {
	unsigned long long* keysA = reinterpret_cast<unsigned long long*>(indicesA);
	unsigned long long* keysB = reinterpret_cast<unsigned long long*>(indicesB);
	unsigned long long* keysC = reinterpret_cast<unsigned long long*>(indicesC);

	packFloatsAndIndicesInUint64(valuesA, keysA, count, offset);
	packFloatsAndIndicesInUint64(valuesB, keysB, count, offset);
	packFloatsAndIndicesInUint64(valuesC, keysC, count, offset);

	radixSortUint64High32X3(keysA, keysB, keysC, keysA + count, keysB + count, keysC + count, count);

	inPlaceUnpackIndicesFromLowUin64ToUint32(keysA, count);
	inPlaceUnpackIndicesFromLowUin64ToUint32(keysB, count);
	inPlaceUnpackIndicesFromLowUin64ToUint32(keysC, count);
}
