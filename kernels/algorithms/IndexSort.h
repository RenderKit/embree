//
//  IndexSort.h
//  RayAccelerator
//
//  Created by Rasmus Barringer on 2014-09-03.
//  Copyright (c) 2014 Rasmus Barringer. All rights reserved.
//

#ifndef RayAccelerator_IndexSort_h
#define RayAccelerator_IndexSort_h

// Note: indices needs to be 2x in size for temporary storage.
// Everything needs to be aligned and padded:
//  values - pad with 128 bytes
//  indices - pad with 256 bytes
void sortedIndicesFromFloats(float* __restrict values, unsigned* __restrict indices, unsigned count);

// Note: indices needs to be 4x in size for temporary storage.
// Everything needs to be aligned and padded:
//  values - pad with 128 bytes
//  indices - pad with 256 bytes
void sortedIndicesFromFloatsX3(float* __restrict valuesA, float* __restrict valuesB, float* __restrict valuesC, unsigned* __restrict indicesA, unsigned* __restrict indicesB, unsigned* __restrict indicesC, unsigned count, int offset);

#endif
