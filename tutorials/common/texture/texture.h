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

/*! This header is shared with ISPC. */
#pragma once

/*! Embree format constants for Texture creation */
typedef enum {
  TEXTURE_RGBA8,
  TEXTURE_SRGBA,
  TEXTURE_RGBA32F,
  TEXTURE_RGB8,
  TEXTURE_SRGB,
  TEXTURE_RGB32F,
  TEXTURE_R8,
  TEXTURE_R32F,
/* TODO
  LogLuv,
  RGBA16F
  RGB16F
  RGBE, // radiance hdr
  compressed (RGTC, BPTC, ETC, ...)
*/
} TextureFormat;

/*! flags that can be passed to ospNewTexture2D(); can be OR'ed together */
typedef enum {
  TEXTURE_SHARED_BUFFER = (1<<0),
  TEXTURE_FILTER_NEAREST = (1<<1) /*!< use nearest-neighbor interpolation rather than the default bilinear interpolation */
} TextureCreationFlags;

