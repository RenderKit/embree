// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

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

