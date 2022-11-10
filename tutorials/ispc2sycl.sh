#!/bin/bash

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

echo Converting ISPC tutorial $1 to CPP tutorial $2
cp $1 $2

# uniform functions
sed -i.backup 's/uniform float uniformSampleConePDF/uniform float _uniformSampleConePDF/g' $2
sed -i.backup 's/uniform float uniformSampleDiskPDF/uniform float _uniformSampleDiskPDF/g' $2

# ISPC language
sed -i.backup  's/device.isph\"/sycl.h\"/g' $2
sed -i.backup  's/.isph\"/.h\"/g' $2
sed -i.backup  's/uniform //g' $2
sed -i.backup  's/ uniform\*/\*/g' $2
sed -i.backup  's/ uniform)/)/g' $2
sed -i.backup  's/*uniform)/*)/g' $2
sed -i.backup  's/varying //g' $2
sed -i.backup  's/ varying//g' $2
sed -i.backup  's/unmasked //g' $2
sed -i.backup  's/extern \"C\"/__EXTERN_C/g' $2
sed -i.backup  's/extern/extern \"C\"/g' $2
sed -i.backup  's/typedef export/typedef/g' $2
sed -i.backup  's/export/extern \"C\"/g' $2
sed -i.backup  's/extern \"C\" RTCScene g_scene/extern RTCScene g_scene/g' $2
sed -i.backup  's/__EXTERN_C/extern \"C\"/g' $2

#sed -i.backup 's/pos_inf/INFINITY/g' $2
#sed -i.backup 's/neg_inf/-INFINITY/g' $2
#sed -i.backup 's/inf/INFINITY/g' $2

sed -i.backup 's/int8/char/g' $2
sed -i.backup 's/int16/int16_t/g' $2
sed -i.backup 's/int32/int32_t/g' $2
sed -i.backup 's/int64/int64_t/g' $2
sed -i.backup 's/uchar/uint8_t/g' $2
sed -i.backup  's/uintptr_t/size_t/g' $2
sed -i.backup  's/intptr_t/ssize_t/g' $2

sed -i.backup  's/__mask/1/g' $2
sed -i.backup  's/lanemask()/1/g' $2
sed -i.backup  's/NULL/nullptr/g' $2

sed -i.backup  's/extract[ ]*(\([^,]*\),[^)]*)/\1/g' $2

#sed -i.backup  's/Vec3f renderPixel\([a-zA-Z0-9]*\)(/Vec3f renderPixel\1(sycl::intel::sub_group sg, /g' $2
#sed -i.backup  's/void renderPixelStandard(/void renderPixelStandard(sycl::intel::sub_group sg, /g' $2
sed -i.backup  's/RayStats_/\/\/RayStats_/g' $2

sed -i.backup  's/programIndex/0/g' $2
sed -i.backup  's/programCount/1/g' $2

sed -i.backup  's/renderPixelStandard(data/\/\/renderPixelStandard(data/g' $2
sed -i.backup  's/task[ ]*void[ ]*\([a-zA-Z0-9_]*\)[ ]*(/void \1 (int taskIndex, int threadIndex, /g' $2

sed -i.backup  's/launch\[\([^]]*\)\][ ]*\([a-zA-Z0-9_]*\)[ ]*(\([^)]*\))/\
  TutorialData ldata = data;\
  global_gpu_queue->submit([=](sycl::handler\& cgh)\{\
    const sycl::nd_range<2> nd_range = make_nd_range(height,width);\
    cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) \{\
        const unsigned int x = item.get_global_id(1); if (x >= width ) return;\
        const unsigned int y = item.get_global_id(0); if (y >= height) return;\
        RayStats stats;\
        renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,stats);\
      \});\
  \});\
  global_gpu_queue->wait_and_throw()/g' $2

sed -i.backup  's/foreach[ ]*([ ]*\([a-zA-Z0-9_]*\)[ ]*=[ ]*\([^ \.]*\)[ ]*\.\.\.[ ]*\([^ ),]*\)[ ]*)/for (unsigned int \1=\2; \1<\3; \1++)/g' $2
sed -i.backup  's/foreach_tiled[ ]*([ ]*\([a-zA-Z0-9_]*\)[ ]*=[ ]*\([^ \.]*\)[ ]*\.\.\.[ ]*\([^ ),]*\)[ ]*)/for (unsigned int \1=\2; \1<\3; \1++)/g' $2

sed -i.backup  's/foreach[ ]*([ ]*\([a-zA-Z0-9_]*\)[ ]*=[ ]*\([^ \.]*\)[ ]*\.\.\.[ ]*\([^ ),]*\)[ ]*\,[ ]*\([a-zA-Z0-9_]*\)[ ]*=[ ]*\([^ \.]*\)[ ]*\.\.\.[ ]*\([^ ),]*\)[ ]*)/for (unsigned int \1=\2; \1<\3; \1++) for (int \4=\5; \4<\6; \4++)/g' $2
sed -i.backup  's/foreach_tiled[ ]*([ ]*\([a-zA-Z0-9_]*\)[ ]*=[ ]*\([^ \.]*\)[ ]*\.\.\.[ ]*\([^ ),]*\)[ ]*\,[ ]*\([a-zA-Z0-9_]*\)[ ]*=[ ]*\([^ \.]*\)[ ]*\.\.\.[ ]*\([^ ),]*\)[ ]*)/for (unsigned int \1=\2; \1<\3; \1++) for (unsigned int \4=\5; \4<\6; \4++)/g' $2

sed -i.backup  's/foreach_unique[ ]*([ ]*\([[:alnum:]_]*\)[ ]*in[ ]*\([][[:alnum:]._]*\))/unsigned int \1 = \2;/g' $2

sed -i.backup  's/new[ ]*\([a-zA-Z0-9_]*\)[ ]*\[\([^]]*\)\]/(\1\*) alignedUSMMalloc(\2\*sizeof(\1),16)/g' $2
sed -i.backup  's/delete[ ]*\[[ ]*\][ ]*\([a-zA-Z0-9_.\>\-]*\)/alignedUSMFree(\1)/g' $2

sed -i.backup  's/new[ ]*\([a-zA-Z0-9_]*\)[ ]*;/(\1\*) alignedUSMMalloc(sizeof(\1),16);/g' $2
sed -i.backup  's/delete[ ]*\([a-zA-Z0-9_\>\-]*\)[ ]*;/alignedUSMFree(\1);/g' $2

# embree ray layout
sed -i.backup  's/[.]tnear/.tnear()/g' $2
sed -i.backup  's/[.]time/.time()/g' $2
sed -i.backup  's/[>]tnear/>tnear()/g' $2
sed -i.backup  's/ray->time/ray->time()/g' $2


# system library
sed -i.backup  's/sync;//g' $2
sed -i.backup  's/print(/printf(/g' $2
sed -i.backup  's/abort()/exit(1)/g' $2
sed -i.backup 's/atomic_compare_exchange_global/atomic_cmpxchg/g' $2
sed -i.backup 's/memory_barrier/__memory_barrier/g' $2

# math library
sed -i.backup  's/Vec3f\([^af_]\)/Vec3fa\1/g' $2
sed -i.backup 's/LinearSpace3f\([^fa]\)/LinearSpace3fa\1/g' $2
sed -i.backup 's/AffineSpace3f\([^fa]\)/AffineSpace3fa\1/g' $2

sed -i.backup  's/ = make_Ray//g' $2
sed -i.backup 's/make_LinearSpace3fa_scale/LinearSpace3fa::scale/g' $2
sed -i.backup 's/make_LinearSpace3fa_rotate/LinearSpace3fa::rotate/g' $2
sed -i.backup 's/make_AffineSpace3fa_scale/AffineSpace3fa::scale/g' $2
sed -i.backup 's/make_AffineSpace3fa_rotate/AffineSpace3fa::rotate/g' $2
sed -i.backup 's/make_AffineSpace3fa_translate/AffineSpace3fa::translate/g' $2
sed -i.backup 's/make_LinearSpace3fa_identity()/LinearSpace3fa(one)/g' $2
sed -i.backup 's/make_LinearSpace3fa\([^_]\)/LinearSpace3fa\1/g' $2
sed -i.backup 's/make_Quaternion3f_rotate/Quaternion3f::rotate/g' $2

sed -i.backup  's/M_PI/float(pi)/g' $2
sed -i.backup  's/\*pi\*/\*float(pi)\*/g' $2
sed -i.backup  's/\*pi\//\*float(pi)\//g' $2
sed -i.backup  's/one_over_pi/float(one_over_pi)/g' $2
sed -i.backup  's/one_over_two_pi/float(one_over_two_pi)/g' $2
sed -i.backup  's/one_over_four_pi/float(one_over_four_pi)/g' $2
sed -i.backup  's/(two_pi/(float(two_pi)/g' $2
sed -i.backup  's/[^_(]two_pi/float(two_pi)/g' $2
sed -i.backup  's/make_Vec3f_/Vec3ff/g' $2
sed -i.backup  's/make_Vec3ff/Vec3ff/g' $2
sed -i.backup  's/make_Vec2f/Vec2f/g' $2
sed -i.backup  's/make_Vec2i/Vec2i/g' $2
sed -i.backup  's/make_Vec3f/Vec3f/g' $2
sed -i.backup  's/make_Vec3fa/Vec3fa/g' $2
sed -i.backup  's/make_Vec4f/Vec4f/g' $2
#sed -i.backup  's/make_Sample3f/Sample3f/g' $2
sed -i.backup  's/make_AffineSpace3f/AffineSpace3f/g' $2

sed -i.backup  's/make_Quaternion3f/Quaternion3f/g' $2
sed -i.backup  's/make_AffineSpace3f/AffineSpace3f/g' $2

sed -i.backup 's/sincos/sincosf/g' $2
sed -i.backup 's/ lerp/ lerpr/g' $2
sed -i 's/\/\/ALIGNED_STRUCT/ALIGNED_STRUCT/g' $2

# Embree specific
sed -i.backup  's/RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING/RTC_INTERSECT1/g' $2
sed -i.backup  's/RTC_INTERSECT_UNIFORM/RTC_INTERSECT1/g' $2
sed -i.backup  's/RTC_INTERSECT_VARYING/RTC_INTERSECT1/g' $2
sed -i.backup  's/RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR/RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR/g' $2

sed -i.backup  's/RTCIntersectFuncVarying/RTCIntersectFunc/g' $2
sed -i.backup  's/RTCOccludedFuncVarying/RTCOccludedFunc/g' $2
sed -i.backup  's/RTCFilterFuncVarying/RTCFilterFunc/g' $2

sed -i.backup  's/RTCRay1/RTCRay/g' $2

sed -i.backup  's/rtcIntersectVM/rtcIntersect1M/g' $2
sed -i.backup  's/rtcOccludedVM/rtcOccluded1M/g' $2

#sed -i.backup  's/rtcIntersect1(/rtcIntersect1(sg,/g' $2
#sed -i.backup  's/rtcOccluded1(/rtcOccluded1(sg,/g' $2

sed -i.backup  's/rtcIntersectV(/rtcIntersect1(/g' $2
sed -i.backup  's/rtcOccludedV(/rtcOccluded1(/g' $2

sed -i.backup  's/rtcInterpolateV/rtcInterpolate/g' $2

sed -i.backup  's/Texture_FLOAT32/Texture::FLOAT32/g' $2
sed -i.backup  's/Texture_RGBA8/Texture::RGBA8/g' $2
sed -i.backup  's/Texture_RGB8/Texture::RGB8/g' $2

# to make 32 bit compile happy under Windows
sed -i.backup 's/const Vec3fa defaultValue/const Vec3fa\& defaultValue/g' $2

# to make static analysis happy
sed -i.backup 's/if (all(1 == 0)) continue;//g' $2

sed -i.backup  's/^RENDER_FRAME_FUNCTION_ISPC/RENDER_FRAME_FUNCTION_SYCL/g' $2

# add Embree namespace
ln=`grep -n -E "#include|#pragma" $2 | tail -1 | cut -d: -f1`
mv $2 $2.backup
head -n $ln $2.backup >> $2
echo >> $2
echo 'namespace embree {' >> $2
tail -n +$(($ln+1)) $2.backup >> $2
echo >> $2
echo '} // namespace embree' >> $2
