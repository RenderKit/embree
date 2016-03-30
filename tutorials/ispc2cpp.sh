#!/bin/bash

echo Converting ISPC tutorial $1 to CPP tutorial $2
cp $1 $2

# ISPC language
sed -i.backup  's/.isph\"/.h\"/g' $2
sed -i.backup  's/uniform //g' $2
sed -i.backup  's/ uniform//g' $2
sed -i.backup  's/varying //g' $2
sed -i.backup  's/ varying//g' $2
sed -i.backup  's/unmasked //g' $2
sed -i.backup  's/extern/extern \"C\"/g' $2
sed -i.backup  's/export/extern \"C\"/g' $2

sed -i.backup 's/int8/char/g' $2
sed -i.backup 's/int16/int16_t/g' $2
sed -i.backup 's/int32/int32_t/g' $2

sed -i.backup  's/__mask/1/g' $2
sed -i.backup  's/NULL/nullptr/g' $2

sed -i.backup  's/programIndex/0/g' $2
sed -i.backup  's/task[ ]*void[ ]*\([a-zA-Z0-9_]*\)[ ]*(/void \1 (int taskIndex, /g' $2
sed -i.backup  's/launch\[\([^]]*\)\][ ]*\([a-zA-Z0-9_]*\)[ ]*(\([^)]*\))/parallel_for(size_t(0),size_t(\1),[\&](const range<size_t>\& range) \{\n    for (size_t i=range.begin(); i<range.end(); i++)\n      \2(i,\3);\n  \})/g' $2

sed -i.backup  's/foreach[ ]*([ ]*\([a-zA-Z0-9_]*\)[ ]*=[ ]*\([^ \.]*\)[ ]*\.\.\.[ ]*\([^ ),]*\)[ ]*)/for (int \1=\2; \1<\3; \1++)/g' $2
sed -i.backup  's/foreach_tiled[ ]*([ ]*\([a-zA-Z0-9_]*\)[ ]*=[ ]*\([^ \.]*\)[ ]*\.\.\.[ ]*\([^ ),]*\)[ ]*)/for (int \1=\2; \1<\3; \1++)/g' $2

sed -i.backup  's/foreach[ ]*([ ]*\([a-zA-Z0-9_]*\)[ ]*=[ ]*\([^ \.]*\)[ ]*\.\.\.[ ]*\([^ ),]*\)[ ]*\,[ ]*\([a-zA-Z0-9_]*\)[ ]*=[ ]*\([^ \.]*\)[ ]*\.\.\.[ ]*\([^ ),]*\)[ ]*)/for (int \1=\2; \1<\3; \1++) for (int \4=\5; \4<\6; \4++)/g' $2
sed -i.backup  's/foreach_tiled[ ]*([ ]*\([a-zA-Z0-9_]*\)[ ]*=[ ]*\([^ \.]*\)[ ]*\.\.\.[ ]*\([^ ),]*\)[ ]*\,[ ]*\([a-zA-Z0-9_]*\)[ ]*=[ ]*\([^ \.]*\)[ ]*\.\.\.[ ]*\([^ ),]*\)[ ]*)/for (int \1=\2; \1<\3; \1++) for (int \4=\5; \4<\6; \4++)/g' $2

sed -i.backup  's/foreach_unique[ ]*([ ]*\([[:alnum:]_]*\)[ ]*in[ ]*\([[:alnum:]._]*\))/int \1 = \2;/g' $2

sed -i.backup  's/new[ ]*\([a-zA-Z0-9_]*\)[ ]*\[\([^]]*\)\]/(\1\*) alignedMalloc(\2\*sizeof(\1))/g' $2
sed -i.backup  's/delete[ ]*\[[ ]*\][ ]*\([a-zA-Z0-9_]*\)/alignedFree(\1)/g' $2

# system library
sed -i.backup  's/sync;//g' $2
sed -i.backup  's/print(/printf(/g' $2
sed -i.backup  's/abort()/exit(1)/g' $2
sed -i.backup 's/atomic_compare_exchange_global/atomic_cmpxchg/g' $2 
sed -i.backup 's/memory_barrier/__memory_barrier/g' $2 

# math library
sed -i.backup  's/Vec3f\([^a]\)/Vec3fa\1/g' $2
sed -i.backup 's/LinearSpace3f/LinearSpace3fa/g' $2
sed -i.backup 's/AffineSpace3f\([^a]\)/AffineSpace3fa\1/g' $2

sed -i.backup  's/make_Ray/RTCRay/g' $2
sed -i.backup 's/make_LinearSpace3fa_rotate/LinearSpace3fa::rotate/g' $2
sed -i.backup 's/make_AffineSpace3fa_rotate/AffineSpace3fa::rotate/g' $2

sed -i.backup  's/M_PI/float(pi)/g' $2
sed -i.backup  's/\*pi\*/\*float(pi)\*/g' $2
sed -i.backup  's/\*pi\//\*float(pi)\//g' $2
sed -i.backup  's/one_over_pi/float(one_over_pi)/g' $2
sed -i.backup  's/one_over_two_pi/float(one_over_two_pi)/g' $2
sed -i.backup  's/one_over_four_pi/float(one_over_four_pi)/g' $2
sed -i.backup  's/[^_]two_pi/float(two_pi)/g' $2
sed -i.backup  's/make_Vec2f/Vec2f/g' $2
sed -i.backup  's/make_Vec3f/Vec3f/g' $2
sed -i.backup  's/make_Vec3fa/Vec3fa/g' $2
sed -i.backup  's/make_Sample3f/Sample3f/g' $2
sed -i.backup  's/make_AffineSpace3f/AffineSpace3f/g' $2

# Embree specific
sed -i.backup  's/RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING/RTC_INTERSECT1/g' $2
sed -i.backup  's/RTC_INTERSECT_VARYING/RTC_INTERSECT1/g' $2
sed -i.backup  's/RTC_MATRIX_COLUMN_MAJOR/RTC_MATRIX_COLUMN_MAJOR_ALIGNED16/g' $2
sed -i.backup  's/RTC_MATRIX_COLUMN_MAJOR_ALIGNED16_ALIGNED16/RTC_MATRIX_COLUMN_MAJOR_ALIGNED16/g' $2

sed -i.backup  's/RTCIntersectFuncVarying/RTCIntersectFunc/g' $2
sed -i.backup  's/RTCOccludedFuncVarying/RTCOccludedFunc/g' $2
sed -i.backup  's/RTCFilterFuncVarying/RTCFilterFunc/g' $2







