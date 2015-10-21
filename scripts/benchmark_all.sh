#!/bin/bash
../scripts/benchmark.py run1 ${1}.bvh4.triangle4  ../../embree-ref/build/pathtracer -rtcore accel=bvh4.triangle4
../scripts/benchmark.py run1 ${1}.bvh4.triangle4v ../../embree-ref/build/pathtracer -rtcore accel=bvh4.triangle4v
../scripts/benchmark.py run1 ${1}.bvh4.triangle4i ../../embree-ref/build/pathtracer -rtcore accel=bvh4.triangle4i
../scripts/benchmark.py run1 ${1}.bvh8.triangle4  ../../embree-ref/build/pathtracer -rtcore accel=bvh8.triangle4
../scripts/benchmark.py run2 ${1}.bvh4obb.bezier1v   ../../embree-ref/build/hair_geometry -rtcore hair_accel=bvh4obb.bezier1v
../scripts/benchmark.py run2 ${1}.bvh4obb.bezier1i   ../../embree-ref/build/hair_geometry -rtcore hair_accel=bvh4obb.bezier1i

../scripts/benchmark.py run1 ${1}.ispc.bvh4.triangle4  ../../embree-ref/build/pathtracer_ispc -rtcore accel=bvh4.triangle4
../scripts/benchmark.py run1 ${1}.ispc.bvh4.triangle4v ../../embree-ref/build/pathtracer_ispc -rtcore accel=bvh4.triangle4v
../scripts/benchmark.py run1 ${1}.ispc.bvh4.triangle4i ../../embree-ref/build/pathtracer_ispc -rtcore accel=bvh4.triangle4i
../scripts/benchmark.py run1 ${1}.ispc.bvh8.triangle4  ../../embree-ref/build/pathtracer_ispc -rtcore accel=bvh8.triangle4
../scripts/benchmark.py run2 ${1}.ispc.bvh4obb.bezier1v   ../../embree-ref/build/hair_geometry_ispc -rtcore hair_accel=bvh4obb.bezier1v
../scripts/benchmark.py run2 ${1}.ispc.bvh4obb.bezier1i   ../../embree-ref/build/hair_geometry_ispc -rtcore hair_accel=bvh4obb.bezier1i

