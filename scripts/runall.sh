#!/bin/bash
./verify
./benchmark
./bvh_builder

./triangle_geometry
./triangle_geometry_ispc

./instanced_geometry 
./instanced_geometry_ispc

./user_geometry
./user_geometry_ispc

./hair_geometry
./hair_geometry_ispc

./subdivision_geometry  
./subdivision_geometry_ispc

./displacement_geometry
./displacement_geometry_ispc

./dynamic_scene
./dynamic_scene_ispc

./intersection_filter
./intersection_filter_ispc

./viewer
./viewer_ispc

./tutorial06
./tutorial06_ispc
