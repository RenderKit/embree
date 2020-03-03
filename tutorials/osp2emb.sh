#!/bin/bash

## ======================================================================== ##
## Copyright 2009-2020 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

echo Converting OSPRay code $1 to Embree code $2
cp $1 $2

sed -i.backup 's/math.ih/math.isph/g' $2
sed -i.backup 's/ospray\/math\/vec.ih/..\/math\/vec.isph/g' $2
sed -i.backup 's/Light.ih/light.isph/g' $2
sed -i.backup 's/ospray\/common\/OSPCommon.ih/..\/common.isph/g' $2
sed -i.backup 's/ospray\/math\/sampling.ih/..\/math\/sampling.isph/g' $2
sed -i.backup 's/ospray\/math\/LinearSpace.ih/..\/math\/linearspace.isph/g' $2
sed -i.backup 's/ospray\/common\/DifferentialGeometry.ih/..\/core\/differential_geometry.isph/g' $2
sed -i.backup 's/ospray\/OSPTexture.h/texture.h/g' $2
sed -i.backup 's/Texture2D.ih/texture2d.isph/g' $2

sed -i.backup 's/vec2/Vec2/g' $2
sed -i.backup 's/vec3/Vec3/g' $2
sed -i.backup 's/vec4/Vec4/g' $2
sed -i.backup 's/linear3/LinearSpace3/g' $2

sed -i.backup 's/OSPRay/Embree/g' $2
sed -i.backup 's/OSP_//g' $2
sed -i.backup 's/OSPTexture/Texture/g' $2

sed -i.backup 's/floatbits(0x7F800000)/(1e100f)/g' $2
sed -i.backup 's/floatbits(0xFF800000)/(-1e100f)/g' $2
