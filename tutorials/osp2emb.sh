#!/bin/bash

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
