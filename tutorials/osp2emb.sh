#!/bin/bash

echo Converting OSPRay code $1 to Embree code $2
cp $1 $2

sed -i.backup 's/ospray\/math\/vec.ih/..\/math\/vec.isph/g' $2
sed -i.backup 's/Light.ih/light.isph/g' $2
sed -i.backup 's/ospray\/math\/sampling.ih/..\/math\/sampling.isph/g' $2
sed -i.backup 's/ospray\/math\/LinearSpace.ih/..\/math\/linearspace.isph/g' $2
sed -i.backup 's/ospray\/common\/DifferentialGeometry.ih/..\/core\/differential_geometry.isph/g' $2

sed -i.backup 's/vec2f/Vec2f/g' $2
sed -i.backup 's/vec3f/Vec3f/g' $2
sed -i.backup 's/vec4f/Vec4f/g' $2
sed -i.backup 's/linear3f/LinearSpace3f/g' $2
