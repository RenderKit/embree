#!/bin/bash

echo Converting OSPRay code $1 to Embree code $2
cp $1 $2

sed -i 's/ospray\/math\/vec.ih/..\/math\/vec.isph/g' $2
sed -i 's/Light.ih/light.isph/g' $2
sed -i 's/ospray\/math\/sampling.ih/..\/math\/sampling.isph/g' $2
sed -i 's/ospray\/math\/LinearSpace.ih/..\/math\/linearspace.isph/g' $2
sed -i 's/ospray\/common\/DifferentialGeometry.ih/..\/core\/differential_geometry.isph/g' $2

sed -i 's/vec2f/Vec2f/g' $2
sed -i 's/vec3f/Vec3f/g' $2
sed -i 's/vec4f/Vec4f/g' $2
sed -i 's/linear3f/LinearSpace3f/g' $2
