## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ispc2cpp: common triangle_geometry point_geometry dynamic_scene user_geometry viewer viewer_stream viewer_anim instanced_geometry intersection_filter pathtracer hair_geometry subdivision_geometry displacement_geometry lazy_geometry motion_blur_geometry interpolation_geometry curve_geometry grid_geometry quaternion_motion_blur

common: FORCE
	@./ispc2cpp.sh common/tutorial/tutorial_device.isph common/tutorial/tutorial_device.h
	@./ispc2cpp.sh common/tutorial/tutorial_device.ispc common/tutorial/tutorial_device.cpp
	@./ispc2cpp.sh common/core/differential_geometry.isph common/core/differential_geometry.h
#	@./ispc2cpp.sh common/math/random_sampler.isph common/math/random_sampler.h # manual optimizations
	@./ispc2cpp.sh common/math/sampling.isph common/math/sampling.h
	@./ispc2cpp.sh common/tutorial/optics.isph common/tutorial/optics.h
	@./ispc2cpp.sh common/lights/light.isph common/lights/light.h
	@./ispc2cpp.sh common/lights/light.ispc common/lights/light.cpp
	@./ispc2cpp.sh common/lights/ambient_light.ispc common/lights/ambient_light.cpp
	@./ispc2cpp.sh common/lights/directional_light.ispc common/lights/directional_light.cpp
	@./ispc2cpp.sh common/lights/point_light.ispc common/lights/point_light.cpp
	@./ispc2cpp.sh common/lights/quad_light.ispc common/lights/quad_light.cpp
	@./ispc2cpp.sh common/lights/spot_light.ispc common/lights/spot_light.cpp
	@./ispc2cpp.sh common/texture/texture2d.isph common/texture/texture2d.h
	@./ispc2cpp.sh common/texture/texture2d.ispc common/texture/texture2d.cpp
	@./ispc2cpp.sh common/texture/texture_param.isph common/texture/texture_param.h

triangle_geometry: FORCE
	@./ispc2cpp.sh triangle_geometry/triangle_geometry_device.isph triangle_geometry/triangle_geometry_device.h
	@./ispc2cpp.sh triangle_geometry/triangle_geometry_device.ispc triangle_geometry/triangle_geometry_device.cpp
point_geometry: FORCE
	@./ispc2cpp.sh point_geometry/point_geometry_device.isph point_geometry/point_geometry_device.h
	@./ispc2cpp.sh point_geometry/point_geometry_device.ispc point_geometry/point_geometry_device.cpp
dynamic_scene: FORCE
	@./ispc2cpp.sh dynamic_scene/dynamic_scene_device.ispc dynamic_scene/dynamic_scene_device.cpp
user_geometry: FORCE
	@./ispc2cpp.sh user_geometry/user_geometry_device.isph user_geometry/user_geometry_device.h
	@./ispc2cpp.sh user_geometry/user_geometry_device.ispc user_geometry/user_geometry_device.cpp
viewer: FORCE
	@./ispc2cpp.sh viewer/viewer_device.isph viewer/viewer_device.h
	@./ispc2cpp.sh viewer/viewer_device.ispc viewer/viewer_device.cpp
viewer_stream: FORCE
	@./ispc2cpp.sh viewer_stream/viewer_stream_device.ispc viewer_stream/viewer_stream_device.cpp
viewer_anim: FORCE
	@./ispc2cpp.sh viewer_anim/viewer_anim_device.ispc viewer_anim/viewer_anim_device.cpp
instanced_geometry: FORCE
	@./ispc2cpp.sh instanced_geometry/instanced_geometry_device.isph instanced_geometry/instanced_geometry_device.h
	@./ispc2cpp.sh instanced_geometry/instanced_geometry_device.ispc instanced_geometry/instanced_geometry_device.cpp
intersection_filter: FORCE
	@./ispc2cpp.sh intersection_filter/intersection_filter_device.isph intersection_filter/intersection_filter_device.h
	@./ispc2cpp.sh intersection_filter/intersection_filter_device.ispc intersection_filter/intersection_filter_device.cpp
pathtracer: FORCE
	@./ispc2cpp.sh pathtracer/pathtracer_device.ispc pathtracer/pathtracer_device.cpp
hair_geometry: FORCE
	@./ispc2cpp.sh hair_geometry/hair_geometry_device.ispc hair_geometry/hair_geometry_device.cpp
subdivision_geometry: FORCE
	@./ispc2cpp.sh subdivision_geometry/subdivision_geometry_device.ispc subdivision_geometry/subdivision_geometry_device.cpp
displacement_geometry: FORCE
	@./ispc2cpp.sh displacement_geometry/displacement_geometry_device.ispc displacement_geometry/displacement_geometry_device.cpp
lazy_geometry: FORCE
	@./ispc2cpp.sh lazy_geometry/lazy_geometry_device.ispc lazy_geometry/lazy_geometry_device.cpp
motion_blur_geometry: FORCE
	@./ispc2cpp.sh motion_blur_geometry/motion_blur_geometry_device.isph motion_blur_geometry/motion_blur_geometry_device.h
	@./ispc2cpp.sh motion_blur_geometry/motion_blur_geometry_device.ispc motion_blur_geometry/motion_blur_geometry_device.cpp
interpolation_geometry: FORCE
	@./ispc2cpp.sh interpolation/interpolation_device.isph interpolation/interpolation_device.h
	@./ispc2cpp.sh interpolation/interpolation_device.ispc interpolation/interpolation_device.cpp
curve_geometry: FORCE
	@./ispc2cpp.sh curve_geometry/curve_geometry_device.ispc curve_geometry/curve_geometry_device.cpp
grid_geometry: FORCE
	@./ispc2cpp.sh grid_geometry/grid_geometry_device.ispc grid_geometry/grid_geometry_device.cpp
quaternion_motion_blur: FORCE
	@./ispc2cpp.sh quaternion_motion_blur/quaternion_motion_blur_device.ispc quaternion_motion_blur/quaternion_motion_blur_device.cpp

osp2emb:
	@./osp2emb.sh ../../ospray/ospray/math/math.ih common/math/math.isph
#	@./osp2emb.sh ../../ospray/ospray/math/vec.ih common/math/vec.isph # requires manual changes
	@./osp2emb.sh ../../ospray/ospray/math/sampling.ih common/math/sampling.isph
	@./osp2emb.sh ../../ospray/ospray/lights/Light.ih common/lights/light.isph
	@./osp2emb.sh ../../ospray/ospray/lights/Light.ispc common/lights/light.ispc
	@./osp2emb.sh ../../ospray/ospray/lights/AmbientLight.ispc common/lights/ambient_light.ispc
	@./osp2emb.sh ../../ospray/ospray/lights/DirectionalLight.ispc common/lights/directional_light.ispc
	@./osp2emb.sh ../../ospray/ospray/lights/PointLight.ispc common/lights/point_light.ispc
	@./osp2emb.sh ../../ospray/ospray/lights/QuadLight.ispc common/lights/quad_light.ispc
	@./osp2emb.sh ../../ospray/ospray/lights/SpotLight.ispc common/lights/spot_light.ispc
	@./osp2emb.sh ../../ospray/ospray/include/ospray/OSPTexture.h common/texture/texture.h
	@./osp2emb.sh ../../ospray/ospray/texture/Texture2D.ih common/texture/texture2d.isph
	@./osp2emb.sh ../../ospray/ospray/texture/Texture2D.ispc common/texture/texture2d.ispc
	@./osp2emb.sh ../../ospray/ospray/texture/TextureParam.ih common/texture/texture_param.isph
	@echo "export void dummy() {} // just to avoid linker warning under MacOSX" >> common/lights/light.ispc

all: osp2emb ispc2cpp

FORCE:
