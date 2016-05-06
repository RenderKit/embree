
ispc2cpp:
	@./ispc2cpp.sh common/tutorial/differential_geometry.isph common/tutorial/differential_geometry.h
	@./ispc2cpp.sh common/tutorial/sampling.isph common/tutorial/sampling.h
	@./ispc2cpp.sh common/tutorial/optics.isph common/tutorial/optics.h
	@./ispc2cpp.sh common/tutorial/lights/light.isph common/tutorial/lights/light.h
	@./ispc2cpp.sh common/tutorial/lights/light.ispc common/tutorial/lights/light.cpp
	@./ispc2cpp.sh common/tutorial/lights/ambient_light.ispc common/tutorial/lights/ambient_light.cpp
	@./ispc2cpp.sh common/tutorial/lights/directional_light.ispc common/tutorial/lights/directional_light.cpp
	@./ispc2cpp.sh common/tutorial/lights/point_light.ispc common/tutorial/lights/point_light.cpp
	@./ispc2cpp.sh triangle_geometry/triangle_geometry_device.ispc triangle_geometry/triangle_geometry_device.cpp
	@./ispc2cpp.sh dynamic_scene/dynamic_scene_device.ispc dynamic_scene/dynamic_scene_device.cpp
	@./ispc2cpp.sh user_geometry/user_geometry_device.ispc user_geometry/user_geometry_device.cpp
	@./ispc2cpp.sh viewer/viewer_device.ispc viewer/viewer_device.cpp
	@./ispc2cpp.sh viewer_stream/viewer_stream_device.ispc viewer_stream/viewer_stream_device.cpp
	@./ispc2cpp.sh instanced_geometry/instanced_geometry_device.ispc instanced_geometry/instanced_geometry_device.cpp
	@./ispc2cpp.sh intersection_filter/intersection_filter_device.ispc intersection_filter/intersection_filter_device.cpp
	@./ispc2cpp.sh pathtracer/pathtracer_device.ispc pathtracer/pathtracer_device.cpp
	@./ispc2cpp.sh hair_geometry/hair_geometry_device.ispc hair_geometry/hair_geometry_device.cpp
	@./ispc2cpp.sh subdivision_geometry/subdivision_geometry_device.ispc subdivision_geometry/subdivision_geometry_device.cpp
	@./ispc2cpp.sh displacement_geometry/displacement_geometry_device.ispc displacement_geometry/displacement_geometry_device.cpp
	@./ispc2cpp.sh lazy_geometry/lazy_geometry_device.ispc lazy_geometry/lazy_geometry_device.cpp
	@./ispc2cpp.sh motion_blur_geometry/motion_blur_geometry_device.ispc motion_blur_geometry/motion_blur_geometry_device.cpp
	@./ispc2cpp.sh interpolation/interpolation_device.ispc interpolation/interpolation_device.cpp
	@./ispc2cpp.sh curve_geometry/curve_geometry_device.ispc curve_geometry/curve_geometry_device.cpp

ospray:
	@./osp2emb.sh ../../ospray/ospray/math/sampling.ih common/tutorial/sampling.isph
	@./osp2emb.sh ../../ospray/ospray/lights/Light.ih common/tutorial/lights/light.isph
	@./osp2emb.sh ../../ospray/ospray/lights/Light.ispc common/tutorial/lights/light.ispc
	@./osp2emb.sh ../../ospray/ospray/lights/AmbientLight.ispc common/tutorial/lights/ambient_light.ispc
	@./osp2emb.sh ../../ospray/ospray/lights/DirectionalLight.ispc common/tutorial/lights/directional_light.ispc
	@./osp2emb.sh ../../ospray/ospray/lights/PointLight.ispc common/tutorial/lights/point_light.ispc

all: ospray ispc2cpp

