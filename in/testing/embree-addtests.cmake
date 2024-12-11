ADD_EMBREE_TEST_ECS(prim_curves_bezier_flat_eyelight embree_viewer 
    XML tests/primitives/curves_bezier_flat.xml 
    REFERENCE tests/primitives/curves_bezier_flat.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_flat_uv embree_viewer 
    XML tests/primitives/curves_bezier_flat.xml 
    REFERENCE tests/primitives/curves_bezier_flat.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_flat_Ng embree_viewer 
    XML tests/primitives/curves_bezier_flat.xml 
    REFERENCE tests/primitives/curves_bezier_flat.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_flat_primID embree_viewer 
    XML tests/primitives/curves_bezier_flat.xml 
    REFERENCE tests/primitives/curves_bezier_flat.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_flat_occlusion embree_viewer 
    XML tests/primitives/curves_bezier_flat.xml 
    REFERENCE tests/primitives/curves_bezier_flat.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_flat_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_bezier_flat_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_flat_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_flat_mblur_uv embree_viewer 
    XML tests/primitives/curves_bezier_flat_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_flat_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_flat_mblur_Ng embree_viewer 
    XML tests/primitives/curves_bezier_flat_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_flat_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_flat_mblur_primID embree_viewer 
    XML tests/primitives/curves_bezier_flat_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_flat_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_flat_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_bezier_flat_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_flat_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_normal_oriented_eyelight embree_viewer 
    XML tests/primitives/curves_bezier_normal_oriented.xml 
    REFERENCE tests/primitives/curves_bezier_normal_oriented.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_normal_oriented_uv embree_viewer 
    XML tests/primitives/curves_bezier_normal_oriented.xml 
    REFERENCE tests/primitives/curves_bezier_normal_oriented.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_normal_oriented_Ng embree_viewer 
    XML tests/primitives/curves_bezier_normal_oriented.xml 
    REFERENCE tests/primitives/curves_bezier_normal_oriented.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_normal_oriented_primID embree_viewer 
    XML tests/primitives/curves_bezier_normal_oriented.xml 
    REFERENCE tests/primitives/curves_bezier_normal_oriented.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_normal_oriented_occlusion embree_viewer 
    XML tests/primitives/curves_bezier_normal_oriented.xml 
    REFERENCE tests/primitives/curves_bezier_normal_oriented.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_normal_oriented_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_bezier_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_normal_oriented_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_normal_oriented_mblur_uv embree_viewer 
    XML tests/primitives/curves_bezier_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_normal_oriented_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_normal_oriented_mblur_Ng embree_viewer 
    XML tests/primitives/curves_bezier_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_normal_oriented_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_normal_oriented_mblur_primID embree_viewer 
    XML tests/primitives/curves_bezier_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_normal_oriented_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_normal_oriented_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_bezier_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_normal_oriented_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 60)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_round_eyelight embree_viewer 
    XML tests/primitives/curves_bezier_round.xml 
    REFERENCE tests/primitives/curves_bezier_round.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_round_uv embree_viewer 
    XML tests/primitives/curves_bezier_round.xml 
    REFERENCE tests/primitives/curves_bezier_round.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_round_Ng embree_viewer 
    XML tests/primitives/curves_bezier_round.xml 
    REFERENCE tests/primitives/curves_bezier_round.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_round_primID embree_viewer 
    XML tests/primitives/curves_bezier_round.xml 
    REFERENCE tests/primitives/curves_bezier_round.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_round_occlusion embree_viewer 
    XML tests/primitives/curves_bezier_round.xml 
    REFERENCE tests/primitives/curves_bezier_round.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_round_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_bezier_round_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_round_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_round_mblur_uv embree_viewer 
    XML tests/primitives/curves_bezier_round_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_round_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_round_mblur_Ng embree_viewer 
    XML tests/primitives/curves_bezier_round_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_round_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_round_mblur_primID embree_viewer 
    XML tests/primitives/curves_bezier_round_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_round_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bezier_round_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_bezier_round_mblur.xml 
    REFERENCE tests/primitives/curves_bezier_round_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bezier_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_flat_eyelight embree_viewer 
    XML tests/primitives/curves_bspline_flat.xml 
    REFERENCE tests/primitives/curves_bspline_flat.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_flat_uv embree_viewer 
    XML tests/primitives/curves_bspline_flat.xml 
    REFERENCE tests/primitives/curves_bspline_flat.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_flat_Ng embree_viewer 
    XML tests/primitives/curves_bspline_flat.xml 
    REFERENCE tests/primitives/curves_bspline_flat.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_flat_primID embree_viewer 
    XML tests/primitives/curves_bspline_flat.xml 
    REFERENCE tests/primitives/curves_bspline_flat.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_flat_occlusion embree_viewer 
    XML tests/primitives/curves_bspline_flat.xml 
    REFERENCE tests/primitives/curves_bspline_flat.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_flat_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_bspline_flat_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_flat_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_flat_mblur_uv embree_viewer 
    XML tests/primitives/curves_bspline_flat_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_flat_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_flat_mblur_Ng embree_viewer 
    XML tests/primitives/curves_bspline_flat_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_flat_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_flat_mblur_primID embree_viewer 
    XML tests/primitives/curves_bspline_flat_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_flat_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_flat_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_bspline_flat_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_flat_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_normal_oriented_eyelight embree_viewer 
    XML tests/primitives/curves_bspline_normal_oriented.xml 
    REFERENCE tests/primitives/curves_bspline_normal_oriented.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_normal_oriented_uv embree_viewer 
    XML tests/primitives/curves_bspline_normal_oriented.xml 
    REFERENCE tests/primitives/curves_bspline_normal_oriented.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_normal_oriented_Ng embree_viewer 
    XML tests/primitives/curves_bspline_normal_oriented.xml 
    REFERENCE tests/primitives/curves_bspline_normal_oriented.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_normal_oriented_primID embree_viewer 
    XML tests/primitives/curves_bspline_normal_oriented.xml 
    REFERENCE tests/primitives/curves_bspline_normal_oriented.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_normal_oriented_occlusion embree_viewer 
    XML tests/primitives/curves_bspline_normal_oriented.xml 
    REFERENCE tests/primitives/curves_bspline_normal_oriented.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_normal_oriented_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_bspline_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_normal_oriented_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_normal_oriented_mblur_uv embree_viewer 
    XML tests/primitives/curves_bspline_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_normal_oriented_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_normal_oriented_mblur_Ng embree_viewer 
    XML tests/primitives/curves_bspline_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_normal_oriented_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_normal_oriented_mblur_primID embree_viewer 
    XML tests/primitives/curves_bspline_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_normal_oriented_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_normal_oriented_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_bspline_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_normal_oriented_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_round_eyelight embree_viewer 
    XML tests/primitives/curves_bspline_round.xml 
    REFERENCE tests/primitives/curves_bspline_round.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_round_uv embree_viewer 
    XML tests/primitives/curves_bspline_round.xml 
    REFERENCE tests/primitives/curves_bspline_round.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_round_Ng embree_viewer 
    XML tests/primitives/curves_bspline_round.xml 
    REFERENCE tests/primitives/curves_bspline_round.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_round_primID embree_viewer 
    XML tests/primitives/curves_bspline_round.xml 
    REFERENCE tests/primitives/curves_bspline_round.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_round_occlusion embree_viewer 
    XML tests/primitives/curves_bspline_round.xml 
    REFERENCE tests/primitives/curves_bspline_round.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_round_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_bspline_round_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_round_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_round_mblur_uv embree_viewer 
    XML tests/primitives/curves_bspline_round_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_round_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_round_mblur_Ng embree_viewer 
    XML tests/primitives/curves_bspline_round_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_round_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_round_mblur_primID embree_viewer 
    XML tests/primitives/curves_bspline_round_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_round_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_bspline_round_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_bspline_round_mblur.xml 
    REFERENCE tests/primitives/curves_bspline_round_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_bspline_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_flat_eyelight embree_viewer 
    XML tests/primitives/curves_catmull_rom_flat.xml 
    REFERENCE tests/primitives/curves_catmull_rom_flat.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_flat_uv embree_viewer 
    XML tests/primitives/curves_catmull_rom_flat.xml 
    REFERENCE tests/primitives/curves_catmull_rom_flat.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_flat_Ng embree_viewer 
    XML tests/primitives/curves_catmull_rom_flat.xml 
    REFERENCE tests/primitives/curves_catmull_rom_flat.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_flat_primID embree_viewer 
    XML tests/primitives/curves_catmull_rom_flat.xml 
    REFERENCE tests/primitives/curves_catmull_rom_flat.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_flat_occlusion embree_viewer 
    XML tests/primitives/curves_catmull_rom_flat.xml 
    REFERENCE tests/primitives/curves_catmull_rom_flat.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_flat_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_catmull_rom_flat_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_flat_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_flat_mblur_uv embree_viewer 
    XML tests/primitives/curves_catmull_rom_flat_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_flat_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_flat_mblur_Ng embree_viewer 
    XML tests/primitives/curves_catmull_rom_flat_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_flat_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_flat_mblur_primID embree_viewer 
    XML tests/primitives/curves_catmull_rom_flat_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_flat_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_flat_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_catmull_rom_flat_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_flat_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_normal_oriented_eyelight embree_viewer 
    XML tests/primitives/curves_catmull_rom_normal_oriented.xml 
    REFERENCE tests/primitives/curves_catmull_rom_normal_oriented.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_normal_oriented_uv embree_viewer 
    XML tests/primitives/curves_catmull_rom_normal_oriented.xml 
    REFERENCE tests/primitives/curves_catmull_rom_normal_oriented.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_normal_oriented_Ng embree_viewer 
    XML tests/primitives/curves_catmull_rom_normal_oriented.xml 
    REFERENCE tests/primitives/curves_catmull_rom_normal_oriented.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_normal_oriented_primID embree_viewer 
    XML tests/primitives/curves_catmull_rom_normal_oriented.xml 
    REFERENCE tests/primitives/curves_catmull_rom_normal_oriented.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_normal_oriented_occlusion embree_viewer 
    XML tests/primitives/curves_catmull_rom_normal_oriented.xml 
    REFERENCE tests/primitives/curves_catmull_rom_normal_oriented.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_normal_oriented_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_normal_oriented_mblur_uv embree_viewer 
    XML tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_normal_oriented_mblur_Ng embree_viewer 
    XML tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_normal_oriented_mblur_primID embree_viewer 
    XML tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_normal_oriented_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_round_eyelight embree_viewer 
    XML tests/primitives/curves_catmull_rom_round.xml 
    REFERENCE tests/primitives/curves_catmull_rom_round.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_round_uv embree_viewer 
    XML tests/primitives/curves_catmull_rom_round.xml 
    REFERENCE tests/primitives/curves_catmull_rom_round.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_round_Ng embree_viewer 
    XML tests/primitives/curves_catmull_rom_round.xml 
    REFERENCE tests/primitives/curves_catmull_rom_round.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_round_primID embree_viewer 
    XML tests/primitives/curves_catmull_rom_round.xml 
    REFERENCE tests/primitives/curves_catmull_rom_round.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_round_occlusion embree_viewer 
    XML tests/primitives/curves_catmull_rom_round.xml 
    REFERENCE tests/primitives/curves_catmull_rom_round.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_round_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_catmull_rom_round_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_round_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_round_mblur_uv embree_viewer 
    XML tests/primitives/curves_catmull_rom_round_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_round_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_round_mblur_Ng embree_viewer 
    XML tests/primitives/curves_catmull_rom_round_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_round_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_round_mblur_primID embree_viewer 
    XML tests/primitives/curves_catmull_rom_round_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_round_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_catmull_rom_round_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_catmull_rom_round_mblur.xml 
    REFERENCE tests/primitives/curves_catmull_rom_round_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_catmull_rom_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_flat_eyelight embree_viewer 
    XML tests/primitives/curves_hermite_flat.xml 
    REFERENCE tests/primitives/curves_hermite_flat.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_flat_uv embree_viewer 
    XML tests/primitives/curves_hermite_flat.xml 
    REFERENCE tests/primitives/curves_hermite_flat.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_flat_Ng embree_viewer 
    XML tests/primitives/curves_hermite_flat.xml 
    REFERENCE tests/primitives/curves_hermite_flat.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_flat_primID embree_viewer 
    XML tests/primitives/curves_hermite_flat.xml 
    REFERENCE tests/primitives/curves_hermite_flat.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_flat_occlusion embree_viewer 
    XML tests/primitives/curves_hermite_flat.xml 
    REFERENCE tests/primitives/curves_hermite_flat.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_flat_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_hermite_flat_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_flat_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_flat_mblur_uv embree_viewer 
    XML tests/primitives/curves_hermite_flat_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_flat_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_flat_mblur_Ng embree_viewer 
    XML tests/primitives/curves_hermite_flat_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_flat_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_flat_mblur_primID embree_viewer 
    XML tests/primitives/curves_hermite_flat_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_flat_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_flat_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_hermite_flat_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_flat_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_normal_oriented_eyelight embree_viewer 
    XML tests/primitives/curves_hermite_normal_oriented.xml 
    REFERENCE tests/primitives/curves_hermite_normal_oriented.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_normal_oriented_uv embree_viewer 
    XML tests/primitives/curves_hermite_normal_oriented.xml 
    REFERENCE tests/primitives/curves_hermite_normal_oriented.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_normal_oriented_Ng embree_viewer 
    XML tests/primitives/curves_hermite_normal_oriented.xml 
    REFERENCE tests/primitives/curves_hermite_normal_oriented.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_normal_oriented_primID embree_viewer 
    XML tests/primitives/curves_hermite_normal_oriented.xml 
    REFERENCE tests/primitives/curves_hermite_normal_oriented.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_normal_oriented_occlusion embree_viewer 
    XML tests/primitives/curves_hermite_normal_oriented.xml 
    REFERENCE tests/primitives/curves_hermite_normal_oriented.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_normal_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_normal_oriented_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_hermite_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_normal_oriented_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_normal_oriented_mblur_uv embree_viewer 
    XML tests/primitives/curves_hermite_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_normal_oriented_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_normal_oriented_mblur_Ng embree_viewer 
    XML tests/primitives/curves_hermite_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_normal_oriented_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_normal_oriented_mblur_primID embree_viewer 
    XML tests/primitives/curves_hermite_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_normal_oriented_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_normal_oriented_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_hermite_normal_oriented_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_normal_oriented_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_normal_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_round_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_hermite_round_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_round_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_round_mblur_uv embree_viewer 
    XML tests/primitives/curves_hermite_round_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_round_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_round_mblur_Ng embree_viewer 
    XML tests/primitives/curves_hermite_round_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_round_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_round_mblur_primID embree_viewer 
    XML tests/primitives/curves_hermite_round_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_round_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_hermite_round_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_hermite_round_mblur.xml 
    REFERENCE tests/primitives/curves_hermite_round_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_hermite_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_eyelight embree_viewer 
    XML tests/primitives/curves_linear_cone.xml 
    REFERENCE tests/primitives/curves_linear_cone.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_uv embree_viewer 
    XML tests/primitives/curves_linear_cone.xml 
    REFERENCE tests/primitives/curves_linear_cone.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_Ng embree_viewer 
    XML tests/primitives/curves_linear_cone.xml 
    REFERENCE tests/primitives/curves_linear_cone.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_primID embree_viewer 
    XML tests/primitives/curves_linear_cone.xml 
    REFERENCE tests/primitives/curves_linear_cone.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_occlusion embree_viewer 
    XML tests/primitives/curves_linear_cone.xml 
    REFERENCE tests/primitives/curves_linear_cone.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_far_eyelight embree_viewer 
    XML tests/primitives/curves_linear_cone_far.xml 
    REFERENCE tests/primitives/curves_linear_cone_far.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_far_uv embree_viewer 
    XML tests/primitives/curves_linear_cone_far.xml 
    REFERENCE tests/primitives/curves_linear_cone_far.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_far_Ng embree_viewer 
    XML tests/primitives/curves_linear_cone_far.xml 
    REFERENCE tests/primitives/curves_linear_cone_far.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_far_primID embree_viewer 
    XML tests/primitives/curves_linear_cone_far.xml 
    REFERENCE tests/primitives/curves_linear_cone_far.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_far_occlusion embree_viewer 
    XML tests/primitives/curves_linear_cone_far.xml 
    REFERENCE tests/primitives/curves_linear_cone_far.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_far_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_linear_cone_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_cone_far_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_far_mblur_uv embree_viewer 
    XML tests/primitives/curves_linear_cone_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_cone_far_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_far_mblur_Ng embree_viewer 
    XML tests/primitives/curves_linear_cone_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_cone_far_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_far_mblur_primID embree_viewer 
    XML tests/primitives/curves_linear_cone_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_cone_far_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_far_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_linear_cone_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_cone_far_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_linear_cone_mblur.xml 
    REFERENCE tests/primitives/curves_linear_cone_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_mblur_uv embree_viewer 
    XML tests/primitives/curves_linear_cone_mblur.xml 
    REFERENCE tests/primitives/curves_linear_cone_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_mblur_Ng embree_viewer 
    XML tests/primitives/curves_linear_cone_mblur.xml 
    REFERENCE tests/primitives/curves_linear_cone_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_mblur_primID embree_viewer 
    XML tests/primitives/curves_linear_cone_mblur.xml 
    REFERENCE tests/primitives/curves_linear_cone_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_cone_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_linear_cone_mblur.xml 
    REFERENCE tests/primitives/curves_linear_cone_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_cone_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_eyelight embree_viewer 
    XML tests/primitives/curves_linear_flat.xml 
    REFERENCE tests/primitives/curves_linear_flat.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_uv embree_viewer 
    XML tests/primitives/curves_linear_flat.xml 
    REFERENCE tests/primitives/curves_linear_flat.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_Ng embree_viewer 
    XML tests/primitives/curves_linear_flat.xml 
    REFERENCE tests/primitives/curves_linear_flat.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_primID embree_viewer 
    XML tests/primitives/curves_linear_flat.xml 
    REFERENCE tests/primitives/curves_linear_flat.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_occlusion embree_viewer 
    XML tests/primitives/curves_linear_flat.xml 
    REFERENCE tests/primitives/curves_linear_flat.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_far_eyelight embree_viewer 
    XML tests/primitives/curves_linear_flat_far.xml 
    REFERENCE tests/primitives/curves_linear_flat_far.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_far_uv embree_viewer 
    XML tests/primitives/curves_linear_flat_far.xml 
    REFERENCE tests/primitives/curves_linear_flat_far.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_far_Ng embree_viewer 
    XML tests/primitives/curves_linear_flat_far.xml 
    REFERENCE tests/primitives/curves_linear_flat_far.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_far_primID embree_viewer 
    XML tests/primitives/curves_linear_flat_far.xml 
    REFERENCE tests/primitives/curves_linear_flat_far.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_far_occlusion embree_viewer 
    XML tests/primitives/curves_linear_flat_far.xml 
    REFERENCE tests/primitives/curves_linear_flat_far.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_far_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_linear_flat_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_flat_far_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_far_mblur_uv embree_viewer 
    XML tests/primitives/curves_linear_flat_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_flat_far_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_far_mblur_Ng embree_viewer 
    XML tests/primitives/curves_linear_flat_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_flat_far_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_far_mblur_primID embree_viewer 
    XML tests/primitives/curves_linear_flat_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_flat_far_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_far_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_linear_flat_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_flat_far_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_linear_flat_mblur.xml 
    REFERENCE tests/primitives/curves_linear_flat_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_mblur_uv embree_viewer 
    XML tests/primitives/curves_linear_flat_mblur.xml 
    REFERENCE tests/primitives/curves_linear_flat_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_mblur_Ng embree_viewer 
    XML tests/primitives/curves_linear_flat_mblur.xml 
    REFERENCE tests/primitives/curves_linear_flat_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_mblur_primID embree_viewer 
    XML tests/primitives/curves_linear_flat_mblur.xml 
    REFERENCE tests/primitives/curves_linear_flat_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_flat_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_linear_flat_mblur.xml 
    REFERENCE tests/primitives/curves_linear_flat_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_flat_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_eyelight embree_viewer 
    XML tests/primitives/curves_linear_round.xml 
    REFERENCE tests/primitives/curves_linear_round.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_uv embree_viewer 
    XML tests/primitives/curves_linear_round.xml 
    REFERENCE tests/primitives/curves_linear_round.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_Ng embree_viewer 
    XML tests/primitives/curves_linear_round.xml 
    REFERENCE tests/primitives/curves_linear_round.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_primID embree_viewer 
    XML tests/primitives/curves_linear_round.xml 
    REFERENCE tests/primitives/curves_linear_round.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_occlusion embree_viewer 
    XML tests/primitives/curves_linear_round.xml 
    REFERENCE tests/primitives/curves_linear_round.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_far_eyelight embree_viewer 
    XML tests/primitives/curves_linear_round_far.xml 
    REFERENCE tests/primitives/curves_linear_round_far.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_far_uv embree_viewer 
    XML tests/primitives/curves_linear_round_far.xml 
    REFERENCE tests/primitives/curves_linear_round_far.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_far_Ng embree_viewer 
    XML tests/primitives/curves_linear_round_far.xml 
    REFERENCE tests/primitives/curves_linear_round_far.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_far_primID embree_viewer 
    XML tests/primitives/curves_linear_round_far.xml 
    REFERENCE tests/primitives/curves_linear_round_far.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_far_occlusion embree_viewer 
    XML tests/primitives/curves_linear_round_far.xml 
    REFERENCE tests/primitives/curves_linear_round_far.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_far_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_linear_round_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_round_far_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_far_mblur_uv embree_viewer 
    XML tests/primitives/curves_linear_round_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_round_far_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_far_mblur_Ng embree_viewer 
    XML tests/primitives/curves_linear_round_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_round_far_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_far_mblur_primID embree_viewer 
    XML tests/primitives/curves_linear_round_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_round_far_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_far_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_linear_round_far_mblur.xml 
    REFERENCE tests/primitives/curves_linear_round_far_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_far_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_mblur_eyelight embree_viewer 
    XML tests/primitives/curves_linear_round_mblur.xml 
    REFERENCE tests/primitives/curves_linear_round_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_mblur_uv embree_viewer 
    XML tests/primitives/curves_linear_round_mblur.xml 
    REFERENCE tests/primitives/curves_linear_round_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_mblur_Ng embree_viewer 
    XML tests/primitives/curves_linear_round_mblur.xml 
    REFERENCE tests/primitives/curves_linear_round_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_mblur_primID embree_viewer 
    XML tests/primitives/curves_linear_round_mblur.xml 
    REFERENCE tests/primitives/curves_linear_round_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_curves_linear_round_mblur_occlusion embree_viewer 
    XML tests/primitives/curves_linear_round_mblur.xml 
    REFERENCE tests/primitives/curves_linear_round_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/curves_linear_round_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_grid_eyelight embree_viewer 
    XML tests/primitives/grid.xml 
    REFERENCE tests/primitives/grid.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/grid.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_grid_uv embree_viewer 
    XML tests/primitives/grid.xml 
    REFERENCE tests/primitives/grid.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/grid.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_grid_Ng embree_viewer 
    XML tests/primitives/grid.xml 
    REFERENCE tests/primitives/grid.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/grid.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_grid_primID embree_viewer 
    XML tests/primitives/grid.xml 
    REFERENCE tests/primitives/grid.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/grid.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_grid_occlusion embree_viewer 
    XML tests/primitives/grid.xml 
    REFERENCE tests/primitives/grid.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/grid.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_grid_deform_eyelight embree_viewer 
    XML tests/primitives/grid_deform.xml 
    REFERENCE tests/primitives/grid_deform.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/grid_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_grid_deform_uv embree_viewer 
    XML tests/primitives/grid_deform.xml 
    REFERENCE tests/primitives/grid_deform.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/grid_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_grid_deform_Ng embree_viewer 
    XML tests/primitives/grid_deform.xml 
    REFERENCE tests/primitives/grid_deform.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/grid_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_grid_deform_primID embree_viewer 
    XML tests/primitives/grid_deform.xml 
    REFERENCE tests/primitives/grid_deform.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/grid_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_grid_deform_occlusion embree_viewer 
    XML tests/primitives/grid_deform.xml 
    REFERENCE tests/primitives/grid_deform.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/grid_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_disc_eyelight embree_viewer 
    XML tests/primitives/points_disc.xml 
    REFERENCE tests/primitives/points_disc.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_disc.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_disc_uv embree_viewer 
    XML tests/primitives/points_disc.xml 
    REFERENCE tests/primitives/points_disc.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_disc.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_disc_Ng embree_viewer 
    XML tests/primitives/points_disc.xml 
    REFERENCE tests/primitives/points_disc.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_disc.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_disc_primID embree_viewer 
    XML tests/primitives/points_disc.xml 
    REFERENCE tests/primitives/points_disc.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_disc.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_disc_occlusion embree_viewer 
    XML tests/primitives/points_disc.xml 
    REFERENCE tests/primitives/points_disc.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_disc.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_disc_mblur_eyelight embree_viewer 
    XML tests/primitives/points_disc_mblur.xml 
    REFERENCE tests/primitives/points_disc_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_disc_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_SYCL_MBLUR == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_disc_mblur_uv embree_viewer 
    XML tests/primitives/points_disc_mblur.xml 
    REFERENCE tests/primitives/points_disc_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_disc_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_SYCL_MBLUR == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_disc_mblur_Ng embree_viewer 
    XML tests/primitives/points_disc_mblur.xml 
    REFERENCE tests/primitives/points_disc_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_disc_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_SYCL_MBLUR == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_disc_mblur_primID embree_viewer 
    XML tests/primitives/points_disc_mblur.xml 
    REFERENCE tests/primitives/points_disc_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_disc_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_SYCL_MBLUR == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_disc_mblur_occlusion embree_viewer 
    XML tests/primitives/points_disc_mblur.xml 
    REFERENCE tests/primitives/points_disc_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_disc_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_SYCL_MBLUR == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_oriented_eyelight embree_viewer 
    XML tests/primitives/points_oriented.xml 
    REFERENCE tests/primitives/points_oriented.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_oriented_uv embree_viewer 
    XML tests/primitives/points_oriented.xml 
    REFERENCE tests/primitives/points_oriented.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_oriented_Ng embree_viewer 
    XML tests/primitives/points_oriented.xml 
    REFERENCE tests/primitives/points_oriented.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_oriented_primID embree_viewer 
    XML tests/primitives/points_oriented.xml 
    REFERENCE tests/primitives/points_oriented.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_oriented_occlusion embree_viewer 
    XML tests/primitives/points_oriented.xml 
    REFERENCE tests/primitives/points_oriented.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_oriented.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_oriented_mblur_eyelight embree_viewer 
    XML tests/primitives/points_oriented_mblur.xml 
    REFERENCE tests/primitives/points_oriented_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_oriented_mblur_uv embree_viewer 
    XML tests/primitives/points_oriented_mblur.xml 
    REFERENCE tests/primitives/points_oriented_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_oriented_mblur_Ng embree_viewer 
    XML tests/primitives/points_oriented_mblur.xml 
    REFERENCE tests/primitives/points_oriented_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_oriented_mblur_primID embree_viewer 
    XML tests/primitives/points_oriented_mblur.xml 
    REFERENCE tests/primitives/points_oriented_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_oriented_mblur_occlusion embree_viewer 
    XML tests/primitives/points_oriented_mblur.xml 
    REFERENCE tests/primitives/points_oriented_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_oriented_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_sphere_eyelight embree_viewer 
    XML tests/primitives/points_sphere.xml 
    REFERENCE tests/primitives/points_sphere.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_sphere.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_sphere_uv embree_viewer 
    XML tests/primitives/points_sphere.xml 
    REFERENCE tests/primitives/points_sphere.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_sphere.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_sphere_Ng embree_viewer 
    XML tests/primitives/points_sphere.xml 
    REFERENCE tests/primitives/points_sphere.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_sphere.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_sphere_primID embree_viewer 
    XML tests/primitives/points_sphere.xml 
    REFERENCE tests/primitives/points_sphere.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_sphere.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_sphere_occlusion embree_viewer 
    XML tests/primitives/points_sphere.xml 
    REFERENCE tests/primitives/points_sphere.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_sphere.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_sphere_mblur_eyelight embree_viewer 
    XML tests/primitives/points_sphere_mblur.xml 
    REFERENCE tests/primitives/points_sphere_mblur.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_sphere_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_sphere_mblur_uv embree_viewer 
    XML tests/primitives/points_sphere_mblur.xml 
    REFERENCE tests/primitives/points_sphere_mblur.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_sphere_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_sphere_mblur_Ng embree_viewer 
    XML tests/primitives/points_sphere_mblur.xml 
    REFERENCE tests/primitives/points_sphere_mblur.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_sphere_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_sphere_mblur_primID embree_viewer 
    XML tests/primitives/points_sphere_mblur.xml 
    REFERENCE tests/primitives/points_sphere_mblur.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_sphere_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_points_sphere_mblur_occlusion embree_viewer 
    XML tests/primitives/points_sphere_mblur.xml 
    REFERENCE tests/primitives/points_sphere_mblur.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/points_sphere_mblur.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_eyelight embree_viewer 
    XML tests/primitives/quad.xml 
    REFERENCE tests/primitives/quad.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_uv embree_viewer 
    XML tests/primitives/quad.xml 
    REFERENCE tests/primitives/quad.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_Ng embree_viewer 
    XML tests/primitives/quad.xml 
    REFERENCE tests/primitives/quad.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_primID embree_viewer 
    XML tests/primitives/quad.xml 
    REFERENCE tests/primitives/quad.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_occlusion embree_viewer 
    XML tests/primitives/quad.xml 
    REFERENCE tests/primitives/quad.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_deform_eyelight embree_viewer 
    XML tests/primitives/quad_deform.xml 
    REFERENCE tests/primitives/quad_deform.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_deform_uv embree_viewer 
    XML tests/primitives/quad_deform.xml 
    REFERENCE tests/primitives/quad_deform.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_deform_Ng embree_viewer 
    XML tests/primitives/quad_deform.xml 
    REFERENCE tests/primitives/quad_deform.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_deform_primID embree_viewer 
    XML tests/primitives/quad_deform.xml 
    REFERENCE tests/primitives/quad_deform.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_deform_occlusion embree_viewer 
    XML tests/primitives/quad_deform.xml 
    REFERENCE tests/primitives/quad_deform.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_deform_far_eyelight embree_viewer 
    XML tests/primitives/quad_deform_far.xml 
    REFERENCE tests/primitives/quad_deform_far.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_deform_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_deform_far_uv embree_viewer 
    XML tests/primitives/quad_deform_far.xml 
    REFERENCE tests/primitives/quad_deform_far.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_deform_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_deform_far_Ng embree_viewer 
    XML tests/primitives/quad_deform_far.xml 
    REFERENCE tests/primitives/quad_deform_far.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_deform_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_deform_far_primID embree_viewer 
    XML tests/primitives/quad_deform_far.xml 
    REFERENCE tests/primitives/quad_deform_far.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_deform_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_deform_far_occlusion embree_viewer 
    XML tests/primitives/quad_deform_far.xml 
    REFERENCE tests/primitives/quad_deform_far.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_deform_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_far_eyelight embree_viewer 
    XML tests/primitives/quad_far.xml 
    REFERENCE tests/primitives/quad_far.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_far_uv embree_viewer 
    XML tests/primitives/quad_far.xml 
    REFERENCE tests/primitives/quad_far.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_far_Ng embree_viewer 
    XML tests/primitives/quad_far.xml 
    REFERENCE tests/primitives/quad_far.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_far_primID embree_viewer 
    XML tests/primitives/quad_far.xml 
    REFERENCE tests/primitives/quad_far.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_quad_far_occlusion embree_viewer 
    XML tests/primitives/quad_far.xml 
    REFERENCE tests/primitives/quad_far.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/quad_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_quad == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_subdiv_eyelight embree_viewer 
    XML tests/primitives/subdiv.xml 
    REFERENCE tests/primitives/subdiv.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/subdiv.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_subdiv_uv embree_viewer 
    XML tests/primitives/subdiv.xml 
    REFERENCE tests/primitives/subdiv.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/subdiv.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_subdiv_Ng embree_viewer 
    XML tests/primitives/subdiv.xml 
    REFERENCE tests/primitives/subdiv.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/subdiv.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_subdiv_primID embree_viewer 
    XML tests/primitives/subdiv.xml 
    REFERENCE tests/primitives/subdiv.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/subdiv.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_subdiv_occlusion embree_viewer 
    XML tests/primitives/subdiv.xml 
    REFERENCE tests/primitives/subdiv.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/subdiv.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_subdiv_deform_eyelight embree_viewer 
    XML tests/primitives/subdiv_deform.xml 
    REFERENCE tests/primitives/subdiv_deform.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/subdiv_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_subdiv_deform_uv embree_viewer 
    XML tests/primitives/subdiv_deform.xml 
    REFERENCE tests/primitives/subdiv_deform.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/subdiv_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_subdiv_deform_Ng embree_viewer 
    XML tests/primitives/subdiv_deform.xml 
    REFERENCE tests/primitives/subdiv_deform.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/subdiv_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_subdiv_deform_primID embree_viewer 
    XML tests/primitives/subdiv_deform.xml 
    REFERENCE tests/primitives/subdiv_deform.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/subdiv_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_subdiv_deform_occlusion embree_viewer 
    XML tests/primitives/subdiv_deform.xml 
    REFERENCE tests/primitives/subdiv_deform.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/subdiv_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_eyelight embree_viewer 
    XML tests/primitives/triangle.xml 
    REFERENCE tests/primitives/triangle.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_uv embree_viewer 
    XML tests/primitives/triangle.xml 
    REFERENCE tests/primitives/triangle.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_Ng embree_viewer 
    XML tests/primitives/triangle.xml 
    REFERENCE tests/primitives/triangle.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_primID embree_viewer 
    XML tests/primitives/triangle.xml 
    REFERENCE tests/primitives/triangle.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_occlusion embree_viewer 
    XML tests/primitives/triangle.xml 
    REFERENCE tests/primitives/triangle.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_deform_eyelight embree_viewer 
    XML tests/primitives/triangle_deform.xml 
    REFERENCE tests/primitives/triangle_deform.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_deform_uv embree_viewer 
    XML tests/primitives/triangle_deform.xml 
    REFERENCE tests/primitives/triangle_deform.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_deform_Ng embree_viewer 
    XML tests/primitives/triangle_deform.xml 
    REFERENCE tests/primitives/triangle_deform.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_deform_primID embree_viewer 
    XML tests/primitives/triangle_deform.xml 
    REFERENCE tests/primitives/triangle_deform.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_deform_occlusion embree_viewer 
    XML tests/primitives/triangle_deform.xml 
    REFERENCE tests/primitives/triangle_deform.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_deform_far_eyelight embree_viewer 
    XML tests/primitives/triangle_deform_far.xml 
    REFERENCE tests/primitives/triangle_deform_far.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_deform_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_deform_far_uv embree_viewer 
    XML tests/primitives/triangle_deform_far.xml 
    REFERENCE tests/primitives/triangle_deform_far.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_deform_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_deform_far_Ng embree_viewer 
    XML tests/primitives/triangle_deform_far.xml 
    REFERENCE tests/primitives/triangle_deform_far.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_deform_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_deform_far_primID embree_viewer 
    XML tests/primitives/triangle_deform_far.xml 
    REFERENCE tests/primitives/triangle_deform_far.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_deform_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_deform_far_occlusion embree_viewer 
    XML tests/primitives/triangle_deform_far.xml 
    REFERENCE tests/primitives/triangle_deform_far.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_deform_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_degenerate_deform_eyelight embree_viewer 
    XML tests/primitives/triangle_degenerate_deform.xml 
    REFERENCE tests/primitives/triangle_degenerate_deform.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_degenerate_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_degenerate_deform_uv embree_viewer 
    XML tests/primitives/triangle_degenerate_deform.xml 
    REFERENCE tests/primitives/triangle_degenerate_deform.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_degenerate_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_degenerate_deform_Ng embree_viewer 
    XML tests/primitives/triangle_degenerate_deform.xml 
    REFERENCE tests/primitives/triangle_degenerate_deform.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_degenerate_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_degenerate_deform_primID embree_viewer 
    XML tests/primitives/triangle_degenerate_deform.xml 
    REFERENCE tests/primitives/triangle_degenerate_deform.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_degenerate_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_degenerate_deform_occlusion embree_viewer 
    XML tests/primitives/triangle_degenerate_deform.xml 
    REFERENCE tests/primitives/triangle_degenerate_deform.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_degenerate_deform.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_far_eyelight embree_viewer 
    XML tests/primitives/triangle_far.xml 
    REFERENCE tests/primitives/triangle_far.xml.embree_viewer_eyelight.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader eyelight --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_far_uv embree_viewer 
    XML tests/primitives/triangle_far.xml 
    REFERENCE tests/primitives/triangle_far.xml.embree_viewer_uv.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader uv --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_far_Ng embree_viewer 
    XML tests/primitives/triangle_far.xml 
    REFERENCE tests/primitives/triangle_far.xml.embree_viewer_Ng.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader Ng --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_far_primID embree_viewer 
    XML tests/primitives/triangle_far.xml 
    REFERENCE tests/primitives/triangle_far.xml.embree_viewer_primID.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader primID --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(prim_triangle_far_occlusion embree_viewer 
    XML tests/primitives/triangle_far.xml 
    REFERENCE tests/primitives/triangle_far.xml.embree_viewer_occlusion.exr 
    INTENSITY 1 
    CONDITION_FILE tests/primitives/triangle_far.xml.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --shader occlusion --time 0.5 --compare-threshold 55)

ADD_EMBREE_TEST_ECS(embree_verify embree_verify 
    NO_REFERENCE 
    INTENSITY 1 
    NO_ISPC 
    NO_SYCL 
    CONDITION_FILE tutorials/verify/embree_verify.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_QUAD == ON" "EMBREE_GEOMETRY_USER == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --no-colors)

SET_EMBREE_TEST_PROPERTIES(embree_verify PROPERTIES;TIMEOUT;7000) 
ADD_EMBREE_TEST_ECS(embree_verify_i2 embree_verify 
    NO_REFERENCE 
    INTENSITY 2 
    NO_ISPC 
    NO_SYCL 
    CONDITION_FILE tutorials/verify/embree_verify_i2.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --no-colors --intensity 2)

SET_EMBREE_TEST_PROPERTIES(embree_verify_i2 PROPERTIES;TIMEOUT;7000) 
ADD_EMBREE_TEST_ECS(embree_verify_memcheck embree_verify 
    NO_REFERENCE 
    INTENSITY 2 
    MEMCHECK 
    NO_ISPC 
    NO_SYCL 
    CONDITION_FILE tutorials/verify/embree_verify_memcheck.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_TESTING_MEMCHECK == ON" 
    ARGS  --no-colors --intensity 0.1 --skip .*memory_consumption.* --skip .*regression_.*_build_join --skip .*SSE4.* --skip .*AVX.* --skip .*AVX2.* --skip .*AVX512.* MEMCHECK)

SET_EMBREE_TEST_PROPERTIES(embree_verify_memcheck PROPERTIES;TIMEOUT;15000) 
ADD_EMBREE_TEST_ECS(embree_verify_benchmark embree_verify 
    NO_REFERENCE 
    INTENSITY 2 
    NO_ISPC 
    NO_SYCL 
    CONDITION_FILE tutorials/verify/embree_verify_benchmark.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_TESTING_BENCHMARK == ON" 
    ARGS  --no-colors --cdash --benchmark-tolerance 0.05 --database "/home/dopitz/embree/build" --run .*benchmarks.* --skip .*_120.* --skip .*_1k.* --skip .*_10k.* --skip .*100k.* --run .*embree_reported_memory.*)

SET_EMBREE_TEST_PROPERTIES(embree_verify_benchmarks PROPERTIES;TIMEOUT;10800) 
ADD_EMBREE_TEST_ECS(triangle_geometry embree_triangle_geometry 
    REFERENCE tutorials/triangle_geometry/triangle_geometry.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/triangle_geometry/triangle_geometry.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --frames 128 --compare-threshold 40)

ADD_EMBREE_TEST_ECS(dynamic_scene embree_dynamic_scene 
    REFERENCE tutorials/dynamic_scene/dynamic_scene.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/dynamic_scene/dynamic_scene.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(voronoi embree_voronoi 
    REFERENCE tutorials/voronoi/voronoi.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/voronoi/voronoi.embree_options 
    CONDITION  "EMBREE_GEOMETRY_USER == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(closest_point embree_closest_point 
    REFERENCE tutorials/closest_point/closest_point.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/closest_point/closest_point.embree_options 
    CONDITION  "EMBREE_GEOMETRY_USER == ON" "EMBREE_GEOMETRY_INSTANCE == ON" "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(user_geometry embree_user_geometry 
    REFERENCE tutorials/user_geometry/user_geometry.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/user_geometry/user_geometry.embree_options 
    CONDITION  "EMBREE_GEOMETRY_USER == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_curve0.ecs embree_viewer 
    ECS tests/models/curves/curve0.ecs 
    REFERENCE tests/models/curves/curve0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_curve1.ecs embree_viewer 
    ECS tests/models/curves/curve1.ecs 
    REFERENCE tests/models/curves/curve1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_curve2.ecs embree_viewer 
    ECS tests/models/curves/curve2.ecs 
    REFERENCE tests/models/curves/curve2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_curve3.ecs embree_viewer 
    ECS tests/models/curves/curve3.ecs 
    REFERENCE tests/models/curves/curve3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_curve4.ecs embree_viewer 
    ECS tests/models/curves/curve4.ecs 
    REFERENCE tests/models/curves/curve4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_curve5.ecs embree_viewer 
    ECS tests/models/curves/curve5.ecs 
    REFERENCE tests/models/curves/curve5.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve5.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_hair0.ecs embree_viewer 
    ECS tests/models/curves/hair0.ecs 
    REFERENCE tests/models/curves/hair0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/hair0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_hair1.ecs embree_viewer 
    ECS tests/models/curves/hair1.ecs 
    REFERENCE tests/models/curves/hair1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/hair1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_oriented_curve0.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve0.ecs 
    REFERENCE tests/models/curves/oriented_curve0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_oriented_curve1.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve1.ecs 
    REFERENCE tests/models/curves/oriented_curve1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_oriented_curve2.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve2.ecs 
    REFERENCE tests/models/curves/oriented_curve2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_oriented_curve3.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve3.ecs 
    REFERENCE tests/models/curves/oriented_curve3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_oriented_curve4.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve4.ecs 
    REFERENCE tests/models/curves/oriented_curve4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_oriented_bspline_curve_twisted.ecs embree_viewer 
    ECS tests/models/curves/oriented_bspline_curve_twisted.ecs 
    REFERENCE tests/models/curves/oriented_bspline_curve_twisted.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_bspline_curve_twisted.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_oriented_hermite_curve_twisted.ecs embree_viewer 
    ECS tests/models/curves/oriented_hermite_curve_twisted.ecs 
    REFERENCE tests/models/curves/oriented_hermite_curve_twisted.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_hermite_curve_twisted.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_round_line_segments_0.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_0.ecs 
    REFERENCE tests/models/curves/round_line_segments_0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/round_line_segments_0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_round_line_segments_1.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_1.ecs 
    REFERENCE tests/models/curves/round_line_segments_1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_round_line_segments_2.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_2.ecs 
    REFERENCE tests/models/curves/round_line_segments_2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_round_line_segments_3.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_3.ecs 
    REFERENCE tests/models/curves/round_line_segments_3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_round_line_segments_4.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_4.ecs 
    REFERENCE tests/models/curves/round_line_segments_4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_round_line_segments_5.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_5.ecs 
    REFERENCE tests/models/curves/round_line_segments_5.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_round_line_segments_6.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_6.ecs 
    REFERENCE tests/models/curves/round_line_segments_6.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_round_line_segments_7.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_7.ecs 
    REFERENCE tests/models/curves/round_line_segments_7.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_round_line_segments_8.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_8.ecs 
    REFERENCE tests/models/curves/round_line_segments_8.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_round_line_segments_9.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_9.ecs 
    REFERENCE tests/models/curves/round_line_segments_9.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/round_line_segments_9.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_min_width_flat_linear_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_flat_linear_curves.ecs 
    REFERENCE tests/models/curves/min_width_flat_linear_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_flat_linear_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_min_width_round_linear_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_round_linear_curves.ecs 
    REFERENCE tests/models/curves/min_width_round_linear_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_round_linear_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_min_width_flat_bezier_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_flat_bezier_curves.ecs 
    REFERENCE tests/models/curves/min_width_flat_bezier_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_flat_bezier_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_min_width_round_bezier_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_round_bezier_curves.ecs 
    REFERENCE tests/models/curves/min_width_round_bezier_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_round_bezier_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_curves_min_width_oriented_hermite_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_oriented_hermite_curves.ecs 
    REFERENCE tests/models/curves/min_width_oriented_hermite_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_oriented_hermite_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_furball_bezier_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_bezier_flat.ecs 
    REFERENCE tests/models/furball/furball_bezier_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bezier_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_furball_bezier_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_bezier_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_bezier_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bezier_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_furball_bezier_round.ecs embree_viewer 
    ECS tests/models/furball/furball_bezier_round.ecs 
    REFERENCE tests/models/furball/furball_bezier_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bezier_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_furball_bspline_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_bspline_flat.ecs 
    REFERENCE tests/models/furball/furball_bspline_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bspline_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_furball_bspline_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_bspline_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_bspline_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bspline_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_furball_bspline_round.ecs embree_viewer 
    ECS tests/models/furball/furball_bspline_round.ecs 
    REFERENCE tests/models/furball/furball_bspline_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bspline_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_furball_hermite_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_hermite_flat.ecs 
    REFERENCE tests/models/furball/furball_hermite_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_hermite_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_furball_hermite_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_hermite_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_hermite_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_hermite_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_furball_catmulrom_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_catmulrom_flat.ecs 
    REFERENCE tests/models/furball/furball_catmulrom_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_catmulrom_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_furball_catmulrom_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_catmulrom_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_catmulrom_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_catmulrom_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_furball_catmulrom_round.ecs embree_viewer 
    ECS tests/models/furball/furball_catmulrom_round.ecs 
    REFERENCE tests/models/furball/furball_catmulrom_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_catmulrom_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_furball_linear_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_linear_flat.ecs 
    REFERENCE tests/models/furball/furball_linear_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_linear_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_furball_linear_round.ecs embree_viewer 
    ECS tests/models/furball/furball_linear_round.ecs 
    REFERENCE tests/models/furball/furball_linear_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_linear_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_spheres_grids.ecs embree_viewer 
    ECS tests/models/furball/spheres_grids.ecs 
    REFERENCE tests/models/furball/spheres_grids.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_grids.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_spheres_points.ecs embree_viewer 
    ECS tests/models/furball/spheres_points.ecs 
    REFERENCE tests/models/furball/spheres_points.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_points.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_spheres_quads.ecs embree_viewer 
    ECS tests/models/furball/spheres_quads.ecs 
    REFERENCE tests/models/furball/spheres_quads.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_quads.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_furball_spheres_triangles.ecs embree_viewer 
    ECS tests/models/furball/spheres_triangles.ecs 
    REFERENCE tests/models/furball/spheres_triangles.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_triangles.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_curves_msmblur.ecs embree_viewer 
    ECS tests/models/msmblur/curves_msmblur.ecs 
    REFERENCE tests/models/msmblur/curves_msmblur.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/curves_msmblur.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_curves_msmblur2.ecs embree_viewer 
    ECS tests/models/msmblur/curves_msmblur2.ecs 
    REFERENCE tests/models/msmblur/curves_msmblur2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/curves_msmblur2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_lines_msmblur.ecs embree_viewer 
    ECS tests/models/msmblur/lines_msmblur.ecs 
    REFERENCE tests/models/msmblur/lines_msmblur.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/lines_msmblur.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_lines_msmblur2.ecs embree_viewer 
    ECS tests/models/msmblur/lines_msmblur2.ecs 
    REFERENCE tests/models/msmblur/lines_msmblur2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/lines_msmblur2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_mblur_time_range_triangle.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_triangle.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_triangle.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_mblur_time_range_quad.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_quad.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_quad.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_quad.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_mblur_time_range_grid.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_grid.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_grid.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_grid.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_mblur_time_range_curve.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_curve.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_curve.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_curve.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_mblur_time_range_line.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_line.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_line.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_line.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_mblur_time_range_instancing.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_instancing.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_instancing.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_instancing.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_mblur_time_range_sphere.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_sphere.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_sphere.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_sphere.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_mblur_time_range_disc.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_disc.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_disc.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_disc.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_mblur_time_range_oriented_disc.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_oriented_disc.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_oriented_disc.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_oriented_disc.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_msmblur_linear_instance_linear_triangle.ecs embree_viewer 
    ECS tests/models/msmblur/linear_instance_linear_triangle.ecs 
    REFERENCE tests/models/msmblur/linear_instance_linear_triangle.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/linear_instance_linear_triangle.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_points_points.ecs embree_viewer 
    ECS tests/models/points/points.ecs 
    REFERENCE tests/models/points/points.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/points.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_points_min_width_spheres.ecs embree_viewer 
    ECS tests/models/points/min_width_spheres.ecs 
    REFERENCE tests/models/points/min_width_spheres.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/min_width_spheres.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_points_min_width_discs.ecs embree_viewer 
    ECS tests/models/points/min_width_discs.ecs 
    REFERENCE tests/models/points/min_width_discs.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/min_width_discs.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_points_min_width_oriented_discs.ecs embree_viewer 
    ECS tests/models/points/min_width_oriented_discs.ecs 
    REFERENCE tests/models/points/min_width_oriented_discs.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/min_width_oriented_discs.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_quaternion_motion_blur_quaternion_quad.ecs embree_viewer 
    ECS tests/models/quaternion_motion_blur/quaternion_quad.ecs 
    REFERENCE tests/models/quaternion_motion_blur/quaternion_quad.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/quaternion_motion_blur/quaternion_quad.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv0.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv0.ecs 
    REFERENCE tests/models/subdiv/subdiv0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv1.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv1.ecs 
    REFERENCE tests/models/subdiv/subdiv1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv3.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv3.ecs 
    REFERENCE tests/models/subdiv/subdiv3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv4.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv4.ecs 
    REFERENCE tests/models/subdiv/subdiv4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv5.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv5.ecs 
    REFERENCE tests/models/subdiv/subdiv5.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv5.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv6.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv6.ecs 
    REFERENCE tests/models/subdiv/subdiv6.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv6.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv7.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv7.ecs 
    REFERENCE tests/models/subdiv/subdiv7.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv7.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv8.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv8.ecs 
    REFERENCE tests/models/subdiv/subdiv8.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv8.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv9.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv9.ecs 
    REFERENCE tests/models/subdiv/subdiv9.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv9.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv_no_boundary.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_no_boundary.ecs 
    REFERENCE tests/models/subdiv/subdiv_no_boundary.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_no_boundary.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv_smooth_boundary.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_smooth_boundary.ecs 
    REFERENCE tests/models/subdiv/subdiv_smooth_boundary.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_smooth_boundary.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv_pin_corners.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_pin_corners.ecs 
    REFERENCE tests/models/subdiv/subdiv_pin_corners.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_pin_corners.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv_pin_boundary.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_pin_boundary.ecs 
    REFERENCE tests/models/subdiv/subdiv_pin_boundary.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_pin_boundary.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_subdiv_pin_all.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_pin_all.ecs 
    REFERENCE tests/models/subdiv/subdiv_pin_all.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_pin_all.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_models_subdiv_cylinder.ecs embree_viewer 
    ECS tests/models/subdiv/cylinder.ecs 
    REFERENCE tests/models/subdiv/cylinder.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/cylinder.ecs.embree_options 
    CONDITION  "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_curve0.ecs embree_viewer 
    ECS tests/models/curves/curve0.ecs 
    REFERENCE tests/models/curves/curve0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_curve1.ecs embree_viewer 
    ECS tests/models/curves/curve1.ecs 
    REFERENCE tests/models/curves/curve1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_curve2.ecs embree_viewer 
    ECS tests/models/curves/curve2.ecs 
    REFERENCE tests/models/curves/curve2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_curve3.ecs embree_viewer 
    ECS tests/models/curves/curve3.ecs 
    REFERENCE tests/models/curves/curve3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_curve4.ecs embree_viewer 
    ECS tests/models/curves/curve4.ecs 
    REFERENCE tests/models/curves/curve4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_curve5.ecs embree_viewer 
    ECS tests/models/curves/curve5.ecs 
    REFERENCE tests/models/curves/curve5.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve5.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_hair0.ecs embree_viewer 
    ECS tests/models/curves/hair0.ecs 
    REFERENCE tests/models/curves/hair0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/hair0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_hair1.ecs embree_viewer 
    ECS tests/models/curves/hair1.ecs 
    REFERENCE tests/models/curves/hair1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/hair1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_oriented_curve0.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve0.ecs 
    REFERENCE tests/models/curves/oriented_curve0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_oriented_curve1.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve1.ecs 
    REFERENCE tests/models/curves/oriented_curve1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_oriented_curve2.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve2.ecs 
    REFERENCE tests/models/curves/oriented_curve2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_oriented_curve3.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve3.ecs 
    REFERENCE tests/models/curves/oriented_curve3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_oriented_curve4.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve4.ecs 
    REFERENCE tests/models/curves/oriented_curve4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_oriented_bspline_curve_twisted.ecs embree_viewer 
    ECS tests/models/curves/oriented_bspline_curve_twisted.ecs 
    REFERENCE tests/models/curves/oriented_bspline_curve_twisted.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_bspline_curve_twisted.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_oriented_hermite_curve_twisted.ecs embree_viewer 
    ECS tests/models/curves/oriented_hermite_curve_twisted.ecs 
    REFERENCE tests/models/curves/oriented_hermite_curve_twisted.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_hermite_curve_twisted.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_round_line_segments_0.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_0.ecs 
    REFERENCE tests/models/curves/round_line_segments_0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/round_line_segments_0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_round_line_segments_1.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_1.ecs 
    REFERENCE tests/models/curves/round_line_segments_1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_round_line_segments_2.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_2.ecs 
    REFERENCE tests/models/curves/round_line_segments_2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_round_line_segments_3.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_3.ecs 
    REFERENCE tests/models/curves/round_line_segments_3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_round_line_segments_4.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_4.ecs 
    REFERENCE tests/models/curves/round_line_segments_4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_round_line_segments_5.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_5.ecs 
    REFERENCE tests/models/curves/round_line_segments_5.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_round_line_segments_6.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_6.ecs 
    REFERENCE tests/models/curves/round_line_segments_6.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_round_line_segments_7.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_7.ecs 
    REFERENCE tests/models/curves/round_line_segments_7.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_round_line_segments_8.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_8.ecs 
    REFERENCE tests/models/curves/round_line_segments_8.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_round_line_segments_9.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_9.ecs 
    REFERENCE tests/models/curves/round_line_segments_9.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/round_line_segments_9.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_min_width_flat_linear_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_flat_linear_curves.ecs 
    REFERENCE tests/models/curves/min_width_flat_linear_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_flat_linear_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_min_width_round_linear_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_round_linear_curves.ecs 
    REFERENCE tests/models/curves/min_width_round_linear_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_round_linear_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_min_width_flat_bezier_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_flat_bezier_curves.ecs 
    REFERENCE tests/models/curves/min_width_flat_bezier_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_flat_bezier_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_min_width_round_bezier_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_round_bezier_curves.ecs 
    REFERENCE tests/models/curves/min_width_round_bezier_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_round_bezier_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_curves_min_width_oriented_hermite_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_oriented_hermite_curves.ecs 
    REFERENCE tests/models/curves/min_width_oriented_hermite_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_oriented_hermite_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_furball_bezier_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_bezier_flat.ecs 
    REFERENCE tests/models/furball/furball_bezier_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bezier_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_furball_bezier_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_bezier_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_bezier_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bezier_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_furball_bezier_round.ecs embree_viewer 
    ECS tests/models/furball/furball_bezier_round.ecs 
    REFERENCE tests/models/furball/furball_bezier_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bezier_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_furball_bspline_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_bspline_flat.ecs 
    REFERENCE tests/models/furball/furball_bspline_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bspline_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_furball_bspline_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_bspline_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_bspline_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bspline_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_furball_bspline_round.ecs embree_viewer 
    ECS tests/models/furball/furball_bspline_round.ecs 
    REFERENCE tests/models/furball/furball_bspline_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bspline_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_furball_hermite_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_hermite_flat.ecs 
    REFERENCE tests/models/furball/furball_hermite_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_hermite_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_furball_hermite_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_hermite_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_hermite_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_hermite_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_furball_catmulrom_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_catmulrom_flat.ecs 
    REFERENCE tests/models/furball/furball_catmulrom_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_catmulrom_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_furball_catmulrom_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_catmulrom_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_catmulrom_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_catmulrom_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_furball_catmulrom_round.ecs embree_viewer 
    ECS tests/models/furball/furball_catmulrom_round.ecs 
    REFERENCE tests/models/furball/furball_catmulrom_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_catmulrom_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_furball_linear_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_linear_flat.ecs 
    REFERENCE tests/models/furball/furball_linear_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_linear_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_furball_linear_round.ecs embree_viewer 
    ECS tests/models/furball/furball_linear_round.ecs 
    REFERENCE tests/models/furball/furball_linear_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_linear_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_spheres_grids.ecs embree_viewer 
    ECS tests/models/furball/spheres_grids.ecs 
    REFERENCE tests/models/furball/spheres_grids.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_grids.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_spheres_points.ecs embree_viewer 
    ECS tests/models/furball/spheres_points.ecs 
    REFERENCE tests/models/furball/spheres_points.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_points.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_spheres_quads.ecs embree_viewer 
    ECS tests/models/furball/spheres_quads.ecs 
    REFERENCE tests/models/furball/spheres_quads.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_quads.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_furball_spheres_triangles.ecs embree_viewer 
    ECS tests/models/furball/spheres_triangles.ecs 
    REFERENCE tests/models/furball/spheres_triangles.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_triangles.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_curves_msmblur.ecs embree_viewer 
    ECS tests/models/msmblur/curves_msmblur.ecs 
    REFERENCE tests/models/msmblur/curves_msmblur.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/curves_msmblur.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_curves_msmblur2.ecs embree_viewer 
    ECS tests/models/msmblur/curves_msmblur2.ecs 
    REFERENCE tests/models/msmblur/curves_msmblur2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/curves_msmblur2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_lines_msmblur.ecs embree_viewer 
    ECS tests/models/msmblur/lines_msmblur.ecs 
    REFERENCE tests/models/msmblur/lines_msmblur.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/lines_msmblur.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_lines_msmblur2.ecs embree_viewer 
    ECS tests/models/msmblur/lines_msmblur2.ecs 
    REFERENCE tests/models/msmblur/lines_msmblur2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/lines_msmblur2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_mblur_time_range_triangle.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_triangle.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_triangle.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_mblur_time_range_quad.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_quad.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_quad.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_quad.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_mblur_time_range_grid.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_grid.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_grid.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_grid.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_mblur_time_range_curve.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_curve.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_curve.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_curve.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_mblur_time_range_line.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_line.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_line.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_line.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_mblur_time_range_instancing.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_instancing.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_instancing.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_instancing.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_mblur_time_range_sphere.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_sphere.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_sphere.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_sphere.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_mblur_time_range_disc.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_disc.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_disc.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_disc.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_mblur_time_range_oriented_disc.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_oriented_disc.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_oriented_disc.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_oriented_disc.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_msmblur_linear_instance_linear_triangle.ecs embree_viewer 
    ECS tests/models/msmblur/linear_instance_linear_triangle.ecs 
    REFERENCE tests/models/msmblur/linear_instance_linear_triangle.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/linear_instance_linear_triangle.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_points_points.ecs embree_viewer 
    ECS tests/models/points/points.ecs 
    REFERENCE tests/models/points/points.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/points.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_points_min_width_spheres.ecs embree_viewer 
    ECS tests/models/points/min_width_spheres.ecs 
    REFERENCE tests/models/points/min_width_spheres.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/min_width_spheres.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_points_min_width_discs.ecs embree_viewer 
    ECS tests/models/points/min_width_discs.ecs 
    REFERENCE tests/models/points/min_width_discs.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/min_width_discs.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_points_min_width_oriented_discs.ecs embree_viewer 
    ECS tests/models/points/min_width_oriented_discs.ecs 
    REFERENCE tests/models/points/min_width_oriented_discs.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/min_width_oriented_discs.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_quaternion_motion_blur_quaternion_quad.ecs embree_viewer 
    ECS tests/models/quaternion_motion_blur/quaternion_quad.ecs 
    REFERENCE tests/models/quaternion_motion_blur/quaternion_quad.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/quaternion_motion_blur/quaternion_quad.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv0.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv0.ecs 
    REFERENCE tests/models/subdiv/subdiv0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv1.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv1.ecs 
    REFERENCE tests/models/subdiv/subdiv1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv3.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv3.ecs 
    REFERENCE tests/models/subdiv/subdiv3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv4.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv4.ecs 
    REFERENCE tests/models/subdiv/subdiv4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv5.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv5.ecs 
    REFERENCE tests/models/subdiv/subdiv5.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv5.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv6.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv6.ecs 
    REFERENCE tests/models/subdiv/subdiv6.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv6.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv7.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv7.ecs 
    REFERENCE tests/models/subdiv/subdiv7.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv7.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv8.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv8.ecs 
    REFERENCE tests/models/subdiv/subdiv8.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv8.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv9.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv9.ecs 
    REFERENCE tests/models/subdiv/subdiv9.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv9.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv_no_boundary.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_no_boundary.ecs 
    REFERENCE tests/models/subdiv/subdiv_no_boundary.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_no_boundary.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv_smooth_boundary.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_smooth_boundary.ecs 
    REFERENCE tests/models/subdiv/subdiv_smooth_boundary.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_smooth_boundary.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv_pin_corners.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_pin_corners.ecs 
    REFERENCE tests/models/subdiv/subdiv_pin_corners.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_pin_corners.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv_pin_boundary.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_pin_boundary.ecs 
    REFERENCE tests/models/subdiv/subdiv_pin_boundary.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_pin_boundary.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_subdiv_pin_all.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_pin_all.ecs 
    REFERENCE tests/models/subdiv/subdiv_pin_all.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_pin_all.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_coherent_models_subdiv_cylinder.ecs embree_viewer 
    ECS tests/models/subdiv/cylinder.ecs 
    REFERENCE tests/models/subdiv/cylinder.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/cylinder.ecs.embree_options 
    CONDITION  "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_curve0.ecs embree_viewer 
    ECS tests/models/curves/curve0.ecs 
    REFERENCE tests/models/curves/curve0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_curve1.ecs embree_viewer 
    ECS tests/models/curves/curve1.ecs 
    REFERENCE tests/models/curves/curve1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_curve2.ecs embree_viewer 
    ECS tests/models/curves/curve2.ecs 
    REFERENCE tests/models/curves/curve2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_curve3.ecs embree_viewer 
    ECS tests/models/curves/curve3.ecs 
    REFERENCE tests/models/curves/curve3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_curve4.ecs embree_viewer 
    ECS tests/models/curves/curve4.ecs 
    REFERENCE tests/models/curves/curve4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_curve5.ecs embree_viewer 
    ECS tests/models/curves/curve5.ecs 
    REFERENCE tests/models/curves/curve5.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve5.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_hair0.ecs embree_viewer 
    ECS tests/models/curves/hair0.ecs 
    REFERENCE tests/models/curves/hair0.ecs.embree_viewer_quad.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/hair0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_hair1.ecs embree_viewer 
    ECS tests/models/curves/hair1.ecs 
    REFERENCE tests/models/curves/hair1.ecs.embree_viewer_quad.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/hair1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_oriented_curve0.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve0.ecs 
    REFERENCE tests/models/curves/oriented_curve0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_oriented_curve1.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve1.ecs 
    REFERENCE tests/models/curves/oriented_curve1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_oriented_curve2.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve2.ecs 
    REFERENCE tests/models/curves/oriented_curve2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_oriented_curve3.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve3.ecs 
    REFERENCE tests/models/curves/oriented_curve3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_oriented_curve4.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve4.ecs 
    REFERENCE tests/models/curves/oriented_curve4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_oriented_bspline_curve_twisted.ecs embree_viewer 
    ECS tests/models/curves/oriented_bspline_curve_twisted.ecs 
    REFERENCE tests/models/curves/oriented_bspline_curve_twisted.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_bspline_curve_twisted.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_oriented_hermite_curve_twisted.ecs embree_viewer 
    ECS tests/models/curves/oriented_hermite_curve_twisted.ecs 
    REFERENCE tests/models/curves/oriented_hermite_curve_twisted.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_hermite_curve_twisted.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_round_line_segments_0.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_0.ecs 
    REFERENCE tests/models/curves/round_line_segments_0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/round_line_segments_0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_round_line_segments_1.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_1.ecs 
    REFERENCE tests/models/curves/round_line_segments_1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_round_line_segments_2.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_2.ecs 
    REFERENCE tests/models/curves/round_line_segments_2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_round_line_segments_3.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_3.ecs 
    REFERENCE tests/models/curves/round_line_segments_3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_round_line_segments_4.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_4.ecs 
    REFERENCE tests/models/curves/round_line_segments_4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_round_line_segments_5.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_5.ecs 
    REFERENCE tests/models/curves/round_line_segments_5.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_round_line_segments_6.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_6.ecs 
    REFERENCE tests/models/curves/round_line_segments_6.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_round_line_segments_7.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_7.ecs 
    REFERENCE tests/models/curves/round_line_segments_7.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_round_line_segments_8.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_8.ecs 
    REFERENCE tests/models/curves/round_line_segments_8.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_round_line_segments_9.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_9.ecs 
    REFERENCE tests/models/curves/round_line_segments_9.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/round_line_segments_9.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_min_width_flat_linear_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_flat_linear_curves.ecs 
    REFERENCE tests/models/curves/min_width_flat_linear_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_flat_linear_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_min_width_round_linear_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_round_linear_curves.ecs 
    REFERENCE tests/models/curves/min_width_round_linear_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_round_linear_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_min_width_flat_bezier_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_flat_bezier_curves.ecs 
    REFERENCE tests/models/curves/min_width_flat_bezier_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_flat_bezier_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_min_width_round_bezier_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_round_bezier_curves.ecs 
    REFERENCE tests/models/curves/min_width_round_bezier_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_round_bezier_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_curves_min_width_oriented_hermite_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_oriented_hermite_curves.ecs 
    REFERENCE tests/models/curves/min_width_oriented_hermite_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_oriented_hermite_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_furball_bezier_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_bezier_flat.ecs 
    REFERENCE tests/models/furball/furball_bezier_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bezier_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_furball_bezier_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_bezier_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_bezier_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bezier_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_furball_bezier_round.ecs embree_viewer 
    ECS tests/models/furball/furball_bezier_round.ecs 
    REFERENCE tests/models/furball/furball_bezier_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bezier_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_furball_bspline_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_bspline_flat.ecs 
    REFERENCE tests/models/furball/furball_bspline_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bspline_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_furball_bspline_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_bspline_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_bspline_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bspline_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_furball_bspline_round.ecs embree_viewer 
    ECS tests/models/furball/furball_bspline_round.ecs 
    REFERENCE tests/models/furball/furball_bspline_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bspline_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_furball_hermite_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_hermite_flat.ecs 
    REFERENCE tests/models/furball/furball_hermite_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_hermite_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_furball_hermite_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_hermite_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_hermite_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_hermite_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_furball_catmulrom_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_catmulrom_flat.ecs 
    REFERENCE tests/models/furball/furball_catmulrom_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_catmulrom_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_furball_catmulrom_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_catmulrom_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_catmulrom_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_catmulrom_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_furball_catmulrom_round.ecs embree_viewer 
    ECS tests/models/furball/furball_catmulrom_round.ecs 
    REFERENCE tests/models/furball/furball_catmulrom_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_catmulrom_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_furball_linear_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_linear_flat.ecs 
    REFERENCE tests/models/furball/furball_linear_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_linear_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_furball_linear_round.ecs embree_viewer 
    ECS tests/models/furball/furball_linear_round.ecs 
    REFERENCE tests/models/furball/furball_linear_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_linear_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_spheres_grids.ecs embree_viewer 
    ECS tests/models/furball/spheres_grids.ecs 
    REFERENCE tests/models/furball/spheres_grids.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_grids.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_spheres_points.ecs embree_viewer 
    ECS tests/models/furball/spheres_points.ecs 
    REFERENCE tests/models/furball/spheres_points.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_points.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_spheres_quads.ecs embree_viewer 
    ECS tests/models/furball/spheres_quads.ecs 
    REFERENCE tests/models/furball/spheres_quads.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_quads.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_furball_spheres_triangles.ecs embree_viewer 
    ECS tests/models/furball/spheres_triangles.ecs 
    REFERENCE tests/models/furball/spheres_triangles.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_triangles.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_curves_msmblur.ecs embree_viewer 
    ECS tests/models/msmblur/curves_msmblur.ecs 
    REFERENCE tests/models/msmblur/curves_msmblur.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/curves_msmblur.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_curves_msmblur2.ecs embree_viewer 
    ECS tests/models/msmblur/curves_msmblur2.ecs 
    REFERENCE tests/models/msmblur/curves_msmblur2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/curves_msmblur2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_lines_msmblur.ecs embree_viewer 
    ECS tests/models/msmblur/lines_msmblur.ecs 
    REFERENCE tests/models/msmblur/lines_msmblur.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/lines_msmblur.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_lines_msmblur2.ecs embree_viewer 
    ECS tests/models/msmblur/lines_msmblur2.ecs 
    REFERENCE tests/models/msmblur/lines_msmblur2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/lines_msmblur2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_mblur_time_range_triangle.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_triangle.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_triangle.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_mblur_time_range_quad.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_quad.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_quad.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_quad.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_mblur_time_range_grid.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_grid.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_grid.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_grid.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_mblur_time_range_curve.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_curve.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_curve.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_curve.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_mblur_time_range_line.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_line.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_line.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_line.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_mblur_time_range_instancing.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_instancing.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_instancing.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_instancing.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_mblur_time_range_sphere.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_sphere.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_sphere.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_sphere.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_mblur_time_range_disc.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_disc.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_disc.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_disc.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_mblur_time_range_oriented_disc.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_oriented_disc.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_oriented_disc.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_oriented_disc.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_msmblur_linear_instance_linear_triangle.ecs embree_viewer 
    ECS tests/models/msmblur/linear_instance_linear_triangle.ecs 
    REFERENCE tests/models/msmblur/linear_instance_linear_triangle.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/linear_instance_linear_triangle.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_points_points.ecs embree_viewer 
    ECS tests/models/points/points.ecs 
    REFERENCE tests/models/points/points.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/points.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_points_min_width_spheres.ecs embree_viewer 
    ECS tests/models/points/min_width_spheres.ecs 
    REFERENCE tests/models/points/min_width_spheres.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/min_width_spheres.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_points_min_width_discs.ecs embree_viewer 
    ECS tests/models/points/min_width_discs.ecs 
    REFERENCE tests/models/points/min_width_discs.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/min_width_discs.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_points_min_width_oriented_discs.ecs embree_viewer 
    ECS tests/models/points/min_width_oriented_discs.ecs 
    REFERENCE tests/models/points/min_width_oriented_discs.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/min_width_oriented_discs.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_quaternion_motion_blur_quaternion_quad.ecs embree_viewer 
    ECS tests/models/quaternion_motion_blur/quaternion_quad.ecs 
    REFERENCE tests/models/quaternion_motion_blur/quaternion_quad.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/quaternion_motion_blur/quaternion_quad.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv0.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv0.ecs 
    REFERENCE tests/models/subdiv/subdiv0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv1.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv1.ecs 
    REFERENCE tests/models/subdiv/subdiv1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv3.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv3.ecs 
    REFERENCE tests/models/subdiv/subdiv3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv4.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv4.ecs 
    REFERENCE tests/models/subdiv/subdiv4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv5.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv5.ecs 
    REFERENCE tests/models/subdiv/subdiv5.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv5.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv6.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv6.ecs 
    REFERENCE tests/models/subdiv/subdiv6.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv6.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv7.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv7.ecs 
    REFERENCE tests/models/subdiv/subdiv7.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv7.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv8.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv8.ecs 
    REFERENCE tests/models/subdiv/subdiv8.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv8.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv9.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv9.ecs 
    REFERENCE tests/models/subdiv/subdiv9.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv9.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv_no_boundary.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_no_boundary.ecs 
    REFERENCE tests/models/subdiv/subdiv_no_boundary.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_no_boundary.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv_smooth_boundary.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_smooth_boundary.ecs 
    REFERENCE tests/models/subdiv/subdiv_smooth_boundary.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_smooth_boundary.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv_pin_corners.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_pin_corners.ecs 
    REFERENCE tests/models/subdiv/subdiv_pin_corners.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_pin_corners.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv_pin_boundary.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_pin_boundary.ecs 
    REFERENCE tests/models/subdiv/subdiv_pin_boundary.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_pin_boundary.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_subdiv_pin_all.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_pin_all.ecs 
    REFERENCE tests/models/subdiv/subdiv_pin_all.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_pin_all.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_quad_coherent_models_subdiv_cylinder.ecs embree_viewer 
    ECS tests/models/subdiv/cylinder.ecs 
    REFERENCE tests/models/subdiv/cylinder.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/cylinder.ecs.embree_options 
    CONDITION  "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_curve0.ecs embree_viewer 
    ECS tests/models/curves/curve0.ecs 
    REFERENCE tests/models/curves/curve0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_curve1.ecs embree_viewer 
    ECS tests/models/curves/curve1.ecs 
    REFERENCE tests/models/curves/curve1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_curve2.ecs embree_viewer 
    ECS tests/models/curves/curve2.ecs 
    REFERENCE tests/models/curves/curve2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_curve3.ecs embree_viewer 
    ECS tests/models/curves/curve3.ecs 
    REFERENCE tests/models/curves/curve3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_curve4.ecs embree_viewer 
    ECS tests/models/curves/curve4.ecs 
    REFERENCE tests/models/curves/curve4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_curve5.ecs embree_viewer 
    ECS tests/models/curves/curve5.ecs 
    REFERENCE tests/models/curves/curve5.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve5.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_hair0.ecs embree_viewer 
    ECS tests/models/curves/hair0.ecs 
    REFERENCE tests/models/curves/hair0.ecs.embree_viewer_grid.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/hair0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_hair1.ecs embree_viewer 
    ECS tests/models/curves/hair1.ecs 
    REFERENCE tests/models/curves/hair1.ecs.embree_viewer_grid.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/hair1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_oriented_curve0.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve0.ecs 
    REFERENCE tests/models/curves/oriented_curve0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_oriented_curve1.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve1.ecs 
    REFERENCE tests/models/curves/oriented_curve1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_oriented_curve2.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve2.ecs 
    REFERENCE tests/models/curves/oriented_curve2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_oriented_curve3.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve3.ecs 
    REFERENCE tests/models/curves/oriented_curve3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_oriented_curve4.ecs embree_viewer 
    ECS tests/models/curves/oriented_curve4.ecs 
    REFERENCE tests/models/curves/oriented_curve4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_oriented_bspline_curve_twisted.ecs embree_viewer 
    ECS tests/models/curves/oriented_bspline_curve_twisted.ecs 
    REFERENCE tests/models/curves/oriented_bspline_curve_twisted.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_bspline_curve_twisted.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_oriented_hermite_curve_twisted.ecs embree_viewer 
    ECS tests/models/curves/oriented_hermite_curve_twisted.ecs 
    REFERENCE tests/models/curves/oriented_hermite_curve_twisted.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_hermite_curve_twisted.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_round_line_segments_0.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_0.ecs 
    REFERENCE tests/models/curves/round_line_segments_0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/round_line_segments_0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_round_line_segments_1.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_1.ecs 
    REFERENCE tests/models/curves/round_line_segments_1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_round_line_segments_2.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_2.ecs 
    REFERENCE tests/models/curves/round_line_segments_2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_round_line_segments_3.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_3.ecs 
    REFERENCE tests/models/curves/round_line_segments_3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_round_line_segments_4.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_4.ecs 
    REFERENCE tests/models/curves/round_line_segments_4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_round_line_segments_5.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_5.ecs 
    REFERENCE tests/models/curves/round_line_segments_5.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_round_line_segments_6.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_6.ecs 
    REFERENCE tests/models/curves/round_line_segments_6.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_round_line_segments_7.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_7.ecs 
    REFERENCE tests/models/curves/round_line_segments_7.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_round_line_segments_8.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_8.ecs 
    REFERENCE tests/models/curves/round_line_segments_8.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_round_line_segments_9.ecs embree_viewer 
    ECS tests/models/curves/round_line_segments_9.ecs 
    REFERENCE tests/models/curves/round_line_segments_9.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/round_line_segments_9.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_min_width_flat_linear_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_flat_linear_curves.ecs 
    REFERENCE tests/models/curves/min_width_flat_linear_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_flat_linear_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_min_width_round_linear_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_round_linear_curves.ecs 
    REFERENCE tests/models/curves/min_width_round_linear_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_round_linear_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_min_width_flat_bezier_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_flat_bezier_curves.ecs 
    REFERENCE tests/models/curves/min_width_flat_bezier_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_flat_bezier_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_min_width_round_bezier_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_round_bezier_curves.ecs 
    REFERENCE tests/models/curves/min_width_round_bezier_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_round_bezier_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_curves_min_width_oriented_hermite_curves.ecs embree_viewer 
    ECS tests/models/curves/min_width_oriented_hermite_curves.ecs 
    REFERENCE tests/models/curves/min_width_oriented_hermite_curves.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/min_width_oriented_hermite_curves.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_furball_bezier_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_bezier_flat.ecs 
    REFERENCE tests/models/furball/furball_bezier_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bezier_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_furball_bezier_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_bezier_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_bezier_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bezier_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_furball_bezier_round.ecs embree_viewer 
    ECS tests/models/furball/furball_bezier_round.ecs 
    REFERENCE tests/models/furball/furball_bezier_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bezier_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_furball_bspline_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_bspline_flat.ecs 
    REFERENCE tests/models/furball/furball_bspline_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bspline_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_furball_bspline_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_bspline_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_bspline_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bspline_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_furball_bspline_round.ecs embree_viewer 
    ECS tests/models/furball/furball_bspline_round.ecs 
    REFERENCE tests/models/furball/furball_bspline_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_bspline_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_furball_hermite_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_hermite_flat.ecs 
    REFERENCE tests/models/furball/furball_hermite_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_hermite_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_furball_hermite_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_hermite_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_hermite_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_hermite_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_furball_catmulrom_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_catmulrom_flat.ecs 
    REFERENCE tests/models/furball/furball_catmulrom_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_catmulrom_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_furball_catmulrom_normaloriented.ecs embree_viewer 
    ECS tests/models/furball/furball_catmulrom_normaloriented.ecs 
    REFERENCE tests/models/furball/furball_catmulrom_normaloriented.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_catmulrom_normaloriented.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_furball_catmulrom_round.ecs embree_viewer 
    ECS tests/models/furball/furball_catmulrom_round.ecs 
    REFERENCE tests/models/furball/furball_catmulrom_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_catmulrom_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_furball_linear_flat.ecs embree_viewer 
    ECS tests/models/furball/furball_linear_flat.ecs 
    REFERENCE tests/models/furball/furball_linear_flat.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_linear_flat.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_furball_linear_round.ecs embree_viewer 
    ECS tests/models/furball/furball_linear_round.ecs 
    REFERENCE tests/models/furball/furball_linear_round.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/furball_linear_round.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_spheres_grids.ecs embree_viewer 
    ECS tests/models/furball/spheres_grids.ecs 
    REFERENCE tests/models/furball/spheres_grids.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_grids.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_spheres_points.ecs embree_viewer 
    ECS tests/models/furball/spheres_points.ecs 
    REFERENCE tests/models/furball/spheres_points.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_points.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_spheres_quads.ecs embree_viewer 
    ECS tests/models/furball/spheres_quads.ecs 
    REFERENCE tests/models/furball/spheres_quads.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_quads.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_furball_spheres_triangles.ecs embree_viewer 
    ECS tests/models/furball/spheres_triangles.ecs 
    REFERENCE tests/models/furball/spheres_triangles.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/furball/spheres_triangles.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_curves_msmblur.ecs embree_viewer 
    ECS tests/models/msmblur/curves_msmblur.ecs 
    REFERENCE tests/models/msmblur/curves_msmblur.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/curves_msmblur.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_curves_msmblur2.ecs embree_viewer 
    ECS tests/models/msmblur/curves_msmblur2.ecs 
    REFERENCE tests/models/msmblur/curves_msmblur2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/curves_msmblur2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_lines_msmblur.ecs embree_viewer 
    ECS tests/models/msmblur/lines_msmblur.ecs 
    REFERENCE tests/models/msmblur/lines_msmblur.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/lines_msmblur.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_lines_msmblur2.ecs embree_viewer 
    ECS tests/models/msmblur/lines_msmblur2.ecs 
    REFERENCE tests/models/msmblur/lines_msmblur2.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/lines_msmblur2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_mblur_time_range_triangle.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_triangle.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_triangle.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_mblur_time_range_quad.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_quad.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_quad.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_quad.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_mblur_time_range_grid.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_grid.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_grid.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_grid.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_mblur_time_range_curve.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_curve.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_curve.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_curve.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_mblur_time_range_line.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_line.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_line.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_line.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_mblur_time_range_instancing.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_instancing.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_instancing.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_instancing.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_mblur_time_range_sphere.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_sphere.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_sphere.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_sphere.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_mblur_time_range_disc.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_disc.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_disc.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_disc.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_mblur_time_range_oriented_disc.ecs embree_viewer 
    ECS tests/models/msmblur/mblur_time_range_oriented_disc.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_oriented_disc.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_oriented_disc.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_msmblur_linear_instance_linear_triangle.ecs embree_viewer 
    ECS tests/models/msmblur/linear_instance_linear_triangle.ecs 
    REFERENCE tests/models/msmblur/linear_instance_linear_triangle.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/linear_instance_linear_triangle.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_points_points.ecs embree_viewer 
    ECS tests/models/points/points.ecs 
    REFERENCE tests/models/points/points.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/points.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_points_min_width_spheres.ecs embree_viewer 
    ECS tests/models/points/min_width_spheres.ecs 
    REFERENCE tests/models/points/min_width_spheres.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/min_width_spheres.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_points_min_width_discs.ecs embree_viewer 
    ECS tests/models/points/min_width_discs.ecs 
    REFERENCE tests/models/points/min_width_discs.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/min_width_discs.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_points_min_width_oriented_discs.ecs embree_viewer 
    ECS tests/models/points/min_width_oriented_discs.ecs 
    REFERENCE tests/models/points/min_width_oriented_discs.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/min_width_oriented_discs.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_MIN_WIDTH == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_quaternion_motion_blur_quaternion_quad.ecs embree_viewer 
    ECS tests/models/quaternion_motion_blur/quaternion_quad.ecs 
    REFERENCE tests/models/quaternion_motion_blur/quaternion_quad.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/quaternion_motion_blur/quaternion_quad.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv0.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv0.ecs 
    REFERENCE tests/models/subdiv/subdiv0.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv1.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv1.ecs 
    REFERENCE tests/models/subdiv/subdiv1.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv3.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv3.ecs 
    REFERENCE tests/models/subdiv/subdiv3.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv4.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv4.ecs 
    REFERENCE tests/models/subdiv/subdiv4.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv5.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv5.ecs 
    REFERENCE tests/models/subdiv/subdiv5.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv5.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv6.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv6.ecs 
    REFERENCE tests/models/subdiv/subdiv6.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv6.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv7.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv7.ecs 
    REFERENCE tests/models/subdiv/subdiv7.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv7.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv8.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv8.ecs 
    REFERENCE tests/models/subdiv/subdiv8.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv8.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv9.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv9.ecs 
    REFERENCE tests/models/subdiv/subdiv9.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv9.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv_no_boundary.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_no_boundary.ecs 
    REFERENCE tests/models/subdiv/subdiv_no_boundary.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_no_boundary.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv_smooth_boundary.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_smooth_boundary.ecs 
    REFERENCE tests/models/subdiv/subdiv_smooth_boundary.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_smooth_boundary.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv_pin_corners.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_pin_corners.ecs 
    REFERENCE tests/models/subdiv/subdiv_pin_corners.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_pin_corners.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv_pin_boundary.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_pin_boundary.ecs 
    REFERENCE tests/models/subdiv/subdiv_pin_boundary.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_pin_boundary.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_subdiv_pin_all.ecs embree_viewer 
    ECS tests/models/subdiv/subdiv_pin_all.ecs 
    REFERENCE tests/models/subdiv/subdiv_pin_all.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/subdiv_pin_all.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(viewer_grid_coherent_models_subdiv_cylinder.ecs embree_viewer 
    ECS tests/models/subdiv/cylinder.ecs 
    REFERENCE tests/models/subdiv/cylinder.ecs.embree_viewer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/subdiv/cylinder.ecs.embree_options 
    CONDITION  "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS  --coherent -convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(instanced_geometry embree_instanced_geometry 
    REFERENCE tutorials/instanced_geometry/instanced_geometry.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/instanced_geometry/instanced_geometry.embree_options 
    CONDITION  "EMBREE_GEOMETRY_INSTANCE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(multi_instanced_geometry embree_multi_instanced_geometry 
    REFERENCE tutorials/multi_instanced_geometry/multi_instanced_geometry.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/multi_instanced_geometry/multi_instanced_geometry.embree_options 
    CONDITION  "EMBREE_GEOMETRY_INSTANCE == ON" "EMBREE_MAX_INSTANCE_LEVEL_COUNT > 1" 
    ARGS )

ADD_EMBREE_TEST_ECS(intersection_filter embree_intersection_filter 
    REFERENCE tutorials/intersection_filter/intersection_filter.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/intersection_filter/intersection_filter.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_FILTER_FUNCTION == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_curves_curve0.ecs embree_pathtracer 
    ECS tests/models/curves/curve0.ecs 
    REFERENCE tests/models/curves/curve0.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_curves_curve1.ecs embree_pathtracer 
    ECS tests/models/curves/curve1.ecs 
    REFERENCE tests/models/curves/curve1.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_curves_curve2.ecs embree_pathtracer 
    ECS tests/models/curves/curve2.ecs 
    REFERENCE tests/models/curves/curve2.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_curves_curve3.ecs embree_pathtracer 
    ECS tests/models/curves/curve3.ecs 
    REFERENCE tests/models/curves/curve3.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_curves_curve4.ecs embree_pathtracer 
    ECS tests/models/curves/curve4.ecs 
    REFERENCE tests/models/curves/curve4.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/curve4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_curves_oriented_curve0.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_curve0.ecs 
    REFERENCE tests/models/curves/oriented_curve0.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_curves_oriented_curve1.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_curve1.ecs 
    REFERENCE tests/models/curves/oriented_curve1.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_curves_oriented_curve2.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_curve2.ecs 
    REFERENCE tests/models/curves/oriented_curve2.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_curves_oriented_curve3.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_curve3.ecs 
    REFERENCE tests/models/curves/oriented_curve3.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_curves_oriented_curve4.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_curve4.ecs 
    REFERENCE tests/models/curves/oriented_curve4.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_curve4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_curves_oriented_bspline_curve_twisted.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_bspline_curve_twisted.ecs 
    REFERENCE tests/models/curves/oriented_bspline_curve_twisted.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_bspline_curve_twisted.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_curves_oriented_hermite_curve_twisted.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_hermite_curve_twisted.ecs 
    REFERENCE tests/models/curves/oriented_hermite_curve_twisted.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/curves/oriented_hermite_curve_twisted.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_msmblur_curves_msmblur.ecs embree_pathtracer 
    ECS tests/models/msmblur/curves_msmblur.ecs 
    REFERENCE tests/models/msmblur/curves_msmblur.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/curves_msmblur.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_msmblur_curves_msmblur2.ecs embree_pathtracer 
    ECS tests/models/msmblur/curves_msmblur2.ecs 
    REFERENCE tests/models/msmblur/curves_msmblur2.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/curves_msmblur2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_msmblur_lines_msmblur.ecs embree_pathtracer 
    ECS tests/models/msmblur/lines_msmblur.ecs 
    REFERENCE tests/models/msmblur/lines_msmblur.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/lines_msmblur.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_msmblur_lines_msmblur2.ecs embree_pathtracer 
    ECS tests/models/msmblur/lines_msmblur2.ecs 
    REFERENCE tests/models/msmblur/lines_msmblur2.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/lines_msmblur2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_msmblur_mblur_time_range_triangle.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_triangle.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_triangle.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE  
    CONDITION  "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_msmblur_mblur_time_range_quad.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_quad.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_quad.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_quad.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_msmblur_mblur_time_range_grid.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_grid.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_grid.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_grid.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_msmblur_mblur_time_range_curve.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_curve.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_curve.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_curve.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_msmblur_mblur_time_range_line.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_line.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_line.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_line.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_msmblur_mblur_time_range_instancing.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_instancing.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_instancing.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_instancing.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_msmblur_mblur_time_range_sphere.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_sphere.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_sphere.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_sphere.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_msmblur_mblur_time_range_disc.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_disc.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_disc.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_disc.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_msmblur_mblur_time_range_oriented_disc.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_oriented_disc.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_oriented_disc.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_oriented_disc.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_points_points.ecs embree_pathtracer 
    ECS tests/models/points/points.ecs 
    REFERENCE tests/models/points/points.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/points/points.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_models_quaternion_motion_blur_quaternion_quad.ecs embree_pathtracer 
    ECS tests/models/quaternion_motion_blur/quaternion_quad.ecs 
    REFERENCE tests/models/quaternion_motion_blur/quaternion_quad.ecs.embree_pathtracer.exr 
    INTENSITY 2 
    CONDITION_FILE tests/models/quaternion_motion_blur/quaternion_quad.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS )

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_curves_curve0.ecs embree_pathtracer 
    ECS tests/models/curves/curve0.ecs 
    REFERENCE tests/models/curves/curve0.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/curves/curve0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_curves_curve1.ecs embree_pathtracer 
    ECS tests/models/curves/curve1.ecs 
    REFERENCE tests/models/curves/curve1.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/curves/curve1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_curves_curve2.ecs embree_pathtracer 
    ECS tests/models/curves/curve2.ecs 
    REFERENCE tests/models/curves/curve2.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/curves/curve2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_curves_curve3.ecs embree_pathtracer 
    ECS tests/models/curves/curve3.ecs 
    REFERENCE tests/models/curves/curve3.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/curves/curve3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_curves_curve4.ecs embree_pathtracer 
    ECS tests/models/curves/curve4.ecs 
    REFERENCE tests/models/curves/curve4.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/curves/curve4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_curves_oriented_curve0.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_curve0.ecs 
    REFERENCE tests/models/curves/oriented_curve0.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/curves/oriented_curve0.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_curves_oriented_curve1.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_curve1.ecs 
    REFERENCE tests/models/curves/oriented_curve1.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/curves/oriented_curve1.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_curves_oriented_curve2.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_curve2.ecs 
    REFERENCE tests/models/curves/oriented_curve2.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/curves/oriented_curve2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_curves_oriented_curve3.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_curve3.ecs 
    REFERENCE tests/models/curves/oriented_curve3.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/curves/oriented_curve3.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_curves_oriented_curve4.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_curve4.ecs 
    REFERENCE tests/models/curves/oriented_curve4.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/curves/oriented_curve4.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_curves_oriented_bspline_curve_twisted.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_bspline_curve_twisted.ecs 
    REFERENCE tests/models/curves/oriented_bspline_curve_twisted.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/curves/oriented_bspline_curve_twisted.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_curves_oriented_hermite_curve_twisted.ecs embree_pathtracer 
    ECS tests/models/curves/oriented_hermite_curve_twisted.ecs 
    REFERENCE tests/models/curves/oriented_hermite_curve_twisted.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/curves/oriented_hermite_curve_twisted.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_msmblur_curves_msmblur.ecs embree_pathtracer 
    ECS tests/models/msmblur/curves_msmblur.ecs 
    REFERENCE tests/models/msmblur/curves_msmblur.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/msmblur/curves_msmblur.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_msmblur_curves_msmblur2.ecs embree_pathtracer 
    ECS tests/models/msmblur/curves_msmblur2.ecs 
    REFERENCE tests/models/msmblur/curves_msmblur2.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/msmblur/curves_msmblur2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_msmblur_lines_msmblur.ecs embree_pathtracer 
    ECS tests/models/msmblur/lines_msmblur.ecs 
    REFERENCE tests/models/msmblur/lines_msmblur.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/msmblur/lines_msmblur.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_msmblur_lines_msmblur2.ecs embree_pathtracer 
    ECS tests/models/msmblur/lines_msmblur2.ecs 
    REFERENCE tests/models/msmblur/lines_msmblur2.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/msmblur/lines_msmblur2.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_msmblur_mblur_time_range_triangle.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_triangle.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_triangle.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE  
    CONDITION  "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_msmblur_mblur_time_range_quad.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_quad.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_quad.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_quad.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_msmblur_mblur_time_range_grid.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_grid.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_grid.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_grid.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_msmblur_mblur_time_range_curve.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_curve.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_curve.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_curve.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_msmblur_mblur_time_range_line.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_line.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_line.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_line.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_msmblur_mblur_time_range_instancing.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_instancing.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_instancing.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_instancing.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_msmblur_mblur_time_range_sphere.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_sphere.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_sphere.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_sphere.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_msmblur_mblur_time_range_disc.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_disc.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_disc.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_disc.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_msmblur_mblur_time_range_oriented_disc.ecs embree_pathtracer 
    ECS tests/models/msmblur/mblur_time_range_oriented_disc.ecs 
    REFERENCE tests/models/msmblur/mblur_time_range_oriented_disc.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/msmblur/mblur_time_range_oriented_disc.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_points_points.ecs embree_pathtracer 
    ECS tests/models/points/points.ecs 
    REFERENCE tests/models/points/points.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/points/points.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(pathtracer_coherent_models_quaternion_motion_blur_quaternion_quad.ecs embree_pathtracer 
    ECS tests/models/quaternion_motion_blur/quaternion_quad.ecs 
    REFERENCE tests/models/quaternion_motion_blur/quaternion_quad.ecs.embree_pathtracer.exr 
    INTENSITY 3 
    CONDITION_FILE tests/models/quaternion_motion_blur/quaternion_quad.ecs.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" "EMBREE_SYCL_AOT_DEVICES != none" 
    ARGS  --coherent)

ADD_EMBREE_TEST_ECS(hair_geometry embree_hair_geometry 
    REFERENCE tutorials/hair_geometry/hair_geometry.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/hair_geometry/hair_geometry.embree_options 
    CONDITION  "EMBREE_GEOMETRY_USER == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(subdivision_geometry embree_subdivision_geometry 
    REFERENCE tutorials/subdivision_geometry/subdivision_geometry.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/subdivision_geometry/subdivision_geometry.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(displacement_geometry embree_displacement_geometry 
    REFERENCE tutorials/displacement_geometry/displacement_geometry.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/displacement_geometry/displacement_geometry.embree_options 
    CONDITION  "EMBREE_GEOMETRY_SUBDIVISION == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(grid_geometry embree_grid_geometry 
    REFERENCE tutorials/grid_geometry/grid_geometry.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/grid_geometry/grid_geometry.embree_options 
    CONDITION  "EMBREE_GEOMETRY_GRID == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(bvh_builder embree_bvh_builder 
    NO_REFERENCE 
    INTENSITY 1 
    NO_ISPC 
    NO_SYCL 
    CONDITION_FILE tutorials/bvh_builder/bvh_builder.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" 
    ARGS )

SET_EMBREE_TEST_PROPERTIES(bvh_builder PROPERTIES;TIMEOUT;2400) 
ADD_EMBREE_TEST_ECS(lazy_geometry embree_lazy_geometry 
    REFERENCE tutorials/lazy_geometry/lazy_geometry.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/lazy_geometry/lazy_geometry.embree_options 
    CONDITION  "EMBREE_GEOMETRY_USER == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(bvh_access embree_bvh_access 
    NO_REFERENCE 
    INTENSITY 1 
    NO_ISPC 
    NO_SYCL 
    CONDITION_FILE tutorials/bvh_access/bvh_access.embree_options 
    CONDITION  "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(quaternion_motion_blur embree_quaternion_motion_blur 
    REFERENCE tutorials/quaternion_motion_blur/quaternion_motion_blur.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/quaternion_motion_blur/quaternion_motion_blur.embree_options 
    CONDITION  "EMBREE_GEOMETRY_INSTANCE == ON" "EMBREE_GEOMETRY_USER == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(motion_blur_geometry embree_motion_blur_geometry 
    REFERENCE tutorials/motion_blur_geometry/motion_blur_geometry.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/motion_blur_geometry/motion_blur_geometry.embree_options 
    CONDITION  "EMBREE_GEOMETRY_USER == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(interpolation embree_interpolation 
    REFERENCE tutorials/interpolation/interpolation.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/interpolation/interpolation.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" "EMBREE_GEOMETRY_CURVE == ON" "EMBREE_SYCL_SUPPORT == OFF" 
    ARGS )

ADD_EMBREE_TEST_ECS(curve_geometry embree_curve_geometry 
    REFERENCE tutorials/curve_geometry/curve_geometry.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/curve_geometry/curve_geometry.embree_options 
    CONDITION  "EMBREE_GEOMETRY_QUAD == ON" "EMBREE_GEOMETRY_CURVE == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(point_geometry embree_point_geometry 
    REFERENCE tutorials/point_geometry/point_geometry.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/point_geometry/point_geometry.embree_options 
    CONDITION  "EMBREE_GEOMETRY_POINT == ON" 
    ARGS )

ADD_EMBREE_TEST_ECS(collide embree_collide 
    NO_REFERENCE 
    INTENSITY 1 
    NO_ISPC 
    NO_SYCL 
    CONDITION_FILE tutorials/collide/collide.embree_options 
    CONDITION  "EMBREE_GEOMETRY_USER == ON" 
    ARGS  --benchmark 0 128)

ADD_EMBREE_TEST_ECS(next_hit_triangle_robust_cornell_box_1 embree_next_hit 
    NO_REFERENCE 
    INTENSITY 2 
    NO_ISPC 
    CONDITION_FILE  
    CONDITION  
    ARGS  --verify --max_next_hits 1 --max_total_hits 1024 -o test.tga)

ADD_EMBREE_TEST_ECS(next_hit_triangle_robust_cornell_box_2 embree_next_hit 
    NO_REFERENCE 
    INTENSITY 2 
    NO_ISPC 
    CONDITION_FILE  
    CONDITION  
    ARGS  --verify --max_next_hits 2 --max_total_hits 1024 -o test.tga)

ADD_EMBREE_TEST_ECS(next_hit_triangle_robust_cornell_box_3 embree_next_hit 
    NO_REFERENCE 
    INTENSITY 2 
    NO_ISPC 
    CONDITION_FILE  
    CONDITION  
    ARGS  --verify --max_next_hits 3 --max_total_hits 1024 -o test.tga)

ADD_EMBREE_TEST_ECS(next_hit_triangle_robust_cornell_box_4 embree_next_hit 
    NO_REFERENCE 
    INTENSITY 2 
    NO_ISPC 
    CONDITION_FILE  
    CONDITION  
    ARGS  --verify --max_next_hits 4 --max_total_hits 1024 -o test.tga)

ADD_EMBREE_TEST_ECS(next_hit_quads_robust_cornell_box_1 embree_next_hit 
    NO_REFERENCE 
    INTENSITY 2 
    NO_ISPC 
    CONDITION_FILE  
    CONDITION  
    ARGS  --verify --max_next_hits 1 --max_total_hits 1024 -o test.tga --convert-triangles-to-quads)

ADD_EMBREE_TEST_ECS(next_hit_grids_robust_cornell_box_1 embree_next_hit 
    NO_REFERENCE 
    INTENSITY 2 
    NO_ISPC 
    CONDITION_FILE  
    CONDITION  
    ARGS  --verify --max_next_hits 1 --max_total_hits 1024 -o test.tga --convert-triangles-to-grids)

ADD_EMBREE_TEST_ECS(multiscene_geometry embree_multiscene_geometry 
    REFERENCE tutorials/multiscene_geometry/multiscene_geometry.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/multiscene_geometry/multiscene_geometry.embree_options 
    CONDITION  "EMBREE_ISPC_SUPPORT == OFF" 
    ARGS  --compare-threshold 40)

ADD_EMBREE_TEST_ECS(ray_mask embree_ray_mask 
    REFERENCE tutorials/ray_mask/ray_mask.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/ray_mask/ray_mask.embree_options 
    CONDITION  "EMBREE_RAY_TRIANGLE == ON" "EMBREE_RAY_MASK == ON" 
    ARGS  --frames 128)

ADD_EMBREE_TEST_ECS(forest embree_forest 
    REFERENCE tutorials/forest/forest.exr 
    INTENSITY 1 
    CONDITION_FILE tutorials/forest/forest.embree_options 
    CONDITION  "EMBREE_GEOMETRY_TRIANGLE == ON" "EMBREE_GEOMETRY_INSTANCE == ON" "EMBREE_GEOMETRY_INSTANCE_ARRAY == ON" 
    ARGS  --frames 4)

ADD_EMBREE_TEST_ECS(embree_tests embree_tests 
    NO_REFERENCE 
    INTENSITY 1 
    NO_ISPC 
    NO_SYCL 
    CONDITION_FILE  
    CONDITION  
    ARGS )

SET_EMBREE_TEST_PROPERTIES(embree_tests PROPERTIES;TIMEOUT;7000) 
