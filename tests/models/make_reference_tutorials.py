from email.mime import base
import glob
import os
import subprocess
import platform
import time
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))

verbose = False

errs = []

def render(tutorial, args):
    outimage = f"{script_dir}/reference/{tutorial}.exr"
    cmd = f"{script_dir}/../../build/embree_{tutorial} -o {outimage} {args}"
    if verbose:
        print(cmd)
    if os.system(cmd) == 0:
        print(f"Rendered {outimage}")
    else:
        errs.append(t)

tutorials = [
    ("closest_point", ""),
    ("curve_geometry", ""),
    ("displacement_geometry", ""),
    ("dynamic_scene", ""),
    ("grid_geometry", ""),
    ("hair_geometry", ""),
    ("instanced_geometry", ""),
    ("interpolation", ""),
    ("intersection_filter", ""),
    ("lazy_geometry", ""),
    ("motion_blur_geometry", ""),
    ("multi_instanced_geometry", ""),
    ("multiscene_geometry", ""),
    ("point_geometry", ""),
    ("quaternion_motion_blur", ""),
    ("ray_mask", ""),
    ("subdivision_geometry", ""),
    ("triangle_geometry", ""),
    ("user_geometry", ""),
    ("voronoi", ""),
]

for (t, a) in tutorials:
    render(t, a)

if errs:
    print("There have been errors!")
    for e in errs:
        print(f"  - {e}")
    print("Could not be rendered")