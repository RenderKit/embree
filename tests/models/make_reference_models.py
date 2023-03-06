from email.mime import base
import glob
import os
import subprocess
import platform
import time
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))

verbose = False

if len(sys.argv) < 3:
    print("""
Compute reference images for model tests.
needs the embree_viewer or embree_pathtracer respectively in the build folder
usage: 
    python make_reference.py <renderer> <tests_file>
where
    <renderer>    - one of [viewer, pathtracer]
    <tests_file>  - file containing a list of tests as relative paths to .ecs files e.g. tests/models/tests_all.txt

example:
    python tests/models/make_reference.py viewer tests/models/tests_all.txt

     - computes reference images for all tests in tests_all.txt using the viewer OVERWRITING old images in folder 
""")
    exit()

renderer = os.path.sep.join([script_dir, "..", "..", "build", "embree_" + sys.argv[1]])

fin = sys.argv[2]
tests = []
with open(fin, 'r') as f:
    tests = [l for l in [l.strip() for l in f.readlines()] if l and not l.startswith("#")]

if not os.path.exists(script_dir + os.path.sep + "reference"):
    os.mkdir(script_dir + os.path.sep + "reference")

errs = []
tt = time.time()
dirname = os.path.dirname(fin)
for t in tests:
    (path, filename) = os.path.split(t)
    (basename, ext) = os.path.splitext(filename)

    outimage = f"{os.path.sep.join([script_dir, 'reference', sys.argv[1] + '_' + path.replace(os.path.sep, '_')])}_{basename}.exr"
    cmd = f"{renderer} -c {dirname + os.path.sep + t} -o {outimage}"

    if verbose:
        print(cmd)
    if os.system(cmd) == 0:
        print(f"Rendered {outimage}")
    else:
        errs.append(dirname + os.path.sep + t)



    if sys.argv[1] == "viewer":
        outimage = f"{os.path.sep.join([script_dir, 'reference', sys.argv[1] + '_quad_coherent_' + path.replace(os.path.sep, '_')])}_{basename}.exr"
        cmd = f"{renderer} -c {dirname + os.path.sep + t} -o {outimage} -convert-triangles-to-quads --coherent"
        if verbose:
            print(cmd)
        os.system(cmd)
        if os.system(cmd) == 0:
            print(f"Rendered {outimage}")
        else:
            if dirname + os.path.sep + t not in errs:
                errs.append(dirname + os.path.sep + t)

print (time.time() - tt)

if errs:
    print("There have been errors!")
    for e in errs:
        print(f"  - {e}")
    print("Could not be rendered")