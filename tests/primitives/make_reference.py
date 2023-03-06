from email.mime import base
import glob
import os
import subprocess
import platform
import time

script_dir = os.path.dirname(os.path.abspath(__file__))
print(script_dir)
print(os.getcwd())

viewer = os.getenv("EMBREE_VIEWER")
verbose = True

if (viewer == None):
    #print("Please set environment variable EMBREE_VIEWER before running this script.")
    viewer = os.path.sep.join([script_dir, "..", "..", "build", "embree_viewer"])
    c = input("EMBREE_VIEWER environment variable not set. Default is set to <embree_root>/build/viewer. Do you want to continue? Y/n")
    if c == "n":
        exit()

# permutations to be tested
shaders =  ["eyelight", "uv", "Ng", "primID", "occlusion"]
instancing = ["none", "geometry", "group", "multi_level" "flattened"]

t = time.time()
for fullpath in glob.iglob(script_dir + os.path.sep + "*.xml"):
    (path, filename) = os.path.split(fullpath)
    (basename, ext) = os.path.splitext(filename)

    for shader in shaders:
        command = f"{viewer} -i {fullpath} -shader {shader} --time 0.5"
        if not os.path.exists(path + os.path.sep + "reference"):
            os.mkdir(path + os.path.sep + "reference")

        imagepath = path + os.path.sep + "reference" + os.path.sep + "prim_" + basename +  "_" + shader + ".exr"
        if not os.path.exists(imagepath):
            if verbose:
                print("Rendering " + filename + " " + shader)
                print(command)
            os.system(command + " -o " + imagepath)

print (time.time() - t)