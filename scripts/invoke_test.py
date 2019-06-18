#!/usr/bin/python

## ======================================================================== ##
## Copyright 2009-2012 Intel Corporation                                    ##
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

import sys
import os
import re
import ctypes
import shutil
import subprocess 
import multiprocessing

name = ""
model = ""
reference = ""
modeldir = ""
sde = "OFF"

def assert_fatal(condition, error):
  if not condition:
    sys.stdout.write("[error] %s\n" % error)
    sys.exit(1)

def assert_image_exists(name, path):
  assert_fatal(os.path.isfile(path), "%s image %s does not exist." % (name, path))


def compareImages(image0,image1,dimage):
  error = float("inf")
  assert_image_exists("output", image0)
  assert_image_exists("reference", image1)
  line, unused_err = subprocess.Popen("compare -metric MAE "+image0+" "+image1+" -compose Src "+dimage, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True).communicate()
  try: error = float(line[line.index('(')+1:line.index(')')])
  except ValueError:
    print("Error: "+line)
    raise ValueError
  return error

def printUsage():
  sys.stderr.write('Usage: ' + sys.argv[0] + ' --name testname --modeldir path --model path/model.ecs --execute executable args\n')
  sys.exit(1)

def parseArgs(argv):
  global name
  global model
  global reference
  global modeldir
  global sde
  if (argv[0] == '--name'):
    name = argv[1]
    return parseArgs(argv[2:len(argv)])
  elif (argv[0] == '--model'):
    model = argv[1]
    return parseArgs(argv[2:len(argv)])
  elif (argv[0] == '--reference'):
    reference = argv[1]
    return parseArgs(argv[2:len(argv)])
  elif (argv[0] == '--modeldir'):
    modeldir = argv[1]
    return parseArgs(argv[2:len(argv)])
  elif (argv[0] == '--sde'):
    sde = argv[1]
    return parseArgs(argv[2:len(argv)])
  elif (argv[0] == '--execute'):
    return " ".join(argv[1:len(argv)])
  else:
    printUsage()
    sys.exit(1)

# detect platform
if sys.platform.startswith("win"):
  dash = '\\'
  SEM_FAILCRITICALERRORS = 0x0001
  SEM_NOGPFAULTERRORBOX  = 0x0002
  SEM_NOOPENFILEERRORBOX = 0x8000
  ctypes.windll.kernel32.SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX | SEM_NOOPENFILEERRORBOX);
elif sys.platform.startswith("linux"):
  dash = '/'
elif sys.platform.startswith("darwin"):
  dash = '/'
else:
  print("unknown platform: "+ sys.platform);
  sys.exit(1)

# parse arguments
executable = parseArgs(sys.argv[1:len(sys.argv)])

assert_fatal(model != "", "--model must be set")
assert_fatal(os.path.isdir(modeldir), "--modeldir must exist")

refImageFileTga = modeldir + dash + "reference" + dash + reference + ".tga"
outImageFileTga = name + ".tga"
diffImageFileTga= name + ".diff.tga"

executable = executable + " -rtcore verbose=0"

if (model != "" and model != "default"):
  executable = executable + " -c " + modeldir + dash + model

executable = executable + " -o " + outImageFileTga

if (sde != "" and sde != "OFF"):
  if sys.platform.startswith("win"):
    executable = "sde -" + sde + " -- " + executable
  else:
    executable = "sde64 -" + sde + " -- " + executable

print(executable)

ret = os.system(executable)

if ret == 0:
  diff = compareImages(outImageFileTga,refImageFileTga,diffImageFileTga)
  if diff > 0.00056:
    sys.stdout.write(" [failed] [images differ by %f]\n" % diff)
    sys.exit(2)

if ret == 0:
  sys.stdout.write(" [passed]\n")
  sys.exit(0)
elif ret == -1073740791:
  sys.stdout.write(" [failed] [assertion]\n"); # assertion under windows
  sys.exit(3)
elif ret == 34304:
  sys.stdout.write(" [failed] [assertion]\n"); # assertion under linux
  sys.exit(3)
elif ret == -1073741819:
  sys.stdout.write(" [failed] [segfault]\n");  # segfault under windows
  sys.exit(4)
elif ret == -1073741571:
  sys.stdout.write(" [failed] [stack overflow]\n");  # stack overflow under windows
  sys.exit(4)
elif ret == 35584:
  sys.stdout.write(" [failed] [segfault]\n");  # segfault under linux
  sys.exit(4)
elif ret == 139:
  sys.stdout.write(" [failed] [segfault]\n");  # segfault under linux
  sys.exit(4)
else:
  sys.stdout.write(" [failed] [" + str(ret) + "]\n")
  sys.exit(1)
