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

name = "unknown"
model = ""
modeldir = ""

def compareImages(image0,image1):
  if not os.path.isfile(image0) or not os.path.isfile(image1): return False
  try: line = subprocess.check_output("compare -metric MAE "+image0+" "+image1+" null:", stderr=subprocess.STDOUT, shell=True)
  except subprocess.CalledProcessError, e: line = e.output
  error = float(line[line.index('(')+1:line.index(')')])
  return error < 0.001

def printUsage():
  sys.stderr.write('Usage: ' + sys.argv[0] + ' --name testname --modeldir path --model path/model.ecs --execute executable args\n')
  sys.exit(1)

def parseArgs(argv):
  global name
  global model
  global modeldir
  if (argv[0] == '--name'):
    name = argv[1]
    return parseArgs(argv[2:len(argv)])
  elif (argv[0] == '--model'):
    model = argv[1]
    return parseArgs(argv[2:len(argv)])
  elif (argv[0] == '--modeldir'):
    modeldir = argv[1]
    return parseArgs(argv[2:len(argv)])
  elif (argv[0] == '--execute'):
    return " ".join(argv[1:len(argv)])
  else:
    printUsage()
    sys.exit(1)

# detect platform
if sys.platform.startswith("win"):
  dash = '\\'
  ctypes.windll.kernel32.SetErrorMode(0x0002);  # enable SEM_NOGPFAULTERRORBOX
elif sys.platform.startswith("linux"):
  dash = '/'
elif sys.platform.startswith("darwin"):
  dash = '/'
else:
  print("unknown platform: "+ sys.platform);
  sys.exit(1)

# parse arguments
executable = parseArgs(sys.argv[1:len(sys.argv)])

refImageFile = modeldir + dash + "reference" + dash + name + ".tga"
imageFile    = name + ".tga"

#executable = executable + " -rtcore verbose=2"

if (model != "" and model != "default"):
  executable = executable + " -c " + modeldir + dash + model

if (model != ""):
  executable = executable + " -o " + imageFile

ret = os.system(executable)

if ret == 0 and model != "":
  if not compareImages(refImageFile,imageFile):
    sys.stdout.write(" [failed] [images differ]\n")
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
elif ret == 35584:
  sys.stdout.write(" [failed] [segfault]\n");  # segfault under linux
  sys.exit(4)
else:
  sys.stdout.write(" [failed]\n")
  sys.exit(1)
