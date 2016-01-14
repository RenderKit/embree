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

# Embree Regression Test Script
# ===============================

# Windows
# -------

# Prerequisites:
#   Install Python 3.2+
#   Install ImageMagick
#   Install Visual Studio 2013
#   Install Intel C++ Compiler
#   Check out Embree into <embree_dir>

# Instructions:
#   Open the "Visual Studio x64 Cross Tools Command Prompt (2013)"
#   cd <embree_dir>
#   <python_dir>\python.exe <embree_dir>\scripts\benchmark.py run     windows test_dir
#   <python_dir>\python.exe <embree_dir>\scripts\benchmark.py compile windows test_dir

# Linux and OS X
# --------------

# Prerequisites:
#   Install Python 2.6+
#   Install ImageMagick
#   Install Intel C++ Compiler
#   Check out Embree into <embree_dir>

# Instructions:
#   Open a shell
#   cd <embree_dir>
#   mkdir TEST
#   ./scripts/benchmark.py run     linux <model_dir> test_dir
#   ./scripts/benchmark.py compile linux <model_dir> test_dir

import sys
import os
import re
import ctypes
import shutil
import subprocess 
import multiprocessing

dash = '/'

########################## configuration ##########################

#compilers_win = ['V140']
#compilers_win = ['ICC']
compilers_win  = ['V110', 'V120', 'ICC']
#compilers_win  = ['ICC', 'V110', 'V120', 'V140']
#compilers_unix = ['ICC']
compilers_unix = ['GCC', 'CLANG', 'ICC']
compilers      = []

#platforms_win  = ['Win32']
#platforms_win  = ['x64']
platforms_win  = ['Win32', 'x64']
platforms_unix = ['x64']
platforms      = []

#builds_win = ['Debug']
builds_win = ['RelWithDebInfo']
#builds_win = ['RelWithDebInfo', 'Debug']
#builds_unix = ['Debug']
builds_unix = ['RelWithDebInfo']
#builds_unix = ['RelWithDebInfo', 'Debug']
builds = []

#ISAs_win  = ['AVX2']
ISAs_win  = ['SSE2', 'AVX', 'AVX2']
#ISAs_unix = ['AVX2']
#ISAs_unix = ['SSE2', 'AVX', 'AVX2']
ISAs_unix = ['SSE2', 'AVX', 'AVX512KNL']
ISAs = []

supported_configurations = [
  'V110_Win32_RelWithDebInfo_SSE2',
  'V110_x64_RelWithDebInfo_SSE2',
  
  'V120_Win32_RelWithDebInfo_SSE2',
  'V120_Win32_RelWithDebInfo_SSE4.2',
  'V120_Win32_RelWithDebInfo_AVX',
  'V120_Win32_RelWithDebInfo_AVX2', 
  'V120_x64_RelWithDebInfo_SSE2',
  'V120_x64_RelWithDebInfo_SSE4.2',
  'V120_x64_RelWithDebInfo_AVX',  
  'V120_x64_RelWithDebInfo_AVX2',

  'V140_Win32_RelWithDebInfo_SSE2',
  'V140_Win32_RelWithDebInfo_SSE4.2',
  'V140_Win32_RelWithDebInfo_AVX',
  'V140_Win32_RelWithDebInfo_AVX2', 
  'V140_x64_RelWithDebInfo_SSE2',
  'V140_x64_RelWithDebInfo_SSE4.2',
  'V140_x64_RelWithDebInfo_AVX',  
  'V140_x64_RelWithDebInfo_AVX2',
  
  'ICC_Win32_RelWithDebInfo_SSE2',
  'ICC_Win32_RelWithDebInfo_SSE4.2',
  'ICC_Win32_RelWithDebInfo_AVX',
  'ICC_Win32_RelWithDebInfo_AVX2',
  
  'ICC_x64_RelWithDebInfo_SSE2',
  'ICC_x64_RelWithDebInfo_SSE4.2',
  'ICC_x64_RelWithDebInfo_AVX',
  'ICC_x64_RelWithDebInfo_AVX2',
  'ICC_x64_RelWithDebInfo_AVX512KNL',
  
  'GCC_x64_RelWithDebInfo_SSE2',
  'GCC_x64_RelWithDebInfo_SSE4.2',
  'GCC_x64_RelWithDebInfo_AVX',
  'GCC_x64_RelWithDebInfo_AVX2',
  
  'CLANG_x64_RelWithDebInfo_SSE2',
  'CLANG_x64_RelWithDebInfo_SSE4.2',
  'CLANG_x64_RelWithDebInfo_AVX',
  'CLANG_x64_RelWithDebInfo_AVX2',  
  ]

models_subdiv = []
models_small_win32 = []
models_small_x64 = []
models_large = []
modelDir  = ''
testDir = ''
generateReferenceImages = False

def configName(OS, compiler, platform, build, isa, tasking, tutorial, scene, flags):
  cfg = OS + '_' + compiler + '_' + platform + '_' + build + '_' + isa + '_' + tasking
  if tutorial != '':
    cfg += '_' + tutorial
  if scene != '':
    cfg += '_' + scene.replace('/','_').replace('.ecs','')
  if flags != '':
    cfg += '_' + flags
  return cfg

########################## compiling ##########################

def compile(OS,compiler,platform,build,isa,tasking):

  base = configName(OS, compiler, platform, build, isa, tasking, 'build', '', '')
  logFile = testDir + dash + base + '.log'

  if OS == 'windows':

    # set CMake generator and compiler
    if (compiler == 'V110'):
      generator = 'Visual Studio 11 2012'
      full_compiler = 'V110'
    elif (compiler == 'V120'):
      generator = 'Visual Studio 12 2013'
      full_compiler = 'V120'
    elif (compiler == 'V140'):
      generator = 'Visual Studio 14 2015'
      full_compiler = 'V140'
    elif (compiler == 'ICC'):
      generator = 'Visual Studio 12 2013'
#      full_compiler = '"Intel C++ Compiler XE 15.0" '
      full_compiler = '"Intel C++ Compiler 16.0" '
    else:
      sys.stderr.write('unknown compiler: ' + compiler + '\n')
      sys.exit(1)

	# set platform
    if (platform == 'Win32'):
      generator += ''
    elif (platform == 'x64'):
	  generator += ' Win64'
    else:
      sys.stderr.write('unknown platform: ' + platform + '\n')
      sys.exit(1)

    # generate build directory
    if os.path.exists('build'):
      ret = os.system('rm -rf build && mkdir build')
      if ret != 0:
        sys.stdout.write("Cannot delete build folder!")
        return ret
    else:	
      os.system('mkdir build')	

    # generate solution files using cmake
    command = 'cmake -L '
    command += ' -G "' + generator + '"'
    command += ' -T ' + full_compiler
    command += ' -D XEON_ISA=' + isa
    command += ' -D RTCORE_RAY_MASK=OFF'
    command += ' -D RTCORE_BACKFACE_CULLING=OFF'
    command += ' -D RTCORE_INTERSECTION_FILTER=ON'
    command += ' -D RTCORE_BUFFER_STRIDE=ON'
    command += ' -D RTCORE_STAT_COUNTERS=OFF'
    if tasking == 'tbb':
      command += ' -D RTCORE_TASKING_SYSTEM=TBB'
    elif tasking == 'internal':
      command += ' -D RTCORE_TASKING_SYSTEM=INTERNAL'
    else:
      sys.stdout.write("invalid tasking system: "+tasking)
      return 1
    command += ' ..'
    os.system('echo ' + command + ' > ' + logFile)
    ret = os.system('cd build && ' + command + ' >> ../' + logFile)
    if ret != 0: return ret

    # compile Embree
    command =  'msbuild build\embree2.sln' + ' /m /nologo /p:Platform=' + platform + ' /p:Configuration=' + build + ' /t:rebuild /verbosity:n' 
    os.system('echo ' + command + ' >> ' + logFile)
    return os.system(command + ' >> ' + logFile)
  
  else:
 
    # set platform
    if (platform != 'x64'):
      sys.stderr.write('unknown platform: ' + platform + '\n')
      sys.exit(1)

    # set compiler
    if (compiler == 'CLANG'):
      c_compiler_bin = 'clang'
      cpp_compiler_bin = 'clang++'
    elif (compiler == 'ICC'):
      c_compiler_bin = 'icc'
      cpp_compiler_bin = 'icpc'
    elif (compiler == 'GCC'):
      c_compiler_bin   = 'gcc'
      cpp_compiler_bin = 'g++'
    else:
      sys.stdout.write("invalid compiler: "+compiler)
      return 1
      
    # first we need to configure the compiler
    command = 'mkdir -p build && cd build && rm -f CMakeCache.txt'
    ret = os.system(command)
    if ret != 0: return ret
      
    command  = 'cd build && cmake ' 
    command += ' -D CMAKE_C_COMPILER:STRING=' + c_compiler_bin
    command += ' -D CMAKE_CXX_COMPILER:STRING=' + cpp_compiler_bin
    command += ' .. &> ../' + logFile 
    ret = os.system(command)
    if ret != 0: return ret
    
    # now we can set all other settings
    command  = 'cd build && cmake '
    command += ' -D CMAKE_BUILD_TYPE=' + build
    command += ' -D XEON_ISA=' + isa
    command += ' -D RTCORE_RAY_MASK=OFF'
    command += ' -D RTCORE_BACKFACE_CULLING=OFF'
    command += ' -D RTCORE_INTERSECTION_FILTER=ON'
    command += ' -D RTCORE_BUFFER_STRIDE=ON'
    command += ' -D RTCORE_STAT_COUNTERS=OFF'
    if tasking == 'tbb':
      command += ' -D RTCORE_TASKING_SYSTEM=TBB'
      command += ' -D TBB_ROOT=../tbb'
    elif tasking == 'internal':
      command += ' -D RTCORE_TASKING_SYSTEM=INTERNAL'
    else:
      sys.stdout.write("invalid tasking system: " + tasking)
      return 1
    command += ' .. >> ../' + logFile 
    ret = os.system(command)
    if ret != 0: return ret

    command  = 'cd build && make clean && make -j ' + str(multiprocessing.cpu_count())
    command += ' 2>> ../' + logFile + ' >> ../' + logFile
    return os.system(command)

def compileLoop(OS):
    for compiler in compilers:
      for platform in platforms:
        for build in builds:
          for isa in ISAs:
            for tasking in ['tbb','internal']:
              if (compiler + '_' + platform + '_' + build + '_' + isa) in supported_configurations:
                sys.stdout.write(OS + ' ' + compiler + ' ' + platform + ' ' + build + ' ' + isa + ' ' + tasking)
                sys.stdout.flush()
                ret = compile(OS,compiler,platform,build,isa,tasking)
                if ret != 0: sys.stdout.write(" [failed]\n")
                else:        sys.stdout.write(" [passed]\n")

########################## rendering ##########################

def tutorialGeneratesImage(tutorial):
  return tutorial != 'verify' and tutorial != 'benchmark' and tutorial != 'bvh_access' and tutorial != "bvh_builder"

def compareImages(image0,image1):
  if not os.path.isfile(image0) or not os.path.isfile(image1): return False
  try: line = subprocess.check_output("compare -metric MAE "+image0+" "+image1+" null:", stderr=subprocess.STDOUT, shell=True)
  except subprocess.CalledProcessError, e: line = e.output
  error = float(line[line.index('(')+1:line.index(')')])
  return error < 0.001

def render(OS, compiler, platform, build, isa, tasking, tutorial, args, scene, name):
  sys.stdout.write("  "+tutorial)
  if scene != '': sys.stdout.write(' '+scene)
  if name != '' and name != 'default': sys.stdout.write(' '+name)
  sys.stdout.flush()
  base = configName(OS, compiler, platform, build, isa, tasking, tutorial, scene, name)
  logFile = testDir + dash + base + '.log'
  imageFile = testDir + dash + base + '.tga'
  refImageFile = modelDir + dash + "reference" + dash + tutorial.replace('_ispc','')
  if (scene != ''): refImageFile += '_' +  scene.replace('/','_').replace('.ecs','')
  refImageFile += '.tga'

  if generateReferenceImages:
    if os.path.isfile(refImageFile):
      sys.stdout.write(" [skipped]\n")
      return
  elif os.path.exists(logFile):
    sys.stdout.write(" [skipped]\n")
    return

  if OS == 'windows': command = 'build' + '\\' + build + '\\' + tutorial + ' ' + args + ' '
  else:               command = 'build' + '/' + tutorial + ' ' + args + ' '
  if tutorialGeneratesImage(tutorial):
    command += '-rtcore verbose=2'
    command += ' -size 512 512 -o ' + imageFile
  elif generateReferenceImages:
    sys.stdout.write(" [skipped]\n")
    return
  command += ' > ' + logFile + ' 2>&1'
  ret = os.system(command)

  if ret == 0 and tutorialGeneratesImage(tutorial):
    if generateReferenceImages and os.path.isfile(imageFile):
      shutil.copy(imageFile,refImageFile)
      sys.stdout.write(" [generated]\n")
      return
    if not compareImages(refImageFile,imageFile):
      sys.stdout.write(" [failed] [images differ]\n")
      return

  if   ret == 0          : sys.stdout.write(" [passed]\n")
  elif ret == -1073741819: sys.stdout.write(" [failed] [segfault]\n");  # segfault under windows
  elif ret == -1073740791: sys.stdout.write(" [failed] [assertion]\n"); # assertion under windows
  elif ret == 35584      : sys.stdout.write(" [failed] [segfault]\n");  # segfault under linux
  elif ret == 34304      : sys.stdout.write(" [failed] [assertion]\n"); # assertion under linux
  else                   : sys.stdout.write(" [failed]\n")

def processConfiguration(OS, compiler, platform, build, isa, tasking, models):
  sys.stdout.write('compiling configuration ' + compiler + ' ' + platform + ' ' + build + ' ' + isa + ' ' + tasking)
  sys.stdout.flush()
  ret = compile(OS,compiler,platform,build,isa,tasking)
  if ret != 0: sys.stdout.write(" [failed]\n")
  else:        
    sys.stdout.write(" [passed]\n")

    render(OS, compiler, platform, build, isa, tasking, 'verify', '', '', '')
    render(OS, compiler, platform, build, isa, tasking, 'benchmark', '', '', '')
    render(OS, compiler, platform, build, isa, tasking, 'bvh_access', '', '', '')
    render(OS, compiler, platform, build, isa, tasking, 'bvh_builder', '', '', '')

    for ty in ['','_ispc']:
      render(OS, compiler, platform, build, isa, tasking, 'triangle_geometry'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'dynamic_scene'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'user_geometry'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'instanced_geometry'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'intersection_filter'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'hair_geometry'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'subdivision_geometry'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'displacement_geometry'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'lazy_geometry'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'interpolation'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'motion_blur_geometry'+ty, '', '', 'default')

      for model in models_subdiv:
        render(OS,compiler,platform,build,isa,tasking,"viewer"+ty," -c " + modelDir + dash + model,model,'default')

      for model in models:
        for flag in ['static','dynamic','high_quality','robust','compact']:
          render(OS,compiler,platform,build,isa,tasking,"viewer"+ty," -rtcore flags="+flag+" -c " + modelDir + dash + model,model,"triangles_"+flag)
          render(OS,compiler,platform,build,isa,tasking,"viewer"+ty," -rtcore flags="+flag+" -c " + modelDir + dash + model + " -convert-triangles-to-quads",model,"quads_"+flag)
# -convert-bezier-to-lines

      for model in models:
        render(OS,compiler,platform,build,isa,tasking,"pathtracer"+ty," -c " + modelDir + dash + model,model,'triangles')
        render(OS,compiler,platform,build,isa,tasking,"pathtracer"+ty," -c " + modelDir + dash + model + " -convert-triangles-to-quads",model,'quads')

			    
def renderLoop(OS):
    for compiler in compilers:
      for platform in platforms:
        for build in builds:
          for isa in ISAs:
            for tasking in ['tbb','internal']:
              if (compiler + '_' + platform + '_' + build + '_' + isa) in supported_configurations:
                models = []
                if platform == 'Win32': models += models_small_win32
                else:                   models += models_small_x64
                if platform == 'x64' and OS != 'macosx':
                  models += models_large
                processConfiguration(OS, compiler, platform, build, isa, tasking, models)
                if generateReferenceImages: return

########################## command line parsing ##########################

def printUsage():
  sys.stderr.write('Usage: ' + sys.argv[0] + ' compile  <testDir>                # only tests compilation\n')
  sys.stderr.write('       ' + sys.argv[0] + ' generate <testDir> <modelDir>     # generates missing reference images\n')
  sys.stderr.write('       ' + sys.argv[0] + ' run      <testDir> <modelDir>     # compiles and runs all tests\n')
  sys.exit(1)

def readLines(fileName):
  lines = open(fileName).read().split('\n')
  lines = [x for x in lines if x != ""]
  return lines

def loadModelList(modelDir):
  global models_subdiv
  global models_small_win32
  global models_small_x64
  global models_large
  models_subdiv      = readLines(modelDir+dash+'embree-models-subdiv.txt')
  models_small_win32 = readLines(modelDir+dash+'embree-models-small-win32.txt')
  models_small_x64   = readLines(modelDir+dash+'embree-models-small-x64.txt')
  models_large       = readLines(modelDir+dash+'embree-models-large.txt')

# detect platform
if sys.platform.startswith("win"):
  OS = "windows"
  dash = '\\'
  compilers = compilers_win
  platforms = platforms_win
  builds = builds_win
  ISAs = ISAs_win
  modelDir = '%HOMEPATH%\\models\\embree-models'
  ctypes.windll.kernel32.SetErrorMode(0x0002);  # enable SEM_NOGPFAULTERRORBOX
elif sys.platform.startswith("linux"):
  OS = "linux"
  dash = '/'
  compilers = compilers_unix
  platforms = platforms_unix
  builds = builds_unix
  ISAs = ISAs_unix
  modelDir = '~/models/embree-models'
elif sys.platform.startswith("darwin"):
  OS = "macosx"
  dash = '/'
  compilers = compilers_unix
  platforms = platforms_unix
  builds = builds_unix
  ISAs = ISAs_unix
  modelDir = '~/models/embree-models'
else:
  print("unknown platform: "+ sys.platform);
  sys.exit(1)
  
if len(sys.argv) < 2: printUsage()
mode = sys.argv[1]

if mode == 'run' or mode == 'generate':
  generateReferenceImages = mode == 'generate'
  if len(sys.argv) < 3: printUsage()
  testDir = sys.argv[2]
  if not os.path.exists(testDir):
    os.system('mkdir '+testDir)
  if len(sys.argv) > 3: 
    modelDir = sys.argv[3]
  loadModelList(modelDir)
  renderLoop(OS)
  sys.exit(1)

if mode == 'compile':
  if len(sys.argv) < 3: printUsage()
  testDir = sys.argv[2]
  if not os.path.exists(testDir):
    os.system('mkdir '+testDir)
  compileLoop(OS)
  sys.exit(1)

