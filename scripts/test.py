#!/usr/bin/env python3

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0


import sys
import subprocess
import os
import pickle
import re
import shutil

cwd = os.getcwd()

g_debugMode = False
g_intensity = 2

def escape(str):
  str = str.replace("\\",r"\\")
  str = str.replace("\"",r"\"")
  return str

def parse_version(v):
  return tuple(map(int, v.split(".")))

# change some CMake paths in the CMakeCache.txt file for execution of tests
# on potentially different machine (e.g, copied build dir in CI)
def fix_cmake_paths():
  with open('build/CMakeCache.txt', 'r') as file:
      file_content = file.read()

  file_content = re.sub(r"(For build in directory: ).*",        os.path.join(r"\1"+cwd, "build"), file_content)
  file_content = re.sub(r"(embree[0-9]+_BINARY_DIR:STATIC=).*", os.path.join(r"\1"+cwd, "build"), file_content)
  file_content = re.sub(r"(CMAKE_CACHEFILE_DIR:INTERNAL=).*",   os.path.join(r"\1"+cwd, "build"), file_content)

  file_content = re.sub(r"(embree[0-9]+_SOURCE_DIR:STATIC=).*", r"\1"+cwd, file_content)
  file_content = re.sub(r"(CMAKE_HOME_DIRECTORY:INTERNAL=).*",  r"\1"+cwd, file_content)

  with open('build/CMakeCache.txt', 'w') as file:
      file.write(file_content)

# detect platform
if sys.platform.startswith("win"):
  import ctypes
  SEM_FAILCRITICALERRORS = 0x0001
  SEM_NOGPFAULTERRORBOX  = 0x0002
  SEM_NOOPENFILEERRORBOX = 0x8000
  ctypes.windll.kernel32.SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX | SEM_NOOPENFILEERRORBOX);
  OS = "windows"
elif sys.platform.startswith("cygwin"):
  import ctypes
  SEM_FAILCRITICALERRORS = 0x0001
  SEM_NOGPFAULTERRORBOX  = 0x0002
  SEM_NOOPENFILEERRORBOX = 0x8000
  ctypes.cdll.kernel32.SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX | SEM_NOOPENFILEERRORBOX);
  OS = "windows"
elif sys.platform.startswith("linux"):
  OS = "linux"
elif sys.platform.startswith("darwin"):
  OS = "macosx"
else:
  print("unknown platform: "+ sys.platform);
  sys.exit(1)

NAS = os.environ["STORAGE_PATH"] + "/packages/apps"
if "klocwork:ON" in sys.argv:
  NAS = "/NAS/packages/apps"

# configures tests for specified host machine
def runConfig(config):

  conf = []  # CMake configuration
  env = []  # shell environment
  rtcore = [] # rtcore configuration

  build = config["build"]
  conf.append("-D CMAKE_BUILD_TYPE="+build+"")

  cmake_build_suffix = ""
  threads = "0"

  if "threads" in config:
    threads = config["threads"]

  # TODO: not used in any of the workflow definitions
  if "memcheck" in config:
    conf.append("-D EMBREE_TESTING_MEMCHECK="+config["memcheck"]+"")

  # TODO: this is not used anywhere in embree cmake...
  if "sde" in config:
    conf.append("-D EMBREE_TESTING_SDE="+config["sde"]+"")

  if "addrsanitizer" in config:
    conf.append("-D EMBREE_ADDRESS_SANITIZER="+config["addrsanitizer"]+"")

  if "intensity" in config:
    g_intensity = config["intensity"]
  else:
    g_intensity = 2

  if "klocwork" in config:
    conf.append("-D EMBREE_TESTING_KLOCWORK="+config["klocwork"])

  if "L0RTAS" in config:
    conf.append("-D EMBREE_SYCL_L0_RTAS_BUILDER="+config["L0RTAS"])

  if "package" in config:
    conf.append("-D EMBREE_STACK_PROTECTOR=ON")

  if "maxinstancelevelcount" in config:
    conf.append("-D EMBREE_MAX_INSTANCE_LEVEL_COUNT="+config["maxinstancelevelcount"])

  enable_sycl_support = False
  if "EMBREE_SYCL_SUPPORT" in config:
    enable_sycl_support = True
    conf.append("-D EMBREE_SYCL_SUPPORT="+config["EMBREE_SYCL_SUPPORT"])
    conf.append("-D EMBREE_TESTING_ONLY_SYCL_TESTS=ON")

  #if "package" in config and OS == 'linux': # we need up to date cmake for RPMs to work properly
  compiler = config["compiler"]
  platform = config["platform"]
  if OS == "windows":
    cmake_build_suffix = "-- /m /t:rebuild"
    ext = ""
    if platform == "x64":
      ext = " Win64"
    if (compiler == "V142"):
      conf.append("-G \"Visual Studio 16 2019\"")
      conf.append("-T \"V142\"")
      conf.append("-A \"x64\"")
    elif (compiler == "V141"):
      conf.append("-G \"Visual Studio 15 2017"+ext+"\"")
      conf.append("-T \"V141\"")
    elif (compiler == "ICC19-VC141"):
      conf.append("-G \"Visual Studio 15 2017"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler 19.0\"")
    elif (compiler == "ICC18-VC141"):
      conf.append("-G \"Visual Studio 15 2017"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler 18.0\"")
    elif (compiler == "V140"):
      conf.append("-G \"Visual Studio 14 2015"+ext+"\"")
      conf.append("-T \"V140\"")
    elif (compiler == "V120"):
      conf.append("-G \"Visual Studio 12 2013"+ext+"\"")
      conf.append("-T \"V120\"")
    elif (compiler == "V110"):
      conf.append("-G \"Visual Studio 11 2012"+ext+"\"")
      conf.append("-T \"V110\"")
    elif (compiler == "ICC19-VC14"):
      conf.append("-G \"Visual Studio 14 2015"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler 19.0\"")
    elif (compiler == "ICC18-VC14"):
      conf.append("-G \"Visual Studio 14 2015"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler 18.0\"")
    elif (compiler == "ICC17-VC14"):
      conf.append("-G \"Visual Studio 14 2015"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler 17.0\"")
    elif (compiler == "ICC17-VC12"):
      conf.append("-G \"Visual Studio 12 2013"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler 17.0\"")
    elif (compiler == "ICC17"):
      conf.append("-G \"Visual Studio 12 2013"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler 17.0\"")
    elif (compiler == "ICC16"):
      conf.append("-G \"Visual Studio 12 2013"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler 16.0\"")
    elif (compiler == "ICC15"):
      conf.append("-G \"Visual Studio 12 2013"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler XE 15.0\"")
    elif (compiler == "LLVM_CLANG"):
      conf.append("-G \"Visual Studio 16 2019\"")
      conf.append("-A "+platform)
      conf.append("-T \"LLVM_v142\"")
    elif (compiler == "V141_CLANG"):
      conf.append("-G \"Visual Studio 15 2017"+ext+"\"")
      conf.append("-T \"v141_clang_c2\"")
    elif (compiler.startswith("ICX")):
      cmake_build_suffix = ""
      conf.append("-G Ninja -D CMAKE_CXX_COMPILER=icx -DCMAKE_C_COMPILER=icx")
    elif (compiler.startswith("dpcpp")):
      cmake_build_suffix=""
      conf.append("-G Ninja -D CMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang")
    else:
      raise ValueError('unknown compiler: ' + compiler + '')

  elif OS == "linux":
    if (compiler == "GCC"):
      conf.append("-D CMAKE_CXX_COMPILER=g++ -D CMAKE_C_COMPILER=gcc")
    elif (compiler == "CLANG"):
      conf.append("-D CMAKE_CXX_COMPILER=clang++ -D CMAKE_C_COMPILER=clang")
    elif (compiler.startswith("ICX")):
      conf.append("-G Ninja -DCMAKE_CXX_COMPILER=icpx -DCMAKE_C_COMPILER=icx")
    elif (compiler.startswith("ICC")):
      conf.append("-D CMAKE_CXX_COMPILER="+NAS+"/intel/"+compiler[3:]+"/bin/icpc -D CMAKE_C_COMPILER="+NAS+"/intel/"+compiler[3:]+"/bin/icc")
    elif (compiler.startswith("CLANG")):
      conf.append("-D CMAKE_CXX_COMPILER="+NAS+"/clang/v"+compiler[5:]+"/bin/clang++ -D CMAKE_C_COMPILER="+NAS+"/clang/v"+compiler[5:]+"/bin/clang")
    elif (compiler.startswith("dpcpp")):
      conf.append("-G Ninja -D CMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang")
    else:
      raise ValueError('unknown compiler: ' + compiler + '')

  else:
    if (compiler == "GCC"):
      conf.append("-D CMAKE_CXX_COMPILER=g++ -D CMAKE_C_COMPILER=gcc")
    elif (compiler == "CLANG"):
      conf.append("-D CMAKE_CXX_COMPILER=clang++ -D CMAKE_C_COMPILER=clang")
    elif (compiler.startswith("ICC")):
      conf.append("-D CMAKE_CXX_COMPILER="+NAS+"/intel/"+compiler[3:]+"-osx/compiler/latest/mac/bin/intel64/icpc -D CMAKE_C_COMPILER="+NAS+"/intel/"+compiler[3:]+"-osx/compiler/latest/mac/bin/intel64/icc")
    elif (compiler.startswith("ICX")):
      conf.append("-D CMAKE_CXX_COMPILER=/opt/intel/oneapi/compiler/"+compiler[3:]+"/mac/bin/intel64/icpc")
      conf.append("-D CMAKE_C_COMPILER=/opt/intel/oneapi/compiler/"+compiler[3:]+"/mac/bin/intel64/icc")
    else:
      raise ValueError('unknown compiler: ' + compiler + '')

  if "ispc" in config:
    conf.append("-D EMBREE_ISPC_SUPPORT=ON")
    ispc_compiler = config["ispc"]
    if ispc_compiler.startswith("ispc"):
  
      ispc_version = ispc_compiler[4:]
  
      if ispc_version != "":
  
        if OS == "windows": bin_folder = "bin\\"
        else              : bin_folder = "bin/"
        if parse_version(ispc_version) < parse_version("1.11.0"): bin_folder = ""
  
        if OS == "linux":
          conf.append("-D EMBREE_ISPC_EXECUTABLE="+NAS + "/ispc/"+ispc_version+"-linux/"+bin_folder+"ispc")
        elif OS == "macosx":
          conf.append("-D EMBREE_ISPC_EXECUTABLE="+NAS + "/ispc/"+ispc_version+"-osx/"+bin_folder+"ispc")
        elif OS == "windows":
          conf.append("-D EMBREE_ISPC_EXECUTABLE="+NAS+"\\ispc\\"+ispc_version+"-windows\\"+bin_folder+"ispc.exe")
        else:
          sys.stderr.write("unknown operating system "+OS)
          sys.exit(1)
    else:
      raise ValueError('unknown ISPC compiler: ' + ispc_compiler + '')
  else:
    conf.append("-D EMBREE_ISPC_SUPPORT=OFF")

  isa = config["isa"]
  if type(isa) == str:
    conf.append("-D EMBREE_MAX_ISA="+isa+"")
  else:
    conf.append("-D EMBREE_MAX_ISA=NONE")
    if "SSE2"      in isa: conf.append("-D EMBREE_ISA_SSE2=ON")
    else                 : conf.append("-D EMBREE_ISA_SSE2=OFF")
    if "SSE42"     in isa: conf.append("-D EMBREE_ISA_SSE42=ON")
    else                 : conf.append("-D EMBREE_ISA_SSE42=OFF")
    if "AVX"       in isa: conf.append("-D EMBREE_ISA_AVX=ON")
    else                 : conf.append("-D EMBREE_ISA_AVX=OFF")
    if "AVX2"      in isa: conf.append("-D EMBREE_ISA_AVX2=ON")
    else                 : conf.append("-D EMBREE_ISA_AVX2=OFF")
    if "AVX512"    in isa: conf.append("-D EMBREE_ISA_AVX512=ON")
    else                 : conf.append("-D EMBREE_ISA_AVX512=OFF")

  if "tasking" in config:
    tasking  = config["tasking"]
    if tasking == "INT":
      conf.append("-D EMBREE_TASKING_SYSTEM=INTERNAL")
    elif tasking == "PPL":
      conf.append("-D EMBREE_TASKING_SYSTEM=PPL")
    elif tasking.startswith("TBB"):
      conf.append("-D EMBREE_TASKING_SYSTEM=TBB")

      if OS == "linux":
        if tasking == "TBB":
          conf.append("-D EMBREE_TBB_ROOT=/usr")
        elif tasking.startswith("TBB"):
          conf.append("-D EMBREE_TBB_ROOT="+NAS+"/tbb/tbb-"+tasking[3:]+"-linux")
        else:
          raise ValueError('unknown tasking system: ' + tasking + '')

      elif OS == "macosx":
        if tasking == "TBB":
          conf.append("-D EMBREE_TBB_ROOT=/opt/local")
        elif tasking == "TBB_HOMEBREW":
          conf.append("-D EMBREE_TBB_ROOT=/opt/homebrew")
        elif tasking.startswith("TBB"):
          conf.append("-D EMBREE_TBB_ROOT="+NAS+"/tbb/tbb-"+tasking[3:]+"-osx")
        else:
          raise ValueError('unknown tasking system: ' + tasking + '')

      elif OS == "windows":
        tbb_path = ""+NAS+"\\tbb\\tbb-"+tasking[3:]+"-windows"
        conf.append("-D EMBREE_TBB_ROOT="+tbb_path)

        # prepend PATH modification to prevent problems with non-delayed
        # evaluation of variables in cmd when running oneAPI DPC++ compiler
        # setup script, for example.
        if platform == "x64":
          env.insert(0, "set PATH="+tbb_path+"\\bin\\intel64\\vc12;"+tbb_path+"\\bin\\intel64\\vc14;"+tbb_path+"\\redist\\intel64\\vc12;"+tbb_path+"\\redist\\intel64\\vc14;%PATH%")
        else:
          env.insert(0, "set PATH="+tbb_path+"\\bin\\ia32\\vc12;"+tbb_path+"\\bin\\ia32\\vc14;"+tbb_path+"\\redist\\ia32\\vc12;"+tbb_path+"\\redist\\ia32\\vc14;%PATH%")

      else:
        sys.stderr.write("unknown operating system "+OS)
        sys.exit(1)

    else:
      raise ValueError('unknown tasking system: ' + tasking)

  if "api_namespace" in config:
    conf.append("-D EMBREE_API_NAMESPACE="+config["api_namespace"])
    conf.append("-D EMBREE_LIBRARY_NAME="+config["api_namespace"])  # we test different library name at the same time
    conf.append("-D EMBREE_ISPC_SUPPORT=OFF")

  # Add embree specific optons
  for opt in ["LEVEL_ZERO", "STATIC_LIB", "TUTORIALS", "BACKFACE_CULLING", "BACKFACE_CULLING_CURVES", "IGNORE_INVALID_RAYS", "FILTER_FUNCTION", "LARGEGRF", "RAY_MASK", "RAY_PACKETS", "STAT_COUNTERS", "COMPACT_POLYS", "MIN_WIDTH"]:
    if opt in config:
      conf.append("-D EMBREE_"+opt+"="+config[opt])

  # TODO: check if we want to chonge the names of theese options, so that the pattern fits here as well
  if "TRI" in config:
    conf.append("-D EMBREE_GEOMETRY_TRIANGLE="+config["TRI"])
  if "QUAD" in config:
    conf.append("-D EMBREE_GEOMETRY_QUAD="+config["QUAD"])
  if "GRID" in config:
    conf.append("-D EMBREE_GEOMETRY_GRID="+config["GRID"])
  if "CURVE" in config:
    conf.append("-D EMBREE_GEOMETRY_CURVE="+config["CURVE"])
  if "SUBDIV" in config:
    conf.append("-D EMBREE_GEOMETRY_SUBDIVISION="+config["SUBDIV"])
  if "USERGEOM" in config:
    conf.append("-D EMBREE_GEOMETRY_USER="+config["USERGEOM"])
  if "INSTANCE" in config:
    conf.append("-D EMBREE_GEOMETRY_INSTANCE="+config["INSTANCE"])
  if "INSTANCE_ARRAY" in config:
    conf.append("-D EMBREE_GEOMETRY_INSTANCE_ARRAY="+config["INSTANCE_ARRAY"])
  if "POINT" in config:
    conf.append("-D EMBREE_GEOMETRY_POINT="+config["POINT"])
  if "GLFW" in config:
    conf.append("-D EMBREE_TUTORIALS_GLFW="+config["GLFW"])
  if "frequency_level" in config:
    rtcore.append("frequency_level="+config["frequency_level"])

  if "sycl" in config:
      conf.append("-D EMBREE_SYCL_AOT_DEVICES="+config["sycl"])
  if "implicit_dispatch_globals" in config:
    conf.append("-D EMBREE_SYCL_IMPLICIT_DISPATCH_GLOBALS="+config["implicit_dispatch_globals"])
  if "sycl_test" in config:
    conf.append("-D EMBREE_SYCL_TEST="+config["sycl_test"])
  if "rt_validation_api" in config:
    conf.append("-D EMBREE_SYCL_RT_VALIDATION_API="+config["rt_validation_api"])
  if "test_only_sycl" in config:
    conf.append("-D EMBREE_TESTING_ONLY_SYCL_TESTS="+config["test_only_sycl"])
  else:
    conf.append("-D EMBREE_TESTING_ONLY_SYCL_TESTS=OFF")

  if "EMBREE_USE_GOOGLE_BENCHMARK" in config:
    conf.append("-D EMBREE_USE_GOOGLE_BENCHMARK="+config["EMBREE_USE_GOOGLE_BENCHMARK"])
  else:
    conf.append("-D EMBREE_USE_GOOGLE_BENCHMARK=OFF")
  if "EMBREE_GOOGLE_BENCHMARK_DIR" in config:
    conf.append("-D benchmark_DIR:PATH="+config["EMBREE_GOOGLE_BENCHMARK_DIR"])

  if "EMBREE_BUILD_GOOGLE_BENCHMARK_FROM_SOURCE" in config:
    conf.append("-D EMBREE_BUILD_GOOGLE_BENCHMARK_FROM_SOURCE=ON")
  else:
    conf.append("-D EMBREE_BUILD_GOOGLE_BENCHMARK_FROM_SOURCE=OFF")

  if "package" in config:
    SIGN_FILE = ""
    if OS == "windows":
      SIGN_FILE = os.environ["SIGN_FILE_WINDOWS"]
    elif OS == "linux":
      SIGN_FILE = os.environ["SIGN_FILE_LINUX"]
    elif OS == "macosx":
      SIGN_FILE = os.environ["SIGN_FILE_MAC"]
    conf.append("-D EMBREE_SIGN_FILE="+SIGN_FILE)

    conf.append("-D BUILD_TESTING=ON")
    conf.append("-D EMBREE_TESTING_INSTALL_TESTS=ON")
    conf.append("-D EMBREE_TESTING_PACKAGE=ON")
    if OS == "linux" and config["package"] == "ZIP":
      conf.append("-D EMBREE_INSTALL_DEPENDENCIES=ON")
      conf.append("-D EMBREE_BUILD_GLFW_FROM_SOURCE=ON")
      conf.append("-D EMBREE_ZIP_MODE=ON")
      conf.append("-D CMAKE_SKIP_INSTALL_RPATH=OFF")
      conf.append("-D CMAKE_INSTALL_INCLUDEDIR=include")
      conf.append("-D CMAKE_INSTALL_LIBDIR=lib")
      conf.append("-D CMAKE_INSTALL_DOCDIR=doc")
      conf.append("-D CMAKE_INSTALL_BINDIR=bin")
      conf.append("-D CMAKE_INSTALL_TESTDIR=testing")
    elif OS == "macosx" and config["package"] == "ZIP":
      conf.append("-D EMBREE_INSTALL_DEPENDENCIES=ON")
      conf.append("-D EMBREE_ZIP_MODE=ON")
      conf.append("-D CMAKE_SKIP_INSTALL_RPATH=OFF")
      conf.append("-D CMAKE_MACOSX_RPATH=ON")
      conf.append("-D CMAKE_INSTALL_INCLUDEDIR=include")
      conf.append("-D CMAKE_INSTALL_LIBDIR=lib")
      conf.append("-D CMAKE_INSTALL_DOCDIR=doc")
      conf.append("-D CMAKE_INSTALL_BINDIR=bin")
      conf.append("-D CMAKE_INSTALL_TESTDIR=testing")
    elif OS == "windows" and config["package"] == "ZIP":
      conf.append("-D EMBREE_INSTALL_DEPENDENCIES=ON")
      conf.append("-D EMBREE_BUILD_GLFW_FROM_SOURCE=ON")
      conf.append("-D EMBREE_ZIP_MODE=ON")
      conf.append("-D CMAKE_INSTALL_INCLUDEDIR=include")
      conf.append("-D CMAKE_INSTALL_LIBDIR=lib")
      conf.append("-D CMAKE_INSTALL_DATAROOTDIR=")
      conf.append("-D CMAKE_INSTALL_DOCDIR=doc")
      conf.append("-D CMAKE_INSTALL_BINDIR=bin")
      conf.append("-D CMAKE_INSTALL_TESTDIR=testing")
    else:
      sys.stderr.write("unknown package mode: "+OS+":"+config["package"])
      sys.exit(1)

  if rtcore:
    conf.append("-D EMBREE_CONFIG="+(",".join(rtcore)))
    
  if "dumptests" in config:
    conf.append("-D EMBREE_TESTING_DUMPTESTS=" + config["dumptests"])
  if "dumpformat" in config:
    conf.append("-D EMBREE_TESTING_DUMPFORMAT=" + config["dumpformat"])

  ctest_suffix = ""
  ctest_suffix += " -D EMBREE_TESTING_INTENSITY="+str(g_intensity)
  if "klocwork" in config:
    ctest_suffix += " -D EMBREE_TESTING_KLOCWORK="+config["klocwork"]

  ctest_suffix += " -D CTEST_CONFIGURATION_TYPE=\""+build+"\""
  ctest_suffix += " -D CTEST_BUILD_OPTIONS=\"" + escape(" ".join(conf))+"\""
  ctest_env = ""
  for e in env:
    ctest_env += e + " && "

  ctest_conf = [ctest_env, ctest_suffix, cmake_build_suffix, threads]
  pickle.dump(ctest_conf, open(".ctest_conf", "wb"), 0)

# builds or runs tests for specified host machine
def run(mode):

  [ctest_env, ctest_suffix, cmake_build_suffix, threads] = pickle.load(open(".ctest_conf", "rb"))

  # pick sde executable
  # don't use sde if no sde cpuid is specified
  sde_cmd = ""
  if mode == "test" and "sde" in g_config:
    if OS == "linux":
      sde_cmd = os.path.sep.join([NAS, 'sde', 'lin', 'sde64'])
    elif OS == "macosx":
      sde_cmd = os.path.sep.join([NAS, 'sde', 'mac', 'sde64'])
    elif OS == "windows":
      sde_cmd = os.path.sep.join([NAS, 'sde', 'win', 'sde64'])
    else:
      sys.stderr.write("unknown operating system "+OS)
      sys.exit(1)
    
    sde_cmd = sde_cmd + " -" + g_config['sde'] + " -- "

  if mode == "test" or mode == "build":
    cmd = ctest_env + sde_cmd + "ctest -VV -S "+ os.path.join("scripts","test.cmake -DSTAGE="+mode+" -DTHREADS="+threads+" -DBUILD_SUFFIX=\""+cmake_build_suffix+"\"") + ctest_suffix
  else:
    cmd = ctest_env + os.path.join("scripts",mode)

  if mode == "env":
    cmd = ctest_env + "echo env";

  if mode == "test" and not OS == "windows":
    fix_cmake_paths()

  # execute step
  if (g_debugMode):
    print(cmd)
  else:
    try:
      if OS == "windows":
        subprocess.check_call(cmd, stderr=subprocess.STDOUT, shell=True)
      else:
        subprocess.check_call(cmd, stderr=subprocess.STDOUT, shell=True, executable='/bin/bash')
    except subprocess.CalledProcessError as e:
      sys.stderr.write("windows test invocation failed with return code "+str(e.returncode))
      sys.exit(1)

g_config = {}
def parseCommandLine(argv):
  global g_config
  global g_debugMode
  if len(argv) == 0:
    return;
  elif len(argv)>=1 and argv[0] == "--debug":
    g_debugMode = True
    parseCommandLine(argv[1:len(argv)])
  elif len(argv)>=1 and argv[0] == "--help":
    return
  elif ':' in argv[0]:
    p = argv[0].split(":")
    if p[0] == "isas":
      g_config["isa"] = p[1].split('-')
    else:
      g_config[p[0]] = p[1]
    parseCommandLine(argv[1:len(argv)])
  else:
    sys.stderr.write("unknown command line option: "+argv[0])
    sys.exit(1)

argv = sys.argv
g_mode = ""
if len(argv) < 2:
  sys.exit(1)
else:
  g_mode = argv[1]
  parseCommandLine(argv[2:len(argv)])

if (g_mode == "configure"):
  runConfig(g_config)
else:
  run(g_mode)
