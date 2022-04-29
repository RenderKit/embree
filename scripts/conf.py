#!/usr/bin/python

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

import sys
import os
import ctypes
import time
import datetime
import json
import socket
import subprocess

g_debugMode = False
g_benchmarkMode = False

def escape(str):
  str = str.replace("\\",r"\\")
  str = str.replace("\"",r"\"")
  return str

def parse_version(v):
  return tuple(map(int, v.split(".")))

# detect platform
if sys.platform.startswith("win"):
  dash = '\\'
  SEM_FAILCRITICALERRORS = 0x0001
  SEM_NOGPFAULTERRORBOX  = 0x0002
  SEM_NOOPENFILEERRORBOX = 0x8000
  ctypes.windll.kernel32.SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX | SEM_NOOPENFILEERRORBOX);
  OS = "windows"
elif sys.platform.startswith("cygwin"):
  dash = '/'
  SEM_FAILCRITICALERRORS = 0x0001
  SEM_NOGPFAULTERRORBOX  = 0x0002
  SEM_NOOPENFILEERRORBOX = 0x8000
  ctypes.cdll.kernel32.SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX | SEM_NOOPENFILEERRORBOX);
  OS = "windows"
elif sys.platform.startswith("linux"):
  dash = '/'
  OS = "linux"
elif sys.platform.startswith("darwin"):
  dash = '/'
  OS = "macosx"
else:
  print("unknown platform: "+ sys.platform);
  sys.exit(1)

if OS == "windows":
  nas_windows = os.environ["NAS_WINDOWS"]
elif OS == "linux":
  nas_linux = os.environ["NAS_LINUX"]
elif OS == "macosx":
  nas_macosx = os.environ["NAS_MACOSX"]

# runs all tests for specified host machine
def runConfig(config):

  conf = []  # CMake configuration
  ctest_conf = []  # CMake configuration
  env = []  # shell environment
  rtcore = [] # rtcore configuration

  build = config["build"]
  conf.append("-D CMAKE_BUILD_TYPE="+build+"")

  if "memcheck" in config:
    conf.append("-D EMBREE_TESTING_MEMCHECK="+config["memcheck"]+"")

  if "sde" in config:
    conf.append("-D EMBREE_TESTING_SDE="+config["sde"]+"")

  if "addrsanitizer" in config:
    conf.append("-D EMBREE_ADDRESS_SANITIZER="+config["addrsanitizer"]+"")

  if "package" in config:
    conf.append("-D EMBREE_STACK_PROTECTOR=ON")

  if "maxinstancelevelcount" in config:
    conf.append("-D EMBREE_MAX_INSTANCE_LEVEL_COUNT="+config["maxinstancelevelcount"])

  if "klocwork" in config:
    ctest_conf.append("-D EMBREE_TESTING_KLOCWORK="+config["klocwork"])
    conf.append("-D EMBREE_TESTING_KLOCWORK="+config["klocwork"])

  if "update_models" in config:
    ctest_conf.append("-D EMBREE_UPDATE_MODELS="+config["update_models"])
  else:
    ctest_conf.append("-D EMBREE_UPDATE_MODELS=ON")

  if "intensity" in config:
    ctest_conf.append("-D EMBREE_TESTING_INTENSITY="+config["intensity"])
    conf      .append("-D EMBREE_TESTING_INTENSITY="+config["intensity"])
  else:
    ctest_conf.append("-D EMBREE_TESTING_INTENSITY=2")
    conf      .append("-D EMBREE_TESTING_INTENSITY=2")

  if g_benchmarkMode:
    conf.append("-D EMBREE_USE_GOOGLE_BENCHMARK=ON")
    conf.append("-D benchmark_DIR:PATH=/NAS/packages/apps/google-benchmark/vis-perf-x8280-1/lib64/cmake/benchmark")

  #if "package" in config and OS == 'linux': # we need up to date cmake for RPMs to work properly
  #  env.append("module load cmake")
  compiler = config["compiler"]

  platform = "x64"
  if "platform" in config:
    platform = config["platform"]

  ispc_ext = "-vs2013"
  if OS == "windows":
    ext = ""
    if platform == "x64":
      ext = " Win64"
    if (compiler == "V142"):
      conf.append("-G \"Visual Studio 16 2019\"")
      conf.append("-T \"V142\"")
      conf.append("-A \"x64\"")
      ispc_ext = "-vs2015"
    elif (compiler == "V141"):
      conf.append("-G \"Visual Studio 15 2017"+ext+"\"")
      conf.append("-T \"V141\"")
      ispc_ext = "-vs2015"
    elif (compiler == "ICC19-VC141"):
      conf.append("-G \"Visual Studio 15 2017"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler 19.0\"")
      ispc_ext = "-vs2015"
    elif (compiler == "ICC18-VC141"):
      conf.append("-G \"Visual Studio 15 2017"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler 18.0\"")
      ispc_ext = "-vs2015"
    elif (compiler == "V140"):
      conf.append("-G \"Visual Studio 14 2015"+ext+"\"")
      conf.append("-T \"V140\"")
      ispc_ext = "-vs2015"
    elif (compiler == "V120"):
      conf.append("-G \"Visual Studio 12 2013"+ext+"\"")
      conf.append("-T \"V120\"")
    elif (compiler == "V110"):
      conf.append("-G \"Visual Studio 11 2012"+ext+"\"")
      conf.append("-T \"V110\"")
    elif (compiler == "ICC19-VC14"):
      conf.append("-G \"Visual Studio 14 2015"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler 19.0\"")
      ispc_ext = "-vs2015"
    elif (compiler == "ICC18-VC14"):
      conf.append("-G \"Visual Studio 14 2015"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler 18.0\"")
      ispc_ext = "-vs2015"
    elif (compiler == "ICC17-VC14"):
      conf.append("-G \"Visual Studio 14 2015"+ext+"\"")
      conf.append("-T \"Intel C++ Compiler 17.0\"")
      ispc_ext = "-vs2015"
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
      ispc_ext = "-vs2015"
    elif (compiler == "V141_CLANG"):
      conf.append("-G \"Visual Studio 15 2017"+ext+"\"")
      conf.append("-T \"v141_clang_c2\"")
      ispc_ext = "-vs2015"
    else:
      raise ValueError('unknown compiler: ' + compiler + '')

  elif OS == "linux":
    if (compiler == "GCC"):
      conf.append("-D CMAKE_CXX_COMPILER=g++ -D CMAKE_C_COMPILER=gcc")
    elif (compiler == "CLANG"):
      conf.append("-D CMAKE_CXX_COMPILER=clang++ -D CMAKE_C_COMPILER=clang")
    elif (compiler.startswith("ICC")):
      conf.append("-D CMAKE_CXX_COMPILER="+nas_linux+"/intel/"+compiler[3:]+"/bin/icpc -D CMAKE_C_COMPILER="+nas_linux+"/intel/"+compiler[3:]+"/bin/icc")
    elif (compiler.startswith("CLANG")):
      conf.append("-D CMAKE_CXX_COMPILER="+nas_linux+"/clang/v"+compiler[5:]+"/bin/clang++ -D CMAKE_C_COMPILER="+nas_linux+"/clang/v"+compiler[5:]+"/bin/clang")
    elif (compiler.startswith("dpcpp")):
      # set up dpcpp version
      if (compiler[5:] != ""):
        os.environ["DPCPP_VERSION"] = compiler[6:] # [6:] to parse dpcpp/
      elif (not "DPCPP_VERSION" in os.environ):
        # if DPCPP_VERSION is not already as environment variable read version from .ci-env.yaml file
        with open(".ci-env.yaml") as f:
          for line in f:
            if not line.startswith("DPCPP_VERSION"):
              continue
            key, value = line.strip().split(":", 1)
            os.environ[key] = value
      DPCPP_VERSION=os.environ['DPCPP_VERSION']

      # set up igc version
      if "igc" in config:
        os.environ["IGC_VERSION"] = config["igc"]
      elif (not "IGC_VERSION" in os.environ):
        # if DPCPP_VERSION is not already as environment variable read version from .ci-env.yaml file
        with open(".ci-env.yaml") as f:
          for line in f:
            if not line.startswith("IGC_VERSION"):
              continue
            key, value = line.strip().split(":", 1)
            os.environ[key] = value
      IGC_VERSION=os.environ['IGC_VERSION']

      # set up backend
      if "backend" in config: # opencl or level_zero
        print("using "+config["backend"]+" backend")
        env.append("SYCL_DEVICE_FILTER="+config["backend"])
      else:
        print("using level_zero backend")
        env.append("SYCL_DEVICE_FILTER=level_zero")

      dpcpp_dir = ""
      igc_dir = ""+nas_linux+"/gfx-driver/"+IGC_VERSION+"/install"
      
      if (compiler.startswith("dpcpp/internal")):
        dpcpp_dir = ""+nas_linux+"/dpcpp_compiler/"+DPCPP_VERSION
        conf.append("-D CMAKE_CXX_COMPILER="+dpcpp_dir+"/bin/dpcpp")
        #conf.append("-D CMAKE_C_COMPILER="  +dpcpp_dir+"/bin/dpcpp")
      else:
        dpcpp_dir = ""+nas_linux+"/dpcpp_compiler/"+DPCPP_VERSION+"/dpcpp_compiler"
        conf.append("-D CMAKE_CXX_COMPILER="+dpcpp_dir+"/bin/clang++")
        conf.append("-D CMAKE_C_COMPILER="  +dpcpp_dir+"/bin/clang")

      sys.stderr.write("dpcpp_dir = "+dpcpp_dir)      
        
      env.append("PATH="+dpcpp_dir+"/bin:"+dpcpp_dir+"/bin-llvm:"+igc_dir+"/usr/bin:"+igc_dir+"/usr/local/bin:$PATH")
      env.append("CPATH="+dpcpp_dir+"/include/sycl:" + dpcpp_dir+"/include")
      env.append("LD_LIBRARY_PATH="+dpcpp_dir+"/lib:"+dpcpp_dir+"/compiler/lib/intel64_lin:"+igc_dir+"/usr/lib/x86_64-linux-gnu:" + igc_dir+"/usr/local/lib:$LD_LIBRARY_PATH")
      env.append("OCL_ICD_FILENAMES="+igc_dir+"/usr/lib/x86_64-linux-gnu/intel-opencl/libigdrcl.so"+":"+igc_dir+"/usr/local/lib/intel-opencl/libigdrcl.so")
      env.append("OCL_ICD_VENDORS="+igc_dir+"/etc/OpenCL/vendors/intel.icd")
      env.append("OPENCL_INCLUDE_DIR="+dpcpp_dir+"/include/sycl")
      env.append("OPENCL_LIBRARY="+dpcpp_dir+"/lib/libOpenCL.so")
    else:
      raise ValueError('unknown compiler: ' + compiler + '')

  else:
    if (compiler == "GCC"):
      conf.append("-D CMAKE_CXX_COMPILER=g++ -D CMAKE_C_COMPILER=gcc")
    elif (compiler == "CLANG"):
      conf.append("-D CMAKE_CXX_COMPILER=clang++ -D CMAKE_C_COMPILER=clang")
    elif (compiler.startswith("ICC")):
      conf.append("-D CMAKE_CXX_COMPILER="+nas_macosx+"/intel/"+compiler[3:]+"-osx/compiler/latest/mac/bin/intel64/icpc -D CMAKE_C_COMPILER="+nas_macosx+"/intel/"+compiler[3:]+"-osx/compiler/latest/mac/bin/intel64/icc")
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
          conf.append("-D EMBREE_ISPC_EXECUTABLE="+nas_linux + "/ispc/"+ispc_version+"-linux/"+bin_folder+"ispc")
        elif OS == "macosx":
          conf.append("-D EMBREE_ISPC_EXECUTABLE="+nas_macosx + "/ispc/"+ispc_version+"-osx/"+bin_folder+"ispc")
        elif OS == "windows":
          conf.append("-D EMBREE_ISPC_EXECUTABLE="+nas_windows+"\\ispc\\" + ispc_version+"-windows"+ispc_ext+"\\"+bin_folder+"ispc.exe")
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
          conf.append("-D EMBREE_TBB_ROOT="+nas_linux+"/tbb/tbb-"+tasking[3:]+"-linux")
        else:
          raise ValueError('unknown tasking system: ' + tasking + '')

      elif OS == "macosx":
        if tasking == "TBB":
          conf.append("-D EMBREE_TBB_ROOT=/opt/local")
        elif tasking.startswith("TBB"):
          conf.append("-D EMBREE_TBB_ROOT="+nas_macosx+"/tbb/tbb-"+tasking[3:]+"-osx")
        else:
          raise ValueError('unknown tasking system: ' + tasking + '')

      elif OS == "windows":
        if not tasking.startswith("TBB"):
          raise ValueError('unknown tasking system: ' + tasking + '')

        tbb_path = ""+nas_windows+"\\tbb\\tbb-"+tasking[3:]+"-windows"
        conf.append("-D EMBREE_TBB_ROOT="+tbb_path)

        if platform == "x64":
          env.append("set PATH="+tbb_path+"\\bin\\intel64\\vc12;"+tbb_path+"\\bin\\intel64\\vc14;"+tbb_path+"\\redist\\intel64\\vc12;"+tbb_path+"\\redist\\intel64\\vc14;%PATH%")
        else:
          env.append("set PATH="+tbb_path+"\\bin\\ia32\\vc12;"+tbb_path+"\\bin\\ia32\\vc14;"+tbb_path+"\\redist\\ia32\\vc12;"+tbb_path+"\\redist\\ia32\\vc14;%PATH%")

      else:
        sys.stderr.write("unknown operating system "+OS)
        sys.exit(1)

    else:
      raise ValueError('unknown tasking system: ' + tasking)

  if "api_namespace" in config:
    conf.append("-D EMBREE_API_NAMESPACE="+config["api_namespace"])
    conf.append("-D EMBREE_LIBRARY_NAME="+config["api_namespace"])  # we test different library name at the same time
    conf.append("-D EMBREE_ISPC_SUPPORT=OFF")
  if "STATIC_LIB" in config:
    conf.append("-D EMBREE_STATIC_LIB="+config["STATIC_LIB"])
  if "TUTORIALS" in config:
    conf.append("-D EMBREE_TUTORIALS="+config["TUTORIALS"])
  if "BACKFACE_CULLING" in config:
    conf.append("-D EMBREE_BACKFACE_CULLING="+config["BACKFACE_CULLING"])
  if "BACKFACE_CULLING_CURVES" in config:
    conf.append("-D EMBREE_BACKFACE_CULLING_CURVES="+config["BACKFACE_CULLING_CRUVES"])
  if "IGNORE_INVALID_RAYS" in config:
    conf.append("-D EMBREE_IGNORE_INVALID_RAYS="+config["IGNORE_INVALID_RAYS"])
  if "FILTER_FUNCTION" in config:
    conf.append("-D EMBREE_FILTER_FUNCTION="+config["FILTER_FUNCTION"])
  if "RAY_MASK" in config:
    conf.append("-D EMBREE_RAY_MASK="+config["RAY_MASK"])
  if "RAY_PACKETS" in config:
    conf.append("-D EMBREE_RAY_PACKETS="+config["RAY_PACKETS"])
  if "STAT_COUNTERS" in config:
    conf.append("-D EMBREE_STAT_COUNTERS="+config["STAT_COUNTERS"])
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
  if "USERGEOM_CONTEXT" in config:
    conf.append("-D EMBREE_GEOMETRY_USER_IN_CONTEXT="+config["USERGEOM_CONTEXT"])
  if "INSTANCE" in config:
    conf.append("-D EMBREE_GEOMETRY_INSTANCE="+config["INSTANCE"])
  if "POINT" in config:
    conf.append("-D EMBREE_GEOMETRY_POINT="+config["POINT"])
  if "COMPACT_POLYS" in config:
    conf.append("-D EMBREE_COMPACT_POLYS="+config["COMPACT_POLYS"])
  if "MIN_WIDTH" in config:
    conf.append("-D EMBREE_MIN_WIDTH="+config["MIN_WIDTH"])
  if "GLFW" in config:
    conf.append("-D EMBREE_TUTORIALS_GLFW="+config["GLFW"])
  if "frequency_level" in config:
    rtcore.append("frequency_level="+config["frequency_level"])

  if "sycl" in config:
      conf.append("-D EMBREE_DPCPP_AOT_DEVICES="+config["sycl"])
  if "sycl_mblur" in config:
    conf.append("-D EMBREE_DPCPP_MBLUR="+config["sycl_mblur"])
  if "sycl_simd_width" in config:
    conf.append("-D EMBREE_DPCPP_SIMD_WIDTH="+config["sycl_simd_width"])
  if "sycl_test" in config:
    conf.append("-D EMBREE_SYCL_TEST="+config["sycl_test"])
  if "EMBREE_DPCPP_SUPPORT" in config:
    conf.append("-D EMBREE_DPCPP_SUPPORT="+config["EMBREE_DPCPP_SUPPORT"])
  else:
    conf.append("-D EMBREE_DPCPP_SUPPORT=OFF")

  if "package" in config:
    conf.append("-D EMBREE_TESTING_PACKAGE=ON")
    conf.append("-D EMBREE_TUTORIALS_OPENIMAGEIO=OFF")
    conf.append("-D EMBREE_TUTORIALS_LIBJPEG=OFF")
    conf.append("-D EMBREE_TUTORIALS_LIBPNG=OFF")
    if OS == "linux" and config["package"] == "ZIP":
      conf.append("-D EMBREE_SIGN_FILE="+nas_linux+"/signfile/linux/SignFile")
      conf.append("-D EMBREE_INSTALL_DEPENDENCIES=ON")
      conf.append("-D EMBREE_ZIP_MODE=ON")
      conf.append("-D CMAKE_SKIP_INSTALL_RPATH=OFF")
      conf.append("-D CMAKE_INSTALL_INCLUDEDIR=include")
      conf.append("-D CMAKE_INSTALL_LIBDIR=lib")
      conf.append("-D CMAKE_INSTALL_DOCDIR=doc")
      conf.append("-D CMAKE_INSTALL_BINDIR=bin")
    elif OS == "linux" and config["package"] == "RPM":
      conf.append("-D EMBREE_SIGN_FILE="+nas_linux+"/signfile/linux/SignFile")
      conf.append("-D EMBREE_INSTALL_DEPENDENCIES=OFF")
      conf.append("-D EMBREE_ZIP_MODE=OFF")
      conf.append("-D CMAKE_SKIP_INSTALL_RPATH=OFF")
      conf.append("-D CMAKE_INSTALL_PREFIX=/usr")
      conf.append("-D EMBREE_TBB_ROOT=/usr")
    elif OS == "macosx" and config["package"] == "ZIP":
      conf.append("-D EMBREE_SIGN_FILE="+nas_macosx+"/signfile/mac/SignFile")
      conf.append("-D EMBREE_INSTALL_DEPENDENCIES=ON")
      conf.append("-D EMBREE_ZIP_MODE=ON")
      conf.append("-D CMAKE_SKIP_INSTALL_RPATH=OFF")
      conf.append("-D CMAKE_MACOSX_RPATH=ON")
      conf.append("-D CMAKE_INSTALL_INCLUDEDIR=include")
      conf.append("-D CMAKE_INSTALL_LIBDIR=lib")
      conf.append("-D CMAKE_INSTALL_DOCDIR=doc")
      conf.append("-D CMAKE_INSTALL_BINDIR=bin")
    elif OS == "macosx" and config["package"] == "PKG":
      conf.append("-D EMBREE_SIGN_FILE="+nas_macosx+"/signfile/mac/SignFile")
      conf.append("-D EMBREE_INSTALL_DEPENDENCIES=OFF")
      conf.append("-D EMBREE_ZIP_MODE=OFF")
      conf.append("-D CMAKE_SKIP_INSTALL_RPATH=OFF")
      conf.append("-D CMAKE_MACOSX_RPATH=ON")
      conf.append("-D CMAKE_INSTALL_PREFIX=/opt/local")
      conf.append("-D CMAKE_INSTALL_INCLUDEDIR=include")
      conf.append("-D CMAKE_INSTALL_LIBDIR=lib")
      conf.append("-D CMAKE_INSTALL_DOCDIR=../../Applications/Embree3/doc")
      conf.append("-D CMAKE_INSTALL_BINDIR=../../Applications/Embree3/bin")
    elif OS == "windows" and config["package"] == "ZIP":
      conf.append("-D EMBREE_SIGN_FILE="+nas_windows+"\\signfile\\windows\\SignFile.exe")
      conf.append("-D EMBREE_INSTALL_DEPENDENCIES=ON")
      conf.append("-D EMBREE_ZIP_MODE=ON")
      conf.append("-D CMAKE_INSTALL_INCLUDEDIR=include")
      conf.append("-D CMAKE_INSTALL_LIBDIR=lib")
      conf.append("-D CMAKE_INSTALL_DATAROOTDIR=")
      conf.append("-D CMAKE_INSTALL_DOCDIR=doc")
      conf.append("-D CMAKE_INSTALL_BINDIR=bin")
    elif OS == "windows" and config["package"] == "MSI":
      conf.append("-D EMBREE_SIGN_FILE="+nas_windows+"\\signfile\\windows\\SignFile.exe")
      conf.append("-D EMBREE_INSTALL_DEPENDENCIES=ON")
      conf.append("-D EMBREE_ZIP_MODE=OFF")
      conf.append("-D CMAKE_INSTALL_INCLUDEDIR=include")
      conf.append("-D CMAKE_INSTALL_LIBDIR=lib")
      conf.append("-D CMAKE_INSTALL_DATAROOTDIR=")
      conf.append("-D CMAKE_INSTALL_DOCDIR=doc")
      conf.append("-D CMAKE_INSTALL_BINDIR=bin")
    else:
      sys.stderr.write("unknown package mode: "+OS+":"+config["package"])
      sys.exit(1)

  if OS == "linux":
    # some additional debug output of igc and dpcpp version
    if (compiler.startswith("dpcpp")):
      if (compiler.startswith("dpcpp/internal")):
        which_dpcpp = str(subprocess.check_output(escape(" ".join(env)) + " which dpcpp", shell=True, stderr=subprocess.PIPE).decode('utf-8').rstrip("\n"))
        print("DEBUG - DPCPP version:", DPCPP_VERSION, " - which dpcpp: ", which_dpcpp)
        assert which_dpcpp == nas_linux+"/dpcpp_compiler/"+DPCPP_VERSION+"/bin/dpcpp"
      else:
        which_clang = str(subprocess.check_output(escape(" ".join(env)) + " which clang++", shell=True, stderr=subprocess.PIPE).decode('utf-8').rstrip("\n"))
        print("DEBUG - DPCPP version:", DPCPP_VERSION, " - which clang++: ", which_clang)
        assert which_clang == nas_linux+"/dpcpp_compiler/"+DPCPP_VERSION+"/dpcpp_compiler/bin/clang++"
        
      which_ocloc = str(subprocess.check_output(escape(" ".join(env)) + " which ocloc", shell=True, stderr=subprocess.PIPE).decode('utf-8').rstrip("\n"))
      print("DEBUG - IGC version:", IGC_VERSION, " - which ocloc: ", which_ocloc)
      assert which_ocloc == nas_linux+"/gfx-driver/"+IGC_VERSION+"/install/usr/bin/ocloc" or which_ocloc == nas_linux+"/gfx-driver/"+IGC_VERSION+"/install/usr/local/bin/ocloc"

  if rtcore:
    conf.append("-D EMBREE_CONFIG="+(",".join(rtcore)))

  ctest_conf.append("-D CTEST_CONFIGURATION_TYPE='"+build+"'")
  ctest_conf.append("-D CTEST_BUILD_OPTIONS='" + escape(" ".join(conf))+"'")

  with open(".ctest_env", "w+") as file:
    json.dump(env, file)
  with open(".ctest_conf", "w+") as file:
    json.dump(ctest_conf, file)

g_config = {}
def parseCommandLine(argv):
  global g_config
  global g_debugMode
  if len(argv) == 0:
    return;
  elif len(argv)>=1 and argv[0] == "--debug":
    g_debugMode = True
    parseCommandLine(argv[1:len(argv)])
  elif len(argv)>=1 and argv[0] == "--benchmark":
    g_benchmarkMode = True
    parseCommandLine(argv[1:len(argv)])
  elif len(argv)>=1 and argv[0] == "--help":
    printUsage()
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

parseCommandLine(sys.argv[1:len(sys.argv)])
runConfig(g_config)
