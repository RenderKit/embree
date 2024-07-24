#! /usr/bin/env python3

import os
import sys
import platform
import subprocess
import shutil

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

class CMakeExtension(Extension):
    def __init__(self, name):
        Extension.__init__(self, name, sources=[])


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                "Cmake must be installed to build the following extensions: \n"
                "\n  ".join(e.name for e in self.extensions)
            )

        for ext in self.extensions:
            self.build_extension(ext)
        

    def build_extension(self, ext):
        extdir = os.path.join(os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name))), ext.name)
        print("AOEUAOEUAOEUAOEU")
        print("AOEUAOEUAOEUAOEU")
        print("AOEUAOEUAOEUAOEU")
        print("extdir")
        print(extdir)

        cmake_args = [
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE=' + extdir,
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG=' + extdir,
            #'-DPYTHON_EXECUTABLE=' + sys.executable,
            #'-G Ninja',
        ]

        ## for SYCL
        #cmake_args += [
        #    '-DCMAKE_CXX_COMPILER=clang++',
        #    '-DCMAKE_C_COMPILER=clang',
        #]


        print("debug")
        print(self.debug)

        cfg = 'Debug' if self.debug else 'Release'
        cmake_args += ['DCMAKE_BUILD_TYPE=' + cfg]

        print("cmake_args")
        print(cmake_args)

        env = os.environ.copy()
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        subprocess.check_call(['cmake', os.path.abspath('')] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.', '--config', cfg, '--verbose'], cwd=self.build_temp)
        subprocess.check_call(['cmake', '--install', '.', '--verbose'], cwd=self.build_temp)

        shutil.copy("__init__.py", extdir)




setup(
    name='pyembree',
    ext_modules=[CMakeExtension('pyembree')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)