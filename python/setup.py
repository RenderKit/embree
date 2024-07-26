#! /usr/bin/env python3

import os
import sys
import platform
import subprocess
import shutil

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext



def tabulate(data):
    if not data:
        return ""
    
    col_widths = [max(len(str(item)) for item in col) for col in zip(*data)]
    format_str = ' | '.join(f'{{:<{width}}}' for width in col_widths)
    table = [format_str.format(*row) for row in data]

    return '\n'.join(table)


class CMakeExtension(Extension):
    def __init__(self, name):
        Extension.__init__(self, name, sources=[])


class CMakeBuild(build_ext):

    user_options = build_ext.user_options + [
        ('embree-config=', None, 'Comma separated list of embree configs')
    ]

    def initialize_options(self):
        build_ext.initialize_options(self)
        self.embree_config = ""

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

        extension_dir = os.path.join(os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name))), ext.name)
        setuppy_dir = os.path.abspath('')
        build_dir = setuppy_dir + os.path.sep + 'build'
        embree_install_dir = build_dir + os.path.sep + 'embree_install'
        print(tabulate([
            ["extension_dir", extension_dir],
            ["setuppy_dir", setuppy_dir],
            ["build_dir", build_dir],
            ["embree_install_dir", embree_install_dir],
        ]))

        cmake_args = [
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extension_dir,
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE=' + extension_dir,
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG=' + extension_dir,
            #'-DPYTHON_EXECUTABLE=' + sys.executable,
            #'-G Ninja',
        ]

        ## for SYCL
        #cmake_args += [
        #    '-DCMAKE_CXX_COMPILER=clang++',
        #    '-DCMAKE_C_COMPILER=clang',
        #]

        cfg = 'Debug' if self.debug else 'Release'
        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
        cmake_args += ['-DEMBREE_MAX_ISA=SSE2']
        cmake_args += ['-DCMAKE_INSTALL_PREFIX=' + embree_install_dir]

        print("cmake_args")
        print(tabulate([s.split('=') for s in cmake_args]))

        env = os.environ.copy()
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        subprocess.check_call(['cmake', setuppy_dir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.', '--config', cfg, '-j', '--verbose'], cwd=self.build_temp)
        subprocess.check_call(['cmake', '--install', '.', '--verbose'], cwd=self.build_temp)

        # copy additional dependencies
        shutil.copy("__init__.py", extension_dir)

        embree_bin_dir = embree_install_dir + os.path.sep + 'bin'
        for f in os.listdir(embree_bin_dir):
            shutil.copy(os.path.join(embree_bin_dir, f), extension_dir)


setup(
    name='pyembree',
    ext_modules=[CMakeExtension('pyembree')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)