# pyembree - Python bindings for embree

## Building pyembree

```
cmake -B build -D embree_DIR=path/to/installed/embree/cmake/files
cmake --build build
```

## Running test

Make sure that build artefacts of pyembree can be sourced by python, i.e. setting your PYTHONPATH.
Make sure that embree binaries are located in you PATH.

### Linux

```
export PYTHONPATH=<pyembree_dir>\build\src:$PYTHONPATH
python

>>> import pyembree
>>> print(pyembree.RTC_INVALID_GEOMETRY_ID)
```

### Windows

```
$env:PYTHONPATH += "<pyembree_dir>\build\src\Debug"
python

>>> import pyembree
>>> print(pyembree.RTC_INVALID_GEOMETRY_ID)
```