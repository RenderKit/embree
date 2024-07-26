# PyEmbree - Python bindings for embree

These Python bindings for Embree are available under the same license as Intel® Embree. 
By utilizing these bindings, you can access the full capabilities of the Embree API through a Python interface, making it easier to integrate Embree's powerful ray tracing functionalities into your Python projects. 
For a comprehensive understanding of the available API functions and their usage, please refer to the Embree C-API documentation. 
This documentation provides detailed descriptions and examples to help you effectively get you started with Embree in python applications.

This is a pre-release version, and PyEmbree is still under active development. 
In its current state, it provides a one-to-one mapping of the C-style Embree API, allowing users to utilize Embree's functionalities directly through Python. 
This is however expected to change in the future, when Embree's SYCL support is integrated in a more unified and streamlined manner. 
As development progresses, users can anticipate enhancements that will simplify and enrich the interaction with Embree's powerful ray tracing capabilities.


## Building, Installing, and Testing PyEmbree

PyEmbree provides a `setup.py` to create a python wheel and install it locally to the system.
To do so follow a few simple steps inside the `python` subfolder inside the Embree repository:

```
pip install .
```

You may then run the tutorials, e.g.:

```
python tutorials/minimal.py
```

### Troubleshooting

Dependencies are built and installed using the pip command. 
If you encounter issues loading the module, please ensure that your PATH environment variable includes the location where the dependencies are installed.


## API Extensions

Embree functions that handle arrays of data, such as the rtcIntersect* class of functions, accept both Python lists and NumPy arrays. 
When using NumPy arrays, it is crucial to ensure that the data is properly aligned to meet Embree's requirements. 
To facilitate the use of these functions with NumPy, several helper functions are provided. 
These helpers assist with generating and accessing ray hit data, making it easier to integrate Embree's capabilities with NumPy arrays.

### rtcCreateRayHits

This function creates a ray hits structure array from several NumPy arrays, which provide the ray origins, directions, and ray IDs component-wise. 
The resulting Python array is properly aligned for use with rtcIntersect*.

### rtcTransformRayHits

Applies a transformation to a list of ray hit structures.

### rtcRayHits_get_org(x|y|z), rtcRayHits_get_dir(x|y/|), rtcRayHits_get_tfar, rtcRayHits_get_rayids

These functions provide convenient access to components within an array of ray hit structures. 
They enable straightforward conversion between a structure of arrays and an array of structures for ray hits.

### rtcIntersectN

This function extends the functionality of the rtcIntersect(1|4|8|16) functions found in the Embree C-API. 
While it is possible to perform ray packet intersections from Python, doing so typically involves complex steps to ensure that arrays of ray hits are correctly aligned. 
Constructing these aligned arrays can be challenging and error-prone from within Python. 
The rtcIntersectN function simplifies this process by providing an interface that allows for the intersection of arrays of ray hit structures of any length. 
It manages the complexities of ray packets and parallelization automatically, making the process more straightforward and efficient from within Python.