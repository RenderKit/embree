## ======================================================================== ##
## Copyright 2009-2014 Intel Corporation                                    ##
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

 SubdivisionMesh

    A proposed extension to Embree to support subdivision surfaces.
    This class is intended to provide mesh connectivity information
    for use during acceleration structure construction and rendering.
    This class contains no subdivide method since it is anticipated
    that Embree will intersect rays directly with a limit surface.

    This class requires manifold surfaces with a consistent vertex
    winding order (though no specific winding order is necessary).
    Mesh faces can contain a varying number of vertices, up to the
    limit of pow(2, HALF_EDGE_VERTEX_BITS).

 API

    Mock up of an Embree API extension to support subdivision meshes.

 CatmullClark

    Throw away code to subdivide meshes.

 TriangulateMesh

    Throw away code to triangulate subdivision meshes.

