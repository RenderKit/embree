// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

 This tutorial demonstrates the use of subdivision surfaces in Embree.  A
 simple cube is constructed as a subdivision mesh.  The subdivision level
 can be controlled using the '+' and '-' keys.

 Subdivision meshes can be constructed directly through the Embree API, or
 through a layer which implements a subset of the OpenSubdiv HBR API from
 Pixar.  If you are using the latter and your vertex type is other than
 embree::Vec3f, you will need to provide a concrete implementation of the
 templated function HbrMesh::Finish() (see opensubdiv/hbr/mesh.h).

 Embree supports subdivision meshes that are closed surfaces (i.e. there are
 no boundary edges) and which have a consistent vertex winding order (though
 no specific winding order is necessary).  Mesh faces can contain a varying
 number of vertices, up to the limit of pow(2, HALF_EDGE_VERTEX_BITS).
 Creases and holes are supported, though not shown in this tutorial.

