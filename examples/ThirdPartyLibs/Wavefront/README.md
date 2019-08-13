tinyobjloader
=============

http://syoyo.github.io/tinyobjloader/

Tiny but poweful single file wavefront obj loader written in C++. No dependency except for C++ STL. It can parse 10M over polygons with moderate memory and time.

Good for embedding .obj loader to your (global illumination) renderer ;-)


Example
-------

![Rungholt](https://github.com/syoyo/tinyobjloader/blob/master/images/rungholt.jpg?raw=true)

tinyobjloader can successfully load 6M triangles Rungholt scene.
http://graphics.cs.williams.edu/data/meshes.xml

Features
--------

* Group
* Vertex
* Texcoord
* Normal
* Material
  * Unknown material attributes are treated as key-value.

Notes
-----

Polygon is converted into triangle.

License
-------

Licensed under 2 clause BSD.

Usage
-----
Data format
attrib_t contains single and linear array of vertex data(position, normal and texcoord).

attrib_t::vertices => 3 floats per vertex

       v[0]        v[1]        v[2]        v[3]               v[n-1]
  +-----------+-----------+-----------+-----------+      +-----------+
  | x | y | z | x | y | z | x | y | z | x | y | z | .... | x | y | z |
  +-----------+-----------+-----------+-----------+      +-----------+

attrib_t::normals => 3 floats per vertex

       n[0]        n[1]        n[2]        n[3]               n[n-1]
  +-----------+-----------+-----------+-----------+      +-----------+
  | x | y | z | x | y | z | x | y | z | x | y | z | .... | x | y | z |
  +-----------+-----------+-----------+-----------+      +-----------+

attrib_t::texcoords => 2 floats per vertex

       t[0]        t[1]        t[2]        t[3]               t[n-1]
  +-----------+-----------+-----------+-----------+      +-----------+
  |  u  |  v  |  u  |  v  |  u  |  v  |  u  |  v  | .... |  u  |  v  |
  +-----------+-----------+-----------+-----------+      +-----------+

attrib_t::colors => 3 floats per vertex(vertex color. optional)

       c[0]        c[1]        c[2]        c[3]               c[n-1]
  +-----------+-----------+-----------+-----------+      +-----------+
  | x | y | z | x | y | z | x | y | z | x | y | z | .... | x | y | z |
  +-----------+-----------+-----------+-----------+      +-----------+

Each shape_t::mesh_t does not contain vertex data but contains array index to attrib_t. See loader_example.cc for more details.


mesh_t::indices => array of vertex indices.

  +----+----+----+----+----+----+----+----+----+----+     +--------+
  | i0 | i1 | i2 | i3 | i4 | i5 | i6 | i7 | i8 | i9 | ... | i(n-1) |
  +----+----+----+----+----+----+----+----+----+----+     +--------+

Each index has an array index to attrib_t::vertices, attrib_t::normals and attrib_t::texcoords.

mesh_t::num_face_vertices => array of the number of vertices per face(e.g. 3 = triangle, 4 = quad , 5 or more = N-gons).


  +---+---+---+        +---+
  | 3 | 4 | 3 | ...... | 3 |
  +---+---+---+        +---+
    |   |   |            |
    |   |   |            +-----------------------------------------+
    |   |   |                                                      |
    |   |   +------------------------------+                       |
    |   |                                  |                       |
    |   +------------------+               |                       |
    |                      |               |                       |
    |/                     |/              |/                      |/

 mesh_t::indices

  |    face[0]   |       face[1]     |    face[2]   |     |      face[n-1]           |
  +----+----+----+----+----+----+----+----+----+----+     +--------+--------+--------+
  | i0 | i1 | i2 | i3 | i4 | i5 | i6 | i7 | i8 | i9 | ... | i(n-3) | i(n-2) | i(n-1) |
  +----+----+----+----+----+----+----+----+----+----+     +--------+--------+--------+

```c++
#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include "tiny_obj_loader.h"

std::string inputfile = "cornell_box.obj";
tinyobj::attrib_t attrib;
std::vector<tinyobj::shape_t> shapes;

LoadObj(attrib, shapes, inputfile.c_str());

// Loop over shapes
for (size_t s = 0; s < shapes.size(); s++) {
  // Loop over faces(polygon)
  size_t index_offset = 0;
  for (size_t f = 0; f < shapes[s].mesh.indices.size(); f++) {
    int fv = 3;
    // Loop over vertices in the face.
    for (size_t v = 0; v < fv; v++) {
      // access to vertex
      tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
      tinyobj::real_t vx = attrib.vertices[3*idx.vertex_index+0];
      tinyobj::real_t vy = attrib.vertices[3*idx.vertex_index+1];
      tinyobj::real_t vz = attrib.vertices[3*idx.vertex_index+2];
      tinyobj::real_t nx = attrib.normals[3*idx.normal_index+0];
      tinyobj::real_t ny = attrib.normals[3*idx.normal_index+1];
      tinyobj::real_t nz = attrib.normals[3*idx.normal_index+2];
      tinyobj::real_t tx = attrib.texcoords[2*idx.texcoord_index+0];
      tinyobj::real_t ty = attrib.texcoords[2*idx.texcoord_index+1];
    }
    index_offset += fv;
  }
}

```

