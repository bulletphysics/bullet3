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

    std::string inputfile = "cornell_box.obj";
    std::vector<tinyobj::shape_t> shapes;
  
    std::string err = tinyobj::LoadObj(shapes, inputfile.c_str());
  
    if (!err.empty()) {
      std::cerr << err << std::endl;
      exit(1);
    }
  
    std::cout << "# of shapes : " << shapes.size() << std::endl;
  
    for (size_t i = 0; i < shapes.size(); i++) {
      printf("shape[%ld].name = %s\n", i, shapes[i].name.c_str());
      printf("shape[%ld].indices: %ld\n", i, shapes[i].mesh.indices.size());
      assert((shapes[i].mesh.indices.size() % 3) == 0);
      for (size_t f = 0; f < shapes[i].mesh.indices.size(); f++) {
        printf("  idx[%ld] = %d\n", f, shapes[i].mesh.indices[f]);
      }
  
      printf("shape[%ld].vertices: %ld\n", i, shapes[i].mesh.positions.size());
      assert((shapes[i].mesh.positions.size() % 3) == 0);
      for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
        printf("  v[%ld] = (%f, %f, %f)\n", v,
          shapes[i].mesh.positions[3*v+0],
          shapes[i].mesh.positions[3*v+1],
          shapes[i].mesh.positions[3*v+2]);
      }
    
      printf("shape[%ld].material.name = %s\n", i, shapes[i].material.name.c_str());
      printf("  material.Ka = (%f, %f ,%f)\n", shapes[i].material.ambient[0], shapes[i].material.ambient[1], shapes[i].material.ambient[2]);
      printf("  material.Kd = (%f, %f ,%f)\n", shapes[i].material.diffuse[0], shapes[i].material.diffuse[1], shapes[i].material.diffuse[2]);
      printf("  material.Ks = (%f, %f ,%f)\n", shapes[i].material.specular[0], shapes[i].material.specular[1], shapes[i].material.specular[2]);
      printf("  material.Tr = (%f, %f ,%f)\n", shapes[i].material.transmittance[0], shapes[i].material.transmittance[1], shapes[i].material.transmittance[2]);
      printf("  material.Ke = (%f, %f ,%f)\n", shapes[i].material.emission[0], shapes[i].material.emission[1], shapes[i].material.emission[2]);
      printf("  material.Ns = %f\n", shapes[i].material.shininess);
      printf("  material.map_Ka = %s\n", shapes[i].material.ambient_texname.c_str());
      printf("  material.map_Kd = %s\n", shapes[i].material.diffuse_texname.c_str());
      printf("  material.map_Ks = %s\n", shapes[i].material.specular_texname.c_str());
      printf("  material.map_Ns = %s\n", shapes[i].material.normal_texname.c_str());
      std::map<std::string, std::string>::iterator it(shapes[i].material.unknown_parameter.begin());
      std::map<std::string, std::string>::iterator itEnd(shapes[i].material.unknown_parameter.end());
      for (; it != itEnd; it++) {
        printf("  material.%s = %s\n", it->first.c_str(), it->second.c_str());
      }
      printf("\n");
    }
