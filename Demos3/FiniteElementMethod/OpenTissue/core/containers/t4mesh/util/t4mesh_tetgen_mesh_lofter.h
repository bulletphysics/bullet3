#ifndef OPENTISSUE_CORE_CONTAINERS_T4MESH_UTIL_T4MESH_TETGEN_MESH_LOFTER_H
#define OPENTISSUE_CORE_CONTAINERS_T4MESH_UTIL_T4MESH_TETGEN_MESH_LOFTER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/common/io/mesh_tetgen_write.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_is_manifold.h>
#include <OpenTissue/core/containers/t4mesh/io/t4mesh_tetgen_read.h>
#include <TetGen/tetgen.h>

#include <cmath>
#include <string>
#include <sstream>
#include <iostream>


namespace OpenTissue
{
  namespace t4mesh
  {

    struct mesh_lofter_settings
    {
      mesh_lofter_settings()
        : m_quality_ratio(2./*std::sqrt(2.)*/)
        , m_maximum_volume(0.)
        , m_intermediate_file("")
        , m_quiet_output(false)
        , m_verify_input(false)
      {}
      double       m_quality_ratio;      ///< quality t4mesh is issued if > 0. A minimum radius-edge ratio may be specifyed (default 2.0).
      double       m_maximum_volume;     ///< max volume constraints on t4mesh if > 0.
      std::string  m_intermediate_file;  ///< use intermediate files to/fro tetget if name specified.
      bool         m_quiet_output;       ///< keep output spam as silent as possible, great for RELEASE.
      bool         m_verify_input;       ///< DEBUG: detects plc intersections, i.e. verify "bad" input mesh.
    };


    /**
    * t4mesh_mesh_lofter.
    * - uses tetgen to loft/extrude a closed two-manifold polymesh to a generic tetrahedal mesh
    *
    * @param polymesh          A poly mesh, which holds a closed two-manifold.
    * @param t4mesh            A generic t4mesh, which upon return holds the generated tetrahedal mesh.
    * @param settings          The settings/configuration for TetGen, can be omitted to use the default settings.
    *
    */
    // TODO: Implement work-in-mem solution, thus avoid using intermediate files [MKC]
    template<typename t4mesh_type, typename polymesh_type>
    bool mesh_lofter(t4mesh_type& t4mesh, const polymesh_type& polymesh, const mesh_lofter_settings & settings = mesh_lofter_settings())
    {
      // local functions are no go in C++, thus we need to wrap them in a local class, tsk!
      class local_aux
      {
      public:
        static bool error(const std::string& text)
        {
          using std::operator<<;
          std::cerr << "ERROR [t4mesh_mesh_lofter]:\n- " << text << std::endl;
          return false;
        }
      };

      // convenient pointer to use with TetGen
      char* tmp_file = settings.m_intermediate_file.size()>0?const_cast<char*>(settings.m_intermediate_file.c_str()):NULL;

      // "no tmp file" won't work check
      if (!tmp_file)
        return local_aux::error("current version only supports using intermediate files");

      if (!is_manifold(polymesh))
        return local_aux::error("polymesh is not a two-manifold");

      if (tmp_file)
        if (!tetgen_write(settings.m_intermediate_file+".poly", polymesh))
          return local_aux::error("mesh_tetgen_write failed in writing the polymesh file: '"+settings.m_intermediate_file+".poly'");

      std::stringstream tmp;
      tmp << "p";  // piecewise linear complex (always)
      if (settings.m_verify_input)
        tmp << "d";
      else {
        if (settings.m_quality_ratio > 0)
          tmp << "q" << settings.m_quality_ratio;  // quality
        if (settings.m_maximum_volume > 0)
          tmp << "a" << settings.m_maximum_volume;  // max volume
        if (settings.m_quiet_output)
          tmp << "Q";  // keep quiet :)
      }
      std::string txt = tmp.str().c_str();
      tetgenbehavior config;
      // MKC: setting vars in tetgenbehavior explicitly is too complicated, and requires a deeper knowledge of the system :(
      if (!config.parse_commandline(const_cast<char*>(txt.c_str())))  // parsing a "cmd" line will set the correct vars and their dependecies.
        return local_aux::error("TetGen parse_commandline failed: '"+txt+"'");

      tetgenio in, out;
      if (tmp_file)
        if (!in.load_poly(tmp_file))
          return local_aux::error("TetGen load_poly failed: '"+settings.m_intermediate_file+"'");

      // let TetGen do some magic :)
      tetrahedralize(&config, &in, &out);

      if (out.numberoftetrahedra < 1)
        return local_aux::error("TetGen tetrahedralize failed: no tetrahedra generated!");

      if (tmp_file) {
        out.save_elements(tmp_file);
        out.save_nodes(tmp_file);

        if (!tetgen_read(settings.m_intermediate_file, t4mesh))
          return local_aux::error("t4mesh_tetgen_read failed in reading the data: '"+settings.m_intermediate_file+"'");
      }

      return true;
    }

  } // namespace t4mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_T4MESH_UTIL_T4MESH_TETGEN_MESH_LOFTER_H
#endif
