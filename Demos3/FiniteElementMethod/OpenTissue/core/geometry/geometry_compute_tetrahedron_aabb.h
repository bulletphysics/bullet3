#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_TETRAHEDRON_AABB_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_TETRAHEDRON_AABB_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_aabb.h>

namespace OpenTissue
{
  namespace geometry
  {
    /**
    * Compute AABB of Tetrahedron.
    *
    *
    * @param p0              The position of a corner of the tetrahedron.
    * @param p1              The position of a corner of the tetrahedron.
    * @param p2              The position of a corner of the tetrahedron.
    * @param p3              The position of a corner of the tetrahedron.
    * @param min_coord       Upon return holds the minimum coordinate corner of the AABB.
    * @param max_coord       Upon return holds the maximum coordinate corner of the AABB.
    */
    template<typename vector3_type>
    void compute_tetrahedron_aabb(
      vector3_type const & p0
      , vector3_type const & p1
      , vector3_type const & p2
      , vector3_type const & p3
      , vector3_type & min_coord
      , vector3_type & max_coord
      )
    {
      using std::min;

      min_coord = min(p0, p1);
      min_coord = min(min_coord, p2);
      min_coord = min(min_coord, p3);

      max_coord = max(p0, p1);
      max_coord = max(max_coord, p2);
      max_coord = max(max_coord, p3);
    }

    /**
    * Compute Tetrahedron AABB.
    *
    * This function computes a tight fitting axis aligned bounding box around the specified tetrahedron.
    *
    * @param tetrahedron  The specified tetrahedron.
    * @param min_coord       Upon return holds the minimum coordinate corner of the AABB.
    * @param max_coord       Upon return holds the maximum coordinate corner of the AABB.
    */
    template<typename tetrahedron_type,typename vector3_type>
    void compute_tetrahedron_aabb(
      tetrahedron_type const & tetrahedron
      , vector3_type & min_coord
      , vector3_type & max_coord
      )
    {
      compute_tetrahedron_aabb(tetrahedron.p0(),tetrahedron.p1(),tetrahedron.p2(),tetrahedron.p3(),min_coord,max_coord);
    }

    /**
    * Compute Tetrahedron AABB.
    *
    * This function computes a tight fitting axis aligned bounding box around the specified tetrahedron.
    *
    * @param tetrahedron  The specified tetrahedron.
    * @param aabb         Upon return holds the computed AABB.
    */
    template<typename tetrahedron_type,typename aabb_type>
    void compute_tetrahedron_aabb(tetrahedron_type const & tetrahedron,aabb_type & aabb)
    {
      compute_tetrahedron_aabb(tetrahedron.p0(),tetrahedron.p1(),tetrahedron.p2(),tetrahedron.p3(),aabb.min(),aabb.max());
    }

    /**
    * Compute Tetrahedron AABB.
    *
    * This function computes a tight fitting axis aligned bounding box around the specified tetrahedron.
    *
    * @param tetrahedron        The specified tetrahedron.
    * @return                   The computed AABB.
    */
    template<typename tetrahedron_type>
    geometry::AABB<typename tetrahedron_type::math_types> compute_tetrahedron_aabb(tetrahedron_type const & tetrahedron)
    {
      geometry::AABB<typename tetrahedron_type::math_types> aabb;
      compute_tetrahedron_aabb(tetrahedron,aabb);
      return aabb;
    }

  } //End of namespace geometry
} //End of namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_TETRAHEDRON_AABB_H
#endif
