#ifndef OPENTISSUE_COLLISION_COLLISION_GEOMETRY_INTERFACE_H
#define OPENTISSUE_COLLISION_COLLISION_GEOMETRY_INTERFACE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/utility_class_id.h>

namespace OpenTissue
{
  namespace collision
  {


    /**
     * Collision Geometry Interface.
     * The purpose of this class is to create a common interface for
     * collision geometries.
     *
     * The interface should make it easier to integrate geometry types into
     * a collision detection engine. In particular it should help make multiple
     * dispatching easier (the class_id interface) and it should support basic
     * functionality for computing bounding boxes (compute_collision_aabb method)
     *
     *
     */
    template< typename math_types >
    class GeometryInterface
      : virtual public OpenTissue::utility::ClassIDInterface
    {
    public:

      typedef typename math_types::vector3_type     vector3_type;
      typedef typename math_types::matrix3x3_type   matrix3x3_type;

    public:

      /**
      * Compute Bounding Box.
      * This method computes an axis aligned bounding
      * box (AABB) that encloses the geometry.
      *
      * @param r           The position of the model frame (i.e the coordinate frame the geometry lives in).
      * @param R           The orientation of the model frame (i.e the coordinate frame the geometry lives in).
      * @param min_coord   Upon return holds the minimum corner of the box.
      * @param max_coord   Upon return holds the maximum corner of the box.
      *
      */
      virtual void compute_collision_aabb(
        vector3_type const & r
        , matrix3x3_type const & R
        , vector3_type & min_coord
        , vector3_type & max_coord
        ) const = 0;

    };

  } // namespace collision

} // namespace OpenTissue

// OPENTISSUE_COLLISION_COLLISION_GEOMETRY_INTERFACE_H
#endif
