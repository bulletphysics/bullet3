#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_BASE_SHAPE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_BASE_SHAPE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/collision_geometry_interface.h>

#include <vector>


namespace OpenTissue
{

  namespace geometry
  {
    
    /**
    * BaseShape Template Class 
    * If you implement a new geometry type then you should derive your geometry class from the BaseShape class 
    * 
    * @tparam math_types  standard math types containing at least the real, vector3 and matrix3x3 types.
    **/
    template< typename math_types >
    class BaseShape
      : public OpenTissue::collision::GeometryInterface< math_types >
    {
    public:

      virtual  ~BaseShape() {}

    public:

      typedef typename math_types::real_type       real_type;
      typedef typename math_types::vector3_type    vector3_type;
      typedef typename math_types::matrix3x3_type  matrix3x3_type;


// 2007-07-24 kenny: generic design problem with compute_surface_points
//   2007-09-11 micky: I agree, it's not possible to have a pure abstract interface and letting the user decide his choice of point container.
// 2007-09-11 micky: compute_surface_points and get_support_point should be removed from this interface. Both functions belong to some collision interface.
//   2007-09-18 kenny: I partial agree... get_support_point is used by GJK (exclusively as I recall?) However, compute_surface_points is used by some of the geometry utils. Fitting tools and such.

      // TODO 2007-02-06 kenny: Yikes what if end-user wants to use std::list or some homebrewed container type? It is a bit ugly that the container type is hardwired into the abstract base class:-( Could we make something like a pure virtual template class?
      virtual void compute_surface_points( std::vector<vector3_type> & points) const = 0;
      virtual vector3_type get_support_point(vector3_type const & v) const = 0;

      virtual void translate(vector3_type const & T) = 0;
      virtual void rotate(matrix3x3_type const & R) = 0;
      virtual void scale(real_type const & s) = 0;

    };

  }  // namespace geometry

}  // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_BASE_SHAPE_H
#endif
