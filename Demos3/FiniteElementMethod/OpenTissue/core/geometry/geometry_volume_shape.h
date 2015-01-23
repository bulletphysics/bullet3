#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_VOLUME_SHAPE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_VOLUME_SHAPE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_base_shape.h>


namespace OpenTissue
{
  namespace geometry
  {
    /**
    * VolumeShape Template Class.
    * If you implement a new volumetric geometry then you should derive your new class from the VolumeShape class .
    * 
    * @tparam math_types  standard math types containing at least the real, vector3 and matrix3x3 types.
    **/
    template<typename math_types >
    class VolumeShape 
      : public BaseShape< math_types >
    {
    public:

      virtual  ~VolumeShape() {}

    public:

      typedef typename math_types::real_type       real_type;
      typedef typename math_types::vector3_type    vector3_type;
      typedef typename math_types::matrix3x3_type  matrix3x3_type;

      /**
       * Get Center Position.
       *
       * @return   The center point of the volume shape, in most cases this would be the mean point.
       */
      virtual vector3_type center() const = 0;

      /**
       * Get Volume.
       *
       * @return   The actual volume of the shape.
       */
      virtual real_type  volume() const = 0;

      /**
       * Get Area.
       *
       * @return    The actual surface area of the volume.
       */
      virtual real_type  area() const = 0;

      /**
       * Get Diameter.
       *
       *
       * @return   A measure of the diameter of the shape. Not all shapes have a well-defined
       *           diameter in such cases the measure would simply be a measure of the longst
       *           extent of the shape.
       */
      virtual real_type  diameter() const = 0;

    };

  }  // namespace geometry

}  // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_VOLUME_SHAPE_H
#endif
