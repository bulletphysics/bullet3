#ifndef OPENTISSUE_CORE_MATH_MATH_COORDSYS_H
#define OPENTISSUE_CORE_MATH_MATH_COORDSYS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_vector3.h>
#include <OpenTissue/core/math/math_matrix3x3.h>
#include <OpenTissue/core/math/math_quaternion.h>
#include <OpenTissue/core/math/math_value_traits.h>

#include <iosfwd>

namespace OpenTissue
{

  namespace math
  {

    /**
    * A Coordinate System.
    *
    * Remeber, this represents a transform that brings you from a local frame
    * into a global frame, that is:
    *
    *   BF -> WCS
    */
    template<
      typename value_type_
      //, typename value_traits_ = ValueTraits<value_type_> 
    >
    class CoordSys
    {
    protected:

      typedef typename OpenTissue::math::ValueTraits<value_type_>  value_traits_ ;  // TODO value_traits_ should be parameterized as a class template parameter.

    public:

      typedef value_traits_           value_traits;  ///< Convenience typedef to make value traits accessible for all template functions using Vector3 types.
      typedef value_type_             value_type;    ///< Typedef is required for compliance with many other libraries and data containers!

      typedef Vector3<value_type>     vector3_type;
      typedef Quaternion<value_type>  quaternion_type;
      typedef Matrix3x3<value_type>   matrix3x3_type;
      typedef Quaternion<value_type>  rotation_type; ///< typedef to make interface for different coordsystypes the same

    protected:

      vector3_type     m_T;      ///< The Position.
      quaternion_type  m_Q;      ///< The orientation in Quaternion form.

    public:

      vector3_type       & T()       { return m_T; }
      vector3_type const & T() const { return m_T; }
      quaternion_type       & Q()       { return m_Q; }
      quaternion_type const & Q() const { return m_Q; }

    public:

      CoordSys()
        : m_T(value_traits::zero(),value_traits::zero(),value_traits::zero())
        , m_Q(value_traits::one(),value_traits::zero(),value_traits::zero(),value_traits::zero())
      {}

      explicit CoordSys(vector3_type const & T_val, quaternion_type const & Q_val) 
      {  
        m_T = T_val;
        m_Q = unit(Q_val);
      }

      explicit CoordSys(vector3_type const & T_val, matrix3x3_type const & R_val) 
      {
        m_T = T_val;
        m_Q = R_val;
      }

      CoordSys & operator=(CoordSys const & C)
      {
        m_T = C.m_T;
        m_Q = C.m_Q;
        return *this;
      }

    public:

      // TODO: Comparing floats with == or != is not safe NOTE T might not be a float type it could be anything? This suggest that we need some kind of metaprogramming technique to deal with this problem?
      bool operator==(CoordSys const & C) const {  return m_T == C.m_T && m_Q==C.m_Q; }

    public:

      void identity()
      {
        m_T.clear();
        m_Q.identity();
      }

      /**
      * This method assumes that the point is in this coordinate system.
      * In other words this method maps local points into non local
      * points:
      *
      * BF -> WCS
      *
      * Let p be the point and let f designate the function of this
      * method then we have
      *
      * [p]_WCS = f(p)
      *
      */
      void xform_point(vector3_type & p) const {    p = m_Q.rotate(p) + m_T;  }

      /**
      * This method assumes that the vector is in this
      * coordinate system. That is it maps the vector
      * from BF into WCS.
      */
      void xform_vector(vector3_type & v) const  {     v = m_Q.rotate(v);   }

      /**
      * Transform Matrix.
      *
      * @param O   A reference to a rotation matrix, which should be transformed.
      */
      void xform_matrix(matrix3x3_type & O) const
      {
        O = matrix3x3_type(m_Q) * O;
      }

      /**
      * Transform Coordinate System.
      * This method transforms the specified coordinate
      * system by this coordinate transform.
      *
      * That is:
      *
      *        Tnew =  Tx Rx Told
      *        Rnew =  Rx Rold
      *
      * @param X    The transform by which the current
      *             transform should be changed with..
      */
      CoordSys operator*(CoordSys const & X1) const
      {
        return CoordSys(    m_Q.rotate(X1.T()) + m_T , unit( prod( m_Q , X1.Q()) )     );
      }

    public:

      bool is_equal(CoordSys const & C, value_type const & accuracy) const
      {
        return m_T.is_equal(C.m_T,accuracy) && m_Q.is_equal(C.m_Q,accuracy);
      }

    };  // class CoordSys



    /**
    * Coordinate Transformation Product.
    * This function should be used to concatenate coordinate transformations.
    * In terms of homogeneous coordinates L and R corresponds to the matrices.
    *
    * L =  | R_l  T_l |
    *      |  0    1  |
    *
    * R =  | R_r  T_r |
    *      |  0    1  |
    *
    * This function computes the equivalent of the product
    *
    *   X = L R
    *
    *    =  | R_l  T_l |  | R_r  T_r |
    *       |  0    1  |  |  0    1  |
    *
    *   =   | R_l R_r    R_l T_r + T_l |
    *       |    0             1       |
    *
    * @param L   The left coordinate transformation
    * @param R   The right coordinate transformation
    *
    * @return   The coordinate transformation corresponding to the product L*R.
    */
    template<typename T>
    inline CoordSys<T> prod(CoordSys<T> const & L, CoordSys<T> const & R)
    {
      return CoordSys<T>(    L.Q().rotate( R.T() )  +  L.T() , unit( prod( L.Q() , R.Q()) ) );
    }

    /**
    * Inverse Transform.
    *
    * If we have
    *
    * BF -> WCS
    *
    * Then we want to find
    *
    * WCS -> BF
    *
    */
    template<typename T>
    inline CoordSys<T> inverse(CoordSys<T> const & X)
    {
      //---
      //---   p' = R p + T
      //---
      //---  =>
      //---
      //---   p = R^{-1} p' + R^{-1}(-T)
      //---
      return CoordSys<T>(  conj(X.Q()).rotate(-X.T()) , conj(X.Q()) );
    }

    /**
    * Model Update Transform.
    * This method computes the necessary transform needed in
    * order to transform coordinates from one local frame
    * into another local frame. This utility is useful when
    * one wants to do model updates instead of world updates.
    *
    * In mathematical terms we have two transforms:
    *
    * C1 : H -> G
    * C2 : F -> G
    *
    * And we want to find
    *
    * C3 : H -> F
    *
    * This transform is computed and assigned to this coordinate
    * system.
    */
    template<typename T>
    inline CoordSys<T> model_update(CoordSys<T> const & A, CoordSys<T> const & B)
    {
      return model_update(A.T(),A.Q(),B.T(),B.Q());
    }

    /**
    * Model Update Transform.
    * This method computes the necessary transform needed in
    * order to transform coordinates from one local frame
    * into another local frame. This utility is useful when
    * one wants to do model updates instead of world updates.
    *
    * In mathematical terms we have two transforms:
    *
    * C1 : H -> G
    * C2 : F -> G
    *
    * And we want to find
    *
    * C3 : H -> F
    *
    * This transform is computed and assigned to this coordinate
    * system.
    *
    * Very important: Note that this method finds the transform A -> B.
    */
    template<typename T>
    inline CoordSys<T> model_update(Vector3<T> const & TA, Quaternion<T> const & QA, Vector3<T> const & TB, Quaternion<T> const & QB)
    {
      //---
      //---  p' = RA p + TA         (*1)  from A->WCS
      //---
      //---  p = RB^T (p' - TB)     (*2)  from WCS-B
      //---
      //--- Insert (*1) into (*2)  A -> B
      //---
      //---   p = RB^T ( RA p + TA - TB)
      //---     =  RB^T  RA p + RB^T (TA - TB)
      //--- So
      //---   R = RB^T  RA
      //---   T = RB^T (TA - TB)
      //---
      Quaternion<T> q;
      if(QA==QB)
      {
        q.identity();
      }
      else
      {
        q = unit( prod( conj(QB), QA) );
      }
      return CoordSys<T>( conj(QB).rotate(TA - TB),  q);
    }

    template<typename T>
    inline std::ostream & operator<< (std::ostream & o, CoordSys<T> const & C)
    {
      o << "[" << C.T() << "," << C.Q() << "]";
      return o;
    }

    template<typename T>
    inline std::istream & operator>>(std::istream & i, CoordSys<T> & C)
    {
      char dummy;
      i >> dummy;
      i >> C.T();
      i >> dummy;
      i >> C.Q();
      i >> dummy;
      return i;
    }

  }  // namespace math

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_MATH_COORDSYS_H
#endif
