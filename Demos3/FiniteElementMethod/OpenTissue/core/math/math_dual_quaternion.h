#ifndef OPENTISSUE_CORE_MATH_MATH_DUAL_QUATERNION_H
#define OPENTISSUE_CORE_MATH_MATH_DUAL_QUATERNION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2010 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>

#include <OpenTissue/core/math/math_quaternion.h>

#include <iosfwd>

// 2010-07-16 Kenny : code review, where is the unit tests?

namespace OpenTissue
{

  namespace math
  {

    // 2010-07-16 Kenny: code review, why do you keep uncommented code around?
    // 2010-07-16 perb: I have keept this as I thought this was the direction OpenTissue was going,
    //                  considering that it is in both the vector3 and quaternion class
    //                  I thought it was a preparation to get rid of the next TODO...
    template <
      typename value_type_
      //, typename value_traits_ = ValueTraits<value_type_>
    >
    class DualQuaternion
    {
    protected:

      // 2010-07-16 Kenny: code review, the TODO comment seems unneeded?
      // 2010-07-16 perb
      typedef typename OpenTissue::math::ValueTraits<value_type_>  value_traits_ ;  // TODO value_traits_ should be parameterized as a class template parameter.

    public:

      typedef value_traits_            value_traits;  ///< Convience typedef to make value traits accesible for all template functions using Vector3 types.
      typedef value_type_              value_type;    ///< Typedef is required for compliance with many other libraries and data containers!
      typedef size_t                   index_type;
      typedef Vector3<value_type>      vector3_type;
      typedef Quaternion<value_type>   quaternion_type;

    protected:

      quaternion_type m_real; ///< The real part of this dual quaternion. This represents the rotation when used as a rigid motion.
      quaternion_type m_dual; ///< The dual part of this dual quaternion. This represents the displacement when used as a rigid motion.

    public:

      quaternion_type       & r()       { return m_real; }
      quaternion_type const & r() const { return m_real; }

      quaternion_type       & d()       { return m_dual; }
      quaternion_type const & d() const { return m_dual; }

    public:

      DualQuaternion()
        : m_real( quaternion_type( value_traits::one(), value_traits::zero(), value_traits::zero(), value_traits::zero() ) )
        , m_dual( quaternion_type( value_traits::zero(), value_traits::zero(), value_traits::zero(), value_traits::zero() ) )
      {}

      /**
       * This constructs a dual quaternion representing the transformation of the given vector
       *
       * @param v Vector describing a translation
       *
       * @return A dual quaternion describing the translation given
       */
      DualQuaternion( vector3_type const & v )
        : m_real( quaternion_type( value_traits::one(), value_traits::zero(), value_traits::zero(), value_traits::zero() ) )
        , m_dual( quaternion_type( value_traits::one(), v[0] / value_traits::two(), v[1] / value_traits::two(), v[2] / value_traits::two() ) )
      {}

      /**
       * This constructs a unit dual quaternion describing a rotation and a translation
       *
       * @param rotation A unit quaternion describing a rotation
       * @param translation A vector describing a translation
       *
       * @return A unit dual quaternion describing the rotation and translation given
       */
      DualQuaternion( quaternion_type const & rotation, vector3_type const & translation )
        : m_real( rotation )
    	, m_dual( prod( quaternion_type( value_traits::zero(), translation[0], translation[1], translation[2] ), rotation) / value_traits::two() )
      {}

      DualQuaternion( DualQuaternion< value_type > const & d ) { *this = d; }

      DualQuaternion( quaternion_type const & s_val, quaternion_type const & d_val )
        : m_real( s_val )
        , m_dual( d_val )
      {}

      ~DualQuaternion()
      {}

      DualQuaternion & operator=( DualQuaternion const & cpy )
      {
        m_real = cpy.m_real;
        m_dual = cpy.m_dual;

        return *this;
      }

      DualQuaternion & operator+= ( DualQuaternion const & qd )
      {
        // 2010-07-16 Kenny: code review, safe guard for self assignment? if (this!=&qd)???
        // 2010-07-16 perb: I am not sure why this would be a problem.
        // 2010-07-16 kenny: Argh, sorry, I read the code too fast, I read it as an assignment operator.
        m_real += qd.r();
        m_dual += qd.d();

        return *this;
      }

    public:

      friend std::ostream & operator<< ( std::ostream & o, DualQuaternion< value_type > const & d )
      {
        o << "[" << d.r() << "," << d.d() << "]";

        return o;
      }

      friend std::istream & operator>>( std::istream & i, DualQuaternion< value_type > & d )
      {
        char dummy;

        i >> dummy;
        i >> d.r();
        i >> dummy;
        i >> d.d();
        i >> dummy;

        return i;
      }
    };  // class DualQuaternion


    //////////////////////////////////////////////////////////////////////////
    /// Declaration of DualQuaternion non-member functions
    //////////////////////////////////////////////////////////////////////////

    template < typename T, typename T2 >
    inline DualQuaternion<T> operator*( DualQuaternion<T> const& dq, T2 const& v ) { return DualQuaternion< T >( dq.r() * v, dq.d() * v ); }

    template < typename T2, typename T >
    inline DualQuaternion<T> operator*( T2 const& v, DualQuaternion<T> const& dq ) { return DualQuaternion< T >( dq.r() * v, dq.d() * v ); }

    template < typename T, typename T2 >
    inline DualQuaternion<T> operator/( DualQuaternion<T> const& dq, T2 const& v ) { return DualQuaternion< T >( dq.r() / v, dq.d() / v ); }

    template < typename T >
    inline DualQuaternion< T > operator-( DualQuaternion< T > q ) {  return DualQuaternion< T >( -q.r(), -q.d() ); }


    /**
     * Dual conjugate. This is the conjugate that considers the dual quaternion as a dual number with quaternion elements.
     *
     * @warning There are two different types. The quaternion conjugate and the dual conjugate.
     */
    template< typename T >
    inline DualQuaternion< T > conj_dual( DualQuaternion< T > const & dq )
    {
      return DualQuaternion< T >( dq.r(), -dq.d() );
    }

    /**
     * Quaternion conjugate. This is the conjugate that considers the dual quaternion as a quaternion with dual number elements.
     *
     * @warning There are two different types. The quaternion conjugate and the dual conjugate.
     */
    template< typename T >
    inline DualQuaternion< T > conj_quaternion( DualQuaternion< T > const & dq )
    {
      return DualQuaternion< T >( conj( dq.r() ), conj( dq.d() ) );
    }

    template< typename T >
    inline DualQuaternion< T > prod( DualQuaternion< T > const & q, DualQuaternion< T > const & p )
    {
      return DualQuaternion< T >( prod( q.r(), p.r() ), prod( q.r(), p.d() ) + prod( q.d(), p.r() ) );
    }

  } // namespace math

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_MATH_DUAL_QUATERNION_H
#endif
