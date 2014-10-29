#ifndef OPENTISSUE_CORE_MATH_MATH_QUATERNION_H
#define OPENTISSUE_CORE_MATH_MATH_QUATERNION_H
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
#include <OpenTissue/core/math/math_value_traits.h>
#include <OpenTissue/core/math/math_is_number.h>

#include <cmath>
#include <iosfwd>


namespace OpenTissue
{

  namespace math
  {

    template<
      typename value_type_
      //, typename value_traits_ = ValueTraits<value_type_>
    >
    class Quaternion
    {
    protected:

      typedef typename OpenTissue::math::ValueTraits<value_type_>  value_traits_ ;  // TODO value_traits_ should be parameterized as a class template parameter.

    public:

      typedef value_traits_          value_traits;  ///< Convenience typedef to make value traits accessible for all template functions using Vector3 types.
      typedef value_type_            value_type;    ///< Typedef is required for compliance with many other libraries and data containers!
      typedef Vector3<value_type>    vector3_type;  // TODO should Vector3 type be template parameterized?
      typedef Matrix3x3<value_type>  matrix3x3_type;
      typedef size_t                 index_type;    // TODO should the index type be template parameterized?

    protected:

      value_type   m_s;      ///< The real part.
      vector3_type m_v;      ///< The R3 part or imaginary part.

    public:

      value_type       & s()       { return m_s; }
      value_type const & s() const { return m_s; }

      vector3_type       & v()       { return m_v; }
      vector3_type const & v() const { return m_v; }

    public:

      Quaternion()
        : m_s(value_traits::one())
        , m_v(value_traits::zero(),value_traits::zero(),value_traits::zero())
      {}

      ~Quaternion(){}

      Quaternion(Quaternion const & q) { *this = q; }

      explicit Quaternion(matrix3x3_type const & M){ *this = M; }

      explicit Quaternion(value_type const & s_val, value_type const & x, value_type const & y, value_type const & z)
        : m_s(s_val)
        , m_v(x,y,z)
      { }

      explicit Quaternion(value_type const & s_val, vector3_type const & v_val)
        : m_s(s_val)
        , m_v(v_val)
      { }

      Quaternion & operator=(Quaternion const & cpy)
      {
        m_s = cpy.m_s;
        m_v = cpy.m_v;
        return *this;
      }

      /**
      * Assignment operator.
      * This method is a little special. Quaternions can be used to represent rotations, so can matrices.
      * This method transforms a rotation matrix into a Quaternion and then it assigns it to this Quaternion.
      * In short this method converts matrices into quaternions.
      *
      * @param M       A reference to a matrix. This matrix should be a rotation matrix. That is an orthogonal matrix.
      */
      Quaternion & operator=(matrix3x3_type const & M)
      {
        using std::sqrt;

        value_type const & M00 = M(0,0);
        value_type const & M01 = M(0,1);
        value_type const & M02 = M(0,2);
        value_type const & M10 = M(1,0);
        value_type const & M11 = M(1,1);
        value_type const & M12 = M(1,2);
        value_type const & M20 = M(2,0);
        value_type const & M21 = M(2,1);
        value_type const & M22 = M(2,2);

        value_type tr = M00 + M11 + M22;
        value_type r;

        value_type const half = value_traits::one()/value_traits::two();

        if(tr>=value_traits::zero())
        {
          r      = sqrt(tr + value_traits::one());
          m_s    = half*r;
          r      = half/r;
          m_v[0] = (M21 - M12) * r;
          m_v[1] = (M02 - M20) * r;
          m_v[2] = (M10 - M01) * r;
        }
        else
        {
          int i = 0;
          if(M11>M00)
            i = 1;
          if(M22>M(i,i))
            i = 2;
          switch(i)
          {
          case 0:
            r      = sqrt((M00 - (M11+M22)) + value_traits::one());
            m_v[0] = half*r;
            r      = half/r;
            m_v[1] = (M01 + M10) * r;
            m_v[2] = (M20 + M02) * r;
            m_s    = (M21 - M12) * r;
            break;
          case 1:
            r      = sqrt((M11 - (M22+M00)) + value_traits::one());
            m_v[1] = half*r;
            r      = half/r;
            m_v[2] = (M12 + M21)*r;
            m_v[0] = (M01 + M10)*r;
            m_s    = (M02 - M20)*r;
            break;
          case 2:
            r      = sqrt((M22 - (M00+M11)) + value_traits::one());
            m_v[2] = half*r;
            r      = half/r;
            m_v[0] = (M20 + M02) * r;
            m_v[1] = (M12 + M21) * r;
            m_s    = (M10 - M01) * r;
            break;
          };
        }
        return *this;
      }

    public:

      // TODO: Comparing floats with == or != is not safe NOTE value_type might not be a float type it could be anything? This suggest that we need some kind of metaprogramming technique to deal with this problem?
      bool operator==(Quaternion const & cmp) const    {      return m_s==cmp.m_s && m_v==cmp.m_v;    }
      bool operator!=(Quaternion const & cmp) const    {      return !(*this==cmp);                   }

    public:

      Quaternion & operator+= (Quaternion const & q )
      {
        m_s += q.m_s;
        m_v += q.m_v;
        return *this;
      }

      Quaternion & operator-= (Quaternion const & q )
      {
        m_s -= q.m_s;
        m_v -= q.m_v;
        return *this;
      }

      Quaternion operator+ ( Quaternion const &q ) const    {      return Quaternion(m_s + q.m_s, m_v + q.m_v);    }
      Quaternion operator- ( Quaternion const &q ) const    {      return Quaternion(m_s - q.m_s, m_v - q.m_v);    }

      Quaternion & operator*=(value_type const &s_val )
      {
        m_s *= s_val;
        m_v *= s_val;
        return *this;
      }

      Quaternion & operator/=( value_type const &s_val )
      {
        assert(s_val || !"Quaternion::operator/=(): division by zero");
        m_s /= s_val;
        m_v /= s_val;
        return *this;
      }

      Quaternion operator-() const  {  return Quaternion(-m_s,-m_v); }

      value_type operator*(Quaternion const &q) const  {  return m_s*q.m_s + m_v*q.m_v;  }

      /**
      * Multiplication of quaternions.
      * This method multiplies two quaternions with each other
      * and assigns the result to this Quaternion.
      *
      * @param b       The second Quaternion.
      */
      Quaternion operator%(Quaternion const & b)
      {
        return Quaternion( m_s*b.s()  - dot(m_v , b.v()),  cross(m_v , b.v()) + b.v()*m_s + m_v*b.s() );
      }

      /**
      * Quaternion Vector Multiplication.
      *
      * @param v    A Quaternion as a vector, i.e. zero scalar value.
      */
      Quaternion operator%(vector3_type const & v_val)
      {
        return Quaternion( - dot( m_v() , v_val) ,  cross(m_v , v_val) + v_val*m_s  );
      }

    public:

      /**
      * Assigns the quaternion to the identity rotation.
      * This is a commonly used constant. It corresponds to the identity matrix.
      */
      void identity()
      {
        m_s = value_traits::one();
        m_v.clear();
      }

      /**
      * This method constructs a unit Quaternion which represents the specified rotation around the x-axis.
      *
      * @param rad    The rotation angle in radians around the axis.
      */
      void Rx(value_type const & rad)
      {
        using std::cos;
        using std::sin;

        value_type teta  = rad/value_traits::two();
        value_type cteta = (value_type)( cos(teta) );
        value_type steta = (value_type)( sin(teta) );

        m_s    = cteta; 
        m_v(0) = steta;
        m_v(1) = value_traits::zero();
        m_v(2) = value_traits::zero();
      }

      /**
      * This method constructs a unit Quaternion which represents the specified rotation around the y-axis.
      *
      * @param rad   The rotation angle in radians around the axis.
      */
      void Ry(value_type const & rad)
      {
        using std::cos;
        using std::sin;

        value_type teta  = rad/value_traits::two();
        value_type cteta = (value_type)( cos(teta) );
        value_type steta = (value_type)( sin(teta) );

        m_s    = cteta;
        m_v(0) = value_traits::zero();
        m_v(1) = steta;
        m_v(2) = value_traits::zero();
      }

      /**
      * This method constructs a unit Quaternion which represents the specified rotation around the z-axis.
      *
      * @param rad    The rotation angle in radians around the axis.
      */
      void Rz(value_type const & rad)
      {
        using std::cos;
        using std::sin;

        value_type teta  = rad/value_traits::two();
        value_type cteta = (value_type)( cos(teta) );
        value_type steta = (value_type)( sin(teta) );

        m_s    = cteta;
        m_v(0) = value_traits::zero();
        m_v(1) = value_traits::zero();
        m_v(2) = steta;
      }

      /**
      * Rotate Vector by Quaternion.
      *
      *  computes  r' = q*r*conj(q)
      */
      vector3_type rotate(vector3_type const & r) const    {      return ((*this) % r  % conj(*this)).v();    }

    public:

      /**
      * Equality comparison with an error bound.
      * The test is exactly the same as with the method without error bound the only difference
      * is that the corresponding terms of the quaternions are said to be equal if they do not
      * differ by more than the error bound value.
      *
      * @param q           A reference to a Quaternion. This is the
      *                    Quaternion to compare with.
      * @param threshold   A reference to the acceptable error bound.
      *
      * @return            The test result. The return value is one if
      *                    the quaternions are equal otherwise the return
      *                    value is zero.
      */
      bool is_equal(Quaternion const & q,value_type const & threshold) const
      {
        using std::fabs;

        assert( threshold>=value_traits::zero() || !"is_equal(): threshold must be non-negative");

        return fabs(m_s-q.m_s)<threshold && m_v.is_equal(q.v(),threshold);
      }

    };


    template<typename T>
    inline Vector3<T> rotate(Quaternion<T> const & q, Vector3<T> const & r)
    {
      return prod( prod(q , r)  , conj(q)).v();
    }

    template<typename T>
    inline Quaternion<T> prod(Quaternion<T> const & a, Quaternion<T> const & b)
    {
      return Quaternion<T>(  a.s()*b.s()  - dot(a.v() , b.v()),  cross(a.v() , b.v()) + b.v()*a.s() + a.v()*b.s()  );
    }

    template<typename T>
    inline Quaternion<T> prod(Quaternion<T> const & a, Vector3<T> const & b)
    {
      return Quaternion<T>(  - dot(a.v() , b),   cross(a.v() , b) + b*a.s()  );
    }

    template<typename T>
    inline Quaternion<T> prod(Vector3<T> const & a, Quaternion<T> const & b)
    {
      return Quaternion<T>( - dot(a , b.v()),  cross(a , b.v()) + a*b.s()  );
    }

    template<typename T>
    inline Quaternion<T> operator%(Quaternion<T> const & a, Quaternion<T> const & b)  { return prod(a,b); }
    template<typename T>
    inline Quaternion<T> operator%(Quaternion<T> const & a, Vector3<T> const & b)     { return prod(a,b); }
    template<typename T>
    inline Quaternion<T> operator%(Vector3<T> const & a, Quaternion<T> const & b)     { return prod(a,b); }
    template <typename T, typename T2>
    inline Quaternion<T> operator*( const Quaternion<T> &q, const T2 &s_val )  {    return Quaternion<T>( q.s()*s_val, q.v()*s_val);  }
    template <typename T2, typename T>
    inline Quaternion<T> operator*( const T2 &s_val, const Quaternion<T> &q )  {    return Quaternion<T>( q.s()*s_val, q.v()*s_val);  }
    template <typename T, typename T2>
    inline Quaternion<T> operator/( const Quaternion<T> &q, const T2 &s_val )  {    return Quaternion<T>( q.s()/s_val, q.v()/s_val);  }
    template <typename T2, typename T>
    inline Quaternion<T> operator/( const T2 &s_val, const Quaternion<T> &q )  {    return Quaternion<T>( q.s()/s_val, q.v()/s_val);  }

    template<typename T>
    inline T const length(Quaternion<T> const & q)
    { 
      using std::sqrt;
      return sqrt( q*q );
    }

    template<typename T>
    inline Quaternion<T> unit(Quaternion<T> const & q)
    {
      typedef typename Quaternion<T>::value_traits   value_traits;

      using std::sqrt;
      using std::fabs;

      T l = length(q);

      if(fabs(l) > value_traits::zero())
        return Quaternion<T> (q.s()/l,q.v()/l);
      return Quaternion<T> (value_traits::zero(),value_traits::zero(),value_traits::zero(),value_traits::zero());
    }

    template<typename T>
    inline Quaternion<T> normalize(Quaternion<T> const & q)  {    return unit(q);      }

    /**
    * Natural Logarithm
    * Returns the Quaternion equal to the natural logarithm of
    * the specified Quaternion.
    *
    * @param q   A reference to an unit quaternion.
    * @return
    */
    template<typename T>
    inline Quaternion<T> log(Quaternion<T> const & q)
    {
      typedef typename Quaternion<T>::value_traits   value_traits;

      using std::acos;
      using std::sin;

      if(q.s()==value_traits::one() && is_zero(q.v()))
        return Quaternion<T>(value_traits::zero(),value_traits::zero(),value_traits::zero(),value_traits::zero());

      T teta = (T)( acos(q.s()) );
      T st   = (T)( sin(teta)   );
      return Quaternion<T>( value_traits::zero(), q.v()*(teta/st) );
    }

    /**
    * Orthogonal Quaternion.
    * This method sets this Quaternion to an orthogonal Quaternion
    * of the specified Quaternion. In other words the resulting
    * angle between the specified Quaternion and this Quaternion
    * is pi/2.
    */
    template<typename T>
    inline Quaternion<T> hat(Quaternion<T> const & q)
    {
      return Quaternion<T>( q.v()(2), - q.v()(1) , q.v()(0), -q.s());
    }

    /**
    * Exponent
    * Sets the Quaternion equal to the exponent of
    * the specified Quaternion.
    *
    * @param q    A reference to a pure Quaternion (zero
    *             T part).
    */
    template<typename T>
    inline Quaternion<T> exp(Quaternion<T> const & q)
    {
      using std::sqrt;
      using std::cos;
      using std::sin;

      //--- teta^2 x^2 + teta^2 y^2 +teta^2 z^2 =
      //--- teta^2 (x^2 + y^2 + z^2) = teta^2
      T teta = (T)(  sqrt(q.v() *q.v())  );
      T ct   = (T)(  cos(teta)           );
      T st   = (T)(  sin(teta)           );

      return Quaternion<T>(ct,q.v()*st);
    }

    /**
    * QLERP - Linear Interpolation of Quaterions.
    *
    * @param A		Quaternion A
    * @param B		Quaternion B
    * @param w    The weight
    *
    * @return     The resulting Quaternion, (1-w)A+w B. Note the resulting
    *             Quaternion may not be a unit quaternion even though A and B are
    *             unit quaternions. If a unit Quaternion is needed write
    *             unit(qlerp(A,B,w)).
    *
    *		-Added by spreak for the SBS algorithm, see OpenTissue/kinematics/skinning/SBS
    */
    template<typename T>
    // TODO why not simply call this function lerp?
    inline Quaternion<T> qlerp(Quaternion<T> const & A,Quaternion<T> const & B,T const & w)
    {
      typedef typename Quaternion<T>::value_traits   value_traits;

      assert(w>=value_traits::zero() || !"qlerp(): w must not be less than 0");
      assert(w<=value_traits::one()  || !"qlerp(): w must not be larger than 1");	  
      T mw = value_traits::one() - w; 
      return ((mw * A) + (w * B));
    }

    /**
    * Spherical Linear Interpolation of Quaternions.
    *
    * @param A
    * @param B
    * @param w        The weight
    *
    * @return          The resulting Quaternion.
    */
    template<typename T>
    inline Quaternion<T> slerp(Quaternion<T> const & A,Quaternion<T> const & B,T const & w)
    {
      typedef typename Quaternion<T>::value_traits   value_traits;

      using std::acos;
      using std::sin;

      assert(w>=value_traits::zero() || !"slerp(): w must not be less than 0");
      assert(w<=value_traits::one()  || !"slerp(): w must not be larger than 1");	  

      T q_tiny = (T)( 10e-7 );  // TODO prober constant type conversion?

      T norm = A*B;

      bool flip = false;
      if( norm < value_traits::zero() )
      {
        norm = -norm;
        flip = true;
      }
      T weight = w;
      T inv_weight;
      if(value_traits::one() - norm < q_tiny)
      {
        inv_weight = value_traits::one() - weight;
      }
      else
      {
        T theta    = (T)( acos(norm)                                          );
        T s_val    = (T)( value_traits::one() / sin(theta)                    ); 
        inv_weight = (T)( sin((value_traits::one() - weight) * theta) * s_val );
        weight     = (T)( sin(weight * theta) * s_val                         );
      }
      if(flip)
      {
        weight = -weight;
      }
      return ( inv_weight * A + weight * B);
    }

    /**
    * "Cubical" Spherical Interpolation.
    * In popular terms this corresponds to a cubic spline in
    * ordinary 3D space. However it is really a series of
    * spherical linear interpolations, which defines a
    * cubic on the unit Quaternion sphere.
    *
    * @param q0
    * @param q1
    * @param q2
    * @param q3
    * @param u
    *
    * @return
    */
    template<typename T>
    inline Quaternion<T> squad(
      Quaternion<T> const & q0
      , Quaternion<T> const & q1
      , Quaternion<T> const & q2
      , Quaternion<T> const & q3
      , T const & u
      )
    {
      typedef typename Quaternion<T>::value_traits   value_traits;

      assert(u>=value_traits::zero() || !"squad(): u must not be less than 0");
      assert(u<=value_traits::one()  || !"squad(): u must not be larger than 1");	  

      T u2 = value_traits::two() *u*(value_traits::one() -u); 
      return slerp( slerp(q0,q3,u), slerp(q1,q2,u), u2);
    }

    /**
    * Quaternion Conjugate.
    */
    template<typename T>
    inline Quaternion<T> conj(Quaternion<T> const & q)
    {
      return Quaternion<T>(q.s(),-q.v());
    }


    /**
    * Get Axis Angle Representation.
    * This function converts a unit-quaternion into the
    * equivalent angle-axis representation.
    *
    * @param Q       The quaternion
    * @param axis    Upon return this argument holds value of the equivalent rotation axis.
    * @param theta   Upon return this argument holds value of the equivalent rotation angle.
    */
    template<typename T>
    inline void get_axis_angle(Quaternion<T> const & Q,Vector3<T> & axis, T & theta)
    {
      using std::atan2;

      typedef typename Quaternion<T>::value_traits     value_traits;
      typedef          Vector3<T>                      V;

      //
      // By definition a unit quaternion Q can be written as
      //
      //    Q = [s,v] = [cos(theta/2), n sin(theta/2)]
      //
      // where n is a unit vector. This is the same as a rotation of
      // theta radian around the axis n.
      //
      //
      // Rotations are difficult to work with for several reasons.
      //
      // Firstly both Q and -Q represent the same rotation. This is
      // easily proven, rotate a arbitrary vector r by Q then we have
      //
      //   r^\prime = Q r Q^*
      //
      // Now rotate the same vector by -Q
      //
      //   r^\prime = (-Q) r (-Q)^* = Q r Q^*
      //
      // because -Q = [-s,-v] and (-Q)^* = [-s , v] = - [s,-v]^* = - Q^*.
      //
      // Thus the quaternion representation of a single rotation is not unique.
      //
      // Secondly the rotation it self is not well-posed. A rotation of theta
      // radians around the unit axis n could equally well be done as a rotation
      // of -theta radians around the negative unit axis n.
      //
      // This is seen by straightforward substitution
      //
      //  [ cos(-theta/2), sin(-theta/2) (-n) ] = [ cos(theta/2), sin(theta/2) n ]
      // 
      // Thus we get the same quaternion regardless of whether we
      // use (+theta,+n) or (-theta,-n).
      //
      //
      // From the Quaternion we see that
      //
      //   \frac{v}{\norm{v}}  = \frac{ sin(theta/2) n }{| sin(theta/2) | } = sign(sin(theta/2)) n
      //
      // Thus we can easily get the rotation axis. However, we can not immediately
      // determine the positive rotation axis direction. The problem boils down to the
      // fact that we can not see the sign of the sinus-factor.
      //
      // Let us proceed by setting
      //
      //   x =    cos(theta/2)   =  s
      //   y =  | sin(theta/2) | =  \norm{v}
      //
      // Then we basically have two possibilities for finding theta
      //
      //  theta_1 = 2 atan2( y, x)        equivalent to      sign(sin(theta/2)) = 1
      //
      // or 
      //
      //  theta_2 = 2 atan2( -y, x)       equivalent to      sign(sin(theta/2)) = -1
      //
      // If theta_1 is the solution we have
      //
      //  n = \frac{v}{\norm{v}}
      //
      // If theta_2 is the solution we must have
      //
      //  n = - \frac{v}{\norm{v}}
      //
      // Observe that we always have theta_2 = 2 pi - theta_1. Therefore theta_1 < theta_2.
      //
      // Let us imagine that we always choose $theta_1$ as the solution then
      // the corresponding quaternion for that solution would be
      //
      //
      //         Q_1 = [cos(theta_1/2),  sin(theta_1/2)   \frac{v}{\norm{v}}]
      //             = [s ,  \norm{v}   \frac{v}{\norm{v}}] 
      //             = Q
      //
      // Now if we choose theta_2 as the solution we would have
      //
      //         Q_2 = [cos(theta_2/2),  sin(theta_2/2)   -\frac{v}{\norm{v}}]
      //             = [s ,  -\norm{v}   -\frac{v}{\norm{v}}] 
      //             = [s ,  \norm{v}   \frac{v}{\norm{v}}] 
      //             = Q
      //
      // Thus we observe that regardless of which solution we pick we always have Q = Q_1 = Q_2.
      //
      // At this point one may be confused. However, it should be clear that theta_2 is equivalent
      // to the theta_1 rotation. The difference is simply that theta_2 corresponds to flipping the
      // rotation axis of the theta_1 case.
      //
      T const ct2   = Q.s();           //---   cos(theta/2)
      T const st2   = length( Q.v() ); //---  |sin(theta/2)|

      theta = value_traits::two()* atan2(st2,ct2);

      assert( st2 >= value_traits::zero()   || !"get_axis_angle(): |sin(theta/2)| must be non-negative");
      assert( theta >= value_traits::zero() || !"get_axis_angle(): theta must be non-negative");
      assert( is_number(theta)              || !"get_axis_angle(): NaN encountered");

      axis = st2 > value_traits::zero() ? Q.v() / st2 : V(value_traits::zero(), value_traits::zero(), value_traits::zero());
    }

    /**
    *  Get Rotation Angle wrt. an Axis.
    * Think of it as if you have a fixated body A so only body B is allowed to move.
    *
    * Let BF_B' indicate the initial orientation of body B's body frame
    * and BF_B the current orientation, now BF_A should be thought of as
    * being immovable ie. constant.
    *
    *   Q_initial : BF_A -> BF_B'
    *   Q_cur     : BF_A -> BF_B
    *
    * Now we must have the relation
    *
    *   Q_rel Q_initial = Q_cur
    *
    * From which we deduce
    *
    *   Q_rel = Q_cur conj(Q_initial)
    *
    * And we see that
    *
    *   Q_rel : BF_B' -> BF_B
    *
    * That is how much the body frame of body B have rotated (measured
    * with respect to the fixed body frame A).
    *
    *
    * @param Q_rel    Rotation from initial orientation of B to current orientation of B.
    * @param axis     The rotation axis in the body frame of body B.
    *
    * @return     The angle in radians.
    */
    template<typename T>
    inline T get_angle(Quaternion<T> const & Q_rel,Vector3<T> const & axis)
    {
      typedef typename Quaternion<T>::value_traits   value_traits;

      using std::atan2;

      //--- The angle between the two bodies is extracted from the Quaternion Q_rel
      //---
      //---    [s,v] = [ cos(theta/2) , sin(theta/2) * u ]
      //---
      //--- where s is a value_type and v is a 3-vector. u is a unit length axis and
      //--- theta is a rotation along that axis.
      //---
      //--- we can get theta/2 by:
      //---
      //---    theta/2 = atan2 ( sin(theta/2) , cos(theta/2) )
      //---
      //--- but we can not get sin(theta/2) directly, only its absolute value:
      //---
      //---    |v| = |sin(theta/2)| * |u|  = |sin(theta/2)|
      //---
      //--- using this value will have a strange effect.
      //---
      //--- Recall that there are two Quaternion representations of a given
      //--- rotation, q and -q.
      //---
      //--- Typically as a body rotates along the axis it will go through a
      //--- complete cycle using one representation and then the next cycle
      //--- will use the other representation.
      //---
      //--- This corresponds to u pointing in the direction of the joint axis and
      //--- then in the opposite direction. The result is that theta
      //--- will appear to go "backwards" every other cycle.
      //---
      //--- Here is a fix: if u points "away" from the direction of the joint
      //--- axis (i.e. more than 90 degrees) then use -q instead of q. This
      //--- represents the same rotation, but results in the cos(theta/2) value
      //--- being sign inverted.
      T ct2 = Q_rel.s();           //---   cos(theta/2)
      T st2 = length( Q_rel.v() ); //---  |sin(theta/2)|
      T theta = value_traits::zero();

      //--- Remember that Q_rel : BF_B' -> BF_B, so we need the axis in body B's local frame
      if( Q_rel.v() * axis  >= value_traits::zero())
      {
        //--- u points in direction of axis.
        //std::cout << "u points in direction of axis" << std::endl;
        theta = value_traits::two()* atan2(st2,ct2);
      }
      else
      {
        //--- u points in opposite direction.
        //std::cout << "u points in opposite direction" << std::endl;
        theta = value_traits::two() * atan2(st2,-ct2);
      }
      //--- The angle we get will be between 0..2*pi, but we want
      //--- to return angles between -pi..pi
      if (theta > value_traits::pi())   
        theta -= value_traits::two()*value_traits::pi();

      //--- The angle we've just extracted has the wrong sign (Why???).
      theta = -theta;
      //
      // Say we have a rotation, R, relating the coordinates of two frames X and Y, now
      // let the coordinates of the vector v be given in frame X as
      //
      //    [v]_X
      //
      // And in frame Y as
      //
      //    [v]_Y
      //
      // Now R is defined such that
      //
      //    R [v]_X = [v]_Y
      //
      // That is it changes the coordinates of v from X to Y.
      //
      // This is pretty straightforward, but there is some subtlety in it, say
      // frame Y is rotated theta radians around the z-axis, then the rotation
      // matrix relating the coordinates is the opposite rotation.
      //
      // What we want to measure is how much the frame axis have rotated, but
      // what we are given is a rotation transforming coordinates, therefore
      // we need to flip the sign of the extracted angle!!!
      return theta;
    }

    template<typename T>
    inline std::ostream & operator<< (std::ostream & o,Quaternion<T> const & q)
    {
      o << "[" << q.s() << "," << q.v()(0) << "," << q.v()(1) << "," << q.v()(2) << "]";
      return o;
    }

    template<typename T>
    inline std::istream & operator>>(std::istream & i,Quaternion<T> & q)
    {
      char dummy;

      i >> dummy;
      i >> q.s();
      i >> dummy;
      i >> q.v()(0);
      i >> dummy;
      i >> q.v()(1);
      i >> dummy;
      i >> q.v()(2);
      i >> dummy;

      return i;
    }

  } // namespace math

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_MATH_QUATERNION_H
#endif
