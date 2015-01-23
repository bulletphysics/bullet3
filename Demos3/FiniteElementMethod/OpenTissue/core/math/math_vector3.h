#ifndef OPENTISSUE_CORE_MATH_MATH_VECTORX3_H
#define OPENTISSUE_CORE_MATH_MATH_VECTORX3_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>
#include <OpenTissue/core/math/math_functions.h>
#include <OpenTissue/core/math/math_value_traits.h>



#include <string>
#include <cmath>
#include <cassert>

// TODO 2007-06-05 hod - Could not get this to work on linux.
// Might have somthing to do with http://gcc.gnu.org/faq.html#friend
//#include <iosfwd>
#include <iostream>


namespace OpenTissue
{

  namespace math
  {

    template <
      typename value_type_
      //, typename value_traits_ = ValueTraits<value_type_> 
    >
    class Vector3
    {
    protected:

      typedef typename OpenTissue::math::ValueTraits<value_type_>  value_traits_ ;  // TODO value_traits_ should be parameterized as a class template parameter.

    public:

      typedef value_traits_  value_traits;  ///< Convience typedef to make value traits accesible for all template functions using Vector3 types.
      typedef value_type_    value_type;    ///< Typedef is required for compliance with many other libraries and data containers!
      typedef size_t         index_type;    // TODO should the index type be template parameterized?

    private:

      // TODO 2007-02-17 KE: Refactor this stuff into an array

      value_type x; ///< The first coordinate of this vector.
      value_type y; ///< The second coordinate of this vector.
      value_type z; ///< The third coordinate of this vector.

    public:


      Vector3()
        : x( value_traits::zero()  ) 
        , y( value_traits::zero()  ) 
        , z( value_traits::zero()  )
      {}

      Vector3( Vector3 const & v ) 
        : x(v(0)) 
        , y(v(1)) 
        , z(v(2))  
      {} 

      explicit Vector3( value_type const& val )
        : x( val )
        , y( val )
        , z( val )
      {}

      template <typename T1,typename T2,typename T3> 
      Vector3( T1 const & x_val, T2 const & y_val, T3 const & z_val )
        : x( (value_type)(x_val) ) 
        , y( (value_type)(y_val) ) 
        , z( (value_type)(z_val) ) 
      {}

      ~Vector3()
      {}

      Vector3 & operator=(Vector3 const & cpy)
      {
        x=cpy(0);
        y=cpy(1);
        z=cpy(2);
        return *this;
      }

    public:

      void clear()  {    x = y = z = value_traits::zero();       }

      size_t size() const { return 3u;}

    public:

      value_type & operator() ( index_type index )
      {
        assert( (index>=0 && index<3)  || !"vecto3():index should be in range [0..2]");
        return *((&x) + index);
      }

      value_type const & operator() ( index_type index ) const
      {
        assert( (index>=0 && index<3)  || !"vecto3():index should be in range [0..2]");
        return *((&x) + index);
      }

      value_type & operator[] ( index_type index )
      {
        assert( (index>=0 && index<3)  || !"vecto3[]:index should be in range [0..2]");
        return *((&x) + index);
      }

      value_type const & operator[] ( index_type index ) const
      {
        assert( (index>=0 && index<3)  || !"vecto3[]:index should be in range [0..2]");
        return *((&x) + index);
      }

    public:

      bool operator< (Vector3 const & v) const
      {
        if ( x < v(0) ) return true;
        if ( x > v(0) ) return false;
        if ( y < v(1) ) return true;
        if ( y > v(1) ) return false;
        return z < v(2);
      }

      bool operator> (Vector3 const & v) const
      {
        if ( x > v(0) ) return true;
        if ( x < v(0) ) return false;
        if ( y > v(1) ) return true;
        if ( y < v(1) ) return false;
        return z > v(2);
      }

      // TODO: Comparing floats with == or != is not safe NOTE value_type might not be a float type it could be anything? This suggest that we need some kinf of metaprogramming technique to deal with ths problem?
      bool operator==(Vector3 const & cmp) const  {  return x==cmp.x && y==cmp.y && z==cmp.z;   }
      bool operator!=(Vector3 const & cmp) const  {  return x!=cmp.x || y!=cmp.y || z!=cmp.z;   }

    public:

      Vector3 & operator+= ( Vector3 const & v )
      {
        x += v(0);
        y += v(1);
        z += v(2);
        return *this;
      }

      Vector3 & operator-= (  Vector3 const & v )
      {
        x -= v(0);
        y -= v(1);
        z -= v(2);
        return *this;
      }

      Vector3 & operator*=( value_type const & s )
      {
        x *= s;
        y *= s;
        z *= s;
        return *this;
      }

      Vector3 & operator/=( value_type const & s )
      {
        assert(s || !"Vector3::/=(): division by zero");
        x /= s;
        y /= s;
        z /= s;
        return *this;
      }

      Vector3    operator+ ( Vector3 const & v ) const {  return Vector3( x+v(0), y+v(1), z+v(2));                      }
      Vector3    operator- ( Vector3 const & v ) const {  return Vector3( x-v(0), y-v(1), z-v(2));                      }
      Vector3    operator- (                   ) const {  return Vector3(-x,-y,-z);                                     }
      Vector3    operator% ( Vector3 const & v ) const {  return Vector3(y*v(2)-v(1)*z, v(0)*z-x*v(2), x*v(1)-v(0)*y);  }
      value_type operator* ( Vector3 const & v ) const {  return x*v(0) + y*v(1) + z*v(2);                              }
      bool       operator<=( Vector3 const & v ) const {  return x <= v(0) && y <= v(1) && z <= v(2);                   }
      bool       operator>=( Vector3 const & v ) const {  return x >= v(0) && y >= v(1) && z >= v(2);                   }

    public:

      friend Vector3 fabs( Vector3 const & v )
      {
        using std::fabs;
        return Vector3 ( 
          (value_type)( fabs( v(0) ) )
          , (value_type)( fabs( v(1) ) )
          , (value_type)( fabs( v(2) ) ) 
          );
      }

      friend Vector3 min( Vector3 const & A, Vector3 const & B )
      {
        using std::min;
        return Vector3( min( A(0), B(0) ), min( A(1), B(1) ), min( A(2), B(2) ) );
      }

      friend Vector3 max( Vector3 const & A, Vector3 const & B )
      {
        using std::max;
        return Vector3( max( A(0), B(0) ), max( A(1), B(1) ), max( A(2), B(2) ) );
      }

      friend Vector3 floor(Vector3 const & v)
      {
        using std::floor;
        return Vector3(
          (value_type)( floor(v(0)) )
          , (value_type)( floor(v(1)) )
          , (value_type)( floor(v(2)) )
          );
      }

      friend Vector3 ceil(Vector3 const & v)
      {
        using std::ceil;
        return Vector3(
          (value_type)( ceil(v(0)) )
          , (value_type)( ceil(v(1)) )
          , (value_type)( ceil(v(2)) )
          );
      }

    friend std::ostream & operator<< (std::ostream & o,Vector3 const & v)
    {
      o << "[";
      o << v(0);
      o << ",";
      o << v(1);
      o << "," << v(2) << "]";
      return o;
    }

    friend std::istream & operator>>(std::istream & i,Vector3 & v)
    {
      char dummy;
      i >> dummy;
      i >> v(0);
      i >> dummy;
      i >> v(1);
      i >> dummy;
      i >> v(2);
      i >> dummy;
      return i;
    }
    public:

      // TODO 2007-02-17 KE: Kill this it should be handled with comparator traits in operator== and operator!=
      bool is_equal(Vector3 const & v, value_type const & threshold) const
      {
        using std::fabs;
        return fabs(x-v.x)<=threshold && fabs(y-v.y)<=threshold && fabs(z-v.z)<=threshold;
      }

    };  // class Vector3


    //////////////////////////////////////////////////////////////////////////
    /// Declaration of vector3 non-member functions
    //////////////////////////////////////////////////////////////////////////

    template <typename T>
    inline Vector3<T> round(Vector3<T> const & v)
    {
      using std::floor;

      static T const half = detail::half<T>();

      return Vector3<T> (
        floor( v(0) + half )
        , floor( v(1) + half )
        , floor( v(2) + half ) 
        );
    }

    template <typename T>
    inline typename Vector3<T>::index_type min_index(Vector3<T> const & v)
    {
      return v(0) <= v(1) && v(0) < v(2) ? 0 : v(1) <= v(0) && v(1) < v(2) ? 1 : 2;
    }
    
    template <typename T>
    inline typename Vector3<T>::index_type max_index(Vector3<T> const & v)
    {
      return v(2) >= v(0) && v(2) >= v(1) ? 2 : v(1) >= v(0) && v(1) > v(2) ? 1 : 0;
    }

    template <typename T>
    inline typename Vector3<T>::index_type mid_index(Vector3<T> const & v)
    {
      typename Vector3<T>::index_type test = min_index(v) + max_index(v);
      return test == 2 ? 1 : test == 1 ? 2 : 0;
    }

    template <typename T>
    inline T min_value(Vector3<T> const & v)
    {
      using std::min;
      return min(v(0),min(v(1),v(2)));
    }

    template <typename T>
    inline T max_value(Vector3<T> const & v)
    {
      using std::max;
      return max(v(0),max(v(1),v(2)));
    }

    template <typename T>
    inline T mid_value(Vector3<T> const & v)
    {
      return v(mid_index(v));
    }

    template <typename T>
    inline bool is_zero(Vector3<T> const & v, T const & threshold)
    {
      using std::fabs;
      return fabs(v(0))<=threshold && fabs(v(1))<=threshold && fabs(v(2))<=threshold;
    }

    template <typename T>
    inline bool is_zero(Vector3<T> const & v) 
    { 
      typedef typename Vector3<T>::value_traits   value_traits;
      return is_zero(v,value_traits::zero()); 
    }




    template <typename T>
    inline Vector3<T> cross( Vector3<T> const & a, Vector3<T> const & b )
    {
      return Vector3<T>( a[1] * b[2] - b[1] * a[2], -a[0] * b[2] + b[0] * a[2],  a[0] * b[1] - b[0] * a[1] );
    }

    template <typename T>
    inline T dot( Vector3<T> const & a, Vector3<T> const & b )  {    return a(0)*b(0) + a(1)*b(1) + a(2)*b(2);  }

    template <typename T>
    inline T inner_prod( Vector3<T> const & a, Vector3<T> const & b )  {    return dot(a,b);  }

    template<typename T>
    inline T length(Vector3<T> const & v)
    {
      using std::sqrt;
      return (T)( sqrt( dot(v,v) ) );
    }

    template<typename T>
    inline T sqr_length(Vector3<T> const & v) { return (T)(v*v); }

    template<typename T>
    inline T norm(Vector3<T> const & v)   {    return length(v);  }

    template<typename T>
    inline T norm_1(Vector3<T> const & v)   
    {
      using std::fabs;
      using std::max;      
      return max( fabs(v(0)), max(  fabs(v(1)), fabs(v(2))  ) );
    }

    template<typename T>
    inline T distance(Vector3<T> const & a, Vector3<T> const & b)
    {
      return length(b-a);
    }

    template<typename T>
    inline T sqr_distance(Vector3<T> const & a, Vector3<T> const & b)
    {
      return sqr_length(b-a);
    }

    template <typename T>
    inline Vector3<T> sgn( Vector3<T> const & v )
    {
      using OpenTissue::math::sgn;
      return Vector3<T>(sgn(v(0)), sgn(v(1)), sgn(v(2)));
    }

    template <typename T>
    inline Vector3<T> unit( Vector3<T> const & v )
    {
      typedef typename Vector3<T>::value_traits   value_traits;


      using std::sqrt;
      T const l = length(v);
      if (l <= value_traits::zero())
      {
        return Vector3<T>( value_traits::zero() );
      }
      T const inv = value_traits::one()/l; 
      return Vector3<T>( inv*v(0), inv*v(1), inv*v(2) );
    }

    template <typename T>
    inline Vector3<T> normalize( Vector3<T> const & v ) {    return unit(v);  }

    template <typename T>
    inline void truncate(Vector3<T> & v, T const & precision_value)
    {
      typedef typename Vector3<T>::value_traits   value_traits;

      v(0) = ((v(0)>-precision_value)&&(v(0)<precision_value)) ? value_traits::zero() : v(0);
      v(1) = ((v(1)>-precision_value)&&(v(1)<precision_value)) ? value_traits::zero() : v(1);
      v(2) = ((v(2)>-precision_value)&&(v(2)<precision_value)) ? value_traits::zero() : v(2);
    }


    template <typename T>
    inline void truncate(Vector3<T> & v)
    { 
      typedef typename Vector3<T>::value_traits   value_traits;

      truncate( v, value_traits::zero() ); 
    }

    /**
    * Get Orthogonal Vector.
    *
    * @param v   Any vector
    *
    * @return    A vector orthogonal to v.
    */
    template <typename T>
    inline Vector3<T> orthogonal(Vector3<T> const & v)
    {
      typedef typename Vector3<T>::value_traits   value_traits;

      using std::fabs;

      //--- Find a vector not collinear with v, we simply pick the
      //--- y-axis direction as our prefered direction, afterwards
      //--- we test if this was a good choice if not we pick the
      //--- x-axis direction
      Vector3<T> tmp;
      if (fabs(v(1)) > fabs(v(0)))
        tmp = Vector3<T>(value_traits::one(),value_traits::zero(),value_traits::zero());
      else
        tmp = Vector3<T>(value_traits::zero(),value_traits::zero(),value_traits::one());
      //--- Now we find an orthogonal vector by using
      //--- the cross product
      return cross(v,tmp);
    }

    template <typename T, typename T2>
    inline Vector3<T> operator*( Vector3<T> const& v, T2 const& s )  {    return Vector3<T>( v(0)*s, v(1)*s, v(2)*s );  }

    template <typename T2, typename T>
    inline Vector3<T> operator*( T2 const& s, Vector3<T> const& v )  {    return Vector3<T>( v(0)*s, v(1)*s, v(2)*s );  }

    template <typename T, typename T2>
    inline Vector3<T> operator/( Vector3<T> const& v, T2 const& s )  {    return Vector3<T>( v(0)/s, v(1)/s, v(2)/s );  }

    /**
    * Compute Orthonormal Vectors.
    * Compute unit vectors of a right handed coordinate frame, given initial z-axis (k-vector).
    *
    * @param i  Upon return contains a orthogonal vector to j and k
    * @param j  Upon return contains a orthogonal vector to i and k
    * @param k  A given direction for the last vector, assumed to be a unit-vector.
    */
    template<typename vector3_type>
    inline void orthonormal_vectors( vector3_type & i, vector3_type & j, vector3_type const & k )
    {
      typedef typename vector3_type::value_traits   value_traits;

      using std::fabs;
      vector3_type m_abs_k = fabs( k);

      if ( m_abs_k( 0 ) > m_abs_k( 1 ) )
      {
        if ( m_abs_k( 0 ) > m_abs_k( 2 ) )
          i = vector3_type( value_traits::zero(), value_traits::one(), value_traits::zero() );
        else
          i = vector3_type( value_traits::one(), value_traits::zero(), value_traits::zero() );
      }
      else
      {
        if ( m_abs_k( 1 ) > m_abs_k( 2 ) )
          i = vector3_type( value_traits::zero(), value_traits::zero(), value_traits::one() );
        else
          i = vector3_type( value_traits::one(), value_traits::zero(), value_traits::zero() );
      }
      j = unit( cross(k,i) );
      i = cross(j,k);
    }

    /**
    * Get increasing order of vector elements as a permutation array.
    *
    * Note this template function generalizes to any size of vectors
    * (not just 3-dimensional ones).
    *
    * @param  v     The vector to be examined.
    * @param  pi    Upon return this container contains the permuation
    *               order, v(pi[0]) is smallest element of v,  v(pi[1]) the
    *               next smallest and so on.
    */
    template<typename vector_type,typename permutation_container>
    inline void get_increasing_order( vector_type const & v, permutation_container & pi )
    {
      unsigned int n = v.size();

      for(unsigned int i=0u;i<n;++i)
        pi[i] = i;

      //--- Use insertion sort to find the increasing order of elements in the vector.
      for ( unsigned int i = 1u;i < n; ++i )
      {
        for ( unsigned int j = i;j > 0u;--j )
        {
          if ( v( pi[ j ] ) < v( pi[ j - 1 ] ) )
          {
            unsigned int tmp = pi[ j - 1 ];
            pi[ j - 1 ] = pi[ j ];
            pi[ j ] = tmp;
          }
        }
      }
    }


    namespace detail
    {

      template <typename T>
      inline Vector3<T> const & axis_x()
      {
        static Vector3<T> const xa(one<T>(), zero<T>(), zero<T>());
        return xa;
      }


      template <typename T>
      inline Vector3<T> const & axis_y()
      {
        static Vector3<T> const ya(zero<T>(), one<T>(), zero<T>());
        return ya;
      }


      template <typename T>
      inline Vector3<T> const & axis_z()
      {
        static Vector3<T> const za(zero<T>(), zero<T>(), one<T>());
        return za;
      }

    }

  } // namespace math

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_MATH_VECTORX3_H
#endif
