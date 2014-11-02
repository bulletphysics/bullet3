#ifndef OPENTISSUE_CORE_MATH_MATH_MATRIX3X3_H
#define OPENTISSUE_CORE_MATH_MATH_MATRIX3X3_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_vector3.h>
#include <OpenTissue/core/math/math_value_traits.h>
#include <OpenTissue/core/math/math_eigen_system_decomposition.h>

#include <cmath>
#include <cassert>
#include <iosfwd>


namespace OpenTissue
{

  namespace math
  {

    template <typename > class Quaternion;

    template<
      typename value_type_
      //, typename value_traits_ = ValueTraits<value_type_> 
    >
    class Matrix3x3
    {
    protected:

      typedef typename math::ValueTraits<value_type_>  value_traits_ ;  // TODO value_traits_ should be parameterized as a class template parameter.

    public:

      typedef value_traits_         value_traits;  ///< Convience typedef to make value traits accesible for all template functions using Vector3 types.
      typedef value_type_           value_type;    ///< Typedef is required for compliance with many other libraries and data containers!
      typedef Vector3<value_type>   vector3_type;  // TODO should Vector3 type be template parameterized?
      typedef Quaternion<value_type>   quaternion_type;
      typedef size_t                index_type;    // TODO should the index type be template parameterized?

    protected:

      // TODO 2007-02-19 KE: Refactor this stuff into an array
      vector3_type  m_row0;  ///<   The 1st row of the matrix
      vector3_type  m_row1;  ///<   The 2nd row of the matrix
      vector3_type  m_row2;  ///<   The 3rd row of the matrix

    public:

      Matrix3x3()
        : m_row0(value_traits::zero() ,value_traits::zero() ,value_traits::zero() )
        , m_row1(value_traits::zero() ,value_traits::zero() ,value_traits::zero() )
        , m_row2(value_traits::zero() ,value_traits::zero() ,value_traits::zero() )
      {}

      ~Matrix3x3(){}

      explicit Matrix3x3(
          value_type const & m00      , value_type const & m01      , value_type const & m02
        , value_type const & m10      , value_type const & m11      , value_type const & m12
        , value_type const & m20      , value_type const & m21      , value_type const & m22
        )
        : m_row0(m00,m01,m02)
        , m_row1(m10,m11,m12)
        , m_row2(m20,m21,m22)
      {}


      explicit Matrix3x3(vector3_type const & row0, vector3_type const & row1, vector3_type const & row2)
        : m_row0(row0)
        , m_row1(row1)
        , m_row2(row2)
      {}

      explicit Matrix3x3(quaternion_type const & q)    {      *this = q;    }

      Matrix3x3(Matrix3x3 const & M)
        : m_row0(M.m_row0)
        , m_row1(M.m_row1)
        , m_row2(M.m_row2)
      {}

    public:

      value_type & operator()(index_type i, index_type j)
      { 
        assert( ( i>=0 && i<3 ) || !"Matrix3x3::(i,j) i must be in range [0..2]");
        assert( ( j>=0 && j<3 ) || !"Matrix3x3::(i,j) j must be in range [0..2]");
        return (*(&m_row0+i))(j); 
      }

      value_type const & operator()(index_type i, index_type j) const 
      { 
        assert( ( i>=0 && i<3 ) || !"Matrix3x3::(i,j) i must be in range [0..2]");
        assert( ( j>=0 && j<3 ) || !"Matrix3x3::(i,j) j must be in range [0..2]");
        return (*(&m_row0+i))(j); 
      }

      vector3_type & operator[](index_type i)
      { 
        assert( ( i>=0 && i<3 ) || !"Matrix3x3::(i,j) i must be in range [0..2]");
        return *(&m_row0+i); 
      }

      vector3_type const & operator[](index_type i) const 
      { 
        assert( ( i>=0 && i<3 ) || !"Matrix3x3::(i,j) i must be in range [0..2]");
        return *(&m_row0+i); 
      }

      Matrix3x3 & operator=( Matrix3x3 const & cpy )
      {
        m_row0=cpy.m_row0;
        m_row1=cpy.m_row1;
        m_row2=cpy.m_row2; 
        return *this;
      }

      // TODO: Comparing floats with == or != is not safe NOTE T might not be a float type it could be anything? This suggest that we need some kinf of metaprogramming technique to deal with ths problem?
      bool operator==(Matrix3x3 const & cmp ) const 
      {
        return (m_row0==cmp.m_row0) && (m_row1==cmp.m_row1) && (m_row2==cmp.m_row2);
      }

      bool operator!=( Matrix3x3 const & cmp ) const 
      {
        return !(*this==cmp);
      }

      Matrix3x3   operator+  ( Matrix3x3 const & m   ) const { return Matrix3x3( m_row0+m.m_row0, m_row1+m.m_row1, m_row2+m.m_row2); }
      Matrix3x3   operator-  ( Matrix3x3 const & m   ) const { return Matrix3x3( m_row0-m.m_row0, m_row1-m.m_row1, m_row2-m.m_row2); }
      Matrix3x3 & operator+= ( Matrix3x3 const & m   )       { m_row0+=m.m_row0; m_row1+=m.m_row1; m_row2+=m.m_row2; return *this; }
      Matrix3x3 & operator-= ( Matrix3x3 const & m   )       { m_row0-=m.m_row0; m_row1-=m.m_row1; m_row2-=m.m_row2; return *this; }
      Matrix3x3 & operator*= ( value_type const & s  )       {m_row0*=s;m_row1*=s;m_row2*=s;return *this;}

      Matrix3x3 & operator/= ( value_type const & s  )       
      {
        assert(s || !"Matrix3x3/=(): division by zero");
        m_row0/=s;
        m_row1/=s;
        m_row2/=s;
        return *this;
      }

      vector3_type operator*(vector3_type const &v) const { return vector3_type(m_row0*v, m_row1*v, m_row2*v); }
      Matrix3x3    operator-()                      const { return Matrix3x3(-m_row0,-m_row1,-m_row2);         }

      size_t size1() const { return 3u; }
      size_t size2() const { return 3u; }

      /**
      * Assigns this quaternion to a matrix.
      * This method performs a conversion of a quaternion that represents a rotation into the correponding rotationmatrix.
      *
      * @param q          A reference to a quaternion. This is the quaternion that represents a rotation.
      */
      Matrix3x3  & operator=(quaternion_type const & q)
      {
        m_row0(0) = value_traits::one() - value_traits::two() * ( (q.v()(1)*q.v()(1)) + (q.v()(2)*q.v()(2)));
        m_row1(1) = value_traits::one() - value_traits::two() * ( (q.v()(0)*q.v()(0)) + (q.v()(2)*q.v()(2)));
        m_row2(2) = value_traits::one() - value_traits::two() * ( (q.v()(1)*q.v()(1)) + (q.v()(0)*q.v()(0)));
        m_row1(0) =                       value_traits::two() * ( (q.v()(0)*q.v()(1)) + (q.s()*q.v()(2)));
        m_row0(1) =                       value_traits::two() * ( (q.v()(0)*q.v()(1)) - (q.s()*q.v()(2)));
        m_row2(0) =                       value_traits::two() * (-(q.s()*q.v()(1))    + (q.v()(0)*q.v()(2)));
        m_row0(2) =                       value_traits::two() * ( (q.s()*q.v()(1))    + (q.v()(0)*q.v()(2)));
        m_row2(1) =                       value_traits::two() * ( (q.v()(2)*q.v()(1)) + (q.s()*q.v()(0)));
        m_row1(2) =                       value_traits::two() * ( (q.v()(2)*q.v()(1)) - (q.s()*q.v()(0)));
        return *this;
      }

      void clear()
      {
        m_row0.clear();
        m_row1.clear();
        m_row2.clear();
      }

    public:

      friend Matrix3x3 fabs(Matrix3x3 const & A)
      {
        using std::fabs;

        return Matrix3x3(
          fabs(A(0,0)),  fabs(A(0,1)), fabs(A(0,2)),
          fabs(A(1,0)),  fabs(A(1,1)), fabs(A(1,2)),
          fabs(A(2,0)),  fabs(A(2,1)), fabs(A(2,2))
          );
      }

    public:

      vector3_type column(index_type i) const
      {
        assert((i>=0 && i<3) || !"Matrix3x3::column(): index must be in range [0..2]");

        return vector3_type(m_row0(i), m_row1(i), m_row2(i));
      }

      void set_column(index_type i, vector3_type const & column)
      {
        assert((i>=0 && i<3) || !"Matrix3x3::set_column(): index must be in range [0..2]");

        m_row0(i) = column(0);
        m_row1(i) = column(1);
        m_row2(i) = column(2);      
      }

      vector3_type       & row(index_type i)       { return (*this)[i]; }
      vector3_type const & row(index_type i) const { return (*this)[i]; }

    }; // class Matrix3x3


    template<typename T>
    inline Matrix3x3<T> operator*( Matrix3x3<T> const & m, T const &s )
    {
      return Matrix3x3<T>(m.row(0)*s, m.row(1)*s, m.row(2)*s);
    }

    template<typename T>
    inline Matrix3x3<T> operator*( T const & s, Matrix3x3<T> const & m )
    {
      return Matrix3x3<T>(m.row(0)*s, m.row(1)*s, m.row(2)*s);
    }

    template<typename T>
    inline Matrix3x3<T> operator/( Matrix3x3<T> const & m, T const & s )
    {
      return Matrix3x3<T>(m.row(0)/s, m.row(1)/s, m.row(2)/s);
    }

    template<typename T>
    inline Matrix3x3<T> operator/( T const & s, Matrix3x3<T> const & m )
    {
      return Matrix3x3<T>(m.row(0)/s, m.row(1)/s, m.row(2)/s);
    }

    template<typename T>
    inline Matrix3x3<T> operator*(Matrix3x3<T> const & A,Matrix3x3<T> const & B)
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;
      Matrix3x3<T> C;

      for (int c=0; c<3; ++c)
        for (int r=0; r<3; ++r){
          C(r,c) = value_traits::zero();
          for(int i=0; i<3; ++i){
            C(r,c) += A(r,i) * B(i,c);
          }
        }
        return C;
    }

    /**
    * Creates a rotation matrix.
    * This method returns a rotation matrix around the x-axe. It
    * assumes that post-multiplication by colum vectors is used.
    *
    * @param radians           The rotation angle in radians.
    */
    template<typename T>
    inline Matrix3x3<T> Rx(T const & radians)
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      using std::cos;
      using std::sin;

      T cosinus = (T)( cos(radians) );
      T sinus   = (T)( sin(radians) );

      return Matrix3x3<T>(
        value_traits::one(), value_traits::zero(),        value_traits::zero(),
        value_traits::zero(),              cosinus,                      -sinus,
        value_traits::zero(),                sinus,                      cosinus
        );
    }

    /**
    * Creates a rotation matrix.
    * This method returns a rotation matrix around the y-axe. It
    * assumes that post-multiplication by colum vectors is used.
    *
    * @param radians           The rotation angle in radians.
    */
    template<typename T>
    inline Matrix3x3<T> Ry(T const & radians)
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      using std::cos;
      using std::sin;

      T cosinus = (T)( cos(radians) );
      T sinus   = (T)( sin(radians) );
      return Matrix3x3<T>(
        cosinus,   value_traits::zero(),                   sinus,
        value_traits::zero(),    value_traits::one(),    value_traits::zero(),
        -sinus,   value_traits::zero(),                 cosinus
        );
    }

    /**
    * Creates a rotation matrix.
    * This method returns a rotation matrix around the z-axe. It assumes that post-multiplication by colum vectors is used.
    *
    * @param radians           The rotation angle in radians.
    */
    template<typename T>
    inline Matrix3x3<T> Rz(T const & radians)
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      using std::cos;
      using std::sin;

      T cosinus = (T)( cos(radians) );
      T sinus   = (T)( sin(radians) );

      return Matrix3x3<T>(
        cosinus,                                      -sinus,       value_traits::zero(),
        sinus,                                       cosinus,       value_traits::zero(),
        value_traits::zero(),           value_traits::zero(),        value_traits::one()
        );
    }

    /**
    * Creates a rotation matrix.
    * This method returns a general rotation matrix around a specified axe. It assumes that post-multiplication by colum vectors is used.
    *
    * @param radians           The rotation angle in radians.
    * @param axe               A vector. This is the rotation axe.
    */
    template<typename T>
    inline Matrix3x3<T> Ru(T const & radians, Vector3<T> const & axis)
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      using std::cos;
      using std::sin;

      T cosinus = (T)( cos(radians) );
      T sinus   = (T)( sin(radians) );
      Vector3<T> u = unit(axis);

      //Foley p.227 (5.76)
      return Matrix3x3<T>(
        u(0)*u(0) + cosinus*(value_traits::one() - u(0)*u(0)),   u(0)*u(1)*(value_traits::one()-cosinus) - sinus*u(2),   u(0)*u(2)*(value_traits::one()-cosinus) + sinus*u(1),
        u(0)*u(1)*(value_traits::one()-cosinus) + sinus*u(2),    u(1)*u(1) + cosinus*(value_traits::one() - u(1)*u(1)),  u(1)*u(2)*(value_traits::one()-cosinus) - sinus*u(0),
        u(0)*u(2)*(value_traits::one()-cosinus) - sinus*u(1),    u(1)*u(2)*(value_traits::one()-cosinus) + sinus*u(0),   u(2)*u(2) + cosinus*(value_traits::one() - u(2)*u(2))
        );
    }

    /**
    * Direction of Flight (DoF)
    *
    * @param k   The desired direction of flight.
    */
    template<typename T>
    inline Matrix3x3<T> z_dof(Vector3<T> const & k)
    {
      Vector3<T> i,j;
      orthonormal_vectors(i,j,unit(k));
      return Matrix3x3<T>(
        i(0) , j(0), k(0)
        , i(1) , j(1), k(1)
        , i(2) , j(2), k(2)
        );
    }

    template<typename T>
    inline bool is_orthonormal(Matrix3x3<T> const & M,T const & threshold)
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      using std::fabs;

      assert(threshold>=value_traits::zero() || !"is_orthonormal(): threshold must be non-negative");

      T dot01 = M.m_row0*M.m_row1;
      T dot02 = M.m_row0*M.m_row2;
      T dot12 = M.m_row1*M.m_row2;
      if(fabs(dot01)>threshold) return false;
      if(fabs(dot02)>threshold) return false;
      if(fabs(dot12)>threshold) return false;
      T dot00 = M.m_row0*M.m_row0;
      T dot11 = M.m_row1*M.m_row1;
      T dot22 = M.m_row2*M.m_row2;
      if((dot00-value_traits::one())>threshold)        return false;
      if((dot11-value_traits::one())>threshold)        return false;
      if((dot22-value_traits::one())>threshold)        return false;
      return true;
    }

    template<typename T>
    inline bool is_orthonormal(Matrix3x3<T> const & M) 
    {  
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      return is_orthonormal(M, value_traits::zero()); 
    }

    template<typename T>
    inline bool is_zero(Matrix3x3<T> M, T const & threshold)
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      using std::fabs;

      assert(threshold>=value_traits::zero() || !"is_zero(): threshold must be non-negative");

      if(fabs(M(0,0))>threshold)      return false;
      if(fabs(M(0,1))>threshold)      return false;
      if(fabs(M(0,2))>threshold)      return false;
      if(fabs(M(1,0))>threshold)      return false;
      if(fabs(M(1,1))>threshold)      return false;
      if(fabs(M(1,2))>threshold)      return false;
      if(fabs(M(2,0))>threshold)      return false;
      if(fabs(M(2,1))>threshold)      return false;
      if(fabs(M(2,2))>threshold)      return false;
      return true;
    }

    template<typename T>
    inline bool is_zero(Matrix3x3<T> const & M) 
    {  
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      return is_zero(M, value_traits::zero()); 
    }

    template<typename T>
    inline bool is_symmetric(Matrix3x3<T> M, T const & threshold)
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      using std::fabs;

      assert(threshold>=value_traits::zero() || !"is_symmetric(): threshold must be non-negative");

      if(fabs(M(0,1)-M(1,0))>threshold)      return false;
      if(fabs(M(0,2)-M(2,0))>threshold)      return false;
      if(fabs(M(1,2)-M(2,1))>threshold)      return false;
      return true;
    }

    template<typename T>
    inline bool is_symmetric(Matrix3x3<T> const & M) 
    {  
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      return is_symmetric(M, value_traits::zero()); 
    }

    template<typename T>
    inline bool is_diagonal(Matrix3x3<T> M, T const & threshold)
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      using std::fabs;

      assert(threshold>=value_traits::zero() || !"is_diagonal(): threshold must be non-negative");

      if(fabs(M(0,1))>threshold)      return false;
      if(fabs(M(0,2))>threshold)      return false;
      if(fabs(M(1,0))>threshold)      return false;
      if(fabs(M(1,2))>threshold)      return false;
      if(fabs(M(2,0))>threshold)      return false;
      if(fabs(M(2,1))>threshold)      return false;
      return true;
    }

    template<typename T>
    inline bool is_diagonal(Matrix3x3<T> const & M) 
    {  
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      return is_diagonal(M, value_traits::zero()); 
    }

    template<typename T>
    inline bool is_identity(Matrix3x3<T> M, T const & threshold)
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      using std::fabs;

      assert(threshold>=value_traits::zero() || !"is_identity(): threshold must be non-negative");

      if(fabs(M(0,0)-value_traits::one())>threshold)    return false;
      if(fabs(M(0,1))>threshold)      return false;
      if(fabs(M(0,2))>threshold)      return false;
      if(fabs(M(1,0))>threshold)      return false;
      if(fabs(M(1,1)-value_traits::one())>threshold)    return false;
      if(fabs(M(1,2))>threshold)      return false;
      if(fabs(M(2,0))>threshold)      return false;
      if(fabs(M(2,1))>threshold)      return false;
      if(fabs(M(2,2)-value_traits::one())>threshold)    return false;
      return true;
    }

    template<typename T>
    inline bool is_identity(Matrix3x3<T> const & M) 
    {  
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      return is_identity(M, value_traits::zero()); 
    }

    template <typename T>
    inline Matrix3x3<T> outer_prod( Vector3<T> const & v1, Vector3<T> const & v2 )
    {
      return Matrix3x3<T>(
        ( v1( 0 ) * v2( 0 ) ), ( v1( 0 ) * v2( 1 ) ), ( v1( 0 ) * v2( 2 ) ),
        ( v1( 1 ) * v2( 0 ) ), ( v1( 1 ) * v2( 1 ) ), ( v1( 1 ) * v2( 2 ) ),
        ( v1( 2 ) * v2( 0 ) ), ( v1( 2 ) * v2( 1 ) ), ( v1( 2 ) * v2( 2 ) ) 
        );
    }

    template<typename T>
    inline Matrix3x3<T> ortonormalize(Matrix3x3<T> const & A)
    {
      typedef typename Matrix3x3<T>::vector3_type  vector3_type;

      vector3_type   row0(A(0,0),A(0,1),A(0,2));
      vector3_type   row1(A(1,0),A(1,1),A(1,2));
      vector3_type   row2(A(2,0),A(2,1),A(2,2));

      T const l0 = length(row0);
      if(l0) row0 /= l0;

      row1 -=  row0 * dot(row0 , row1);
      T const l1 = length(row1);
      if(l1) row1 /= l1;

      row2 = cross( row0 , row1);

      return Matrix3x3<T>(
        row0(0), row0(1), row0(2),
        row1(0), row1(1), row1(2),
        row2(0), row2(1), row2(2)
        );
    }

    template<typename T>
    inline Matrix3x3<T> trans(Matrix3x3<T> const & M)
    {
      return Matrix3x3<T>(
        M(0,0),  M(1,0), M(2,0),
        M(0,1),  M(1,1), M(2,1),
        M(0,2),  M(1,2), M(2,2)
        );
    }


    template<typename T>
    inline Matrix3x3<T> diag(T const & d0,T const & d1,T const & d2 )
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      return Matrix3x3<T>(
        d0,  value_traits::zero(),  value_traits::zero(),
        value_traits::zero(),                    d1,  value_traits::zero(),
        value_traits::zero(),  value_traits::zero(),                    d2
        );
    }

    template<typename T>
    inline Matrix3x3<T> diag(Vector3<T> const & d)  {    return diag( d(0), d(1), d(2));  }

    template<typename T>
    inline Matrix3x3<T> diag(T const & d )  {    return diag(d,d,d);  }

    template<typename T>
    inline Matrix3x3<T> inverse(Matrix3x3<T> const & A)
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      //From messer p.283 we know
      //
      //  -1     1
      // A   = -----  adj A
      //       det A
      //
      //                      i+j
      // ij-cofactor of A = -1    det A
      //                               ij
      //
      // i,j entry of the adjoint.
      //                                   i+j
      // adjoint A   = ji-cofactor = A = -1    det A
      //          ij                                ji
      //
      // As it can be seen the only numerical error
      // in these calculations is from the resolution
      // of the scalars. So it is a very accurate method.
      //
      Matrix3x3<T> adj;
      adj(0,0) = A(1,1)*A(2,2) - A(2,1)*A(1,2);
      adj(1,1) = A(0,0)*A(2,2) - A(2,0)*A(0,2);
      adj(2,2) = A(0,0)*A(1,1) - A(1,0)*A(0,1);
      adj(0,1) = A(1,0)*A(2,2) - A(2,0)*A(1,2);
      adj(0,2) = A(1,0)*A(2,1) - A(2,0)*A(1,1);
      adj(1,0) = A(0,1)*A(2,2) - A(2,1)*A(0,2);
      adj(1,2) = A(0,0)*A(2,1) - A(2,0)*A(0,1);
      adj(2,0) = A(0,1)*A(1,2) - A(1,1)*A(0,2);
      adj(2,1) = A(0,0)*A(1,2) - A(1,0)*A(0,2);
      T det = A(0,0)*adj(0,0) -  A(0,1)*adj(0,1) +   A(0,2)*adj(0,2);
      if(det)
      {
        adj(0,1) = -adj(0,1);
        adj(1,0) = -adj(1,0);
        adj(1,2) = -adj(1,2);
        adj(2,1) = -adj(2,1);
        return trans(adj)/det;
      }

      return diag(value_traits::one());
    }

    template<typename T>
    inline T max_value(Matrix3x3<T> const & A)
    {
      using std::max;

      return max(A(0,0), max( A(0,1), max( A(0,2), max ( A(1,0), max ( A(1,1), max ( A(1,2), max (A(2,0), max ( A(2,1) , A(2,2) ) ) ) ) ) ) ) );
    }

    template<typename T>
    inline T min_value(Matrix3x3<T> const & A)
    {
      using std::min;

      return min(A(0,0), min( A(0,1), min( A(0,2), min ( A(1,0), min ( A(1,1), min ( A(1,2), min (A(2,0), min ( A(2,1) , A(2,2) ) ) ) ) ) ) ) );
    }

    template<typename T>
    inline T det(Matrix3x3<T> const & A)
    {
      return A(0,0)*(A(1,1)*A(2,2) - A(2,1)*A(1,2)) -  A(0,1)*(A(1,0)*A(2,2) - A(2,0)*A(1,2)) +   A(0,2)*(A(1,0)*A(2,1) - A(2,0)*A(1,1));
    }

    template<typename T>
    inline T trace(Matrix3x3<T> const & A)
    {
      return (A(0,0) + A(1,1) + A(2,2));
    }

    template<typename T>
    inline T  norm_1(Matrix3x3<T> const & A)
    {
      using std::fabs;
      using std::max;

      T r0 = fabs(A(0,0)) + fabs(A(0,1)) +  fabs(A(0,2));
      T r1 = fabs(A(1,0)) + fabs(A(1,1)) +  fabs(A(1,2));
      T r2 = fabs(A(2,0)) + fabs(A(2,1)) +  fabs(A(2,2));

      return max ( r0, max( r1, r2 ) );
    }

    template<typename T>
    inline T norm_2(Matrix3x3<T> const & A)
    {
      using std::fabs;
      using std::max;
      using std::sqrt;

      Matrix3x3<T> V;
      typename Matrix3x3<T>::vector3_type d;
      math::eigen(A,V,d);

      T lambda = max( fabs(d(0)), max( fabs(d(1)) , fabs(d(2)) ));
      return sqrt( lambda );
    }

    template<typename T>
    inline T  norm_inf(Matrix3x3<T> const & A)
    {
      using std::fabs;
      using std::max;

      T c0 = fabs(A(0,0)) + fabs(A(1,0)) +  fabs(A(2,0));
      T c1 = fabs(A(0,1)) + fabs(A(1,1)) +  fabs(A(2,1));
      T c2 = fabs(A(0,2)) + fabs(A(1,2)) +  fabs(A(2,2));
      return max ( c0, max( c1, c2 ) );
    }

    template<typename T>
    inline Matrix3x3<T> truncate(Matrix3x3<T> const & A,T const & epsilon)
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      using std::fabs;

      assert(epsilon>value_traits::zero() || !"truncate(Matrix3x3,epsilon): epsilon must be positive");

      return Matrix3x3<T>(
        fabs(A(0,0))<epsilon?value_traits::zero():A(0,0),
        fabs(A(0,1))<epsilon?value_traits::zero():A(0,1),
        fabs(A(0,2))<epsilon?value_traits::zero():A(0,2),

        fabs(A(1,0))<epsilon?value_traits::zero():A(1,0),
        fabs(A(1,1))<epsilon?value_traits::zero():A(1,1),
        fabs(A(1,2))<epsilon?value_traits::zero():A(1,2),

        fabs(A(2,0))<epsilon?value_traits::zero():A(2,0),
        fabs(A(2,1))<epsilon?value_traits::zero():A(2,1),
        fabs(A(2,2))<epsilon?value_traits::zero():A(2,2)
        );
    }

    /**
    * Sets up the cross product matrix.
    * This method is usefull for expression the cross product as a matrix multiplication.
    *
    * @param  v       A reference to a vector. This is first argument of the cross product.
    */
    template<typename T>
    inline Matrix3x3<T> star(Vector3<T> const & v)
    {
      typedef typename Matrix3x3<T>::value_traits  value_traits;

      //--- Changes a cross-product into a matrix multiplication.
      //--- Rewrites the component of a vector3_type cross-product as a matrix.
      //--- a x b = a*b = ba*
      return Matrix3x3<T>(   
        value_traits::zero(),                   -v(2),                     v(1),  
        v(2),     value_traits::zero(),                   -v(0),
        -v(1),                     v(0),    value_traits::zero()
        );
    }

    template<typename T>
    inline std::ostream & operator<< (std::ostream & o,Matrix3x3<T> const & A)
    {
      o << "[" << A(0,0)
        << "," << A(0,1)
        << "," << A(0,2)
        << ";" << A(1,0)
        << "," << A(1,1)
        << "," << A(1,2)
        << ";" << A(2,0)
        << "," << A(2,1)
        << "," << A(2,2)
        << "]";
      return o;
    }

    template<typename T>
    inline std::istream & operator>>(std::istream & i,Matrix3x3<T> & A)
    {
      char dummy;
      i >> dummy;
      i >> A(0,0);
      i >> dummy;
      i >> A(0,1);
      i >> dummy;
      i >> A(0,2);
      i >> dummy;
      i >> A(1,0);
      i >> dummy;
      i >> A(1,1);
      i >> dummy;
      i >> A(1,2);
      i >> dummy;
      i >> A(2,0);
      i >> dummy;
      i >> A(2,1);
      i >> dummy;
      i >> A(2,2);
      i >> dummy;
      return i;
    }

  } // namespace math

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_MATH_MATRIX3X3_H
#endif
