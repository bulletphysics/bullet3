#ifndef OPENTISSUE_CORE_GEOMETRY_BARYCENTRIC_H
#define OPENTISSUE_CORE_GEOMETRY_BARYCENTRIC_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>
#include <OpenTissue/core/math/math_value_traits.h>
#include <OpenTissue/core/math/math_matrix3x3.h>

#include <cmath>
#include <cassert>

namespace OpenTissue
{
  namespace geometry
  {

    /*
    * Compute Barycentric Coordinates.
    * This method computes the barycentric coodinates for a point x3 of an edge
    * given by the points x1 and x2.
    *
    * The barycentric coordinates w1 and w2 are defined such
    * that x3' = w1*x1 + w2*x2, is the point on the line closest to x3.
    *
    * if 0 <= w1,w2 <= 1 then the point lies inside or on the perimeter of the triangle.
    *
    * @warning  This method uses a geometric approach to compute the barycentric coordinates.
    *
    * @param x1    The first point of the edge.
    * @param x2    The second point of the edge.
    * @param x3    The point for which the barycentric coordinates should be computed.
    * @param w1    Upon return this parameter contains the value of the first barycentric coordinate.
    * @param w2    Upon return this parameter contains the value of the second barycentric coordinate.
    */
    template<typename V>
    inline void barycentric_geometric(
      V const & x1
      , V const & x2
      , V const & x3
      , typename V::value_type & w1
      , typename V::value_type & w2
      )
    {
      using std::sqrt;

      typedef typename V::value_type      T;
      typedef typename V::value_traits    value_traits;

      V const u  = x2-x1;
      T const uu = dot(u,u);

      assert( is_number(uu) || !"barycentric_geometric(): NaN encountered");
      assert( uu > value_traits::zero() || !"barycentric_geometric(): Degenerate edge encountered");

      // Project x3 onto edge running from x1 to x2.
      V const q  = (dot(u, x3-x1)/ uu )*u + x1;
      V const a = q - x2;
      //V const b = q-x1;
      T const aa = dot(a,a);

      assert( is_number(aa) || !"barycentric_geometric(): NaN encountered");

      //T const bb = dot(b,b);          
      w1 = sqrt( aa  / uu );
      w2 = value_traits::one() - w1; // sqrt( bb  / uu );

      assert( is_number(w1) || !"barycentric_geometric(): NaN encountered");
      assert( is_number(w2) || !"barycentric_geometric(): NaN encountered");

      assert( w1 >= value_traits::zero() || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w1 <= value_traits::one()  || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w2 >= value_traits::zero() || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w2 <= value_traits::one()  || !"barycentric_geometric(): Illegal coordinate encountered");
    }

    /*
    * Compute Barycentric Coordinates.
    * This method computes the barycentric coodinates for a point x4 of a triangle
    * given by the points x1,x2, and x3 (in counter clockwise order).
    *
    * The barycentric coordinates w1,w2, and w3 are defined such
    * that x4' = w1*x1 + w2*x2 + w3*x3, is the point in plane of the
    * triangle closest to x4.
    *
    * if 0 <= w1,w2,w3 <= 1 then the point lies inside or on the perimeter of the triangle.
    *
    * @warning  This method uses a geometric approach to compute the barycentric coordinates.
    *
    * @param x1    The first point of the triangle.
    * @param x2    The second point of the triangle.
    * @param x3    The third point of the triangle.
    * @param x4    The point for which the barycentric coordinates should be computed.
    * @param w1    Upon return this parameter contains the value of the first barycentric coordinate.
    * @param w2    Upon return this parameter contains the value of the second barycentric coordinate.
    * @param w3    Upon return this parameter contains the value of the third barycentric coordinate.
    */
    template<typename V>
    inline void barycentric_geometric(
      V const & x1
      , V const & x2
      , V const & x3
      , V const & x4
      , typename V::value_type & w1
      , typename V::value_type & w2
      , typename V::value_type & w3  
      )
    {
      using std::sqrt;

      typedef typename V::value_type      T;
      typedef typename V::value_traits    value_traits;

      V const n  = cross( x1-x3, x2-x3);
      T const nn = dot(n,n);

      assert( is_number(nn) || !"barycentric_geometric(): NaN encountered");
      assert( nn > value_traits::zero() || !"barycentric_geometric(): Degenerate triangle encountered");

      V const q  =  x4 - ( dot(n, x4 - x1)*n / nn );
      V const a  = cross( x2-q, x3-q);
      V const b  = cross( x1-q, x3-q);
      //V const c  = cross( x1-q, x2-q);

      T const aa = dot(a,a);
      T const bb = dot(b,b);
      //T const cc = dot(c,c);

      assert( is_number(aa) || !"barycentric_geometric(): NaN encountered");
      assert( is_number(bb) || !"barycentric_geometric(): NaN encountered");
      //assert( is_number(cc) || !"barycentric_geometric(): NaN encountered");

      w1 = sqrt( aa / nn );
      w2 = sqrt( bb / nn );
      w3 = value_traits::one() - w1 - w2; // sqrt( cc / nn );

      assert( is_number(w1) || !"barycentric_geometric(): NaN encountered");
      assert( is_number(w2) || !"barycentric_geometric(): NaN encountered");
      assert( is_number(w3) || !"barycentric_geometric(): NaN encountered");

      assert( w1 >= value_traits::zero() || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w1 <= value_traits::one()  || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w2 >= value_traits::zero() || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w2 <= value_traits::one()  || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w3 >= value_traits::zero() || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w3 <= value_traits::one()  || !"barycentric_geometric(): Illegal coordinate encountered");
    }

    /*
    * Compute Barycentric Coordinates.
    * This method computes the barycentric coodinates for a point p of a tetrahedron
    * given by the points x1,x2,x3, x4 (in right-hand order).
    *
    * @warning  This method uses a geometric approach to compute the barycentric coordinates.
    *
    * @param x1    The first point of the triangle.
    * @param x2    The second point of the triangle.
    * @param x3    The third point of the triangle.
    * @param x4    The fourth point of the triangle.
    * @param p     The point for which the barycentric coordinates should be computed.
    * @param w1    Upon return this parameter contains the value of the first barycentric coordinate.
    * @param w2    Upon return this parameter contains the value of the second barycentric coordinate.
    * @param w3    Upon return this parameter contains the value of the third barycentric coordinate.
    * @param w4    Upon return this parameter contains the value of the fourth barycentric coordinate.
    */
    template<typename V>
    inline void barycentric_geometric(
      V const & x1
      , V const & x2
      , V const & x3
      , V const & x4
      , V const & p
      , typename V::value_type & w1
      , typename V::value_type & w2
      , typename V::value_type & w3
      , typename V::value_type & w4
      )
    {
      using std::fabs;

      typedef typename V::value_type      T;
      typedef typename V::value_traits    value_traits;

      T const V6 = fabs( dot( (x1-x4), cross( x2-x4, x3-x4 ) ) );

      assert( is_number(V6) || !"barycentric_geometric(): NaN encountered");
      assert( V6 > value_traits::zero() || !"barycentric_geometric(): Degenerate tetrahedron encountered");

      w1 = fabs( dot( (x2-p), cross( (x3-p), (x4-p) ) ) ) / V6;
      w2 = fabs( dot( (x1-p), cross( (x3-p), (x4-p) ) ) ) / V6;
      w3 = fabs( dot( (x1-p), cross( (x2-p), (x4-p) ) ) ) / V6;
      //w4 = fabs( dot( (x1-p), cross( (x2-p), (x3-p) ) ) );
      w4 = value_traits::one() - w1 -w2 - w3;

      assert( is_number(w1) || !"barycentric_geometric(): NaN encountered");
      assert( is_number(w2) || !"barycentric_geometric(): NaN encountered");
      assert( is_number(w3) || !"barycentric_geometric(): NaN encountered");
      assert( is_number(w4) || !"barycentric_geometric(): NaN encountered");

      assert( w1 >= value_traits::zero() || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w1 <= value_traits::one()  || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w2 >= value_traits::zero() || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w2 <= value_traits::one()  || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w3 >= value_traits::zero() || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w3 <= value_traits::one()  || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w4 >= value_traits::zero() || !"barycentric_geometric(): Illegal coordinate encountered");
      assert( w4 <= value_traits::one()  || !"barycentric_geometric(): Illegal coordinate encountered");
    }

    /*
    * Compute Barycentric Coordinates.
    * This method computes the barycentric coodinates for a point p of a triangle
    * given by the points x1,x2, and x3 (in counter clockwise order).
    *
    * The barycentric coordinates w1, w2, and w3 are defined such
    * that p' = w1*x1 + w2*x2 + w3*x3, is the point in plane of the
    * triangle closest to p.
    *
    * if 0 <= w1,w2,w3 <= 1 then the point lies inside or on the perimeter of the triangle.
    *
    * @warning  This method uses a algebraic approach to compute the barycentric coordinates.
    *
    * @param x1    The first point of the triangle.
    * @param x2    The second point of the triangle.
    * @param x3    The third point of the triangle.
    * @param p     The point for which the barycentric coordinates should be computed.
    * @param w1    Upon return this parameter contains the value of the first barycentric coordinate.
    * @param w2    Upon return this parameter contains the value of the second barycentric coordinate.
    * @param w3    Upon return this parameter contains the value of the third barycentric coordinate.
    */
    template<typename V>
    inline void barycentric_algebraic(
        V const & x1
      , V const & x2
      , V const & x3
      , V const & p
      , typename V::value_type & w1
      , typename V::value_type & w2
      , typename V::value_type & w3
      )
    {
      typedef typename V::value_type     T;
      typedef math::ValueTraits<T>       value_traits;

      //--- We compute barycentric coordinates, w1,w2,w3, of p, that is we solve the linear system
      //---
      //---  |(x1-x3).(x1-x3)   (x1-x3).(x2-x3)| |w1|    |(x1-x3).(p-x3)|
      //---  |(x1-x3).(x2-x3)   (x2-x3).(x2-x3)| |w2| =  |(x2-x3).(p-x3)|
      //---   w1 + w2 + w3 = 1
      //---
      V x13 = x1-x3;
      V x23 = x2-x3;
      V x43 = p-x3;

      //---
      //---  We introduce the shorthand notation
      //---
      //---  |a11  a12|   |w1|    |b1|
      //---  |a12  a22|   |w2|  = |b2|
      //---
      //--- Isolating w2 from the second equation  yields
      //---
      //---  w2 = (b2 - a12 w1)/a22
      //---     =  b2/a22 - (a12/a22)w1
      //---
      //--- And substituting into the first gives
      //---
      //--- a11 w1 + a12 ((b2 - a12 w1)/a22)          = b1
      //--- a11 w1 + (a12/a22)*b2 - a12*(a12/a22)*w1  = b1
      //---                 (a11 - a12*(a12/a22))*w1  = b1 - (a12/a22)*b2
      //---
      //--- Letting f = a12/a22 (to minimize computations by collecting common subterms) and
      //---
      //--- m = (a11 - a12*f)
      //--- n = b1 - f*b2
      //---
      //--- we have
      //---
      //---   w1 = n/m
      //---   w2 = b2/a22 - f*w1
      //---   w3 = 1 - w1 - w2
      //---
      //--- Since we always have a11>0 and a22>0 a solution will always exist
      //--- regardless of the value of a12,b1 and b2
      //---

      T a11 = dot(x13 , x13);
      T a12 = dot(x13 , x23);
      T a22 = dot(x23 , x23);
      T b1  = dot(x13 , x43);
      T b2  = dot(x23 , x43);

      assert( is_number(a11) || !"barycentric_algebraic(): NaN encountered");
      assert( is_number(a12) || !"barycentric_algebraic(): NaN encountered");
      assert( is_number(a22) || !"barycentric_algebraic(): NaN encountered");
      assert( is_number(b1) || !"barycentric_algebraic(): NaN encountered");
      assert( is_number(b2) || !"barycentric_algebraic(): NaN encountered");

      assert( a22 < value_traits::zero() || a22>value_traits::zero() || !"barycentric_algebraic(): Degenerate triangle encountered");

      T f = a12/a22;
      T m = (a11 - a12*f);
      T n = b1-(b2*f);

      assert( is_number(f) || !"barycentric_algebraic(): NaN encountered");
      assert( is_number(m) || !"barycentric_algebraic(): NaN encountered");
      assert( is_number(n) || !"barycentric_algebraic(): NaN encountered");

      assert( m < value_traits::zero() || m > value_traits::zero() || !"barycentric_algebraic(): potential division by zero");

      w1 = n/m;
      w2 = b2/a22 - f*w1;
      w3 = value_traits::one() - w1 - w2;

      assert( is_number(w1) || !"barycentric_algebraic(): NaN encountered");
      assert( is_number(w2) || !"barycentric_algebraic(): NaN encountered");
      assert( is_number(w3) || !"barycentric_algebraic(): NaN encountered");
    }

    /*
    * Compute Barycentric Coordinates.
    * This method computes the barycentric coodinates for a point p of a tetrahedron
    * given by the points x1,x2,x3, p (in right-hand order).
    *
    * @warning  This method uses a algebraic approach to compute the barycentric coordinates.
    *
    * @param x1    The first point of the triangle.
    * @param x2    The second point of the triangle.
    * @param x3    The third point of the triangle.
    * @param x4    The fourth point of the triangle.
    * @param p     The point for which the barycentric coordinates should be computed.
    * @param w1    Upon return this parameter contains the value of the first barycentric coordinate.
    * @param w2    Upon return this parameter contains the value of the second barycentric coordinate.
    * @param w3    Upon return this parameter contains the value of the third barycentric coordinate.
    * @param w4    Upon return this parameter contains the value of the fourth barycentric coordinate.
    */
    template<typename V>
    inline void barycentric_algebraic(
        V const & x1
      , V const & x2
      , V const & x3
      , V const & x4
      , V const & p
      , typename V::value_type & w1
      , typename V::value_type & w2
      , typename V::value_type & w3
      , typename V::value_type & w4
      )
    {
      typedef typename V::value_type       T;
      typedef math::ValueTraits<T>         value_traits;
      typedef math::Matrix3x3<T>           M;   // 2008-08-01 kenny: This is rather bad, it makes this function hardwired to our own matrix3x3 type!

      M P;

      P(0,0) = x1(0) - x4(0);
      P(1,0) = x1(1) - x4(1);
      P(2,0) = x1(2) - x4(2);

      P(0,1) = x2(0) - x4(0);
      P(1,1) = x2(1) - x4(1);
      P(2,1) = x2(2) - x4(2);

      P(0,2) = x3(0) - x4(0);
      P(1,2) = x3(1) - x4(1);
      P(2,2) = x3(2) - x4(2);

      P = math::inverse(P);  // 2008-08-01 kenny: This is very specific for our OT library, it should probably be a policy?

      V w = P * V(p-x4);

      w1 = w(0);
      w2 = w(1);
      w3 = w(2);
      w4 = value_traits::one() - w1 - w2 - w3;

      assert( is_number(w1) || !"barycentric_algebraic(): NaN encountered");
      assert( is_number(w2) || !"barycentric_algebraic(): NaN encountered");
      assert( is_number(w3) || !"barycentric_algebraic(): NaN encountered");
      assert( is_number(w4) || !"barycentric_algebraic(): NaN encountered");

    }

  } // namespace geometry
} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_BARYCENTRIC_H
#endif
