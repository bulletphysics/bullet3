#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_FAST_BLUR_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_FAST_BLUR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace grid
  {
    // The idea is to represent the blurred image as f(x,s) in terms of position
    // x and scale s.  Gaussian blurring is accomplished by using the input image
    // I(x,s0) as the initial image (of scale s0 > 0) for the partial differential
    // equation
    //   s*df/ds = s^2*Laplacian(f)
    // where the Laplacian operator is
    //   Laplacian = (d/dx)^2, dimension 1
    //   Laplacian = (d/dx)^2+(d/dy)^2, dimension 2
    //   Laplacian = (d/dx)^2+(d/dy)^2+(d/dz)^2, dimension 3
    //
    // The term s*df/ds is approximated by
    //   s*df(x,s)/ds = (f(x,b*s)-f(x,s))/ln(b)
    // for b > 1, but close to 1, where ln(b) is the natural logarithm of b.  If
    // you take the limit of the right-hand side as b approaches 1, you get the
    // left-hand side.
    //
    // The term s^2*((d/dx)^2)f is approximated by
    //   s^2*((d/dx)^2)f = (f(x+h*s,s)-2*f(x,s)+f(x-h*s,s))/h^2
    // for h > 0, but close to zero.
    //
    // Equating the approximations for the left-hand side and the right-hand side
    // of the partial differential equation leads to the numerical method used in
    // this code.
    //
    // For iterative application of these functions, the caller is responsible
    // for constructing a geometric sequence of scales,
    //   s0, s1 = s0*b, s2 = s1*b = s0*b^2, ...
    // where the base b satisfies 1 < b < exp(0.5*d) where d is the dimension of
    // the image.  The upper bound on b guarantees stability of the finite
    // difference method used to approximate the partial differential equation.
    // The method assumes a pixel size of h = 1.
    //
    //
    // Sample usage:
    //
    // const int iterations = <number of passes to blur>;
    // double scale = 1.0, log_base = 0.125, base = exp(0.125);
    // grid_type tmp();  // Same dimensions as image!
    // for (int i = 0; i < iterations; ++i, scale *= base)
    // {
    //     fast_blur(image, tmp, scale, log_base);
    //     <use the blurred image aakImage here if desired>;
    // }

    //----------------------------------------------------------------------------

    namespace detail
    {
      template <typename grid_type>
      inline void fast_blur ( grid_type & image, grid_type & tmp, double si, double sj, double sk, double log_base)
      {
        using std::min;
        using std::ceil;
        using std::floor;

        typedef typename grid_type::value_type       value_type;
        typedef typename grid_type::iterator         iterator;
        typedef typename grid_type::index_iterator   index_iterator;
        typedef typename grid_type::math_types       math_types;
        typedef typename math_types::real_type       real_type;

        size_t I = image.I();
        size_t J = image.J();
        size_t K = image.K();

        index_iterator s      = image.begin();
        index_iterator s_end  = image.end();
        iterator       t      = tmp.begin();
        for( ; s != s_end; ++s, ++t)
        {
          size_t i = s.i();
          size_t j = s.j();
          size_t k = s.k();

          double dip = i + si;
          double dim = i - si;
          int ip = static_cast<int>( floor(dip) );
          int im = static_cast<int>( ceil(dim) );

          double djp = j + sj;
          double djm = j - sj;
          int jp = static_cast<int>( floor(djp) );
          int jm = static_cast<int>( ceil(djm) );

          double dkp = k + sk;
          double dkm = k - sk;
          int kp = static_cast<int>( floor(dkp) );
          int km = static_cast<int>( ceil(dkm) );

          im = ( i ) ?  im : 0;
          jm = ( j ) ?  jm : 0;
          km = ( k ) ?  km : 0;
          ip = min( ip, I - 1 );
          jp = min( jp, J - 1 );
          kp = min( kp, K - 1 );

          //size_t idx   = ( k  * J + j )  * I + i;
          size_t idx_im  = ( k  * J + j  ) * I + im;
          size_t idx_ip  = ( k  * J + j  ) * I + ip;
          size_t idx_jm  = ( k  * J + jm ) * I + i;
          size_t idx_jp  = ( k  * J + jp ) * I + i;
          size_t idx_km  = ( km * J + j  ) * I + i;
          size_t idx_kp  = ( kp * J + j  ) * I + i;

          real_type v000  = *s;
          real_type vm00  = image(idx_im);
          real_type vp00  = image(idx_ip);
          real_type v0m0  = image(idx_jm);
          real_type v0p0  = image(idx_jp);
          real_type v00m  = image(idx_km);
          real_type v00p  = image(idx_kp);

          // i portion of second central difference
          double isum = -2.0*v000 + vp00 + vm00;
          if ( ip < I-1 )  // not on boundary, interpolate linearly
          {
            real_type vpp00 = image( (k*J+j)*I+(ip+1) );
            isum += (dip-ip)*( vpp00 - vp00 );
          }
          if ( im > 0 )    // not on boundary, interpolate linearly
          {
            real_type vmm00 = image( (k*J+j)*I+(im-1) );
            isum += (dim-im)*( vm00 - vmm00 );
          }

          // j portion of second central difference
          double jsum = -2.0*v000 + v0p0 + v0m0;
          if ( jp < J-1 )  // not on boundary, interpolate linearly
          {
            real_type v0pp0 = image( (k*J+(jp+1))*I+i );
            jsum += (djp-jp)*( v0pp0 - v0p0 );
          }
          if ( jm > 0 )    // not on boundary, interpolate linearly
          {
            real_type v0mm0 = image( (k*J+(jm-1))*I+i );
            jsum += (djm-jm)*( v0m0 - v0mm0 );
          }

          // k portion of second central difference
          double ksum = -2.0*v000 + v00p + v00m;
          if ( kp < K-1 )  // use boundary value
          {
            real_type v00pp = image( ((kp+1)*J+j)*I+i );
            ksum += (dkp-kp)*( v00pp - v00p );
          }
          if ( km > 0 )  // use boundary value
          {
            real_type v00mm = image( ((km-1)*J+j)*I+i );
            ksum += (dkm-km)*( v00m - v00mm );
          }

          *t = static_cast<value_type>(v000 + log_base*(isum+jsum+ksum));
        }

        image = tmp;
      }

      //     for (k = 0; k < K; ++k)
      //     {
      //       double dkp = k + sk;
      //       double dkm = k - sk;
      //       int kp = static_cast<int>( floor(dkp) );
      //       int km = static_cast<int>( ceil(dkm) );
      //
      //       for (j = 0; j < J; ++j)
      //       {
      //         double djp = j + sj;
      //         double djm = j - sj;
      //         int jp = static_cast<int>( floor(djp) );
      //         int jm = static_cast<int>( ceil(djm) );
      //
      //         for (i = 0; i < I; ++i)
      //         {
      //           double dip = i + si;
      //           double dim = i - si;
      //           int ip = static_cast<int>( floor(dip) );
      //           int im = static_cast<int>( ceil(dim) );
      //
      //           // i portion of second central difference
      //           double isum = -2.0*image(i,j,k);
      //           if ( ip >= I-1 )  // use boundary value
      //           {
      //             isum += image(I-1,j,k);
      //           }
      //           else  // linearly interpolate
      //           {
      //             isum += image(ip,j,k)+(dip-ip)*(image(ip+1,j,k)-image(ip,j,k));
      //           }
      //
      //           if ( im <= 0 )  // use boundary value
      //           {
      //             isum += image(0,j,k);
      //           }
      //           else  // linearly interpolate
      //           {
      //             isum += image(im,j,k)+(dim-im)*(image(im,j,k)-image(im-1,j,k));
      //           }
      //
      //           // j portion of second central difference
      //           double jsum = -2.0*image(i,j,k);
      //           if ( jp >= J-1 )  // use boundary value
      //           {
      //             jsum += image(i,J-1,k);
      //           }
      //           else  // linearly interpolate
      //           {
      //             jsum += image(i,jp,k)+(djp-jp)*(image(i,jp+1,k)-image(i,jp,k));
      //           }
      //
      //           if ( jm <= 0 )  // use boundary value
      //           {
      //             jsum += image(i,0,k);
      //           }
      //           else  // linearly interpolate
      //           {
      //             jsum += image(i,jm,k)+(djm-jm)*(image(i,jm,k)-image(i,jm-1,k));
      //           }
      //
      //           // k portion of second central difference
      //           double ksum = -2.0*image(i,j,k);
      //           if ( kp >= K-1 )  // use boundary value
      //           {
      //             ksum += image(i,j,K-1);
      //           }
      //           else  // linearly interpolate
      //           {
      //             ksum += image(i,j,kp)+(dkp-kp)*(image(i,j,kp+1)-image(i,j,kp));
      //           }
      //
      //           if ( km <= 0 )  // use boundary value
      //           {
      //             ksum += image(i,j,0);
      //           }
      //           else  // linearly interpolate
      //           {
      //             ksum += image(i,j,km)+(dkm-km)*(image(i,j,km)-image(i,j,km-1));
      //           }
      //
      //           tmp(i,j,k) = static_cast<value_type>(image(i,j,k) + log_base*(isum+jsum+ksum));
      //         }
      //       }
      //     }

      //    for (k = 0; k < K; ++k)
      //    {
      //      for (j = 0; j < J; ++j)
      //      {
      //        for (i = 0; i < I; ++i)
      //          image(i,j,k) = tmp(i,j,k);
      //      }
      //    }

    } // namespace detail

    template <typename grid_type>
    inline void fast_blur ( grid_type & image, double sx, double sy, double sz, double log_base, size_t iterations)
    {
      using std::exp;

      double base = exp(log_base);
      grid_type tmp(image);
      for( size_t i=0; i<iterations; ++i, sx*=base, sy*=base, sz*=base)
      {
        detail::fast_blur(image, tmp, sz, sy, sz, log_base);
      }
    }

    template <typename grid_type>
    inline void fast_blur ( grid_type & image, double s, double log_base, size_t iterations)
    {
      fast_blur(image, s, s, s, log_base, iterations);
    }
  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_FAST_BLUR_H
#endif
