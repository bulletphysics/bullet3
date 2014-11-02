#ifndef OPENTISSUE_CORE_MATH_MATH_INVERT4x4_H
#define OPENTISSUE_CORE_MATH_MATH_INVERT4x4_H
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

  namespace math
  {

    template<typename real_type>
    inline void invert4x4(real_type M[4][4])
    {
      real_type a00 = M[0][0]; real_type a01 = M[0][1]; real_type a02 = M[0][2]; real_type a03 = M[0][3];
      real_type a10 = M[1][0]; real_type a11 = M[1][1]; real_type a12 = M[1][2]; real_type a13 = M[1][3];
      real_type a20 = M[2][0]; real_type a21 = M[2][1]; real_type a22 = M[2][2]; real_type a23 = M[2][3];
      real_type a30 = M[3][0]; real_type a31 = M[3][1]; real_type a32 = M[3][2]; real_type a33 = M[3][3];

      M[0][0] =      a11*a22*a33 - a11*a23*a32 - a21*a12*a33 + a21*a13*a32 + a31*a12*a23 - a31*a13*a22;
      M[0][1] =    - a01*a22*a33 + a01*a23*a32 + a21*a02*a33 - a21*a03*a32 - a31*a02*a23 + a31*a03*a22;
      M[0][2] =      a01*a12*a33 - a01*a13*a32 - a11*a02*a33 + a11*a03*a32 + a31*a02*a13 - a31*a03*a12;
      M[0][3] =    - a01*a12*a23 + a01*a13*a22 + a11*a02*a23 - a11*a03*a22 - a21*a02*a13 + a21*a03*a12;
      M[1][0] =    - a10*a22*a33 + a10*a23*a32 + a20*a12*a33 - a20*a13*a32 - a30*a12*a23 + a30*a13*a22;
      M[1][1] =      a00*a22*a33 - a00*a23*a32 - a20*a02*a33 + a20*a03*a32 + a30*a02*a23 - a30*a03*a22;
      M[1][2] =    - a00*a12*a33 + a00*a13*a32 + a10*a02*a33 - a10*a03*a32 - a30*a02*a13 + a30*a03*a12;
      M[1][3] =      a00*a12*a23 - a00*a13*a22 - a10*a02*a23 + a10*a03*a22 + a20*a02*a13 - a20*a03*a12;
      M[2][0] =      a10*a21*a33 - a10*a23*a31 - a20*a11*a33 + a20*a13*a31 + a30*a11*a23 - a30*a13*a21;
      M[2][1] =    - a00*a21*a33 + a00*a23*a31 + a20*a01*a33 - a20*a03*a31 - a30*a01*a23 + a30*a03*a21;
      M[2][2] =      a00*a11*a33 - a00*a13*a31 - a10*a01*a33 + a10*a03*a31 + a30*a01*a13 - a30*a03*a11;
      M[2][3] =    - a00*a11*a23 + a00*a13*a21 + a10*a01*a23 - a10*a03*a21 - a20*a01*a13 + a20*a03*a11;
      M[3][0] =    - a10*a21*a32 + a10*a22*a31 + a20*a11*a32 - a20*a12*a31 - a30*a11*a22 + a30*a12*a21;
      M[3][1] =      a00*a21*a32 - a00*a22*a31 - a20*a01*a32 + a20*a02*a31 + a30*a01*a22 - a30*a02*a21;
      M[3][2] =    - a00*a11*a32 + a00*a12*a31 + a10*a01*a32 - a10*a02*a31 - a30*a01*a12 + a30*a02*a11;
      M[3][3] =      a00*a11*a22 - a00*a12*a21 - a10*a01*a22 + a10*a02*a21 + a20*a01*a12 - a20*a02*a11;
      //real_type D =
      //  (
      //       a00*a11*a22*a33 - a00*a11*a23*a32 - a00*a21*a12*a33 + a00*a21*a13*a32 + a00*a31*a12*a23 - a00*a31*a13*a22
      //     - a10*a01*a22*a33 + a10*a01*a23*a32 + a10*a21*a02*a33 - a10*a21*a03*a32 - a10*a31*a02*a23 + a10*a31*a03*a22
      //     + a20*a01*a12*a33 - a20*a01*a13*a32 - a20*a11*a02*a33 + a20*a11*a03*a32 + a20*a31*a02*a13 - a20*a31*a03*a12
      //     - a30*a01*a12*a23 + a30*a01*a13*a22 + a30*a11*a02*a23 - a30*a11*a03*a22 - a30*a21*a02*a13 + a30*a21*a03*a12
      //  );
      real_type D = a00*M[0][0] + a10*M[0][1] +  a20*M[0][2] + a30*M[0][3];
      if(D)
      {
        M[0][0] /=D; M[0][1] /=D; M[0][2] /=D; M[0][3] /=D;
        M[1][0] /=D; M[1][1] /=D; M[1][2] /=D; M[1][3] /=D;
        M[2][0] /=D; M[2][1] /=D; M[2][2] /=D; M[2][3] /=D;
        M[3][0] /=D; M[3][1] /=D; M[3][2] /=D; M[3][3] /=D;
      }
    }

  } // namespace math

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_MATH_INVERT4x4_H
#endif
