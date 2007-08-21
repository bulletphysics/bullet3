/*
   Copyright (C) 2006, 2007 Sony Computer Entertainment Inc.
   All rights reserved.

   Redistribution and use in source and binary forms,
   with or without modification, are permitted provided that the
   following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Sony Computer Entertainment Inc nor the names
      of its contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _VECTORMATH_QUAT_AOS_V_C_H
#define _VECTORMATH_QUAT_AOS_V_C_H
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*-----------------------------------------------------------------------------
 * Definitions
 */
#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

static inline VmathQuat vmathQMakeFromElems_V( float _x, float _y, float _z, float _w )
{
    VmathQuat result;
    vmathQMakeFromElems(&result, _x, _y, _z, _w);
    return result;
}

static inline VmathQuat vmathQMakeFromV3Scalar_V( VmathVector3 xyz, float _w )
{
    VmathQuat result;
    vmathQMakeFromV3Scalar(&result, &xyz, _w);
    return result;
}

static inline VmathQuat vmathQMakeFromV4_V( VmathVector4 vec )
{
    VmathQuat result;
    vmathQMakeFromV4(&result, &vec);
    return result;
}

static inline VmathQuat vmathQMakeFromScalar_V( float scalar )
{
    VmathQuat result;
    vmathQMakeFromScalar(&result, scalar);
    return result;
}

static inline VmathQuat vmathQMakeIdentity_V( )
{
    VmathQuat result;
    vmathQMakeIdentity(&result);
    return result;
}

static inline VmathQuat vmathQLerp_V( float t, VmathQuat quat0, VmathQuat quat1 )
{
    VmathQuat result;
    vmathQLerp(&result, t, &quat0, &quat1);
    return result;
}

static inline VmathQuat vmathQSlerp_V( float t, VmathQuat unitQuat0, VmathQuat unitQuat1 )
{
    VmathQuat result;
    vmathQSlerp(&result, t, &unitQuat0, &unitQuat1);
    return result;
}

static inline VmathQuat vmathQSquad_V( float t, VmathQuat unitQuat0, VmathQuat unitQuat1, VmathQuat unitQuat2, VmathQuat unitQuat3 )
{
    VmathQuat result;
    vmathQSquad(&result, t, &unitQuat0, &unitQuat1, &unitQuat2, &unitQuat3);
    return result;
}

static inline void vmathQSetXYZ_V( VmathQuat *result, VmathVector3 vec )
{
    vmathQSetXYZ(result, &vec);
}

static inline VmathVector3 vmathQGetXYZ_V( VmathQuat quat )
{
    VmathVector3 result;
    vmathQGetXYZ(&result, &quat);
    return result;
}

static inline void vmathQSetX_V( VmathQuat *result, float _x )
{
    vmathQSetX(result, _x);
}

static inline float vmathQGetX_V( VmathQuat quat )
{
    return vmathQGetX(&quat);
}

static inline void vmathQSetY_V( VmathQuat *result, float _y )
{
    vmathQSetY(result, _y);
}

static inline float vmathQGetY_V( VmathQuat quat )
{
    return vmathQGetY(&quat);
}

static inline void vmathQSetZ_V( VmathQuat *result, float _z )
{
    vmathQSetZ(result, _z);
}

static inline float vmathQGetZ_V( VmathQuat quat )
{
    return vmathQGetZ(&quat);
}

static inline void vmathQSetW_V( VmathQuat *result, float _w )
{
    vmathQSetW(result, _w);
}

static inline float vmathQGetW_V( VmathQuat quat )
{
    return vmathQGetW(&quat);
}

static inline void vmathQSetElem_V( VmathQuat *result, int idx, float value )
{
    vmathQSetElem(result, idx, value);
}

static inline float vmathQGetElem_V( VmathQuat quat, int idx )
{
    return vmathQGetElem(&quat, idx);
}

static inline VmathQuat vmathQAdd_V( VmathQuat quat0, VmathQuat quat1 )
{
    VmathQuat result;
    vmathQAdd(&result, &quat0, &quat1);
    return result;
}

static inline VmathQuat vmathQSub_V( VmathQuat quat0, VmathQuat quat1 )
{
    VmathQuat result;
    vmathQSub(&result, &quat0, &quat1);
    return result;
}

static inline VmathQuat vmathQScalarMul_V( VmathQuat quat, float scalar )
{
    VmathQuat result;
    vmathQScalarMul(&result, &quat, scalar);
    return result;
}

static inline VmathQuat vmathQScalarDiv_V( VmathQuat quat, float scalar )
{
    VmathQuat result;
    vmathQScalarDiv(&result, &quat, scalar);
    return result;
}

static inline VmathQuat vmathQNeg_V( VmathQuat quat )
{
    VmathQuat result;
    vmathQNeg(&result, &quat);
    return result;
}

static inline float vmathQDot_V( VmathQuat quat0, VmathQuat quat1 )
{
    return vmathQDot(&quat0, &quat1);
}

static inline float vmathQNorm_V( VmathQuat quat )
{
    return vmathQNorm(&quat);
}

static inline float vmathQLength_V( VmathQuat quat )
{
    return vmathQLength(&quat);
}

static inline VmathQuat vmathQNormalize_V( VmathQuat quat )
{
    VmathQuat result;
    vmathQNormalize(&result, &quat);
    return result;
}

static inline VmathQuat vmathQMakeRotationArc_V( VmathVector3 unitVec0, VmathVector3 unitVec1 )
{
    VmathQuat result;
    vmathQMakeRotationArc(&result, &unitVec0, &unitVec1);
    return result;
}

static inline VmathQuat vmathQMakeRotationAxis_V( float radians, VmathVector3 unitVec )
{
    VmathQuat result;
    vmathQMakeRotationAxis(&result, radians, &unitVec);
    return result;
}

static inline VmathQuat vmathQMakeRotationX_V( float radians )
{
    VmathQuat result;
    vmathQMakeRotationX(&result, radians);
    return result;
}

static inline VmathQuat vmathQMakeRotationY_V( float radians )
{
    VmathQuat result;
    vmathQMakeRotationY(&result, radians);
    return result;
}

static inline VmathQuat vmathQMakeRotationZ_V( float radians )
{
    VmathQuat result;
    vmathQMakeRotationZ(&result, radians);
    return result;
}

static inline VmathQuat vmathQMul_V( VmathQuat quat0, VmathQuat quat1 )
{
    VmathQuat result;
    vmathQMul(&result, &quat0, &quat1);
    return result;
}

static inline VmathVector3 vmathQRotate_V( VmathQuat quat, VmathVector3 vec )
{
    VmathVector3 result;
    vmathQRotate(&result, &quat, &vec);
    return result;
}

static inline VmathQuat vmathQConj_V( VmathQuat quat )
{
    VmathQuat result;
    vmathQConj(&result, &quat);
    return result;
}

static inline VmathQuat vmathQSelect_V( VmathQuat quat0, VmathQuat quat1, unsigned int select1 )
{
    VmathQuat result;
    vmathQSelect(&result, &quat0, &quat1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathQPrint_V( VmathQuat quat )
{
    vmathQPrint(&quat);
}

static inline void vmathQPrints_V( VmathQuat quat, const char *name )
{
    vmathQPrints(&quat, name);
}

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
