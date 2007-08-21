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

#ifndef _VECTORMATH_QUAT_SOA_V_C_H
#define _VECTORMATH_QUAT_SOA_V_C_H
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*-----------------------------------------------------------------------------
 * Definitions
 */
#ifndef _VECTORMATH_INTERNAL_FUNCTIONS
#define _VECTORMATH_INTERNAL_FUNCTIONS

#endif

static inline VmathSoaQuat vmathSoaQMakeFromElems_V( vec_float4 _x, vec_float4 _y, vec_float4 _z, vec_float4 _w )
{
    VmathSoaQuat result;
    vmathSoaQMakeFromElems(&result, _x, _y, _z, _w);
    return result;
}

static inline VmathSoaQuat vmathSoaQMakeFromV3Scalar_V( VmathSoaVector3 xyz, vec_float4 _w )
{
    VmathSoaQuat result;
    vmathSoaQMakeFromV3Scalar(&result, &xyz, _w);
    return result;
}

static inline VmathSoaQuat vmathSoaQMakeFromV4_V( VmathSoaVector4 vec )
{
    VmathSoaQuat result;
    vmathSoaQMakeFromV4(&result, &vec);
    return result;
}

static inline VmathSoaQuat vmathSoaQMakeFromScalar_V( vec_float4 scalar )
{
    VmathSoaQuat result;
    vmathSoaQMakeFromScalar(&result, scalar);
    return result;
}

static inline VmathSoaQuat vmathSoaQMakeFromAos_V( VmathQuat quat )
{
    VmathSoaQuat result;
    vmathSoaQMakeFromAos(&result, &quat);
    return result;
}

static inline VmathSoaQuat vmathSoaQMakeFrom4Aos_V( VmathQuat quat0, VmathQuat quat1, VmathQuat quat2, VmathQuat quat3 )
{
    VmathSoaQuat result;
    vmathSoaQMakeFrom4Aos(&result, &quat0, &quat1, &quat2, &quat3);
    return result;
}

static inline VmathSoaQuat vmathSoaQMakeIdentity_V( )
{
    VmathSoaQuat result;
    vmathSoaQMakeIdentity(&result);
    return result;
}

static inline VmathSoaQuat vmathSoaQLerp_V( vec_float4 t, VmathSoaQuat quat0, VmathSoaQuat quat1 )
{
    VmathSoaQuat result;
    vmathSoaQLerp(&result, t, &quat0, &quat1);
    return result;
}

static inline VmathSoaQuat vmathSoaQSlerp_V( vec_float4 t, VmathSoaQuat unitQuat0, VmathSoaQuat unitQuat1 )
{
    VmathSoaQuat result;
    vmathSoaQSlerp(&result, t, &unitQuat0, &unitQuat1);
    return result;
}

static inline VmathSoaQuat vmathSoaQSquad_V( vec_float4 t, VmathSoaQuat unitQuat0, VmathSoaQuat unitQuat1, VmathSoaQuat unitQuat2, VmathSoaQuat unitQuat3 )
{
    VmathSoaQuat result;
    vmathSoaQSquad(&result, t, &unitQuat0, &unitQuat1, &unitQuat2, &unitQuat3);
    return result;
}

static inline void vmathSoaQGet4Aos_V( VmathSoaQuat quat, VmathQuat *result0, VmathQuat *result1, VmathQuat *result2, VmathQuat *result3 )
{
    vmathSoaQGet4Aos(&quat, result0, result1, result2, result3);
}

static inline void vmathSoaQSetXYZ_V( VmathSoaQuat *result, VmathSoaVector3 vec )
{
    vmathSoaQSetXYZ(result, &vec);
}

static inline VmathSoaVector3 vmathSoaQGetXYZ_V( VmathSoaQuat quat )
{
    VmathSoaVector3 result;
    vmathSoaQGetXYZ(&result, &quat);
    return result;
}

static inline void vmathSoaQSetX_V( VmathSoaQuat *result, vec_float4 _x )
{
    vmathSoaQSetX(result, _x);
}

static inline vec_float4 vmathSoaQGetX_V( VmathSoaQuat quat )
{
    return vmathSoaQGetX(&quat);
}

static inline void vmathSoaQSetY_V( VmathSoaQuat *result, vec_float4 _y )
{
    vmathSoaQSetY(result, _y);
}

static inline vec_float4 vmathSoaQGetY_V( VmathSoaQuat quat )
{
    return vmathSoaQGetY(&quat);
}

static inline void vmathSoaQSetZ_V( VmathSoaQuat *result, vec_float4 _z )
{
    vmathSoaQSetZ(result, _z);
}

static inline vec_float4 vmathSoaQGetZ_V( VmathSoaQuat quat )
{
    return vmathSoaQGetZ(&quat);
}

static inline void vmathSoaQSetW_V( VmathSoaQuat *result, vec_float4 _w )
{
    vmathSoaQSetW(result, _w);
}

static inline vec_float4 vmathSoaQGetW_V( VmathSoaQuat quat )
{
    return vmathSoaQGetW(&quat);
}

static inline void vmathSoaQSetElem_V( VmathSoaQuat *result, int idx, vec_float4 value )
{
    vmathSoaQSetElem(result, idx, value);
}

static inline vec_float4 vmathSoaQGetElem_V( VmathSoaQuat quat, int idx )
{
    return vmathSoaQGetElem(&quat, idx);
}

static inline VmathSoaQuat vmathSoaQAdd_V( VmathSoaQuat quat0, VmathSoaQuat quat1 )
{
    VmathSoaQuat result;
    vmathSoaQAdd(&result, &quat0, &quat1);
    return result;
}

static inline VmathSoaQuat vmathSoaQSub_V( VmathSoaQuat quat0, VmathSoaQuat quat1 )
{
    VmathSoaQuat result;
    vmathSoaQSub(&result, &quat0, &quat1);
    return result;
}

static inline VmathSoaQuat vmathSoaQScalarMul_V( VmathSoaQuat quat, vec_float4 scalar )
{
    VmathSoaQuat result;
    vmathSoaQScalarMul(&result, &quat, scalar);
    return result;
}

static inline VmathSoaQuat vmathSoaQScalarDiv_V( VmathSoaQuat quat, vec_float4 scalar )
{
    VmathSoaQuat result;
    vmathSoaQScalarDiv(&result, &quat, scalar);
    return result;
}

static inline VmathSoaQuat vmathSoaQNeg_V( VmathSoaQuat quat )
{
    VmathSoaQuat result;
    vmathSoaQNeg(&result, &quat);
    return result;
}

static inline vec_float4 vmathSoaQDot_V( VmathSoaQuat quat0, VmathSoaQuat quat1 )
{
    return vmathSoaQDot(&quat0, &quat1);
}

static inline vec_float4 vmathSoaQNorm_V( VmathSoaQuat quat )
{
    return vmathSoaQNorm(&quat);
}

static inline vec_float4 vmathSoaQLength_V( VmathSoaQuat quat )
{
    return vmathSoaQLength(&quat);
}

static inline VmathSoaQuat vmathSoaQNormalize_V( VmathSoaQuat quat )
{
    VmathSoaQuat result;
    vmathSoaQNormalize(&result, &quat);
    return result;
}

static inline VmathSoaQuat vmathSoaQMakeRotationArc_V( VmathSoaVector3 unitVec0, VmathSoaVector3 unitVec1 )
{
    VmathSoaQuat result;
    vmathSoaQMakeRotationArc(&result, &unitVec0, &unitVec1);
    return result;
}

static inline VmathSoaQuat vmathSoaQMakeRotationAxis_V( vec_float4 radians, VmathSoaVector3 unitVec )
{
    VmathSoaQuat result;
    vmathSoaQMakeRotationAxis(&result, radians, &unitVec);
    return result;
}

static inline VmathSoaQuat vmathSoaQMakeRotationX_V( vec_float4 radians )
{
    VmathSoaQuat result;
    vmathSoaQMakeRotationX(&result, radians);
    return result;
}

static inline VmathSoaQuat vmathSoaQMakeRotationY_V( vec_float4 radians )
{
    VmathSoaQuat result;
    vmathSoaQMakeRotationY(&result, radians);
    return result;
}

static inline VmathSoaQuat vmathSoaQMakeRotationZ_V( vec_float4 radians )
{
    VmathSoaQuat result;
    vmathSoaQMakeRotationZ(&result, radians);
    return result;
}

static inline VmathSoaQuat vmathSoaQMul_V( VmathSoaQuat quat0, VmathSoaQuat quat1 )
{
    VmathSoaQuat result;
    vmathSoaQMul(&result, &quat0, &quat1);
    return result;
}

static inline VmathSoaVector3 vmathSoaQRotate_V( VmathSoaQuat quat, VmathSoaVector3 vec )
{
    VmathSoaVector3 result;
    vmathSoaQRotate(&result, &quat, &vec);
    return result;
}

static inline VmathSoaQuat vmathSoaQConj_V( VmathSoaQuat quat )
{
    VmathSoaQuat result;
    vmathSoaQConj(&result, &quat);
    return result;
}

static inline VmathSoaQuat vmathSoaQSelect_V( VmathSoaQuat quat0, VmathSoaQuat quat1, vec_uint4 select1 )
{
    VmathSoaQuat result;
    vmathSoaQSelect(&result, &quat0, &quat1, select1);
    return result;
}

#ifdef _VECTORMATH_DEBUG

static inline void vmathSoaQPrint_V( VmathSoaQuat quat )
{
    vmathSoaQPrint(&quat);
}

static inline void vmathSoaQPrints_V( VmathSoaQuat quat, const char *name )
{
    vmathSoaQPrints(&quat, name);
}

#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
