//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PSFOUNDATION_PSWINDOWSTRIGCONSTANTS_H
#define PSFOUNDATION_PSWINDOWSTRIGCONSTANTS_H

namespace physx
{
namespace shdfnd
{
namespace aos
{

#define PX_GLOBALCONST extern const __declspec(selectany)

__declspec(align(16)) struct PX_VECTORF32
{
	float f[4];
};

//#define PX_PI               3.141592654f
//#define PX_2PI              6.283185307f
//#define PX_1DIVPI           0.318309886f
//#define PX_1DIV2PI          0.159154943f
//#define PX_PIDIV2           1.570796327f
//#define PX_PIDIV4           0.785398163f

PX_GLOBALCONST PX_VECTORF32 g_PXSinCoefficients0 = { { 1.0f, -0.166666667f, 8.333333333e-3f, -1.984126984e-4f } };
PX_GLOBALCONST PX_VECTORF32
g_PXSinCoefficients1 = { { 2.755731922e-6f, -2.505210839e-8f, 1.605904384e-10f, -7.647163732e-13f } };
PX_GLOBALCONST PX_VECTORF32
g_PXSinCoefficients2 = { { 2.811457254e-15f, -8.220635247e-18f, 1.957294106e-20f, -3.868170171e-23f } };
PX_GLOBALCONST PX_VECTORF32 g_PXCosCoefficients0 = { { 1.0f, -0.5f, 4.166666667e-2f, -1.388888889e-3f } };
PX_GLOBALCONST PX_VECTORF32
g_PXCosCoefficients1 = { { 2.480158730e-5f, -2.755731922e-7f, 2.087675699e-9f, -1.147074560e-11f } };
PX_GLOBALCONST PX_VECTORF32
g_PXCosCoefficients2 = { { 4.779477332e-14f, -1.561920697e-16f, 4.110317623e-19f, -8.896791392e-22f } };
PX_GLOBALCONST PX_VECTORF32 g_PXTanCoefficients0 = { { 1.0f, 0.333333333f, 0.133333333f, 5.396825397e-2f } };
PX_GLOBALCONST PX_VECTORF32
g_PXTanCoefficients1 = { { 2.186948854e-2f, 8.863235530e-3f, 3.592128167e-3f, 1.455834485e-3f } };
PX_GLOBALCONST PX_VECTORF32
g_PXTanCoefficients2 = { { 5.900274264e-4f, 2.391290764e-4f, 9.691537707e-5f, 3.927832950e-5f } };
PX_GLOBALCONST PX_VECTORF32
g_PXASinCoefficients0 = { { -0.05806367563904f, -0.41861972469416f, 0.22480114791621f, 2.17337241360606f } };
PX_GLOBALCONST PX_VECTORF32
g_PXASinCoefficients1 = { { 0.61657275907170f, 4.29696498283455f, -1.18942822255452f, -6.53784832094831f } };
PX_GLOBALCONST PX_VECTORF32
g_PXASinCoefficients2 = { { -1.36926553863413f, -4.48179294237210f, 1.41810672941833f, 5.48179257935713f } };
PX_GLOBALCONST PX_VECTORF32 g_PXATanCoefficients0 = { { 1.0f, 0.333333334f, 0.2f, 0.142857143f } };
PX_GLOBALCONST PX_VECTORF32
g_PXATanCoefficients1 = { { 1.111111111e-1f, 9.090909091e-2f, 7.692307692e-2f, 6.666666667e-2f } };
PX_GLOBALCONST PX_VECTORF32
g_PXATanCoefficients2 = { { 5.882352941e-2f, 5.263157895e-2f, 4.761904762e-2f, 4.347826087e-2f } };
PX_GLOBALCONST PX_VECTORF32
g_PXSinEstCoefficients = { { 1.0f, -1.66521856991541e-1f, 8.199913018755e-3f, -1.61475937228e-4f } };
PX_GLOBALCONST PX_VECTORF32
g_PXCosEstCoefficients = { { 1.0f, -4.95348008918096e-1f, 3.878259962881e-2f, -9.24587976263e-4f } };
PX_GLOBALCONST PX_VECTORF32 g_PXTanEstCoefficients = { { 2.484f, -1.954923183e-1f, 2.467401101f, PxInvPi } };
PX_GLOBALCONST PX_VECTORF32
g_PXATanEstCoefficients = { { 7.689891418951e-1f, 1.104742493348f, 8.661844266006e-1f, PxPiDivTwo } };
PX_GLOBALCONST PX_VECTORF32
g_PXASinEstCoefficients = { { -1.36178272886711f, 2.37949493464538f, -8.08228565650486e-1f, 2.78440142746736e-1f } };
PX_GLOBALCONST PX_VECTORF32 g_PXASinEstConstants = { { 1.00000011921f, PxPiDivTwo, 0.0f, 0.0f } };
PX_GLOBALCONST PX_VECTORF32 g_PXPiConstants0 = { { PxPi, PxTwoPi, PxInvPi, PxInvTwoPi } };
PX_GLOBALCONST PX_VECTORF32 g_PXReciprocalTwoPi = { { PxInvTwoPi, PxInvTwoPi, PxInvTwoPi, PxInvTwoPi } };
PX_GLOBALCONST PX_VECTORF32 g_PXTwoPi = { { PxTwoPi, PxTwoPi, PxTwoPi, PxTwoPi } };

} // namespace aos
} // namespace shdfnd
} // namespace physx

#endif
