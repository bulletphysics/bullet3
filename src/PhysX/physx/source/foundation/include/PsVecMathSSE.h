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
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PSFOUNDATION_PSVECMATHSSE_H
#define PSFOUNDATION_PSVECMATHSSE_H

namespace physx
{
namespace shdfnd
{
namespace aos
{

namespace
{
	const PX_ALIGN(16, PxF32) minus1w[4] = { 0.0f, 0.0f, 0.0f, -1.0f };
}

PX_FORCE_INLINE void QuatGetMat33V(const QuatVArg q, Vec3V& column0, Vec3V& column1, Vec3V& column2)
{
    const __m128 q2 = V4Add(q, q);
    const __m128 qw2 = V4MulAdd(q2, V4GetW(q), _mm_load_ps(minus1w));			// (2wx, 2wy, 2wz, 2ww-1)
    const __m128 nw2 = Vec3V_From_Vec4V(V4Neg(qw2));							// (-2wx, -2wy, -2wz, 0)
    const __m128 v = Vec3V_From_Vec4V(q);

    const __m128 a0 = _mm_shuffle_ps(qw2, nw2, _MM_SHUFFLE(3, 1, 2, 3));		// (2ww-1, 2wz, -2wy, 0)
    column0 = V4MulAdd(v, V4GetX(q2), a0);

    const __m128 a1 = _mm_shuffle_ps(qw2, nw2, _MM_SHUFFLE(3, 2, 0, 3));		// (2ww-1, 2wx, -2wz, 0)
    column1 = V4MulAdd(v, V4GetY(q2), _mm_shuffle_ps(a1, a1, _MM_SHUFFLE(3, 1, 0, 2)));

    const __m128 a2 = _mm_shuffle_ps(qw2, nw2, _MM_SHUFFLE(3, 0, 1, 3));		// (2ww-1, 2wy, -2wx, 0)
    column2 = V4MulAdd(v, V4GetZ(q2), _mm_shuffle_ps(a2, a2, _MM_SHUFFLE(3, 0, 2, 1)));
}

} // namespace aos
} // namespace shdfnd
} // namespace physx


#endif // PSFOUNDATION_PSVECMATHSSE_H

