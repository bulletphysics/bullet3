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

#include "GuIntersectionBoxBox.h"

using namespace physx;

bool Gu::intersectOBBOBB(const PxVec3& e0, const PxVec3& c0, const PxMat33& r0, const PxVec3& e1, const PxVec3& c1, const PxMat33& r1, bool full_test)
{
	// Translation, in parent frame
	const PxVec3 v = c1 - c0;
	// Translation, in A's frame
	const PxVec3 T(v.dot(r0[0]), v.dot(r0[1]), v.dot(r0[2]));

	// B's basis with respect to A's local frame
	PxReal R[3][3];
	PxReal FR[3][3];
	PxReal ra, rb, t;

	// Calculate rotation matrix
	for(PxU32 i=0;i<3;i++)
	{
		for(PxU32 k=0;k<3;k++)
		{
			R[i][k] = r0[i].dot(r1[k]);
			FR[i][k] = 1e-6f + PxAbs(R[i][k]);	// Precompute fabs matrix
		}
	}

	// A's basis vectors
	for(PxU32 i=0;i<3;i++)
	{
		ra = e0[i];

		rb = e1[0]*FR[i][0] + e1[1]*FR[i][1] + e1[2]*FR[i][2];

		t = PxAbs(T[i]);

		if(t > ra + rb)		return false;
	}

	// B's basis vectors
	for(PxU32 k=0;k<3;k++)
	{
		ra = e0[0]*FR[0][k] + e0[1]*FR[1][k] + e0[2]*FR[2][k];

		rb = e1[k];

		t = PxAbs(T[0]*R[0][k] + T[1]*R[1][k] + T[2]*R[2][k]);

		if( t > ra + rb )	return false;
	}

	if(full_test)
	{
		//9 cross products

		//L = A0 x B0
		ra	= e0[1]*FR[2][0] + e0[2]*FR[1][0];
		rb	= e1[1]*FR[0][2] + e1[2]*FR[0][1];
		t	= PxAbs(T[2]*R[1][0] - T[1]*R[2][0]);
		if(t > ra + rb)	return false;

		//L = A0 x B1
		ra	= e0[1]*FR[2][1] + e0[2]*FR[1][1];
		rb	= e1[0]*FR[0][2] + e1[2]*FR[0][0];
		t	= PxAbs(T[2]*R[1][1] - T[1]*R[2][1]);
		if(t > ra + rb)	return false;

		//L = A0 x B2
		ra	= e0[1]*FR[2][2] + e0[2]*FR[1][2];
		rb	= e1[0]*FR[0][1] + e1[1]*FR[0][0];
		t	= PxAbs(T[2]*R[1][2] - T[1]*R[2][2]);
		if(t > ra + rb)	return false;

		//L = A1 x B0
		ra	= e0[0]*FR[2][0] + e0[2]*FR[0][0];
		rb	= e1[1]*FR[1][2] + e1[2]*FR[1][1];
		t	= PxAbs(T[0]*R[2][0] - T[2]*R[0][0]);
		if(t > ra + rb)	return false;

		//L = A1 x B1
		ra	= e0[0]*FR[2][1] + e0[2]*FR[0][1];
		rb	= e1[0]*FR[1][2] + e1[2]*FR[1][0];
		t	= PxAbs(T[0]*R[2][1] - T[2]*R[0][1]);
		if(t > ra + rb)	return false;

		//L = A1 x B2
		ra	= e0[0]*FR[2][2] + e0[2]*FR[0][2];
		rb	= e1[0]*FR[1][1] + e1[1]*FR[1][0];
		t	= PxAbs(T[0]*R[2][2] - T[2]*R[0][2]);
		if(t > ra + rb)	return false;

		//L = A2 x B0
		ra	= e0[0]*FR[1][0] + e0[1]*FR[0][0];
		rb	= e1[1]*FR[2][2] + e1[2]*FR[2][1];
		t	= PxAbs(T[1]*R[0][0] - T[0]*R[1][0]);
		if(t > ra + rb)	return false;

		//L = A2 x B1
		ra	= e0[0]*FR[1][1] + e0[1]*FR[0][1];
		rb	= e1[0] *FR[2][2] + e1[2]*FR[2][0];
		t	= PxAbs(T[1]*R[0][1] - T[0]*R[1][1]);
		if(t > ra + rb)	return false;

		//L = A2 x B2
		ra	= e0[0]*FR[1][2] + e0[1]*FR[0][2];
		rb	= e1[0]*FR[2][1] + e1[1]*FR[2][0];
		t	= PxAbs(T[1]*R[0][2] - T[0]*R[1][2]);
		if(t > ra + rb)	return false;
	}
	return true;
}
