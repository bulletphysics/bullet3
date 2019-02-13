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

#include "foundation/PxMat33.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "PsUtilities.h"
#include "PsUserAllocated.h"
#include "PsFPU.h"

namespace physx
{
namespace shdfnd
{

bool checkValid(const float& f)
{
	return PxIsFinite(f);
}
bool checkValid(const PxVec3& v)
{
	return PxIsFinite(v.x) && PxIsFinite(v.y) && PxIsFinite(v.z);
}

bool checkValid(const PxTransform& t)
{
	return checkValid(t.p) && checkValid(t.q);
}

bool checkValid(const PxQuat& q)
{
	return PxIsFinite(q.x) && PxIsFinite(q.y) && PxIsFinite(q.z) && PxIsFinite(q.w);
}
bool checkValid(const PxMat33& m)
{
	return PxIsFinite(m(0, 0)) && PxIsFinite(m(1, 0)) && PxIsFinite(m(2, 0)) && PxIsFinite(m(0, 1)) &&
	       PxIsFinite(m(1, 1)) && PxIsFinite(m(2, 1)) && PxIsFinite(m(0, 3)) && PxIsFinite(m(1, 3)) &&
	       PxIsFinite(m(2, 3));
}
bool checkValid(const char* string)
{
	static const PxU32 maxLength = 4096;
	return strnlen(string, maxLength) != maxLength;
}

} // namespace shdfnd
} // namespace physx
