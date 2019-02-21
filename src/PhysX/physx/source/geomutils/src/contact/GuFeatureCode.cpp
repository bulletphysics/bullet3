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

#include "GuConvexEdgeFlags.h"
#include "GuFeatureCode.h"

using namespace physx;
using namespace Gu;

static FeatureCode computeFeatureCode(PxReal u, PxReal v)
{
	// Analysis
	if(u==0.0f)
	{
		if(v==0.0f)
		{
			// Vertex 0
			return FC_VERTEX0;
		}
		else if(v==1.0f)
		{
			// Vertex 2
			return FC_VERTEX2;
		}
		else
		{
			// Edge 0-2
			return FC_EDGE20;
		}
	}
	else if(u==1.0f)
	{
		if(v==0.0f)
		{
			// Vertex 1
			return FC_VERTEX1;
		}
	}
	else
	{
		if(v==0.0f)
		{
			// Edge 0-1
			return FC_EDGE01;
		}
		else
		{
			if((u+v)>=0.9999f)
			{
				// Edge 1-2
				return FC_EDGE12;
			}
			else
			{
				// Face
				return FC_FACE;
			}
		}
	}
	return FC_UNDEFINED;
}


bool Gu::selectNormal(PxU8 data, PxReal u, PxReal v)
{
	bool useFaceNormal = false;
	const FeatureCode FC = computeFeatureCode(u, v);
	switch(FC)
	{
		case FC_VERTEX0:
			if(!(data & (Gu::ETD_CONVEX_EDGE_01|Gu::ETD_CONVEX_EDGE_20)))
				useFaceNormal = true;
			break;
		case FC_VERTEX1:
			if(!(data & (Gu::ETD_CONVEX_EDGE_01|Gu::ETD_CONVEX_EDGE_12)))
				useFaceNormal = true;
			break;
		case FC_VERTEX2:
			if(!(data & (Gu::ETD_CONVEX_EDGE_12|Gu::ETD_CONVEX_EDGE_20)))
				useFaceNormal = true;
			break;
		case FC_EDGE01:
			if(!(data & Gu::ETD_CONVEX_EDGE_01))
				useFaceNormal = true;
			break;
		case FC_EDGE12:
			if(!(data & Gu::ETD_CONVEX_EDGE_12))
				useFaceNormal = true;
			break;
		case FC_EDGE20:
			if(!(data & Gu::ETD_CONVEX_EDGE_20))
				useFaceNormal = true;
			break;
		case FC_FACE:
			useFaceNormal = true;
			break;
		case FC_UNDEFINED:
			break;
	};
	return useFaceNormal;
}

