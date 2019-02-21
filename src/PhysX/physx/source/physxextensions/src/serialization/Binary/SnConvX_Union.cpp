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

#include "SnConvX.h"
#include <assert.h>

using namespace physx;

void Sn::ConvX::resetUnions()
{
	mUnions.clear();
}

bool Sn::ConvX::registerUnion(const char* name)
{
	displayMessage(PxErrorCode::eDEBUG_INFO, "Registering union: %s\n", name);

	Sn::Union u;
	u.mName	= name;

	mUnions.pushBack(u);
	return true;
}

bool Sn::ConvX::registerUnionType(const char* unionName, const char* typeName, int typeValue)
{
	const PxU32 nb = mUnions.size();
	for(PxU32 i=0;i<nb;i++)
	{
		if(strcmp(mUnions[i].mName, unionName)==0)
		{
			UnionType t;
			t.mTypeName		= typeName;
			t.mTypeValue	= typeValue;
			mUnions[i].mTypes.pushBack(t);			
			displayMessage(PxErrorCode::eDEBUG_INFO, "Registering union type: %s | %s | %d\n", unionName, typeName, typeValue);
			return true;
		}
	}

	displayMessage(PxErrorCode::eINTERNAL_ERROR, "PxBinaryConverter: union not found: %s, please check the source metadata.\n", unionName);
	return false;
}

const char* Sn::ConvX::getTypeName(const char* unionName, int typeValue)
{
	const PxU32 nb = mUnions.size();
	for(PxU32 i=0;i<nb;i++)
	{
		if(strcmp(mUnions[i].mName, unionName)==0)
		{
			const PxU32 nbTypes = mUnions[i].mTypes.size();
			for(PxU32 j=0;j<nbTypes;j++)
			{
				const UnionType& t = mUnions[i].mTypes[j];
				if(t.mTypeValue==typeValue)
					return t.mTypeName;
			}
			break;
		}
	}
	displayMessage(PxErrorCode::eINTERNAL_ERROR,
		"PxBinaryConverter: union type not found: %s, type %d, please check the source metadata.\n", unionName, typeValue);		
	assert(0);
	return NULL;
}
