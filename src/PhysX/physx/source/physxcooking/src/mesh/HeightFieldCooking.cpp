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

#include "foundation/PxIO.h"
#include "GuHeightField.h"
#include "GuSerialize.h"

using namespace physx;
using namespace Gu;

namespace physx
{

bool saveHeightField(const HeightField& hf, PxOutputStream& stream, bool endian)
{
	// write header
	if(!writeHeader('H', 'F', 'H', 'F', PX_HEIGHTFIELD_VERSION, endian, stream))
		return false;

	const Gu::HeightFieldData& hfData = hf.getData();

	// write mData members
	writeDword(hfData.rows, endian, stream);
	writeDword(hfData.columns, endian, stream);
	writeFloat(hfData.rowLimit, endian, stream);
	writeFloat(hfData.colLimit, endian, stream);
	writeFloat(hfData.nbColumns, endian, stream);
	writeFloat(0.0f, endian, stream);	// thickness
	writeFloat(hfData.convexEdgeThreshold, endian, stream);
	writeWord(hfData.flags, endian, stream);
	writeDword(hfData.format, endian, stream);

	writeFloat(hfData.mAABB.getMin(0), endian, stream);
	writeFloat(hfData.mAABB.getMin(1), endian, stream);
	writeFloat(hfData.mAABB.getMin(2), endian, stream);
	writeFloat(hfData.mAABB.getMax(0), endian, stream);
	writeFloat(hfData.mAABB.getMax(1), endian, stream);
	writeFloat(hfData.mAABB.getMax(2), endian, stream);

	// write this-> members
	writeDword(hf.mSampleStride, endian, stream);
	writeDword(hf.mNbSamples, endian, stream);
	writeFloat(hf.mMinHeight, endian, stream);
	writeFloat(hf.mMaxHeight, endian, stream);

	// write samples
	for(PxU32 i = 0; i < hf.mNbSamples; i++)
	{
		const PxHeightFieldSample& s = hfData.samples[i];
		writeWord(PxU16(s.height), endian, stream);
		stream.write(&s.materialIndex0, sizeof(s.materialIndex0));
		stream.write(&s.materialIndex1, sizeof(s.materialIndex1));
	}

	return true;
}

}
