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


#include "foundation/PxMat44.h"
#include "CmRenderOutput.h"
#include "PsMathUtils.h"
#include "PsString.h"
#include <stdarg.h>

#if PX_VC
#pragma warning(disable: 4342 ) // behavior change: 'function' called, but a member operator was called in previous versions
#pragma warning(disable: 4996 ) // intentionally suppressing this warning message
#endif

using namespace physx;
using namespace Cm;


namespace physx
{
namespace Cm
{
	RenderOutput& RenderOutput::operator<<(Primitive prim)
	{
		mPrim = prim; mVertexCount = 0; return *this;
	}

	RenderOutput& RenderOutput::operator<<(PxU32 color)
	{
		mColor = color; return *this;
	}

	RenderOutput& RenderOutput::operator<<(const PxMat44& transform)
	{
		mTransform = transform; return *this;
	}

	RenderOutput& RenderOutput::operator<<(const PxTransform&t)
	{
		mTransform = PxMat44(t);
		return *this;
	}

	RenderOutput& RenderOutput::operator<<(PxVec3 vertex)
	{
		// apply transformation
		vertex = mTransform.transform(vertex);
		++mVertexCount;

		// add primitive to render buffer
		switch(mPrim)
		{
		case POINTS:
			mBuffer.mPoints.pushBack(PxDebugPoint(vertex, mColor)); break;
		case LINES:
			if(mVertexCount == 2)
			{
				mBuffer.mLines.pushBack(PxDebugLine(mVertex0, vertex, mColor)); 
				mVertexCount = 0;
			}
			break;
		case LINESTRIP:
			if(mVertexCount >= 2)
				mBuffer.mLines.pushBack(PxDebugLine(mVertex0, vertex, mColor)); 
			break;
		case TRIANGLES:
			if(mVertexCount == 3)
			{
				mBuffer.mTriangles.pushBack(PxDebugTriangle(mVertex1, mVertex0, vertex, mColor)); 
				mVertexCount = 0;
			}
			break;
		case TRIANGLESTRIP:
			if(mVertexCount >= 3)
				mBuffer.mTriangles.pushBack(PxDebugTriangle(
					(mVertexCount & 0x1) ? mVertex0 : mVertex1, 
					(mVertexCount & 0x1) ? mVertex1 : mVertex0, vertex, mColor)); 
			break;
		case TEXT:	
			break;
		}

		// cache the last 2 vertices (for strips)
		if(1 < mVertexCount)
		{
			mVertex1 = mVertex0;
			mVertex0 = vertex;
		} else {
			mVertex0 = vertex;
		}
		return *this;
	}

	DebugText::DebugText(const PxVec3& position_, PxReal size_, const char* string, ...) 
	: position(position_), size(size_) 
	{
		va_list argList;
		va_start(argList, string);
		if(0 >= Ps::vsnprintf(buffer, sBufferSize-1, string, argList))
			buffer[sBufferSize-1] = 0; // terminate string
		va_end(argList);
	}

	RenderOutput& RenderOutput::operator<<(const DebugText& text)
	{
		const PxU32 n = PxU32(strlen(text.buffer));
		const PxU32 newCharBufSize = mBuffer.mCharBuf.size()+n+1;
		if(mBuffer.mCharBuf.capacity() < newCharBufSize)
		{
			char* oldBuf = mBuffer.mCharBuf.begin(); 
			mBuffer.mCharBuf.reserve(newCharBufSize);
			intptr_t diff =  mBuffer.mCharBuf.begin() - oldBuf;
			for (PxU32 i = 0; i < mBuffer.mTexts.size(); ++i)
				mBuffer.mTexts[i].string += diff;
		}
		mBuffer.mTexts.pushBack(PxDebugText(mTransform.transform(text.position), text.size, mColor, mBuffer.mCharBuf.end()));
		for(size_t i=0; i<=n; ++i)
			mBuffer.mCharBuf.pushBack(text.buffer[i]);
		return *this;
	}

	RenderOutput& operator<<(RenderOutput& out, const DebugBox& box)
	{
		if(box.wireframe)
		{
			out << RenderOutput::LINESTRIP;
			out << PxVec3(box.minimum.x, box.minimum.y, box.minimum.z);
			out << PxVec3(box.maximum.x, box.minimum.y, box.minimum.z);
			out << PxVec3(box.maximum.x, box.maximum.y, box.minimum.z);
			out << PxVec3(box.minimum.x, box.maximum.y, box.minimum.z);
			out << PxVec3(box.minimum.x, box.minimum.y, box.minimum.z);
			out << PxVec3(box.minimum.x, box.minimum.y, box.maximum.z);
			out << PxVec3(box.maximum.x, box.minimum.y, box.maximum.z);
			out << PxVec3(box.maximum.x, box.maximum.y, box.maximum.z);
			out << PxVec3(box.minimum.x, box.maximum.y, box.maximum.z);
			out << PxVec3(box.minimum.x, box.minimum.y, box.maximum.z);
			out << RenderOutput::LINES;
			out << PxVec3(box.maximum.x, box.minimum.y, box.minimum.z); 
			out << PxVec3(box.maximum.x, box.minimum.y, box.maximum.z);
			out << PxVec3(box.maximum.x, box.maximum.y, box.minimum.z); 
			out << PxVec3(box.maximum.x, box.maximum.y, box.maximum.z);
			out << PxVec3(box.minimum.x, box.maximum.y, box.minimum.z); 
			out << PxVec3(box.minimum.x, box.maximum.y, box.maximum.z);
		}
		else
		{
			out << RenderOutput::TRIANGLESTRIP;
			out << PxVec3(box.minimum.x, box.minimum.y, box.minimum.z); // 0
			out << PxVec3(box.minimum.x, box.maximum.y, box.minimum.z); // 2
			out << PxVec3(box.maximum.x, box.minimum.y, box.minimum.z); // 1
			out << PxVec3(box.maximum.x, box.maximum.y, box.minimum.z); // 3
			out << PxVec3(box.maximum.x, box.maximum.y, box.maximum.z); // 7
			out << PxVec3(box.minimum.x, box.maximum.y, box.minimum.z); // 2
			out << PxVec3(box.minimum.x, box.maximum.y, box.maximum.z); // 6
			out << PxVec3(box.minimum.x, box.minimum.y, box.minimum.z); // 0
			out << PxVec3(box.minimum.x, box.minimum.y, box.maximum.z); // 4
			out << PxVec3(box.maximum.x, box.minimum.y, box.minimum.z); // 1
			out << PxVec3(box.maximum.x, box.minimum.y, box.maximum.z); // 5
			out << PxVec3(box.maximum.x, box.maximum.y, box.maximum.z); // 7
			out << PxVec3(box.minimum.x, box.minimum.y, box.maximum.z); // 4
			out << PxVec3(box.minimum.x, box.maximum.y, box.maximum.z); // 6
		}
		return out;
	}

	RenderOutput& operator<<(RenderOutput& out, const DebugArrow& arrow)
	{
		PxVec3 t0 = arrow.tip - arrow.base, t1, t2; 

		t0.normalize();
		Ps::normalToTangents(t0, t1, t2);

		const PxReal tipAngle = 0.25f;
		t1 *= arrow.headLength * tipAngle; 
		t2 *= arrow.headLength * tipAngle * PxSqrt(3.0f); 
		PxVec3 headBase = arrow.tip - t0 * arrow.headLength;

		out << RenderOutput::LINES;
		out << arrow.base << arrow.tip;

		out << RenderOutput::TRIANGLESTRIP;
		out << arrow.tip;
		out << headBase + t1 + t1;
		out << headBase - t1 - t2;
		out << headBase - t1 + t2;
		out << arrow.tip;
		out << headBase + t1 + t1;
		return out;
	}

	RenderOutput& operator<<(RenderOutput& out, const DebugBasis& basis)
	{
		const PxReal headLength = basis.extends.magnitude() * 0.15f;
		out << basis.colorX << DebugArrow(PxVec3(0.0f), PxVec3(basis.extends.x, 0, 0), headLength);
		out << basis.colorY << DebugArrow(PxVec3(0.0f), PxVec3(0, basis.extends.y, 0), headLength);
		out << basis.colorZ << DebugArrow(PxVec3(0.0f), PxVec3(0, 0, basis.extends.z), headLength);
		return out;
	}

	RenderOutput& operator<<(RenderOutput& out, const DebugCircle& circle)
	{
		const PxF32 step = PxTwoPi/circle.nSegments;
		PxF32 angle = 0;
		out << RenderOutput::LINESTRIP;
		for(PxU32 i=0; i<circle.nSegments; i++, angle += step)
			out << PxVec3(circle.radius * PxSin(angle), circle.radius * PxCos(angle), 0);
		out << PxVec3(0, circle.radius, 0);
		return out;
	}

	RenderOutput& operator<<(RenderOutput& out, const DebugArc& arc)
	{
		const PxF32 step = (arc.maxAngle - arc.minAngle) / arc.nSegments;
		PxF32 angle = arc.minAngle;
		out << RenderOutput::LINESTRIP;
		for(PxU32 i=0; i<arc.nSegments; i++, angle += step)
			out << PxVec3(arc.radius * PxSin(angle), arc.radius * PxCos(angle), 0);
		out << PxVec3(arc.radius * PxSin(arc.maxAngle), arc.radius * PxCos(arc.maxAngle), 0);
		return out;
	}

	// PT: I need those functions available here so that I don't have to duplicate all the code in other modules like the CCT.
	// PT: TODO: move other functions here as well

	RenderOutput& RenderOutput::outputCapsule(PxF32 radius, PxF32 halfHeight, const PxMat44& absPose)
	{
		RenderOutput& out = *this;

		const PxVec3 vleft2(-halfHeight, 0.0f, 0.0f);
		PxMat44 left2 = absPose;
		left2.column3 += PxVec4(left2.rotate(vleft2), 0.0f);
		out << left2 << Cm::DebugArc(100, radius, PxPi, PxTwoPi);

		PxMat44 rotPose = left2;
		Ps::swap(rotPose.column1, rotPose.column2);
		rotPose.column1 = -rotPose.column1;
		out << rotPose << Cm::DebugArc(100, radius, PxPi, PxTwoPi);

		Ps::swap(rotPose.column0, rotPose.column2);
		rotPose.column0 = -rotPose.column0;
		out << rotPose << Cm::DebugCircle(100, radius);

		const PxVec3 vright2(halfHeight, 0.0f, 0.0f);
		PxMat44 right2 = absPose;
		right2.column3 += PxVec4(right2.rotate(vright2), 0.0f);
		out << right2 << Cm::DebugArc(100, radius, 0.0f, PxPi);

		rotPose = right2;
		Ps::swap(rotPose.column1, rotPose.column2);
		rotPose.column1 = -rotPose.column1;
		out << rotPose << Cm::DebugArc(100, radius, 0.0f, PxPi);

		Ps::swap(rotPose.column0, rotPose.column2);
		rotPose.column0 = -rotPose.column0;
		out << rotPose << Cm::DebugCircle(100, radius);

		out << absPose;
		out.outputSegment(	absPose.transform(PxVec3(-halfHeight,  radius, 0)),
							absPose.transform(PxVec3( halfHeight,  radius, 0)));
		out.outputSegment(	absPose.transform(PxVec3(-halfHeight, -radius, 0)),
							absPose.transform(PxVec3( halfHeight, -radius, 0)));
		out.outputSegment(	absPose.transform(PxVec3(-halfHeight,  0, radius)),
							absPose.transform(PxVec3( halfHeight,  0, radius)));
		out.outputSegment(	absPose.transform(PxVec3(-halfHeight, 0, -radius)),
							absPose.transform(PxVec3( halfHeight, 0, -radius)));

		return *this;
	}



} // Cm

}
