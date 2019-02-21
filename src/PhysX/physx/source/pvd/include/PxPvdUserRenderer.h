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
#ifndef PXPVDSDK_PXPVDUSERRENDERER_H
#define PXPVDSDK_PXPVDUSERRENDERER_H

/** \addtogroup pvd
@{
*/
#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "pvd/PxPvd.h"

#include "PxPvdDataStream.h"
#include "PxPvdRenderBuffer.h"
#include "PsUserAllocated.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxPvd;

#if !PX_DOXYGEN
namespace pvdsdk
{
#endif

class RendererEventClient;

class PvdUserRenderer : public shdfnd::UserAllocated
{
  protected:
	virtual ~PvdUserRenderer()
	{
	}

  public:
	virtual void release() = 0;
	virtual void setClient(RendererEventClient* client) = 0;

	// Instance to associate the further rendering with.
	virtual void setInstanceId(const void* instanceId) = 0;
	// Draw these points associated with this instance
	virtual void drawPoints(const PvdDebugPoint* points, uint32_t count) = 0;
	// Draw these lines associated with this instance
	virtual void drawLines(const PvdDebugLine* lines, uint32_t count) = 0;
	// Draw these triangles associated with this instance
	virtual void drawTriangles(const PvdDebugTriangle* triangles, uint32_t count) = 0;
	// Draw this text associated with this instance
	virtual void drawText(const PvdDebugText& text) = 0;

	// Draw SDK debug render
	virtual void drawRenderbuffer(const PvdDebugPoint* pointData, uint32_t pointCount, const PvdDebugLine* lineData,
	                              uint32_t lineCount, const PvdDebugTriangle* triangleData, uint32_t triangleCount) = 0;

	// Constraint visualization routines
	virtual void visualizeJointFrames(const PxTransform& parent, const PxTransform& child) = 0;
	virtual void visualizeLinearLimit(const PxTransform& t0, const PxTransform& t1, float value, bool active) = 0;
	virtual void visualizeAngularLimit(const PxTransform& t0, float lower, float upper, bool active) = 0;
	virtual void visualizeLimitCone(const PxTransform& t, float tanQSwingY, float tanQSwingZ, bool active) = 0;
	virtual void visualizeDoubleCone(const PxTransform& t, float angle, bool active) = 0;

	// Clear the immedate buffer.
	virtual void flushRenderEvents() = 0;

	static PvdUserRenderer* create(uint32_t bufferSize = 0x2000);
};

class RendererEventClient 
{
 public:
	virtual ~RendererEventClient(){}

	virtual void handleBufferFlush(const uint8_t* inData, uint32_t inLength) = 0;
};

#if !PX_DOXYGEN
}
}
#endif
/** @} */
#endif // PXPVDSDK_PXPVDUSERRENDERER_H
