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

#ifndef PX_PVD_SCENE_CLIENT_H
#define PX_PVD_SCENE_CLIENT_H

/** \addtogroup pvd
@{
*/

#include "foundation/PxFlags.h"

namespace physx
{
	namespace pvdsdk
	{
		class PvdClient;
		struct PvdDebugPoint;
		struct PvdDebugLine;
		struct PvdDebugTriangle;
		struct PvdDebugText;
	}
}

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief PVD scene Flags. They are disabled by default, and only works if PxPvdInstrumentationFlag::eDEBUG is set.
*/
struct PxPvdSceneFlag
{
	enum Enum
	{
		eTRANSMIT_CONTACTS     = (1 << 0), //! Transmits contact stream to PVD.
		eTRANSMIT_SCENEQUERIES = (1 << 1), //! Transmits scene query stream to PVD.
		eTRANSMIT_CONSTRAINTS  = (1 << 2)  //! Transmits constraints visualize stream to PVD.
	};
};

/**
\brief Bitfield that contains a set of raised flags defined in PxPvdSceneFlag.

@see PxPvdSceneFlag
*/
typedef PxFlags<PxPvdSceneFlag::Enum, PxU8> PxPvdSceneFlags;
PX_FLAGS_OPERATORS(PxPvdSceneFlag::Enum, PxU8)

/**
\brief Special client for PxScene.
It provides access to the PxPvdSceneFlag.
It also provides simple user debug services that associated scene position such as immediate rendering and camera updates.
*/
class PxPvdSceneClient
{
  public:
	/**
	Sets the PVD flag. See PxPvdSceneFlag.
	\param flag Flag to set.
	\param value value the flag gets set to.
	*/
	virtual void setScenePvdFlag(PxPvdSceneFlag::Enum flag, bool value) = 0;

	/**
	Sets the PVD flags. See PxPvdSceneFlags.
	\param flags Flags to set.
	*/
	virtual void setScenePvdFlags(PxPvdSceneFlags flags) = 0;

	/**
	Retrieves the PVD flags. See PxPvdSceneFlags.
	*/
	virtual PxPvdSceneFlags getScenePvdFlags() const = 0;

	/**
	update camera on PVD application's render window
	*/
	virtual void updateCamera(const char* name, const PxVec3& origin, const PxVec3& up, const PxVec3& target) = 0;

	/**
	draw points on PVD application's render window
	*/
	virtual void drawPoints(const physx::pvdsdk::PvdDebugPoint* points, PxU32 count) = 0;

	/**
	draw lines on PVD application's render window
	*/
	virtual void drawLines(const physx::pvdsdk::PvdDebugLine* lines, PxU32 count) = 0;

	/**
	draw triangles on PVD application's render window
	*/
	virtual void drawTriangles(const physx::pvdsdk::PvdDebugTriangle* triangles, PxU32 count) = 0;

	/**
	draw text on PVD application's render window
	*/
	virtual void drawText(const physx::pvdsdk::PvdDebugText& text) = 0;

	/**
	get the underlying client, for advanced users
	*/
	virtual physx::pvdsdk::PvdClient* getClientInternal() = 0;

protected:
	virtual ~PxPvdSceneClient(){}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif // PX_PVD_SCENE_CLIENT_H
