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


#ifndef PX_PHYSICS_EXTENSIONS_SMOOTH_NORMALS_H
#define PX_PHYSICS_EXTENSIONS_SMOOTH_NORMALS_H
/** \addtogroup extensions
  @{
*/

#include "common/PxPhysXCommonConfig.h"

/**
\brief Builds smooth vertex normals over a mesh.

- "smooth" because smoothing groups are not supported here
- takes angles into account for correct cube normals computation

To use 32bit indices pass a pointer in dFaces and set wFaces to zero. Alternatively pass a pointer to 
wFaces and set dFaces to zero.

\param[in] nbTris Number of triangles
\param[in] nbVerts Number of vertices
\param[in] verts Array of vertices
\param[in] dFaces Array of dword triangle indices, or null
\param[in] wFaces Array of word triangle indices, or null
\param[out] normals Array of computed normals (assumes nbVerts vectors)
\param[in] flip Flips the normals or not
\return True on success.
*/
PX_C_EXPORT bool PX_CALL_CONV PxBuildSmoothNormals(physx::PxU32 nbTris, physx::PxU32 nbVerts, const physx::PxVec3* verts,
												   const physx::PxU32* dFaces, const physx::PxU16* wFaces, physx::PxVec3* normals, bool flip);

/** @} */
#endif
