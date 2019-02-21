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


#ifndef PX_EDGELIST
#define PX_EDGELIST

// PT: this file should be moved to cooking lib

#include "foundation/Px.h"
#include "PsUserAllocated.h"

// Data/code shared with LL
#include "GuEdgeListData.h"

namespace physx
{
namespace Gu
{
	//! The edge-list creation structure.
	struct EDGELISTCREATE
	{
								EDGELISTCREATE() :
								NbFaces			(0),
								DFaces			(NULL),
								WFaces			(NULL),
								FacesToEdges	(false),
								EdgesToFaces	(false),
								Verts			(NULL),
								Epsilon			(0.1f)
								{}
				
				PxU32			NbFaces;		//!< Number of faces in source topo
				const PxU32*	DFaces;			//!< List of faces (dwords) or NULL
				const PxU16*	WFaces;			//!< List of faces (words) or NULL

				bool			FacesToEdges;
				bool			EdgesToFaces;
				const PxVec3*	Verts;
				float			Epsilon;
	};

	class EdgeList : public Ps::UserAllocated
	{
		public:
												EdgeList();
												~EdgeList();

				bool							load(PxInputStream& stream);
		// Data access
		PX_INLINE	PxU32						getNbEdges()							const	{ return mData.mNbEdges;						}
		PX_INLINE	const Gu::EdgeData*			getEdges()								const	{ return mData.mEdges;							}
		PX_INLINE	const Gu::EdgeData&			getEdge(PxU32 edge_index)				const	{ return mData.mEdges[edge_index];				}
		//
		PX_INLINE	PxU32						getNbFaces()							const	{ return mData.mNbFaces;						}
		PX_INLINE	const Gu::EdgeTriangleData* getEdgeTriangles()						const	{ return mData.mEdgeFaces;						}
		PX_INLINE	const Gu::EdgeTriangleData& getEdgeTriangle(PxU32 face_index)		const	{ return mData.mEdgeFaces[face_index];			}
		//
		PX_INLINE	const Gu::EdgeDescData*		getEdgeToTriangles()					const	{ return mData.mEdgeToTriangles;				}
		PX_INLINE	const Gu::EdgeDescData&		getEdgeToTriangles(PxU32 edge_index)	const	{ return mData.mEdgeToTriangles[edge_index];	}
		PX_INLINE	const PxU32*				getFacesByEdges()						const	{ return mData.mFacesByEdges;					}
		PX_INLINE	PxU32						getFacesByEdges(PxU32 face_index)		const	{ return mData.mFacesByEdges[face_index];		}

		protected:
				Gu::EdgeListData				mData;					//!< Pointer to edgelist data
	};

	class EdgeListBuilder : public EdgeList
	{
		public:
												EdgeListBuilder();
												~EdgeListBuilder();

					bool						init(const EDGELISTCREATE& create);
		private:
					bool						createFacesToEdges(PxU32 nb_faces, const PxU32* dfaces, const PxU16* wfaces);
					bool						createEdgesToFaces(PxU32 nb_faces, const PxU32* dfaces, const PxU16* wfaces);
					bool						computeActiveEdges(PxU32 nb_faces, const PxU32* dfaces, const PxU16* wfaces, const PxVec3* verts, float epsilon);
	};
}

}

#endif
