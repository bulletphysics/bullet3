/*
 *	OPCODE - Optimized Collision Detection
 * http://www.codercorner.com/Opcode.htm
 * 
 * Copyright (c) 2001-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains a mesh interface.
 *	\file		OPC_MeshInterface.h
 *	\author		Pierre Terdiman
 *	\date		November, 27, 2002
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __OPC_MESHINTERFACE_H__
#define __OPC_MESHINTERFACE_H__

	struct VertexPointers
	{
		const Point*	Vertex[3];

		bool BackfaceCulling(const Point& source)
		{
			const Point& p0 = *Vertex[0];
			const Point& p1 = *Vertex[1];
			const Point& p2 = *Vertex[2];

			// Compute normal direction
			Point Normal = (p2 - p1)^(p0 - p1);

			// Backface culling
			return (Normal | (source - p0)) >= 0.0f;
		}
	};

#ifdef OPC_USE_CALLBACKS
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	User-callback, called by OPCODE to request vertices from the app.
	 *	\param		triangle_index	[in] face index for which the system is requesting the vertices
	 *	\param		triangle		[out] triangle's vertices (must be provided by the user)
	 *	\param		user_data		[in] user-defined data from SetCallback()
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	typedef void	(*RequestCallback)	(udword triangle_index, VertexPointers& triangle, void* user_data);
#endif

	class OPCODE_API MeshInterface
	{
		public:
		// Constructor / Destructor
											MeshInterface();
											~MeshInterface();
		// Common settings
		inline_			udword				GetNbTriangles()	const	{ return mNbTris;	}
		inline_			udword				GetNbVertices()		const	{ return mNbVerts;	}
		inline_			void				SetNbTriangles(udword nb)	{ mNbTris = nb;		}
		inline_			void				SetNbVertices(udword nb)	{ mNbVerts = nb;	}

#ifdef OPC_USE_CALLBACKS
		// Callback settings

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Callback control: setups object callback. Must provide triangle-vertices for a given triangle index.
		 *	\param		callback	[in] user-defined callback
		 *	\param		user_data	[in] user-defined data
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						bool				SetCallback(RequestCallback callback, void* user_data);
		inline_			void*				GetUserData()		const	{ return mUserData;		}
		inline_			RequestCallback		GetCallback()		const	{ return mObjCallback;	}
#else
		// Pointers settings

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Pointers control: setups object pointers. Must provide access to faces and vertices for a given object.
		 *	\param		tris	[in] pointer to triangles
		 *	\param		verts	[in] pointer to vertices
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						bool				SetPointers(const IndexedTriangle* tris, const Point* verts);
		inline_	const	IndexedTriangle*	GetTris()			const	{ return mTris;			}
		inline_	const	Point*				GetVerts()			const	{ return mVerts;		}

	#ifdef OPC_USE_STRIDE
		// Strides settings

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Strides control
		 *	\param		tri_stride		[in] size of a triangle in bytes. The first sizeof(IndexedTriangle) bytes are used to get vertex indices.
		 *	\param		vertex_stride	[in] size of a vertex in bytes. The first sizeof(Point) bytes are used to get vertex position.
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						bool				SetStrides(udword tri_stride=sizeof(IndexedTriangle), udword vertex_stride=sizeof(Point));
		inline_			udword				GetTriStride()		const	{ return mTriStride;	}
		inline_			udword				GetVertexStride()	const	{ return mVertexStride;	}
	#endif
#endif

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Fetches a triangle given a triangle index.
		 *	\param		vp		[out] required triangle's vertex pointers
		 *	\param		index	[in] triangle index
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			void				GetTriangle(VertexPointers& vp, udword index)	const
											{
#ifdef OPC_USE_CALLBACKS
												(mObjCallback)(index, vp, mUserData);
#else
	#ifdef OPC_USE_STRIDE
												const IndexedTriangle* T = (const IndexedTriangle*)(((ubyte*)mTris) + index * mTriStride);
												vp.Vertex[0] = (const Point*)(((ubyte*)mVerts) + T->mVRef[0] * mVertexStride);
												vp.Vertex[1] = (const Point*)(((ubyte*)mVerts) + T->mVRef[1] * mVertexStride);
												vp.Vertex[2] = (const Point*)(((ubyte*)mVerts) + T->mVRef[2] * mVertexStride);
	#else
												const IndexedTriangle* T = &mTris[index];
												vp.Vertex[0] = &mVerts[T->mVRef[0]];
												vp.Vertex[1] = &mVerts[T->mVRef[1]];
												vp.Vertex[2] = &mVerts[T->mVRef[2]];
	#endif
#endif
											}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Remaps client's mesh according to a permutation.
		 *	\param		nb_indices	[in] number of indices in the permutation (will be checked against number of triangles)
		 *	\param		permutation	[in] list of triangle indices
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			bool				RemapClient(udword nb_indices, const udword* permutation)	const;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Checks the mesh interface is valid, i.e. things have been setup correctly.
		 *	\return		true if valid
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						bool				IsValid()		const;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Checks the mesh itself is valid.
		 *	Currently we only look for degenerate faces.
		 *	\return		number of degenerate faces
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						udword				CheckTopology()	const;
		private:

						udword				mNbTris;			//!< Number of triangles in the input model
						udword				mNbVerts;			//!< Number of vertices in the input model
#ifdef OPC_USE_CALLBACKS
		// User callback
						void*				mUserData;			//!< User-defined data sent to callback
						RequestCallback		mObjCallback;		//!< Object callback
#else
		// User pointers
				const	IndexedTriangle*	mTris;				//!< Array of indexed triangles
				const	Point*				mVerts;				//!< Array of vertices
	#ifdef OPC_USE_STRIDE
						udword				mTriStride;			//!< Possible triangle stride in bytes [Opcode 1.3]
						udword				mVertexStride;		//!< Possible vertex stride in bytes [Opcode 1.3]
	#endif
#endif
	};

#endif //__OPC_MESHINTERFACE_H__