/*
 *	ICE / OPCODE - Optimized Collision Detection
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
 *	Contains a handy indexed triangle class.
 *	\file		IceIndexedTriangle.h
 *	\author		Pierre Terdiman
 *	\date		January, 17, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __ICEINDEXEDTRIANGLE_H__
#define __ICEINDEXEDTRIANGLE_H__

	// Forward declarations
	enum CubeIndex;

	// An indexed triangle class.
	class ICEMATHS_API IndexedTriangle
	{
		public:
		//! Constructor
		inline_					IndexedTriangle()									{}
		//! Constructor
		inline_					IndexedTriangle(udword r0, udword r1, udword r2)	{ mVRef[0]=r0; mVRef[1]=r1; mVRef[2]=r2; }
		//! Copy constructor
		inline_					IndexedTriangle(const IndexedTriangle& triangle)
								{
									mVRef[0] = triangle.mVRef[0];
									mVRef[1] = triangle.mVRef[1];
									mVRef[2] = triangle.mVRef[2];
								}
		//! Destructor
		inline_					~IndexedTriangle()									{}
		//! Vertex-references
				udword			mVRef[3];

		// Methods
				void			Flip();
				float			Area(const Point* verts)											const;
				float			Perimeter(const Point* verts)										const;
				float			Compacity(const Point* verts)										const;
				void			Normal(const Point* verts, Point& normal)							const;
				void			DenormalizedNormal(const Point* verts, Point& normal)				const;
				void			Center(const Point* verts, Point& center)							const;
				void			CenteredNormal(const Point* verts, Point& normal)					const;
				void			RandomPoint(const Point* verts, Point& random)						const;
				bool			IsVisible(const Point* verts, const Point& source)					const;
				bool			BackfaceCulling(const Point* verts, const Point& source)			const;
				float			ComputeOcclusionPotential(const Point* verts, const Point& view)	const;
				bool			ReplaceVertex(udword oldref, udword newref);
				bool			IsDegenerate()														const;
				bool			HasVertex(udword ref)												const;
				bool			HasVertex(udword ref, udword* index)								const;
				ubyte			FindEdge(udword vref0, udword vref1)								const;
				udword			OppositeVertex(udword vref0, udword vref1)							const;
		inline_	udword			OppositeVertex(ubyte edgenb)										const	{ return mVRef[2-edgenb];	}
				void			GetVRefs(ubyte edgenb, udword& vref0, udword& vref1, udword& vref2)	const;
				float			MinEdgeLength(const Point* verts)									const;
				float			MaxEdgeLength(const Point* verts)									const;
				void			ComputePoint(const Point* verts, float u, float v, Point& pt, udword* nearvtx=null)	const;
				float			Angle(const IndexedTriangle& tri, const Point* verts)				const;
		inline_	Plane			PlaneEquation(const Point* verts)									const	{ return Plane(verts[mVRef[0]], verts[mVRef[1]], verts[mVRef[2]]);	}
				bool			Equal(const IndexedTriangle& tri)									const;
				CubeIndex		ComputeCubeIndex(const Point* verts)								const;
	};

#endif // __ICEINDEXEDTRIANGLE_H__
