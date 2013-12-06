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
 *	Contains code for a triangle container.
 *	\file		IceTrilist.h
 *	\author		Pierre Terdiman
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __ICETRILIST_H__
#define __ICETRILIST_H__

	class ICEMATHS_API TriList : public Container
	{
		public:
		// Constructor / Destructor
								TriList()					{}
								~TriList()					{}

		inline_	udword			GetNbTriangles()	const	{ return GetNbEntries()/9;			}
		inline_	Triangle*		GetTriangles()		const	{ return (Triangle*)GetEntries();	}

				void			AddTri(const Triangle& tri)
								{
									Add(tri.mVerts[0].x).Add(tri.mVerts[0].y).Add(tri.mVerts[0].z);
									Add(tri.mVerts[1].x).Add(tri.mVerts[1].y).Add(tri.mVerts[1].z);
									Add(tri.mVerts[2].x).Add(tri.mVerts[2].y).Add(tri.mVerts[2].z);
								}

				void			AddTri(const Point& p0, const Point& p1, const Point& p2)
								{
									Add(p0.x).Add(p0.y).Add(p0.z);
									Add(p1.x).Add(p1.y).Add(p1.z);
									Add(p2.x).Add(p2.y).Add(p2.z);
								}
	};

	class ICEMATHS_API TriangleList : public Container
	{
		public:
		// Constructor / Destructor
									TriangleList()				{}
									~TriangleList()				{}

		inline_	udword				GetNbTriangles()	const	{ return GetNbEntries()/3;				}
		inline_	IndexedTriangle*	GetTriangles()		const	{ return (IndexedTriangle*)GetEntries();}

				void				AddTriangle(const IndexedTriangle& tri)
									{
										Add(tri.mVRef[0]).Add(tri.mVRef[1]).Add(tri.mVRef[2]);
									}

				void				AddTriangle(udword vref0, udword vref1, udword vref2)
									{
										Add(vref0).Add(vref1).Add(vref2);
									}
	};

#endif //__ICETRILIST_H__
