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
 *	Contains common classes & defs used in OPCODE.
 *	\file		OPC_Common.h
 *	\author		Pierre Terdiman
 *	\date		March, 20, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __OPC_COMMON_H__
#define __OPC_COMMON_H__

// [GOTTFRIED]: Just a small change for readability.
#ifdef OPC_CPU_COMPARE
	#define GREATER(x, y)	AIR(x) > IR(y)
#else
	#define GREATER(x, y)	fabsf(x) > (y)
#endif

	class OPCODE_API CollisionAABB
	{
		public:
		//! Constructor
		inline_				CollisionAABB()						{}
		//! Constructor
		inline_				CollisionAABB(const AABB& b)		{ b.GetCenter(mCenter);	b.GetExtents(mExtents);	}
		//! Destructor
		inline_				~CollisionAABB()					{}

		//! Get min point of the box
		inline_	void		GetMin(Point& min)		const		{ min = mCenter - mExtents;					}
		//! Get max point of the box
		inline_	void		GetMax(Point& max)		const		{ max = mCenter + mExtents;					}

		//! Get component of the box's min point along a given axis
		inline_	float		GetMin(udword axis)		const		{ return mCenter[axis] - mExtents[axis];	}
		//! Get component of the box's max point along a given axis
		inline_	float		GetMax(udword axis)		const		{ return mCenter[axis] + mExtents[axis];	}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Setups an AABB from min & max vectors.
		 *	\param		min			[in] the min point
		 *	\param		max			[in] the max point
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_	void		SetMinMax(const Point& min, const Point& max)		{ mCenter = (max + min)*0.5f; mExtents = (max - min)*0.5f;		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Checks a box is inside another box.
		 *	\param		box		[in] the other box
		 *	\return		true if current box is inside input box
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_	BOOL		IsInside(const CollisionAABB& box) const
							{
								if(box.GetMin(0)>GetMin(0))	return FALSE;
								if(box.GetMin(1)>GetMin(1))	return FALSE;
								if(box.GetMin(2)>GetMin(2))	return FALSE;
								if(box.GetMax(0)<GetMax(0))	return FALSE;
								if(box.GetMax(1)<GetMax(1))	return FALSE;
								if(box.GetMax(2)<GetMax(2))	return FALSE;
								return TRUE;
							}

				Point		mCenter;				//!< Box center
				Point		mExtents;				//!< Box extents
	};

	class OPCODE_API QuantizedAABB
	{
		public:
		//! Constructor
		inline_				QuantizedAABB()			{}
		//! Destructor
		inline_				~QuantizedAABB()		{}

				sword		mCenter[3];				//!< Quantized center
				uword		mExtents[3];			//!< Quantized extents
	};

	//! Quickly rotates & translates a vector
	inline_ void TransformPoint(Point& dest, const Point& source, const Matrix3x3& rot, const Point& trans)
	{
		dest.x = trans.x + source.x * rot.m[0][0] + source.y * rot.m[1][0] + source.z * rot.m[2][0];
		dest.y = trans.y + source.x * rot.m[0][1] + source.y * rot.m[1][1] + source.z * rot.m[2][1];
		dest.z = trans.z + source.x * rot.m[0][2] + source.y * rot.m[1][2] + source.z * rot.m[2][2];
	}

#endif //__OPC_COMMON_H__