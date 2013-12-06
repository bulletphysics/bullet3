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
 *	Contains AABB-related code. (axis-aligned bounding box)
 *	\file		IceAABB.h
 *	\author		Pierre Terdiman
 *	\date		January, 13, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __ICEAABB_H__
#define __ICEAABB_H__

	// Forward declarations
	class Sphere;

//! Declarations of type-independent methods (most of them implemented in the .cpp)
#define AABB_COMMON_METHODS																											\
			AABB&			Add(const AABB& aabb);																					\
			float			MakeCube(AABB& cube)																			const;	\
			void			MakeSphere(Sphere& sphere)																		const;	\
			const sbyte*	ComputeOutline(const Point& local_eye, sdword& num)												const;	\
			float			ComputeBoxArea(const Point& eye, const Matrix4x4& mat, float width, float height, sdword& num)	const;	\
			bool			IsInside(const AABB& box)																		const;	\
			bool			ComputePlanes(Plane* planes)																	const;	\
			bool			ComputePoints(Point* pts)																		const;	\
			const Point*	GetVertexNormals()																				const;	\
			const udword*	GetEdges()																						const;	\
			const Point*	GetEdgeNormals()																				const;	\
	inline_	BOOL			ContainsPoint(const Point& p)																	const	\
							{																										\
								if(p.x > GetMax(0) || p.x < GetMin(0)) return FALSE;												\
								if(p.y > GetMax(1) || p.y < GetMin(1)) return FALSE;												\
								if(p.z > GetMax(2) || p.z < GetMin(2)) return FALSE;												\
								return TRUE;																						\
							}

	enum AABBType
	{
		AABB_RENDER			= 0,	//!< AABB used for rendering. Not visible == not rendered.
		AABB_UPDATE			= 1,	//!< AABB used for dynamic updates. Not visible == not updated.

		AABB_FORCE_DWORD	= 0x7fffffff,
	};

#ifdef USE_MINMAX

	struct ICEMATHS_API ShadowAABB
	{
		Point	mMin;
		Point	mMax;
	};

	class ICEMATHS_API AABB
	{
		public:
		//! Constructor
		inline_						AABB()	{}
		//! Destructor
		inline_						~AABB()	{}

		//! Type-independent methods
									AABB_COMMON_METHODS;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Setups an AABB from min & max vectors.
		 *	\param		min			[in] the min point
		 *	\param		max			[in] the max point
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						void		SetMinMax(const Point& min, const Point& max)		{ mMin = min;		mMax = max;									}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Setups an AABB from center & extents vectors.
		 *	\param		c			[in] the center point
		 *	\param		e			[in] the extents vector
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						void		SetCenterExtents(const Point& c, const Point& e)	{ mMin = c - e;		mMax = c + e;								}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Setups an empty AABB.
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						void		SetEmpty()											{ Point p(MIN_FLOAT, MIN_FLOAT, MIN_FLOAT);	mMin = -p; mMax = p;}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Setups a point AABB.
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						void		SetPoint(const Point& pt)							{ mMin = mMax = pt;												}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Gets the size of the AABB. The size is defined as the longest extent.
		 *	\return		the size of the AABB
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						float		GetSize()								const		{ Point e; GetExtents(e);	return e.Max();	}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Extends the AABB.
		 *	\param		p	[in] the next point
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						void		Extend(const Point& p)
									{
										if(p.x > mMax.x)	mMax.x = p.x;
										if(p.x < mMin.x)	mMin.x = p.x;

										if(p.y > mMax.y)	mMax.y = p.y;
										if(p.y < mMin.y)	mMin.y = p.y;

										if(p.z > mMax.z)	mMax.z = p.z;
										if(p.z < mMin.z)	mMin.z = p.z;
									}
		// Data access

		//! Get min point of the box
		inline_			void		GetMin(Point& min)						const		{ min = mMin;								}
		//! Get max point of the box
		inline_			void		GetMax(Point& max)						const		{ max = mMax;								}

		//! Get component of the box's min point along a given axis
		inline_			float		GetMin(udword axis)						const		{ return mMin[axis];						}
		//! Get component of the box's max point along a given axis
		inline_			float		GetMax(udword axis)						const		{ return mMax[axis];						}

		//! Get box center
		inline_			void		GetCenter(Point& center)				const		{ center = (mMax + mMin)*0.5f;				}
		//! Get box extents
		inline_			void		GetExtents(Point& extents)				const		{ extents = (mMax - mMin)*0.5f;				}

		//! Get component of the box's center along a given axis
		inline_			float		GetCenter(udword axis)					const		{ return (mMax[axis] + mMin[axis])*0.5f;	}
		//! Get component of the box's extents along a given axis
		inline_			float		GetExtents(udword axis)					const		{ return (mMax[axis] - mMin[axis])*0.5f;	}

		//! Get box diagonal
		inline_			void		GetDiagonal(Point& diagonal)			const		{ diagonal = mMax - mMin;					}
		inline_			float		GetWidth()								const		{ return mMax.x - mMin.x;					}
		inline_			float		GetHeight()								const		{ return mMax.y - mMin.y;					}
		inline_			float		GetDepth()								const		{ return mMax.z - mMin.z;					}

		//! Volume
		inline_			float		GetVolume()								const		{ return GetWidth() * GetHeight() * GetDepth();		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Computes the intersection between two AABBs.
		 *	\param		a		[in] the other AABB
		 *	\return		true on intersection
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			BOOL		Intersect(const AABB& a)				const
									{
										if(mMax.x < a.mMin.x
										|| a.mMax.x < mMin.x
										|| mMax.y < a.mMin.y
										|| a.mMax.y < mMin.y
										|| mMax.z < a.mMin.z
										|| a.mMax.z < mMin.z)	return FALSE;

										return TRUE;
									}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Computes the 1D-intersection between two AABBs, on a given axis.
		 *	\param		a		[in] the other AABB
		 *	\param		axis	[in] the axis (0, 1, 2)
		 *	\return		true on intersection
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			BOOL		Intersect(const AABB& a, udword axis)	const
									{
										if(mMax[axis] < a.mMin[axis] || a.mMax[axis] < mMin[axis])	return FALSE;
										return TRUE;
									}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Recomputes the AABB after an arbitrary transform by a 4x4 matrix.
		 *	Original code by Charles Bloom on the GD-Algorithm list. (I slightly modified it)
		 *	\param		mtx			[in] the transform matrix
		 *	\param		aabb		[out] the transformed AABB [can be *this]
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			void		Rotate(const Matrix4x4& mtx, AABB& aabb)	const
									{
										// The three edges transformed: you can efficiently transform an X-only vector
										// by just getting the "X" column of the matrix
										Point vx,vy,vz;
										mtx.GetRow(0, vx);	vx *= (mMax.x - mMin.x);
										mtx.GetRow(1, vy);	vy *= (mMax.y - mMin.y);
										mtx.GetRow(2, vz);	vz *= (mMax.z - mMin.z);

										// Transform the min point
										aabb.mMin = aabb.mMax = mMin * mtx;

										// Take the transformed min & axes and find new extents
										// Using CPU code in the right place is faster...
										if(IS_NEGATIVE_FLOAT(vx.x))	aabb.mMin.x += vx.x; else aabb.mMax.x += vx.x;
										if(IS_NEGATIVE_FLOAT(vx.y))	aabb.mMin.y += vx.y; else aabb.mMax.y += vx.y;
										if(IS_NEGATIVE_FLOAT(vx.z))	aabb.mMin.z += vx.z; else aabb.mMax.z += vx.z;
										if(IS_NEGATIVE_FLOAT(vy.x))	aabb.mMin.x += vy.x; else aabb.mMax.x += vy.x;
										if(IS_NEGATIVE_FLOAT(vy.y))	aabb.mMin.y += vy.y; else aabb.mMax.y += vy.y;
										if(IS_NEGATIVE_FLOAT(vy.z))	aabb.mMin.z += vy.z; else aabb.mMax.z += vy.z;
										if(IS_NEGATIVE_FLOAT(vz.x))	aabb.mMin.x += vz.x; else aabb.mMax.x += vz.x;
										if(IS_NEGATIVE_FLOAT(vz.y))	aabb.mMin.y += vz.y; else aabb.mMax.y += vz.y;
										if(IS_NEGATIVE_FLOAT(vz.z))	aabb.mMin.z += vz.z; else aabb.mMax.z += vz.z;
									}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Checks the AABB is valid.
		 *	\return		true if the box is valid
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			BOOL		IsValid()	const
									{
										// Consistency condition for (Min, Max) boxes: min < max
										if(mMin.x > mMax.x)	return FALSE;
										if(mMin.y > mMax.y)	return FALSE;
										if(mMin.z > mMax.z)	return FALSE;
										return TRUE;
									}

		//! Operator for AABB *= float. Scales the extents, keeps same center.
		inline_			AABB&		operator*=(float s)
									{
										Point Center;	GetCenter(Center);
										Point Extents;	GetExtents(Extents);
										SetCenterExtents(Center, Extents * s);
										return *this;
									}

		//! Operator for AABB /= float. Scales the extents, keeps same center.
		inline_			AABB&		operator/=(float s)
									{
										Point Center;	GetCenter(Center);
										Point Extents;	GetExtents(Extents);
										SetCenterExtents(Center, Extents / s);
										return *this;
									}

		//! Operator for AABB += Point. Translates the box.
		inline_			AABB&		operator+=(const Point& trans)
									{
										mMin+=trans;
										mMax+=trans;
										return *this;
									}
		private:
						Point		mMin;			//!< Min point
						Point		mMax;			//!< Max point
	};

#else

	class ICEMATHS_API AABB
	{
		public:
		//! Constructor
		inline_						AABB()	{}
		//! Destructor
		inline_						~AABB()	{}

		//! Type-independent methods
									AABB_COMMON_METHODS;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Setups an AABB from min & max vectors.
		 *	\param		min			[in] the min point
		 *	\param		max			[in] the max point
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						void		SetMinMax(const Point& min, const Point& max)		{ mCenter = (max + min)*0.5f; mExtents = (max - min)*0.5f;		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Setups an AABB from center & extents vectors.
		 *	\param		c			[in] the center point
		 *	\param		e			[in] the extents vector
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						void		SetCenterExtents(const Point& c, const Point& e)	{ mCenter = c;	 mExtents = e;									}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Setups an empty AABB.
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						void		SetEmpty()											{ mCenter.Zero(); mExtents.Set(MIN_FLOAT, MIN_FLOAT, MIN_FLOAT);}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Setups a point AABB.
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						void		SetPoint(const Point& pt)							{ mCenter = pt; mExtents.Zero();								}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Gets the size of the AABB. The size is defined as the longest extent.
		 *	\return		the size of the AABB
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						float		GetSize()								const		{ return mExtents.Max();					}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Extends the AABB.
		 *	\param		p	[in] the next point
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						void		Extend(const Point& p)
									{
										Point Max = mCenter + mExtents;
										Point Min = mCenter - mExtents;

										if(p.x > Max.x)	Max.x = p.x;
										if(p.x < Min.x)	Min.x = p.x;

										if(p.y > Max.y)	Max.y = p.y;
										if(p.y < Min.y)	Min.y = p.y;

										if(p.z > Max.z)	Max.z = p.z;
										if(p.z < Min.z)	Min.z = p.z;

										SetMinMax(Min, Max);
									}
		// Data access

		//! Get min point of the box
		inline_			void		GetMin(Point& min)						const		{ min = mCenter - mExtents;					}
		//! Get max point of the box
		inline_			void		GetMax(Point& max)						const		{ max = mCenter + mExtents;					}

		//! Get component of the box's min point along a given axis
		inline_			float		GetMin(udword axis)						const		{ return mCenter[axis] - mExtents[axis];	}
		//! Get component of the box's max point along a given axis
		inline_			float		GetMax(udword axis)						const		{ return mCenter[axis] + mExtents[axis];	}

		//! Get box center
		inline_			void		GetCenter(Point& center)				const		{ center = mCenter;							}
		//! Get box extents
		inline_			void		GetExtents(Point& extents)				const		{ extents = mExtents;						}

		//! Get component of the box's center along a given axis
		inline_			float		GetCenter(udword axis)					const		{ return mCenter[axis];						}
		//! Get component of the box's extents along a given axis
		inline_			float		GetExtents(udword axis)					const		{ return mExtents[axis];					}

		//! Get box diagonal
		inline_			void		GetDiagonal(Point& diagonal)			const		{ diagonal = mExtents * 2.0f;				}
		inline_			float		GetWidth()								const		{ return mExtents.x * 2.0f;					}
		inline_			float		GetHeight()								const		{ return mExtents.y * 2.0f;					}
		inline_			float		GetDepth()								const		{ return mExtents.z * 2.0f;					}

		//! Volume
		inline_			float		GetVolume()								const		{ return mExtents.x * mExtents.y * mExtents.z * 8.0f;	}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Computes the intersection between two AABBs.
		 *	\param		a		[in] the other AABB
		 *	\return		true on intersection
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			BOOL		Intersect(const AABB& a)				const
									{
										float tx = mCenter.x - a.mCenter.x;	float ex = a.mExtents.x + mExtents.x;	if(AIR(tx) > IR(ex))	return FALSE;
										float ty = mCenter.y - a.mCenter.y;	float ey = a.mExtents.y + mExtents.y;	if(AIR(ty) > IR(ey))	return FALSE;
										float tz = mCenter.z - a.mCenter.z;	float ez = a.mExtents.z + mExtents.z;	if(AIR(tz) > IR(ez))	return FALSE;
										return TRUE;
									}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	The standard intersection method from Gamasutra. Just here to check its speed against the one above.
		 *	\param		a		[in] the other AABB
		 *	\return		true on intersection
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			bool		GomezIntersect(const AABB& a)
									{
										Point	T = mCenter - a.mCenter;	// Vector from A to B
										return	((fabsf(T.x) <= (a.mExtents.x + mExtents.x))
												&& (fabsf(T.y) <= (a.mExtents.y + mExtents.y))
												&& (fabsf(T.z) <= (a.mExtents.z + mExtents.z)));
									}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Computes the 1D-intersection between two AABBs, on a given axis.
		 *	\param		a		[in] the other AABB
		 *	\param		axis	[in] the axis (0, 1, 2)
		 *	\return		true on intersection
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			BOOL		Intersect(const AABB& a, udword axis)	const
									{
										float t = mCenter[axis] - a.mCenter[axis];
										float e = a.mExtents[axis] + mExtents[axis];
										if(AIR(t) > IR(e))	return FALSE;
										return TRUE;
									}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Recomputes the AABB after an arbitrary transform by a 4x4 matrix.
		 *	\param		mtx			[in] the transform matrix
		 *	\param		aabb		[out] the transformed AABB [can be *this]
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			void		Rotate(const Matrix4x4& mtx, AABB& aabb)	const
									{
										// Compute new center
										aabb.mCenter = mCenter * mtx;

										// Compute new extents. FPU code & CPU code have been interleaved for improved performance.
										Point Ex(mtx.m[0][0] * mExtents.x, mtx.m[0][1] * mExtents.x, mtx.m[0][2] * mExtents.x);
										IR(Ex.x)&=0x7fffffff;	IR(Ex.y)&=0x7fffffff;	IR(Ex.z)&=0x7fffffff;

										Point Ey(mtx.m[1][0] * mExtents.y, mtx.m[1][1] * mExtents.y, mtx.m[1][2] * mExtents.y);
										IR(Ey.x)&=0x7fffffff;	IR(Ey.y)&=0x7fffffff;	IR(Ey.z)&=0x7fffffff;

										Point Ez(mtx.m[2][0] * mExtents.z, mtx.m[2][1] * mExtents.z, mtx.m[2][2] * mExtents.z);
										IR(Ez.x)&=0x7fffffff;	IR(Ez.y)&=0x7fffffff;	IR(Ez.z)&=0x7fffffff;

										aabb.mExtents.x = Ex.x + Ey.x + Ez.x;
										aabb.mExtents.y = Ex.y + Ey.y + Ez.y;
										aabb.mExtents.z = Ex.z + Ey.z + Ez.z;
									}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Checks the AABB is valid.
		 *	\return		true if the box is valid
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_			BOOL		IsValid()	const
									{
										// Consistency condition for (Center, Extents) boxes: Extents >= 0
										if(IS_NEGATIVE_FLOAT(mExtents.x))	return FALSE;
										if(IS_NEGATIVE_FLOAT(mExtents.y))	return FALSE;
										if(IS_NEGATIVE_FLOAT(mExtents.z))	return FALSE;
										return TRUE;
									}

		//! Operator for AABB *= float. Scales the extents, keeps same center.
		inline_			AABB&		operator*=(float s)		{ mExtents*=s;	return *this;	}

		//! Operator for AABB /= float. Scales the extents, keeps same center.
		inline_			AABB&		operator/=(float s)		{ mExtents/=s;	return *this;	}

		//! Operator for AABB += Point. Translates the box.
		inline_			AABB&		operator+=(const Point& trans)
									{
										mCenter+=trans;
										return *this;
									}
		private:
						Point		mCenter;			//!< AABB Center
						Point		mExtents;			//!< x, y and z extents
	};

#endif

	inline_ void ComputeMinMax(const Point& p, Point& min, Point& max)
	{
		if(p.x > max.x)	max.x = p.x;
		if(p.x < min.x)	min.x = p.x;

		if(p.y > max.y)	max.y = p.y;
		if(p.y < min.y)	min.y = p.y;

		if(p.z > max.z)	max.z = p.z;
		if(p.z < min.z)	min.z = p.z;
	}

	inline_ void ComputeAABB(AABB& aabb, const Point* list, udword nb_pts)
	{
		if(list)
		{
			Point Maxi(MIN_FLOAT, MIN_FLOAT, MIN_FLOAT);
			Point Mini(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
			while(nb_pts--)
			{
//				_prefetch(list+1);	// off by one ?
				ComputeMinMax(*list++, Mini, Maxi);
			}
			aabb.SetMinMax(Mini, Maxi);
		}
	}

#endif	// __ICEAABB_H__
