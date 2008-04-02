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
 *	Contains code for planes.
 *	\file		IcePlane.h
 *	\author		Pierre Terdiman
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __ICEPLANE_H__
#define __ICEPLANE_H__

	#define PLANE_EPSILON		(1.0e-7f)

	class ICEMATHS_API Plane
	{
		public:
		//! Constructor
		inline_			Plane()															{												}
		//! Constructor from a normal and a distance
		inline_			Plane(float nx, float ny, float nz, float d)					{ Set(nx, ny, nz, d);							}
		//! Constructor from a point on the plane and a normal
		inline_			Plane(const Point& p, const Point& n)							{ Set(p, n);									}
		//! Constructor from three points
		inline_			Plane(const Point& p0, const Point& p1, const Point& p2)		{ Set(p0, p1, p2);								}
		//! Constructor from a normal and a distance
		inline_			Plane(const Point& _n, float _d)								{ n = _n; d = _d;								}
		//! Copy constructor
		inline_			Plane(const Plane& plane) : n(plane.n), d(plane.d)				{												}
		//! Destructor
		inline_			~Plane()														{												}

		inline_	Plane&	Zero()															{ n.Zero(); d = 0.0f;			return *this;	}
		inline_	Plane&	Set(float nx, float ny, float nz, float _d)						{ n.Set(nx, ny, nz); d = _d;	return *this;	}
		inline_	Plane&	Set(const Point& p, const Point& _n)							{ n = _n; d = - p | _n;			return *this;	}
				Plane&	Set(const Point& p0, const Point& p1, const Point& p2);

		inline_	float	Distance(const Point& p)			const						{ return (p | n) + d;							}
		inline_	bool	Belongs(const Point& p)				const						{ return fabsf(Distance(p)) < PLANE_EPSILON;	}

		inline_	void	Normalize()
						{
							float Denom = 1.0f / n.Magnitude();
							n.x	*= Denom;
							n.y	*= Denom;
							n.z	*= Denom;
							d	*= Denom;
						}
		public:
		// Members
				Point	n;		//!< The normal to the plane
				float	d;		//!< The distance from the origin

		// Cast operators
		inline_			operator Point()					const						{ return n;										}
		inline_			operator HPoint()					const						{ return HPoint(n, d);							}

		// Arithmetic operators
		inline_	Plane	operator*(const Matrix4x4& m)		const
						{
							// Old code from Irion. Kept for reference.
							Plane Ret(*this);
							return Ret *= m;
						}

		inline_	Plane&	operator*=(const Matrix4x4& m)
						{
							// Old code from Irion. Kept for reference.
							Point n2 = HPoint(n, 0.0f) * m;
							d = -((Point) (HPoint( -d*n, 1.0f ) * m) | n2);
							n = n2;
							return *this;
						}
	};

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	Transforms a plane by a 4x4 matrix. Same as Plane * Matrix4x4 operator, but faster.
	 *	\param		transformed	[out] transformed plane
	 *	\param		plane		[in] source plane
	 *	\param		transform	[in] transform matrix
	 *	\warning	the plane normal must be unit-length
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	inline_	void TransformPlane(Plane& transformed, const Plane& plane, const Matrix4x4& transform)
	{
		// Rotate the normal using the rotation part of the 4x4 matrix
		transformed.n = plane.n * Matrix3x3(transform);

		// Compute new d
		transformed.d = plane.d - (Point(transform.GetTrans())|transformed.n);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	Transforms a plane by a 4x4 matrix. Same as Plane * Matrix4x4 operator, but faster.
	 *	\param		plane		[in/out] source plane (transformed on return)
	 *	\param		transform	[in] transform matrix
	 *	\warning	the plane normal must be unit-length
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	inline_	void TransformPlane(Plane& plane, const Matrix4x4& transform)
	{
		// Rotate the normal using the rotation part of the 4x4 matrix
		plane.n *= Matrix3x3(transform);

		// Compute new d
		plane.d -= Point(transform.GetTrans())|plane.n;
	}

#endif // __ICEPLANE_H__
