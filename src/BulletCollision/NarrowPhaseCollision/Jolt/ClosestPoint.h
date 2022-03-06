// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#if defined(JPH_USE_SSE)
	#include <immintrin.h>
	#include <smmintrin.h>
#elif defined(JPH_USE_NEON)
	#include <arm_neon.h>
#endif

namespace BTJPH {

class UVec4
{
	public:
			// Underlying vector type
#if defined(JPH_USE_SSE)
	using Type = __m128i;
#elif defined(JPH_USE_NEON)
	using Type = uint32x4_t;
#else
	using Type = jUint32a_t[4];
#endif

	union
	{
		Type					mValue;
		jUint32a_t					mU32[4];
	};
	


	inline					UVec4(int a,int b, int c, int d)
	{
		mU32[0] = a;
		mU32[1] = b;
		mU32[2] = c;
		mU32[3] = d;
	}

	#if defined(JPH_USE_SSE)
	inline					UVec4(Type inRHS) : mValue(inRHS)					{ }
	#elif defined(JPH_USE_NEON)
	inline					UVec4(Type inRHS) : mValue(inRHS)					{ }
	#else
	inline					UVec4(Type inRHS)
	{
		mU32[0] = inRHS[0];
		mU32[1] = inRHS[1];
		mU32[2] = inRHS[2];
		mU32[3] = inRHS[3];
	}

	#endif
	static UVec4 sReplicate(jUint32a_t inV)
	{

	#if defined(JPH_USE_SSE)
		return _mm_set1_epi32(int(inV));
	#elif defined(JPH_USE_NEON)
		return vdupq_n_u32(inV);
	#else
		return UVec4 (inV,inV,inV,inV);
	#endif
	}

		/// Get individual components
#if defined(JPH_USE_SSE)
	inline jUint32a_t			GetX() const										{ return (jUint32a_t)_mm_cvtsi128_si32(mValue); }
	inline jUint32a_t			GetY() const										{ return mU32[1]; }
	inline jUint32a_t			GetZ() const										{ return mU32[2]; }
	inline jUint32a_t			GetW() const										{ return mU32[3]; }
#elif defined(JPH_USE_NEON)
	inline jUint32a_t			GetX() const										{ return vgetq_lane_u32(mValue, 0); }
	inline jUint32a_t			GetY() const										{ return vgetq_lane_u32(mValue, 1); }
	inline jUint32a_t			GetZ() const										{ return vgetq_lane_u32(mValue, 2); }
	inline jUint32a_t			GetW() const										{ return vgetq_lane_u32(mValue, 3); }
#else
	inline jUint32a_t			GetX() const										{ return mU32[0]; }
	inline jUint32a_t			GetY() const										{ return mU32[1]; }
	inline jUint32a_t			GetZ() const										{ return mU32[2]; }
	inline jUint32a_t			GetW() const										{ return mU32[3]; }
#endif
};


class Vec4
{
	public:
	// Underlying vector type
#if defined(JPH_USE_SSE)
	using Type = __m128;
#elif defined(JPH_USE_NEON)
	using Type = float32x4_t;
#else
	using Type = btScalar[4];
#endif
	Type mValue;
	#if defined(JPH_USE_SSE)
	inline					Vec4(Type inRHS) : mValue(inRHS)					{ }
	#elif defined(JPH_USE_NEON)
	inline					Vec4(Type inRHS) : mValue(inRHS)					{ }
	#else
	inline					Vec4(Type inRHS)
	{
		mValue[0] = inRHS[0];
		mValue[1] = inRHS[1];
		mValue[2] = inRHS[2];
		mValue[3] = inRHS[3];
	}
	#endif
	inline Vec4(Vec3 inRHS, btScalar inW)
	{
	#if defined(JPH_USE_SSE)
		mValue = _mm_blend_ps(inRHS.mVec128, _mm_set1_ps(inW), 8);
	#elif defined(JPH_USE_NEON)
		mValue = vsetq_lane_f32(inW, inRHS.mValue, 3);
	#else
		mValue[0] = inRHS.x();
		mValue[1] = inRHS.y();
		mValue[2] = inRHS.z();
		mValue[3] = inRHS.w();
	#endif
	}

	Vec4(btScalar inX, btScalar inY, btScalar inZ, btScalar inW)
	{
	#if defined(JPH_USE_SSE)
		mValue = _mm_set_ps(inW, inZ, inY, inX);
	#elif defined(JPH_USE_NEON)
		uint32x2_t xy = vcreate_f32(static_cast<uint64>(*reinterpret_cast<jUint32a_t *>(&inX)) | (static_cast<uint64>(*reinterpret_cast<jUint32a_t *>(&inY)) << 32));
		uint32x2_t zw = vcreate_f32(static_cast<uint64>(*reinterpret_cast<jUint32a_t* >(&inZ)) | (static_cast<uint64>(*reinterpret_cast<jUint32a_t *>(&inW)) << 32));
		mValue = vcombine_f32(xy, zw);
	#else
		mValue[0] = inX;
		mValue[1] = inY;
		mValue[2] = inZ;
		mValue[3] = inW;
	#endif
	}

	static UVec4 sLess(Vec4 inV1, Vec4 inV2)
	{
	#if defined(JPH_USE_SSE)
		return _mm_castps_si128(_mm_cmplt_ps(inV1.mValue, inV2.mValue));
	#elif defined(JPH_USE_NEON)
		return vcltq_f32(inV1.mValue, inV2.mValue);
	#else
		return UVec4(inV1.mValue[0] < inV2.mValue[0],
					inV1.mValue[1] < inV2.mValue[1],
					inV1.mValue[2] < inV2.mValue[2],
					inV1.mValue[3] < inV2.mValue[3]);
	#endif
	}

	int GetSignBits() const
	{
	#if defined(JPH_USE_SSE)
		return _mm_movemask_ps(mValue);
	#elif defined(JPH_USE_NEON)
		int32x4_t shift = { 0, 1, 2, 3 };
		return vaddvq_u32(vshlq_u32(vshrq_n_u32(vreinterpretq_u32_f32(mValue), 31), shift));
	#else
		return (mValue[3]<0)<<3 | (mValue[2]<0)<<2 | (mValue[1]<0)<<1 | (mValue[0]<0);
	#endif
	}

	
	static UVec4 sGreaterOrEqual(Vec4 inV1, Vec4 inV2)
	{
	#if defined(JPH_USE_SSE)
		return _mm_castps_si128(_mm_cmpge_ps(inV1.mValue, inV2.mValue));
	#elif defined(JPH_USE_NEON)
		return vcgeq_f32(inV1.mValue, inV2.mValue);
	#else
		return UVec4(inV1.mValue[0] >= inV2.mValue[0],
					inV1.mValue[1] >= inV2.mValue[1],
					inV1.mValue[2] >= inV2.mValue[2],
					inV1.mValue[3] >= inV2.mValue[3]);
	#endif
	}

	static UVec4 sLessOrEqual(Vec4 inV1, Vec4 inV2)
	{
	#if defined(JPH_USE_SSE)
		return _mm_castps_si128(_mm_cmple_ps(inV1.mValue, inV2.mValue));
	#elif defined(JPH_USE_NEON)
		return vcleq_f32(inV1.mValue, inV2.mValue);
	#else
		return UVec4(inV1.mValue[0] <= inV2.mValue[0],
					inV1.mValue[1] <= inV2.mValue[1],
					inV1.mValue[2] <= inV2.mValue[2],
					inV1.mValue[3] <= inV2.mValue[3]);
	#endif
	}

	static Vec4 sReplicate(btScalar inV)
	{
	#if defined(JPH_USE_SSE)
		return _mm_set1_ps(inV);
	#elif defined(JPH_USE_NEON)
		return vdupq_n_f32(inV);
	#else
		return Vec4(inV,inV,inV,inV);
	#endif
	}
};


Vec4 DotV4(Vec3 inV1, Vec3 inV2)
{
#if defined(JPH_USE_SSE)
	return _mm_dp_ps(inV1.mVec128, inV2.mVec128, 0x7f);
#elif defined(JPH_USE_NEON)
    float32x4_t mul = vmulq_f32(mValue, inV2.mValue);
	mul = vsetq_lane_f32(0, mul, 3);
    return vdupq_n_f32(vaddvq_f32(mul));
#else
	return Vec4::sReplicate(inV1.dot(inV2));
#endif
}

#if defined (JPH_USE_SSE) || defined (JPH_USE_NEON)
Vec3 sSelect(Vec3 inV1, Vec3 inV2, UVec4 inControl)
{
#if defined(JPH_USE_SSE)
	auto v = _mm_blendv_ps(inV1.mVec128, inV2.mVec128, _mm_castsi128_ps(inControl.mValue));
	Vec3 res;
	res.set128(_mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 1, 0)));
	return res;
#elif defined(JPH_USE_NEON)
	Type v = vbslq_f32(vshrq_n_s32(inControl.mValue, 31), inV2.mValue, inV1.mValue);
	return __builtin_shufflevector(v, v, 0, 1, 2, 2);
#else
	#error Unsupported CPU architecture
#endif
}
#endif

// Turn off fused multiply add instruction because it makes the equations of the form a * b - c * d inaccurate below
//JPH_PRECISE_MATH_ON

/// Helper utils to find the closest point to a line segment, triangle or tetrahedron
namespace ClosestPoint
{
	/// Compute barycentric coordinates of closest point to origin for infinite line defined by (inA, inB)
	/// Point can then be computed as inA * outU + inB * outV
	inline void GetBaryCentricCoordinates(Vec3 inA, Vec3 inB, btScalar &outU, btScalar &outV)
	{
		Vec3 ab = inB - inA;
		btScalar denominator = ab.length2();
		if (denominator < (SIMD_EPSILON*SIMD_EPSILON))
		{
			// Degenerate line segment, fallback to points
			if (inA.length2() < inB.length2())
			{
				// A closest
				outU = 1.0f;
				outV = 0.0f;
			}
			else
			{
				// B closest
				outU = 0.0f;
				outV = 1.0f;
			}
		}
		else
		{
			outV = -inA.dot(ab) / denominator;
			outU = 1.0f - outV;
		}
	}
	
	/// Compute barycentric coordinates of closest point to origin for plane defined by (inA, inB, inC)
	/// Point can then be computed as inA * outU + inB * outV + inC * outW
	inline void GetBaryCentricCoordinates(Vec3 inA, Vec3 inB, Vec3 inC, btScalar &outU, btScalar &outV, btScalar &outW)
	{
		// Taken from: Real-Time Collision Detection - Christer Ericson (Section: Barycentric Coordinates)
		// With p = 0
		// Adjusted to always include the shortest edge of the triangle in the calculation to improve numerical accuracy

		// First calculate the three edges
		Vec3 v0 = inB - inA;
		Vec3 v1 = inC - inA;
		Vec3 v2 = inC - inB;

		// Make sure that the shortest edge is included in the calculation to keep the products a * b - c * d as small as possible to preserve accuracy
		btScalar d00 = v0.dot(v0); 
		btScalar d11 = v1.dot(v1); 
		btScalar d22 = v2.dot(v2);
		if (d00 <= d22)
		{
			// Use v0 and v1 to calculate barycentric coordinates
			btScalar d01 = v0.dot(v1); 
		
			btScalar denominator = d00 * d11 - d01 * d01; 
			if (abs(denominator) < SIMD_EPSILON)
			{
				// Degenerate triangle, return coordinates along longest edge
				if (d00 > d11)
				{
					GetBaryCentricCoordinates(inA, inB, outU, outV);
					outW = 0.0f;
				}
				else
				{
					GetBaryCentricCoordinates(inA, inC, outU, outW);
					outV = 0.0f;
				}
			}
			else
			{
				btScalar a0 = inA.dot(v0);
				btScalar a1 = inA.dot(v1); 
				outV = (d01 * a1 - d11 * a0) / denominator; 
				outW = (d01 * a0 - d00 * a1) / denominator; 
				outU = 1.0f - outV - outW;
			}
		}
		else
		{
			// Use v1 and v2 to calculate barycentric coordinates
			btScalar d12 = v1.dot(v2); 
		
			btScalar denominator = d11 * d22 - d12 * d12; 
			if (abs(denominator) < SIMD_EPSILON)
			{
				// Degenerate triangle, return coordinates along longest edge
				if (d11 > d22)
				{
					GetBaryCentricCoordinates(inA, inC, outU, outW);
					outV = 0.0f;
				}
				else
				{
					GetBaryCentricCoordinates(inB, inC, outV, outW);
					outU = 0.0f;
				}
			}
			else
			{
				btScalar c1 = inC.dot(v1);
				btScalar c2 = inC.dot(v2); 
				outU = (d22 * c1 - d12 * c2) / denominator; 
				outV = (d11 * c2 - d12 * c1) / denominator; 
				outW = 1.0f - outU - outV;
			}
		}
	}

	/// Get the closest point to the origin of line (inA, inB)
	/// outSet describes which features are closest: 1 = a, 2 = b, 3 = line segment ab
	inline Vec3	GetClosestPointOnLine(Vec3 inA, Vec3 inB, jUint32a_t &outSet) 
	{
		btScalar u, v;
		GetBaryCentricCoordinates(inA, inB, u, v);
		if (v <= 0.0f)
		{
			// inA is closest point
			outSet = 0b0001;
			return inA;
		}
		else if (u <= 0.0f)
		{
			// inB is closest point
			outSet = 0b0010;
			return inB;
		}
		else
		{
			// Closest point lies on line inA inB
			outSet = 0b0011;
			return u * inA + v * inB;
		}
	}

	/// Get the closest point to the origin of triangle (inA, inB, inC)
	/// outSet describes which features are closest: 1 = a, 2 = b, 4 = c, 5 = line segment ac, 7 = triangle interior etc.
	inline Vec3	GetClosestPointOnTriangle(Vec3 inA, Vec3 inB, Vec3 inC, jUint32a_t &outSet)
	{
		// Taken from: Real-Time Collision Detection - Christer Ericson (Section: Closest Point on Triangle to Point)
		// With p = 0

		// Calculate edges
		Vec3 ab = inB - inA; 
		Vec3 ac = inC - inA; 
		Vec3 bc = inC - inB;

		// The most accurate normal is calculated by using the two shortest edges
		// See: https://box2d.org/posts/2014/01/troublesome-triangle/
		// The difference in normals is most pronounced when one edge is much smaller than the others (in which case the other 2 must have roughly the same length).
		// Therefore we can suffice by just picking the shortest from 2 edges and use that with the 3rd edge to calculate the normal.
		// We first check which of the edges is shorter
#if defined (JPH_USE_SSE) || defined (JPH_USE_NEON)
		UVec4 bc_shorter_than_ac = Vec4::sLess(DotV4(bc,bc), DotV4(ac,ac));
		// We calculate both normals and then select the one that had the shortest edge for our normal (this avoids branching)
		Vec3 normal_bc = ab.cross(bc);
		Vec3 normal_ac = ab.cross(ac);
		Vec3 n = sSelect(normal_ac, normal_bc, bc_shorter_than_ac);
#else
		Vec3 normal_bc = ab.cross(bc);
		Vec3 normal_ac = ab.cross(ac);
		btScalar bcbc = bc.dot(bc);
		btScalar acac = bc.dot(bc);
		Vec3 n = bcbc<acac? normal_ac : normal_bc;
#endif
		btScalar n_len_sq = n.length2();

		// Check degenerate
		if (n_len_sq < (SIMD_EPSILON*SIMD_EPSILON))
		{
			// Degenerate, fallback to edges

			// Edge AB
			jUint32a_t closest_set;
			Vec3 closest_point = GetClosestPointOnLine(inA, inB, closest_set);
			btScalar best_dist_sq = closest_point.length2();

			// Edge AC
			jUint32a_t set;
			Vec3 q = GetClosestPointOnLine(inA, inC, set);
			btScalar dist_sq = q.length2();
			if (dist_sq < best_dist_sq)
			{
				closest_point = q;
				best_dist_sq = dist_sq;
				closest_set = (set & 0b0001) + ((set & 0b0010) << 1);
			}

			// Edge BC
			q = GetClosestPointOnLine(inB, inC, set);
			dist_sq = q.length2();
			if (dist_sq < best_dist_sq)
			{
				closest_point = q;
				best_dist_sq = dist_sq;
				closest_set = set << 1;
			}

			outSet = closest_set;
			return closest_point;
		}

		// Check if P in vertex region outside A 
		Vec3 ap = -inA; 
		btScalar d1 = ab.dot(ap); 
		btScalar d2 = ac.dot(ap); 
		if (d1 <= 0.0f && d2 <= 0.0f)
		{
			outSet = 0b0001;
			return inA; // barycentric coordinates (1,0,0)
		}

		// Check if P in vertex region outside B 
		Vec3 bp = -inB; 
		btScalar d3 = ab.dot(bp); 
		btScalar d4 = ac.dot(bp); 
		if (d3 >= 0.0f && d4 <= d3) 
		{
			outSet = 0b0010;
			return inB; // barycentric coordinates (0,1,0)
		}

		// Check if P in edge region of AB, if so return projection of P onto AB 
		btScalar vc = d1 * d4 - d3 * d2; 
		if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) 
		{ 
			btScalar v = d1 / (d1 - d3); 
			outSet = 0b0011;
			return inA + v * ab; // barycentric coordinates (1-v,v,0) 
		}

		// Check if P in vertex region outside C 
		Vec3 cp = -inC; 
		btScalar d5 = ab.dot(cp); 
		btScalar d6 = ac.dot(cp); 
		if (d6 >= 0.0f && d5 <= d6) 
		{
			outSet = 0b0100;
			return inC; // barycentric coordinates (0,0,1)
		}

		// Check if P in edge region of AC, if so return projection of P onto AC 
		btScalar vb = d5 * d2 - d1 * d6; 
		if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) 
		{ 
			btScalar w = d2 / (d2 - d6); 
			outSet = 0b0101;
			return inA + w * ac; // barycentric coordinates (1-w,0,w) 
		}

		// Check if P in edge region of BC, if so return projection of P onto BC 
		btScalar va = d3 * d6 - d5 * d4;
		btScalar d4_d3 = d4 - d3;
		btScalar d5_d6 = d5 - d6;
		if (va <= 0.0f && d4_d3 >= 0.0f && d5_d6 >= 0.0f) 
		{ 
			btScalar w = d4_d3 / (d4_d3 + d5_d6); 
			outSet = 0b0110;
			return inB + w * bc; // barycentric coordinates (0,1-w,w) 
		}

		// P inside face region.
		// Here we deviate from Christer Ericson's article to improve accuracy.
		// Determine distance between triangle and origin: distance = (centroid - origin) . normal / |normal|
		// Closest point to origin is then: distance . normal / |normal|
		// Note that this way of calculating the closest point is much more accurate than first calculating barycentric coordinates 
		// and then calculating the closest point based on those coordinates.
		outSet = 0b0111;
		return n * (inA + inB + inC).dot(n) / (3.0f * n_len_sq);
	}

	/// Check if the origin is outside the plane of triangle (inA, inB, inC). inD specifies the front side of the plane.
	inline bool OriginOutsideOfPlane(Vec3 inA, Vec3 inB, Vec3 inC, Vec3 inD)
	{
		// Taken from: Real-Time Collision Detection - Christer Ericson (Section: Closest Point on Tetrahedron to Point)
		// With p = 0

		// Test if point p and d lie on opposite sides of plane through abc 
		Vec3 n = (inB - inA).cross(inC - inA);
		btScalar signp = inA.dot(n); // [AP AB AC]
		btScalar signd = (inD - inA).dot(n); // [AD AB AC] 
													   
		// Points on opposite sides if expression signs are the same
		// Note that we left out the minus sign in signp so we need to check > 0 instead of < 0 as in Christer's book
		// We compare against a small negative value to allow for a little bit of slop in the calculations
		return signp * signd > -SIMD_EPSILON; 
	}

	/// Returns for each of the planes of the tetrahedron if the origin is inside it
	/// Roughly equivalent to: 
	///	[OriginOutsideOfPlane(inA, inB, inC, inD), 
	///	 OriginOutsideOfPlane(inA, inC, inD, inB), 
	///	 OriginOutsideOfPlane(inA, inD, inB, inC), 
	///	 OriginOutsideOfPlane(inB, inD, inC, inA)]
	inline UVec4 OriginOutsideOfTetrahedronPlanes(Vec3 inA, Vec3 inB, Vec3 inC, Vec3 inD)
	{
		Vec3 ab = inB - inA;
		Vec3 ac = inC - inA;
		Vec3 ad = inD - inA;
		Vec3 bd = inD - inB;
		Vec3 bc = inC - inB;

		Vec3 ab_cross_ac = ab.cross(ac);
		Vec3 ac_cross_ad = ac.cross(ad);
		Vec3 ad_cross_ab = ad.cross(ab);
		Vec3 bd_cross_bc = bd.cross(bc);
		
		// For each plane get the side on which the origin is
		btScalar signp0 = inA.dot(ab_cross_ac); // ABC
		btScalar signp1 = inA.dot(ac_cross_ad); // ACD
		btScalar signp2 = inA.dot(ad_cross_ab); // ADB
		btScalar signp3 = inB.dot(bd_cross_bc); // BDC
		Vec4 signp(signp0, signp1, signp2, signp3);

		// For each plane get the side that is outside (determined by the 4th point)
		btScalar signd0 = ad.dot(ab_cross_ac);  // D
		btScalar signd1 = ab.dot(ac_cross_ad);  // B
		btScalar signd2 = ac.dot(ad_cross_ab);  // C
		btScalar signd3 = -ab.dot(bd_cross_bc); // A
		Vec4 signd(signd0, signd1, signd2, signd3);

		// The winding of all triangles has been chosen so that signd should have the
		// same sign for all components. If this is not the case the tetrahedron
		// is degenerate and we return that the origin is in front of all sides
		int sign_bits = signd.GetSignBits();
		switch (sign_bits)
		{
		case 0:
			// All positive
			return Vec4::sGreaterOrEqual(signp, Vec4::sReplicate(-SIMD_EPSILON));

		case 0xf:
			// All negative
			return Vec4::sLessOrEqual(signp, Vec4::sReplicate(SIMD_EPSILON));

		default:
			// Mixed signs, degenerate tetrahedron
			return UVec4::sReplicate(0xffffffff);
		}
	}

	/// Get the closest point between tetrahedron (inA, inB, inC, inD) to the origin
	/// outSet specifies which feature was closest, 1 = a, 2 = b, 4 = c, 8 = d. Edges have 2 bits set, triangles 3 and if the point is in the interior 4 bits are set.
	inline Vec3	GetClosestPointOnTetrahedron(Vec3 inA, Vec3 inB, Vec3 inC, Vec3 inD, jUint32a_t &outSet)
	{
		// Taken from: Real-Time Collision Detection - Christer Ericson (Section: Closest Point on Tetrahedron to Point)
		// With p = 0

		// Start out assuming point inside all halfspaces, so closest to itself 
		jUint32a_t closest_set = 0b1111;
		Vec3 closest_point(0,0,0);
		btScalar best_dist_sq = SIMD_INFINITY; 
		
		// Determine for each of the faces of the tetrahedron if the origin is in front of the plane
		UVec4 origin_out_of_planes = OriginOutsideOfTetrahedronPlanes(inA, inB, inC, inD);

		// If point outside face abc then compute closest point on abc 
		if (origin_out_of_planes.GetX()) // OriginOutsideOfPlane(inA, inB, inC, inD)
		{ 
			Vec3 q = GetClosestPointOnTriangle(inA, inB, inC, closest_set); 
			btScalar dist_sq = q.length2(); 
			
			// Update best closest point if (squared) distance is less than current best 
			if (dist_sq < best_dist_sq) 
			{
				best_dist_sq = dist_sq;
				closest_point = q; 
			}
		} 
		
		// Repeat test for face acd 
		if (origin_out_of_planes.GetY()) // OriginOutsideOfPlane(inA, inC, inD, inB)
		{ 
			jUint32a_t set;
			Vec3 q = GetClosestPointOnTriangle(inA, inC, inD, set); 
			btScalar dist_sq = q.length2(); 
			if (dist_sq < best_dist_sq) 
			{
				best_dist_sq = dist_sq;
				closest_point = q;
				closest_set = (set & 0b0001) + ((set & 0b0110) << 1);
			}
		}

		// Repeat test for face adb 
		if (origin_out_of_planes.GetZ()) // OriginOutsideOfPlane(inA, inD, inB, inC)
		{
			jUint32a_t set;
			Vec3 q = GetClosestPointOnTriangle(inA, inD, inB, set); 
			btScalar dist_sq = q.length2(); 
			if (dist_sq < best_dist_sq) 
			{
				best_dist_sq = dist_sq;
				closest_point = q;
				closest_set = (set & 0b0001) + ((set & 0b0010) << 2) + ((set & 0b0100) >> 1); 
			}
		} 
		
		// Repeat test for face bdc 
		if (origin_out_of_planes.GetW()) // OriginOutsideOfPlane(inB, inD, inC, inA)
		{ 
			jUint32a_t set;
			Vec3 q = GetClosestPointOnTriangle(inB, inD, inC, set); 
			btScalar dist_sq = q.length2(); 
			if (dist_sq < best_dist_sq) 
			{
				best_dist_sq = dist_sq;
				closest_point = q;
				closest_set = ((set & 0b0001) << 1) + ((set & 0b0010) << 2) + (set & 0b0100); 
			}
		} 
	
		outSet = closest_set;
		return closest_point;
	}
};

//JPH_PRECISE_MATH_OFF

} // BTJPH
