// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT
#ifndef __JOLT_GJK_CLOSEST_POINT_H
#define __JOLT_GJK_CLOSEST_POINT_H

#pragma once

#include "NonCopyable.h"
#include "ClosestPoint.h"
#include <string.h>
#include "LinearMath/btMinMax.h"
#include <assert.h>

//#define JPH_GJK_DEBUG
#ifdef JPH_GJK_DEBUG
	#include <Core/StringTools.h>
	#include <Renderer/DebugRenderer.h>
#endif

namespace BTJPH {

/// Convex vs convex collision detection
/// Based on: A Fast and Robust GJK Implementation for Collision Detection of Convex Objects - Gino van den Bergen
class GJKClosestPoint : public NonCopyable
{
private:
	/// Get new closest point to origin given simplex mY of mNumPoints points
	///
	/// @param inPrevVLenSq Length of |outV|^2 from the previous iteration, used as a maximum value when selecting a new closest point.
	/// @param outV Closest point
	/// @param outVLenSq |outV|^2
	/// @param outSet Set of points that form the new simplex closest to the origin (bit 1 = mY[0], bit 2 = mY[1], ...)
	///
	/// @return True if new closest point was found.
	/// False if the function failed, in this case the output variables are not modified
	bool		GetClosest(btScalar inPrevVLenSq, Vec3 &outV, btScalar &outVLenSq, jUint32a_t &outSet) const
	{
#ifdef JPH_GJK_DEBUG
		for (int i = 0; i < mNumPoints; ++i)
			Trace("y[%d] = [%s], |y[%d]| = %g", i, ConvertToString(mY[i]).c_str(), i, mY[i].Length());
#endif

		jUint32a_t set;
		Vec3 v;

		switch (mNumPoints)
		{
		case 1:
			// Single point
			set = 0b0001;
			v = mY[0];
			break;

		case 2:
			// Line segment
			v = ClosestPoint::GetClosestPointOnLine(mY[0], mY[1], set);
			break;

		case 3:
			// Triangle
			v = ClosestPoint::GetClosestPointOnTriangle(mY[0], mY[1], mY[2], set);
			break;

		case 4:
			// Tetrahedron
			v = ClosestPoint::GetClosestPointOnTetrahedron(mY[0], mY[1], mY[2], mY[3], set);
			break;

		default:
			assert(false);
			return false;
		}

#ifdef JPH_GJK_DEBUG
 		Trace("GetClosest: set = 0b%s, v = [%s], |v| = %g", NibbleToBinary(set), ConvertToString(v).c_str(), v.Length());
#endif

		btScalar v_len_sq = v.length2();
		if (v_len_sq >= inPrevVLenSq)
		{
			// No better match found
#ifdef JPH_GJK_DEBUG
			Trace("New closer point is further away, failed to converge");
#endif
			return false;
		}

		// Return closest point
		outV = v;
		outVLenSq = v_len_sq;
		outSet = set;
		return true;
	}

	// Get max(|Y_0|^2 .. |Y_n|^2)
	btScalar		GetMaxYLengthSq() const
	{
		btScalar y_len_sq = mY[0].length2();
		for (int i = 1; i < mNumPoints; ++i)
			y_len_sq = btMax(y_len_sq, mY[i].length2());
		return y_len_sq;
	}

	// Remove points that are not in the set, only updates mY
	void		UpdatePointSetY(jUint32a_t inSet)
	{
		int num_points = 0;
		for (int i = 0; i < mNumPoints; ++i)
			if ((inSet & (1 << i)) != 0)
			{
				mY[num_points] = mY[i];
				++num_points;
			}
		mNumPoints = num_points;
	}

	// Remove points that are not in the set, only updates mP
	void		UpdatePointSetP(jUint32a_t inSet)
	{
		int num_points = 0;
		for (int i = 0; i < mNumPoints; ++i)
			if ((inSet & (1 << i)) != 0)
			{
				mP[num_points] = mP[i];
				++num_points;
			}
		mNumPoints = num_points;
	}

	// Remove points that are not in the set, only updates mP and mQ
	void		UpdatePointSetPQ(jUint32a_t inSet)
	{
		int num_points = 0;
		for (int i = 0; i < mNumPoints; ++i)
			if ((inSet & (1 << i)) != 0)
			{
				mP[num_points] = mP[i];
				mQ[num_points] = mQ[i];
				++num_points;
			}
		mNumPoints = num_points;
	}

	// Remove points that are not in the set, updates mY, mP and mQ
	void		UpdatePointSetYPQ(jUint32a_t inSet)
	{
		int num_points = 0;
		for (int i = 0; i < mNumPoints; ++i)
			if ((inSet & (1 << i)) != 0)
			{
				mY[num_points] = mY[i];
				mP[num_points] = mP[i];
				mQ[num_points] = mQ[i];
				++num_points;
			}
		mNumPoints = num_points;
	}

	// Calculate closest points on A and B
	void		CalculatePointAAndB(Vec3 &outPointA, Vec3 &outPointB)
	{
		switch (mNumPoints)		
		{
		case 1:
			outPointA = mP[0]; 
			outPointB = mQ[0];
			break;

		case 2:
			{
				btScalar u, v;
				ClosestPoint::GetBaryCentricCoordinates(mY[0], mY[1], u, v);
				outPointA = u * mP[0] + v * mP[1];
				outPointB = u * mQ[0] + v * mQ[1];
			}
			break;

		case 3:
			{
				btScalar u, v, w;
				ClosestPoint::GetBaryCentricCoordinates(mY[0], mY[1], mY[2], u, v, w);
				outPointA = u * mP[0] + v * mP[1] + w * mP[2];
				outPointB = u * mQ[0] + v * mQ[1] + w * mQ[2];
			}
			break;

		case 4:
		#ifdef _DEBUG
			memset(&outPointA, 0xcd, sizeof(outPointA));
			memset(&outPointB, 0xcd, sizeof(outPointB));
		#endif
			break;
		}
	}
	
public:
	/// Test if inA and inB intersect
	///
	/// @param inA The convex object A, must support the GetSupport(Vec3) function.
	/// @param inB The convex object B, must support the GetSupport(Vec3) function.
	///	@param inTolerance Minimal distance between objects when the objects are considered to be colliding
	///	@param ioV is used as initial separating axis (provide a zero vector if you don't know yet)
	///
	///	@return True if they intersect (in which case ioV = (0, 0, 0)).
	///	False if they don't intersect in which case ioV is a separating axis in the direction from A to B (magnitude is meaningless)
	template <typename A, typename B>
	bool		Intersects(const A &inA, const B &inB, btScalar inTolerance, Vec3 &ioV)
	{
		btScalar tolerance_sq = Square(inTolerance);

		// Reset state
		mNumPoints = 0;

#ifdef JPH_GJK_DEBUG
		for (int i = 0; i < 4; ++i)
			mY[i] = Vec3::sZero();
#endif

		// Previous length^2 of v
		btScalar prev_v_len_sq = FLT_MAX;

		for (;;)
		{
#ifdef JPH_GJK_DEBUG
			Trace("v = [%s], num_points = %d", ConvertToString(ioV).c_str(), mNumPoints);
#endif

			// Get support points for shape A and B in the direction of v
			Vec3 p = inA.GetSupport(ioV);
			Vec3 q = inB.GetSupport(-ioV);

			// Get support point of the minkowski sum A - B of v
			Vec3 w = p - q;

			// If the support point sA-B(v) is in the opposite direction as v, then we have found a separating axis and there is no intersection
			if (ioV.dot(w) < 0.0f)
			{
				// Separating axis found
#ifdef JPH_GJK_DEBUG
				Trace("Seperating axis");
#endif
				return false;
			}

			// Store the point for later use
			mY[mNumPoints] = w;
			++mNumPoints;

#ifdef JPH_GJK_DEBUG
			Trace("w = [%s]", ConvertToString(w).c_str());
#endif

			// Determine the new closest point
			btScalar v_len_sq;			// Length^2 of v
			jUint32a_t set;				// Set of points that form the new simplex
			if (!GetClosest(prev_v_len_sq, ioV, v_len_sq, set))
				return false;

			// If there are 4 points, the origin is inside the tetrahedron and we're done
			if (set == 0xf)
			{
#ifdef JPH_GJK_DEBUG
				Trace("Full simplex");
#endif
				ioV = Vec3::sZero();
				return true;
			}

			// If v is very close to zero, we consider this a collision
			if (v_len_sq <= tolerance_sq)
			{
#ifdef JPH_GJK_DEBUG
				Trace("Distance zero");
#endif
				ioV = Vec3::sZero();
				return true;
			}

			// If v is very small compared to the length of y, we also consider this a collision
			if (v_len_sq <= FLT_EPSILON * GetMaxYLengthSq())
			{
#ifdef JPH_GJK_DEBUG
				Trace("Machine precision reached");
#endif
				ioV = Vec3::sZero();
				return true;
			}

			// The next seperation axis to test is the negative of the closest point of the Minkowski sum to the origin
			// Note: This must be done before terminating as converged since the separating axis is -v
			ioV = -ioV;

			// If the squared length of v is not changing enough, we've converged and there is no collision
			assert(prev_v_len_sq >= v_len_sq);
			if (prev_v_len_sq - v_len_sq <= FLT_EPSILON * prev_v_len_sq)
			{
				// v is a separating axis
#ifdef JPH_GJK_DEBUG
				Trace("Converged");
#endif
				return false;
			}
			prev_v_len_sq = v_len_sq;

			// Update the points of the simplex
			UpdatePointSetY(set);
		}
	}

	/// Get closest points between inA and inB
	///
	/// @param inA The convex object A, must support the GetSupport(Vec3) function.
	/// @param inB The convex object B, must support the GetSupport(Vec3) function.
	///	@param inTolerance The minimal distance between A and B before the objects are considered colliding and processing is terminated.
	///	@param inMaxDistSq The maximum squared distance between A and B before the objects are considered infinitely far away and processing is terminated.
	///	@param ioV Initial guess for the separating axis. Start with any non-zero vector if you don't know.
	///		If return value is 0, ioV = (0, 0, 0).
	///		If the return value is bigger than 0 but smaller than FLT_MAX, ioV will be the separating axis in the direction from A to B and its length the squared distance between A and B.
	///		If the return value is FLT_MAX, ioV will be the separating axis in the direction from A to B and the magnitude of the vector is meaningless.
	///	@param outPointA , outPointB
	///		If the return value is 0 the points are invalid.
	///		If the return value is bigger than 0 but smaller than FLT_MAX these will contain the closest point on A and B.
	///		If the return value is FLT_MAX the points are invalid.
	///
	///	@return The squared distance between A and B or FLT_MAX when they are further away than inMaxDistSq.
	template <typename A, typename B>
	btScalar		GetClosestPoints(const A &inA, const B &inB, btScalar inTolerance, btScalar inMaxDistSq, Vec3 &ioV, Vec3 &outPointA, Vec3 &outPointB)
	{
		btScalar tolerance_sq = inTolerance*inTolerance;

		// Reset state
		mNumPoints = 0;

#ifdef JPH_GJK_DEBUG
		// Generate the hull of the Minkowski difference for visualization
		MinkowskiDifference<A, B> diff(inA, inB);
		mGeometry = DebugRenderer::sInstance->CreateTriangleGeometryForConvex([&diff](Vec3Arg inDirection) { return diff.GetSupport(inDirection); });

		for (int i = 0; i < 4; ++i)
		{
			mY[i] = Vec3::sZero();
			mP[i] = Vec3::sZero();
			mQ[i] = Vec3::sZero();
		}
#endif

		// Length^2 of v
		btScalar v_len_sq = ioV.length2();

		// Previous length^2 of v
		btScalar prev_v_len_sq = SIMD_INFINITY; //FLT_MAX or DBL_MAX

		for (;;)
		{
#ifdef JPH_GJK_DEBUG
			Trace("v = [%s], num_points = %d", ConvertToString(ioV).c_str(), mNumPoints);
#endif

			// Get support points for shape A and B in the direction of v
			Vec3 p = inA.GetSupport(ioV);
			Vec3 q = inB.GetSupport(-ioV);

			// Get support point of the minkowski sum A - B of v
			Vec3 w = p - q;

			btScalar dot = ioV.dot(w);

#ifdef JPH_GJK_DEBUG
			// Draw -ioV to show the closest point to the origin from the previous simplex
			DebugRenderer::sInstance->DrawArrow(mOffset, mOffset - ioV, Color::sOrange, 0.05f);

			// Draw ioV to show where we're probing next
			DebugRenderer::sInstance->DrawArrow(mOffset, mOffset + ioV, Color::sCyan, 0.05f);

			// Draw w, the support point
			DebugRenderer::sInstance->DrawArrow(mOffset, mOffset + w, Color::sGreen, 0.05f);
			DebugRenderer::sInstance->DrawMarker(mOffset + w, Color::sGreen, 1.0f);

			// Draw the simplex and the Minkowski difference around it
			DrawState();
#endif

			// Test if we have a separation of more than inMaxDistSq, in which case we terminate early
			if (dot < 0.0f && dot * dot > v_len_sq * inMaxDistSq)
			{
#ifdef JPH_GJK_DEBUG
				Trace("Distance bigger than max");
#endif
#ifdef _DEBUG
				memset(&outPointA, 0xcd, sizeof(outPointA));
				memset(&outPointB, 0xcd, sizeof(outPointB));
#endif
				return SIMD_INFINITY;
			}

			// Store the point for later use
			mY[mNumPoints] = w;
			mP[mNumPoints] = p;
			mQ[mNumPoints] = q;
			++mNumPoints;

#ifdef JPH_GJK_DEBUG
			Trace("w = [%s]", ConvertToString(w).c_str());
#endif

			jUint32a_t set;
			if (!GetClosest(prev_v_len_sq, ioV, v_len_sq, set))
			{
				--mNumPoints; // Undo add last point
				break;
			}

			// If there are 4 points, the origin is inside the tetrahedron and we're done
			if (set == 0xf)
			{
#ifdef JPH_GJK_DEBUG
				Trace("Full simplex");
#endif
				ioV = Vec3::sZero();
				v_len_sq = 0.0f;
				break;
			}

			// Update the points of the simplex
			UpdatePointSetYPQ(set);

			// If v is very close to zero, we consider this a collision
			if (v_len_sq <= tolerance_sq)
			{
#ifdef JPH_GJK_DEBUG
				Trace("Distance zero");
#endif
				ioV = Vec3::sZero();
				v_len_sq = 0.0f;
				break;
			}

			// If v is very small compared to the length of y, we also consider this a collision
#ifdef JPH_GJK_DEBUG
			Trace("Check v small compared to y: %g <= %g", v_len_sq, FLT_EPSILON * GetMaxYLengthSq());
#endif
			if (v_len_sq <= FLT_EPSILON * GetMaxYLengthSq())
			{
#ifdef JPH_GJK_DEBUG
				Trace("Machine precision reached");
#endif
				ioV = Vec3::sZero();
				v_len_sq = 0.0f;
				break;
			}

			// The next seperation axis to test is the negative of the closest point of the Minkowski sum to the origin
			// Note: This must be done before terminating as converged since the separating axis is -v
			ioV = -ioV;

			// If the squared length of v is not changing enough, we've converged and there is no collision
#ifdef JPH_GJK_DEBUG
			Trace("Check v not changing enough: %g <= %g", prev_v_len_sq - v_len_sq, FLT_EPSILON * prev_v_len_sq);
#endif
			assert(prev_v_len_sq >= v_len_sq);
			if (prev_v_len_sq - v_len_sq <= FLT_EPSILON * prev_v_len_sq)
			{
				// v is a separating axis
#ifdef JPH_GJK_DEBUG
				Trace("Converged");
#endif
				break;
			}
			prev_v_len_sq = v_len_sq;
		}

		// Get the closest points
		CalculatePointAAndB(outPointA, outPointB);

#ifdef JPH_GJK_DEBUG
		Trace("Return: v = [%s], |v| = %g", ConvertToString(ioV).c_str(), ioV.Length());

		// Draw -ioV to show the closest point to the origin from the previous simplex
		DebugRenderer::sInstance->DrawArrow(mOffset, mOffset - ioV, Color::sOrange, 0.05f);

		// Draw the closest points
		DebugRenderer::sInstance->DrawMarker(mOffset + outPointA, Color::sGreen, 1.0f);
		DebugRenderer::sInstance->DrawMarker(mOffset + outPointB, Color::sPurple, 1.0f);

		// Draw the simplex and the Minkowski difference around it
		DrawState();
#endif
		btScalar l = ioV.length2();
		assert(ioV.length2() == v_len_sq);
		return v_len_sq;
	}

	/// Get the resulting simplex after the GetClosestPoints algorithm finishes.
	/// If it returned a squared distance of 0, the origin will be contained in the simplex.
	void		GetClosestPointsSimplex(Vec3 *outY, Vec3 *outP, Vec3 *outQ, uint &outNumPoints)
	{
		uint size = sizeof(Vec3) * mNumPoints;
		memcpy(outY, mY, size);
		memcpy(outP, mP, size);
		memcpy(outQ, mQ, size);
		outNumPoints = mNumPoints;
	}

	/// Test if a ray inRayOrigin + lambda * inRayDirection for lambda e [0, ioLambda> instersects inA
	///
	/// Code based upon: Ray Casting against General Convex Objects with Application to Continuous Collision Detection - Gino van den Bergen
	///
	/// @param inRayOrigin Origin of the ray
	/// @param inRayDirection Direction of the ray (ioLambda * inDirection determines length)
	///	@param inTolerance The minimal distance between the ray and A before it is considered colliding
	/// @param inA A convex object that has the GetSupport(Vec3) function
	/// @param ioLambda The max fraction along the ray, on output updated with the actual collision fraction.
	/// 
	///	@return true if a hit was found, ioLambda is the solution for lambda.
	template <typename A>
	bool		CastRay(Vec3Arg inRayOrigin, Vec3Arg inRayDirection, btScalar inTolerance, const A &inA, btScalar &ioLambda)
	{
		btScalar tolerance_sq = Square(inTolerance);

		// Reset state
		mNumPoints = 0;

		btScalar lambda = 0.0f;
		Vec3 x = inRayOrigin;
		Vec3 v = x - inA.GetSupport(Vec3::sZero());
		btScalar v_len_sq = FLT_MAX;
		bool allow_restart = false;
				
		for (;;)
		{
#ifdef JPH_GJK_DEBUG
			Trace("v = [%s], num_points = %d", ConvertToString(v).c_str(), mNumPoints);
#endif

			// Get new support point
			Vec3 p = inA.GetSupport(v);
			Vec3 w = x - p;

#ifdef JPH_GJK_DEBUG
			Trace("w = [%s]", ConvertToString(w).c_str());
#endif

			btScalar v_dot_w = v.dot(w);
#ifdef JPH_GJK_DEBUG
			Trace("v . w = %g", v_dot_w);
#endif
			if (v_dot_w > 0.0f)
			{
				// If ray and normal are in the same direction, we've passed A and there's no collision
				btScalar v_dot_r = v.dot(inRayDirection);
#ifdef JPH_GJK_DEBUG
				Trace("v . r = %g", v_dot_r);
#endif
				if (v_dot_r >= 0.0f)
					return false;

				// Update the lower bound for lambda
				btScalar delta = v_dot_w / v_dot_r;
				btScalar old_lambda = lambda;
				lambda -= delta;
#ifdef JPH_GJK_DEBUG
				Trace("lambda = %g, delta = %g", lambda, delta);
#endif

				// If lambda didn't change, we cannot converge any further and we assume a hit
				if (old_lambda == lambda)
					break;

				// If lambda is bigger or equal than max, we don't have a hit
				if (lambda >= ioLambda)
					return false;

				// Update x to new closest point on the ray
				x = inRayOrigin + lambda * inRayDirection;
				
				// We've shifted x, so reset v_len_sq so that it is not used as early out for GetClosest
				v_len_sq = FLT_MAX;

				// We allow rebuilding the simplex once after x changes because the simplex was built
				// for another x and numerical round off builds up as you keep adding points to an
				// existing simplex
				allow_restart = true;
			}

			// Add p to set P: P = P U {p}
			mP[mNumPoints] = p;
			++mNumPoints;

			// Calculate Y = {x} - P
			for (int i = 0; i < mNumPoints; ++i)
				mY[i] = x - mP[i];

			// Determine the new closest point from Y to origin
			bool needs_restart = false;
			jUint32a_t set;						// Set of points that form the new simplex
			if (!GetClosest(v_len_sq, v, v_len_sq, set))
			{
#ifdef JPH_GJK_DEBUG
				Trace("Failed to converge");
#endif

				// We failed to converge, restart
				needs_restart = true;
			}
			else if (set == 0xf)
			{
#ifdef JPH_GJK_DEBUG
				Trace("Full simplex");
#endif

				// If there are 4 points, x is inside the tetrahedron and we've found a hit
				// Double check if this is indeed the case
				if (v_len_sq <= tolerance_sq)
					break;

				// We failed to converge, restart
				needs_restart = true;
			}

			if (needs_restart)
			{
				// Only allow 1 restart, if we still can't get a closest point 
				// we're so close that we return this as a hit
				if (!allow_restart)
					break;

				// If we fail to converge, we start again with the last point as simplex
#ifdef JPH_GJK_DEBUG
				Trace("Restarting");
#endif
				allow_restart = false;
				mP[0] = p;
				mNumPoints = 1;
				v = x - p;
				v_len_sq = FLT_MAX;
				continue;
			}

			// Update the points P to form the new simplex
			// Note: We're not updating Y as Y will shift with x so we have to calculate it every iteration
			UpdatePointSetP(set);

			// Check if x is close enough to inA
			if (v_len_sq <= tolerance_sq)
			{
#ifdef JPH_GJK_DEBUG
				Trace("Converged");
#endif
				break;
			}
		}

		// Store hit fraction
		ioLambda = lambda;
		return true;
	}


private:
#ifdef JPH_GJK_DEBUG
	/// Draw state of algorithm
	void		DrawState()
	{
		Mat44 origin = Mat44::sTranslation(mOffset);

		// Draw origin
		DebugRenderer::sInstance->DrawCoordinateSystem(origin, 1.0f);

		// Draw the hull
		DebugRenderer::sInstance->DrawGeometry(origin, mGeometry->mBounds, mGeometry->mBounds.GetExtent().length2(), Color::sYellow, mGeometry);

		// Draw Y
		for (int i = 0; i < mNumPoints; ++i)
		{
			// Draw support point
			Vec3 y_i = origin * mY[i];
			DebugRenderer::sInstance->DrawMarker(y_i, Color::sRed, 1.0f);
			for (int j = i + 1; j < mNumPoints; ++j)
			{
				// Draw edge
				Vec3 y_j = origin * mY[j];
				DebugRenderer::sInstance->DrawLine(y_i, y_j, Color::sRed);
				for (int k = j + 1; k < mNumPoints; ++k)
				{
					// Make sure triangle faces the origin
					Vec3 y_k = origin * mY[k];
					Vec3 center = (y_i + y_j + y_k) / 3.0f;
					Vec3 normal = (y_j - y_i).Cross(y_k - y_i);
					if (normal.dot(center) < 0.0f)
						DebugRenderer::sInstance->DrawTriangle(y_i, y_j, y_k, Color::sLightGrey);
					else
						DebugRenderer::sInstance->DrawTriangle(y_i, y_k, y_j, Color::sLightGrey);
				}
			}
		}

		// Offset to the right
		mOffset += Vec3(mGeometry->mBounds.GetSize().GetX() + 2.0f, 0, 0);
	}
#endif // JPH_GJK_DEBUG

	Vec3		mY[4];						///< Support points on A - B
	Vec3		mP[4];						///< Support point on A
	Vec3		mQ[4];						///< Support point on B
	int			mNumPoints = 0;				///< Number of points in mY, mP and mQ that are valid

#ifdef JPH_GJK_DEBUG
	DebugRenderer::GeometryRef	mGeometry;	///< A visualization of the minkowski difference for state drawing
	Vec3		mOffset = Vec3::sZero();	///< Offset to use for state drawing
#endif
};

} // BTJPH
#endif //__JOLT_GJK_CLOSEST_POINT_H
