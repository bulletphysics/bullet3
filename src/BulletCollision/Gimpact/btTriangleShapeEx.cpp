/*! \file btGImpactTriangleShape.h
\author Francisco Leon Najera
*/
/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
This is a modified version of the Bullet Continuous Collision Detection and Physics Library
*/


#include "btTriangleShapeEx.h"

void GIM_TRIANGLE_CONTACT::merge_points(const btVector4& plane,
										btScalar margin, const btVector3* points, int point_count)
{
	m_point_count = 0;
	m_penetration_depth = -1000.0f;

	int point_indices[MAX_TRI_CLIPPING];

	int _k;

	for (_k = 0; _k < point_count; _k++)
	{
		btScalar _dist = -bt_distance_point_plane(plane, points[_k]) + margin;

		if (_dist >= 0.0f)
		{
			if (_dist > m_penetration_depth)
			{
				m_penetration_depth = _dist;
				point_indices[0] = _k;
				m_point_count = 1;
			}
			else if ((_dist + SIMD_EPSILON) >= m_penetration_depth)
			{
				point_indices[m_point_count] = _k;
				m_point_count++;
			}
		}
	}

	for (_k = 0; _k < m_point_count; _k++)
	{
		m_points[_k] = points[point_indices[_k]];
	}
}

///class btPrimitiveTriangle
bool btPrimitiveTriangle::overlap_test_conservative(const btPrimitiveTriangle& other)
{
	btScalar total_margin = m_margin + other.m_margin;
	// classify points on other triangle
	btScalar dis0 = bt_distance_point_plane(m_plane, other.m_vertices[0]) - total_margin;

	btScalar dis1 = bt_distance_point_plane(m_plane, other.m_vertices[1]) - total_margin;

	btScalar dis2 = bt_distance_point_plane(m_plane, other.m_vertices[2]) - total_margin;

	if (dis0 > 0.0f && dis1 > 0.0f && dis2 > 0.0f) return false;

	// classify points on this triangle
	dis0 = bt_distance_point_plane(other.m_plane, m_vertices[0]) - total_margin;

	dis1 = bt_distance_point_plane(other.m_plane, m_vertices[1]) - total_margin;

	dis2 = bt_distance_point_plane(other.m_plane, m_vertices[2]) - total_margin;

	if (dis0 > 0.0f && dis1 > 0.0f && dis2 > 0.0f) return false;

	return true;
}

bool btPrimitiveTriangle::overlap_test(const btPrimitiveTriangle& other) const
{
	btScalar total_margin = m_margin + other.m_margin;
	// classify points on other triangle
	btScalar dis0 = bt_distance_point_plane(m_plane, other.m_vertices[0]);

	btScalar dis1 = bt_distance_point_plane(m_plane, other.m_vertices[1]);

	btScalar dis2 = bt_distance_point_plane(m_plane, other.m_vertices[2]);

	if ((dis0 - total_margin) > 0.0f && (dis1 - total_margin) > 0.0f && (dis2 - total_margin) > 0.0f) return false;
	if ((dis0 + total_margin) < 0.0f && (dis1 + total_margin) < 0.0f && (dis2 + total_margin) < 0.0f) return false;

	// classify points on this triangle
	dis0 = bt_distance_point_plane(other.m_plane, m_vertices[0]);

	dis1 = bt_distance_point_plane(other.m_plane, m_vertices[1]);

	dis2 = bt_distance_point_plane(other.m_plane, m_vertices[2]);

	if ((dis0 - total_margin) > 0.0f && (dis1 - total_margin) > 0.0f && (dis2 - total_margin) > 0.0f) return false;
	if ((dis0 + total_margin) < 0.0f && (dis1 + total_margin) < 0.0f && (dis2 + total_margin) < 0.0f) return false;

	return true;
}

bool btPrimitiveTriangle::validity_test() const
{
	const btScalar epsilon = SIMD_EPSILON * SIMD_EPSILON;
	if ((m_vertices[0] - m_vertices[1]).length2() < epsilon)
		return false;
	if ((m_vertices[0] - m_vertices[2]).length2() < epsilon)
		return false;
	if ((m_vertices[1] - m_vertices[2]).length2() < epsilon)
		return false;

	if ((m_vertices[1] - m_vertices[0]).cross(m_vertices[2] - m_vertices[0]).length2() < epsilon)
		return false;

	return true;

}

int btPrimitiveTriangle::clip_triangle(btPrimitiveTriangle& other, btVector3* clipped_points)
{
	// edge 0

	btVector3 temp_points[MAX_TRI_CLIPPING];

	btVector4 edgeplane;

	get_edge_plane(0, edgeplane);

	int clipped_count = bt_plane_clip_triangle(
		edgeplane, other.m_vertices[0], other.m_vertices[1], other.m_vertices[2], temp_points);

	if (clipped_count == 0) return 0;

	btVector3 temp_points1[MAX_TRI_CLIPPING];

	// edge 1
	get_edge_plane(1, edgeplane);

	clipped_count = bt_plane_clip_polygon(edgeplane, temp_points, clipped_count, temp_points1);

	if (clipped_count == 0) return 0;

	// edge 2
	get_edge_plane(2, edgeplane);

	clipped_count = bt_plane_clip_polygon(
		edgeplane, temp_points1, clipped_count, clipped_points);

	return clipped_count;
}

bool btPrimitiveTriangle::intersectSegmentTriangle(btVector3 p, const btVector3& q, const btPrimitiveTriangle& triangle, btVector3& C)
{
	btVector3 ab(triangle.m_vertices[1] - triangle.m_vertices[0]);
	btVector3 ac(triangle.m_vertices[2] - triangle.m_vertices[0]);
	btVector3 n(ab.cross(ac));

	btVector3 qp(p - q);
	btScalar d = qp.dot(n);
	if (d == 0.0f)
		return false;
	else if (d < 0.0f)
	{
		//Vector3 tmp = p;
		p = q;
		//q = tmp;

		qp = -qp;
		d = qp.dot(n);
	}

	btVector3 ap(p - triangle.m_vertices[0]);

	float t = ap.dot(n);
	if (t < 0.0f || t > d)
		return false;

	btVector3 e(qp.cross(ap));
	float v = ac.dot(e);
	if (v < 0.0f || v > d)
		return false;

	float w = -ab.dot(e);
	if (w < 0.0f || v + w > d)
		return false;

	// Segment intersect triangle, compute intersetionPoint
	C = p - qp * t / d;
	return true;  //*/
}

bool btPrimitiveTriangle::find_triangle_collision_alt_method(const btPrimitiveTriangle& other, GIM_TRIANGLE_CONTACT& contacts)
{
	btVector3 startPoint, endPoint;

	static const float epsilon = SIMD_EPSILON;
	// 2 triangles collide if 2 segments of any triangle intersects the other one,
	// or if one segment from both triangles intersects the other one

	bool bCollide = false;

	btVector3 t1Normal(this->m_plane.x(), this->m_plane.y(), this->m_plane.z());
	btVector3 t2Normal(other.m_plane.x(), other.m_plane.y(), other.m_plane.z());
	btScalar t1PlaneD = -this->m_plane.w();
	btScalar t2PlaneD = -other.m_plane.w();

	// If all vertexes of one triangle are on the same side of 1 plane of the other triangle, no collision occurs
	// Check vertices of the second triangle against a plane of the first triangle
	btScalar dt2v1 = t1Normal.dot(other.m_vertices[0]) + t1PlaneD;
	btScalar dt2v2 = t1Normal.dot(other.m_vertices[1]) + t1PlaneD;
	btScalar dt2v3 = t1Normal.dot(other.m_vertices[2]) + t1PlaneD;
	if (((dt2v1 > epsilon) && (dt2v2 > epsilon) && (dt2v3 > epsilon)) ||
		((dt2v1 < -epsilon) && (dt2v2 < -epsilon) && (dt2v3 < -epsilon)))
		return false;
	// And vice-versa
	btScalar dt1v1 = t2Normal.dot(this->m_vertices[0]) + t2PlaneD;
	btScalar dt1v2 = t2Normal.dot(this->m_vertices[1]) + t2PlaneD;
	btScalar dt1v3 = t2Normal.dot(this->m_vertices[2]) + t2PlaneD;
	if (((dt1v1 > epsilon) && (dt1v2 > epsilon) && (dt1v3 > epsilon)) ||
		((dt1v1 < -epsilon) && (dt1v2 < -epsilon) && (dt1v3 < -epsilon)))
		return false;

	btVector3 intersectionPoint1, intersectionPoint2, intersectionPoint3, intersectionPoint4, intersectionPoint5, intersectionPoint6;

	int bV1 = (int)intersectSegmentTriangle(this->m_vertices[0], this->m_vertices[1], other, intersectionPoint1);
	int bV2 = (int)intersectSegmentTriangle(this->m_vertices[0], this->m_vertices[2], other, intersectionPoint2);
	int bV3 = (int)intersectSegmentTriangle(this->m_vertices[1], this->m_vertices[2], other, intersectionPoint3);

	int bV4, bV5, bV6;

	int bV = bV1 + bV2 + bV3;

	switch (bV)
	{
		case 0:
			bV4 = (int)intersectSegmentTriangle(other.m_vertices[0], other.m_vertices[1], *this, intersectionPoint4);
			bV5 = (int)intersectSegmentTriangle(other.m_vertices[0], other.m_vertices[2], *this, intersectionPoint5);
			bV6 = (int)intersectSegmentTriangle(other.m_vertices[1], other.m_vertices[2], *this, intersectionPoint6);
			bV = bV4 + bV5 + bV6;
			if (bV == 2)
			{
				bCollide = true;
				if (bV4 == 1 && bV5 == 1)
				{
					startPoint = intersectionPoint4;
					endPoint = intersectionPoint5;
				}
				else if (bV4 == 1 && bV6 == 1)
				{
					startPoint = intersectionPoint6;
					endPoint = intersectionPoint4;
				}
				else
				{
					startPoint = intersectionPoint5;
					endPoint = intersectionPoint6;
				}
			}
			else if (bV == 1)
			{
				bCollide = true;
				if (bV4 == 1)
					startPoint = endPoint = intersectionPoint4;
				else if (bV5 == 1)
					startPoint = endPoint = intersectionPoint5;
				else
					startPoint = endPoint = intersectionPoint6;
			}
			break;
		case 1:
			bV4 = (int)intersectSegmentTriangle(other.m_vertices[0], other.m_vertices[1], *this, intersectionPoint4);
			if (bV4 == 1)
			{
				bCollide = true;
				if (bV1 == 1)
				{
					startPoint = intersectionPoint1;
					endPoint = intersectionPoint4;
				}
				else if (bV2 == 1)
				{
					startPoint = intersectionPoint2;
					endPoint = intersectionPoint4;
				}
				else
				{
					startPoint = intersectionPoint3;
					endPoint = intersectionPoint4;
				}

				break;
			}

			bV5 = (int)intersectSegmentTriangle(other.m_vertices[0], other.m_vertices[2], *this, intersectionPoint5);
			if (bV5 == 1)
			{
				bCollide = true;
				if (bV1 == 1)
				{
					startPoint = intersectionPoint1;
					endPoint = intersectionPoint5;
				}
				else if (bV2 == 1)
				{
					startPoint = intersectionPoint2;
					endPoint = intersectionPoint5;
				}
				else
				{
					startPoint = intersectionPoint3;
					endPoint = intersectionPoint5;
				}
				break;
			}

			bV6 = (int)intersectSegmentTriangle(other.m_vertices[1], other.m_vertices[2], *this, intersectionPoint6);
			if (bV6 == 1)
			{
				bCollide = true;
				if (bV1 == 1)
				{
					startPoint = intersectionPoint1;
					endPoint = intersectionPoint6;
				}
				else if (bV2 == 1)
				{
					startPoint = intersectionPoint2;
					endPoint = intersectionPoint6;
				}
				else
				{
					startPoint = intersectionPoint3;
					endPoint = intersectionPoint6;
				}
			}

			if ((bV4 + bV5 + bV6) == 0)
			{
				bCollide = true;
				if (bV1 == 1)
					startPoint = endPoint = intersectionPoint1;
				else if (bV2 == 1)
					startPoint = endPoint = intersectionPoint2;
				else
					startPoint = endPoint = intersectionPoint3;
			}
			break;
		case 2:
			bCollide = true;
			if (bV1 == 1 && bV2 == 1)
			{
				startPoint = intersectionPoint1;
				endPoint = intersectionPoint2;
			}
			else if (bV1 == 1 && bV3 == 1)
			{
				startPoint = intersectionPoint1;
				endPoint = intersectionPoint3;
			}
			else
			{
				startPoint = intersectionPoint2;
				endPoint = intersectionPoint3;
			}
			break;
		default:
			break;
	}

	if (bCollide)
	{
		GIM_TRIANGLE_CONTACT contact;
		contact.m_separating_normal = this->m_plane;
		contact.m_penetration_depth = (endPoint - startPoint).length() * 2.0f;
		contact.m_point_count = 2;
		contact.m_points[0] = startPoint;
		contact.m_points[1] = endPoint;
		contacts.copy_from(contact);
	}

	return bCollide;
}

bool btPrimitiveTriangle::segmentSegmentDistance(const btVector3& p1, const btVector3& p2, const btVector3& s1, const btVector3& s2,
							btScalar& tp_out, btScalar& ts_out, btVector3& p_closest_out, btVector3& s_closest_out, btScalar& dist_sq_out, btScalar max_distance_sq)
{
	// http://geomalgorithms.com/a07-_distance.html

	btVector3 u = p2 - p1;
	btVector3 v = s2 - s1;
	btVector3 w = p1 - s1;
	btScalar a = u.dot(u);
	btScalar b = u.dot(v);
	btScalar c = v.dot(v);
	btScalar d = u.dot(w);
	btScalar e = v.dot(w);

	btScalar denom = a * c - b * b;  // = |u x v| ^ 2

	// tp = tp_numer / tp_denom, ts = ts_numer / ts_denom
	btScalar tp_numer, ts_numer, tp_denom, ts_denom;

	// Compute tp and ts as if it was a line and check its distance. If it is larger than max distance,
	// than there is no need to compute the rest
	tp_numer = b * e - c * d;
	tp_denom = denom;
	ts_numer = a * e - b * d;
	ts_denom = denom;
	if (max_distance_sq < FLT_MAX)
	{
		btScalar lines_dist_denom_sq = (w * denom + u * tp_numer - v * ts_numer).length2();
		if (lines_dist_denom_sq > (max_distance_sq * denom * denom))
			return false;  // The lines are too far
	}

	if (denom < SIMD_EPSILON)
	{
		// Segments are parallel. Choose p1 as the closest point on p1-p2 and then check whether
		// it is the closest point from the other segment
		tp_numer = 0.0f;
		tp_denom = 1.0f;
		ts_numer = e;
		ts_denom = c;
	}
	else
	{
		// Segments are not parallel, compute the closest points (on lines first)

		// Use tp_numer and tp_denom as if the segments were lines

		// Check whether tp is in interval [0, 1] and adjust it and ts if it is not
		if (tp_numer < 0.0f)
		{
			tp_numer = 0.0;
			ts_numer = e;
			ts_denom = c;
		}
		else if (tp_numer > tp_denom)
		{
			tp_numer = tp_denom;
			ts_numer = e + b;
			ts_denom = c;
		}
		else
		{
			// Use ts_numer and ts_denom as if the segments were lines
		}
	}

	// Now, check whether ts is in interval [0,1]
	if (ts_numer < 0.0f)
	{
		ts_numer = 0.0;

		// Recompute tp. tp should be -d/a, check whether it is in interval [0,1]
		tp_numer = -d;
		if (tp_numer < 0.0f)
		{
			tp_numer = 0.0f;
		}
		else if (tp_numer > a)
		{
			tp_numer = tp_denom;
		}
		else
		{
			tp_denom = a;
		}
	}
	else if (ts_numer > ts_denom)
	{
		ts_numer = ts_denom;

		// Recompute tp. tp should be (-d+b)/a, check whether it is in interval [0,1]
		tp_numer = b - d;
		if (tp_numer < 0.0f)
		{
			tp_numer = 0.0f;
		}
		else if (tp_numer > a)
		{
			tp_numer = tp_denom;
		}
		else
		{
			tp_denom = a;
		}
	}
	else
	{
		// ts is OK, and so is tp
	}

	// Be careful to divide by really small numbers.
	btScalar ts = (fabs(ts_numer) < SIMD_EPSILON) ? 0.0f : (ts_numer / ts_denom);
	btScalar tp = (fabs(tp_numer) < SIMD_EPSILON) ? 0.0f : (tp_numer / tp_denom);

	// Compute the distance
	btVector3 p_closest = p1 + u * tp;
	btVector3 s_closest = s1 + v * ts;
	btScalar dist_sq = (p_closest - s_closest).length2();

	if (dist_sq <= max_distance_sq)
	{
		tp_out = tp;
		ts_out = ts;
		p_closest_out = p_closest;
		s_closest_out = s_closest;
		dist_sq_out = dist_sq;
		return true;
	}
	else
		return false;
}

bool btPrimitiveTriangle::pointTriangleDistance(const btVector3& q, const btVector3& p1, const btVector3& p2, const btVector3& p3,
						   btScalar& tp_out, btScalar& ts_out, btVector3& closest_out, btScalar& dist_sq_out, btScalar max_distance_sq)
{
	// According to http://web.mit.edu/ehliu/Public/Darmofal/DistancePoint3Triangle3.pdf
	btVector3 P = q;
	btVector3 B = p1;
	btVector3 E0 = p2 - p1;
	btVector3 E1 = p3 - p1;
	btVector3 W = B - P;
	btScalar a = E0.dot(E0);
	btScalar b = E0.dot(E1);
	btScalar c = E1.dot(E1);
	btScalar d = E0.dot(W);
	btScalar e = E1.dot(W);
	//btScalar f = W.Dot(W);

	btScalar det = a * c - b * b;  // = |u x v| ^ 2
	btScalar s = b * e - c * d;
	btScalar t = b * d - a * e;

	// Use s and t as if it was a plane check the distance. If it is larger than max distance,
	// than there is no need to compute the rest.
	if (max_distance_sq < FLT_MAX)
	{
		btScalar dist_det_sq = ((B - q) * det + E0 * s + E1 * t).length2();
		if (dist_det_sq > (max_distance_sq * det * det))
			return false;  // The point is too far
	}

	// Q(s,t) = as^2 + 2bst + ct^2 + 2ds + 2et + f

	if ((s + t) <= det)
	{
		if (s < 0.0f)
		{
			if (t < 0.0f)
			{
				// region 4

				// Grad(Q) = 2(as+bt+d,bs+ct+e)
				// (1,0)*Grad(Q(0,0)) = (1,0)*(d,e) = d
				// (0,1)*Grad(Q(0,0)) = (0,1)*(d,e) = e
				// min in (0,0) if both grads are >= 0
				// min on edge t=0 if (1,0)*Grad(Q(0,0)) < 0
				// min on edge s=0 otherwise
				if (d < 0.0f)  // minimum on edge t=0
				{
					// Like in region 5, but d is already < 0
					s = (-d >= a) ? 1.0f : (-d / a);
					t = 0.0f;
				}
				else if (e < 0.0f)
				{
					// Like in region 3, but e is already < 0
					s = 0.0f;
					t = (-e >= c) ? 1.0f : (-e / c);
				}
				else
				{
					s = 0.0f;
					t = 0.0f;
				}
			}
			else
			{
				// region 3
				// F(t) = Q(0,t) = ct^2 + 2et + f
				// F�(t)/2 = ct+e
				// F�(T) = 0 when T = -e/c
				s = 0.0f;
				t = (e >= 0.0f) ? 0.0f : ((-e >= c) ? 1.0f : (-e / c));
			}
		}
		else if (t < 0.0f)
		{
			// region 5
			// F(s) = Q(s,0) = as^2 + 2ds + f
			// F�(s)/2 = as+d
			// F�(S) = 0 when S = -d/a
			s = (d >= 0.0f) ? 0.0f : ((-d >= a) ? 1.0f : (-d / a));
			t = 0.0f;
		}
		else
		{
			// region 0
			s /= det;
			t /= det;
		}
	}
	else
	{
		if (s < 0.0f)
		{
			// region 2

			// Grad(Q) = 2(as+bt+d,bs+ct+e)
			// (0,-1)*Grad(Q(0,1)) = (0,-1)*(b+d,c+e) = -(c+e)
			// (1,-1)*Grad(Q(0,1)) = (1,-1)*(b+d,c+e) = (b+d)-(c+e)
			// min on edge s+t=1 if (1,-1)*Grad(Q(0,1)) < 0
			// min on edge s=0 otherwise
			btScalar tmp0 = b + d;
			btScalar tmp1 = c + e;
			if (tmp1 > tmp0)  // minimum on edge s+t=1
			{
				btScalar numer = tmp1 - tmp0;
				btScalar denom = a - 2.0f * b + c;
				s = ((numer >= denom) ? 1.0f : (numer / denom));
				t = 1.0f - s;
			}
			else  // minimum on edge s=0
			{
				s = 0.0f;
				t = (tmp1 <= 0.0f) ? 1.0f : ((e >= 0.0f) ? 0.0f : (-e / c));
			}
		}
		else if (t < 0.0f)
		{
			// region 6

			// Grad(Q) = 2(as+bt+d,bs+ct+e)
			// (-1,0)*Grad(Q(1,0)) = (-1,0)*(a+d,b+e) = -(a+d)
			// (-1,1)*Grad(Q(1,0)) = (-1,1)*(a+d,b+e) = (b+e)-(a+d)
			// min on edge s+t=1 if (-1,1)*Grad(Q(1,0)) < 0
			// min on edge t=0 otherwise
			btScalar tmp0 = b + e;
			btScalar tmp1 = a + d;
			if (tmp1 > tmp0)  // minimum on edge s+t=1
			{
				btScalar numer = tmp1 - tmp0;
				btScalar denom = a - 2.0f * b + c;
				t = ((numer >= denom) ? 1.0f : (numer / denom));
				s = 1.0f - t;
			}
			else  // minimum on edge t=0
			{
				s = (tmp1 <= 0.0f) ? 1.0f : ((d >= 0.0f) ? 0.0f : (-d / a));
				t = 0.0f;
			}
		}
		else
		{
			// region 1
			// F(s) = Q(s,1-s) = (a-2b+c)s^2 + 2(b-c+d-e)s + (c+2e+f)
			// F�(s)/2 = (a-2b+c)s + (b-c+d-e)
			// F�(S) = 0 when S = (c+e-b-d)/(a-2b+c)
			// a-2b+c = |E0-E1|^2 > 0, so only sign of c+e-b-d need be considered
			//
			// Note: F(t) = trying Q(1-t,t) = a(1-t)^2 + 2b(1-t)t + ct^2 + 2d(1-t) + 2et + f =
			//			= at^2 - 2at + a + 2bt - 2bt^2 + ct^2 + 2d - 2dt + 2et + f =
			//			= (a-2b+c)t^2 + 2(-a+b-d+e)t + (a+2d+f)
			// F'(t)/2 = (a-2b+c)t + (-a+b-d+e)
			// F'(T) = 0 when T = (a-b+d-e)/(a-2b+c)

			btScalar numer = c + e - b - d;
			if (numer <= 0.0f)
			{
				s = 0.0f;
			}
			else
			{
				btScalar denom = a - 2.0f * b + c;  // positive quantity
				s = ((numer >= denom) ? 1.0f : (numer / denom));
			}
			t = 1.0f - s;
		}
	}

	btScalar tp = s;
	btScalar ts = t;

	// Compute the distance
	btVector3 closest = B + E0 * tp + E1 * ts;
	btScalar dist_sq = (closest - q).length2();

	if (dist_sq <= max_distance_sq)
	{
		tp_out = tp;
		ts_out = ts;
		closest_out = closest;
		dist_sq_out = dist_sq;
		return true;
	}
	else
		return false;
}

bool btPrimitiveTriangle::triangle_triangle_distance(const btPrimitiveTriangle& b, btScalar& dist_sq_out, btVector3& a_closest_out, btVector3& b_closest_out,
	float max_distance_sq)
{
	btVector3 a_closest, b_closest;
	GIM_TRIANGLE_CONTACT contact;

	if (find_triangle_collision_alt_method(b, contact))
	{
		dist_sq_out = 0.0f;
		a_closest = contact.m_points[0], b_closest = contact.m_points[1];
		a_closest_out = a_closest;
		b_closest_out = b_closest;
		return true;
	}

	// The triangles don't collide, the distance is the closest of 9 ExE and 6 VxF distances
	btScalar curr_dist_sq = max_distance_sq, tp, ts;
	this->m_vertices[1], b.m_vertices[0];
	segmentSegmentDistance(this->m_vertices[0], this->m_vertices[1], b.m_vertices[0], b.m_vertices[1], tp, ts, a_closest, b_closest, curr_dist_sq, curr_dist_sq);
	segmentSegmentDistance(this->m_vertices[1], this->m_vertices[2], b.m_vertices[0], b.m_vertices[1], tp, ts, a_closest, b_closest, curr_dist_sq, curr_dist_sq);
	segmentSegmentDistance(this->m_vertices[2], this->m_vertices[0], b.m_vertices[0], b.m_vertices[1], tp, ts, a_closest, b_closest, curr_dist_sq, curr_dist_sq);
	segmentSegmentDistance(this->m_vertices[0], this->m_vertices[1], b.m_vertices[1], b.m_vertices[2], tp, ts, a_closest, b_closest, curr_dist_sq, curr_dist_sq);
	segmentSegmentDistance(this->m_vertices[1], this->m_vertices[2], b.m_vertices[1], b.m_vertices[2], tp, ts, a_closest, b_closest, curr_dist_sq, curr_dist_sq);
	segmentSegmentDistance(this->m_vertices[2], this->m_vertices[0], b.m_vertices[1], b.m_vertices[2], tp, ts, a_closest, b_closest, curr_dist_sq, curr_dist_sq);
	segmentSegmentDistance(this->m_vertices[0], this->m_vertices[1], b.m_vertices[2], b.m_vertices[0], tp, ts, a_closest, b_closest, curr_dist_sq, curr_dist_sq);
	segmentSegmentDistance(this->m_vertices[1], this->m_vertices[2], b.m_vertices[2], b.m_vertices[0], tp, ts, a_closest, b_closest, curr_dist_sq, curr_dist_sq);
	segmentSegmentDistance(this->m_vertices[2], this->m_vertices[0], b.m_vertices[2], b.m_vertices[0], tp, ts, a_closest, b_closest, curr_dist_sq, curr_dist_sq);

	if (pointTriangleDistance(this->m_vertices[0], b.m_vertices[0], b.m_vertices[1], b.m_vertices[2], tp, ts, b_closest, curr_dist_sq, curr_dist_sq)) a_closest = this->m_vertices[0];
	if (pointTriangleDistance(this->m_vertices[1], b.m_vertices[0], b.m_vertices[1], b.m_vertices[2], tp, ts, b_closest, curr_dist_sq, curr_dist_sq)) a_closest = this->m_vertices[1];
	if (pointTriangleDistance(this->m_vertices[2], b.m_vertices[0], b.m_vertices[1], b.m_vertices[2], tp, ts, b_closest, curr_dist_sq, curr_dist_sq)) a_closest = this->m_vertices[2];
	if (pointTriangleDistance(b.m_vertices[0], this->m_vertices[0], this->m_vertices[1], this->m_vertices[2], tp, ts, a_closest, curr_dist_sq, curr_dist_sq)) b_closest = b.m_vertices[0];
	if (pointTriangleDistance(b.m_vertices[1], this->m_vertices[0], this->m_vertices[1], this->m_vertices[2], tp, ts, a_closest, curr_dist_sq, curr_dist_sq)) b_closest = b.m_vertices[1];
	if (pointTriangleDistance(b.m_vertices[2], this->m_vertices[0], this->m_vertices[1], this->m_vertices[2], tp, ts, a_closest, curr_dist_sq, curr_dist_sq)) b_closest = b.m_vertices[2];

	if (curr_dist_sq < max_distance_sq)
	{
		dist_sq_out = curr_dist_sq;
		a_closest_out = a_closest;
		b_closest_out = b_closest;
		return true;
	}
	else
		return false;
}

bool btPrimitiveTriangle::find_triangle_collision_clip_method(btPrimitiveTriangle& other, GIM_TRIANGLE_CONTACT& contacts)
{
	btScalar margin = m_margin + other.m_margin;

	btVector3 clipped_points[MAX_TRI_CLIPPING];
	int clipped_count;
	//create planes
	// plane v vs U points

	GIM_TRIANGLE_CONTACT contacts1;

	contacts1.m_separating_normal = m_plane;

	clipped_count = clip_triangle(other, clipped_points);

	if (clipped_count == 0)
	{
		return false;  //Reject
	}

	//find most deep interval face1
	contacts1.merge_points(contacts1.m_separating_normal, margin, clipped_points, clipped_count);
	if (contacts1.m_point_count == 0) return false;  // too far
	//Normal pointing to this triangle
	contacts1.m_separating_normal *= -1.f;

	//Clip tri1 by tri2 edges
	GIM_TRIANGLE_CONTACT contacts2;
	contacts2.m_separating_normal = other.m_plane;

	clipped_count = other.clip_triangle(*this, clipped_points);

	if (clipped_count == 0)
	{
		return false;  //Reject
	}

	//find most deep interval face1
	contacts2.merge_points(contacts2.m_separating_normal, margin, clipped_points, clipped_count);
	if (contacts2.m_point_count == 0) return false;  // too far

	////check most dir for contacts
	if (contacts2.m_penetration_depth < contacts1.m_penetration_depth)
	{
		contacts.copy_from(contacts2);
	}
	else
	{
		contacts.copy_from(contacts1);
	}

	return true;
}

bool btPrimitiveTriangle::find_triangle_collision_alt_method_outer(btPrimitiveTriangle& other, GIM_TRIANGLE_CONTACT& contacts, btScalar marginZoneRecoveryStrengthFactor,
																   const btTransform& thisTransformLastSafe, const btTransform& otherTransformLastSafe,
																   const btPrimitiveTriangle& thisBackup, const btPrimitiveTriangle& otherBackup, bool doUnstuck)
{
	btScalar margin = m_margin + other.m_margin;

	contacts.m_point_count = 0;
	btScalar dist_sq_out, dist, t = 1.0;
	btVector3 a_closest_out, b_closest_out;
	bool ret = false;
	const btScalar maxDepth = margin * marginZoneRecoveryStrengthFactor;

	auto create_contact = [&]() {
		btVector3 diff = a_closest_out - b_closest_out;
		if (dist == 0.0)
			dist = diff.length();
		btVector3 dir = diff / dist;
		contacts.m_point_count = 1;
		contacts.m_points[0] = a_closest_out;
		contacts.m_separating_normal = btVector4(dir.x(), dir.y(), dir.z(), 1.0);
		// Inversion so that smaller distance means bigger impulse, up to the maxDepth when distance is 0. Margin distance means 0 depth.
		if (fabs(margin) < SIMD_EPSILON)
			contacts.m_penetration_depth = dist;
		else
			contacts.m_penetration_depth = -dist * ((1.0 / margin) * maxDepth) + maxDepth;
		//printf("contacts.m_penetration_depth %f\n", contacts.m_penetration_depth);
	};
	
	ret = triangle_triangle_distance(other, dist_sq_out, a_closest_out, b_closest_out);
	dist = sqrtf(dist_sq_out);
	if (ret && dist_sq_out != 0.0 && dist < margin)
	{
		// In the margin zone. No actual penetration yet. Calculating m_separating_normal is very easy thanks to this.
		create_contact();
		return true;
	}
	else if (ret && dist_sq_out == 0.0)
	{
		// Triangle penetration. Use the last safe transforms.
		if (doUnstuck)
		{
			auto thisLastSafe = thisBackup;
			auto otherLastSafe = otherBackup;
			thisLastSafe.applyTransform(thisTransformLastSafe);
			otherLastSafe.applyTransform(otherTransformLastSafe);
			thisLastSafe.buildTriPlane();
			otherLastSafe.buildTriPlane();

			ret = thisLastSafe.triangle_triangle_distance(otherLastSafe, dist_sq_out, a_closest_out, b_closest_out);
			// Since we use the safe positions, nothing should be penetrating. This assert is very useful for testing if some change in code
			// didn't mess things up, but in a more complex scenario where there is another movable which places itself into the first's safe zone,
			// this assert should be commented out.
			//btAssert(!(ret && dist_sq_out == 0.0));

			dist = sqrtf(dist_sq_out);
			create_contact();
			contacts.m_penetration_depth = -maxDepth;  // Mark it as penetration by making it negative
		}
		else
		{
			find_triangle_collision_clip_method(other, contacts);
			contacts.m_penetration_depth = -contacts.m_penetration_depth;
		}

		return true;
	}

	return false;
}

///class btTriangleShapeEx: public btTriangleShape

bool btTriangleShapeEx::overlap_test_conservative(const btTriangleShapeEx& other)
{
	btScalar total_margin = getMargin() + other.getMargin();

	btVector4 plane0;
	buildTriPlane(plane0);
	btVector4 plane1;
	other.buildTriPlane(plane1);

	// classify points on other triangle
	btScalar dis0 = bt_distance_point_plane(plane0, other.m_vertices1[0]) - total_margin;

	btScalar dis1 = bt_distance_point_plane(plane0, other.m_vertices1[1]) - total_margin;

	btScalar dis2 = bt_distance_point_plane(plane0, other.m_vertices1[2]) - total_margin;

	if (dis0 > 0.0f && dis1 > 0.0f && dis2 > 0.0f) return false;

	// classify points on this triangle
	dis0 = bt_distance_point_plane(plane1, m_vertices1[0]) - total_margin;

	dis1 = bt_distance_point_plane(plane1, m_vertices1[1]) - total_margin;

	dis2 = bt_distance_point_plane(plane1, m_vertices1[2]) - total_margin;

	if (dis0 > 0.0f && dis1 > 0.0f && dis2 > 0.0f) return false;

	return true;
}