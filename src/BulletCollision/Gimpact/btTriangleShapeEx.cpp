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
	btScalar dis0 = bt_distance_point_plane(m_plane, other.m_vertices[0]) - total_margin;

	btScalar dis1 = bt_distance_point_plane(m_plane, other.m_vertices[1]) - total_margin;

	btScalar dis2 = bt_distance_point_plane(m_plane, other.m_vertices[2]) - total_margin;

	if (dis0 > 0.0f && dis1 > 0.0f && dis2 > 0.0f) return false;
	if (dis0 < 0.0f && dis1 < 0.0f && dis2 < 0.0f) return false;

	// classify points on this triangle
	dis0 = bt_distance_point_plane(other.m_plane, m_vertices[0]) - total_margin;

	dis1 = bt_distance_point_plane(other.m_plane, m_vertices[1]) - total_margin;

	dis2 = bt_distance_point_plane(other.m_plane, m_vertices[2]) - total_margin;

	if (dis0 > 0.0f && dis1 > 0.0f && dis2 > 0.0f) return false;
	if (dis0 < 0.0f && dis1 < 0.0f && dis2 < 0.0f) return false;

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

bool btPrimitiveTriangle::find_triangle_collision_vrut_method(const btPrimitiveTriangle& other, GIM_TRIANGLE_CONTACT& contacts)
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
