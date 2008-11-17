/*
  Bullet for XNA Copyright (c) 2003-2007 Vsevolod Klementjev http://www.codeplex.com/xnadevru
  Bullet original C++ version Copyright (c) 2003-2007 Erwin Coumans http://bulletphysics.com

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using System.Text;
using Microsoft.Xna.Framework;

namespace XnaDevRu.BulletX
{
	public abstract class TriangleRaycastCallback : ITriangleCallback
	{
		private Vector3 _from;
		private Vector3 _to;
		private float _hitFraction;

		public TriangleRaycastCallback(Vector3 from, Vector3 to)
		{
			_from = from;
			_to = to;
			_hitFraction = 1;
		}

		public Vector3 From { get { return _from; } set { _from = value; } }
		public Vector3 To { get { return _to; } set { _to = value; } }
		public float HitFraction { get { return _hitFraction; } set { _hitFraction = value; } }

		public abstract float ReportHit(Vector3 hitNormalLocal, float hitFraction, int partId, int triangleIndex);

		#region ITriangleCallback Members

		public void ProcessTriangle(Vector3[] triangle, int partID, int triangleIndex)
		{
			Vector3 vertA = triangle[0];
			Vector3 vertB = triangle[1];
			Vector3 vertC = triangle[2];

			Vector3 vBA = vertB - vertA;
			Vector3 vCA = vertC - vertA;

			Vector3 triangleNormal = Vector3.Cross(vBA, vCA);

			float dist = Vector3.Dot(vertA, triangleNormal);
			float distA = Vector3.Dot(triangleNormal, _from);
			distA -= dist;
			float distB = Vector3.Dot(triangleNormal, _to);
			distB -= dist;

			if (distA * distB >= 0.0f)
			{
				return; // same sign
			}

			float projLength = distA - distB;
			float distance = (distA) / (projLength);
			// Now we have the intersection point on the plane, we'll see if it's inside the triangle
			// Add an epsilon as a tolerance for the raycast,
			// in case the ray hits exacly on the edge of the triangle.
			// It must be scaled for the triangle size.

			if (distance < _hitFraction)
			{
				float edgeTolerance = triangleNormal.LengthSquared();
				edgeTolerance *= -0.0001f;
				Vector3 point = new Vector3();
				MathHelper.SetInterpolate3(_from, _to, distance, ref point);

				Vector3 vertexAPoint = vertA - point;
				Vector3 vertexBPoint = vertB - point;
				Vector3 contactPointA = Vector3.Cross(vertexAPoint, vertexBPoint);

				if (Vector3.Dot(contactPointA, triangleNormal) >= edgeTolerance)
				{
					Vector3 vertexCPoint = vertC - point;
					Vector3 contactPointB = Vector3.Cross(vertexBPoint, vertexCPoint);
					if (Vector3.Dot(contactPointB, triangleNormal) >= edgeTolerance)
					{
						Vector3 contactPointC = Vector3.Cross(vertexCPoint, vertexAPoint);

						if (Vector3.Dot(contactPointC, triangleNormal) >= edgeTolerance)
						{
							if (distA > 0)
							{
								_hitFraction = ReportHit(triangleNormal, distance, partID, triangleIndex);
							}
							else
							{
								_hitFraction = ReportHit(-triangleNormal, distance, partID, triangleIndex);
							}
						}
					}
				}
			}
		}
		#endregion
	}
}
