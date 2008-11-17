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
	public class SphereTriangleDetector : DiscreteCollisionDetectorInterface
	{
		private SphereShape _sphere;
		private TriangleShape _triangle;
		private const int MaxOverlap = 0;

		public SphereTriangleDetector(SphereShape sphere, TriangleShape triangle)
		{
			this._sphere = sphere;
			this._triangle = triangle;
		}

		public override void GetClosestPoints(DiscreteCollisionDetectorInterface.ClosestPointInput input, DiscreteCollisionDetectorInterface.Result output, IDebugDraw debugDraw)
		{
			Matrix transformA = input.TransformA;
			Matrix transformB = input.TransformB;

			Vector3 point = new Vector3();
			Vector3 normal = new Vector3();
			Single timeOfImpact = 1.0f;
			Single depth = 0.0f;

			//move sphere into triangle space
			Matrix sphereInTr = MathHelper.InverseTimes(transformB, transformA);

			if (Collide(sphereInTr.Translation, point, normal, depth, timeOfImpact))
				output.AddContactPoint(Vector3.TransformNormal(normal, transformB), Vector3.TransformNormal(point, transformB), depth);
		}

		/// <summary>
		/// See also geometrictools.com
		/// Basic idea: D = |p - (lo + t0*lv)| where t0 = lv . (p - lo) / lv . lv
		/// </summary>
		/// <param name="from"></param>
		/// <param name="to"></param>
		/// <param name="p"></param>
		/// <param name="nearest"></param>
		/// <returns></returns>
		private float SegmentSquareDistance(Vector3 from, Vector3 to, Vector3 point, Vector3 nearest)
		{
			Vector3 diff = point - from;
			Vector3 v = to - from;
			float t = Vector3.Dot(v, diff);

			if (t > 0)
			{
				float dotVV = Vector3.Dot(v, v);
				if (t < dotVV)
				{
					t /= dotVV;
					diff -= t * v;
				}
				else
				{
					t = 1;
					diff -= v;
				}
			}
			else
				t = 0;

			nearest = from + t * v;
			return Vector3.Dot(diff, diff);
		}

		private bool Collide(Vector3 sphereCenter, Vector3 point, Vector3 resultNormal, float depth, float timeOfImpact)
		{
			Vector3[] vertices = _triangle.Vertices;
			Vector3 c = sphereCenter;
			float r = _sphere.Radius;

			Vector3 delta = new Vector3();

			Vector3 normal = Vector3.Cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
			normal = Vector3.Normalize(normal);
			Vector3 p1ToCentre = c - vertices[0];
			float distanceFromPlane = Vector3.Dot(p1ToCentre, normal);

			if (distanceFromPlane < 0)
			{
				//triangle facing the other way
				distanceFromPlane *= -1;
				normal *= -1;
			}

			float contactMargin = PersistentManifold.ContactBreakingThreshold;
			bool isInsideContactPlane = distanceFromPlane < r + contactMargin;
			bool isInsideShellPlane = distanceFromPlane < r;

			float deltaDotNormal = Vector3.Dot(delta, normal);
			if (!isInsideShellPlane && deltaDotNormal >= 0.0f)
				return false;

			// Check for contact / intersection
			bool hasContact = false;
			Vector3 contactPoint = new Vector3();
			if (isInsideContactPlane)
			{
				if (FaceContains(c, vertices, normal))
				{
					// Inside the contact wedge - touches a point on the shell plane
					hasContact = true;
					contactPoint = c - normal * distanceFromPlane;
				}
				else
				{
					// Could be inside one of the contact capsules
					float contactCapsuleRadiusSqr = (r + contactMargin) * (r + contactMargin);
					Vector3 nearestOnEdge = new Vector3();
					for (int i = 0; i < _triangle.EdgeCount; i++)
					{
						Vector3 pa, pb;
						_triangle.GetEdge(i, out pa, out pb);

						float distanceSqr = SegmentSquareDistance(pa, pb, c, nearestOnEdge);
						if (distanceSqr < contactCapsuleRadiusSqr)
						{
							// Yep, we're inside a capsule
							hasContact = true;
							contactPoint = nearestOnEdge;
						}
					}
				}
			}

			if (hasContact)
			{
				Vector3 contactToCentre = c - contactPoint;
				float distanceSqr = contactToCentre.LengthSquared();
				if (distanceSqr < (r - MaxOverlap) * (r - MaxOverlap))
				{
					float distance = (float)Math.Sqrt(distanceSqr);
					resultNormal = contactToCentre;
					resultNormal = Vector3.Normalize(resultNormal);
					point = contactPoint;
					depth = -(r - distance);
					return true;
				}

				if (Vector3.Dot(delta, contactToCentre) >= 0.0f)
					return false;

				// Moving towards the contact point -> collision
				point = contactPoint;
				timeOfImpact = 0.0f;
				return true;
			}
			return false;
		}

		private bool PointInTriangle(Vector3[] vertices, Vector3 normal, Vector3 p)
		{
			Vector3 p1 = vertices[0];
			Vector3 p2 = vertices[1];
			Vector3 p3 = vertices[2];

			Vector3 edge1 = p2 - p1;
			Vector3 edge2 = p3 - p2;
			Vector3 edge3 = p1 - p3;

			Vector3 p1ToP = p - p1;
			Vector3 p2ToP = p - p2;
			Vector3 p3ToP = p - p3;

			Vector3 edge1Normal = Vector3.Cross(edge1, normal);
			Vector3 edge2Normal = Vector3.Cross(edge2, normal);
			Vector3 edge3Normal = Vector3.Cross(edge3, normal);

			float r1, r2, r3;
			r1 = Vector3.Dot(edge1Normal, p1ToP);
			r2 = Vector3.Dot(edge2Normal, p2ToP);
			r3 = Vector3.Dot(edge3Normal, p3ToP);
			if ((r1 > 0 && r2 > 0 && r3 > 0) ||
				 (r1 <= 0 && r2 <= 0 && r3 <= 0))
				return true;
			return false;
		}

		private bool FaceContains(Vector3 p, Vector3[] vertices, Vector3 normal)
		{
			Vector3 lp = p;
			Vector3 lnormal = normal;
			return PointInTriangle(vertices, lnormal, lp);
		}
	}
}