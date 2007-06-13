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
	/// <summary>
	/// SphereBoxCollisionAlgorithm  provides sphere-box collision detection.
	/// Other features are frame-coherency (persistent data) and collision response.
	/// </summary>
	public class SphereBoxCollisionAlgorithm : CollisionAlgorithm, IDisposable
	{
		private bool _ownManifold;
		private PersistentManifold _manifold;
		private bool _isSwapped;

		public SphereBoxCollisionAlgorithm(PersistentManifold manifold, CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo, CollisionObject collisionObjectA, CollisionObject collisionObjectB, bool isSwapped)
			: base(collisionAlgorithmConstructionInfo)
		{
			_ownManifold = false;
			_manifold = manifold;
			_isSwapped = isSwapped;

			CollisionObject sphereObject = _isSwapped ? collisionObjectB : collisionObjectA;
			CollisionObject boxObject = _isSwapped ? collisionObjectA : collisionObjectB;

			if (_manifold == null && Dispatcher.NeedsCollision(sphereObject, boxObject))
			{
				_manifold = Dispatcher.GetNewManifold(sphereObject, boxObject);
				_ownManifold = true;
			}
		}

		public float GetSphereDistance(CollisionObject boxObject, out Vector3 pointOnBox, out Vector3 pointOnSphere, Vector3 sphereCenter, float radius)
		{
			pointOnBox = new Vector3();
			pointOnSphere = new Vector3();

			float margins;
			Vector3[] bounds = new Vector3[2];
			BoxShape boxShape = boxObject.CollisionShape as BoxShape;

			bounds[0] = -boxShape.HalfExtents;
			bounds[1] = boxShape.HalfExtents;

			margins = boxShape.Margin; //also add sphereShape margin?

			Matrix m44T = boxObject.WorldTransform;

			Vector3[] boundsVec = new Vector3[2];
			float penetration;

			boundsVec[0] = bounds[0];
			boundsVec[1] = bounds[1];

			Vector3 marginsVec = new Vector3(margins, margins, margins);

			// add margins
			bounds[0] += marginsVec;
			bounds[1] -= marginsVec;

			/////////////////////////////////////////////////

			Vector3 tmp, prel, normal, v3P;
			Vector3[] n = new Vector3[6];
			float sep = 10000000.0f, sepThis;

			n[0] = new Vector3(-1.0f, 0.0f, 0.0f);
			n[1] = new Vector3(0.0f, -1.0f, 0.0f);
			n[2] = new Vector3(0.0f, 0.0f, -1.0f);
			n[3] = new Vector3(1.0f, 0.0f, 0.0f);
			n[4] = new Vector3(0.0f, 1.0f, 0.0f);
			n[5] = new Vector3(0.0f, 0.0f, 1.0f);

			// convert  point in local space
			prel = MathHelper.InvXForm(m44T, sphereCenter);

			bool found = false;

			v3P = prel;

			for (int i = 0; i < 6; i++)
			{
				int j = i < 3 ? 0 : 1;
				if ((sepThis = (Vector3.Dot(v3P - bounds[j], n[i]))) > 0.0f)
				{
					v3P = v3P - n[i] * sepThis;
					found = true;
				}
			}

			//

			if (found)
			{
				bounds[0] = boundsVec[0];
				bounds[1] = boundsVec[1];

				normal = Vector3.Normalize(prel - v3P);
				pointOnBox = v3P + normal * margins;
				pointOnSphere = prel - normal * radius;

				if ((Vector3.Dot(pointOnSphere - pointOnBox, normal)) > 0.0f)
				{
					return 1.0f;
				}

				// transform back in world space
				tmp = MathHelper.MatrixToVector(m44T, pointOnBox);
				pointOnBox = tmp;
				tmp = MathHelper.MatrixToVector(m44T, pointOnSphere);
				pointOnSphere = tmp;
				float seps2 = (pointOnBox - pointOnSphere).LengthSquared();

				//if this fails, fallback into deeper penetration case, below
				if (seps2 > MathHelper.Epsilon)
				{
					sep = -(float)Math.Sqrt(seps2);
					normal = (pointOnBox - pointOnSphere);
					normal *= 1f / sep;
				}
				return sep;
			}

			//////////////////////////////////////////////////
			// Deep penetration case

			penetration = GetSpherePenetration(boxObject, ref pointOnBox, ref pointOnSphere, sphereCenter, radius, bounds[0], bounds[1]);

			bounds[0] = boundsVec[0];
			bounds[1] = boundsVec[1];

			if (penetration <= 0.0f)
				return (penetration - margins);
			else
				return 1.0f;
		}

		public float GetSpherePenetration(CollisionObject boxObject, ref Vector3 pointOnBox, ref Vector3 pointOnSphere, Vector3 sphereCenter, float radius, Vector3 aabbMin, Vector3 aabbMax)
		{
			Vector3[] bounds = new Vector3[2];

			bounds[0] = aabbMin;
			bounds[1] = aabbMax;

			Vector3 p0 = new Vector3(), tmp, prel, normal = new Vector3();
			Vector3[] n = new Vector3[6];
			float sep = -10000000.0f, sepThis;

			n[0] = new Vector3(-1.0f, 0.0f, 0.0f);
			n[1] = new Vector3(0.0f, -1.0f, 0.0f);
			n[2] = new Vector3(0.0f, 0.0f, -1.0f);
			n[3] = new Vector3(1.0f, 0.0f, 0.0f);
			n[4] = new Vector3(0.0f, 1.0f, 0.0f);
			n[5] = new Vector3(0.0f, 0.0f, 1.0f);

			Matrix m44T = boxObject.WorldTransform;

			// convert point in local space
			prel = MathHelper.InvXForm(m44T, sphereCenter);

			///////////

			for (int i = 0; i < 6; i++)
			{
				int j = i < 3 ? 0 : 1;
				if ((sepThis = (Vector3.Dot(prel - bounds[j], n[i])) - radius) > 0.0f) return 1.0f;
				if (sepThis > sep)
				{
					p0 = bounds[j];
					normal = n[i];
					sep = sepThis;
				}
			}

			pointOnBox = prel - normal * (Vector3.Dot(normal, (prel - p0)));
			pointOnSphere = pointOnBox + normal * sep;

			// transform back in world space
			tmp = MathHelper.MatrixToVector(m44T, pointOnBox);
			pointOnBox = tmp;
			tmp = MathHelper.MatrixToVector(m44T, pointOnSphere);
			pointOnSphere = tmp;
			normal = Vector3.Normalize(pointOnBox - pointOnSphere);

			return sep;
		}

		public override void ProcessCollision(CollisionObject bodyA, CollisionObject bodyB, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
		{
			if (_manifold == null)
				return;

			CollisionObject sphereObject = _isSwapped ? bodyB : bodyA;
			CollisionObject boxObject = _isSwapped ? bodyA : bodyB;

			SphereShape sphereA = sphereObject.CollisionShape as SphereShape;

			Vector3 pOnBox, pOnSphere;
			Vector3 sphereCenter = sphereObject.WorldTransform.Translation;
			float radius = sphereA.Radius;

			float dist = GetSphereDistance(boxObject, out pOnBox, out pOnSphere, sphereCenter, radius);

			if (dist < MathHelper.Epsilon)
			{
				Vector3 normalOnSurfaceB = Vector3.Normalize(pOnBox - pOnSphere);

				// report a contact. internally this will be kept persistent, and contact reduction is done
				resultOut.SetPersistentManifold(_manifold);
				resultOut.AddContactPoint(normalOnSurfaceB, pOnBox, dist);
			}
		}

		public override float CalculateTimeOfImpact(CollisionObject collisionObjectA, CollisionObject collisionObjectB, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
		{
			//not yet
			return 1;
		}

		public class CreateFunc : CollisionAlgorithmCreateFunction
		{
			public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo, CollisionObject bodyA, CollisionObject bodyB)
			{
				if (!IsSwapped)
					return new SphereBoxCollisionAlgorithm(null, collisionAlgorithmConstructionInfo, bodyA, bodyB, false);
				else
					return new SphereBoxCollisionAlgorithm(null, collisionAlgorithmConstructionInfo, bodyA, bodyB, true);
			}
		}

		#region IDisposable Members
		public void Dispose()
		{
			Dispose(true);
		}

		public void Dispose(bool disposing)
		{
			if (disposing && _ownManifold)
			{
				if (_manifold != null)
					Dispatcher.ReleaseManifold(_manifold);
			}
		}
		#endregion
	}
}
