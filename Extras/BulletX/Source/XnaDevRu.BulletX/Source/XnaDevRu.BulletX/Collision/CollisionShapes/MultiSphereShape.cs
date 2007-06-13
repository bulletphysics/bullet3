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
	/// MultiSphereShape represents implicit convex hull of a collection of spheres (using getSupportingVertex)
	/// </summary>
	public class MultiSphereShape : ConvexShape
	{
		private const int _maxNumSpheres = 5;
		private Vector3[] _localPositions = new Vector3[MaxNumSpheres];
		private float[] _radi = new float[MaxNumSpheres];
		private Vector3 _inertiaHalfExtents;

		private int m_numSpheres;

		public MultiSphereShape(Vector3 inertiaHalfExtents, Vector3[] positions, float[] radi, int numSpheres)
		{
			_inertiaHalfExtents = inertiaHalfExtents;
			float startMargin = 1e30f;

			m_numSpheres = numSpheres;
			for (int i = 0; i < m_numSpheres; i++)
			{
				_localPositions[i] = positions[i];
				_radi[i] = radi[i];
				if (radi[i] < startMargin)
					startMargin = radi[i];
			}
			Margin = startMargin;
		}

		public static int MaxNumSpheres { get { return _maxNumSpheres; } }

		public override BroadphaseNativeTypes ShapeType
		{
			get
			{
				return BroadphaseNativeTypes.MultiSphere;
			}
		}

		public override string Name
		{
			get
			{
				return "MultiSphere";
			}
		}

		public override Vector3 LocalGetSupportingVertexWithoutMargin(Vector3 vecA)
		{
			Vector3 supVec = new Vector3();

			float maxDot = -1e30f;


			Vector3 vec = vecA;
			float lenSqr = vec.LengthSquared();
			if (lenSqr < 0.0001f)
			{
				vec = new Vector3(1, 0, 0);
			}
			else
			{
				float rlen = 1f / (float)Math.Sqrt(lenSqr);
				vec *= rlen;
			}

			Vector3 vtx;
			float newDot;

			for (int i = 0; i < m_numSpheres; i++)
			{
				vtx = _localPositions[i] + vec * LocalScaling * _radi[i] - vec * Margin;
				newDot = Vector3.Dot(vec, vtx);
				if (newDot > maxDot)
				{
					maxDot = newDot;
					supVec = vtx;
				}
			}

			return supVec;
		}

		public override void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector3[] supportVerticesOut)
		{
			for (int j = 0; j < vectors.Length; j++)
			{
				float maxDot = -1e30f;
				Vector3 vtx;
				float newDot;

				for (int i = 0; i < m_numSpheres; i++)
				{
					vtx = _localPositions[i] + vectors[j] * LocalScaling * _radi[i] - vectors[j] * Margin;
					newDot = Vector3.Dot(vectors[j], vtx);
					if (newDot > maxDot)
					{
						maxDot = newDot;
						supportVerticesOut[j] = vtx;
					}
				}
			}
		}

		public override void CalculateLocalInertia(float mass, out Vector3 inertia)
		{
			//as an approximation, take the inertia of the box that bounds the spheres
			Matrix ident = Matrix.Identity;
			Vector3 halfExtents = _inertiaHalfExtents;

			float margin = ConvexDistanceMargin;

			float lx = 2f * (halfExtents.X + margin);
			float ly = 2f * (halfExtents.Y + margin);
			float lz = 2f * (halfExtents.Z + margin);
			float x2 = lx * lx;
			float y2 = ly * ly;
			float z2 = lz * lz;
			float scaledmass = mass * 0.08333333f;

			inertia = new Vector3();
			inertia.X = scaledmass * (y2 + z2);
			inertia.Y = scaledmass * (x2 + z2);
			inertia.Z = scaledmass * (x2 + y2);
		}
	}
}
