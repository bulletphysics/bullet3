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
    /// CastResult stores the closest result
    /// alternatively, add a callback method to decide about closest/all results
    /// </summary>
	public class CastResult
	{
		private Vector3 _normal;
		private float _fraction;
		private Matrix _hitTransformA;
		private Matrix _hitTransformB;
		private IDebugDraw _debugDrawer;

		public CastResult()
		{
			_fraction = 1e30f;
		}

		public Vector3 Normal { get { return _normal; } set { _normal = value; } }
		public float Fraction { get { return _fraction; } set { _fraction = value; } }
		public Matrix HitTransformA { get { return _hitTransformA; } set { _hitTransformA = value; } }
		public Matrix HitTransformB { get { return _hitTransformB; } set { _hitTransformB = value; } }
		public IDebugDraw DebugDrawer { get { return _debugDrawer; } set { _debugDrawer = value; } }

		public virtual void DebugDraw(float fraction) { }
		public virtual void DrawCoordSystem(Matrix trans) { }
	}

	/// <summary>
    /// ConvexCast is an interface for Casting
	/// </summary>
	public interface IConvexCast
	{
		/// <summary>
        /// cast a convex against another convex object
		/// </summary>
		/// <param name="fromA"></param>
		/// <param name="toA"></param>
		/// <param name="fromB"></param>
		/// <param name="toB"></param>
		/// <param name="result"></param>
		/// <returns></returns>
		bool CalcTimeOfImpact(Matrix fromA, Matrix toA, Matrix fromB, Matrix toB, CastResult result);
	}
}
