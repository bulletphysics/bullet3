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
	/// ISimplexSolver can incrementally calculate distance between origin and up to 4 vertices
	/// Used by GJK or Linear Casting. Can be implemented by the Johnson-algorithm or alternative approaches based on
	/// voronoi regions or barycentric coordinates
	public interface ISimplexSolver
	{
		void Reset();
		void AddVertex(Vector3 w, Vector3 p, Vector3 q);
		bool Closest(out Vector3 v);

		int GetSimplex(out Vector3[] pBuf, out Vector3[] qBuf, out Vector3[] yBuf);
		bool InSimplex(Vector3 w);
		void BackupClosest(out Vector3 v);
		void ComputePoints(out Vector3 pA, out Vector3 pB);

		int NumVertices { get;}
		bool EmptySimplex { get;}
		float MaxVertex { get;}
		bool FullSimplex { get;}
	}
}