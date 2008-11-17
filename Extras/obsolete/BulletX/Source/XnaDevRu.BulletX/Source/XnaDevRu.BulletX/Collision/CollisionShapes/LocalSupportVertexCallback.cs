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
	public class LocalSupportVertexCallback : ITriangleIndexCallback
	{
		private Vector3 _supportVertexLocal;
		private float _maxDot;
		private Vector3 _supportVecLocal;

		public LocalSupportVertexCallback(Vector3 supportVecLocal)
		{
			_supportVertexLocal = new Vector3();
			_maxDot = -1e30f;
			_supportVecLocal = supportVecLocal;
		}

		public float MaxDot { get { return _maxDot; } set { _maxDot = value; } }
		public Vector3 SupportVertexLocal { get { return _supportVecLocal; } set { _supportVecLocal = value; } }

		public void ProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
		{
			for (int i = 0; i < 3; i++)
			{
				float dot = Vector3.Dot(_supportVecLocal, triangle[i]);
				if (dot > _maxDot)
				{
					_maxDot = dot;
					_supportVertexLocal = triangle[i];
				}
			}
		}
	}
}
