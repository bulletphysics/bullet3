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
	public class FilteredCallback : ITriangleIndexCallback
	{
		private ITriangleCallback _callback;
		private Vector3 _aabbMin;
		private Vector3 _aabbMax;

		public FilteredCallback(ITriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax)
		{
			_callback = callback;
			_aabbMin = aabbMin;
			_aabbMax = aabbMax;
		}

		public ITriangleCallback TriangleCallback { get { return _callback; } set { _callback = value; } }
		public Vector3 AabbMin { get { return _aabbMin; } set { _aabbMin = value; } }
		public Vector3 AabbMax { get { return _aabbMax; } set { _aabbMax = value; } }

		public void ProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
		{
			if (MathHelper.TestTriangleAgainstAabb2(triangle, _aabbMin, _aabbMax))
			{
				//check aabb in triangle-space, before doing this
				_callback.ProcessTriangle(triangle, partId, triangleIndex);
			}
		}
	}
}
