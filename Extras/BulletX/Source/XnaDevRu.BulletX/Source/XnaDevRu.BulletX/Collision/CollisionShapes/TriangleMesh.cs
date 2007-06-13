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
    class TriangleMesh : StridingMeshInterface
    {
        int _numTriangles;
        List<Vector3> _verts;

        public TriangleMesh()
        {
            _numTriangles = 0;
            _verts = new List<Vector3>();
        }

        void AddTriangle(Vector3 vertex0, Vector3 vertex1, Vector3 vertex2)
        {
            _verts.Add(vertex0);
            _verts.Add(vertex1);
            _verts.Add(vertex2);
            _numTriangles++;
        }

        public override void GetLockedVertexIndexBase(out List<Vector3> verts, out List<int> indicies, out int numfaces, int subpart)
        {
            verts = new List<Vector3>();
            for (int i = 0; i < 3; i++)
            {
                verts.Add(_verts[subpart * 3 + i]);
            }
            indicies = new List<int>();
            indicies.Add(0);
            indicies.Add(1);
            indicies.Add(2);
            numfaces = 1;
        }

        public override void GetLockedReadOnlyVertexIndexBase(out List<Vector3> verts, out List<int> indicies, out int numfaces, int subpart)
        {
            verts = new List<Vector3>();
            for (int i = 0; i < 3; i++)
            {
                verts.Add(_verts[subpart * 3 + i]);
            }
            indicies = new List<int>();
            indicies.Add(0);
            indicies.Add(1);
            indicies.Add(2);
            numfaces = 1;
        }

        public override void UnLockVertexBase(int subpart)
        {

        }

        public override void UnLockReadOnlyVertexBase(int subpart)
        {

        }

        public override int SubPartsCount()
        {
            return _numTriangles;
        }

        public override void PreallocateVertices(int numverts)
        {

        }

        public override void PreallocateIndices(int numindices)
        {

        }
    }
}
