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
	/// PHY_ScalarType enumerates possible scalar types.
	/// See the StridingMeshInterface for its use
	/// </summary>
	public enum PHY_ScalarType
	{
		PHY_FLOAT,
		PHY_DOUBLE,
		PHY_INTEGER,
		PHY_SHORT,
		PHY_FIXEDPOINT88
	}

	/// <summary>
	/// StridingMeshInterface is the interface class for high performance access to triangle meshes
	/// It allows for sharing graphics and collision meshes. Also it provides locking/unlocking of graphics meshes that are in gpu memory.
	/// </summary>
	public abstract class StridingMeshInterface
	{
		protected Vector3 _scaling;

		public StridingMeshInterface()
		{
			_scaling = new Vector3(1f,1f,1f);
		}

		public void InternalProcessAllTriangles(ITriangleIndexCallback callback, Vector3 aabbMin, Vector3 aabbMax)
		{
			int numtotalphysicsverts = 0;
            int numtriangles, gfxindex;
			int part, graphicssubparts = SubPartsCount();
            Vector3[] triangle = new Vector3[3];
            List<Vector3> verts;
            List<int> indicies;

			Vector3 meshScaling = Scaling;

			//if the number of parts is big, the performance might drop due to the innerloop switch on indextype
			for (part = 0; part < graphicssubparts; part++)
			{
                GetLockedReadOnlyVertexIndexBase(out verts, out indicies, out numtriangles, part);
			    numtotalphysicsverts += numtriangles * 3; //upper bound

                for (gfxindex = 0; gfxindex < numtriangles; gfxindex++)
                {
                    triangle[0] = verts[indicies[gfxindex * 3 + 0]];
                    triangle[1] = verts[indicies[gfxindex * 3 + 1]];
                    triangle[2] = verts[indicies[gfxindex * 3 + 2]];

                    callback.ProcessTriangleIndex(triangle, part, gfxindex);
                }

			    UnLockReadOnlyVertexBase(part);
			}
		}


		// get read and write access to a subpart of a triangle mesh
		// this subpart has a continuous array of vertices and indices
		// in this way the mesh can be handled as chunks of memory with striding
		// very similar to OpenGL vertexarray support
		// make a call to unLockVertexBase when the read and write access is finished	
		public abstract void GetLockedVertexIndexBase(out List<Vector3> verts, out List<int> indicies, out int numfaces, int subpart);

        public abstract void GetLockedReadOnlyVertexIndexBase(out List<Vector3> verts, out List<int> indicies, out int numfaces, int subpart);
	
		// unLockVertexBase finishes the access to a subpart of the triangle mesh
		// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
		public abstract void UnLockVertexBase(int subpart);

		public abstract void UnLockReadOnlyVertexBase(int subpart);


		// getNumSubParts returns the number of seperate subparts
		// each subpart has a continuous array of vertices and indices
        public abstract int SubPartsCount();

		public abstract void PreallocateVertices(int numverts);
		public abstract void PreallocateIndices(int numindices);

		public Vector3 Scaling
        {           
			get { return _scaling; }
            set { _scaling = value; }
		}
	}
}
