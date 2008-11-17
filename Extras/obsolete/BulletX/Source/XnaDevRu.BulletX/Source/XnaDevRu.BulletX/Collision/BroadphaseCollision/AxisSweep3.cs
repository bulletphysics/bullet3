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
    public class AxisSweep3: OverlappingPairCache
    {
        Vector3 _worldAabbMin;
        Vector3 _worldAabbMax;

        Vector3 _quantize;

        int _numHandles;
        int _maxHandles;

        Handle[] _handles;
        Edge[][] _edges = new Edge[3][];

        ushort _firstFreeHandle;

		int _invalidPair;

        public AxisSweep3(Vector3 worldAabbMin, Vector3 worldAabbMax, int maxHandles)
            : base()
        {
            BulletDebug.Assert(maxHandles > 1 && maxHandles < 32767);

            // init bounds
            _worldAabbMin = worldAabbMin;
            _worldAabbMax = worldAabbMax;

            Vector3 aabbSize = _worldAabbMax - _worldAabbMin;
            _quantize = new Vector3(65535.0f, 65535.0f, 65535.0f) / aabbSize;

            // allocate handles buffer and put all handles on free list
            _handles = new Handle[maxHandles];
            for (int i = 0; i < maxHandles; i++)
                _handles[i] = new Handle();
            _maxHandles = maxHandles;
            _numHandles = 0;

            // handle 0 is reserved as the null index, and is also used as the sentinel
            _firstFreeHandle = 1;
            {
                for (int i = _firstFreeHandle; i < maxHandles; i++)
                {
                    _handles[i].NextFree = (ushort)(i + 1);
                }
                _handles[maxHandles - 1].NextFree = 0;
            }

            {
                // allocate edge buffers
                for (int i = 0; i < 3; i++)
                {
                    _edges[i] = new Edge[maxHandles * 2];
                    for (int j = 0; j < maxHandles * 2; j++)
                    {
                        _edges[i][j] = new Edge();
                    }
                }
            }
            //removed overlap management

            // make boundary sentinels

            _handles[0].ClientData = 0;

            for (int axis = 0; axis < 3; axis++)
            {
                _handles[0].MinEdges[axis] = 0;
                _handles[0].MaxEdges[axis] = 1;

                _edges[axis][0].Position = 0;
                _edges[axis][0].Handle = 0;
                _edges[axis][1].Position = 0xffff;
                _edges[axis][1].Handle = 0;
            }
        }

        public ushort AddHandle(Vector3 aabbMin, Vector3 aabbMax, object owner, BroadphaseProxy.CollisionFilterGroups collisionFilterGroup, BroadphaseProxy.CollisionFilterGroups collisionFilterMask)
        {
            ushort[] min = new ushort[3], max = new ushort[3];
            Quantize(out min, aabbMin, 0);
            Quantize(out max, aabbMax, 1);

            ushort handle = AllocateHandle();
            Handle oHandle = GetHandle(handle);

            oHandle.HandleID = handle;
            oHandle.ClientData = owner;
            oHandle.CollisionFilterGroup = collisionFilterGroup;
            oHandle.CollisionFilterMask = collisionFilterMask;

            int limit = _numHandles * 2;

            // (Gluk    )
            //  ( Inside )
            for (int axis = 0; axis < 3; axis++)
            {
                _handles[0].MaxEdges[axis] += 2;

                _edges[axis][limit + 1].Position = _edges[axis][limit - 1].Position;
                _edges[axis][limit + 1].Handle = _edges[axis][limit - 1].Handle;

                _edges[axis][limit - 1].Position = min[axis];
                _edges[axis][limit - 1].Handle = handle;

                _edges[axis][limit].Position = max[axis];
                _edges[axis][limit].Handle = handle;

                oHandle.MinEdges[axis] = (ushort)(limit - 1);
                oHandle.MaxEdges[axis] = (ushort)limit;
            }

            SortMinDown(0, oHandle.MinEdges[0], false);
            SortMaxDown(0, oHandle.MaxEdges[0], false);
            SortMinDown(1, oHandle.MinEdges[1], false);
            SortMaxDown(1, oHandle.MaxEdges[1], false);
            SortMinDown(2, oHandle.MinEdges[2], true);
            SortMaxDown(2, oHandle.MaxEdges[2], true);

            return handle;
        }

        public void RemoveHandle(ushort handle)
        {
            Handle pHandle = GetHandle(handle);

            //explicitly remove the pairs containing the proxy
            //we could do it also in the sortMinUp (passing true)
            //todo: compare performance
            RemoveOverlappingPairsContainingProxy(pHandle);


            // compute current limit of edge arrays
            int limit = _numHandles * 2;
            int axis;

            for (axis = 0; axis < 3; axis++)
            {
                _handles[0].MaxEdges[axis] -= 2;
            }

            // remove the edges by sorting them up to the end of the list
            for (axis = 0; axis < 3; axis++)
            {
                Edge[] pEdges = _edges[axis];
                ushort max = pHandle.MaxEdges[axis];
                pEdges[max].Position = 0xffff;

                SortMaxUp(axis, max, false);

                ushort i = pHandle.MinEdges[axis];
                pEdges[i].Position = 0xffff;

                SortMinUp(axis, i, false);

                pEdges[limit - 1].Handle = 0;
                pEdges[limit - 1].Position = 0xffff;
            }

            // free the handle
            FreeHandle(handle);            
        }

		public override void ProcessAllOverlappingPairs(IOverlapCallback callback)
		{
			OverlappingPairs.Sort(new Comparison<BroadphasePair>(BroadphasePair.ComparisonSort));

			if (_invalidPair != 0)
				OverlappingPairs.RemoveRange(OverlappingPairs.Count - _invalidPair, _invalidPair);
			_invalidPair = 0;

			BroadphasePair previousPair = new BroadphasePair();
			previousPair.ProxyA = null;
			previousPair.ProxyB = null;
			previousPair.CollisionAlgorithm = null;

			List<BroadphasePair> removal = new List<BroadphasePair>();

			for (int i = 0; i < OverlappingPairs.Count; i++)
			{
				bool isDuplicate = (OverlappingPairs[i] == previousPair);
				previousPair = OverlappingPairs[i];
				bool needsRemoval;
				if (!isDuplicate)
				{
					bool hasOverlap = TestOverlap(previousPair.ProxyA, previousPair.ProxyB);
					if (hasOverlap)
					{
						needsRemoval = callback.ProcessOverlap(ref previousPair);
					}
					else
					{
						needsRemoval = true;
					}
				}
				else
				{
					needsRemoval = true;
					BulletDebug.Assert(previousPair.CollisionAlgorithm == null);
				}

				if (needsRemoval)
				{
					removal.Add(previousPair);
				}
			}

			for (int i = 0; i < removal.Count; i++)
			{
				BroadphasePair pair = removal[i];
				CleanOverlappingPair(ref pair);
				pair.ProxyA = null;
				pair.ProxyB = null;
				_invalidPair++;
				OverlappingPairCount--;
			}
		}

		private bool TestOverlap(BroadphaseProxy proxyA, BroadphaseProxy proxyB)
		{
			if (proxyA == null || proxyB == null)
				return false;

			Handle handleA = proxyA as Handle;
			Handle handleB = proxyB as Handle;

			for (int axis = 0; axis < 3; axis++)
			{
				if (handleA.MaxEdges[axis] < handleB.MinEdges[axis] ||
					handleB.MaxEdges[axis] < handleA.MinEdges[axis])
				{
					return false;
				}
			}
			return true;
		}

		private bool TestOverlap(int ignoreAxis, Handle pHandleA, Handle pHandleB)
        {
	        for (int axis = 0; axis < 3; axis++)
	        { 
		        if (axis != ignoreAxis)
		        {
                    if (pHandleA.MaxEdges[axis] < pHandleB.MinEdges[axis] ||
                        pHandleB.MaxEdges[axis] < pHandleA.MinEdges[axis]) 
			        { 
				        return false; 
			        } 
		        }
	        } 

	        return true;
        }

        private ushort AllocateHandle() 
        {
            ushort handle = _firstFreeHandle;
            _firstFreeHandle = GetHandle(handle).NextFree;
            _numHandles++;

            return handle;
        }

        private void FreeHandle(ushort handle)
        {
            BulletDebug.Assert(handle > 0 && handle < _maxHandles);

            GetHandle(handle).NextFree = _firstFreeHandle;
            _firstFreeHandle = handle;

            _numHandles--;            
        }

        private Handle GetHandle(ushort handle)
        {
            return _handles[handle];
        }

        private void UpdateHandle(ushort handle, Vector3 aabbMin, Vector3 aabbMax)
        {
	        Handle pHandle = GetHandle(handle);

	        // quantize the new bounds
	        ushort[] min = new ushort[3];
            ushort[] max = new ushort[3];
	        Quantize(out min, aabbMin, 0);
	        Quantize(out max, aabbMax, 1);

	        // update changed edges
	        for (int axis = 0; axis < 3; axis++)
	        {
		        ushort emin = pHandle.MinEdges[axis];
                ushort emax = pHandle.MaxEdges[axis];

		        int dmin = (int)min[axis] - (int)_edges[axis][emin].Position;
		        int dmax = (int)max[axis] - (int)_edges[axis][emax].Position;

		        _edges[axis][emin].Position = min[axis];
                _edges[axis][emax].Position = max[axis];

		        // expand (only adds overlaps)
		        if (dmin < 0)
			        SortMinDown(axis, emin, true);

		        if (dmax > 0)
                    SortMaxUp(axis, emax, true);

		        // shrink (only removes overlaps)
		        if (dmin > 0)
                    SortMinUp(axis, emin, true);

		        if (dmax < 0)
                    SortMaxDown(axis, emax, true);
	        }
        }

        private void Quantize(out ushort[] result, Vector3 point, int isMax)
        {
            Vector3 clampedPoint = new Vector3(
                point.X,
                point.Y,
                point.Z
            );

            MathHelper.SetMax(ref clampedPoint, _worldAabbMin);
            MathHelper.SetMin(ref clampedPoint, _worldAabbMax);

            Vector3 v = (clampedPoint - _worldAabbMin) * _quantize;

            result = new ushort[3];
            result[0] = (ushort)(((int)v.X & 0xfffe) | isMax);
            result[1] = (ushort)(((int)v.Y & 0xfffe) | isMax);
            result[2] = (ushort)(((int)v.Z & 0xfffe) | isMax);
        }

        private void SortMinDown(int axis, ushort edge, bool updateOverlaps)
        {
            Edge pEdge = _edges[axis][edge];
            Edge pPrev = _edges[axis][edge - 1];
            Handle pHandleEdge = GetHandle(pEdge.Handle);

            while (pEdge.Position < pPrev.Position)
            {
                Handle pHandlePrev = GetHandle(pPrev.Handle);

                if (pPrev.IsMax())
                {
                    // if previous edge is a maximum check the bounds and add an overlap if necessary
                    if (updateOverlaps && TestOverlap(axis, pHandleEdge, pHandlePrev))
                    {
                        AddOverlappingPair(pHandleEdge, pHandlePrev);
                    }

                    // update edge reference in other handle
                    pHandlePrev.MaxEdges[axis]++;
                }
                else
                    pHandlePrev.MinEdges[axis]++;

                pHandleEdge.MinEdges[axis]--;

                // swap the edges
                pEdge.Swap(ref pPrev);

                // decrement
                edge--;
                pEdge = _edges[axis][edge];
                pPrev = _edges[axis][edge - 1];
            }
        }

        private void SortMinUp(int axis, ushort edge, bool updateOverlaps)
        {
            Edge pEdge = _edges[axis][edge];
            Edge pNext = _edges[axis][edge + 1];
	        Handle pHandleEdge = GetHandle(pEdge.Handle);

            while ((pNext.Handle != 0) && (pEdge.Position >= pNext.Position))
	        {
		        Handle pHandleNext = GetHandle(pNext.Handle);

		        if (pNext.IsMax())
		        {
			        // if next edge is maximum remove any overlap between the two handles
			        if (updateOverlaps)
			        {
						//Handle handle0 = GetHandle(pEdge.Handle);
						//Handle handle1 = GetHandle(pNext.Handle);
						//BroadphasePair tmpPair = new BroadphasePair(handle0, handle1);
						//RemoveOverlappingPair(tmpPair);
			        }

			        // update edge reference in other handle
			        pHandleNext.MaxEdges[axis]--;
		        }
		        else
                    pHandleNext.MinEdges[axis]--;

                pHandleEdge.MinEdges[axis]++;

                // swap the edges
                pEdge.Swap(ref pNext);

                // increment
                edge++;
                pEdge = _edges[axis][edge];
                pNext = _edges[axis][edge + 1];
	        }
        }

        private void SortMaxDown(int axis, ushort edge, bool updateOverlaps)
        {
            Edge pEdge = _edges[axis][edge];
            Edge pPrev = _edges[axis][edge - 1];
            Handle pHandleEdge = GetHandle(pEdge.Handle);

            while (pEdge.Position < pPrev.Position)
            {
                Handle pHandlePrev = GetHandle(pPrev.Handle);

                if (!pPrev.IsMax())
                {
                    // if previous edge was a minimum remove any overlap between the two handles
                    if (updateOverlaps)
                    {
						//this is done during the overlappingpairarray iteration/narrowphase collision
						//Handle handle0 = GetHandle(pEdge.Handle);
						//Handle handle1 = GetHandle(pPrev.Handle);
						//BroadphasePair pair = FindPair(handle0, handle1);

						//if (pair != null)
						//{
						//    RemoveOverlappingPair(pair);
						//}
                    }

                    // update edge reference in other handle
                    pHandlePrev.MinEdges[axis]++; ;
                }
                else
                    pHandlePrev.MaxEdges[axis]++;

                pHandleEdge.MaxEdges[axis]--;

                // swap the edges
                pEdge.Swap(ref pPrev);

                // decrement
                edge--;
                pEdge = _edges[axis][edge];
                pPrev = _edges[axis][edge - 1];
            }
        }

        private void SortMaxUp(int axis, ushort edge, bool updateOverlaps) 
        {
            Edge pEdge = _edges[axis][edge];
            Edge pNext = _edges[axis][edge + 1];
            Handle pHandleEdge = GetHandle(pEdge.Handle);

            while ((pNext.Handle!=0) && (pEdge.Position >= pNext.Position))
            {
                Handle pHandleNext = GetHandle(pNext.Handle);

                if (!pNext.IsMax())
                {
                    // if next edge is a minimum check the bounds and add an overlap if necessary
                    if (updateOverlaps && TestOverlap(axis, pHandleEdge, pHandleNext))
                    {
                        Handle handle0 = GetHandle(pEdge.Handle);
                        Handle handle1 = GetHandle(pNext.Handle);
                        AddOverlappingPair(handle0, handle1);
                    }

                    // update edge reference in other handle
                    pHandleNext.MinEdges[axis]--;
                }
                else
                    pHandleNext.MaxEdges[axis]--;

                pHandleEdge.MaxEdges[axis]++;

                // swap the edges
                pEdge.Swap(ref pNext);

                // increment
                edge++;
                pEdge = _edges[axis][edge];
                pNext = _edges[axis][edge + 1];
            }
        }

        #region Abstract

        public override void RefreshOverlappingPairs()
        {
        }

        public override BroadphaseProxy CreateProxy(Vector3 min, Vector3 max, BroadphaseNativeTypes shapeType, object userData, BroadphaseProxy.CollisionFilterGroups collisionFilterGroup, BroadphaseProxy.CollisionFilterGroups collisionFilterMask)
        {
		    ushort handleId = AddHandle(min, max, userData, collisionFilterGroup, collisionFilterMask);
    		
		    Handle handle = GetHandle(handleId);
    				
		    return handle;
        }

        public override void DestroyProxy(BroadphaseProxy proxy)
        {
            Handle handle = proxy as Handle;
            RemoveHandle(handle.HandleID);
        }

        public override void SetAabb(BroadphaseProxy proxy, Vector3 aabbMin, Vector3 aabbMax)
        {
            Handle handle = proxy as Handle;
            UpdateHandle(handle.HandleID, aabbMin, aabbMax);
        }
        #endregion
    }

    public class Edge
    {
        ushort position;
        ushort handle;

        public ushort Position
        {
            get { return position; }
            set { position = value; }
        }

        public ushort Handle
        {
            get { return handle; }
            set { handle = value; }
        }

        public bool IsMax()
        {
            return (position & (ushort)1) == 1;
        }

        public void Swap(ref Edge e)
        {
            ushort tmpPosition = this.position;
            ushort tmpHandle = this.handle;
            this.position = e.position;
            this.handle = e.handle;
            e.position = tmpPosition;
            e.handle = tmpHandle;
        }
    }

    public class Handle: BroadphaseProxy
    {
        ushort[] minEdges, maxEdges;
        ushort pad;
        ushort handleID;

        public ushort[] MinEdges
        {
            get { return minEdges; }
            set { minEdges = value; }
        }

        public ushort[] MaxEdges
        {
            get { return maxEdges; }
            set { maxEdges = value; }
        }

        public ushort HandleID
        {
            get { return handleID; }
            set { handleID = value; }
        }

        public ushort Pad
        {
            get { return pad; }
            set { pad = value; }
        }

        public ushort NextFree
        {
            get { return minEdges[0]; }
            set { minEdges[0] = value;}
        }

        public Handle()
        {
            minEdges = new ushort[3];
            maxEdges = new ushort[3];
        }
    }
}
