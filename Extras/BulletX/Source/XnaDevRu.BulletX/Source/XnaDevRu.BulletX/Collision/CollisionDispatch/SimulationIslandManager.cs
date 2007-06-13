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
	public class SimulationIslandManager
	{
		private UnionFind _unionFind = new UnionFind();

		public void InitUnionFind(int n)
		{
			_unionFind.Reset(n);
		}

		public UnionFind UnionFind { get { return _unionFind; } }

		public virtual void UpdateActivationState(CollisionWorld world, IDispatcher dispatcher)
		{
			InitUnionFind(world.CollisionObjectsCount);

			// put the index into m_controllers into m_tag	
			int index = 0;
			for (int i = 0; i < world.CollisionObjects.Count; i++)
			{
				world.CollisionObjects[i].IslandTag = index;
				world.CollisionObjects[i].HitFraction = 1;
				world.CollisionObjects[i].CompanionID = -1;
				index++;
			}
			// do the union find
			FindUnions(dispatcher);
		}

		public virtual void StoreIslandActivationState(CollisionWorld world)
		{
			// put the islandId ('find' value) into m_tag			
			int index = 0;
			for (int i = 0; i < world.CollisionObjects.Count; i++)
			{
				if (world.CollisionObjects[i].MergesSimulationIslands)
				{
					world.CollisionObjects[i].IslandTag = _unionFind.Find(index);
					world.CollisionObjects[i].CompanionID = -1;
				}
				else
				{
					world.CollisionObjects[i].IslandTag = -1;
					world.CollisionObjects[i].CompanionID = -2;
				}
				index++;
			}
		}

		public void FindUnions(IDispatcher dispatcher)
		{
			for (int i = 0; i < dispatcher.ManifoldCount; i++)
			{
				PersistentManifold manifold = dispatcher.GetManifoldByIndex(i);
				//static objects (invmass 0.f) don't merge !

				CollisionObject colObjA = manifold.BodyA as CollisionObject;
				CollisionObject colObjB = manifold.BodyB as CollisionObject;

				if (((colObjA != null) && (colObjA.MergesSimulationIslands)) &&
					((colObjB != null) && (colObjB.MergesSimulationIslands)))
				{
					_unionFind.Unite(colObjA.IslandTag, colObjB.IslandTag);
				}
			}
		}

		public void BuildAndProcessIslands(IDispatcher dispatcher, List<CollisionObject> collisionObjects, IIslandCallback callback)
		{
			//we are going to sort the unionfind array, and store the element id in the size
			//afterwards, we clean unionfind, to make sure no-one uses it anymore
			UnionFind.SortIslands();
			int numElem = UnionFind.ElementCount;

			int endIslandIndex = 1;
			int startIslandIndex;

			//update the sleeping state for bodies, if all are sleeping
			for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex)
			{
				int islandId = UnionFind[startIslandIndex].ID;
				for (endIslandIndex = startIslandIndex + 1; (endIslandIndex < numElem) && (UnionFind[endIslandIndex].ID == islandId); endIslandIndex++)
				{
				}

				//int numSleeping = 0;

				bool allSleeping = true;

				int idx;
				for (idx = startIslandIndex; idx < endIslandIndex; idx++)
				{
					int i = UnionFind[idx].Size;

					CollisionObject colObjA = collisionObjects[i];
					if ((colObjA.IslandTag != islandId) && (colObjA.IslandTag != -1))
					{
						Console.WriteLine("error in island management");
					}

					BulletDebug.Assert((colObjA.IslandTag == islandId) || (colObjA.IslandTag == -1));
					if (colObjA.IslandTag == islandId)
					{
						if (colObjA.ActivationState == ActivationState.Active)
						{
							allSleeping = false;
						}
						if (colObjA.ActivationState == ActivationState.DisableDeactivation)
						{
							allSleeping = false;
						}
					}
				}


				if (allSleeping)
				{
					for (idx = startIslandIndex; idx < endIslandIndex; idx++)
					{
						int i = UnionFind[idx].Size;
						CollisionObject colObjA = collisionObjects[i];
						if ((colObjA.IslandTag != islandId) && (colObjA.IslandTag != -1))
						{
							Console.WriteLine("error in island management");
						}

						BulletDebug.Assert((colObjA.IslandTag == islandId) || (colObjA.IslandTag == -1));

						if (colObjA.IslandTag == islandId)
						{
							colObjA.ActivationState =ActivationState.IslandSleeping;
						}
					}
				}
				else
				{
					for (idx = startIslandIndex; idx < endIslandIndex; idx++)
					{
						int i = UnionFind[idx].Size;

						CollisionObject colObjA = collisionObjects[i];
						if ((colObjA.IslandTag != islandId) && (colObjA.IslandTag != -1))
						{
							Console.WriteLine("error in island management");
						}

						BulletDebug.Assert((colObjA.IslandTag == islandId) || (colObjA.IslandTag == -1));

						if (colObjA.IslandTag == islandId)
						{
							if (colObjA.ActivationState == ActivationState.IslandSleeping)
							{
								colObjA.ActivationState = ActivationState.WantsDeactivation;
							}
						}
					}
				}
			}

			//int maxNumManifolds = dispatcher.ManifoldCount;
			List<PersistentManifold> islandmanifold = new List<PersistentManifold>(dispatcher.ManifoldCount);

			for (int i = 0; i < dispatcher.ManifoldCount; i++)
			{
				PersistentManifold manifold = dispatcher.GetManifoldByIndex(i);

				CollisionObject colObjA = manifold.BodyA as CollisionObject;
				CollisionObject colObjB = manifold.BodyB as CollisionObject;

				//todo: check sleeping conditions!
				if (((colObjA != null) && colObjA.ActivationState != ActivationState.IslandSleeping) ||
				   ((colObjB != null) && colObjB.ActivationState != ActivationState.IslandSleeping))
				{

					//kinematic objects don't merge islands, but wake up all connected objects
					if (colObjA.IsStaticOrKinematicObject && colObjA.ActivationState != ActivationState.IslandSleeping)
					{
						colObjB.Activate();
					}
					if (colObjB.IsStaticOrKinematicObject && colObjB.ActivationState != ActivationState.IslandSleeping)
					{
						colObjA.Activate();
					}

					//filtering for response
					if (dispatcher.NeedsResponse(colObjA, colObjB))
						islandmanifold.Add(manifold);
				}
			}

			int numManifolds = islandmanifold.Count;

			// Sort manifolds, based on islands
			// Sort the vector using predicate and std::sort
			islandmanifold.Sort(new Comparison<PersistentManifold>(PersistentManifoldSortPredicate));

			//now process all active islands (sets of manifolds for now)
			int startManifoldIndex = 0;
			int endManifoldIndex = 1;

			List<CollisionObject> islandBodies = new List<CollisionObject>();

			for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex)
			{
				int islandId = UnionFind[startIslandIndex].ID;
				bool islandSleeping = false;
				for (endIslandIndex = startIslandIndex; (endIslandIndex < numElem) && (UnionFind[endIslandIndex].ID == islandId); endIslandIndex++)
				{
					int i = UnionFind[endIslandIndex].Size;
					CollisionObject colObjA = collisionObjects[i];
					islandBodies.Add(colObjA);
					if (!colObjA.IsActive)
						islandSleeping = true;
				}

				//find the accompanying contact manifold for this islandId
				int numIslandManifolds = 0;
				List<PersistentManifold> startManifold = new List<PersistentManifold>(numIslandManifolds);

				if (startManifoldIndex < numManifolds)
				{
					int curIslandID = GetIslandId(islandmanifold[startManifoldIndex]);
					if (curIslandID == islandId)
					{
						for (int k = startManifoldIndex; k < islandmanifold.Count; k++)
						{
							startManifold.Add(islandmanifold[k]);
						}
						for (endManifoldIndex = startManifoldIndex + 1; (endManifoldIndex < numManifolds) && (islandId == GetIslandId(islandmanifold[endManifoldIndex])); endManifoldIndex++) { }

						// Process the actual simulation, only if not sleeping/deactivated
						numIslandManifolds = endManifoldIndex - startManifoldIndex;
					}
				}

				if (!islandSleeping)
				{
					callback.ProcessIsland(islandBodies, startManifold, numIslandManifolds, islandId);
				}

				if (numIslandManifolds != 0)
				{
					startManifoldIndex = endManifoldIndex;
				}

				islandBodies.Clear();
			}
		}

		private static int GetIslandId(PersistentManifold lhs)
		{
			int islandId;
			CollisionObject rcolObjA = lhs.BodyA as CollisionObject;
			CollisionObject rcolObjB = lhs.BodyB as CollisionObject;
			islandId = rcolObjA.IslandTag >= 0 ? rcolObjA.IslandTag : rcolObjB.IslandTag;
			return islandId;
		}

		private static int PersistentManifoldSortPredicate(PersistentManifold lhs, PersistentManifold rhs)
		{
			int rIslandIdA, lIslandIdB;
			rIslandIdA = GetIslandId(rhs);
			lIslandIdB = GetIslandId(lhs);
			//return lIslandId0 < rIslandId0;
			if (lIslandIdB < rIslandIdA)
				return -1;
			//else if (lIslandIdB > rIslandIdA)
			//    return 1;
			return 1;
		}

		public interface IIslandCallback
		{
			void ProcessIsland(List<CollisionObject> bodies, List<PersistentManifold> manifolds, int numManifolds, int islandID);
		}
	}
}
