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
	public delegate void NearCallback(ref BroadphasePair collisionPair, CollisionDispatcher dispatcher, DispatcherInfo dispatchInfo);

	public class CollisionDispatcher : IDispatcher
	{
		private List<PersistentManifold> _manifolds = new List<PersistentManifold>();

		//private bool _useIslands;
		private NearCallback _nearCallback;

		//private ManifoldResult _defaultManifoldResult;

		private CollisionAlgorithmCreateFunction[,] _doubleDispatch = new CollisionAlgorithmCreateFunction[(int)BroadphaseNativeTypes.MaxBroadphaseCollisionTypes, (int)BroadphaseNativeTypes.MaxBroadphaseCollisionTypes];

		//default CreationFunctions, filling the m_doubleDispatch table
		private CollisionAlgorithmCreateFunction _convexConvexCreateFunc;
		private CollisionAlgorithmCreateFunction _convexConcaveCreateFunc;
		private CollisionAlgorithmCreateFunction _swappedConvexConcaveCreateFunc;
		private CollisionAlgorithmCreateFunction _compoundCreateFunc;
		private CollisionAlgorithmCreateFunction _swappedCompoundCreateFunc;
		private CollisionAlgorithmCreateFunction _emptyCreateFunc;

		private int _count;
		private static int _manifoldCount = 0;

		public CollisionDispatcher()
		{
			NearCallback = DefaultNearCallback;
			//_useIslands = true;
			//default CreationFunctions, filling the m_doubleDispatch table
			_convexConvexCreateFunc = new ConvexConvexAlgorithm.CreateFunc();
			_convexConcaveCreateFunc = new ConvexConcaveCollisionAlgorithm.CreateFunc();
			_swappedConvexConcaveCreateFunc = new ConvexConcaveCollisionAlgorithm.SwappedCreateFunc();
			_compoundCreateFunc = new CompoundCollisionAlgorithm.CreateFunc();
			_swappedCompoundCreateFunc = new CompoundCollisionAlgorithm.SwappedCreateFunc();
			_emptyCreateFunc = new EmptyAlgorithm.CreateFunc();

			for (BroadphaseNativeTypes i = BroadphaseNativeTypes.Box; i < BroadphaseNativeTypes.MaxBroadphaseCollisionTypes; i++)
			{
				for (BroadphaseNativeTypes j = BroadphaseNativeTypes.Box; j < BroadphaseNativeTypes.MaxBroadphaseCollisionTypes; j++)
				{
					_doubleDispatch[(int)i, (int)j] = FindCreateFunction(i, j);
					if (_doubleDispatch[(int)i, (int)j] == null)
						throw new BulletException();
				}
			}
		}

		public int Count { get { return _count; } set { _count = value; } }
		public int ManifoldCount { get { return _manifolds.Count; } }
		public List<PersistentManifold> Manifolds { get { return _manifolds; } }

		public static int GlobalManifoldCount { get { return _manifoldCount; } set { _manifoldCount = value; } }

		public PersistentManifold GetManifoldByIndex(int index)
		{
			return _manifolds[index];
		}

		//registerCollisionCreateFunc allows registration of custom/alternative collision create functions
		public void RegisterCollisionCreateFunc(BroadphaseNativeTypes proxyTypeA, BroadphaseNativeTypes proxyTypeB, CollisionAlgorithmCreateFunction createFunc)
		{
			_doubleDispatch[(int)proxyTypeA, (int)proxyTypeB] = createFunc;
		}

		public virtual PersistentManifold GetNewManifold(object bodyA, object bodyB)
		{
			_manifoldCount++;

			CollisionObject body0 = bodyA as CollisionObject;
			CollisionObject body1 = bodyB as CollisionObject;

			PersistentManifold manifold = new PersistentManifold(body0, body1);
			_manifolds.Add(manifold);

			return manifold;
		}

		public virtual void ReleaseManifold(PersistentManifold manifold)
		{
			_manifoldCount--;

			ClearManifold(manifold);
			_manifolds.Remove(manifold);
		}


		public virtual void ClearManifold(PersistentManifold manifold)
		{
			manifold.ClearManifold();
		}

		public CollisionAlgorithm FindAlgorithm(CollisionObject bodyA, CollisionObject bodyB)
		{
			return FindAlgorithm(bodyA, bodyB, null);
		}

		public CollisionAlgorithm FindAlgorithm(CollisionObject bodyA, CollisionObject bodyB, PersistentManifold sharedManifold)
		{
			CollisionAlgorithmConstructionInfo collisionAlgorithmConstructionInfo = new CollisionAlgorithmConstructionInfo();
			collisionAlgorithmConstructionInfo.Dispatcher = this;
			collisionAlgorithmConstructionInfo.Manifold = sharedManifold;
			CollisionAlgorithm collisionAlgorithm = _doubleDispatch[(int)bodyA.CollisionShape.ShapeType, (int)bodyB.CollisionShape.ShapeType].CreateCollisionAlgorithm(collisionAlgorithmConstructionInfo, bodyA, bodyB);
			return collisionAlgorithm;
		}

		/*public CollisionAlgorithm internalFindAlgorithm(CollisionObject body0, CollisionObject body1)
		{
			return internalFindAlgorithm(body0, body1, null);
		}

		public CollisionAlgorithm internalFindAlgorithm(CollisionObject body0, CollisionObject body1, PersistentManifold sharedManifold)
		{
			m_count++;

			CollisionAlgorithmConstructionInfo ci = new CollisionAlgorithmConstructionInfo();
			ci.m_dispatcher = this;

			if (body0.getCollisionShape().isConvex() && body1.getCollisionShape().isConvex())
			{
				return new ConvexConvexAlgorithm(sharedManifold, ci, body0, body1);
			}

			if (body0.getCollisionShape().isConvex() && body1.getCollisionShape().isConcave())
			{
				return new ConvexConcaveCollisionAlgorithm(ci, body0, body1, false);
			}

			if (body1.getCollisionShape().isConvex() && body0.getCollisionShape().isConcave())
			{
				return new ConvexConcaveCollisionAlgorithm(ci, body0, body1, true);
			}

			if (body0.getCollisionShape().isCompound())
			{
				return new CompoundCollisionAlgorithm(ci, body0, body1, false);
			}
			else
			{
				if (body1.getCollisionShape().isCompound())
				{
					return new CompoundCollisionAlgorithm(ci, body0, body1, true);
				}
			}

			//failed to find an algorithm
			return new EmptyAlgorithm(ci);
		}*/

		public virtual bool NeedsCollision(CollisionObject bodyA, CollisionObject bodyB)
		{
			if (bodyA == null || bodyB == null)
				throw new BulletException();

			bool needsCollision = true;

			//broadphase filtering already deals with this
			/*if ((body0.isStaticObject() || body0.isKinematicObject()) &&
				(body1.isStaticObject() || body1.isKinematicObject()))
			{
				printf("warning btCollisionDispatcher::needsCollision: static-static collision!\n");
			}*/

			if ((!bodyA.IsActive) && (!bodyB.IsActive))
				needsCollision = false;

			return needsCollision;
		}

		public virtual bool NeedsResponse(CollisionObject bodyA, CollisionObject bodyB)
		{
			//here you can do filtering
			bool hasResponse = bodyA.HasContactResponse && bodyB.HasContactResponse;
			hasResponse = hasResponse && (!bodyA.IsStaticOrKinematicObject || !bodyB.IsStaticOrKinematicObject);
			return hasResponse;
		}

		public virtual void DispatchAllCollisionPairs(OverlappingPairCache pairCache, DispatcherInfo dispatchInfo)
		{
			CollisionPairCallback collisionCallback = new CollisionPairCallback(dispatchInfo, this);
			pairCache.ProcessAllOverlappingPairs(collisionCallback);
		}

		private CollisionAlgorithmCreateFunction FindCreateFunction(BroadphaseNativeTypes proxyTypeA, BroadphaseNativeTypes proxyTypeB)
		{
			if (BroadphaseProxy.IsConvex(proxyTypeA) && BroadphaseProxy.IsConvex(proxyTypeB))
			{
				return _convexConvexCreateFunc;
			}

			if (BroadphaseProxy.IsConvex(proxyTypeA) && BroadphaseProxy.IsConcave(proxyTypeB))
			{
				return _convexConcaveCreateFunc;
			}

			if (BroadphaseProxy.IsConvex(proxyTypeB) && BroadphaseProxy.IsConcave(proxyTypeA))
			{
				return _swappedConvexConcaveCreateFunc;
			}

			if (BroadphaseProxy.IsCompound(proxyTypeA))
			{
				return _compoundCreateFunc;
			}
			else
			{
				if (BroadphaseProxy.IsCompound(proxyTypeB))
				{
					return _swappedCompoundCreateFunc;
				}
			}

			//failed to find an algorithm
			return _emptyCreateFunc;
		}

		public NearCallback NearCallback { get { return _nearCallback; } set { _nearCallback = value; } }

		//by default, Bullet will use this near callback
		public static void DefaultNearCallback(ref BroadphasePair collisionPair, CollisionDispatcher dispatcher, DispatcherInfo dispatchInfo)
		{
			CollisionObject collisionObjectA = collisionPair.ProxyA.ClientData as CollisionObject;
			CollisionObject collisionObjectB = collisionPair.ProxyB.ClientData as CollisionObject;

			if (dispatcher.NeedsCollision(collisionObjectA, collisionObjectB))
			{
				//dispatcher will keep algorithms persistent in the collision pair
				if (collisionPair.CollisionAlgorithm == null)
				{
					collisionPair.CollisionAlgorithm = dispatcher.FindAlgorithm(collisionObjectA, collisionObjectB);
				}

				if (collisionPair.CollisionAlgorithm != null)
				{
					ManifoldResult contactPointResult = new ManifoldResult(collisionObjectA, collisionObjectB);

					if (dispatchInfo.DispatchFunction == DispatchFunction.Discrete)
					{
						//discrete collision detection query
						collisionPair.CollisionAlgorithm.ProcessCollision(collisionObjectA, collisionObjectB, dispatchInfo, contactPointResult);
					}
					else
					{
						//continuous collision detection query, time of impact (toi)
						float timeOfImpact = collisionPair.CollisionAlgorithm.CalculateTimeOfImpact(collisionObjectA, collisionObjectB, dispatchInfo, contactPointResult);
						if (dispatchInfo.TimeOfImpact > timeOfImpact)
							dispatchInfo.TimeOfImpact = timeOfImpact;
					}
				}
			}
		}
	}
}
