/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Experimental Buoyancy fluid demo written by John McCutchan
*/

#ifndef __HFFLUID_H
#define __HFFLUID_H

#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btTriangleCallback.h"

class btPersistentManifold;
class btManifoldResult;

// FIX AABB calculation for whole btHfFluid shape
// Fix flags and fill ratio
// -> Figure out the constants used in flags and fill ratio code
// Fix volume removal
// add buoyant convex vs. convex / concave
// add buoyant concave support (try bunny model)

///experimental buyancy fluid demo
class btHfFluid : public btCollisionObject
{
public:
	btHfFluid (btScalar gridCellWidth, int numNodesWidth, int numNodesLength);

	~btHfFluid ();

	void predictMotion(btScalar dt);

	/* Prep does some initial setup of the height field fluid.
	 * You should call this at initialization time.
	 */
	void prep ();
		
	static const btHfFluid*	upcast(const btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_HF_FLUID)
			return (const btHfFluid*)colObj;
		return 0;
	}
	static btHfFluid*			upcast(btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_HF_FLUID)
			return (btHfFluid*)colObj;
		return 0;
	}

	//
	// ::btCollisionObject
	//

	virtual void getAabb(btVector3& aabbMin,btVector3& aabbMax) const
	{
		aabbMin = m_aabbMin;
		aabbMax = m_aabbMax;
	}

	int getNumNodesWidth () const { return m_numNodesWidth; }
	int getNumNodesLength () const { return m_numNodesLength; }

	btScalar getGridCellWidth () const { return m_gridCellWidth; }
	btScalar widthPos (int i) const;
	btScalar lengthPos (int j) const;

	int arrayIndex (int i, int j) const;
	int arrayIndex (btScalar i, btScalar j) const;
	int arrayIndex (unsigned int i, unsigned int j) const;
	const btScalar* getHeightArray () const;
	const btScalar* getGroundArray () const;
	const btScalar* getEtaArray () const;
	const btScalar* getVelocityUArray () const;
	const btScalar* getVelocityVArray () const;
	const bool* getFlagsArray () const;

	void setFluidHeight (int x, int y, btScalar height);
	void setFluidHeight (int index, btScalar height);

	void addFluidHeight (int x, int y, btScalar height);
	void addDisplaced (int i, int j, btScalar r);

	void getAabbForColumn (int x, int y, btVector3& aabbMin, btVector3& aabbMax);

	btScalar* getHeightArray ();
	btScalar* getGroundArray ();
	btScalar* getEtaArray ();
	btScalar* getVelocityUArray ();
	btScalar* getVelocityVArray ();
	bool* getFlagsArray ();

	void foreachGroundTriangle(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax);
	class btHfFluidColumnCallback 
	{
	public:
		btHfFluidColumnCallback () {}

		virtual ~btHfFluidColumnCallback () {}

		virtual bool processColumn (btHfFluid* fluid, int w, int l)
		{
			return true; // keep going
		}
	};

	void foreachFluidColumn (btHfFluidColumnCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax);

	void foreachSurfaceTriangle (btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax);
protected:
	int m_numNodesWidth;
	int m_numNodesLength;

	btScalar m_gridCellWidth;
	btScalar m_gridWidth;
	btScalar m_gridLength;

	btScalar m_gridCellWidthInv;
	
	btVector3 m_aabbMin;
	btVector3 m_aabbMax;

	void setGridDimensions (btScalar gridCellWidth,
							int numNodesWidth, int numNodesLength);

	btScalar bilinearInterpolate (const btScalar* array, btScalar i, btScalar j);

	btScalar advect (const btScalar* array, btScalar i, btScalar j, btScalar di, btScalar dj, btScalar dt);

	void advectEta (btScalar dt);
	void updateHeight (btScalar dt);

	void advectVelocityU (btScalar dt);
	void advectVelocityV (btScalar dt);
	void updateVelocity (btScalar dt);

	void transferDisplaced (btScalar dt);

	void setReflectBoundaryLeft ();
	void setReflectBoundaryRight ();
	void setReflectBoundaryTop ();
	void setReflectBoundaryBottom ();

	void setAbsorbBoundaryLeft (btScalar dt);
	void setAbsorbBoundaryRight (btScalar dt);
	void setAbsorbBoundaryTop (btScalar dt);
	void setAbsorbBoundaryBottom (btScalar dt);

	void computeFlagsAndFillRatio ();
	btScalar computeHmin (int i, int j);
	btScalar computeHmax (int i, int j);
	btScalar computeEtaMax (int i, int j);

	void allocateArrays ();

	void debugTests ();

	btScalar* m_temp; // temp
	int m_heightIndex;
	btScalar* m_height[2];
	btScalar* m_ground;
	btScalar* m_eta; // height - ground
	btScalar* m_u[2];
	btScalar* m_v[2];
	int m_rIndex;
	btScalar* m_r[2];
	int m_velocityIndex;
	bool* m_flags;
	btScalar* m_fillRatio;

	// tweakables
	btScalar m_globalVelocityU;
	btScalar m_globalVelocityV;
	btScalar m_gravity;
	btScalar m_volumeDisplacementScale;
	btScalar m_horizontalVelocityScale;

	btScalar m_epsHeight;
	btScalar m_epsEta;
public:
	// You can enforce a global velocity at the surface of the fluid
	// default: 0.0 and 0.0
	void setGlobaVelocity (btScalar globalVelocityU, btScalar globalVelocityV);
	void getGlobalVelocity (btScalar& globalVelocityU, btScalar& globalVelocityV) const;

	// Control force of gravity, should match physics world
	// default: -10.0
	void setGravity (btScalar gravity);
	btScalar getGravity () const;

	// When a body is submerged into the fluid, the displaced fluid
	// is spread to adjacent cells. You can control the percentage of this
	// by setting this value between 0.0 and 1.0
	// default: 0.5
	void setVolumeDisplacementScale (btScalar volumeDisplacementScale);
	btScalar getVolumeDisplacementScale () const;

	// The horizontal velocity of the fluid can influence bodies submerged
	// in the fluid. You can control how much influence by setting this
	// between 0.0 and 1.0
	// default: 0.5
	void setHorizontalVelocityScale (btScalar horizontalVelocityScale);
	btScalar getHorizontalVelocityScale () const;
};

class btRigidBody;
class btIDebugDraw;
class btHfFluidBuoyantConvexShape;

class btHfFluidColumnRigidBodyCallback : public btHfFluid::btHfFluidColumnCallback
{
protected:
	btRigidBody* m_rigidBody;
	btHfFluidBuoyantConvexShape* m_buoyantShape;
	btIDebugDraw* m_debugDraw;
	int m_numVoxels;
	btVector3* m_voxelPositionsXformed;
	bool* m_voxelSubmerged;
	btVector3 m_aabbMin;
	btVector3 m_aabbMax;
	btScalar m_volume;
	btScalar m_density;
	btScalar m_floatyness;
public:
	btHfFluidColumnRigidBodyCallback (btRigidBody* rigidBody, btIDebugDraw* debugDraw, btScalar density, btScalar floatyness);
	~btHfFluidColumnRigidBodyCallback ();
	bool processColumn (btHfFluid* fluid, int w, int l);
	btScalar getVolume () const { return m_volume; }
};

#endif

