#ifndef __BT_HFFLUID_BUOYANT_CONVEX_SHAPE_H
#define __BT_HFFLUID_BUOYANT_CONVEX_SHAPE_H

#include "LinearMath/btVector3.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"

#define MAX_VOXEL_DIMENSION 32

class btConvexShape;

class btHfFluidBuoyantConvexShape : public btCollisionShape
{
public:
	btHfFluidBuoyantConvexShape (btConvexShape* convexShape);

	void generateShape (btScalar radius, btScalar gap);

	btConvexShape* getConvexShape () { return m_convexShape; }

	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;
	virtual void	setMargin(btScalar margin);
	virtual btScalar	getMargin() const;
	virtual void	setLocalScaling(const btVector3& scaling);
	virtual const btVector3& getLocalScaling() const;
	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia) const;
	virtual const char*	getName()const;

	btScalar getVoxelRadius () const { return m_radius; }
	btScalar getTotalVolume () const { return m_totalVolume; }
	btScalar getVolumePerVoxel () const { return m_volumePerVoxel; }
	btScalar getFloatyness () const { return m_floatyness; }
	void setFloatyness (btScalar floatyness) { m_floatyness = floatyness; }
	int getNumVoxels () const { return m_numVoxels; }
	const btVector3* getVoxelPositionsArray() { return m_voxelPositions; }

protected:
	btScalar m_floatyness;
	btScalar m_radius;
	btScalar m_totalVolume;
	btScalar m_volumePerVoxel;
	int m_numVoxels;
	btVector3 m_voxelPositions[MAX_VOXEL_DIMENSION*MAX_VOXEL_DIMENSION*MAX_VOXEL_DIMENSION];
	btConvexShape* m_convexShape;
};

#endif
