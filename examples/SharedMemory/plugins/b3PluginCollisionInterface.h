#ifndef B3_PLUGIN_COLLISION_INTERFACE_H
#define B3_PLUGIN_COLLISION_INTERFACE_H

enum b3PluginCollisionFilterModes
{
	B3_FILTER_GROUPAMASKB_AND_GROUPBMASKA = 0,
	B3_FILTER_GROUPAMASKB_OR_GROUPBMASKA
};

struct b3PluginCollisionInterface
{
	virtual void setBroadphaseCollisionFilter(
		int objectUniqueIdA, int objectUniqueIdB,
		int linkIndexA, int linkIndexB,
		bool enableCollision) = 0;

	virtual void removeBroadphaseCollisionFilter(
		int objectUniqueIdA, int objectUniqueIdB,
		int linkIndexA, int linkIndexB) = 0;

	virtual int getNumRules() const = 0;

	virtual void resetAll() = 0;

	virtual int needsBroadphaseCollision(int objectUniqueIdA, int linkIndexA,
										 int collisionFilterGroupA, int collisionFilterMaskA,
										 int objectUniqueIdB, int linkIndexB,
										 int collisionFilterGroupB, int collisionFilterMaskB,
										 int filterMode) = 0;
};

#endif  //B3_PLUGIN_COLLISION_INTERFACE_H