#ifndef COLLISION_TUTORIAL_H
#define COLLISION_TUTORIAL_H

enum EnumCollisionTutorialTypes
{
	TUT_SPHERE_PLANE_BULLET2 = 0,
	TUT_SPHERE_PLANE_RTB3,
};

class CommonExampleInterface* CollisionTutorialBullet2CreateFunc(struct CommonExampleOptions& options);

#endif  //COLLISION_TUTORIAL_H
