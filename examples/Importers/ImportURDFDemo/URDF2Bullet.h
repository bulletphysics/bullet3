#ifndef _URDF2BULLET_H
#define _URDF2BULLET_H
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include <string>
class btVector3;
class btTransform;
class btMultiBodyDynamicsWorld;
class btTransform;


class URDFImporterInterface;
class MultiBodyCreationInterface;


void printTree(const URDFImporterInterface& u2b, int linkIndex, int identationLevel=0);


void ConvertURDF2Bullet(const URDFImporterInterface& u2b, 
			MultiBodyCreationInterface& creationCallback, 
			const btTransform& rootTransformInWorldSpace, 
			btMultiBodyDynamicsWorld* world,
			bool createMultiBody, 
			const char* pathPrefix);


#endif //_URDF2BULLET_H

