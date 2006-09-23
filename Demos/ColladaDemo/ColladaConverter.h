/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef COLLADA_CONVERTER_H
#define COLLADA_CONVERTER_H

#include "SimdTransform.h"
#include "SimdVector3.h"

class CollisionShape;
class PHY_IPhysicsController;
class CcdPhysicsController;
class ConstraintInput;

//use some reasonable number here
#define COLLADA_CONVERTER_MAX_NUM_OBJECTS 32768

//namespace..

///ColladaConverter helps converting the physics assets from COLLADA DOM into physics objects
class ColladaConverter
{

protected:

	class DAE* m_collada;
	class domCOLLADA* m_dom;
	const char* m_filename;
	
	int	m_numObjects;
	CcdPhysicsController* m_physObjects[COLLADA_CONVERTER_MAX_NUM_OBJECTS];
	
	void	PreparePhysicsObject(struct RigidBodyInput& input, bool isDynamics, float mass,CollisionShape* colShape);
	
	void	PrepareConstraints(ConstraintInput& input);

	void	ConvertRigidBodyRef( struct RigidBodyInput& , struct RigidBodyOutput& output );


public:
	
	ColladaConverter();

	///open a COLLADA .dae file
	bool	load(const char* filename);
	
	///save a snapshot in COLLADA physics .dae format.
	///if the filename is left empty, modify the filename used during loading
	bool	saveAs(const char* filename = 0);

	///convert a Collada DOM document and call the 2 virtual methods for each rigid body/constraint
	bool convert();

	///those 2 virtuals are called for each constraint/physics object
	virtual int			createUniversalD6Constraint(
		class PHY_IPhysicsController* ctrlRef,class PHY_IPhysicsController* ctrlOther,
			SimdTransform& localAttachmentFrameRef,
			SimdTransform& localAttachmentOther,
			const SimdVector3& linearMinLimits,
			const SimdVector3& linearMaxLimits,
			const SimdVector3& angularMinLimits,
			const SimdVector3& angularMaxLimits
			) = 0;

	virtual CcdPhysicsController*  CreatePhysicsObject(bool isDynamic, 
		float mass, 
		const SimdTransform& startTransform,
		CollisionShape* shape) = 0;

	virtual	void	SetGravity(const SimdVector3& gravity) = 0;
	
	virtual	void	SetCameraInfo(const SimdVector3& up, int forwardAxis) = 0;

};

#endif //COLLADA_CONVERTER_H


