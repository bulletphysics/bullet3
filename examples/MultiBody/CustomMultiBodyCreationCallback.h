/* Copyright (C) 2015 Google

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef CUSTOM_MULTIBODY_CALLBACK_H
#define CUSTOM_MULTIBODY_CALLBACK_H

#ifdef USE_EIGEN
	typedef Eigen::Quaternion<double> QuaternionType;
	typedef Eigen::Vector3d<double> Vector3dType;
	typedef double ScalarType;
#else
	typedef btQuaternion QuaternionType;
	typedef btVector3 Vector3dType;
	typedef btScalar	ScalarType;
#endif

class CustomMultiBodyCreationCallback
{
    
public:

	virtual ~CustomMultiBodyCreationCallback() {}

    enum {
        RevoluteJoint=1,
        PrismaticJoint,
        FixedJoint,
    };
 
    virtual int allocateMultiBodyBase(int urdfLinkIndex, int totalNumJoints,ScalarType baseMass, const Vector3dType& localInertiaDiagonal, bool isFixedBase) const =0;

    virtual void addLinkAndJoint(int jointType, int linkIndex,            // 0 to num_links-1
                       int parentIndex,
                       double mass,
                       const Vector3dType&	inertia,
                       const QuaternionType &rotParentFrameToLinkFrame,  // rotate points in parent frame to this frame, when q = 0
                       const Vector3dType&	jointAxisInLinkFrame,    // in Link frame
                       const Vector3dType&	parentComToThisJointOffset,    // vector from parent COM to joint frame, in Parent frame
                       const Vector3dType&	thisJointToThisComOffset) = 0;       // vector from joint frame to my COM, in Link frame);
                 
// @todo: Decide if we need this link mapping? 
// virtual void addLinkMapping(int urdfLinkIndex, int mbLinkIndex) const = 0;
};

#endif //CUSTOM_MULTIBODY_CALLBACK_H
