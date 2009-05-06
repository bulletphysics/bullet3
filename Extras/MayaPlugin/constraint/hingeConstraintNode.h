/*
Bullet Continuous Collision Detection and Physics Library Maya Plugin
Copyright (c) 2008 Herbert Law
 
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising
from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:
 
1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this
software in a product, an acknowledgment in the product documentation
would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must
not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 
Written by: Herbert Law <Herbert.Law@gmail.com>
*/

//hingeConstraintNode.h

#ifndef DYN_HINGE_CONSTRAINT_NODE_H
#define DYN_HINGE_CONSTRAINT_NODE_H

#include <maya/MString.h>
#include <maya/MTypeId.h>
#include <maya/MPxLocatorNode.h>

#include "solver.h"

class hingeConstraintNode: public MPxLocatorNode
{
public:
    hingeConstraintNode();
    virtual ~hingeConstraintNode();

    virtual bool        setInternalValueInContext ( const  MPlug & plug,
                                                    const  MDataHandle & dataHandle,
                                                    MDGContext & ctx);

  //  virtual MStatus  	setDependentsDirty ( const  MPlug & plug,  MPlugArray & plugArray);

    virtual MStatus     compute( const MPlug& plug, MDataBlock& data );

    virtual void        draw( M3dView & view, const MDagPath & path,
                              M3dView::DisplayStyle style,
                              M3dView::DisplayStatus status );


    virtual bool            isBounded() const ;
    virtual MBoundingBox    boundingBox() const;

    virtual bool        excludeAsLocator() const { return false; }
    virtual bool        isTransparent() const { return false; }

    static  void *      creator();
    static  MStatus     initialize();

public:

    hinge_constraint_t::pointer constraint();
    
public:

    //Attributes
    static  MObject     ia_rigidBody;
    static  MObject     ia_damping;

    static  MObject     ia_lowerLimit;
    static  MObject     ia_upperLimit;	
	static  MObject     ia_limitSoftness;
	static  MObject     ia_biasFactor;
	static  MObject     ia_relaxationFactor;

    static  MObject     ia_hingeAxis;

    static  MObject     ca_constraint;
    static  MObject     ca_constraintParam;

	static  MObject		ia_enableAngularMotor;
	static  MObject		ia_motorTargetVelocity;
	static  MObject		ia_maxMotorImpulse;

public:
    static  MTypeId     typeId;
    static  MString     typeName;

private:
    void update();
    void computeConstraint(const MPlug& plug, MDataBlock& data);
    void computeConstraintParam(const MPlug& plug, MDataBlock& data);
    void computeWorldMatrix(const MPlug& plug, MDataBlock& data);

public:
    static void nodeRemoved(MObject& node, void *clientData);

private:
    hinge_constraint_t::pointer       m_constraint;
};



#endif //DYN_HINGE_CONSTRAINT_NODE_H
