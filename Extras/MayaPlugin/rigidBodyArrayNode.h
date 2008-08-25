/*
Bullet Continuous Collision Detection and Physics Library Maya Plugin
Copyright (c) 2008 Walt Disney Studios
 
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
 
Written by: Nicola Candussi <nicola@fluidinteractive.com>
*/

//rigidBodyArrayNode.h

#ifndef DYN_RIGID_BODY_ARRAY_NODE_H
#define DYN_RIGID_BODY_ARRAY_NODE_H

#include <maya/MString.h>
#include <maya/MTypeId.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MMatrix.h>

#include <vector>

#include "solver.h"

class rigidBodyArrayNode: public MPxLocatorNode
{
public:
    rigidBodyArrayNode();
    virtual ~rigidBodyArrayNode();

    virtual void postConstructor();

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

    std::vector<rigid_body_t::pointer>& rigid_bodies();
    
public:

    //Attributes
    static  MObject     ia_collisionShape;
    static  MObject     ia_solver;
    static  MObject     ia_numBodies;
    static  MObject     ia_active;
    static  MObject     ia_mass;
    static  MObject     ia_restitution;
    static  MObject     ia_friction;
    static  MObject     ia_linearDamping;
    static  MObject     ia_angularDamping;

    static  MObject     ia_initialPosition;
    static  MObject     ia_initialRotation;
    static  MObject     ia_initialVelocity;
    static  MObject     ia_initialSpin;

    static  MObject     ia_fileIO;
    static  MObject     ia_fioFiles;
    static  MObject     ia_fioPositionAttribute;
    static  MObject     ia_fioRotationAttribute;

    static  MObject     io_position;
    static  MObject     io_rotation;

    static  MObject     ca_rigidBodies;
    static  MObject     ca_rigidBodyParam;

public:
    static  MTypeId     typeId;
    static  MString     typeName;

private:
    void update();
    void computeRigidBodies(const MPlug& plug, MDataBlock& data);
    void computeRigidBodyParam(const MPlug& plug, MDataBlock& data); 
    void computeWorldMatrix(const MPlug& plug, MDataBlock& data);

public:
    static void nodeRemoved(MObject& node, void *clientData);

private:
    std::vector<rigid_body_t::pointer>  m_rigid_bodies;
    MMatrix                             m_worldMatrix;
};



#endif
