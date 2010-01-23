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

Modified by Roman Ponomarev <rponom@gmail.com>
01/22/2010 : Constraints reworked
*/

//sliderConstraintNode.h

#ifndef DYN_SLIDER_CONSTRAINT_NODE_H
#define DYN_SLIDER_CONSTRAINT_NODE_H

#include <maya/MString.h>
#include <maya/MTypeId.h>
#include <maya/MPxLocatorNode.h>

#include "solver.h"

class sliderConstraintNode: public MPxLocatorNode
{
public:
    sliderConstraintNode();
    virtual ~sliderConstraintNode();

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

    slider_constraint_t::pointer constraint();
    void update();
    
public:

    //Attributes
    static  MObject     ia_rigidBodyA;
    static  MObject     ia_rigidBodyB;
    static  MObject     ia_damping;

    static  MObject     ia_lowerLinLimit;
    static  MObject     ia_upperLinLimit;	
    static  MObject     ia_lowerAngLimit;
    static  MObject     ia_upperAngLimit;	

    static  MObject     ia_rotationInA;
    static  MObject     ia_rotationInB;

	static  MObject     ia_pivotInA;
    static  MObject     ia_pivotInB;


    static  MObject     ca_constraint;
    static  MObject     ca_constraintParam;

public:
    static  MTypeId     typeId;
    static  MString     typeName;

private:
    void computeConstraint(const MPlug& plug, MDataBlock& data);
    void computeConstraintParam(const MPlug& plug, MDataBlock& data);
    void computeWorldMatrix(const MPlug& plug, MDataBlock& data);

public:
    static void nodeRemoved(MObject& node, void *clientData);

private:
    slider_constraint_t::pointer       m_constraint;
};



#endif //DYN_SLIDER_CONSTRAINT_NODE_H
