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

//collisionShapeNode.h

#ifndef DYN_COLLISION_SHAPE_NODE_H
#define DYN_COLLISION_SHAPE_NODE_H

#include <maya/MString.h>
#include <maya/MTypeId.h>
#include <maya/MPxNode.h>

#include "collision_shape.h"


class collisionShapeNode: public MPxNode
{
public:
    collisionShapeNode();
    virtual ~collisionShapeNode();

    virtual bool        setInternalValueInContext ( const  MPlug & plug,
                                                    const  MDataHandle & dataHandle,
                                                    MDGContext & ctx);

    virtual MStatus     compute( const MPlug& plug, MDataBlock& data );

    static  void *      creator();
    static  MStatus     initialize();

public:

    collision_shape_t::pointer collisionShape();
  
public:

    //Attributes
    static  MObject     ia_shape;
    static  MObject     ia_type;
    static  MObject     ia_scale;
    static  MObject     ca_collisionShape;
    static  MObject     ca_collisionShapeParam;
    static  MObject     oa_collisionShape;

public:
    static  MTypeId     typeId;
    static  MString     typeName;

protected:
    void computeCollisionShape(const MPlug& plug, MDataBlock& data);
    void computeCollisionShapeParam(const MPlug& plug, MDataBlock& data);
    void computeOutputShape(const MPlug& plug, MDataBlock& data);

private:
    collision_shape_t::pointer m_collision_shape;
    
};



#endif
