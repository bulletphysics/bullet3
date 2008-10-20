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

//nailConstraintNode.cpp

#include <maya/MFnDependencyNode.h>
#include <maya/MPlugArray.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnTransform.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>
#include <maya/MVector.h>

#include "rigidBodyNode.h"
#include "nailConstraintNode.h"
#include "mayaUtils.h"

#include "solver.h"

MTypeId     nailConstraintNode::typeId(0x10033A);
MString     nailConstraintNode::typeName("dNailConstraint");

MObject     nailConstraintNode::ia_rigidBody;
MObject     nailConstraintNode::ia_damping;
MObject     nailConstraintNode::ca_constraint;
MObject     nailConstraintNode::ca_constraintParam;


MStatus nailConstraintNode::initialize()
{
    MStatus status;
    MFnMessageAttribute fnMsgAttr;
    MFnNumericAttribute fnNumericAttr;
    MFnMatrixAttribute fnMatrixAttr;

    ia_rigidBody = fnMsgAttr.create("inRigidBody", "inrb", &status);
    MCHECKSTATUS(status, "creating inRigidBody attribute")
    status = addAttribute(ia_rigidBody);
    MCHECKSTATUS(status, "adding inRigidBody attribute")

    ia_damping = fnNumericAttr.create("damping", "dmp", MFnNumericData::kDouble, 1.0, &status);
    MCHECKSTATUS(status, "creating damping attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_damping);
    MCHECKSTATUS(status, "adding damping attribute")

    ca_constraint = fnNumericAttr.create("ca_constraint", "caco", MFnNumericData::kBoolean, 0, &status);
    MCHECKSTATUS(status, "creating ca_constraint attribute")
    fnNumericAttr.setConnectable(false);
    fnNumericAttr.setHidden(true);
    fnNumericAttr.setStorable(false);
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ca_constraint);
    MCHECKSTATUS(status, "adding ca_constraint attribute")

    ca_constraintParam = fnNumericAttr.create("ca_constraintParam", "cacop", MFnNumericData::kBoolean, 0, &status);
    MCHECKSTATUS(status, "creating ca_constraintParam attribute")
    fnNumericAttr.setConnectable(false);
    fnNumericAttr.setHidden(true);
    fnNumericAttr.setStorable(false);
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ca_constraintParam);
    MCHECKSTATUS(status, "adding ca_constraintParam attribute")


    status = attributeAffects(ia_rigidBody, ca_constraint);
    MCHECKSTATUS(status, "adding attributeAffects(ia_rigidBody, ca_constraint)")

    status = attributeAffects(ia_rigidBody, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_rigidBody, ca_constraintParam)")

    status = attributeAffects(ia_damping, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_damping, ca_constraintParam)")

    return MS::kSuccess;
}

nailConstraintNode::nailConstraintNode()
{
    // std::cout << "nailConstraintNode::nailConstraintNode" << std::endl;
}

nailConstraintNode::~nailConstraintNode()
{
    // std::cout << "nailConstraintNode::~nailConstraintNode" << std::endl;
}

void nailConstraintNode::nodeRemoved(MObject& node, void *clientData)
{
   // std::cout << "nailConstraintNode::nodeRemoved" << std::endl;
    MFnDependencyNode fnNode(node);
    nailConstraintNode *pNode = static_cast<nailConstraintNode*>(fnNode.userNode());
    constraint_t::pointer constraint = static_cast<constraint_t::pointer>(pNode->m_constraint);
    solver_t::remove_constraint(constraint);
}

void* nailConstraintNode::creator()
{
    return new nailConstraintNode();
}


bool nailConstraintNode::setInternalValueInContext ( const  MPlug & plug,
                                                   const  MDataHandle & dataHandle,
                                                   MDGContext & ctx)
{
   /* if ((plug == pdbFiles) || (plug == ia_scale) || (plug == ia_percent)) {
        m_framesDirty = true;
    } else if(plug == textureFiles) {
        gpufx::m_renderer.setColorTextureDirty();
    }*/
    return false; //setInternalValueInContext(plug,dataHandle,ctx);
}

MStatus nailConstraintNode::compute(const MPlug& plug, MDataBlock& data)
{
    //std::cout << "nailConstraintNode::compute: " << plug.name() << std::endl;
    //MTime time = data.inputValue( nailConstraintNode::inTime ).asTime();
    if(plug == ca_constraint) {
        computeConstraint(plug, data);
    } else if(plug == ca_constraintParam) {
        computeConstraintParam(plug, data);
    } else if(plug.isElement()) {
        if(plug.array() == worldMatrix && plug.logicalIndex() == 0) {
            computeWorldMatrix(plug, data);
        } else {
            return MStatus::kUnknownParameter;
        }
    } else {
        return MStatus::kUnknownParameter;
    }
    return MStatus::kSuccess;
}

void nailConstraintNode::draw( M3dView & view, const MDagPath &path,
                             M3dView::DisplayStyle style,
                             M3dView::DisplayStatus status )
{
  //  std::cout << "nailConstraintNode::draw" << std::endl;

    update();

    view.beginGL();
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    glDisable(GL_LIGHTING);

    if( !(status == M3dView::kActive ||
        status == M3dView::kLead ||
        status == M3dView::kHilite ||
        ( style != M3dView::kGouraudShaded && style != M3dView::kFlatShaded )) ) {
        glColor3f(1.0, 1.0, 0.0); 
    }

    glBegin(GL_LINES);
    glVertex3f(-1.0, 0.0, 0.0);
    glVertex3f(1.0, 0.0, 0.0);

    glVertex3f(0.0, -1.0, 0.0);
    glVertex3f(0.0, 1.0, 0.0);

    glVertex3f(0.0, 0.0, -1.0);
    glVertex3f(0.0, 0.0, 1.0);
    glEnd();

    glPopAttrib();
    view.endGL();
}

bool nailConstraintNode::isBounded() const
{
    //return true;
    return false;
}

MBoundingBox nailConstraintNode::boundingBox() const
{
    // std::cout << "nailConstraintNode::boundingBox()" << std::endl;
    //load the pdbs
    MObject node = thisMObject();

    MPoint corner1(-1, -1, -1);
    MPoint corner2(1, 1, 1);
    return MBoundingBox(corner1, corner2);
}

//standard attributes
void nailConstraintNode::computeConstraint(const MPlug& plug, MDataBlock& data)
{
   // std::cout << "nailConstraintNode::computeConstraint" << std::endl;

    MObject thisObject(thisMObject());
    MPlug plgRigidBody(thisObject, ia_rigidBody);
    MObject update;
    //force evaluation of the rigidBody
    plgRigidBody.getValue(update);

    rigid_body_t::pointer  rigid_body;
    if(plgRigidBody.isConnected()) {
        MPlugArray connections;
        plgRigidBody.connectedTo(connections, true, true);
        if(connections.length() != 0) {
            MFnDependencyNode fnNode(connections[0].node());
            if(fnNode.typeId() == rigidBodyNode::typeId) {
                rigidBodyNode *pRigidBodyNode = static_cast<rigidBodyNode*>(fnNode.userNode());
                rigid_body = pRigidBodyNode->rigid_body();    
            } else {
                std::cout << "nailConstraintNode connected to a non-rigidbody node!" << std::endl;
            }
        }
    }

    if(rigid_body) {
        //not connected to a rigid body, put a default one
        constraint_t::pointer constraint = static_cast<constraint_t::pointer>(m_constraint);
        solver_t::remove_constraint(constraint);
        m_constraint = solver_t::create_nail_constraint(rigid_body);
        constraint = static_cast<constraint_t::pointer>(m_constraint);
        solver_t::add_constraint(constraint);
    }

    data.outputValue(ca_constraint).set(true);
    data.setClean(plug);
}


void nailConstraintNode::computeWorldMatrix(const MPlug& plug, MDataBlock& data)
{
  //  std::cout << "nailConstraintNode::computeWorldMatrix" << std::endl;

    MObject thisObject(thisMObject());
    MFnDagNode fnDagNode(thisObject);

    MObject update;
    MPlug(thisObject, ca_constraint).getValue(update);
    MPlug(thisObject, ca_constraintParam).getValue(update);


    MStatus status;

    MFnTransform fnParentTransform(fnDagNode.parent(0, &status));

    MVector mtranslation = fnParentTransform.getTranslation(MSpace::kTransform, &status);

  //  MQuaternion mrotation;
  //  fnParentTransform.getRotation(mrotation, MSpace::kTransform);

    if(m_constraint) {
        vec3f world_pivot;
        m_constraint->get_world_pivot(world_pivot);
        if(world_pivot[0] != float(mtranslation.x) ||
            world_pivot[1] != float(mtranslation.y) ||
            world_pivot[2] != float(mtranslation.z)) {
           
            mat4x4f xform;
            m_constraint->rigid_body()->get_transform(xform);
            vec4f pivot = prod(trans(xform), vec4f(mtranslation.x, mtranslation.y, mtranslation.z, 1.0));
            m_constraint->set_pivot(vec3f(pivot[0], pivot[1], pivot[2]));
        }
    }

    data.setClean(plug);
}

void nailConstraintNode::computeConstraintParam(const MPlug& plug, MDataBlock& data)
{
   // std::cout << "nailConstraintNode::computeRigidBodyParam" << std::endl;

    MObject thisObject(thisMObject());
    MObject update;
    
    MPlug(thisObject, ca_constraint).getValue(update);
    if(m_constraint) {
        m_constraint->set_damping(data.inputValue(ia_damping).asDouble());
    }

    data.outputValue(ca_constraintParam).set(true);
    data.setClean(plug);
}

nail_constraint_t::pointer nailConstraintNode::constraint()
{
 //   std::cout << "nailConstraintNode::rigid_body" << std::endl;

    MObject thisObject(thisMObject());
    MObject update;
    MPlug(thisObject, ca_constraint).getValue(update);
    MPlug(thisObject, ca_constraintParam).getValue(update);

    return m_constraint;
} 

void nailConstraintNode::update()
{
    MObject thisObject(thisMObject());

    MObject update;
    MPlug(thisObject, ca_constraint).getValue(update);
    MPlug(thisObject, ca_constraintParam).getValue(update);
    MPlug(thisObject, worldMatrix).elementByLogicalIndex(0).getValue(update);
}
