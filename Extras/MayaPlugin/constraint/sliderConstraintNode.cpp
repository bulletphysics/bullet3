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

//sliderConstraintNode.cpp

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
#include "sliderConstraintNode.h"
#include "mayaUtils.h"

#include "solver.h"

MTypeId     sliderConstraintNode::typeId(0x100385);
MString     sliderConstraintNode::typeName("dSliderConstraint");

MObject     sliderConstraintNode::ia_rigidBodyA;
MObject     sliderConstraintNode::ia_rigidBodyB;
MObject     sliderConstraintNode::ia_damping;
MObject     sliderConstraintNode::ca_constraint;
MObject     sliderConstraintNode::ca_constraintParam;
MObject     sliderConstraintNode::ia_lowerLinLimit;
MObject     sliderConstraintNode::ia_upperLinLimit;
MObject     sliderConstraintNode::ia_lowerAngLimit;
MObject     sliderConstraintNode::ia_upperAngLimit;

MStatus sliderConstraintNode::initialize()
{
    MStatus status;
    MFnMessageAttribute fnMsgAttr;
    MFnNumericAttribute fnNumericAttr;
    MFnMatrixAttribute fnMatrixAttr;

    ia_rigidBodyA = fnMsgAttr.create("inRigidBodyA", "inrbA", &status);
    MCHECKSTATUS(status, "creating inRigidBodyA attribute")
    status = addAttribute(ia_rigidBodyA);
    MCHECKSTATUS(status, "adding inRigidBody attribute")

    ia_rigidBodyB = fnMsgAttr.create("inRigidBodyB", "inrbB", &status);
    MCHECKSTATUS(status, "creating inRigidBodyB attribute")
    status = addAttribute(ia_rigidBodyB);
    MCHECKSTATUS(status, "adding inRigidBodyB attribute")

	ia_damping = fnNumericAttr.create("damping", "dmp", MFnNumericData::kDouble, 1.0, &status);
    MCHECKSTATUS(status, "creating damping attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_damping);
    MCHECKSTATUS(status, "adding damping attribute")

    ia_lowerLinLimit = fnNumericAttr.create("lowerLinLimit", "lllt", MFnNumericData::kDouble, 1, &status);
    MCHECKSTATUS(status, "creating lower linear limit attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_lowerLinLimit);
    MCHECKSTATUS(status, "adding lower linear limit attribute")

	ia_upperLinLimit = fnNumericAttr.create("upperLinLimit", "ullt", MFnNumericData::kDouble, -1, &status);
    MCHECKSTATUS(status, "creating upper linear limit attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_upperLinLimit);
    MCHECKSTATUS(status, "adding upper linear limit attribute")

    ia_lowerAngLimit = fnNumericAttr.create("lowerAngLimit", "lalt", MFnNumericData::kDouble, 0, &status);
    MCHECKSTATUS(status, "creating lower angular limit attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_lowerAngLimit);
    MCHECKSTATUS(status, "adding lower angular limit attribute")

	ia_upperAngLimit = fnNumericAttr.create("upperAngLimit", "ualt", MFnNumericData::kDouble, 0, &status);
    MCHECKSTATUS(status, "creating upper angular limit attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_upperAngLimit);
    MCHECKSTATUS(status, "adding upper angular limit attribute")

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


    status = attributeAffects(ia_rigidBodyA, ca_constraint);
    MCHECKSTATUS(status, "adding attributeAffects(ia_rigidBodyA, ca_constraint)")

    status = attributeAffects(ia_rigidBodyA, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_rigidBodyA, ca_constraintParam)")

    status = attributeAffects(ia_rigidBodyB, ca_constraint);
    MCHECKSTATUS(status, "adding attributeAffects(ia_rigidBodyB, ca_constraint)")

    status = attributeAffects(ia_rigidBodyB, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_rigidBodyB, ca_constraintParam)")

	status = attributeAffects(ia_damping, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_damping, ca_constraintParam)")

    status = attributeAffects(ia_lowerLinLimit, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_lowerLinLimit, ca_constraintParam)")

	status = attributeAffects(ia_upperLinLimit, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_upperLinLimit, ca_constraintParam)")

    status = attributeAffects(ia_lowerAngLimit, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_lowerAngLimit, ca_constraintParam)")

	status = attributeAffects(ia_upperAngLimit, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_upperAngLimit, ca_constraintParam)")

	return MS::kSuccess;
}

sliderConstraintNode::sliderConstraintNode()
{
    // std::cout << "sliderConstraintNode::sliderConstraintNode" << std::endl;
}

sliderConstraintNode::~sliderConstraintNode()
{
    // std::cout << "sliderConstraintNode::~sliderConstraintNode" << std::endl;
}

void sliderConstraintNode::nodeRemoved(MObject& node, void *clientData)
{
   // std::cout << "sliderConstraintNode::nodeRemoved" << std::endl;
    MFnDependencyNode fnNode(node);
    sliderConstraintNode *pNode = static_cast<sliderConstraintNode*>(fnNode.userNode());
    constraint_t::pointer constraint = static_cast<constraint_t::pointer>(pNode->m_constraint);
    solver_t::remove_constraint(constraint);
}

void* sliderConstraintNode::creator()
{
    return new sliderConstraintNode();
}


bool sliderConstraintNode::setInternalValueInContext ( const  MPlug & plug,
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

MStatus sliderConstraintNode::compute(const MPlug& plug, MDataBlock& data)
{
    //std::cout << "sliderConstraintNode::compute: " << plug.name() << std::endl;
    //MTime time = data.inputValue( sliderConstraintNode::inTime ).asTime();
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

void sliderConstraintNode::draw( M3dView & view, const MDagPath &path,
                             M3dView::DisplayStyle style,
                             M3dView::DisplayStatus status )
{
  //  std::cout << "sliderConstraintNode::draw" << std::endl;

    update();

    view.beginGL();
    glPushAttrib( GL_ALL_ATTRIB_BITS );

//	glPushMatrix();
    glDisable(GL_LIGHTING);

    if( !(status == M3dView::kActive ||
        status == M3dView::kLead ||
        status == M3dView::kHilite ||
        ( style != M3dView::kGouraudShaded && style != M3dView::kFlatShaded )) ) {
        glColor3f(1.0, 1.0, 0.0); 
    }

	vec3f posA;
	vec3f posB;
	if (m_constraint) {
		vec3f world;
		m_constraint->get_world(world);
		quatf rotA;
		m_constraint->rigid_bodyA()->get_transform(posA, rotA);
		posA = posA - world;
		quatf rotB;
		m_constraint->rigid_bodyB()->get_transform(posB, rotB);
		posB = posB - world;
	}

//	glLoadIdentity();
    glBegin(GL_LINES);
    glVertex3f(posA[0], posA[1], posA[2]);
    glVertex3f(posB[0], posB[1], posB[2]);

    glVertex3f(-1.0, 0.0, 0.0);
    glVertex3f(1.0, 0.0, 0.0);

    glVertex3f(0.0, -1.0, 0.0);
    glVertex3f(0.0, 1.0, 0.0);

    glVertex3f(0.0, 0.0, -1.0);
    glVertex3f(0.0, 0.0, 1.0);
    glEnd();

//	glPopMatrix();

    glPopAttrib();
    view.endGL();
}

bool sliderConstraintNode::isBounded() const
{
    //return true;
    return false;
}

MBoundingBox sliderConstraintNode::boundingBox() const
{
    // std::cout << "sliderConstraintNode::boundingBox()" << std::endl;
    //load the pdbs
    MObject node = thisMObject();

    MPoint corner1(-1, -1, -1);
    MPoint corner2(1, 1, 1);
    return MBoundingBox(corner1, corner2);
}

//standard attributes
void sliderConstraintNode::computeConstraint(const MPlug& plug, MDataBlock& data)
{
   // std::cout << "sliderConstraintNode::computeConstraint" << std::endl;

    MObject thisObject(thisMObject());
    MPlug plgRigidBodyA(thisObject, ia_rigidBodyA);
    MPlug plgRigidBodyB(thisObject, ia_rigidBodyB);
    MObject update;
    //force evaluation of the rigidBody
    plgRigidBodyA.getValue(update);
    plgRigidBodyB.getValue(update);

    rigid_body_t::pointer  rigid_bodyA;
    if(plgRigidBodyA.isConnected()) {
        MPlugArray connections;
        plgRigidBodyA.connectedTo(connections, true, true);
        if(connections.length() != 0) {
            MFnDependencyNode fnNodeA(connections[0].node());
            if(fnNodeA.typeId() == rigidBodyNode::typeId) {
                rigidBodyNode *pRigidBodyNodeA = static_cast<rigidBodyNode*>(fnNodeA.userNode());
                rigid_bodyA = pRigidBodyNodeA->rigid_body();    
            } else {
                std::cout << "sliderConstraintNode connected to a non-rigidbody node!" << std::endl;
            }
        }
    }

    rigid_body_t::pointer  rigid_bodyB;
	if(plgRigidBodyB.isConnected()) {
        MPlugArray connections;
        plgRigidBodyB.connectedTo(connections, true, true);
        if(connections.length() != 0) {
            MFnDependencyNode fnNodeB(connections[0].node());
            if(fnNodeB.typeId() == rigidBodyNode::typeId) {
                rigidBodyNode *pRigidBodyNodeB = static_cast<rigidBodyNode*>(fnNodeB.userNode());
                rigid_bodyB = pRigidBodyNodeB->rigid_body();    
            } else {
                std::cout << "sliderConstraintNode connected to a non-rigidbody node!" << std::endl;
            }
        }
    }

    if(rigid_bodyA && rigid_bodyB) {
        //not connected to a rigid body, put a default one
        constraint_t::pointer constraint = static_cast<constraint_t::pointer>(m_constraint);
        solver_t::remove_constraint(constraint);
        m_constraint = solver_t::create_slider_constraint(rigid_bodyA, vec3f(), rigid_bodyB, vec3f());
        constraint = static_cast<constraint_t::pointer>(m_constraint);
        solver_t::add_constraint(constraint);
    }

    data.outputValue(ca_constraint).set(true);
    data.setClean(plug);
}


void sliderConstraintNode::computeWorldMatrix(const MPlug& plug, MDataBlock& data)
{
  //  std::cout << "sliderConstraintNode::computeWorldMatrix" << std::endl;

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
        m_constraint->get_world(world_pivot);
        if(world_pivot[0] != float(mtranslation.x) ||
            world_pivot[1] != float(mtranslation.y) ||
            world_pivot[2] != float(mtranslation.z)) {
           
//            mat4x4f xform;
//            m_constraint->rigid_body()->get_transform(xform);
//            vec4f pivot = prod(trans(xform), vec4f(mtranslation.x, mtranslation.y, mtranslation.z, 1.0));
//            m_constraint->set_pivot(vec3f(pivot[0], pivot[1], pivot[2]));
			m_constraint->set_world(vec3f((float) mtranslation[0], (float) mtranslation[1], (float) mtranslation[2]));
        }
    }

    data.setClean(plug);
}

void sliderConstraintNode::computeConstraintParam(const MPlug& plug, MDataBlock& data)
{
   // std::cout << "sliderConstraintNode::computeRigidBodyParam" << std::endl;

    MObject thisObject(thisMObject());
    MObject update;
    
    MPlug(thisObject, ca_constraint).getValue(update);
    if(m_constraint) {
        m_constraint->set_damping((float) data.inputValue(ia_damping).asDouble());
		float lin_lower = (float) data.inputValue(ia_lowerLinLimit).asDouble();
		float lin_upper = (float) data.inputValue(ia_upperLinLimit).asDouble();
		m_constraint->set_LinLimit(lin_lower, lin_upper);
		float ang_lower = (float) data.inputValue(ia_lowerAngLimit).asDouble();
		float ang_upper = (float) data.inputValue(ia_upperAngLimit).asDouble();
		m_constraint->set_AngLimit(ang_lower, ang_upper);
    }

    data.outputValue(ca_constraintParam).set(true);
    data.setClean(plug);
}

slider_constraint_t::pointer sliderConstraintNode::constraint()
{
 //   std::cout << "sliderConstraintNode::rigid_body" << std::endl;

    MObject thisObject(thisMObject());
    MObject update;
    MPlug(thisObject, ca_constraint).getValue(update);
    MPlug(thisObject, ca_constraintParam).getValue(update);

    return m_constraint;
} 

void sliderConstraintNode::update()
{
    MObject thisObject(thisMObject());

    MObject update;
    MPlug(thisObject, ca_constraint).getValue(update);
    MPlug(thisObject, ca_constraintParam).getValue(update);
    MPlug(thisObject, worldMatrix).elementByLogicalIndex(0).getValue(update);
}
