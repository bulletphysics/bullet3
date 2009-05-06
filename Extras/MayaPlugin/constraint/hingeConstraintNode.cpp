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

//hingeConstraintNode.cpp

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
#include "hingeConstraintNode.h"
#include "mayaUtils.h"

#include "solver.h"

MTypeId     hingeConstraintNode::typeId(0x10033B);
MString     hingeConstraintNode::typeName("dHingeConstraint");

MObject     hingeConstraintNode::ia_rigidBody;
MObject     hingeConstraintNode::ia_damping;
MObject     hingeConstraintNode::ia_lowerLimit;
MObject     hingeConstraintNode::ia_upperLimit;
MObject     hingeConstraintNode::ia_limitSoftness;
MObject     hingeConstraintNode::ia_biasFactor;
MObject     hingeConstraintNode::ia_relaxationFactor;
MObject     hingeConstraintNode::ia_hingeAxis;

MObject     hingeConstraintNode::ia_enableAngularMotor;
MObject     hingeConstraintNode::ia_motorTargetVelocity;
MObject     hingeConstraintNode::ia_maxMotorImpulse;

MObject     hingeConstraintNode::ca_constraint;
MObject     hingeConstraintNode::ca_constraintParam;

MStatus hingeConstraintNode::initialize()
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

    ia_lowerLimit = fnNumericAttr.create("lowerLimit", "llmt", MFnNumericData::kDouble, -1.57, &status);
    MCHECKSTATUS(status, "creating lower limit attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_lowerLimit);
    MCHECKSTATUS(status, "adding lower limit attribute")

	ia_upperLimit = fnNumericAttr.create("upperLimit", "ulmt", MFnNumericData::kDouble, 1.57, &status);
    MCHECKSTATUS(status, "creating upper limit attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_upperLimit);
    MCHECKSTATUS(status, "adding upper limit attribute")

	ia_limitSoftness = fnNumericAttr.create("limitSoftness", "lmSo", MFnNumericData::kDouble, 0.9, &status);
    MCHECKSTATUS(status, "creating limitSoftness attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_limitSoftness);
    MCHECKSTATUS(status, "adding limitSoftness attribute")

	ia_biasFactor = fnNumericAttr.create("biasFactor", "biFa", MFnNumericData::kDouble, 0.3, &status);
    MCHECKSTATUS(status, "creating biasFactor attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_biasFactor);
    MCHECKSTATUS(status, "adding biasFactor attribute")

	ia_relaxationFactor = fnNumericAttr.create("relaxationFactor", "reFa", MFnNumericData::kDouble, 1.0, &status);
    MCHECKSTATUS(status, "creating relaxationFactor attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_relaxationFactor);
    MCHECKSTATUS(status, "adding relaxationFactor attribute")

	ia_hingeAxis = fnNumericAttr.createPoint("hingeAxis", "hgAx", &status);
    MCHECKSTATUS(status, "creating hingeAxis attribute")
    status = addAttribute(ia_hingeAxis);
    MCHECKSTATUS(status, "adding hingeAxis attribute")

	//------------------------------------------------------------------------------
	
	ia_enableAngularMotor = fnNumericAttr.create("enableAngularMotor", "enAM", MFnNumericData::kBoolean, false, &status);
    MCHECKSTATUS(status, "creating enableAngularMotor attribute")
    status = addAttribute(ia_enableAngularMotor);
    MCHECKSTATUS(status, "adding enableAngularMotor attribute")

    ia_motorTargetVelocity = fnNumericAttr.create("motorTargetVelocity", "mTV", MFnNumericData::kDouble, 1, &status);
    MCHECKSTATUS(status, "creating motorTargetVelocity attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_motorTargetVelocity);
    MCHECKSTATUS(status, "adding motorTargetVelocity attribute")

	ia_maxMotorImpulse = fnNumericAttr.create("maxMotorImpulse", "mMI", MFnNumericData::kDouble, 1, &status);
    MCHECKSTATUS(status, "creating maxMotorImpulse attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_maxMotorImpulse);
    MCHECKSTATUS(status, "adding maxMotorImpulse attribute")

	//------------------------------------------------------------------------------

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

    status = attributeAffects(ia_lowerLimit, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_lowerLimit, ca_constraintParam)")

	status = attributeAffects(ia_upperLimit, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_upperLimit, ca_constraintParam)")

	status = attributeAffects(ia_limitSoftness, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_limitSoftness, ca_constraintParam)")
	status = attributeAffects(ia_biasFactor, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_biasFactor, ca_constraintParam)")
	status = attributeAffects(ia_relaxationFactor, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_relaxationFactor, ca_constraintParam)")

	status = attributeAffects(ia_hingeAxis, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_hingeAxis, ca_constraintParam)")

	status = attributeAffects(ia_enableAngularMotor, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_enableAngularMotor, ca_constraintParam)")
	status = attributeAffects(ia_motorTargetVelocity, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_motorTargetVelocity, ca_constraintParam)")
	status = attributeAffects(ia_maxMotorImpulse, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_maxMotorImpulse, ca_constraintParam)")

	return MS::kSuccess;
}

hingeConstraintNode::hingeConstraintNode()
{
    // std::cout << "hingeConstraintNode::hingeConstraintNode" << std::endl;
}

hingeConstraintNode::~hingeConstraintNode()
{
    // std::cout << "hingeConstraintNode::~hingeConstraintNode" << std::endl;
}

void hingeConstraintNode::nodeRemoved(MObject& node, void *clientData)
{
   // std::cout << "hingeConstraintNode::nodeRemoved" << std::endl;
    MFnDependencyNode fnNode(node);
    hingeConstraintNode *pNode = static_cast<hingeConstraintNode*>(fnNode.userNode());
    constraint_t::pointer constraint = static_cast<constraint_t::pointer>(pNode->m_constraint);
    solver_t::remove_constraint(constraint);
}

void* hingeConstraintNode::creator()
{
    return new hingeConstraintNode();
}


bool hingeConstraintNode::setInternalValueInContext ( const  MPlug & plug,
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

MStatus hingeConstraintNode::compute(const MPlug& plug, MDataBlock& data)
{
    //std::cout << "hingeConstraintNode::compute: " << plug.name() << std::endl;
    //MTime time = data.inputValue( hingeConstraintNode::inTime ).asTime();
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

void hingeConstraintNode::draw( M3dView & view, const MDagPath &path,
                             M3dView::DisplayStyle style,
                             M3dView::DisplayStatus status )
{
  //  std::cout << "hingeConstraintNode::draw" << std::endl;

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

	vec3f pos;
	if (m_constraint) {
		vec3f world;
		m_constraint->get_world(world);
		vec3f posA;
		quatf rotA;
		m_constraint->rigid_body()->get_transform(posA, rotA);
		pos = posA - world;
	}

    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(pos[0], pos[1], pos[2]);

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

bool hingeConstraintNode::isBounded() const
{
    //return true;
    return false;
}

MBoundingBox hingeConstraintNode::boundingBox() const
{
    // std::cout << "hingeConstraintNode::boundingBox()" << std::endl;
    //load the pdbs
    MObject node = thisMObject();

    MPoint corner1(-1, -1, -1);
    MPoint corner2(1, 1, 1);
    return MBoundingBox(corner1, corner2);
}

//standard attributes
void hingeConstraintNode::computeConstraint(const MPlug& plug, MDataBlock& data)
{
   // std::cout << "hingeConstraintNode::computeConstraint" << std::endl;

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
                std::cout << "hingeConstraintNode connected to a non-rigidbody node!" << std::endl;
            }
        }
    }

    if(rigid_body) {
        //not connected to a rigid body, put a default one
        constraint_t::pointer constraint = static_cast<constraint_t::pointer>(m_constraint);
        solver_t::remove_constraint(constraint);
        m_constraint = solver_t::create_hinge_constraint(rigid_body);
        constraint = static_cast<constraint_t::pointer>(m_constraint);
        solver_t::add_constraint(constraint);
    }

    data.outputValue(ca_constraint).set(true);
    data.setClean(plug);
}


void hingeConstraintNode::computeWorldMatrix(const MPlug& plug, MDataBlock& data)
{
  //  std::cout << "hingeConstraintNode::computeWorldMatrix" << std::endl;

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

void hingeConstraintNode::computeConstraintParam(const MPlug& plug, MDataBlock& data)
{
    //std::cout << "hingeConstraintNode::computeRigidBodyParam data.className=" << std::endl;

    MObject thisObject(thisMObject());
    MObject update;
    
    MPlug(thisObject, ca_constraint).getValue(update);
    if(m_constraint) {
		float damping = (float) data.inputValue(ia_damping).asDouble();
		m_constraint->set_damping(damping);
		float lower = (float) data.inputValue(ia_lowerLimit).asDouble();
		float upper = (float) data.inputValue(ia_upperLimit).asDouble();
		float limit_softness = (float) data.inputValue(ia_limitSoftness).asDouble();
		float bias_factor = (float) data.inputValue(ia_biasFactor).asDouble();
		float relaxation_factor = (float) data.inputValue(ia_relaxationFactor).asDouble();

		m_constraint->set_limit(lower, upper, limit_softness, bias_factor, relaxation_factor);

		float* axis = data.inputValue(ia_hingeAxis).asFloat3();
		m_constraint->set_axis(vec3f(axis[0], axis[1], axis[2]));

		bool enable_motor = data.inputValue(ia_enableAngularMotor).asBool();
		float motorTargetVelocity = (float) data.inputValue(ia_motorTargetVelocity).asDouble();
		float maxMotorImpulse = (float) data.inputValue(ia_maxMotorImpulse).asDouble();
		m_constraint->enable_motor(enable_motor, motorTargetVelocity, maxMotorImpulse);
    }

    data.outputValue(ca_constraintParam).set(true);
    data.setClean(plug);
}

hinge_constraint_t::pointer hingeConstraintNode::constraint()
{
 //   std::cout << "hingeConstraintNode::rigid_body" << std::endl;

    MObject thisObject(thisMObject());
    MObject update;
    MPlug(thisObject, ca_constraint).getValue(update);
    MPlug(thisObject, ca_constraintParam).getValue(update);

    return m_constraint;
} 

void hingeConstraintNode::update()
{
    MObject thisObject(thisMObject());

    MObject update;
    MPlug(thisObject, ca_constraint).getValue(update);
    MPlug(thisObject, ca_constraintParam).getValue(update);
    MPlug(thisObject, worldMatrix).elementByLogicalIndex(0).getValue(update);
}
