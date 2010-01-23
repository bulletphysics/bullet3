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

Modified by Roman Ponomarev <rponom@gmail.com>
01/22/2010 : Constraints reworked
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
#include "dSolverNode.h"
#include "constraint/bt_nail_constraint.h"

MTypeId     nailConstraintNode::typeId(0x10033A);
MString     nailConstraintNode::typeName("dNailConstraint");

MObject     nailConstraintNode::ia_rigidBodyA;
MObject     nailConstraintNode::ia_rigidBodyB;
MObject     nailConstraintNode::ia_damping;
MObject     nailConstraintNode::ia_pivotInA;
MObject     nailConstraintNode::ia_pivotInB;
MObject     nailConstraintNode::ca_constraint;
MObject     nailConstraintNode::ca_constraintParam;


MStatus nailConstraintNode::initialize()
{
    MStatus status;
    MFnMessageAttribute fnMsgAttr;
    MFnNumericAttribute fnNumericAttr;
    MFnMatrixAttribute fnMatrixAttr;

    ia_rigidBodyA = fnMsgAttr.create("inRigidBodyA", "inrbA", &status);
    MCHECKSTATUS(status, "creating inRigidBodyA attribute")
    status = addAttribute(ia_rigidBodyA);
    MCHECKSTATUS(status, "adding inRigidBodyA attribute")

    ia_rigidBodyB = fnMsgAttr.create("inRigidBodyB", "inrbB", &status);
    MCHECKSTATUS(status, "creating inRigidBodyB attribute")
    status = addAttribute(ia_rigidBodyB);
    MCHECKSTATUS(status, "adding inRigidBodyB attribute")

    ia_damping = fnNumericAttr.create("damping", "dmp", MFnNumericData::kDouble, 1.0, &status);
    MCHECKSTATUS(status, "creating damping attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_damping);
    MCHECKSTATUS(status, "adding damping attribute")

    ia_pivotInA = fnNumericAttr.createPoint("pivotInA", "piva", &status);
    MCHECKSTATUS(status, "creating pivotInA attribute")
    status = addAttribute(ia_pivotInA);
    MCHECKSTATUS(status, "adding pivotInA attribute")

    ia_pivotInB = fnNumericAttr.createPoint("pivotInB", "pivb", &status);
    MCHECKSTATUS(status, "creating pivotInB attribute")
    status = addAttribute(ia_pivotInB);
    MCHECKSTATUS(status, "adding pivotInB attribute")

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

    status = attributeAffects(ia_pivotInA, ca_constraint);
    MCHECKSTATUS(status, "adding attributeAffects(ia_pivotInA, ca_constraint)")

	status = attributeAffects(ia_pivotInA, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_pivotInA, ca_constraintParam)")

    status = attributeAffects(ia_pivotInB, ca_constraint);
    MCHECKSTATUS(status, "adding attributeAffects(ia_pivotInB, ca_constraint)")

	status = attributeAffects(ia_pivotInB, ca_constraintParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_pivotInB, ca_constraintParam)")

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
	if (pNode->m_constraint) 
	{
        bt_nail_constraint_t* nail_impl = dynamic_cast<bt_nail_constraint_t*>(pNode->m_constraint->pubImpl());

		rigid_body_t::pointer rigid_bodyA = pNode->m_constraint->rigid_bodyA();
		if(rigid_bodyA)
		{
			rigid_bodyA->remove_constraint(nail_impl);
		}
		rigid_body_t::pointer rigid_bodyB = pNode->m_constraint->rigid_bodyB();
		if(rigid_bodyB)
		{
			rigid_bodyB->remove_constraint(nail_impl);
		}
	}
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
	vec3f posA, posB;
	rigid_body_t::pointer rigid_bodyB = NULL;
	vec3f world;
	if (m_constraint) {
		m_constraint->get_world(world);
		vec3f posT;
		quatf rotT;
		m_constraint->rigid_bodyA()->get_transform(posT, rotT);
		posA = posT - world;
		rigid_bodyB = m_constraint->rigid_bodyB();
		if(rigid_bodyB)
		{
			rigid_bodyB->get_transform(posT, rotT);
			posB = posT - world;
		}
	}

    glBegin(GL_LINES);

    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(posA[0], posA[1], posA[2]);

	if(rigid_bodyB)
	{
		vec3f pivB;
		m_constraint->get_world_pivotB(pivB);
		for(int j = 0; j < 3; j++) pivB[j] -= world[j];
		glVertex3f(pivB[0], pivB[1], pivB[2]);
		glVertex3f(posB[0], posB[1], posB[2]);
		glVertex3f(-1.0f+pivB[0],  0.0f+pivB[1],  0.0f+pivB[2]);
		glVertex3f( 1.0f+pivB[0],  0.0f+pivB[1],  0.0f+pivB[2]);
		glVertex3f( 0.0f+pivB[0], -1.0f+pivB[1],  0.0f+pivB[2]);
		glVertex3f( 0.0f+pivB[0],  1.0f+pivB[1],  0.0f+pivB[2]);
	    glVertex3f( 0.0f+pivB[0],  0.0f+pivB[1], -1.0f+pivB[2]);
		glVertex3f( 0.0f+pivB[0],  0.0f+pivB[1],  1.0f+pivB[2]);
	}


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
            MFnDependencyNode fnNode(connections[0].node());
            if(fnNode.typeId() == rigidBodyNode::typeId) {
                rigidBodyNode *pRigidBodyNodeA = static_cast<rigidBodyNode*>(fnNode.userNode());
                rigid_bodyA = pRigidBodyNodeA->rigid_body();    
            } else {
                std::cout << "nailConstraintNode connected to a non-rigidbody node A!" << std::endl;
            }
        }
    }

    rigid_body_t::pointer  rigid_bodyB;
    if(plgRigidBodyB.isConnected()) {
        MPlugArray connections;
        plgRigidBodyB.connectedTo(connections, true, true);
        if(connections.length() != 0) {
            MFnDependencyNode fnNode(connections[0].node());
            if(fnNode.typeId() == rigidBodyNode::typeId) {
                rigidBodyNode *pRigidBodyNodeB = static_cast<rigidBodyNode*>(fnNode.userNode());
                rigid_bodyB = pRigidBodyNodeB->rigid_body();    
            } else {
                std::cout << "nailConstraintNode connected to a non-rigidbody node B!" << std::endl;
            }
        }
    }

	vec3f pivInA, pivInB;

	if((rigid_bodyA != NULL) && (rigid_bodyB != NULL))
	{
        constraint_t::pointer constraint = static_cast<constraint_t::pointer>(m_constraint);
        solver_t::remove_constraint(constraint);
		float3& mPivInA = data.inputValue(ia_pivotInA).asFloat3();
		float3& mPivInB = data.inputValue(ia_pivotInB).asFloat3();
		for(int i = 0; i < 3; i++)
		{
			pivInA[i] = (float)mPivInA[i];
			pivInB[i] = (float)mPivInB[i];
		}
        m_constraint = solver_t::create_nail_constraint(rigid_bodyA, rigid_bodyB, pivInA, pivInB);
        constraint = static_cast<constraint_t::pointer>(m_constraint);
        solver_t::add_constraint(constraint);
	}
    else if(rigid_bodyA) 
	{
        //not connected to a rigid body, put a default one
        constraint_t::pointer constraint = static_cast<constraint_t::pointer>(m_constraint);
        solver_t::remove_constraint(constraint);
		float3& mPivInA = data.inputValue(ia_pivotInA).asFloat3();
		for(int i = 0; i < 3; i++)
		{
			pivInA[i] = (float)mPivInA[i];
		}
        m_constraint = solver_t::create_nail_constraint(rigid_bodyA, pivInA);
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
	fnParentTransform.setRotation(MEulerRotation(0., 0., 0.)); // lock rotation
	double fixScale[3] = { 1., 1., 1. };  // lock scale
	fnParentTransform.setScale(fixScale);
    MVector mtranslation = fnParentTransform.getTranslation(MSpace::kTransform, &status);

	if(dSolverNode::isStartTime)
	{	// allow to edit pivots
		MPlug plgRigidBodyA(thisObject, ia_rigidBodyA);
		MPlug plgRigidBodyB(thisObject, ia_rigidBodyB);
		MObject update;
		//force evaluation of the rigidBody
		plgRigidBodyA.getValue(update);
		if(plgRigidBodyA.isConnected()) 
		{
			MPlugArray connections;
			plgRigidBodyA.connectedTo(connections, true, true);
			if(connections.length() != 0) 
			{
				MFnDependencyNode fnNode(connections[0].node());
				if(fnNode.typeId() == rigidBodyNode::typeId) 
				{
					MObject rbAObj = fnNode.object();
					rigidBodyNode *pRigidBodyNodeA = static_cast<rigidBodyNode*>(fnNode.userNode());
					MPlug(rbAObj, pRigidBodyNodeA->worldMatrix).elementByLogicalIndex(0).getValue(update);
				}
			}
		}
		plgRigidBodyB.getValue(update);
		if(plgRigidBodyB.isConnected()) 
		{
			MPlugArray connections;
			plgRigidBodyB.connectedTo(connections, true, true);
			if(connections.length() != 0) 
			{
	            MFnDependencyNode fnNode(connections[0].node());
				if(fnNode.typeId() == rigidBodyNode::typeId) 
				{
					MObject rbBObj = fnNode.object();
					rigidBodyNode *pRigidBodyNodeB = static_cast<rigidBodyNode*>(fnNode.userNode());
					MPlug(rbBObj, pRigidBodyNodeB->worldMatrix).elementByLogicalIndex(0).getValue(update);
				}
			}
		}
		if(m_constraint) 
		{
			bool doUpdatePivot = m_constraint->getPivotChanged();
			if(!doUpdatePivot)
			{
				vec3f world;
				m_constraint->get_world(world);
				float deltaX = world[0] - float(mtranslation.x);
				float deltaY = world[1] - float(mtranslation.y);
				float deltaZ = world[2] - float(mtranslation.z);
				float deltaSq = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
				doUpdatePivot = (deltaSq > FLT_EPSILON);
			}
			if(doUpdatePivot)
			{
				m_constraint->set_world(vec3f((float) mtranslation[0], (float) mtranslation[1], (float) mtranslation[2]));
				vec3f pivInA, pivInB;
				m_constraint->get_pivotA(pivInA);
				m_constraint->get_pivotB(pivInB);
				MDataHandle hPivInA = data.outputValue(ia_pivotInA);
				float3 &ihPivInA = hPivInA.asFloat3();
				MDataHandle hPivInB = data.outputValue(ia_pivotInB);
				float3 &ihPivInB = hPivInB.asFloat3();
				for(int i = 0; i < 3; i++) 
				{ 
					ihPivInA[i] = pivInA[i]; 
					ihPivInB[i] = pivInB[i]; 
				}
				m_constraint->setPivotChanged(false);
			}
		}
	}
	else
	{ // if not start time, lock position
		if(m_constraint) 
		{
			vec3f world;
			m_constraint->get_world(world);
            fnParentTransform.setTranslation(MVector(world[0], world[1], world[2]), MSpace::kTransform);
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
        m_constraint->set_damping((float) data.inputValue(ia_damping).asDouble());
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
