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

//dSolverNode.cpp

#include <maya/MFnNumericAttribute.h>           
#include <maya/MFnEnumAttribute.h>           
#include <maya/MFnTypedAttribute.h>           
#include <maya/MFnUnitAttribute.h>           
#include <maya/MFnMessageAttribute.h>           
#include <maya/MFnStringData.h>           
#include <maya/MFnStringArrayData.h>
#include <maya/MPlugArray.h>
#include <maya/MFnDagNode.h>
#include <maya/MEulerRotation.h>
#include <maya/MQuaternion.h>
#include <maya/MFnTransform.h>
#include <maya/MVector.h>
#include <maya/MGlobal.h>
#include <maya/MFnField.h>
#include <maya/MVectorArray.h>
#include <maya/MItDag.h>
#include <maya/MConditionMessage.h>

#include <fstream>
#include <sstream>

#include "mayaUtils.h"
#include "dSolverNode.h"
#include "solver.h"
#include "rigidBodyNode.h"
#include "rigidBodyArrayNode.h"
#include "pdbIO.h"

MTypeId dSolverNode::typeId(0x100331);
MString dSolverNode::typeName("dSolver");

MObject dSolverNode::ia_time;
MObject dSolverNode::ia_startTime;
MObject dSolverNode::ia_gravity;
MObject dSolverNode::ia_enabled;
MObject dSolverNode::ia_splitImpulse;
MObject dSolverNode::ia_substeps;
MObject dSolverNode::oa_rigidBodies;
MObject dSolverNode::ssSolverType;

#define ATTR_POSITION "position"
//#define ATTR_POSITION_TYPE VECTOR_ATTR
#define ATTR_VELOCITY "velocity"
//#define ATTR_VELOCITY_TYPE VECTOR_ATTR

#define ATTR_IN_RAXIS  "in_rotvec"               /* vector */
//#define ATTR_IN_RAXIS_TYPE VECTOR_ATTR
#define ATTR_IN_RANGLE "in_rotang"               /* float  */
//#define ATTR_IN_RANGLE_TYPE FLOAT_ATTR
#define ATTR_IN_ROTVEL "in_rotvel"               /* float  */
//#define ATTR_IN_ROTVEL_TYPE FLOAT_ATTR
#define ATTR_IN_RAXISNEXT  "in_rotvec_next"      /* vector */
//#define ATTR_IN_RAXISNEXT_TYPE VECTOR_ATTR
#define ATTR_IN_RANGLENEXT "in_rotang_next"      /* float  */
//#define ATTR_IN_RANGLENEXT_TYPE FLOAT_ATTR



MStatus dSolverNode::initialize()
{ 
    MStatus                 status;
    MFnEnumAttribute        fnEnumAttr;
    MFnMessageAttribute     fnMsgAttr;
    MFnUnitAttribute        fnUnitAttr;
    MFnNumericAttribute     fnNumericAttr;

    //
    ssSolverType = fnEnumAttr.create( "ssSolverType", "ssst", 0, &status );
    MCHECKSTATUS(status, "creating ssSolverType attribute")
    fnEnumAttr.addField( "Bullet Physics", 0 );
    fnEnumAttr.addField( "Ageia PhysX", 1 );
    fnEnumAttr.addField( "Stanford PhysBAM", 2 );
    status = addAttribute(ssSolverType);
    MCHECKSTATUS(status, "adding ssSolverType attribute")

    //
    ia_time = fnUnitAttr.create( "inTime", "it", MFnUnitAttribute::kTime, 0.0, &status );
    MCHECKSTATUS(status, "creating ia_time attribute")
    fnUnitAttr.setHidden(true);
    status = addAttribute(ia_time);
    MCHECKSTATUS(status, "adding ia_time attribute")

    ia_startTime = fnUnitAttr.create( "startTime", "stm", MFnUnitAttribute::kTime, 1.0, &status );
    MCHECKSTATUS(status, "creating ia_startTime attribute")
    status = addAttribute(ia_startTime);
    MCHECKSTATUS(status, "adding ia_startTime attribute")

    oa_rigidBodies = fnMsgAttr.create("rigidBodies", "rbds", &status);
    MCHECKSTATUS(status, "creating oa_rigidBodies attribute")
    status = addAttribute(oa_rigidBodies);
    MCHECKSTATUS(status, "adding oa_rigidBodies attribute")

    ia_gravity = fnNumericAttr.createPoint("gravity", "grvt", &status);
    MCHECKSTATUS(status, "creating gravity attribute")
    fnNumericAttr.setDefault(0.0, -9.81, 0.0);
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_gravity);
    MCHECKSTATUS(status, "adding ia_gravity attribute")

    ia_substeps = fnNumericAttr.create("substeps", "sbs", MFnNumericData::kInt, 1, &status);
    MCHECKSTATUS(status, "creating substeps attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_substeps);
    MCHECKSTATUS(status, "adding ia_substeps attribute")

    ia_enabled = fnNumericAttr.create("enabled", "enbl", MFnNumericData::kBoolean, true, &status);
    MCHECKSTATUS(status, "creating enabled attribute")
    status = addAttribute(ia_enabled);
    MCHECKSTATUS(status, "adding ia_enabled attribute")

    ia_splitImpulse = fnNumericAttr.create("splitImpulse", "spli", MFnNumericData::kBoolean, false, &status);
    MCHECKSTATUS(status, "creating splitImpulse attribute")
    status = addAttribute(ia_splitImpulse);
    MCHECKSTATUS(status, "adding ia_splitImpulse attribute")

    status = attributeAffects(ia_time, oa_rigidBodies);
    MCHECKSTATUS(status, "adding attributeAffects(ia_time, oa_rigidBodies)")

    status = attributeAffects(ia_enabled, oa_rigidBodies);
    MCHECKSTATUS(status, "adding attributeAffects(ia_enabled, oa_rigidBodies)")

    return MS::kSuccess;
}

dSolverNode::dSolverNode()
{
}

dSolverNode::~dSolverNode()
{
}

void dSolverNode::postConstructor()
{
    //prevent deletion if all the rigidbodies are deleted
    setExistWithoutOutConnections(true);
}

void* dSolverNode::creator()
{
    return new dSolverNode();
}

bool dSolverNode::setInternalValueInContext( const  MPlug & plug, const  MDataHandle & dataHandle,  MDGContext & ctx )
{
    return false;
}

MStatus dSolverNode::compute(const MPlug& plug, MDataBlock& data)
{
    if(plug == oa_rigidBodies) {
         computeRigidBodies(plug, data);
    } else {
        return MStatus::kUnknownParameter;
    } 
    return MStatus::kSuccess;
}

void initRigidBody(MObject &node)
{
    MFnDagNode fnDagNode(node);

    rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
    rigid_body_t::pointer rb = rbNode->rigid_body();

    if(fnDagNode.parentCount() == 0) {
        std::cout << "No transform found!" << std::endl;
        return;
    }

    MFnTransform fnTransform(fnDagNode.parent(0));

    MPlug plgMass(node, rigidBodyNode::ia_mass);
    float mass = 0.f;
    plgMass.getValue(mass);
    if(mass>0.f) {
        //active rigid body, set the world transform from the initial* attributes
        MObject obj;

        vec3f pos;
        MPlug plgInitialValue(node, rigidBodyNode::ia_initialPosition);
        plgInitialValue.child(0).getValue(pos[0]);
        plgInitialValue.child(1).getValue(pos[1]);
        plgInitialValue.child(2).getValue(pos[2]);

        vec3f rot;
        plgInitialValue.setAttribute(rigidBodyNode::ia_initialRotation);
        plgInitialValue.child(0).getValue(rot[0]);
        plgInitialValue.child(1).getValue(rot[1]);
        plgInitialValue.child(2).getValue(rot[2]);

        vec3f vel;
        plgInitialValue.setAttribute(rigidBodyNode::ia_initialVelocity);
        plgInitialValue.child(0).getValue(vel[0]);
        plgInitialValue.child(1).getValue(vel[1]);
        plgInitialValue.child(2).getValue(vel[2]);

        vec3f spin;
        plgInitialValue.setAttribute(rigidBodyNode::ia_initialSpin);
        plgInitialValue.child(0).getValue(spin[0]);
        plgInitialValue.child(1).getValue(spin[1]);
        plgInitialValue.child(2).getValue(spin[2]);

        MEulerRotation meuler(deg2rad(rot[0]), deg2rad(rot[1]), deg2rad(rot[2]));
        MQuaternion mquat = meuler.asQuaternion();
        rb->set_transform(pos, quatf(mquat.w, mquat.x, mquat.y, mquat.z)); 
        rb->set_linear_velocity(vel);
        rb->set_angular_velocity(spin);
        rb->set_kinematic(false);

        fnTransform.setRotation(meuler);
        fnTransform.setTranslation(MVector(pos[0], pos[1], pos[2]), MSpace::kTransform);
    } else {
        //passive rigid body, get the world trasform from from the shape node
        MQuaternion mquat;
        fnTransform.getRotation(mquat);
        MVector mpos(fnTransform.getTranslation(MSpace::kTransform));
        rb->set_transform(vec3f(mpos.x, mpos.y, mpos.z), quatf(mquat.w, mquat.x, mquat.y, mquat.z));
        rb->set_kinematic(true);
    }
}

void initRigidBodyArray(MObject &node)
{
    MFnDagNode fnDagNode(node);

    rigidBodyArrayNode *rbNode = static_cast<rigidBodyArrayNode*>(fnDagNode.userNode()); 
    std::vector<rigid_body_t::pointer>& rbs = rbNode->rigid_bodies();

    if(fnDagNode.parentCount() == 0) {
        std::cout << "No transform found!" << std::endl;
        return;
    }

    MFnTransform fnTransform(fnDagNode.parent(0));

    MPlug plgMass(node, rigidBodyArrayNode::ia_mass);
    float mass = 0.f;
    plgMass.getValue(mass);
	bool active = (mass>0.f);

    if(active) {
        //active rigid body, set the world transform from the initial* attributes
        MObject obj;

        MPlug plgInitialPosition(node, rigidBodyArrayNode::ia_initialPosition);
        MPlug plgInitialRotation(node, rigidBodyArrayNode::ia_initialRotation);
        MPlug plgInitialVelocity(node, rigidBodyArrayNode::ia_initialVelocity);
        MPlug plgInitialSpin(node, rigidBodyArrayNode::ia_initialSpin);

        MPlug plgPosition(node, rigidBodyArrayNode::io_position);
        MPlug plgRotation(node, rigidBodyArrayNode::io_rotation);

        MPlug plgElement;
        for(size_t j = 0; j < rbs.size(); ++j) {
            vec3f pos;
            plgElement = plgInitialPosition.elementByLogicalIndex(j);
            plgElement.child(0).getValue(pos[0]);
            plgElement.child(1).getValue(pos[1]);
            plgElement.child(2).getValue(pos[2]);

            vec3f rot;
            plgElement = plgInitialRotation.elementByLogicalIndex(j);
            plgElement.child(0).getValue(rot[0]);
            plgElement.child(1).getValue(rot[1]);
            plgElement.child(2).getValue(rot[2]);

            vec3f vel;
            plgElement = plgInitialVelocity.elementByLogicalIndex(j);
            plgElement.child(0).getValue(vel[0]);
            plgElement.child(1).getValue(vel[1]);
            plgElement.child(2).getValue(vel[2]);

            vec3f spin;
            plgElement = plgInitialSpin.elementByLogicalIndex(j);
            plgElement.child(0).getValue(spin[0]);
            plgElement.child(1).getValue(spin[1]);
            plgElement.child(2).getValue(spin[2]);

            MEulerRotation meuler(deg2rad(rot[0]), deg2rad(rot[1]), deg2rad(rot[2]));
            MQuaternion mquat = meuler.asQuaternion();

            rbs[j]->set_transform(pos, quatf(mquat.w, mquat.x, mquat.y, mquat.z)); 
            rbs[j]->set_linear_velocity(vel);
            rbs[j]->set_angular_velocity(spin);
            rbs[j]->set_kinematic(false);

            plgElement = plgPosition.elementByLogicalIndex(j);
            plgElement.child(0).setValue(pos[0]);
            plgElement.child(1).setValue(pos[1]);
            plgElement.child(2).setValue(pos[2]);

            plgElement = plgRotation.elementByLogicalIndex(j);
            plgElement.child(0).setValue(rot[0]);
            plgElement.child(1).setValue(rot[1]);
            plgElement.child(2).setValue(rot[2]);
        }

    } else {
        //passive rigid body, get the world trasform from from the position/rotation attributes
        MPlug plgPosition(node, rigidBodyArrayNode::io_position);
        MPlug plgRotation(node, rigidBodyArrayNode::io_rotation);

        MPlug plgElement;
        for(size_t j = 0; j < rbs.size(); ++j) {
            vec3f pos;
            plgElement = plgPosition.elementByLogicalIndex(j);
            plgElement.child(0).getValue(pos[0]);
            plgElement.child(1).getValue(pos[1]);
            plgElement.child(2).getValue(pos[2]);

            vec3f rot;
            plgElement = plgRotation.elementByLogicalIndex(j);
            plgElement.child(0).getValue(rot[0]);
            plgElement.child(1).getValue(rot[1]);
            plgElement.child(2).getValue(rot[2]);

            MEulerRotation meuler(deg2rad(rot[0]), deg2rad(rot[1]), deg2rad(rot[2]));
            MQuaternion mquat = meuler.asQuaternion();
            rbs[j]->set_transform(pos, quatf(mquat.w, mquat.x, mquat.y, mquat.z)); 
            rbs[j]->set_kinematic(false);
        }
    }
}

//init the rigid bodies to it's first frame configuration
void dSolverNode::initRigidBodies(MPlugArray &rbConnections)
{
    for(size_t i = 0; i < rbConnections.length(); ++i) {
        MObject node = rbConnections[i].node();
        MFnDependencyNode fnNode(node);

        if(fnNode.typeId() == rigidBodyNode::typeId) {
            initRigidBody(node);
        } else if(fnNode.typeId() == rigidBodyArrayNode::typeId) {
            initRigidBodyArray(node);
        }
    }
}

//gather previous and current frame transformations for substep interpolation
void dSolverNode::gatherPassiveTransforms(MPlugArray &rbConnections, std::vector<xforms_t> &xforms)
{
    xforms.resize(0);
    xforms_t xform;


    for(size_t i = 0; i < rbConnections.length(); ++i) {
        MObject node = rbConnections[i].node();
        MFnDagNode fnDagNode(node);
        if(fnDagNode.typeId() == rigidBodyNode::typeId) {
            rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
            rigid_body_t::pointer rb = rbNode->rigid_body();

            if(fnDagNode.parentCount() == 0) {
                std::cout << "No transform found!" << std::endl;
                continue;
            }

            MFnTransform fnTransform(fnDagNode.parent(0));

            MPlug plgMass(node, rigidBodyNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(!active) {
                MQuaternion mquat;
                fnTransform.getRotation(mquat);
                MVector mpos(fnTransform.getTranslation(MSpace::kTransform));
                rb->get_transform(xform.m_x0, xform.m_q0); 
                
                xform.m_x1 = vec3f(mpos.x, mpos.y, mpos.z);
                xform.m_q1 = quatf(mquat.w, mquat.x, mquat.y, mquat.z); 
                xforms.push_back(xform);
            }
        } else if(fnDagNode.typeId() == rigidBodyArrayNode::typeId) {
            rigidBodyArrayNode *rbNode = static_cast<rigidBodyArrayNode*>(fnDagNode.userNode()); 
            std::vector<rigid_body_t::pointer>& rbs = rbNode->rigid_bodies();
        
            if(fnDagNode.parentCount() == 0) {
                std::cout << "No transform found!" << std::endl;
                return;
            }

            MPlug plgMass(node, rigidBodyArrayNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(!active) {
                MPlug plgPosition(node, rigidBodyArrayNode::io_position);
                MPlug plgRotation(node, rigidBodyArrayNode::io_rotation);

                MPlug plgElement;
                for(size_t j = 0; j < rbs.size(); ++j) {
                    rbs[j]->get_transform(xform.m_x0, xform.m_q0); 

                    plgElement = plgPosition.elementByLogicalIndex(j);
                    plgElement.child(0).getValue(xform.m_x1[0]);
                    plgElement.child(1).getValue(xform.m_x1[1]);
                    plgElement.child(2).getValue(xform.m_x1[2]);

                    vec3f rot;
                    plgElement = plgRotation.elementByLogicalIndex(j);
                    plgElement.child(0).getValue(rot[0]);
                    plgElement.child(1).getValue(rot[1]);
                    plgElement.child(2).getValue(rot[2]);

                    MEulerRotation meuler(deg2rad(rot[0]), deg2rad(rot[1]), deg2rad(rot[2]));
                    MQuaternion mquat = meuler.asQuaternion();
                    xform.m_q1 = quatf(mquat.w, mquat.x, mquat.y, mquat.z); 
                    xforms.push_back(xform);
                }
            }
        }
    }
}


//update the passive rigid bodies by interpolation of the previous and current frame
void dSolverNode::updatePassiveRigidBodies(MPlugArray &rbConnections, std::vector<xforms_t> &xforms, float t)
{
    size_t pb = 0;
    for(size_t i = 0; i < rbConnections.length(); ++i) {
        MObject node = rbConnections[i].node();
        MFnDagNode fnDagNode(node);
        if(fnDagNode.typeId() == rigidBodyNode::typeId) {
            rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
            rigid_body_t::pointer rb = rbNode->rigid_body();

            if(fnDagNode.parentCount() == 0) {
                std::cout << "No transform found!" << std::endl;
                continue;
            }

            MPlug plgMass(node, rigidBodyNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(!active) {
                //do linear interpolation for now
                rb->set_transform(xforms[pb].m_x0 + t * (xforms[pb].m_x1 - xforms[pb].m_x0),
                                  normalize(xforms[pb].m_q0 + t * (xforms[pb].m_q1 - xforms[pb].m_q0))); 
                ++pb;
            }
        } else if(fnDagNode.typeId() == rigidBodyArrayNode::typeId) {
            rigidBodyArrayNode *rbNode = static_cast<rigidBodyArrayNode*>(fnDagNode.userNode()); 
            std::vector<rigid_body_t::pointer>& rbs = rbNode->rigid_bodies();
        
            if(fnDagNode.parentCount() == 0) {
                std::cout << "No transform found!" << std::endl;
                return;
            }

            MPlug plgMass(node, rigidBodyArrayNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(!active) {
                for(size_t j = 0; j < rbs.size(); ++j) {
                    rbs[j]->set_transform(xforms[pb].m_x0 + t * (xforms[pb].m_x1 - xforms[pb].m_x0),
                                      normalize(xforms[pb].m_q0 + t * (xforms[pb].m_q1 - xforms[pb].m_q0))); 
                    ++pb;
                }
            }
        }
    }
}

//update the scene after a simulation step
void dSolverNode::updateActiveRigidBodies(MPlugArray &rbConnections)
{
   //update the active rigid bodies to the new configuration
    for(size_t i = 0; i < rbConnections.length(); ++i) {
        MObject node = rbConnections[i].node();
        MFnDagNode fnDagNode(node);
        if(fnDagNode.typeId() == rigidBodyNode::typeId) {
            rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
            rigid_body_t::pointer rb = rbNode->rigid_body();

            if(fnDagNode.parentCount() == 0) {
                std::cout << "No transform found!" << std::endl;
                continue;
            }

            MFnTransform fnTransform(fnDagNode.parent(0));

            MPlug plgMass(node, rigidBodyNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(active) {
                quatf rot;
                vec3f pos; 
                rb->get_transform(pos, rot);
                fnTransform.setRotation(MQuaternion(rot[1], rot[2], rot[3], rot[0]));
                fnTransform.setTranslation(MVector(pos[0], pos[1], pos[2]), MSpace::kTransform);
            }
        } else if(fnDagNode.typeId() == rigidBodyArrayNode::typeId) {
            rigidBodyArrayNode *rbNode = static_cast<rigidBodyArrayNode*>(fnDagNode.userNode()); 
            std::vector<rigid_body_t::pointer>& rbs = rbNode->rigid_bodies();

            MPlug plgMass(node, rigidBodyArrayNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            //write the position and rotations 
            if(active) {
                MPlug plgPosition(node, rigidBodyArrayNode::io_position);
                MPlug plgRotation(node, rigidBodyArrayNode::io_rotation);

                MPlug plgElement;
                for(size_t j = 0; j < rbs.size(); ++j) {
                    vec3f pos;
                    quatf rot;
                    rbs[j]->get_transform(pos, rot); 

                    MEulerRotation meuler(MQuaternion(rot[1], rot[2], rot[3], rot[0]).asEulerRotation());

                    plgElement = plgPosition.elementByLogicalIndex(j);
                    plgElement.child(0).setValue(pos[0]);
                    plgElement.child(1).setValue(pos[1]);
                    plgElement.child(2).setValue(pos[2]);

                    plgElement = plgRotation.elementByLogicalIndex(j);
                    plgElement.child(0).setValue(rad2deg(meuler.x));
                    plgElement.child(1).setValue(rad2deg(meuler.y));
                    plgElement.child(2).setValue(rad2deg(meuler.z));

                }
            }

            //check if we have to output the rigid bodies to a file
            MPlug plgFileIO(node, rigidBodyArrayNode::ia_fileIO);
            bool doIO;
            plgFileIO.getValue(doIO);
            if(doIO) {
                dumpRigidBodyArray(node);
            }
        }
    }
}

//apply fields in the scene from the rigid body
void dSolverNode::applyFields(MPlugArray &rbConnections, float dt)
{
    MVectorArray position;
    MVectorArray velocity;
    MDoubleArray mass;

    std::vector<rigid_body_t*> rigid_bodies;
    //gather active rigid bodies
    for(size_t i = 0; i < rbConnections.length(); ++i) {
        MObject node = rbConnections[i].node();
        MFnDagNode fnDagNode(node);
        if(fnDagNode.typeId() == rigidBodyNode::typeId) {
            rigidBodyNode *rbNode = static_cast<rigidBodyNode*>(fnDagNode.userNode()); 
            rigid_body_t::pointer rb = rbNode->rigid_body();

            MPlug plgMass(node, rigidBodyNode::ia_mass);
            float mass = 0.f;
            plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(active) {
                rigid_bodies.push_back(rb.get());
            }
        } else if(fnDagNode.typeId() == rigidBodyArrayNode::typeId) {
            rigidBodyArrayNode *rbNode = static_cast<rigidBodyArrayNode*>(fnDagNode.userNode()); 
            std::vector<rigid_body_t::pointer>& rbs = rbNode->rigid_bodies();

            MPlug plgMass(node, rigidBodyArrayNode::ia_mass);
            float mass = 0.f;
			plgMass.getValue(mass);
			bool active = (mass>0.f);
            if(active) {
                for(size_t j = 0; j < rbs.size(); ++j) {
                    rigid_bodies.push_back(rbs[j].get());
                }
            }
        }
    }

    //clear forces and get the properties needed for field computation    
    for(size_t i = 0; i < rigid_bodies.size(); ++i) {
        rigid_bodies[i]->clear_forces();
        vec3f pos, vel;
        quatf rot;
        rigid_bodies[i]->get_transform(pos, rot); 
        rigid_bodies[i]->get_linear_velocity(vel);
        position.append(MVector(pos[0], pos[1], pos[2]));
        velocity.append(MVector(vel[0], vel[1], vel[2]));
        //TODO
        mass.append(1.0);
        //
    }

    //apply the fields to the rigid bodies
    MVectorArray force;
    for(MItDag it(MItDag::kDepthFirst, MFn::kField); !it.isDone(); it.next()) {
        MFnField fnField(it.item());
        fnField.getForceAtPoint(position, velocity, mass, force, dt);
        for(size_t i = 0; i < rigid_bodies.size(); ++i) {
            rigid_bodies[i]->apply_central_force(vec3f(force[i].x, force[i].y, force[i].z));
        }
    }
}

void dSolverNode::computeRigidBodies(const MPlug& plug, MDataBlock& data)
{
   // std::cout << "dSolverNode::computeRigidBodies" << std::endl;

    bool enabled = data.inputValue(ia_enabled).asBool();
    if(!enabled) {
        data.outputValue(oa_rigidBodies).set(true);
        data.setClean(plug);
        return;
    }

    MTime time = data.inputValue(ia_time).asTime();
    MTime startTime = data.inputValue(ia_startTime).asTime();
    int subSteps = data.inputValue(ia_substeps).asInt();
    MObject thisObject = thisMObject();
    MPlug plgRigidBodies(thisObject, oa_rigidBodies); 
    MPlugArray rbConnections;
    plgRigidBodies.connectedTo(rbConnections, true, true);

    MPlug plgSplitImpulse(thisObject, ia_splitImpulse);
    bool splitImpulseEnabled;
    plgSplitImpulse.getValue(splitImpulseEnabled);

    if(time == startTime) {
        //first frame, init the simulation
        initRigidBodies(rbConnections);
        solver_t::set_split_impulse(splitImpulseEnabled);
        m_prevTime = time;
    } else {
        double delta_frames = (time - m_prevTime).value();
        bool playback = MConditionMessage::getConditionState("playingBack");
        if(time > m_prevTime && 
           ((delta_frames <= 1.0) || (playback && delta_frames < 100))) { 
             //step the simulation forward,
            //don't update if we are jumping more than one frame
    
            float dt = (time - m_prevTime).as(MTime::kSeconds);
    
            //gather start and end transform for passive rigid bodies, used for interpolation
            std::vector<xforms_t> passiveXForms;
            gatherPassiveTransforms(rbConnections, passiveXForms);
    
    
            //set the gravity in the solver
            MPlug plgGravity(thisObject, ia_gravity);
            vec3f gravity;
            plgGravity.child(0).getValue(gravity[0]);
            plgGravity.child(1).getValue(gravity[1]);
            plgGravity.child(2).getValue(gravity[2]);
    
            solver_t::set_gravity(gravity);
    
            solver_t::set_split_impulse(splitImpulseEnabled);
    
            for(int i = 1; i <= subSteps; ++i) {
                //first update the passive rigid bodies
                updatePassiveRigidBodies(rbConnections, passiveXForms, i * dt / subSteps);
    
                //fields
                applyFields(rbConnections, dt / subSteps);
    
                //step the simulation
                solver_t::step_simulation(dt / subSteps);
            }
    
            m_prevTime = time;

            updateActiveRigidBodies(rbConnections);
        }
    } 

    data.outputValue(oa_rigidBodies).set(true);
    data.setClean(plug);
}

void tokenize(const std::string& str,
              std::vector<std::string>& tokens,
              const std::string& delimiters = " ")
{
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

    while (std::string::npos != pos || std::string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

void replace(std::string &str, std::string const& what, std::string const& with_what)
{
    size_t pos;
    while((pos = str.find(what)) != std::string::npos) {
        str.replace(pos, pos + what.size(), with_what);
    }
}

bool dSolverNode::expandFileExpression(std::string const& expr, std::string &base_name, std::string &extension)
{
    std::vector<std::string> tokens;

    //check for extension
    tokenize(expr, tokens, ".");
    if(tokens.size() < 2) return false;
    if(tokens.back().size() != 3) return false;
    extension = tokens.back();

    std::copy(expr.begin(), expr.end() - 4, std::back_inserter(base_name));

    int time = (int) m_prevTime.value();

    std::stringstream ss;
    ss << time;
    std::string str_time(ss.str());
    //replace ${frame}
    replace(base_name, "${frame}", str_time);

    //replace ${waveFrame}
    while(str_time.size() < 4) str_time = "0" + str_time;
    replace(base_name, "${waveFrame}", str_time);

    return true;
}

void dSolverNode::dumpRigidBodyArray(MObject &node)
{ 
   // std::cout << "dSolverNode::dumpRigidBodyArray" << std::endl;

    MFnDagNode fnDagNode(node);
    rigidBodyArrayNode *rbaNode = static_cast<rigidBodyArrayNode*>(fnDagNode.userNode());

    MPlug plgFiles(node, rigidBodyArrayNode::ia_fioFiles);
  //  MPlug plgPositionAttribute(node, rigidBodyArrayNode::ia_fioPositionAttribute);
   // MPlug plgRotationAttribute(node, rigidBodyArrayNode::ia_fioRotationAttribute);

    MString mstr;
    plgFiles.getValue(mstr);
    std::string expr(mstr.asChar());
    std::string base_path, extension;
    if(!expandFileExpression(expr, base_path, extension)) {
        std::cout << "dSolverNode::dumpRigidBodyArray: syntax error in file expression: " << std::endl <<
            expr << std::endl;
        return;
    }
    if(extension != "pdb") {
        std::cout << "dSolverNode::dumpRigidBodyArray: only pdb files are supported" << std::endl;
        return;
    }
    std::string file_name = base_path + "." + extension;

    std::vector<rigid_body_t::pointer>& rbs = rbaNode->rigid_bodies();
    pdb_io_t pdb_io;
    pdb_io.m_num_particles = rbs.size();
    pdb_io.m_time = m_prevTime.value();
    pdb_io.m_attributes.resize(3);
    //position
    pdb_io.m_attributes[0].m_num_elements = 1;
    pdb_io.m_attributes[0].m_name = ATTR_POSITION;
    pdb_io.m_attributes[0].m_type = pdb_io_t::kVector;
    pdb_io.m_attributes[0].m_vector_data.resize(rbs.size());

    //rotation angle (in degrees)
    pdb_io.m_attributes[1].m_num_elements = 1;
    pdb_io.m_attributes[1].m_name = ATTR_IN_RANGLE;
    pdb_io.m_attributes[1].m_type = pdb_io_t::kReal;
    pdb_io.m_attributes[1].m_real_data.resize(rbs.size());

    //rotation vector
    pdb_io.m_attributes[2].m_num_elements = 1;
    pdb_io.m_attributes[2].m_name = ATTR_IN_RAXIS;
    pdb_io.m_attributes[2].m_type = pdb_io_t::kVector;
    pdb_io.m_attributes[2].m_vector_data.resize(rbs.size());

    for(size_t i = 0; i < rbs.size(); ++i) {
        vec3f pos;
        quatf rot;
        rbs[i]->get_transform(pos, rot);
        pdb_io.m_attributes[0].m_vector_data[i].x = pos[0];
        pdb_io.m_attributes[0].m_vector_data[i].y = pos[1];
        pdb_io.m_attributes[0].m_vector_data[i].z = pos[2];

        //
        vec3f axis;
        float angle;
        q_to_axis_angle(rot, axis, angle);
        pdb_io.m_attributes[1].m_real_data[i] = rad2deg(angle);

        pdb_io.m_attributes[2].m_vector_data[i].x = axis[0];
        pdb_io.m_attributes[2].m_vector_data[i].y = axis[1];
        pdb_io.m_attributes[2].m_vector_data[i].z = axis[2];
    }

    std::ofstream out(file_name.c_str());
    pdb_io.write(out);
}

