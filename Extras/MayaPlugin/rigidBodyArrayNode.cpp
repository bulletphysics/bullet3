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

//rigidBodyArrayNode.cpp

#include <maya/MFnDependencyNode.h>
#include <maya/MPlugArray.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnStringData.h>
#include <maya/MFnTransform.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>
#include <maya/MNodeMessage.h>
#include <maya/MVector.h>

#include "rigidBodyArrayNode.h"
#include "collisionShapeNode.h"
#include "mayaUtils.h"

#include "solver.h"

MTypeId     rigidBodyArrayNode::typeId(0x100330);
MString     rigidBodyArrayNode::typeName("dRigidBodyArray");

MObject     rigidBodyArrayNode::ia_collisionShape;
MObject     rigidBodyArrayNode::ia_solver;
MObject     rigidBodyArrayNode::ia_numBodies;
MObject     rigidBodyArrayNode::ia_active;
MObject     rigidBodyArrayNode::ia_mass;
MObject     rigidBodyArrayNode::ia_restitution;
MObject     rigidBodyArrayNode::ia_friction;
MObject     rigidBodyArrayNode::ia_linearDamping;
MObject     rigidBodyArrayNode::ia_angularDamping;
MObject     rigidBodyArrayNode::ia_initialPosition;
MObject     rigidBodyArrayNode::ia_initialRotation;
MObject     rigidBodyArrayNode::ia_initialVelocity;
MObject     rigidBodyArrayNode::ia_initialSpin;
MObject     rigidBodyArrayNode::ia_fileIO;
MObject     rigidBodyArrayNode::ia_fioFiles;
MObject     rigidBodyArrayNode::ia_fioPositionAttribute;
MObject     rigidBodyArrayNode::ia_fioRotationAttribute;
MObject     rigidBodyArrayNode::io_position;
MObject     rigidBodyArrayNode::io_rotation;
MObject     rigidBodyArrayNode::ca_rigidBodies;
MObject     rigidBodyArrayNode::ca_rigidBodyParam;

MStatus rigidBodyArrayNode::initialize()
{
    MStatus status;
    MFnMessageAttribute fnMsgAttr;
    MFnNumericAttribute fnNumericAttr;
    MFnMatrixAttribute  fnMatrixAttr;
    MFnTypedAttribute	fnTypedAttr;
    MFnStringData       fnStringData;

    ia_collisionShape = fnMsgAttr.create("inCollisionShape", "incs", &status);
    MCHECKSTATUS(status, "creating inCollisionShape attribute")
    status = addAttribute(ia_collisionShape);
    MCHECKSTATUS(status, "adding inCollisionShape attribute")

    ia_solver = fnMsgAttr.create("solver", "solv", &status);
    MCHECKSTATUS(status, "creating solver attribute")
    status = addAttribute(ia_solver);
    MCHECKSTATUS(status, "adding solver attribute")

    ia_numBodies = fnNumericAttr.create("numBodies", "nbds", MFnNumericData::kInt, 1, &status);
    MCHECKSTATUS(status, "creating numBodies attribute")
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ia_numBodies);
    MCHECKSTATUS(status, "adding numBodies attribute")

    ia_active = fnNumericAttr.create("active", "ac", MFnNumericData::kBoolean, 1, &status);
    MCHECKSTATUS(status, "creating active attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_active);
    MCHECKSTATUS(status, "adding active attribute")

    ia_mass = fnNumericAttr.create("mass", "ma", MFnNumericData::kDouble, 1.0, &status);
    MCHECKSTATUS(status, "creating mass attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_mass);
    MCHECKSTATUS(status, "adding mass attribute")

    ia_restitution = fnNumericAttr.create("restitution", "rst", MFnNumericData::kDouble, 0.1, &status);
    MCHECKSTATUS(status, "creating restitution attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_restitution);
    MCHECKSTATUS(status, "adding restitution attribute")

    ia_friction = fnNumericAttr.create("friction", "fc", MFnNumericData::kDouble, 0.5, &status);
    MCHECKSTATUS(status, "creating friction attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_friction);
    MCHECKSTATUS(status, "adding friction attribute")

    ia_linearDamping = fnNumericAttr.create("linearDamping", "ld", MFnNumericData::kDouble, 0.3, &status);
    MCHECKSTATUS(status, "creating linearDamping attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_linearDamping);
    MCHECKSTATUS(status, "adding linearDamping attribute")

    ia_angularDamping = fnNumericAttr.create("angularDamping", "ad", MFnNumericData::kDouble, 0.3, &status);
    MCHECKSTATUS(status, "creating angularDamping attribute")
    fnNumericAttr.setKeyable(true);
    status = addAttribute(ia_angularDamping);
    MCHECKSTATUS(status, "adding angularDamping attribute")

    ia_initialPosition = fnNumericAttr.createPoint("initialPosition", "inpo", &status);
    MCHECKSTATUS(status, "creating initialPosition attribute")
    fnNumericAttr.setArray(true);
    status = addAttribute(ia_initialPosition);
    MCHECKSTATUS(status, "adding initialPosition attribute")

    ia_initialRotation = fnNumericAttr.createPoint("initialRotation", "inro", &status);
    MCHECKSTATUS(status, "creating initialRotation attribute")
    fnNumericAttr.setArray(true);
    status = addAttribute(ia_initialRotation);
    MCHECKSTATUS(status, "adding initialRotation attribute")

    ia_initialVelocity = fnNumericAttr.createPoint("initialVelocity", "inve", &status);
    MCHECKSTATUS(status, "creating initialVelocity attribute")
    fnNumericAttr.setArray(true);
    status = addAttribute(ia_initialVelocity);
    MCHECKSTATUS(status, "adding initialVelocity attribute")

    ia_initialSpin = fnNumericAttr.createPoint("initialSpin", "insp", &status);
    MCHECKSTATUS(status, "creating initialSpin attribute")
    fnNumericAttr.setArray(true);
    status = addAttribute(ia_initialSpin);
    MCHECKSTATUS(status, "adding initialSpin attribute")

    ia_fileIO = fnNumericAttr.create("fileIO", "fio", MFnNumericData::kBoolean, 0, &status);
    MCHECKSTATUS(status, "creating fileIO attribute")
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ia_fileIO);
    MCHECKSTATUS(status, "adding fileIO attribute")

    ia_fioFiles = fnTypedAttr.create( "files", "fls", MFnData::kString, 
                                     fnStringData.create("/tmp/rigidBodyArray.${frame}.pdb"), &status );
    MCHECKSTATUS(status, "creating ia_fioFiles attribute")
    fnTypedAttr.setKeyable(false);
    status = addAttribute(ia_fioFiles);
    MCHECKSTATUS(status, "adding ia_fioFiles attribute")

    io_position = fnNumericAttr.createPoint("position", "pos", &status);
    MCHECKSTATUS(status, "creating position attribute")
    fnNumericAttr.setArray(true);
    status = addAttribute(io_position);
    MCHECKSTATUS(status, "adding io_position attribute")

    io_rotation = fnNumericAttr.createPoint("rotation", "rot", &status);
    MCHECKSTATUS(status, "creating rotation attribute")
    fnNumericAttr.setArray(true);
    status = addAttribute(io_rotation);
    MCHECKSTATUS(status, "adding io_rotation attribute")

    ca_rigidBodies = fnNumericAttr.create("ca_rigidBodies", "carb", MFnNumericData::kBoolean, 0, &status);
    MCHECKSTATUS(status, "creating ca_rigidBodies attribute")
    fnNumericAttr.setConnectable(false);
    fnNumericAttr.setHidden(true);
    fnNumericAttr.setStorable(false);
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ca_rigidBodies);
    MCHECKSTATUS(status, "adding ca_rigidBodies attribute")

    ca_rigidBodyParam = fnNumericAttr.create("ca_rigidBodyParam", "carbp", MFnNumericData::kBoolean, 0, &status);
    MCHECKSTATUS(status, "creating ca_rigidBodyParam attribute")
    fnNumericAttr.setConnectable(false);
    fnNumericAttr.setHidden(true);
    fnNumericAttr.setStorable(false);
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding ca_rigidBodyParam attribute")

    status = attributeAffects(ia_numBodies, ca_rigidBodies);
    MCHECKSTATUS(status, "adding attributeAffects(ia_numBodies, ca_rigidBodies)")

    status = attributeAffects(ia_numBodies, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_numBodies, ca_rigidBodyParam)")

    status = attributeAffects(ia_collisionShape, ca_rigidBodies);
    MCHECKSTATUS(status, "adding attributeAffects(ia_collisionShape, ca_rigidBodies)")

    status = attributeAffects(ia_collisionShape, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_collisionShape, ca_rigidBodyParam)")

    status = attributeAffects(ia_mass, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_mass, ca_rigidBodyParam)")

    status = attributeAffects(ia_restitution, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_restitution, ca_rigidBodyParam)")

    status = attributeAffects(ia_friction, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_friction, ca_rigidBodyParam)")

    status = attributeAffects(ia_linearDamping, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_linearDamping, ca_rigidBodyParam)")

    status = attributeAffects(ia_angularDamping, ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding attributeAffects(ia_angularDamping, ca_rigidBodyParam)")

    return MS::kSuccess;
}

rigidBodyArrayNode::rigidBodyArrayNode()
{
    // std::cout << "rigidBodyArrayNode::rigidBodyArrayNode" << std::endl;
}

rigidBodyArrayNode::~rigidBodyArrayNode()
{
    // std::cout << "rigidBodyArrayNode::~rigidBodyArrayNode" << std::endl;
    for(size_t i = 0; i < m_rigid_bodies.size(); ++i) {
        solver_t::remove_rigid_body(m_rigid_bodies[i]);
    }
}

void rigidBodyArrayNode::nodeRemoved(MObject& node, void *clientData)
{
   // std::cout << "rigidBodyArrayNode::nodeRemoved" << std::endl;
    MFnDependencyNode fnNode(node);
    rigidBodyArrayNode *thisNode = static_cast<rigidBodyArrayNode*>(fnNode.userNode());
    for(size_t i = 0; i < thisNode->m_rigid_bodies.size(); ++i) {
        solver_t::remove_rigid_body(thisNode->m_rigid_bodies[i]);
    }
}

void rigidBodyArrayNode::postConstructor()
{
   // MStatus status;
   // MObject thisObject = thisMObject();
  //  MCallbackId cid = MNodeMessage::addAttributeAddedOrRemovedCallback(thisObject, attributeAddedOrRemoved, NULL, &status);
}

void* rigidBodyArrayNode::creator()
{
    return new rigidBodyArrayNode();
}


bool rigidBodyArrayNode::setInternalValueInContext ( const  MPlug & plug,
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

MStatus rigidBodyArrayNode::compute(const MPlug& plug, MDataBlock& data)
{
    //std::cout << "rigidBodyArrayNode::compute: " << plug.name() << std::endl;
    //MTime time = data.inputValue( rigidBodyArrayNode::inTime ).asTime();
    if(plug == ca_rigidBodies) {
        computeRigidBodies(plug, data);
    } else if(plug == ca_rigidBodyParam) {
        computeRigidBodyParam(plug, data);
    } else if(plug.isElement()) {
        if(plug.array() == worldMatrix) {
            computeWorldMatrix(plug, data);
        } else {
            return MStatus::kUnknownParameter;
        }
    } else {
        return MStatus::kUnknownParameter;
    }
    return MStatus::kSuccess;
}

void rigidBodyArrayNode::draw( M3dView & view, const MDagPath &path,
                             M3dView::DisplayStyle style,
                             M3dView::DisplayStatus status )
{
  //  std::cout << "rigidBodyArrayNode::draw" << std::endl;

    update();

    view.beginGL();
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    glPushMatrix();
    //remove the parent transform here
    double m[4][4];
    m_worldMatrix.inverse().get(m);
    glMultMatrixd(&(m[0][0]));

    if(!m_rigid_bodies.empty()) {
        mat4x4f xform;
        if(style == M3dView::kFlatShaded ||
           style == M3dView::kGouraudShaded) {
            glEnable(GL_LIGHTING);
            float material[] = { 0.4f, 0.3f, 1.0f, 1.0f };
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, material);
            for(size_t i = 0; i < m_rigid_bodies.size(); ++i) {  
                glPushMatrix();
                m_rigid_bodies[i]->get_transform(xform);
                glMultMatrixf(xform.begin());
                m_rigid_bodies[i]->collision_shape()->gl_draw(collision_shape_t::kDSSolid);
                glPopMatrix();
            }
        } 
    
    
        if( status == M3dView::kActive ||
            status == M3dView::kLead ||
            status == M3dView::kHilite ||
            ( style != M3dView::kGouraudShaded && style != M3dView::kFlatShaded ) ) {
            
            glDisable(GL_LIGHTING);
            for(size_t i = 0; i < m_rigid_bodies.size(); ++i) {  
                glPushMatrix();
                m_rigid_bodies[i]->get_transform(xform);
                glMultMatrixf(xform.begin());
                m_rigid_bodies[i]->collision_shape()->gl_draw(collision_shape_t::kDSWireframe);
                glPopMatrix();
            }
        }
    }
    glPopMatrix();
    glPopAttrib();
    view.endGL();
}

bool rigidBodyArrayNode::isBounded() const
{
    //return true;
    return false;
}

MBoundingBox rigidBodyArrayNode::boundingBox() const
{
    // std::cout << "rigidBodyArrayNode::boundingBox()" << std::endl;
    //load the pdbs
    MObject node = thisMObject();

    MPoint corner1(-1, -1, -1);
    MPoint corner2(1, 1, 1);
    return MBoundingBox(corner1, corner2);
}

//standard attributes

void rigidBodyArrayNode::computeRigidBodies(const MPlug& plug, MDataBlock& data)
{
  //  std::cout << "rigidBodyArrayNode::computeRigidBodies" << std::endl;

    MObject thisObject(thisMObject());
    MPlug plgCollisionShape(thisObject, ia_collisionShape);
    MObject update;
    //force evaluation of the shape
    plgCollisionShape.getValue(update);
    size_t numBodies = data.inputValue(ia_numBodies).asInt();
   // MDataHandle coll = data.inputValue(ia_collisionShape);

   // collisionShapeNode * pCollisionShapeNode = NULL;
    collision_shape_t::pointer  collision_shape;
    if(plgCollisionShape.isConnected()) {
        MPlugArray connections;
        plgCollisionShape.connectedTo(connections, true, true);
        if(connections.length() != 0) {
            MFnDependencyNode fnNode(connections[0].node());
            if(fnNode.typeId() == collisionShapeNode::typeId) {
                collisionShapeNode *pCollisionShapeNode = static_cast<collisionShapeNode*>(fnNode.userNode());
                collision_shape = pCollisionShapeNode->collisionShape();    
            } else {
                std::cout << "rigidBodyArrayNode connected to a non-collision shape node!" << std::endl;
            }
        }
    }

    if(!collision_shape) {
        //not connected to a collision shape, put a default one
        collision_shape = solver_t::create_sphere_shape();
    }

    //save the positions and orientations to restore them
    std::vector<vec3f> positions(m_rigid_bodies.size());
    std::vector<quatf> rotations(m_rigid_bodies.size());
    for(size_t i = 0; i < m_rigid_bodies.size(); ++i) {
        m_rigid_bodies[i]->get_transform(positions[i], rotations[i]);
        solver_t::remove_rigid_body(m_rigid_bodies[i]);
    }

    m_rigid_bodies.resize(numBodies);
    for(size_t i = 0; i < m_rigid_bodies.size(); ++i) {
        m_rigid_bodies[i] = solver_t::create_rigid_body(collision_shape);
	if (i < positions.size()) {
            m_rigid_bodies[i]->set_transform(positions[i], rotations[i]);
	} else {
            m_rigid_bodies[i]->set_transform(vec3f(0.f,0.f,0.f),quatf(1.f,0.f,0.f,0.f));
	}
        solver_t::add_rigid_body(m_rigid_bodies[i]);
    }

    data.outputValue(ca_rigidBodies).set(true);
    data.setClean(plug);
}


void rigidBodyArrayNode::computeRigidBodyParam(const MPlug& plug, MDataBlock& data)
{
  //  std::cout << "rigidBodyArrayNode::computeRigidBodyParam" << std::endl;

    MObject thisObject(thisMObject());
    MObject update;
    
    MPlug(thisObject, ca_rigidBodies).getValue(update);

    float mass = data.inputValue(ia_mass).asFloat();
    vec3f inertia;
    if(!m_rigid_bodies.empty())  {
        inertia = mass * m_rigid_bodies[0]->collision_shape()->local_inertia();
    }
    float restitution = data.inputValue(ia_restitution).asFloat();
    float friction = data.inputValue(ia_friction).asFloat();
    float linearDamping = data.inputValue(ia_linearDamping).asFloat();
    float angularDamping = data.inputValue(ia_angularDamping).asFloat();

    for(size_t i = 0; i < m_rigid_bodies.size(); ++i) {
        m_rigid_bodies[i]->set_mass(mass);
        m_rigid_bodies[i]->set_inertia(inertia);
        m_rigid_bodies[i]->set_restitution(restitution);
        m_rigid_bodies[i]->set_friction(friction);
        m_rigid_bodies[i]->set_linear_damping(linearDamping);
        m_rigid_bodies[i]->set_angular_damping(angularDamping);    
    }

    data.outputValue(ca_rigidBodyParam).set(true);
    data.setClean(plug);
}

void rigidBodyArrayNode::computeWorldMatrix(const MPlug& plug, MDataBlock& data)
{
    //std::cout << "rigidBodyArrayNode::computeWorldMatrix" << std::endl;
    MObject thisObject(thisMObject());
    MFnDagNode fnDagNode(thisObject);

    MObject update;
    MPlug(thisObject, ca_rigidBodies).getValue(update);
    MPlug(thisObject, ca_rigidBodyParam).getValue(update);
    
    MFnTransform fnParentTransform(fnDagNode.parent(0));
    MTransformationMatrix dm(fnParentTransform.transformation().asMatrix() * m_worldMatrix.inverse());
    MVector mtranslation = dm.getTranslation(MSpace::kTransform);
    MQuaternion mrotation = dm.rotation();

  //  std::cout << mtranslation << std::endl;

    MArrayDataHandle hInitPosArray = data.outputArrayValue(ia_initialPosition);
    if(hInitPosArray.elementCount() < m_rigid_bodies.size()) {
        std::cout << "rigidBodyArrayNode::computeWorldMatrix: array size mismatch" << std::endl;
    }
    for(size_t i = 0; i < m_rigid_bodies.size(); ++i) {
        MDataHandle hInitPos = hInitPosArray.outputValue(); 
        float3 &ipos = hInitPos.asFloat3();

        vec3f pos, newpos;
        quatf rot, newrot;

        m_rigid_bodies[i]->get_transform(pos, rot);

        newpos = pos + vec3f(mtranslation.x, mtranslation.y, mtranslation.z);
//        newrot = qprod(rot, quatf(mrotation.w, mrotation.x, mrotation.y, mrotation.z));
        newrot = rot;

        m_rigid_bodies[i]->set_transform(newpos, newrot);

		float3 &ihpos=hInitPos.asFloat3();
        //hInitPos.set3Float(ipos[0] + mtranslation.x, ipos[1] + mtranslation.y, ipos[2] + mtranslation.z);
		ihpos[0] = ipos[0] + mtranslation.x;
		ihpos[1] = ipos[1] + mtranslation.y;
		ihpos[2] = ipos[2] + mtranslation.z;


        hInitPosArray.next();
    }

    m_worldMatrix = fnParentTransform.transformation().asMatrix();

    data.setClean(plug);
}

std::vector<rigid_body_t::pointer>& rigidBodyArrayNode::rigid_bodies()
{
  //  std::cout << "rigidBodyArrayNode::rigid_bodies" << std::endl;

    MObject thisObject(thisMObject());
    MObject update;
    MPlug(thisObject, ca_rigidBodies).getValue(update);
    MPlug(thisObject, ca_rigidBodyParam).getValue(update);

    return m_rigid_bodies;
} 


void rigidBodyArrayNode::update()
{
    MObject thisObject(thisMObject());

    MObject update;
    MPlug(thisObject, ca_rigidBodies).getValue(update);
    MPlug(thisObject, ca_rigidBodyParam).getValue(update);
    MPlug(thisObject, ia_solver).getValue(update);
    MPlug(thisObject, worldMatrix).elementByLogicalIndex(0).getValue(update);
}
