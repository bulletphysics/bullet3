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

//rigidBodyNode.cpp

//#include <GL/glut.h>

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
#include "collisionShapeNode.h"
#include "mayaUtils.h"

#include "solver.h"

MTypeId     rigidBodyNode::typeId(0x10032f);
MString     rigidBodyNode::typeName("dRigidBody");

MObject     rigidBodyNode::ia_collisionShape;
MObject     rigidBodyNode::ia_solver;
MObject     rigidBodyNode::ia_active;
MObject     rigidBodyNode::ia_mass;
MObject     rigidBodyNode::ia_restitution;
MObject     rigidBodyNode::ia_friction;
MObject     rigidBodyNode::ia_linearDamping;
MObject     rigidBodyNode::ia_angularDamping;
MObject     rigidBodyNode::ia_initialPosition;
MObject     rigidBodyNode::ia_initialRotation;
MObject     rigidBodyNode::ia_initialVelocity;
MObject     rigidBodyNode::ia_initialSpin;
MObject     rigidBodyNode::ca_rigidBody;
MObject     rigidBodyNode::ca_rigidBodyParam;


MStatus rigidBodyNode::initialize()
{
    MStatus status;
    MFnMessageAttribute fnMsgAttr;
    MFnNumericAttribute fnNumericAttr;
    MFnMatrixAttribute fnMatrixAttr;

    ia_collisionShape = fnMsgAttr.create("inCollisionShape", "incs", &status);
    MCHECKSTATUS(status, "creating inCollisionShape attribute")
    status = addAttribute(ia_collisionShape);
    MCHECKSTATUS(status, "adding inCollisionShape attribute")

    ia_solver = fnMsgAttr.create("solver", "solv", &status);
    MCHECKSTATUS(status, "creating solver attribute")
    status = addAttribute(ia_solver);
    MCHECKSTATUS(status, "adding solver attribute")

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
    status = addAttribute(ia_initialPosition);
    MCHECKSTATUS(status, "adding initialPosition attribute")

    ia_initialRotation = fnNumericAttr.createPoint("initialRotation", "inro", &status);
    MCHECKSTATUS(status, "creating initialRotation attribute")
    status = addAttribute(ia_initialRotation);
    MCHECKSTATUS(status, "adding initialRotation attribute")

    ia_initialVelocity = fnNumericAttr.createPoint("initialVelocity", "inve", &status);
    MCHECKSTATUS(status, "creating initialVelocity attribute")
    status = addAttribute(ia_initialVelocity);
    MCHECKSTATUS(status, "adding initialVelocity attribute")

    ia_initialSpin = fnNumericAttr.createPoint("initialSpin", "insp", &status);
    MCHECKSTATUS(status, "creating initialSpin attribute")
    status = addAttribute(ia_initialSpin);
    MCHECKSTATUS(status, "adding initialSpin attribute")

    ca_rigidBody = fnNumericAttr.create("ca_rigidBody", "carb", MFnNumericData::kBoolean, 0, &status);
    MCHECKSTATUS(status, "creating ca_rigidBody attribute")
    fnNumericAttr.setConnectable(false);
    fnNumericAttr.setHidden(true);
    fnNumericAttr.setStorable(false);
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ca_rigidBody);
    MCHECKSTATUS(status, "adding ca_rigidBody attribute")

    ca_rigidBodyParam = fnNumericAttr.create("ca_rigidBodyParam", "carbp", MFnNumericData::kBoolean, 0, &status);
    MCHECKSTATUS(status, "creating ca_rigidBodyParam attribute")
    fnNumericAttr.setConnectable(false);
    fnNumericAttr.setHidden(true);
    fnNumericAttr.setStorable(false);
    fnNumericAttr.setKeyable(false);
    status = addAttribute(ca_rigidBodyParam);
    MCHECKSTATUS(status, "adding ca_rigidBodyParam attribute")


    status = attributeAffects(ia_collisionShape, ca_rigidBody);
    MCHECKSTATUS(status, "adding attributeAffects(ia_collisionShape, ca_rigidBody)")

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

 //   status = attributeAffects(ia_active, ca_rigidBodyParam);
  //  MCHECKSTATUS(status, "adding attributeAffects(ia_active, ca_rigidBodyParam)")

    return MS::kSuccess;
}

rigidBodyNode::rigidBodyNode()
{
    // std::cout << "rigidBodyNode::rigidBodyNode" << std::endl;
}

rigidBodyNode::~rigidBodyNode()
{
    // std::cout << "rigidBodyNode::~rigidBodyNode" << std::endl;
}

void rigidBodyNode::nodeRemoved(MObject& node, void *clientData)
{
   // std::cout << "rigidBodyNode::nodeRemoved" << std::endl;
    MFnDependencyNode fnNode(node);
    solver_t::remove_rigid_body(static_cast<rigidBodyNode*>(fnNode.userNode())->m_rigid_body);
}

void* rigidBodyNode::creator()
{
    return new rigidBodyNode();
}


bool rigidBodyNode::setInternalValueInContext ( const  MPlug & plug,
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

/*
MStatus rigidBodyNode::setDependentsDirty( const  MPlug & plug,  MPlugArray & plugArray)
{
    std::cout << "rigidBodyNode::setDependentsDirty: " << plug.name().asChar() << std::endl;
    MObject thisNode = thisMObject();
    if(plug == ia_solver) {
        MPlug plgAffected(thisNode, ca_update);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);
    } else if(plug == ia_collisionShape) {
        //ia_collisionShape -> ca_rigidBody
        MPlug plgAffected(thisNode, ca_rigidBody);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);

        //ia_collisionShape -> ca_rigidBodyParam
        plgAffected.setAttribute(ca_rigidBodyParam);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);
    } else if(plug == ca_rigidBody) {
        //ca_rigidBody -> ca_update
        MPlug plgAffected(thisNode, ca_update);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);

        //ca_rigidBody -> ca_rigidBodyParam
        plgAffected.setAttribute(ca_rigidBodyParam);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);
    } else if(plug == ia_mass) {
        //ia_mass -> ca_rigidBodyParam
        MPlug plgAffected(thisNode, ca_rigidBodyParam);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);

        plgAffected.setAttribute(ca_update);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);
    } else if(plug == ca_rigidBodyParam) {
        //ca_rigidBodyParam -> ca_update
        MPlug plgAffected(thisNode, ca_update);
        plugArray.append(plgAffected);
        plgAffected.setValue(true);
    }
    return MS::kSuccess;
}*/


MStatus rigidBodyNode::compute(const MPlug& plug, MDataBlock& data)
{
    //std::cout << "rigidBodyNode::compute: " << plug.name() << std::endl;
    //MTime time = data.inputValue( rigidBodyNode::inTime ).asTime();
    if(plug == ca_rigidBody) {
        computeRigidBody(plug, data);
    } else if(plug == ca_rigidBodyParam) {
        computeRigidBodyParam(plug, data);
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

void rigidBodyNode::draw( M3dView & view, const MDagPath &path,
                             M3dView::DisplayStyle style,
                             M3dView::DisplayStatus status )
{
  //  std::cout << "rigidBodyNode::draw" << std::endl;

    update();

    view.beginGL();
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    if(m_rigid_body) {
        //remove the scale, since it's already include in the node transform 
        vec3f scale;
        m_rigid_body->collision_shape()->get_scale(scale);

        glPushMatrix();
        glScalef(1/scale[0], 1/scale[1], 1/scale[2]); 
    
        if(style == M3dView::kFlatShaded ||
           style == M3dView::kGouraudShaded) {
            glEnable(GL_LIGHTING);
            float material[] = { 0.4, 0.7, 1.0, 1.0  };
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, material);
            m_rigid_body->collision_shape()->gl_draw(collision_shape_t::kDSSolid);
        } 
    
    
        if( status == M3dView::kActive ||
            status == M3dView::kLead ||
            status == M3dView::kHilite ||
            ( style != M3dView::kGouraudShaded && style != M3dView::kFlatShaded ) ) {
            
            glDisable(GL_LIGHTING);
            m_rigid_body->collision_shape()->gl_draw(collision_shape_t::kDSWireframe);
    
        }
        glPopMatrix();
    }
    glPopAttrib();
    view.endGL();
}

bool rigidBodyNode::isBounded() const
{
    //return true;
    return false;
}

MBoundingBox rigidBodyNode::boundingBox() const
{
    // std::cout << "rigidBodyNode::boundingBox()" << std::endl;
    //load the pdbs
    MObject node = thisMObject();

    MPoint corner1(-1, -1, -1);
    MPoint corner2(1, 1, 1);
    return MBoundingBox(corner1, corner2);
}

//standard attributes
void rigidBodyNode::computeRigidBody(const MPlug& plug, MDataBlock& data)
{
   // std::cout << "rigidBodyNode::computeRigidBody" << std::endl;

    MObject thisObject(thisMObject());
    MPlug plgCollisionShape(thisObject, ia_collisionShape);
    MObject update;
    //force evaluation of the shape
    plgCollisionShape.getValue(update);

    vec3f prevCenter(0, 0, 0);
    quatf prevRotation(qidentity<float>());
    if(m_rigid_body) {
        prevCenter = m_rigid_body->collision_shape()->center();
        prevRotation = m_rigid_body->collision_shape()->rotation();
    }

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
                std::cout << "rigidBodyNode connected to a non-collision shape node!" << std::endl;
            }
        }
    }

    if(!collision_shape) {
        //not connected to a collision shape, put a default one
        collision_shape = solver_t::create_sphere_shape();
    }

   /* vec3f deltaCenter = prevCenter - collision_shape->center();
    quatf deltaRotation = qprod(qconj(collision_shape->rotation()), prevRotation); 

    MDataHandle hInitPos = data.outputValue(ia_initialPosition);
    float3 &ipos = hInitPos.asFloat3();

    MDataHandle hInitRot = data.outputValue(ia_initialRotation);
    float3 &irot = hInitRot.asFloat3();
    MQuaternion iquat = MEulerRotation(deg2rad(irot[0]), deg2rad(irot[1]), deg2rad(irot[2])).asQuaternion();

   // MVector deltapos(mtranslation.x - pos[0], mtranslation.y - pos[1], mtranslation.z - pos[2]);
   // MQuaternion deltarot = MQuaternion(rot[1], rot[2], rot[3], rot[0]).conjugate() * mrotation; 

    MVector newipos(ipos[0] + deltaCenter[0], ipos[1] + deltaCenter[1], ipos[2] + deltaCenter[2]); */
   // MEulerRotation newirot((iquat * deltarot).asEulerRotation());

    //hInitPos.set3Float(newipos.x, newipos.y, newipos.z);
   // hInitRot.set3Float(rad2deg(newirot.x), rad2deg(newirot.y), rad2deg(newirot.z));*/

    solver_t::remove_rigid_body(m_rigid_body);
    m_rigid_body = solver_t::create_rigid_body(collision_shape);
    solver_t::add_rigid_body(m_rigid_body);

    data.outputValue(ca_rigidBody).set(true);
    data.setClean(plug);
}


void rigidBodyNode::computeWorldMatrix(const MPlug& plug, MDataBlock& data)
{
   // std::cout << "rigidBodyNode::computeWorldMatrix" << std::endl;

    MObject thisObject(thisMObject());
    MFnDagNode fnDagNode(thisObject);

    MObject update;
    MPlug(thisObject, ca_rigidBody).getValue(update);
    MPlug(thisObject, ca_rigidBodyParam).getValue(update);
    
    vec3f pos;
    quatf rot;

    MFnTransform fnParentTransform(fnDagNode.parent(0));
    MVector mtranslation = fnParentTransform.getTranslation(MSpace::kTransform);
    MQuaternion mrotation;
    fnParentTransform.getRotation(mrotation, MSpace::kTransform);
    double mscale[3];
    fnParentTransform.getScale(mscale);

  // std::cout << mtranslation << std::endl;

    m_rigid_body->get_transform(pos, rot);

    MDataHandle hInitPos = data.outputValue(ia_initialPosition);
    float3 &ipos = hInitPos.asFloat3();

    MDataHandle hInitRot = data.outputValue(ia_initialRotation);
    float3 &irot = hInitRot.asFloat3();
    MQuaternion iquat = MEulerRotation(deg2rad(irot[0]), deg2rad(irot[1]), deg2rad(irot[2])).asQuaternion();

    MVector deltapos(mtranslation.x - pos[0], mtranslation.y - pos[1], mtranslation.z - pos[2]);
    MQuaternion deltarot = MQuaternion(rot[1], rot[2], rot[3], rot[0]).conjugate() * mrotation; 

    MVector newipos(ipos[0] + deltapos.x, ipos[1] + deltapos.y, ipos[2] + deltapos.z); 
    MEulerRotation newirot((iquat * deltarot).asEulerRotation());

    hInitPos.set3Float(newipos.x, newipos.y, newipos.z);
    hInitRot.set3Float(rad2deg(newirot.x), rad2deg(newirot.y), rad2deg(newirot.z));

    m_rigid_body->set_transform(vec3f(mtranslation.x, mtranslation.y, mtranslation.z),
                                quatf(mrotation.w, mrotation.x, mrotation.y, mrotation.z));

    data.setClean(plug);

    //set the scale to the collision shape
    m_rigid_body->collision_shape()->set_scale(vec3f(mscale[0], mscale[1], mscale[2]));
/*
    MPlug plgCollisionShape(thisObject, ia_collisionShape);
    //force evaluation of the shape
    plgCollisionShape.getValue(update);
    
    if(plgCollisionShape.isConnected()) {
        MPlugArray connections;
        plgCollisionShape.connectedTo(connections, true, true);
        if(connections.length() != 0) {
            MFnDependencyNode fnNode(connections[0].node());
            if(fnNode.typeId() == collisionShapeNode::typeId) {
                MPlug plgScale(fnNode.object(), collisionShapeNode::ia_scale);
                plgScale.child(0).setValue(mscale[0]);
                plgScale.child(1).setValue(mscale[1]);
                plgScale.child(2).setValue(mscale[2]);
            } else {
                std::cout << "rigidBodyNode connected to a non-collision shape node!" << std::endl;
            }
        }
    }*/

}

void rigidBodyNode::computeRigidBodyParam(const MPlug& plug, MDataBlock& data)
{
   // std::cout << "rigidBodyNode::computeRigidBodyParam" << std::endl;

    MObject thisObject(thisMObject());
    MObject update;
    
    MPlug(thisObject, ca_rigidBody).getValue(update);
    double mass = data.inputValue(ia_mass).asDouble();
    m_rigid_body->set_mass(mass);
    m_rigid_body->set_inertia(mass * m_rigid_body->collision_shape()->local_inertia());
    m_rigid_body->set_restitution(data.inputValue(ia_restitution).asDouble());
    m_rigid_body->set_friction(data.inputValue(ia_friction).asDouble());
    m_rigid_body->set_linear_damping(data.inputValue(ia_linearDamping).asDouble());
    m_rigid_body->set_angular_damping(data.inputValue(ia_angularDamping).asDouble());
   // m_rigid_body->set_kinematic(!data.inputValue(ia_active).asBool());

    data.outputValue(ca_rigidBodyParam).set(true);
    data.setClean(plug);
}

rigid_body_t::pointer rigidBodyNode::rigid_body()
{
 //   std::cout << "rigidBodyNode::rigid_body" << std::endl;

    MObject thisObject(thisMObject());
    MObject update;
    MPlug(thisObject, ca_rigidBody).getValue(update);
    MPlug(thisObject, ca_rigidBodyParam).getValue(update);

    return m_rigid_body;
} 

void rigidBodyNode::update()
{
    MObject thisObject(thisMObject());

    MObject update;
    MPlug(thisObject, ca_rigidBody).getValue(update);
    MPlug(thisObject, ca_rigidBodyParam).getValue(update);
    MPlug(thisObject, ia_solver).getValue(update);
    MPlug(thisObject, worldMatrix).elementByLogicalIndex(0).getValue(update);
//    MPlug plg = MPlug(thisObject, worldMatrix).elementByLogicalIndex(0);
}
