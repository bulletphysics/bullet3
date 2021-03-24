#include <math.h>
#include <limits>

#include "Dof6Spring2Setup.h"

#include "../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3FileUtils.h"
#include "../Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "../Utils/b3BulletDefaultFileIO.h"

#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4 0.785398163397448309616
#endif

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../CommonInterfaces/CommonMultiBodyBase.h"

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
////  helper function
////
/////////////////////////////////////////////////////////////
btScalar inline rad2degree(btScalar rad){
    return rad / SIMD_PI * 180.0f;
}

btScalar inline degree2rad(btScalar degree){
    return degree / 180.0f * SIMD_PI;
}

void getEulerValFromQuaternion(btVector3& eulerVal, const btScalar* q)
{
    btQuaternion qt(q[0], q[1], q[2], q[3]);
    qt.getEulerZYX(eulerVal[2], eulerVal[1], eulerVal[0]);
}

btScalar cosOffset(btScalar amp, btScalar T, btScalar phase, btScalar time){
    return amp * cos(time / T * SIMD_2_PI + phase);
}

btScalar sinOffset(btScalar amp, btScalar T, btScalar phase, btScalar time){
    return amp * sin(time / T * SIMD_2_PI + phase);
}

// TODO use it in code
btScalar adjustKs(btScalar ks, btScalar mass, btScalar deltaTime)
{
    btScalar r = ks;
    btScalar angular_freq = btSqrt(r / mass);
    if (0.25 < angular_freq * deltaTime)
    {
        r = BT_ONE / deltaTime / deltaTime / btScalar(16.0) * mass;
    }
    return r;
}

// TODO adjust kd


/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
////  multibody skeleton
////
/////////////////////////////////////////////////////////////
#define WIRE_FRAME 0

char mfileName[1024];

const btVector3 zeroTransOffset(0.0, 0.0, 0.0);

enum CollisionTypes
{
    NOTHING  = 0,        //< Collide with nothing
    BONE_BODY = 1 << 1,
    TARGET_BODY = 1 << 2
};

static btScalar radius(0.05);

#define MAX_DRAG_FORCE 0.5f
#define MAX_SPRING_FORCE 0.5f

class gravityGenerator
{
private:
    int m_status;
    int m_step;
    float m_curr_g;

public:
    float m_gravity;
    float m_tup;
    float m_tdown;

    gravityGenerator(float gravity = -0.1, int tup = 10, int tdown = 10) : m_gravity(gravity),
        m_tup(tup), m_tdown(tdown)
    {
        m_step = 0;
        m_status = 0;
        m_curr_g = 0;
    }

    float getGVal()
    {
        float g = 0.0f;
        if ( m_status  == 1 ){
            g = m_gravity * m_step / m_tup;
            if ( m_step < m_tup )
                m_step += 1;
        } else if ( m_status == -1){
            float g = m_curr_g * ( 1.0 - m_step / m_tdown );
            if ( m_step < m_tdown )
                m_step += 1;
            else{
                m_status = 0;
            }
        }

        if ( m_status != -1 )
            m_curr_g = g;

        return g;
    }

    void startGravity()
    {
        if ( m_status == 0 || m_status == -1) {
            m_step = 0;
        }
        m_status = 1;
    }

    void stopGravity()
    {
        if ( m_status == 0 || m_status == -1 )
            return;
        m_status = -1;
        m_step = 0;
    }
};

struct Skeleton : public CommonMultiBodyBase
{
    btMultiBody* m_multiBody;
    btRigidBody* m_collider;

    int m_solverType;
    btScalar m_time;
    int m_step;
    int m_numLinks;
    btVector3 m_prevBaseVel;
    btTransform m_prevBaseTrans;
    btAlignedObjectArray<btQuaternion> m_balanceRot;
    btAlignedObjectArray<btScalar> m_Ks;
    gravityGenerator m_g;
    btScalar m_linearDragEffect;
    btScalar m_centrifugalDragEffect;

public:
    Skeleton(struct GUIHelperInterface* helper);
    virtual ~Skeleton();
    virtual void initPhysics();
    virtual void stepSimulation(float deltaTime);
    virtual void resetCamera()
    {
        float dist = 5;
        float pitch = -21;
        float yaw = 270;
        float targetPos[3] = {0, 0, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
    }
    void addColliders(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents);
    btTransform transformBase(btScalar time);
    void moveCollider(btScalar time);
    void applySpringForce(float deltaTime);
    void applyBaseLinearDragForce(float deltaTime, int m_step, const btTransform& trans);
    void applyBaseCentrifugalForce(float deltaTime, int m_step, const btTransform& trans);
    static void OnInternalTickCallback(btDynamicsWorld* world, btScalar timeStep);
};

Skeleton::Skeleton(struct GUIHelperInterface* helper)
        : CommonMultiBodyBase(helper)
{
    m_time = btScalar(0.0);
    m_step = 0;
    m_numLinks = 8;
    m_solverType = 0;
    m_prevBaseVel = btVector3(0.0, 0.0, 0.0);
    m_prevBaseTrans.setIdentity();
    m_g.m_gravity = -0.08;
    m_linearDragEffect = btScalar(25.0);
    m_centrifugalDragEffect = btScalar(0.3);
}

Skeleton::~Skeleton()
{
}

void Skeleton::initPhysics()
{
    int upAxis = 1;

    m_guiHelper->setUpAxis(upAxis);

    // set btDefaultCollisionConfiguration, dispatcher, solver and other things
    this->createEmptyDynamicsWorld(m_solverType);

    bool isPreTick = false;
    m_dynamicsWorld->setInternalTickCallback(OnInternalTickCallback, this, isPreTick);

    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    if (m_dynamicsWorld->getDebugDrawer())
    {
        int mode = btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints + btIDebugDraw::DBG_DrawAabb;
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(mode);  //+btIDebugDraw::DBG_DrawConstraintLimits);
    }

    const bool floating = false;
    const bool damping = true;   // disable bullet internal damp
    const bool gyro = false;
    const bool canSleep = false;
    const bool selfCollide = false;

    /////////////////////////////////////////////////////////////////
    // construct the skeleton
    /////////////////////////////////////////////////////////////////
    btVector3 linkHalfExtents(0.05, 0.5, 0.1);
    btVector3 baseHalfExtents(0.0, 0.0, 0.0);

    btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
    float baseMass = 0.f;

    btMultiBody* pMultiBody = new btMultiBody(m_numLinks, baseMass, baseInertiaDiag, !floating, canSleep);
    //pMultiBody->useRK4Integration(true);

    // set base position
    btQuaternion baseQ(0.f, 0.f, 0.f, 1.f);
    pMultiBody->setWorldToBaseRot(baseQ);
    btVector3 basePos = btVector3(0.0, 0.0, 0.0);
    pMultiBody->setBasePos(basePos);

    m_prevBaseTrans.setOrigin(basePos);
    m_prevBaseTrans.setRotation(baseQ);
    m_multiBody = pMultiBody;

    //y-axis assumed up
    btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1], 0);
    for (int i = 0; i < m_numLinks; ++i)
    {
        float linkMass = 1.f;
        btVector3 linkInertiaDiag(0.f, 0.f, 0.f);
        btCollisionShape* shape = 0;
        {
            shape = new btBoxShape(linkHalfExtents);
        }
        shape->calculateLocalInertia(linkMass, linkInertiaDiag);
        delete shape;

        btVector3 parentComToCurrentCom;
        if (i == 0){
            parentComToCurrentCom = btVector3(0, -linkHalfExtents[1] * 1, 0);
        }
        else{
            parentComToCurrentCom = btVector3(0, -linkHalfExtents[1] * 2, 0);
        }
        pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1,
                btQuaternion(0.f, 0.f, 0.f, 1.f),
                                  (parentComToCurrentCom - currentPivotToCurrentCom),
                                  currentPivotToCurrentCom, !selfCollide);
    }

    pMultiBody->finalizeMultiDof();
    btMultiBodyDynamicsWorld* world = m_dynamicsWorld;
    world->addMultiBody(pMultiBody);

    pMultiBody->setCanSleep(canSleep);
    pMultiBody->setHasSelfCollision(selfCollide);
    pMultiBody->setUseGyroTerm(gyro);
    if (damping)
    {
        // TODO set linear and angular damp for each joint
        pMultiBody->setLinearDamping(0.05f);
        pMultiBody->setAngularDamping(0.7f);
    }

    // set init pose
    if (m_numLinks > 0)
    {
        btScalar angle = -23.57817848 * SIMD_PI / 180.f * 2;
        btQuaternion quat0(btVector3(1, 0, 0).normalized(), angle);
        quat0.normalize();
        pMultiBody->setJointPosMultiDof(0, quat0);
    }

    // init default params
    m_Ks.resize(pMultiBody->getNumLinks());
    m_balanceRot.resize(pMultiBody->getNumLinks());
    for (int i = 0; i < m_numLinks; i++)
    {
        m_balanceRot[i] = btQuaternion(0.0, 0.0, 0.0, 1.0);
        m_Ks[i] = 0.3;
    }

    // adjust balance rot, ks and damp
    btScalar angle = -23.57817848 * SIMD_PI / 180.f;
    m_balanceRot[0] = btQuaternion(btVector3(1, 0, 0).normalized(), angle);

    addColliders(pMultiBody, world, baseHalfExtents, linkHalfExtents);

    // load triangles from obj file
    {
        const char* fileName = "teddy.obj";  //sphere8.obj";//sponza_closed.obj";//sphere8.obj";
        char relativeFileName[1024];
        if (b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024,0))
        {
            char pathPrefix[1024];
            b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
        }

        b3BulletDefaultFileIO fileIO;
        GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(relativeFileName, "",&fileIO);
        if (!glmesh){
            printf("fail to load file &s\n", fileName);
            return;
        }
        else {
            printf("[INFO] Obj loaded: Extracted %d verticed from obj file [%s]\n", glmesh->m_numvertices, fileName);
        }

        btAlignedObjectArray<btVector3> convertedVerts;
        convertedVerts.reserve(glmesh->m_numvertices);
        for (int i=0; i<glmesh->m_numvertices; i++)
        {
            convertedVerts.push_back(btVector3(
                    glmesh->m_vertices->at(i).xyzw[0],
                    glmesh->m_vertices->at(i).xyzw[1],
                    glmesh->m_vertices->at(i).xyzw[2]));
        }
        btTriangleMesh* meshInterface = new btTriangleMesh();
        for (int i=0; i<glmesh->m_numIndices/3; i++)
        {
            const btVector3& v0 = convertedVerts[glmesh->m_indices->at(i*3)];
            const btVector3& v1 = convertedVerts[glmesh->m_indices->at(i*3+1)];
            const btVector3& v2 = convertedVerts[glmesh->m_indices->at(i*3+2)];
            meshInterface->addTriangle(v0,v1,v2);
        }
        btCollisionShape* shape = new btBvhTriangleMeshShape(meshInterface, true, true);

        float scaling[4] = {0.03, 0.03, 0.03, 1};
        btVector3 localScaling(scaling[0], scaling[1], scaling[2]);
        shape->setLocalScaling(localScaling);

        /// Create Dynamic Objects
        btScalar mass(0.f);
        bool isDynamic = (mass != 0.f);
        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            shape->calculateLocalInertia(mass, localInertia);

        btVector3 startPos(0.0, -1.5, -2.0);
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(startPos);
        m_collider = createRigidBody(mass, startTransform, shape);

        int shapeId = m_guiHelper->registerGraphicsShape(&glmesh->m_vertices->at(0).xyzw[0],
                                                         glmesh->m_numvertices,
                                                         &glmesh->m_indices->at(0),
                                                         glmesh->m_numIndices,
                                                         B3_GL_TRIANGLES, -1);
        shape->setUserIndex(shapeId);
        float color[4] = {1, 0, 0, 1};
        float orn[4] = {0, 0, 0, 1};
        float pos[4] = {float(startPos[0]), float(startPos[1]), float(startPos[2]), 0.0};
        int renderInstance = m_guiHelper->registerGraphicsInstance(shapeId, pos, orn, color, scaling);
        m_collider->setUserIndex(renderInstance);
        m_collider->getBroadphaseHandle()->m_collisionFilterGroup = TARGET_BODY;
        m_collider->getBroadphaseHandle()->m_collisionFilterMask = BONE_BODY;
    }
}

void Skeleton::addColliders(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents)
{
    btAlignedObjectArray<btQuaternion> world_to_local;
    world_to_local.resize(pMultiBody->getNumLinks() + 1);

    btAlignedObjectArray<btVector3> local_origin;
    local_origin.resize(pMultiBody->getNumLinks() + 1);
    world_to_local[0] = pMultiBody->getWorldToBaseRot();
    local_origin[0] = pMultiBody->getBasePos();

    for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
    {
        const int parent = pMultiBody->getParent(i);
        world_to_local[i + 1] = pMultiBody->getParentToLocalRot(i) * world_to_local[parent + 1];
        local_origin[i + 1] = local_origin[parent + 1] + (quatRotate(world_to_local[i + 1].inverse(), pMultiBody->getRVector(i)));
    }

    for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
    {
        btVector3 posr = local_origin[i + 1];
        btScalar quat[4] = {-world_to_local[i + 1].x(), -world_to_local[i + 1].y(), -world_to_local[i + 1].z(), world_to_local[i + 1].w()};

        btCollisionShape* shape = new btBoxShape(linkHalfExtents);
        btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);
        col->setCollisionShape(shape);

        btTransform tr;
        tr.setIdentity();
        tr.setOrigin(posr);
        tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
        col->setWorldTransform(tr);

        col->setRestitution(0.0);

        // set collision group and mask, only collide with objects with mask is true where it collide with collider
        pWorld->addCollisionObject(col, BONE_BODY, TARGET_BODY);  //,2,1+2);

        if (!WIRE_FRAME)
        {
            m_guiHelper->createCollisionShapeGraphicsObject(shape);
            btVector4 color(0, 1, 0, 1);
            m_guiHelper->createCollisionObjectGraphicsObject(col, color);
        }

        pMultiBody->getLink(i).m_collider = col;
    }
}

btTransform Skeleton::transformBase(btScalar time){
    btTransform trans;
    trans.setIdentity();

    // btMultiBodyFixedConstraint does not work as we like, cause it is not totally fixed to the body.
    // move the base of the chain does not apply torque on the multibody chains
    btScalar amp = 0.5f;
    btScalar T = 50.0f;
    btScalar phase =  SIMD_PI / 2.0;
    btVector3 basePos = btVector3(0.0, cosOffset(amp, T, phase, time), 0.0);
    trans.setOrigin(basePos);

    btScalar cycle = 50.0f;
    btScalar omega = SIMD_2_PI / cycle;
    btQuaternion q;
    q.setRotation(btVector3(0, 1, 0), time * omega);
    trans.setRotation(q);

    m_multiBody->setBaseWorldTransform(trans);
    return trans;
}

void Skeleton::moveCollider(btScalar time){
    btTransform tr;
    tr.setIdentity();
    btScalar amp = 1.8f;
    btScalar T = 100.0f;
    btScalar phase = SIMD_PI;
    tr.setOrigin(btVector3(0.0, -1.5, cosOffset(amp, T, phase, time)));
    tr.setRotation(btQuaternion(0.0, 0.0, 0.0, 1.0));
    m_collider->setWorldTransform(tr);
}

void Skeleton::applySpringForce(float deltaTime){
    for (int i = 0; i < m_multiBody->getNumLinks(); ++i) {
        if (m_Ks[i] > 0.0)
        {
            btScalar* q = m_multiBody->getJointPosMultiDof(i);
            btQuaternion curr_q = btQuaternion(q[0], q[1], q[2], q[3]);
            btScalar _ks = m_Ks[i];
            // TODO ?
//            _ks = adjustKs(m_Ks[i], m_multiBody->getLink(i).m_mass, deltaTime);
            btVector3 spring(m_balanceRot[i][0] - curr_q[0],m_balanceRot[i][1] - curr_q[1], m_balanceRot[i][2] - curr_q[2]);
            spring *= _ks;
            if (spring.length() > MAX_SPRING_FORCE) {
                printf("spring force %f is too large\n", spring.length());
                spring = spring.normalized() * MAX_SPRING_FORCE;
            }
            // apply the torque
            for (int d = 0; d < m_multiBody->getLink(i).m_dofCount; d++) {
                m_multiBody->addJointTorqueMultiDof(i, d, spring[d]);
            }
        }
    }
}

void Skeleton::applyBaseLinearDragForce(float deltaTime, int m_step, const btTransform& trans)
{
    btVector3 currBaseVel;

    if ( m_step == 1 )
        m_prevBaseVel = (trans.getOrigin() - m_prevBaseTrans.getOrigin()) / deltaTime;

    if ( m_step > 1 ) {
        currBaseVel = (trans.getOrigin() - m_prevBaseTrans.getOrigin()) / deltaTime;
        btVector3 currBaseAcc = (currBaseVel - m_prevBaseVel) / deltaTime;
        for (int i = 0; i < m_numLinks; ++i) {
            btVector3 orient = m_multiBody->getLink(i).m_dVector;
            orient = m_multiBody->localDirToWorld(i, orient);
            btVector3 torque = btCross(orient, -currBaseAcc * m_multiBody->getLink(i).m_mass / (i + 1));
            // TODO at the beginning, the force is too big. but at the next Cycle, it is small. NEED to check and fix.
            torque *= m_linearDragEffect;
            if (torque.length() > MAX_DRAG_FORCE) {
                printf("drag force %f is too large\n", torque.length());
                torque = torque.normalized() * MAX_DRAG_FORCE;
            }
            // TODO addlink is different from addJointTorqueMultiDof, and the later looks better in animation.
//        m_multiBody->addLinkTorque(i, torque);
            for (int d = 0; d < m_multiBody->getLink(i).m_dofCount; d++) {
                m_multiBody->addJointTorqueMultiDof(i, d, torque[d]);
            }
        }
        m_prevBaseVel = currBaseVel;
    }
}

void Skeleton::applyBaseCentrifugalForce(float deltaTime, int m_step, const btTransform& trans)
{
    if ( m_step > 0 ) {
        btVector3 basePos = trans.getOrigin();
        /*
         * NOTE : do not use btTransform, cause btTransform will cause rotation axis flip at some degree when
         * using trans.getRotation()
         * ref : https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=1942
         */
        btMatrix3x3 deltaTrans = trans.getBasis() * m_prevBaseTrans.getBasis().transpose();
        btQuaternion q;
        deltaTrans.getRotation(q);
        btVector3 axis = q.getAxis();
        float omega = q.getAngle() / deltaTime;
        btVector3 omega_v = omega * axis;

        for (int i = 0; i < m_numLinks; ++i) {
            btVector3 r = m_multiBody->getLink(i).m_cachedWorldTransform.getOrigin();
            r = r - basePos;
            btVector3 c_force = -m_multiBody->getLink(i).m_mass * btCross(omega_v, btCross(omega_v, r));
            c_force *= m_centrifugalDragEffect / pow((i + 1), 3.5) ;
            m_multiBody->addLinkForce(i, c_force);
        }
    }
}

void Skeleton::OnInternalTickCallback(btDynamicsWorld* world, btScalar timeStep)
{
    Skeleton* demo = static_cast<Skeleton*>(world->getWorldUserInfo());

    // draw center of the link
    if (WIRE_FRAME)
    {
        btVector4 color(1, 0, 0, 1);
        btVector3 base = demo->m_multiBody->getBasePos();
        for ( int i = 0; i < demo->m_multiBody->getNumLinks(); i++ ){
            btVector3 c = demo->m_multiBody->getLink(i).m_cachedWorldTransform.getOrigin();
            demo->m_guiHelper->getRenderInterface()->drawPoint(c, color, btScalar(5));
        }
    }

    int numManifolds = world->getDispatcher()->getNumManifolds();
    bool has_collision = false;
    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();

        // only detect collision between bones and collider
        if (
                (!(obA->getBroadphaseHandle()->m_collisionFilterGroup & BONE_BODY)
                    && !(obB->getBroadphaseHandle()->m_collisionFilterGroup | BONE_BODY))
                ||
                (!(obA->getBroadphaseHandle()->m_collisionFilterGroup & TARGET_BODY)
                 && !(obB->getBroadphaseHandle()->m_collisionFilterGroup | TARGET_BODY))
        )
            continue;

        int numContacts = contactManifold->getNumContacts();
        for (int j = 0; j < numContacts; j++)
        {
            // when there is no collision, set gravity to go downside
            has_collision = true;

            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            printf("step : %d, collision impulse: %f\n", demo->m_step, pt.getAppliedImpulse());

            btVector3 ptOnBone, normalOnBone;
            // if obA is bone
            if ( obA->getBroadphaseHandle()->m_collisionFilterGroup ){
                ptOnBone = pt.getPositionWorldOnA();
                normalOnBone = pt.m_normalWorldOnB;
            }
            else{
                ptOnBone = pt.getPositionWorldOnB();
                normalOnBone = -pt.m_normalWorldOnB;
            }
            normalOnBone.normalize();
            printf("ptA: %f, %f, %f\n", ptOnBone.getX(), ptOnBone.getY(), ptOnBone.getZ());
            printf("normal: %f, %f, %f\n", normalOnBone.getX(), normalOnBone.getY(), normalOnBone.getZ());

//            btVector3 to = ptOnBone + normalOnBone * (pt.getAppliedImpulse() + 1e-2) * 10;
            btVector3 to = ptOnBone + normalOnBone;
            btVector4 color(1, 0, 0, 1);
            if (demo->m_guiHelper->getRenderInterface())
            {
                if (WIRE_FRAME)
                {
                    demo->m_guiHelper->getRenderInterface()->drawLine(ptOnBone, to, color, btScalar(1));
                }
            }
        }
    }

    if ( has_collision )
        demo->m_g.startGravity();
    else
        demo->m_g.stopGravity(); // when there is no collision, set gravity to zero
}

void Skeleton::stepSimulation(float deltaTime) {
    float g = m_g.getGVal();
    m_dynamicsWorld->setGravity(btVector3(0, g, 0));

    // calculate and apply the impulse, the damp use the difference between prev pos and curr pos
    applySpringForce(deltaTime);

    // calculate the drag force
    btTransform trans = transformBase(m_time);  // in the beginning, m_time == 0
    applyBaseLinearDragForce(deltaTime, m_step, trans);
    applyBaseCentrifugalForce(deltaTime, m_step, trans);
    m_prevBaseTrans = trans;

    moveCollider(m_time);

    // capture the frames
//    {
//        const char* gPngFileName = "multibody";
//        sprintf(mfileName, "%s_%d.png", gPngFileName, m_step);
//        this->m_guiHelper->getAppInterface()->dumpNextFrameToPng(mfileName);
//    }

    // step and update positions
    btScalar  fixedTimeStep = deltaTime / btScalar(2.0);
    m_dynamicsWorld->stepSimulation(deltaTime, 2, fixedTimeStep);
    if ( WIRE_FRAME ) {
        m_dynamicsWorld->debugDrawWorld();
    }

    m_time += deltaTime;
    m_step += 1;
}


class CommonExampleInterface* Dof6Spring2CreateFunc(CommonExampleOptions& options)
{
//    return new Dof6Spring2Setup(options.m_guiHelper);
	return new Skeleton(options.m_guiHelper);
}
