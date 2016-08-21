import bullet

# This is a Hello World program for running a basic Bullet physics simulation

def Main():
    # collision configuration contains default setup for memory, collision
    # setup. Advanced users can create their own configuration.
    collision_config = bullet.btDefaultCollisionConfiguration()

    # Use the default collision dispatcher.
    dispatcher = bullet.btCollisionDispatcher(collision_config)

    # btDbvtBroadphase is a good general purpose broadphase.
    # You can also try out btAxis3Sweep.
    broadphase = bullet.btDbvtBroadphase()

    # The default constraint solver.
    solver = bullet.btMultiBodyConstraintSolver()
    world = bullet.btMultiBodyDynamicsWorld(dispatcher, broadphase, solver, collision_config)

    # Set the gravity.
    grav = bullet.btVector3(0, -10.0, 0)
    world.setGravity(grav);

    # Create a few basic rigid bodies.

    # The ground is a cube of side 100 at position y = -56.
    groundShape = bullet.btBoxShape(bullet.btVector3(50.0, 50.0, 50.0))
    groundTransform = bullet.btTransform()
    groundTransform.setIdentity();
    groundTransform.setOrigin(bullet.btVector3(0,-56,0))
    mass = 0.0
    localInertia = bullet.btVector3(0,0,0)
    groundShape.calculateLocalInertia(mass,localInertia)
    myMotionState = bullet.btDefaultMotionState(groundTransform)
    rbInfo = bullet.btRigidBodyConstructionInfo(mass,myMotionState,groundShape,localInertia)
    body = bullet.btRigidBody(rbInfo)
    world.addRigidBody(body)


    # Box to drop onto the ground.
    boxShape = bullet.btBoxShape(bullet.btVector3(1.0, 1.0, 1.0))
    startTransform = bullet.btTransform()
    startTransform.setIdentity()
    startTransform.setOrigin(bullet.btVector3(2,5,0))
    boxMass = 1.0
    boxInertia = bullet.btVector3(0,0,0)
    boxShape.calculateLocalInertia(boxMass,localInertia)
    boxMotionState = bullet.btDefaultMotionState(startTransform)
    box_rbInfo = bullet.btRigidBodyConstructionInfo(boxMass,boxMotionState,boxShape,boxInertia)
    box = bullet.btRigidBody(box_rbInfo)
    world.addRigidBody(box)

    # Do some simulation.
    deltaTime = 1/60.0
    for _ in range(200):
        world.stepSimulation(deltaTime)
        t = bullet.btTransform()
        boxMotionState.getWorldTransform(t)
        pos = t.getOrigin()
        print pos.m_floats


    # Shutdown order is important. Reverse of above.
    del world
    del solver
    del broadphase
    del dispatcher
    del collision_config

if __name__ == '__main__':
  Main()
