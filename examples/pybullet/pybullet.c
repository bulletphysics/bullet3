#include "../SharedMemory/PhysicsClientC_API.h"
#include "../SharedMemory/PhysicsDirectC_API.h"
#include "../SharedMemory/SharedMemoryInProcessPhysicsC_API.h"


#ifdef __APPLE__
#include <Python/Python.h>
#else
#include <Python.h>
#endif

#ifdef PYBULLET_USE_NUMPY
#include <numpy/arrayobject.h>
#endif

#if PY_MAJOR_VERSION >= 3
#define PyInt_FromLong PyLong_FromLong
#define PyString_FromString PyBytes_FromString
#endif

enum eCONNECT_METHOD {
  eCONNECT_GUI = 1,
  eCONNECT_DIRECT = 2,
  eCONNECT_SHARED_MEMORY = 3,
};

static PyObject* SpamError;
static b3PhysicsClientHandle sm = 0;

// Step through one timestep of the simulation
static PyObject* pybullet_stepSimulation(PyObject* self, PyObject* args) {
  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  {
    b3SharedMemoryStatusHandle statusHandle;
    int statusType;

    if (b3CanSubmitCommand(sm)) {
      statusHandle = b3SubmitClientCommandAndWaitStatus(
          sm, b3InitStepSimulationCommand(sm));
      statusType = b3GetStatusType(statusHandle);
    }
  }

  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject* pybullet_connectPhysicsServer(PyObject* self, PyObject* args) {
  if (0 != sm) {
    PyErr_SetString(SpamError,
                    "Already connected to physics server, disconnect first.");
    return NULL;
  }

  {
    int method = eCONNECT_GUI;
    if (!PyArg_ParseTuple(args, "i", &method)) {
      PyErr_SetString(SpamError,
                      "connectPhysicsServer expected argument  eCONNECT_GUI, "
                      "eCONNECT_DIRECT or eCONNECT_SHARED_MEMORY");
      return NULL;
    }

    switch (method) {
      case eCONNECT_GUI: {
        int argc = 0;
        char* argv[1] = {0};

#ifdef __APPLE__
        sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
#else
        sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
#endif
        break;
      }
      case eCONNECT_DIRECT: {
        sm = b3ConnectPhysicsDirect();
        break;
      }
      case eCONNECT_SHARED_MEMORY: {
        sm = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
        break;
      }

      default: {
        PyErr_SetString(SpamError, "connectPhysicsServer unexpected argument");
        return NULL;
      }
    };
  }

  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject* pybullet_disconnectPhysicsServer(PyObject* self,
                                                  PyObject* args) {
  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }
  {
    b3DisconnectSharedMemory(sm);
    sm = 0;
  }

  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject* pybullet_saveWorld(PyObject* self, PyObject* args) {
	int size = PySequence_Size(args);
	const char* worldFileName = "";

	if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

   if (size == 1) {
    if (!PyArg_ParseTuple(args, "s", &worldFileName))
	{
		return NULL;
	}
	else
	{
		b3SharedMemoryCommandHandle	command;
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;

		command = b3SaveWorldCommandInit(sm, worldFileName);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		statusType = b3GetStatusType(statusHandle);
		if (statusType != CMD_SAVE_WORLD_COMPLETED) {
			PyErr_SetString(SpamError, "saveWorld command execution failed.");
			return NULL;
		}
		Py_INCREF(Py_None);
		return Py_None;
	}
  }


     
	PyErr_SetString(SpamError, "Cannot execute saveWorld command.");
	return NULL;
    
    return NULL;
}



// Load a URDF file indicating the links and joints of an object
// function can be called without arguments and will default
// to position (0,0,1) with orientation(0,0,0,1)
// els(x,y,z) or
// loadURDF(pos_x, pos_y, pos_z, orn_x, orn_y, orn_z, orn_w)
static PyObject* pybullet_loadURDF(PyObject* self, PyObject* args) {
  int size = PySequence_Size(args);

  int bodyIndex = -1;
  const char* urdfFileName = "";

  double startPosX = 0.0;
  double startPosY = 0.0;
  double startPosZ = 0.0;
  double startOrnX = 0.0;
  double startOrnY = 0.0;
  double startOrnZ = 0.0;
  double startOrnW = 1.0;

  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }
  if (size == 1) {
    if (!PyArg_ParseTuple(args, "s", &urdfFileName)) return NULL;
  }
  if (size == 4) {
    if (!PyArg_ParseTuple(args, "sddd", &urdfFileName, &startPosX, &startPosY,
                          &startPosZ))
      return NULL;
  }
  if (size == 8) {
    if (!PyArg_ParseTuple(args, "sddddddd", &urdfFileName, &startPosX,
                          &startPosY, &startPosZ, &startOrnX, &startOrnY,
                          &startOrnZ, &startOrnW))
      return NULL;
  }

  if (strlen(urdfFileName)) {
    // printf("(%f, %f, %f) (%f, %f, %f, %f)\n",
    // startPosX,startPosY,startPosZ,startOrnX, startOrnY,startOrnZ, startOrnW);

    b3SharedMemoryStatusHandle statusHandle;
    int statusType;
    b3SharedMemoryCommandHandle command =
        b3LoadUrdfCommandInit(sm, urdfFileName);

    // setting the initial position, orientation and other arguments are
    // optional
    b3LoadUrdfCommandSetStartPosition(command, startPosX, startPosY, startPosZ);
    b3LoadUrdfCommandSetStartOrientation(command, startOrnX, startOrnY,
                                         startOrnZ, startOrnW);
    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
    statusType = b3GetStatusType(statusHandle);
    if (statusType != CMD_URDF_LOADING_COMPLETED) {
      PyErr_SetString(SpamError, "Cannot load URDF file.");
      return NULL;
    }
    bodyIndex = b3GetStatusBodyIndex(statusHandle);
  } else {
    PyErr_SetString(SpamError,
                    "Empty filename, method expects 1, 4 or 8 arguments.");
    return NULL;
  }
  return PyLong_FromLong(bodyIndex);
}

static double pybullet_internalGetFloatFromSequence(PyObject* seq, int index) {
  double v = 0.0;
  PyObject* item;

  if (PyList_Check(seq)) {
    item = PyList_GET_ITEM(seq, index);
    v = PyFloat_AsDouble(item);
  } else {
    item = PyTuple_GET_ITEM(seq, index);
    v = PyFloat_AsDouble(item);
  }
  return v;
}

#define MAX_SDF_BODIES 512

static PyObject* pybullet_loadSDF(PyObject* self, PyObject* args) {
  const char* sdfFileName = "";
  int size = PySequence_Size(args);
  int numBodies = 0;
  int i;
  int bodyIndicesOut[MAX_SDF_BODIES];
  PyObject* pylist = 0;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  b3SharedMemoryCommandHandle commandHandle;

  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  if (size == 1) {
    if (!PyArg_ParseTuple(args, "s", &sdfFileName)) return NULL;
  }

  commandHandle = b3LoadSdfCommandInit(sm, sdfFileName);
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  statusType = b3GetStatusType(statusHandle);
  if (statusType != CMD_SDF_LOADING_COMPLETED) {
    PyErr_SetString(SpamError, "Cannot load SDF file.");
    return NULL;
  }

  numBodies =
      b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES);
  if (numBodies > MAX_SDF_BODIES) {
    PyErr_SetString(SpamError, "SDF exceeds body capacity");
    return NULL;
  }

  pylist = PyTuple_New(numBodies);

  if (numBodies > 0 && numBodies <= MAX_SDF_BODIES) {
    for (i = 0; i < numBodies; i++) {
      PyTuple_SetItem(pylist, i, PyInt_FromLong(bodyIndicesOut[i]));
    }
  }
  return pylist;
}

// Reset the simulation to remove all loaded objects
static PyObject* pybullet_resetSimulation(PyObject* self, PyObject* args) {
  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  {
    b3SharedMemoryStatusHandle statusHandle;
    statusHandle = b3SubmitClientCommandAndWaitStatus(
        sm, b3InitResetSimulationCommand(sm));
  }
  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject* pybullet_setJointMotorControl(PyObject* self, PyObject* args) {
  int size;
  int bodyIndex, jointIndex, controlMode;

  double targetPosition = 0.0;
  double targetVelocity = 0.0;
  double maxForce = 100000.0;
  double appliedForce = 0.0;
  double kp = 0.1;
  double kd = 1.0;
  int valid = 0;

  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  size = PySequence_Size(args);
  if (size == 4) {
    double targetValue = 0.0;
    // see switch statement below for convertsions dependent on controlMode
    if (!PyArg_ParseTuple(args, "iiid", &bodyIndex, &jointIndex, &controlMode,
                          &targetValue)) {
      PyErr_SetString(SpamError, "Error parsing arguments");
      return NULL;
    }
    valid = 1;
    switch (controlMode) {
      case CONTROL_MODE_POSITION_VELOCITY_PD: {
        targetPosition = targetValue;
        break;
      }
      case CONTROL_MODE_VELOCITY: {
        targetVelocity = targetValue;
        break;
      }
      case CONTROL_MODE_TORQUE: {
        appliedForce = targetValue;
        break;
      }
      default: { valid = 0; }
    }
  }
  if (size == 5) {
    double targetValue = 0.0;
    // See switch statement for conversions
    if (!PyArg_ParseTuple(args, "iiidd", &bodyIndex, &jointIndex, &controlMode,
                          &targetValue, &maxForce)) {
      PyErr_SetString(SpamError, "Error parsing arguments");
      return NULL;
    }
    valid = 1;

    switch (controlMode) {
      case CONTROL_MODE_POSITION_VELOCITY_PD: {
        targetPosition = targetValue;
        break;
      }
      case CONTROL_MODE_VELOCITY: {
        targetVelocity = targetValue;
        break;
      }
      case CONTROL_MODE_TORQUE: {
        valid = 0;
        break;
      }
      default: { valid = 0; }
    }
  }
  if (size == 6) {
    double gain = 0.0;
    double targetValue = 0.0;
    if (!PyArg_ParseTuple(args, "iiiddd", &bodyIndex, &jointIndex, &controlMode,
                          &targetValue, &maxForce, &gain)) {
      PyErr_SetString(SpamError, "Error parsing arguments");
      return NULL;
    }
    valid = 1;

    switch (controlMode) {
      case CONTROL_MODE_POSITION_VELOCITY_PD: {
        targetPosition = targetValue;
        kp = gain;
        break;
      }
      case CONTROL_MODE_VELOCITY: {
        targetVelocity = targetValue;
        kd = gain;
        break;
      }
      case CONTROL_MODE_TORQUE: {
        valid = 0;
        break;
      }
      default: { valid = 0; }
    }
  }
  if (size == 8) {
    // only applicable for CONTROL_MODE_POSITION_VELOCITY_PD.
    if (!PyArg_ParseTuple(args, "iiiddddd", &bodyIndex, &jointIndex,
                          &controlMode, &targetPosition, &targetVelocity,
                          &maxForce, &kp, &kd)) {
      PyErr_SetString(SpamError, "Error parsing arguments");
      return NULL;
    }
    valid = 1;
  }

  if (valid) {
    int numJoints;
    b3SharedMemoryCommandHandle commandHandle;
    b3SharedMemoryStatusHandle statusHandle;
    struct b3JointInfo info;

    numJoints = b3GetNumJoints(sm, bodyIndex);
    if ((jointIndex >= numJoints) || (jointIndex < 0)) {
      PyErr_SetString(SpamError, "Joint index out-of-range.");
      return NULL;
    }

    if ((controlMode != CONTROL_MODE_VELOCITY) &&
        (controlMode != CONTROL_MODE_TORQUE) &&
        (controlMode != CONTROL_MODE_POSITION_VELOCITY_PD)) {
      PyErr_SetString(SpamError, "Illegral control mode.");
      return NULL;
    }

    commandHandle = b3JointControlCommandInit2(sm, bodyIndex, controlMode);

    b3GetJointInfo(sm, bodyIndex, jointIndex, &info);

    switch (controlMode) {
      case CONTROL_MODE_VELOCITY: {
        b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
                                         targetVelocity);
        b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
        b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, maxForce);
        break;
      }

      case CONTROL_MODE_TORQUE: {
        b3JointControlSetDesiredForceTorque(commandHandle, info.m_uIndex,
                                            appliedForce);
        break;
      }

      case CONTROL_MODE_POSITION_VELOCITY_PD: {
        b3JointControlSetDesiredPosition(commandHandle, info.m_qIndex,
                                         targetPosition);
        b3JointControlSetKp(commandHandle, info.m_uIndex, kp);
        b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
                                         targetVelocity);
        b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
        b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, maxForce);
        break;
      }
      default: {}
    };

    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);

    Py_INCREF(Py_None);
    return Py_None;
  }
  PyErr_SetString(SpamError, "Error parsing arguments in setJointControl.");
  return NULL;
}

static PyObject* pybullet_setRealTimeSimulation(PyObject* self,
                                                PyObject* args) {
  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  {
    int enableRealTimeSimulation = 0;
    int ret;

    b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
    b3SharedMemoryStatusHandle statusHandle;

    if (!PyArg_ParseTuple(args, "i", &enableRealTimeSimulation)) {
      PyErr_SetString(
          SpamError,
          "setRealTimeSimulation expected a single value (integer).");
      return NULL;
    }
    ret =
        b3PhysicsParamSetRealTimeSimulation(command, enableRealTimeSimulation);

    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
    // ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
  }

  Py_INCREF(Py_None);
  return Py_None;
}

// Set the gravity of the world with (x, y, z) arguments
static PyObject* pybullet_setGravity(PyObject* self, PyObject* args) {
  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  {
    double gravX = 0.0;
    double gravY = 0.0;
    double gravZ = -10.0;
    int ret;

    b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
    b3SharedMemoryStatusHandle statusHandle;

    if (!PyArg_ParseTuple(args, "ddd", &gravX, &gravY, &gravZ)) {
      PyErr_SetString(SpamError, "setGravity expected (x,y,z) values.");
      return NULL;
    }
    ret = b3PhysicsParamSetGravity(command, gravX, gravY, gravZ);
    // ret = b3PhysicsParamSetTimeStep(command,  timeStep);
    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
    // ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
  }

  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject* pybullet_setTimeStep(PyObject* self, PyObject* args) {
  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  {
    double timeStep = 0.001;
    int ret;


    b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
    b3SharedMemoryStatusHandle statusHandle;

    if (!PyArg_ParseTuple(args, "d", &timeStep)) {
      PyErr_SetString(SpamError,
                      "setTimeStep expected a single value (double).");
      return NULL;
    }
    ret = b3PhysicsParamSetTimeStep(command, timeStep);
    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
    // ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
  }

  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject *
pybullet_setDefaultContactERP(PyObject* self, PyObject* args)
{
    if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }
    
    {
        double defaultContactERP=0.005;
        int ret;
        
        b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
        b3SharedMemoryStatusHandle statusHandle;
        
        if (!PyArg_ParseTuple(args, "d", &defaultContactERP))
        {
            PyErr_SetString(SpamError, "default Contact ERP expected a single value (double).");
            return NULL;
        }
        ret = b3PhysicsParamSetDefaultContactERP(command,  defaultContactERP);

        statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
    }
    
    Py_INCREF(Py_None);
    return Py_None;
}


// Internal function used to get the base position and orientation
// Orientation is returned in quaternions
static int pybullet_internalGetBasePositionAndOrientation(
    int bodyIndex, double basePosition[3], double baseOrientation[3]) {
  basePosition[0] = 0.;
  basePosition[1] = 0.;
  basePosition[2] = 0.;

  baseOrientation[0] = 0.;
  baseOrientation[1] = 0.;
  baseOrientation[2] = 0.;
  baseOrientation[3] = 1.;

  {
    {
      b3SharedMemoryCommandHandle cmd_handle =
          b3RequestActualStateCommandInit(sm, bodyIndex);
      b3SharedMemoryStatusHandle status_handle =
          b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

      const int status_type = b3GetStatusType(status_handle);
      const double* actualStateQ;
      // const double* jointReactionForces[];
      

      if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED) {
        PyErr_SetString(SpamError, "getBasePositionAndOrientation failed.");
        return 0;
      }

      b3GetStatusActualState(
          status_handle, 0 /* body_unique_id */,
          0 /* num_degree_of_freedom_q */, 0 /* num_degree_of_freedom_u */,
          0 /*root_local_inertial_frame*/, &actualStateQ,
          0 /* actual_state_q_dot */, 0 /* joint_reaction_forces */);

      // printf("joint reaction forces=");
      // for (i=0; i < (sizeof(jointReactionForces)/sizeof(double)); i++) {
      //   printf("%f ", jointReactionForces[i]);
      // }
      // now, position x,y,z = actualStateQ[0],actualStateQ[1],actualStateQ[2]
      // and orientation x,y,z,w =
      // actualStateQ[3],actualStateQ[4],actualStateQ[5],actualStateQ[6]
      basePosition[0] = actualStateQ[0];
      basePosition[1] = actualStateQ[1];
      basePosition[2] = actualStateQ[2];

      baseOrientation[0] = actualStateQ[3];
      baseOrientation[1] = actualStateQ[4];
      baseOrientation[2] = actualStateQ[5];
      baseOrientation[3] = actualStateQ[6];
    }
  }
  return 1;
}

// Get the positions (x,y,z) and orientation (x,y,z,w) in quaternion
// values for the base link of your object
// Object is retrieved based on body index, which is the order
// the object was loaded into the simulation (0-based)
static PyObject* pybullet_getBasePositionAndOrientation(PyObject* self,
                                                        PyObject* args) {
  int bodyIndex = -1;
  double basePosition[3];
  double baseOrientation[4];
  PyObject* pylistPos;
  PyObject* pylistOrientation;

  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  if (!PyArg_ParseTuple(args, "i", &bodyIndex)) {
    PyErr_SetString(SpamError, "Expected a body index (integer).");
    return NULL;
  }

  if (0 == pybullet_internalGetBasePositionAndOrientation(
               bodyIndex, basePosition, baseOrientation)) {
    PyErr_SetString(SpamError,
                    "GetBasePositionAndOrientation failed (#joints/links "
                    "exceeds maximum?).");
    return NULL;
  }

  {
    PyObject* item;
    int i;
    int num = 3;
    pylistPos = PyTuple_New(num);
    for (i = 0; i < num; i++) {
      item = PyFloat_FromDouble(basePosition[i]);
      PyTuple_SetItem(pylistPos, i, item);
    }
  }

  {
    PyObject* item;
    int i;
    int num = 4;
    pylistOrientation = PyTuple_New(num);
    for (i = 0; i < num; i++) {
      item = PyFloat_FromDouble(baseOrientation[i]);
      PyTuple_SetItem(pylistOrientation, i, item);
    }
  }

  {
    PyObject* pylist;
    pylist = PyTuple_New(2);
    PyTuple_SetItem(pylist, 0, pylistPos);
    PyTuple_SetItem(pylist, 1, pylistOrientation);
    return pylist;
  }
}

static PyObject* pybullet_getNumBodies(PyObject* self, PyObject* args)
{
	if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

 {
    int numBodies = b3GetNumBodies(sm);

#if PY_MAJOR_VERSION >= 3
    return PyLong_FromLong(numBodies);
#else
    return PyInt_FromLong(numBodies);
#endif
  }
}

static PyObject* pybullet_getBodyUniqueId(PyObject* self, PyObject* args)
{
	if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

	{
    int serialIndex = -1;
    int bodyUniqueId = -1;
    if (!PyArg_ParseTuple(args, "i", &serialIndex)) {
      PyErr_SetString(SpamError, "Expected a serialIndex in range [0..number of bodies).");
      return NULL;
    }
    bodyUniqueId = b3GetBodyUniqueId(sm, serialIndex);

#if PY_MAJOR_VERSION >= 3
    return PyLong_FromLong(bodyUniqueId);
#else
    return PyInt_FromLong(bodyUniqueId);
#endif
  }
}

static PyObject* pybullet_getBodyInfo(PyObject* self, PyObject* args)
{
	if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

	{
    int bodyUniqueId= -1;
    int numJoints = 0;
    if (!PyArg_ParseTuple(args, "i", &bodyUniqueId)) 
	{
      PyErr_SetString(SpamError, "Expected a body unique id (integer).");
      return NULL;
    }
		{
			struct b3BodyInfo info;
			if (b3GetBodyInfo(sm,bodyUniqueId,&info))
			{
				PyObject* pyListJointInfo = PyTuple_New(1);
				PyTuple_SetItem(pyListJointInfo, 0, PyString_FromString(info.m_baseName));
				return pyListJointInfo;
			} else
			{
				PyErr_SetString(SpamError, "Couldn't get body info");
				return NULL;
			}
		}
	}

  PyErr_SetString(SpamError, "error in getBodyInfo.");
  return NULL;
}


// Return the number of joints in an object based on
// body index; body index is based on order of sequence
// the object is loaded into simulation
static PyObject* pybullet_getNumJoints(PyObject* self, PyObject* args) 
{
  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  {
    int bodyIndex = -1;
    int numJoints = 0;
    if (!PyArg_ParseTuple(args, "i", &bodyIndex)) {
      PyErr_SetString(SpamError, "Expected a body index (integer).");
      return NULL;
    }
    numJoints = b3GetNumJoints(sm, bodyIndex);

#if PY_MAJOR_VERSION >= 3
    return PyLong_FromLong(numJoints);
#else
    return PyInt_FromLong(numJoints);
#endif
  }
}

// Initalize all joint positions given a list of values
static PyObject* pybullet_resetJointState(PyObject* self, PyObject* args) {
  int size;
  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  size = PySequence_Size(args);

  if (size == 3) {
    int bodyIndex;
    int jointIndex;
    double targetValue;

    if (PyArg_ParseTuple(args, "iid", &bodyIndex, &jointIndex, &targetValue)) {
      b3SharedMemoryCommandHandle commandHandle;
      b3SharedMemoryStatusHandle statusHandle;
      int numJoints;

      numJoints = b3GetNumJoints(sm, bodyIndex);
      if ((jointIndex >= numJoints) || (jointIndex < 0)) {
        PyErr_SetString(SpamError, "Joint index out-of-range.");
        return NULL;
      }

      commandHandle = b3CreatePoseCommandInit(sm, bodyIndex);

      b3CreatePoseCommandSetJointPosition(sm, commandHandle, jointIndex,
                                          targetValue);

      statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
      Py_INCREF(Py_None);
      return Py_None;
    }
  }
  PyErr_SetString(SpamError, "error in resetJointState.");
  return NULL;
}

// Reset the position and orientation of the base/root link, position [x,y,z]
// and orientation quaternion [x,y,z,w]
static PyObject* pybullet_resetBasePositionAndOrientation(PyObject* self,
                                                          PyObject* args) {
  int size;
  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  size = PySequence_Size(args);

  if (size == 3) {
    int bodyIndex;
    PyObject* posObj;
    PyObject* ornObj;
    double pos[3];
    double orn[4];  // as a quaternion

    if (PyArg_ParseTuple(args, "iOO", &bodyIndex, &posObj, &ornObj)) {
      b3SharedMemoryCommandHandle commandHandle;
      b3SharedMemoryStatusHandle statusHandle;

      {
        PyObject* seq;
        int len, i;
        seq = PySequence_Fast(posObj, "expected a sequence");
        len = PySequence_Size(posObj);
        if (len == 3) {
          for (i = 0; i < 3; i++) {
            pos[i] = pybullet_internalGetFloatFromSequence(seq, i);
          }
        } else {
          PyErr_SetString(SpamError, "position needs a 3 coordinates [x,y,z].");
          Py_DECREF(seq);
          return NULL;
        }
        Py_DECREF(seq);
      }

      {
        PyObject* seq;
        int len, i;
        seq = PySequence_Fast(ornObj, "expected a sequence");
        len = PySequence_Size(ornObj);
        if (len == 4) {
          for (i = 0; i < 4; i++) {
            orn[i] = pybullet_internalGetFloatFromSequence(seq, i);
          }
        } else {
          PyErr_SetString(
              SpamError,
              "orientation needs a 4 coordinates, quaternion [x,y,z,w].");
          Py_DECREF(seq);
          return NULL;
        }
        Py_DECREF(seq);
      }

      commandHandle = b3CreatePoseCommandInit(sm, bodyIndex);

      b3CreatePoseCommandSetBasePosition(commandHandle, pos[0], pos[1], pos[2]);
      b3CreatePoseCommandSetBaseOrientation(commandHandle, orn[0], orn[1],
                                            orn[2], orn[3]);

      statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
      Py_INCREF(Py_None);
      return Py_None;
    }
  }
  PyErr_SetString(SpamError, "error in resetJointState.");
  return NULL;
}

// Get the a single joint info for a specific bodyIndex
//
// Args:
//  bodyIndex - integer indicating body in simulation
//  jointIndex - integer indicating joint for a specific body
//
// Joint information includes:
//  index, name, type, q-index, u-index,
//  flags, joint damping, joint friction
//
// The format of the returned list is
// [int, str, int, int, int, int, float, float]
//
// TODO(hellojas): get joint positions for a body
static PyObject* pybullet_getJointInfo(PyObject* self, PyObject* args) {
  PyObject* pyListJointInfo;

  struct b3JointInfo info;

  int bodyIndex = -1;
  int jointIndex = -1;
  int jointInfoSize = 8;  // size of struct b3JointInfo

  int size = PySequence_Size(args);

  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  if (size == 2)  // get body index and joint index
  {
    if (PyArg_ParseTuple(args, "ii", &bodyIndex, &jointIndex)) {
      // printf("body index = %d, joint index =%d\n", bodyIndex, jointIndex);

      pyListJointInfo = PyTuple_New(jointInfoSize);

      if (b3GetJointInfo(sm, bodyIndex, jointIndex, &info)) {
        //  printf("Joint%d %s, type %d, at q-index %d and u-index %d\n",
        //          info.m_jointIndex,
        //          info.m_jointName,
        //          info.m_jointType,
        //          info.m_qIndex,
        //          info.m_uIndex);
        //  printf("  flags=%d jointDamping=%f jointFriction=%f\n",
        //          info.m_flags,
        //          info.m_jointDamping,
        //          info.m_jointFriction);
        PyTuple_SetItem(pyListJointInfo, 0, PyInt_FromLong(info.m_jointIndex));
        PyTuple_SetItem(pyListJointInfo, 1,
                        PyString_FromString(info.m_jointName));
        PyTuple_SetItem(pyListJointInfo, 2, PyInt_FromLong(info.m_jointType));
        PyTuple_SetItem(pyListJointInfo, 3, PyInt_FromLong(info.m_qIndex));
        PyTuple_SetItem(pyListJointInfo, 4, PyInt_FromLong(info.m_uIndex));
        PyTuple_SetItem(pyListJointInfo, 5, PyInt_FromLong(info.m_flags));
        PyTuple_SetItem(pyListJointInfo, 6,
                        PyFloat_FromDouble(info.m_jointDamping));
        PyTuple_SetItem(pyListJointInfo, 7,
                        PyFloat_FromDouble(info.m_jointFriction));
        return pyListJointInfo;
      } else {
        PyErr_SetString(SpamError, "GetJointInfo failed.");
        return NULL;
      }
    }
  }

  Py_INCREF(Py_None);
  return Py_None;
}

// Returns the state of a specific joint in a given bodyIndex
//
// Args:
//  bodyIndex - integer indicating body in simulation
//  jointIndex - integer indicating joint for a specific body
//
// The state of a joint includes the following:
//  position, velocity, force torque (6 values), and motor torque
// The returned pylist is an array of [float, float, float[6], float]

// TODO(hellojas): check accuracy of position and velocity
// TODO(hellojas): check force torque values

static PyObject* pybullet_getJointState(PyObject* self, PyObject* args) {
  PyObject* pyListJointForceTorque;
  PyObject* pyListJointState;


  struct b3JointSensorState sensorState;

  int bodyIndex = -1;
  int jointIndex = -1;
  int sensorStateSize = 4;  // size of struct b3JointSensorState
  int forceTorqueSize = 6;  // size of force torque list from b3JointSensorState
  int j;

  int size = PySequence_Size(args);

  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  if (size == 2)  // get body index and joint index
  {
    if (PyArg_ParseTuple(args, "ii", &bodyIndex, &jointIndex)) {
	  int status_type = 0;
	  b3SharedMemoryCommandHandle cmd_handle;
	  b3SharedMemoryStatusHandle status_handle;


      if (bodyIndex < 0) {
        PyErr_SetString(SpamError, "getJointState failed; invalid bodyIndex");
        return NULL;
      }
      if (jointIndex < 0) {
        PyErr_SetString(SpamError, "getJointState failed; invalid jointIndex");
        return NULL;
      }

      
      cmd_handle =
          b3RequestActualStateCommandInit(sm, bodyIndex);
      status_handle =
          b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

      status_type = b3GetStatusType(status_handle);
      if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED) {
        PyErr_SetString(SpamError, "getJointState failed.");
        return NULL;
      }

      pyListJointState = PyTuple_New(sensorStateSize);
      pyListJointForceTorque = PyTuple_New(forceTorqueSize);

      b3GetJointState(sm, status_handle, jointIndex, &sensorState);

      PyTuple_SetItem(pyListJointState, 0,
                      PyFloat_FromDouble(sensorState.m_jointPosition));
      PyTuple_SetItem(pyListJointState, 1,
                      PyFloat_FromDouble(sensorState.m_jointVelocity));

      for (j = 0; j < forceTorqueSize; j++) {
        PyTuple_SetItem(pyListJointForceTorque, j,
                        PyFloat_FromDouble(sensorState.m_jointForceTorque[j]));
      }

      PyTuple_SetItem(pyListJointState, 2, pyListJointForceTorque);

      PyTuple_SetItem(pyListJointState, 3,
                      PyFloat_FromDouble(sensorState.m_jointMotorTorque));

      return pyListJointState;
    }
  } else {
    PyErr_SetString(
        SpamError,
        "getJointState expects 2 arguments (objectUniqueId and joint index).");
    return NULL;
  }

  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject* pybullet_getLinkState(PyObject* self, PyObject* args) {
  PyObject* pyLinkState;
  PyObject* pyLinkStateWorldPosition;
  PyObject* pyLinkStateWorldOrientation;
  PyObject* pyLinkStateLocalInertialPosition;
  PyObject* pyLinkStateLocalInertialOrientation;

  struct b3LinkState linkState;

  int bodyIndex = -1;
  int linkIndex = -1;
  int i;

  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  if (PySequence_Size(args) == 2)  // body index and link index
  {
    if (PyArg_ParseTuple(args, "ii", &bodyIndex, &linkIndex)) {
	int status_type = 0;
	b3SharedMemoryCommandHandle cmd_handle;
	b3SharedMemoryStatusHandle status_handle;

      if (bodyIndex < 0) {
        PyErr_SetString(SpamError, "getLinkState failed; invalid bodyIndex");
        return NULL;
      }
      if (linkIndex < 0) {
        PyErr_SetString(SpamError, "getLinkState failed; invalid jointIndex");
        return NULL;
      }


      cmd_handle =
          b3RequestActualStateCommandInit(sm, bodyIndex);
      status_handle =
          b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

      status_type = b3GetStatusType(status_handle);
      if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED) {
        PyErr_SetString(SpamError, "getLinkState failed.");
        return NULL;
      }

      b3GetLinkState(sm, status_handle, linkIndex, &linkState);

      pyLinkStateWorldPosition = PyTuple_New(3);
      for (i = 0; i < 3; ++i) {
        PyTuple_SetItem(pyLinkStateWorldPosition, i,
                        PyFloat_FromDouble(linkState.m_worldPosition[i]));
      }

      pyLinkStateWorldOrientation = PyTuple_New(4);
      for (i = 0; i < 4; ++i) {
        PyTuple_SetItem(pyLinkStateWorldOrientation, i,
                        PyFloat_FromDouble(linkState.m_worldOrientation[i]));
      }

      pyLinkStateLocalInertialPosition = PyTuple_New(3);
      for (i = 0; i < 3; ++i) {
        PyTuple_SetItem(pyLinkStateLocalInertialPosition, i,
                        PyFloat_FromDouble(linkState.m_localInertialPosition[i]));
      }

      pyLinkStateLocalInertialOrientation = PyTuple_New(4);
      for (i = 0; i < 4; ++i) {
        PyTuple_SetItem(pyLinkStateLocalInertialOrientation, i,
                        PyFloat_FromDouble(linkState.m_localInertialOrientation[i]));
      }

      pyLinkState = PyTuple_New(4);
      PyTuple_SetItem(pyLinkState, 0, pyLinkStateWorldPosition);
      PyTuple_SetItem(pyLinkState, 1, pyLinkStateWorldOrientation);
      PyTuple_SetItem(pyLinkState, 2, pyLinkStateLocalInertialPosition);
      PyTuple_SetItem(pyLinkState, 3, pyLinkStateLocalInertialOrientation);

      return pyLinkState;
    }
  } else {
    PyErr_SetString(
        SpamError,
        "getLinkState expects 2 arguments (objectUniqueId and link index).");
    return NULL;
  }

  Py_INCREF(Py_None);
  return Py_None;
}

// internal function to set a float matrix[16]
// used to initialize camera position with
// a view and projection matrix in renderImage()
//
// // Args:
//  matrix - float[16] which will be set by values from objMat
static int pybullet_internalSetMatrix(PyObject* objMat, float matrix[16]) {
  int i, len;
  PyObject* seq;

  seq = PySequence_Fast(objMat, "expected a sequence");
  len = PySequence_Size(objMat);
  if (len == 16) {
    for (i = 0; i < len; i++) {
      matrix[i] = pybullet_internalGetFloatFromSequence(seq, i);
    }
    Py_DECREF(seq);
    return 1;
  }
  Py_DECREF(seq);
  return 0;
}

// internal function to set a float vector[3]
// used to initialize camera position with
// a view and projection matrix in renderImage()
//
// // Args:
//  vector - float[3] which will be set by values from objMat
static int pybullet_internalSetVector(PyObject* objMat, float vector[3]) {
  int i, len;
  PyObject* seq;

  seq = PySequence_Fast(objMat, "expected a sequence");
  len = PySequence_Size(objMat);
  if (len == 3) {
    for (i = 0; i < len; i++) {
      vector[i] = pybullet_internalGetFloatFromSequence(seq, i);
    }
    Py_DECREF(seq);
    return 1;
  }
  Py_DECREF(seq);
  return 0;
}

//  vector - double[3] which will be set by values from obVec
static int pybullet_internalSetVectord(PyObject* obVec, double vector[3]) {
    int i, len;
    PyObject* seq;
    
    seq = PySequence_Fast(obVec, "expected a sequence");
    len = PySequence_Size(obVec);
    if (len == 3) {
        for (i = 0; i < len; i++) {
            vector[i] = pybullet_internalGetFloatFromSequence(seq, i);
        }
        Py_DECREF(seq);
        return 1;
    }
    Py_DECREF(seq);
    return 0;
}

//  vector - double[3] which will be set by values from obVec
static int pybullet_internalSetVector4(PyObject* obVec, double vector[4]) {
    int i, len;
    PyObject* seq;
    
    seq = PySequence_Fast(obVec, "expected a sequence");
    len = PySequence_Size(obVec);
    if (len == 4) {
        for (i = 0; i < len; i++) {
            vector[i] = pybullet_internalGetFloatFromSequence(seq, i);
        }
        Py_DECREF(seq);
        return 1;
    }
    Py_DECREF(seq);
    return 0;
}

static PyObject* pybullet_getContactPointData(PyObject* self, PyObject* args) {
  int size = PySequence_Size(args);
  int objectUniqueIdA = -1;
  int objectUniqueIdB = -1;
  b3SharedMemoryCommandHandle commandHandle;
  struct b3ContactInformation contactPointData;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  int i;
  PyObject* pyResultList = 0;

  if (size == 1) {
    if (!PyArg_ParseTuple(args, "i", &objectUniqueIdA)) {
      PyErr_SetString(SpamError, "Error parsing object unique id");
      return NULL;
    }
  }
  if (size == 2) {
    if (!PyArg_ParseTuple(args, "ii", &objectUniqueIdA, &objectUniqueIdB)) {
      PyErr_SetString(SpamError, "Error parsing object unique id");
      return NULL;
    }
  }

  commandHandle = b3InitRequestContactPointInformation(sm);
  b3SetContactFilterBodyA(commandHandle, objectUniqueIdA);
  b3SetContactFilterBodyB(commandHandle, objectUniqueIdB);
  b3SubmitClientCommand(sm, commandHandle);
  
  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  statusType = b3GetStatusType(statusHandle);
  if (statusType == CMD_CONTACT_POINT_INFORMATION_COMPLETED) {
    /*
     0     int m_contactFlags;
     1     int m_bodyUniqueIdA;
     2     int m_bodyUniqueIdB;
     3     int m_linkIndexA;
     4     int m_linkIndexB;
     5-6-7     double m_positionOnAInWS[3];//contact point location on object A,
     in world space coordinates
     8-9-10     double m_positionOnBInWS[3];//contact point location on object
     A, in world space coordinates
     11-12-13     double m_contactNormalOnBInWS[3];//the separating contact
     normal, pointing from object B towards object A
     14     double m_contactDistance;//negative number is penetration, positive
     is distance.

     15    double m_normalForce;
     */

    b3GetContactPointInformation(sm, &contactPointData);
    pyResultList = PyTuple_New(contactPointData.m_numContactPoints);
    for (i = 0; i < contactPointData.m_numContactPoints; i++) {
      PyObject* contactObList = PyTuple_New(16);  // see above 16 fields
      PyObject* item;
      item =
          PyInt_FromLong(contactPointData.m_contactPointData[i].m_contactFlags);
      PyTuple_SetItem(contactObList, 0, item);
      item = PyInt_FromLong(
          contactPointData.m_contactPointData[i].m_bodyUniqueIdA);
      PyTuple_SetItem(contactObList, 1, item);
      item = PyInt_FromLong(
          contactPointData.m_contactPointData[i].m_bodyUniqueIdB);
      PyTuple_SetItem(contactObList, 2, item);
      item =
          PyInt_FromLong(contactPointData.m_contactPointData[i].m_linkIndexA);
      PyTuple_SetItem(contactObList, 3, item);
      item =
          PyInt_FromLong(contactPointData.m_contactPointData[i].m_linkIndexB);
      PyTuple_SetItem(contactObList, 4, item);
      item = PyFloat_FromDouble(
          contactPointData.m_contactPointData[i].m_positionOnAInWS[0]);
      PyTuple_SetItem(contactObList, 5, item);
      item = PyFloat_FromDouble(
          contactPointData.m_contactPointData[i].m_positionOnAInWS[1]);
      PyTuple_SetItem(contactObList, 6, item);
      item = PyFloat_FromDouble(
          contactPointData.m_contactPointData[i].m_positionOnAInWS[2]);
      PyTuple_SetItem(contactObList, 7, item);

      item = PyFloat_FromDouble(
          contactPointData.m_contactPointData[i].m_positionOnBInWS[0]);
      PyTuple_SetItem(contactObList, 8, item);
      item = PyFloat_FromDouble(
          contactPointData.m_contactPointData[i].m_positionOnBInWS[1]);
      PyTuple_SetItem(contactObList, 9, item);
      item = PyFloat_FromDouble(
          contactPointData.m_contactPointData[i].m_positionOnBInWS[2]);
      PyTuple_SetItem(contactObList, 10, item);

      item = PyFloat_FromDouble(
          contactPointData.m_contactPointData[i].m_contactNormalOnBInWS[0]);
      PyTuple_SetItem(contactObList, 11, item);
      item = PyFloat_FromDouble(
          contactPointData.m_contactPointData[i].m_contactNormalOnBInWS[1]);
      PyTuple_SetItem(contactObList, 12, item);
      item = PyFloat_FromDouble(
          contactPointData.m_contactPointData[i].m_contactNormalOnBInWS[2]);
      PyTuple_SetItem(contactObList, 13, item);

      item = PyFloat_FromDouble(
          contactPointData.m_contactPointData[i].m_contactDistance);
      PyTuple_SetItem(contactObList, 14, item);
      item = PyFloat_FromDouble(
          contactPointData.m_contactPointData[i].m_normalForce);
      PyTuple_SetItem(contactObList, 15, item);

      PyTuple_SetItem(pyResultList, i, contactObList);
    }
    return pyResultList;
  }

  Py_INCREF(Py_None);
  return Py_None;
}

// Render an image from the current timestep of the simulation
//
// Examples:
//  renderImage() - default image resolution and camera position
//  renderImage(w, h) - image resolution of (w,h), default camera
//  renderImage(w, h, view[16], projection[16]) - set both resolution
//    and initialize camera to the view and projection values
//  renderImage(w, h, cameraPos, targetPos, cameraUp, nearVal, farVal) - set
//    resolution and initialize camera based on camera position, target
//    position, camera up and fulstrum near/far values.
//  renderImage(w, h, cameraPos, targetPos, cameraUp, nearVal, farVal, fov) -
//    set resolution and initialize camera based on camera position, target
//    position, camera up, fulstrum near/far values and camera field of view.
//  renderImage(w, h, targetPos, distance, yaw, pitch, upAxisIndex, nearVal,
//  farVal, fov)

//
// Note if the (w,h) is too small, the objects may not appear based on
// where the camera has been set
//
// TODO(hellojas): fix image is cut off at head
// TODO(hellojas): should we add check to give minimum image resolution
//  to see object based on camera position?
static PyObject* pybullet_renderImage(PyObject* self, PyObject* args) {
  /// request an image from a simulated camera, using a software renderer.
  struct b3CameraImageData imageData;
  PyObject* objViewMat, *objProjMat;
  PyObject* objCameraPos, *objTargetPos, *objCameraUp;

  int width, height;
  int size = PySequence_Size(args);
  float viewMatrix[16];
  float projectionMatrix[16];

  float cameraPos[3];
  float targetPos[3];
  float cameraUp[3];

  float left, right, bottom, top, aspect;
  float nearVal, farVal;
  float fov;

  // inialize cmd
  b3SharedMemoryCommandHandle command;

  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  command = b3InitRequestCameraImage(sm);

  if (size == 2)  // only set camera resolution
  {
    if (PyArg_ParseTuple(args, "ii", &width, &height)) {
      b3RequestCameraImageSetPixelResolution(command, width, height);
    }
  } else if (size == 4)  // set camera resolution and view and projection matrix
  {
    if (PyArg_ParseTuple(args, "iiOO", &width, &height, &objViewMat,
                         &objProjMat)) {
      b3RequestCameraImageSetPixelResolution(command, width, height);

      // set camera matrices only if set matrix function succeeds
      if (pybullet_internalSetMatrix(objViewMat, viewMatrix) &&
          (pybullet_internalSetMatrix(objProjMat, projectionMatrix))) {
        b3RequestCameraImageSetCameraMatrices(command, viewMatrix,
                                              projectionMatrix);
      } else {
        PyErr_SetString(SpamError, "Error parsing view or projection matrix.");
        return NULL;
      }
    }
  } else if (size == 7)  // set camera resolution, camera positions and
                         // calculate projection using near/far values.
  {
    if (PyArg_ParseTuple(args, "iiOOOff", &width, &height, &objCameraPos,
                         &objTargetPos, &objCameraUp, &nearVal, &farVal)) {
      b3RequestCameraImageSetPixelResolution(command, width, height);
      if (pybullet_internalSetVector(objCameraPos, cameraPos) &&
          pybullet_internalSetVector(objTargetPos, targetPos) &&
          pybullet_internalSetVector(objCameraUp, cameraUp)) {
        b3RequestCameraImageSetViewMatrix(command, cameraPos, targetPos,
                                          cameraUp);
      } else {
        PyErr_SetString(SpamError,
                        "Error parsing camera position, target or up.");
        return NULL;
      }

      aspect = width / height;
      left = -aspect * nearVal;
      right = aspect * nearVal;
      bottom = -nearVal;
      top = nearVal;
      b3RequestCameraImageSetProjectionMatrix(command, left, right, bottom, top,
                                              nearVal, farVal);
    }
  } else if (size == 8)  // set camera resolution, camera positions and
                         // calculate projection using near/far values & field
                         // of view
  {
    if (PyArg_ParseTuple(args, "iiOOOfff", &width, &height, &objCameraPos,
                         &objTargetPos, &objCameraUp, &nearVal, &farVal,
                         &fov)) {
      b3RequestCameraImageSetPixelResolution(command, width, height);
      if (pybullet_internalSetVector(objCameraPos, cameraPos) &&
          pybullet_internalSetVector(objTargetPos, targetPos) &&
          pybullet_internalSetVector(objCameraUp, cameraUp)) {
        b3RequestCameraImageSetViewMatrix(command, cameraPos, targetPos,
                                          cameraUp);
      } else {
        PyErr_SetString(SpamError,
                        "Error parsing camera position, target or up.");
        return NULL;
      }

      aspect = width / height;
      b3RequestCameraImageSetFOVProjectionMatrix(command, fov, aspect, nearVal,
                                                 farVal);
    }
  } else if (size == 11) {
    int upAxisIndex = 1;
    float camDistance, yaw, pitch, roll;

    // sometimes more arguments are better :-)
    if (PyArg_ParseTuple(args, "iiOffffifff", &width, &height, &objTargetPos,
                         &camDistance, &yaw, &pitch, &roll, &upAxisIndex,
                         &nearVal, &farVal, &fov)) {
      b3RequestCameraImageSetPixelResolution(command, width, height);
      if (pybullet_internalSetVector(objTargetPos, targetPos)) {
        // printf("width = %d, height = %d, targetPos = %f,%f,%f, distance = %f,
        // yaw = %f, pitch = %f,   upAxisIndex = %d, near=%f, far=%f,
        // fov=%f\n",width,height,targetPos[0],targetPos[1],targetPos[2],camDistance,yaw,pitch,upAxisIndex,nearVal,farVal,fov);

        b3RequestCameraImageSetViewMatrix2(command, targetPos, camDistance, yaw,
                                           pitch, roll, upAxisIndex);
        aspect = width / height;
        b3RequestCameraImageSetFOVProjectionMatrix(command, fov, aspect,
                                                   nearVal, farVal);
      } else {
        PyErr_SetString(SpamError, "Error parsing camera target pos");
      }
    } else {
      PyErr_SetString(SpamError, "Error parsing arguments");
    }

  } else {
    PyErr_SetString(SpamError, "Invalid number of args passed to renderImage.");
    return NULL;
  }

  if (b3CanSubmitCommand(sm)) {
    b3SharedMemoryStatusHandle statusHandle;
    int statusType;

    // b3RequestCameraImageSelectRenderer(command,ER_BULLET_HARDWARE_OPENGL);

    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
    statusType = b3GetStatusType(statusHandle);
    if (statusType == CMD_CAMERA_IMAGE_COMPLETED) {
      PyObject* item2;
      PyObject* pyResultList;  // store 4 elements in this result: width,
                               // height, rgbData, depth

#ifdef PYBULLET_USE_NUMPY
      PyObject* pyRGB;
      PyObject* pyDep;
      PyObject* pySeg;

      int i, j, p;

      b3GetCameraImageData(sm, &imageData);
      // TODO(hellojas): error handling if image size is 0
      pyResultList = PyTuple_New(5);
      PyTuple_SetItem(pyResultList, 0, PyInt_FromLong(imageData.m_pixelWidth));
      PyTuple_SetItem(pyResultList, 1, PyInt_FromLong(imageData.m_pixelHeight));

      int bytesPerPixel = 4;  // Red, Green, Blue, and Alpha each 8 bit values

      npy_intp rgb_dims[3] = {imageData.m_pixelHeight, imageData.m_pixelWidth,
        bytesPerPixel};
      npy_intp dep_dims[2] = {imageData.m_pixelHeight, imageData.m_pixelWidth};
      npy_intp seg_dims[2] = {imageData.m_pixelHeight, imageData.m_pixelWidth};

      pyRGB = PyArray_SimpleNew(3, rgb_dims, NPY_UINT8);
      pyDep = PyArray_SimpleNew(2, dep_dims, NPY_FLOAT32);
      pySeg = PyArray_SimpleNew(2, seg_dims, NPY_INT32);

      memcpy(PyArray_DATA(pyRGB), imageData.m_rgbColorData,
        imageData.m_pixelHeight * imageData.m_pixelWidth * bytesPerPixel);
      memcpy(PyArray_DATA(pyDep), imageData.m_depthValues,
        imageData.m_pixelHeight * imageData.m_pixelWidth);
      memcpy(PyArray_DATA(pySeg), imageData.m_segmentationMaskValues,
        imageData.m_pixelHeight * imageData.m_pixelWidth);
    
      PyTuple_SetItem(pyResultList, 2, pyRGB);
      PyTuple_SetItem(pyResultList, 3, pyDep);
      PyTuple_SetItem(pyResultList, 4, pySeg);
#else//PYBULLET_USE_NUMPY
   PyObject* pylistRGB;
      PyObject* pylistDep;
      PyObject* pylistSeg;

      int i, j, p;

      b3GetCameraImageData(sm, &imageData);
      // TODO(hellojas): error handling if image size is 0
      pyResultList = PyTuple_New(5);
      PyTuple_SetItem(pyResultList, 0, PyInt_FromLong(imageData.m_pixelWidth));
      PyTuple_SetItem(pyResultList, 1, PyInt_FromLong(imageData.m_pixelHeight));

      {
        PyObject* item;
        int bytesPerPixel = 4;  // Red, Green, Blue, and Alpha each 8 bit values
        int num =
            bytesPerPixel * imageData.m_pixelWidth * imageData.m_pixelHeight;
        pylistRGB = PyTuple_New(num);
        pylistDep =
            PyTuple_New(imageData.m_pixelWidth * imageData.m_pixelHeight);
        pylistSeg =
            PyTuple_New(imageData.m_pixelWidth * imageData.m_pixelHeight);
        for (i = 0; i < imageData.m_pixelWidth; i++) {
          for (j = 0; j < imageData.m_pixelHeight; j++) {
            // TODO(hellojas): validate depth values make sense
            int depIndex = i + j * imageData.m_pixelWidth;
            {
              item = PyFloat_FromDouble(imageData.m_depthValues[depIndex]);
              PyTuple_SetItem(pylistDep, depIndex, item);
            }
            {
              item2 =
                  PyLong_FromLong(imageData.m_segmentationMaskValues[depIndex]);
              PyTuple_SetItem(pylistSeg, depIndex, item2);
            }

            for (p = 0; p < bytesPerPixel; p++) {
              int pixelIndex =
                  bytesPerPixel * (i + j * imageData.m_pixelWidth) + p;
              item = PyInt_FromLong(imageData.m_rgbColorData[pixelIndex]);
              PyTuple_SetItem(pylistRGB, pixelIndex, item);
            }
          }
        }
      }

      PyTuple_SetItem(pyResultList, 2, pylistRGB);
      PyTuple_SetItem(pyResultList, 3, pylistDep);
      PyTuple_SetItem(pyResultList, 4, pylistSeg);
      return pyResultList;
#endif//PYBULLET_USE_NUMPY

      return pyResultList;
    }
  }

  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject* pybullet_applyExternalForce(PyObject* self, PyObject* args) {
  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }
  {
    int objectUniqueId, linkIndex, flags;
    double force[3];
    double position[3] = {0.0, 0.0, 0.0};
    PyObject* forceObj, *posObj;

    b3SharedMemoryCommandHandle command;
    b3SharedMemoryStatusHandle statusHandle;
    int size = PySequence_Size(args);

    if (size == 5) {
      if (!PyArg_ParseTuple(args, "iiOOi", &objectUniqueId, &linkIndex,
                            &forceObj, &posObj, &flags)) {
        PyErr_SetString(SpamError, "applyBaseForce couldn't parse arguments");
        return NULL;
      }
    } else {
      PyErr_SetString(SpamError,
                      "applyBaseForce needs 5 arguments: objectUniqueId, "
                      "linkIndex (-1 for base/root link), force [x,y,z], "
                      "position [x,y,z], flags");

      return NULL;
    }

    {
      PyObject* seq;
      int len, i;
      seq = PySequence_Fast(forceObj, "expected a sequence");
      len = PySequence_Size(forceObj);
      if (len == 3) {
        for (i = 0; i < 3; i++) {
          force[i] = pybullet_internalGetFloatFromSequence(seq, i);
        }
      } else {
        PyErr_SetString(SpamError, "force needs a 3 coordinates [x,y,z].");
        Py_DECREF(seq);
        return NULL;
      }
      Py_DECREF(seq);
    }
    {
      PyObject* seq;
      int len, i;
      seq = PySequence_Fast(posObj, "expected a sequence");
      len = PySequence_Size(posObj);
      if (len == 3) {
        for (i = 0; i < 3; i++) {
          position[i] = pybullet_internalGetFloatFromSequence(seq, i);
        }
      } else {
        PyErr_SetString(SpamError, "position needs a 3 coordinates [x,y,z].");
        Py_DECREF(seq);
        return NULL;
      }
      Py_DECREF(seq);
    }
    if ((flags != EF_WORLD_FRAME) && (flags != EF_LINK_FRAME)) {
      PyErr_SetString(SpamError,
                      "flag has to be either WORLD_FRAME or LINK_FRAME");
      return NULL;
    }
    command = b3ApplyExternalForceCommandInit(sm);
    b3ApplyExternalForce(command, objectUniqueId, linkIndex, force, position,
                         flags);
    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
  }
  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject* pybullet_applyExternalTorque(PyObject* self, PyObject* args) {
  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }
  {
    int objectUniqueId, linkIndex, flags;
    double torque[3];
    PyObject* torqueObj;

    if (PyArg_ParseTuple(args, "iiOi", &objectUniqueId, &linkIndex, &torqueObj,
                         &flags)) {
      PyObject* seq;
      int len, i;
      seq = PySequence_Fast(torqueObj, "expected a sequence");
      len = PySequence_Size(torqueObj);
      if (len == 3) {
        for (i = 0; i < 3; i++) {
          torque[i] = pybullet_internalGetFloatFromSequence(seq, i);
        }
      } else {
        PyErr_SetString(SpamError, "torque needs a 3 coordinates [x,y,z].");
        Py_DECREF(seq);
        return NULL;
      }
      Py_DECREF(seq);

      if (linkIndex < -1) {
        PyErr_SetString(SpamError,
                        "Invalid link index, has to be -1 or larger");
        return NULL;
      }
      if ((flags != EF_WORLD_FRAME) && (flags != EF_LINK_FRAME)) {
        PyErr_SetString(SpamError,
                        "flag has to be either WORLD_FRAME or LINK_FRAME");
        return NULL;
      }
      {
        b3SharedMemoryStatusHandle statusHandle;
        b3SharedMemoryCommandHandle command =
            b3ApplyExternalForceCommandInit(sm);
        b3ApplyExternalTorque(command, objectUniqueId, -1, torque, flags);
        statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
      }
    }
  }

  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject* pybullet_getQuaternionFromEuler(PyObject* self,
                                                 PyObject* args) {
  double rpy[3];

  PyObject* eulerObj;

  if (PyArg_ParseTuple(args, "O", &eulerObj)) {
    PyObject* seq;
    int len, i;
    seq = PySequence_Fast(eulerObj, "expected a sequence");
    len = PySequence_Size(eulerObj);
    if (len == 3) {
      for (i = 0; i < 3; i++) {
        rpy[i] = pybullet_internalGetFloatFromSequence(seq, i);
      }
    } else {
      PyErr_SetString(SpamError,
                      "Euler angles need a 3 coordinates [roll, pitch, yaw].");
      Py_DECREF(seq);
      return NULL;
    }
    Py_DECREF(seq);
  } else {
    PyErr_SetString(SpamError,
                    "Euler angles need a 3 coordinates [roll, pitch, yaw].");
    return NULL;
  }

  {
    double phi, the, psi;
    double roll = rpy[0];
    double pitch = rpy[1];
    double yaw = rpy[2];
    phi = roll / 2.0;
    the = pitch / 2.0;
    psi = yaw / 2.0;
    {
      double quat[4] = {
          sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi),
          cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi),
          cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi),
          cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi)};

      // normalize the quaternion
      double len = sqrt(quat[0] * quat[0] + quat[1] * quat[1] +
                        quat[2] * quat[2] + quat[3] * quat[3]);
      quat[0] /= len;
      quat[1] /= len;
      quat[2] /= len;
      quat[3] /= len;
      {
        PyObject* pylist;
        int i;
        pylist = PyTuple_New(4);
        for (i = 0; i < 4; i++)
          PyTuple_SetItem(pylist, i, PyFloat_FromDouble(quat[i]));
        return pylist;
      }
    }
  }
  Py_INCREF(Py_None);
  return Py_None;
}
/// quaternion <-> euler yaw/pitch/roll convention from URDF/SDF, see Gazebo
/// https://github.com/arpg/Gazebo/blob/master/gazebo/math/Quaternion.cc
static PyObject* pybullet_getEulerFromQuaternion(PyObject* self,
                                                 PyObject* args) {
  double squ;
  double sqx;
  double sqy;
  double sqz;

  double quat[4];

  PyObject* quatObj;

  if (PyArg_ParseTuple(args, "O", &quatObj)) {
    PyObject* seq;
    int len, i;
    seq = PySequence_Fast(quatObj, "expected a sequence");
    len = PySequence_Size(quatObj);
    if (len == 4) {
      for (i = 0; i < 4; i++) {
        quat[i] = pybullet_internalGetFloatFromSequence(seq, i);
      }
    } else {
      PyErr_SetString(SpamError, "Quaternion need a 4 components [x,y,z,w].");
      Py_DECREF(seq);
      return NULL;
    }
    Py_DECREF(seq);
  } else {
    PyErr_SetString(SpamError, "Quaternion need a 4 components [x,y,z,w].");
    return NULL;
  }

  {
    double rpy[3];
    double sarg;
    sqx = quat[0] * quat[0];
    sqy = quat[1] * quat[1];
    sqz = quat[2] * quat[2];
    squ = quat[3] * quat[3];
    rpy[0] = atan2(2 * (quat[1] * quat[2] + quat[3] * quat[0]),
                   squ - sqx - sqy + sqz);
    sarg = -2 * (quat[0] * quat[2] - quat[3] * quat[1]);
    rpy[1] = sarg <= -1.0 ? -0.5 * 3.141592538
                          : (sarg >= 1.0 ? 0.5 * 3.141592538 : asin(sarg));
    rpy[2] = atan2(2 * (quat[0] * quat[1] + quat[3] * quat[2]),
                   squ + sqx - sqy - sqz);
    {
      PyObject* pylist;
      int i;
      pylist = PyTuple_New(3);
      for (i = 0; i < 3; i++)
        PyTuple_SetItem(pylist, i, PyFloat_FromDouble(rpy[i]));
      return pylist;
    }
  }
  Py_INCREF(Py_None);
  return Py_None;
}


///Experimental Inverse Kinematics binding ,7-dof KUKA IIWA only
static PyObject* pybullet_calculateInverseKinematicsKuka(PyObject* self,
                                                   PyObject* args) {
    int size;
    if (0 == sm) {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }
    
    size = PySequence_Size(args);
    if (size == 2)
    {
        int bodyIndex;
		int endEffectorLinkIndex;
		
        PyObject* targetPosObj;
		PyObject* targetOrnObj;

        if (PyArg_ParseTuple(args, "iiOO", &bodyIndex, &endEffectorLinkIndex, &targetPosObj,&targetOrnObj))
        {
            double pos[3];
            double ori[4]={0,1.0,0,0};
            
            if (pybullet_internalSetVectord(targetPosObj,pos) && pybullet_internalSetVector4(targetOrnObj,ori))
            {
                b3SharedMemoryStatusHandle statusHandle;
                int numPos=0;
                int resultBodyIndex;
                int result;

                b3SharedMemoryCommandHandle command = b3CalculateInverseKinematicsCommandInit(sm,bodyIndex);
				b3CalculateInverseKinematicsAddTargetPositionWithOrientation(command,6,pos,ori);
                statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
                
                result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
                                                                         &resultBodyIndex,
                                                                         &numPos,
                                                                         0);
                if (result && numPos)
                {
                    int i;
                    PyObject* pylist;
                    double* ikOutPutJointPos = (double*)malloc(numPos*sizeof(double));
                    result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
                                                                        &resultBodyIndex,
                                                                        &numPos,
                                                                        ikOutPutJointPos);
                    pylist = PyTuple_New(numPos);
                    for (i = 0; i < numPos; i++)
                    {
                        PyTuple_SetItem(pylist, i,
                                        PyFloat_FromDouble(ikOutPutJointPos[i]));
                    }
                    
                    free(ikOutPutJointPos);
                    return pylist;
                }
                else
                {
                    PyErr_SetString(SpamError,
                                    "Error in calculateInverseKinematics");
                    return NULL;
                }
            } else
            {
                PyErr_SetString(SpamError,
                    "calculateInverseKinematics couldn't extract position vector3");
                return NULL;
            }
        }
    } else
    {
        PyErr_SetString(SpamError,
                        "calculateInverseKinematics expects 2 arguments, body index, "
                        "and target position for end effector");
        return NULL;
    }
    Py_INCREF(Py_None);
    return Py_None;
}



/// Given an object id, joint positions, joint velocities and joint
/// accelerations,
/// compute the joint forces using Inverse Dynamics
static PyObject* pybullet_calculateInverseDynamics(PyObject* self,
                                                   PyObject* args) {
  int size;
  if (0 == sm) {
    PyErr_SetString(SpamError, "Not connected to physics server.");
    return NULL;
  }

  size = PySequence_Size(args);
  if (size == 4) {
    int bodyIndex;
    PyObject* objPositionsQ;
    PyObject* objVelocitiesQdot;
    PyObject* objAccelerations;

    if (PyArg_ParseTuple(args, "iOOO", &bodyIndex, &objPositionsQ,
                         &objVelocitiesQdot, &objAccelerations)) {
      int szObPos = PySequence_Size(objPositionsQ);
      int szObVel = PySequence_Size(objVelocitiesQdot);
      int szObAcc = PySequence_Size(objAccelerations);
      int numJoints = b3GetNumJoints(sm, bodyIndex);
      if (numJoints && (szObPos == numJoints) && (szObVel == numJoints) &&
          (szObAcc == numJoints)) {
        int szInBytes = sizeof(double) * numJoints;
        int i;
        PyObject* pylist = 0;
        double* jointPositionsQ = (double*)malloc(szInBytes);
        double* jointVelocitiesQdot = (double*)malloc(szInBytes);
        double* jointAccelerations = (double*)malloc(szInBytes);
        double* jointForcesOutput = (double*)malloc(szInBytes);

        for (i = 0; i < numJoints; i++) {
          jointPositionsQ[i] =
              pybullet_internalGetFloatFromSequence(objPositionsQ, i);
          jointVelocitiesQdot[i] =
              pybullet_internalGetFloatFromSequence(objVelocitiesQdot, i);
          jointAccelerations[i] =
              pybullet_internalGetFloatFromSequence(objAccelerations, i);
        }

        {
          b3SharedMemoryStatusHandle statusHandle;
          int statusType;
          b3SharedMemoryCommandHandle commandHandle =
              b3CalculateInverseDynamicsCommandInit(
                  sm, bodyIndex, jointPositionsQ, jointVelocitiesQdot,
                  jointAccelerations);
          statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);

          statusType = b3GetStatusType(statusHandle);

          if (statusType == CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED) {
            int bodyUniqueId;
            int dofCount;

            b3GetStatusInverseDynamicsJointForces(statusHandle, &bodyUniqueId,
                                                  &dofCount, 0);

            if (dofCount) {
              b3GetStatusInverseDynamicsJointForces(statusHandle, 0, 0,
                                                    jointForcesOutput);
              {
                {
                  int i;
                  pylist = PyTuple_New(dofCount);
                  for (i = 0; i < dofCount; i++)
                    PyTuple_SetItem(pylist, i,
                                    PyFloat_FromDouble(jointForcesOutput[i]));
                }
              }
            }

          } else {
            PyErr_SetString(SpamError,
                            "Internal error in calculateInverseDynamics");
          }
        }
        free(jointPositionsQ);
        free(jointVelocitiesQdot);
        free(jointAccelerations);
        free(jointForcesOutput);
        if (pylist) return pylist;
      } else {
        PyErr_SetString(SpamError,
                        "calculateInverseDynamics numJoints needs to be "
                        "positive and [joint positions], [joint velocities], "
                        "[joint accelerations] need to match the number of "
                        "joints.");
        return NULL;
      }

    } else {
      PyErr_SetString(SpamError,
                      "calculateInverseDynamics expects 4 arguments, body "
                      "index, [joint positions], [joint velocities], [joint "
                      "accelerations].");
      return NULL;
    }
  } else {
    PyErr_SetString(SpamError,
                    "calculateInverseDynamics expects 4 arguments, body index, "
                    "[joint positions], [joint velocities], [joint "
                    "accelerations].");
    return NULL;
  }
  Py_INCREF(Py_None);
  return Py_None;
}

static PyMethodDef SpamMethods[] = {

    {"connect", pybullet_connectPhysicsServer, METH_VARARGS,
     "Connect to an existing physics server (using shared memory by default)."},

    {"disconnect", pybullet_disconnectPhysicsServer, METH_VARARGS,
     "Disconnect from the physics server."},

    {"resetSimulation", pybullet_resetSimulation, METH_VARARGS,
     "Reset the simulation: remove all objects and start from an empty world."},

    {"stepSimulation", pybullet_stepSimulation, METH_VARARGS,
     "Step the simulation using forward dynamics."},

    {"setGravity", pybullet_setGravity, METH_VARARGS,
     "Set the gravity acceleration (x,y,z)."},

    {"setTimeStep", pybullet_setTimeStep, METH_VARARGS,
     "Set the amount of time to proceed at each call to stepSimulation. (unit "
     "is seconds, typically range is 0.01 or 0.001)"},


    {"setTimeStep",  pybullet_setTimeStep, METH_VARARGS,
        "Set the amount of time to proceed at each call to stepSimulation."
        " (unit is seconds, typically range is 0.01 or 0.001)"},

	{"setDefaultContactERP",  pybullet_setDefaultContactERP, METH_VARARGS,
        "Set the amount of contact penetration Error Recovery Paramater "
        "(ERP) in each time step. \
		This is an tuning parameter to control resting contact stability. "
		"This value depends on the time step."},
    
	{ "setRealTimeSimulation", pybullet_setRealTimeSimulation, METH_VARARGS,
	"Enable or disable real time simulation (using the real time clock,"
	" RTC) in the physics server. Expects one integer argument, 0 or 1" },

    {"loadURDF", pybullet_loadURDF, METH_VARARGS,
     "Create a multibody by loading a URDF file."},

    {"loadSDF", pybullet_loadSDF, METH_VARARGS,
     "Load multibodies from an SDF file."},

	{"saveWorld", pybullet_saveWorld, METH_VARARGS,
     "Save an approximate Python file to reproduce the current state of the world: saveWorld"
	"(filename). (very preliminary and approximately)"},

	{"getNumBodies", pybullet_getNumBodies, METH_VARARGS,
     "Get the number of bodies in the simulation."},

	{"getBodyUniqueId", pybullet_getBodyUniqueId, METH_VARARGS,
     "Get the unique id of the body, given a integer serial index in range [0.. number of bodies)."},

	{"getBodyInfo", pybullet_getBodyInfo, METH_VARARGS,
     "Get the body info, given a body unique id."},
	
    {"getBasePositionAndOrientation", pybullet_getBasePositionAndOrientation,
     METH_VARARGS,
     "Get the world position and orientation of the base of the object. "
     "(x,y,z) position vector and (x,y,z,w) quaternion orientation."},

    {"resetBasePositionAndOrientation",
     pybullet_resetBasePositionAndOrientation, METH_VARARGS,
     "Reset the world position and orientation of the base of the object "
     "instantaneously, not through physics simulation. (x,y,z) position vector "
     "and (x,y,z,w) quaternion orientation."},

    {"getNumJoints", pybullet_getNumJoints, METH_VARARGS,
     "Get the number of joints for an object."},

    {"getJointInfo", pybullet_getJointInfo, METH_VARARGS,
     "Get the name and type info for a joint on a body."},

    {"getJointState", pybullet_getJointState, METH_VARARGS,
     "Get the state (position, velocity etc) for a joint on a body."},

    {"getLinkState", pybullet_getLinkState, METH_VARARGS,
     "Provides extra information such as the Cartesian world coordinates"
     " center of mass (COM) of the link, relative to the world reference"
     " frame."},

    {"resetJointState", pybullet_resetJointState, METH_VARARGS,
     "Reset the state (position, velocity etc) for a joint on a body "
     "instantaneously, not through physics simulation."},

    {"setJointMotorControl", pybullet_setJointMotorControl, METH_VARARGS,
     "Set a single joint motor control mode and desired target value. There is "
     "no immediate state change, stepSimulation will process the motors."},

    {"applyExternalForce", pybullet_applyExternalForce, METH_VARARGS,
     "for objectUniqueId, linkIndex (-1 for base/root link), apply a force "
     "[x,y,z] at the a position [x,y,z], flag to select FORCE_IN_LINK_FRAME or "
     "FORCE_IN_WORLD_FRAME coordinates"},

    {"applyExternalTorque", pybullet_applyExternalTorque, METH_VARARGS,
     "for objectUniqueId, linkIndex (-1 for base/root link) apply a torque "
     "[x,y,z] in Cartesian coordinates, flag to select TORQUE_IN_LINK_FRAME or "
     "TORQUE_IN_WORLD_FRAME coordinates"},

    {"renderImage", pybullet_renderImage, METH_VARARGS,
     "Render an image (given the pixel resolution width, height, camera view "
     "matrix, projection matrix, near, and far values), and return the "
     "8-8-8bit RGB pixel data and floating point depth values"
#ifdef PYBULLET_USE_NUMPY
     " as NumPy arrays"
#endif
     },

    {"getContactPointData", pybullet_getContactPointData, METH_VARARGS,
     "Return the contact point information for all or some of pairwise "
     "object-object collisions. Optional arguments one or two object unique "
     "ids, that need to be involved in the contact."},

    {"getQuaternionFromEuler", pybullet_getQuaternionFromEuler, METH_VARARGS,
     "Convert Euler [roll, pitch, yaw] as in URDF/SDF convention, to "
     "quaternion [x,y,z,w]"},

    {"getEulerFromQuaternion", pybullet_getEulerFromQuaternion, METH_VARARGS,
     "Convert quaternion [x,y,z,w] to Euler [roll, pitch, yaw] as in URDF/SDF "
     "convention"},

    {"calculateInverseDynamics", pybullet_calculateInverseDynamics,
     METH_VARARGS,
     "Given an object id, joint positions, joint velocities and joint "
     "accelerations, compute the joint forces using Inverse Dynamics"},
    
    {"calculateInverseKinematicsKuka", pybullet_calculateInverseKinematicsKuka,
        METH_VARARGS,
        "Experimental, KUKA IIWA only: Given an object id, "
        "current joint positions and target position"
        " for the end effector,"
        "compute the inverse kinematics and return the new joint state"},
    
    // todo(erwincoumans)
    // saveSnapshot
    // loadSnapshot
    // raycast info
    // object names

    {NULL, NULL, 0, NULL} /* Sentinel */
};

#if PY_MAJOR_VERSION >= 3
static struct PyModuleDef moduledef = {
    PyModuleDef_HEAD_INIT, "pybullet", /* m_name */
    "Python bindings for Bullet Physics Robotics API (also known as Shared "
    "Memory API)", /* m_doc */
    -1,            /* m_size */
    SpamMethods,   /* m_methods */
    NULL,          /* m_reload */
    NULL,          /* m_traverse */
    NULL,          /* m_clear */
    NULL,          /* m_free */
};
#endif

PyMODINIT_FUNC
#if PY_MAJOR_VERSION >= 3
PyInit_pybullet(void)
#else
initpybullet(void)
#endif
{

  PyObject* m;
#if PY_MAJOR_VERSION >= 3
  m = PyModule_Create(&moduledef);
#else
  m = Py_InitModule3("pybullet", SpamMethods, "Python bindings for Bullet");
#endif

#if PY_MAJOR_VERSION >= 3
  if (m == NULL) return m;
#else
  if (m == NULL) return;
#endif

  PyModule_AddIntConstant(m, "SHARED_MEMORY",
                          eCONNECT_SHARED_MEMORY);        // user read
  PyModule_AddIntConstant(m, "DIRECT", eCONNECT_DIRECT);  // user read
  PyModule_AddIntConstant(m, "GUI", eCONNECT_GUI);        // user read

  PyModule_AddIntConstant(m, "TORQUE_CONTROL", CONTROL_MODE_TORQUE);
  PyModule_AddIntConstant(m, "VELOCITY_CONTROL",
                          CONTROL_MODE_VELOCITY);  // user read
  PyModule_AddIntConstant(m, "POSITION_CONTROL",
                          CONTROL_MODE_POSITION_VELOCITY_PD);  // user read

  PyModule_AddIntConstant(m, "LINK_FRAME", EF_LINK_FRAME);
  PyModule_AddIntConstant(m, "WORLD_FRAME", EF_WORLD_FRAME);

  SpamError = PyErr_NewException("pybullet.error", NULL, NULL);
  Py_INCREF(SpamError);
  PyModule_AddObject(m, "error", SpamError);

#ifdef PYBULLET_USE_NUMPY
  // Initialize numpy array.
  import_array();
#endif //PYBULLET_USE_NUMPY


#if PY_MAJOR_VERSION >= 3
  return m;
#endif
}
