#include "../SharedMemory/PhysicsClientC_API.h"
#include "../SharedMemory/PhysicsDirectC_API.h"
#include "../SharedMemory/SharedMemoryInProcessPhysicsC_API.h"
#ifdef BT_ENABLE_ENET
#include "../SharedMemory/PhysicsClientUDP_C_API.h"
#endif //BT_ENABLE_ENET

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
  eCONNECT_UDP = 4,
};

static PyObject* SpamError;

#define MAX_PHYSICS_CLIENTS 1024
static b3PhysicsClientHandle sPhysicsClients1[MAX_PHYSICS_CLIENTS] = {0};
static int sPhysicsClientsGUI[MAX_PHYSICS_CLIENTS] = {0};
static int sNumPhysicsClients=0;

b3PhysicsClientHandle getPhysicsClient(int physicsClientId)
{
	b3PhysicsClientHandle sm;
	if ((physicsClientId  <0) || (physicsClientId>=MAX_PHYSICS_CLIENTS) || (0 == sPhysicsClients1[physicsClientId])) 
	{
		return 0;
	}
	sm = sPhysicsClients1[physicsClientId];
	if (sm)
	{
		if (b3CanSubmitCommand(sm))
		{
			return sm;
		}
		//broken connection?
		b3DisconnectSharedMemory(sm);
		sPhysicsClients1[physicsClientId] = 0;
		sPhysicsClientsGUI[physicsClientId] = 0;

		sNumPhysicsClients--;
	}
	return 0;

}

static double pybullet_internalGetFloatFromSequence(PyObject* seq, int index) {
	double v = 0.0;
	PyObject* item;

	if (PyList_Check(seq)) {
		item = PyList_GET_ITEM(seq, index);
		v = PyFloat_AsDouble(item);
	}
	else {
		item = PyTuple_GET_ITEM(seq, index);
		v = PyFloat_AsDouble(item);
	}
	return v;
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
	if (seq)
	{
		len = PySequence_Size(objMat);
		if (len == 16) {
			for (i = 0; i < len; i++) {
				matrix[i] = pybullet_internalGetFloatFromSequence(seq, i);
			}
			Py_DECREF(seq);
			return 1;
		}
		Py_DECREF(seq);
	}
	return 0;
}

// internal function to set a float vector[3]
// used to initialize camera position with
// a view and projection matrix in renderImage()
//
// // Args:
//  vector - float[3] which will be set by values from objMat
static int pybullet_internalSetVector(PyObject* objVec, float vector[3]) {
	int i, len;
	PyObject* seq = 0;
	if (objVec == NULL)
		return 0;

	seq = PySequence_Fast(objVec, "expected a sequence");
	if (seq)
	{

		len = PySequence_Size(objVec);
		if (len == 3) {
			for (i = 0; i < len; i++) {
				vector[i] = pybullet_internalGetFloatFromSequence(seq, i);
			}
			Py_DECREF(seq);
			return 1;
		}
		Py_DECREF(seq);
	}
	return 0;
}

//  vector - double[3] which will be set by values from obVec
static int pybullet_internalSetVectord(PyObject* obVec, double vector[3]) {
	int i, len;
	PyObject* seq;
	if (obVec == NULL)
		return 0;

	seq = PySequence_Fast(obVec, "expected a sequence");
	if (seq)
	{
		len = PySequence_Size(obVec);
		if (len == 3) {
			for (i = 0; i < len; i++) {
				vector[i] = pybullet_internalGetFloatFromSequence(seq, i);
			}
			Py_DECREF(seq);
			return 1;
		}
		Py_DECREF(seq);
	}
	return 0;
}

//  vector - double[3] which will be set by values from obVec
static int pybullet_internalSetVector4d(PyObject* obVec, double vector[4]) {
	int i, len;
	PyObject* seq;
	if (obVec == NULL)
		return 0;

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



// Step through one timestep of the simulation
static PyObject* pybullet_stepSimulation(PyObject* self, PyObject* args, PyObject *keywds) 
{
	int physicsClientId = 0;
	static char *kwlist[] = { "physicsClientId", NULL };
	b3PhysicsClientHandle sm=0;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist,&physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
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

static PyObject* pybullet_connectPhysicsServer(PyObject* self, PyObject* args, PyObject* keywds) {

	
  int freeIndex = -1;
  int method = eCONNECT_GUI;
  int i;
	b3PhysicsClientHandle sm=0;

		if (sNumPhysicsClients>=MAX_PHYSICS_CLIENTS)
	{
		    PyErr_SetString(SpamError,
                    "Exceeding maximum number of physics connections.");
	   return NULL;
	}


  {
   
	int key = SHARED_MEMORY_KEY;
	int port = 1234;
	const char* hostName = "localhost";

	int size = PySequence_Size(args);
	if (size == 1)
	{
		if (!PyArg_ParseTuple(args, "i", &method)) {
			PyErr_SetString(SpamError,
				"connectPhysicsServer expected argument  GUI, "
				"DIRECT, SHARED_MEMORY or UDP");
			return NULL;
		}
	}

	//Only one local in-process GUI connection allowed.
	if (method == eCONNECT_GUI)
	{
		int i;
		for (i=0;i<MAX_PHYSICS_CLIENTS;i++)
		{
			if (sPhysicsClientsGUI[i] ==eCONNECT_GUI)
			{
				PyErr_SetString(SpamError,
					"Only one local in-process GUI connection allowed. Use DIRECT connection mode or start a separate GUI physics server (ExampleBrowser, App_SharedMemoryPhysics_GUI, App_SharedMemoryPhysics_VR) and connect over SHARED_MEMORY or UDP instead.");
				return NULL;
			}
		}
	}
	
	if (size == 2)
	{
		if (!PyArg_ParseTuple(args, "ii", &method, &key)) 
		{
			if (!PyArg_ParseTuple(args, "is", &method, &hostName))
			{
				PyErr_SetString(SpamError,
					"connectPhysicsServer cannot parse second argument (either integer or string)");
				return NULL;

			} else
			{
				PyErr_Clear();
			}
		}
	}

	if (size == 3)
	{
		if (!PyArg_ParseTuple(args, "isi", &method, &hostName, &port))
		{
			PyErr_SetString(SpamError,
				"connectPhysicsServer 3 arguments: method, hostname, port");
			return NULL;
		}
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
        sm = b3ConnectSharedMemory(key);
        break;
      }
	  case eCONNECT_UDP: 
	{
#ifdef BT_ENABLE_ENET

		  sm = b3ConnectPhysicsUDP(hostName, port);
#else
 		PyErr_SetString(SpamError, "UDP is not enabled in this pybullet build");
		return NULL;
#endif //BT_ENABLE_ENET

		  break;
	}

      default: {
        PyErr_SetString(SpamError, "connectPhysicsServer unexpected argument");
        return NULL;
      }
    };
  }

  if (sm && b3CanSubmitCommand(sm))
  {
	  for (i=0;i<MAX_PHYSICS_CLIENTS;i++)
	  {
		  if (sPhysicsClients1[i]==0)
		  {
			  freeIndex = i;
			  break;
		  }
	  }

	  if (freeIndex>=0)
	  {
		  	b3SharedMemoryCommandHandle	command;
			b3SharedMemoryStatusHandle statusHandle;
			int statusType;


		  sPhysicsClients1[freeIndex] = sm;
		  sPhysicsClientsGUI[freeIndex] = method;
		  sNumPhysicsClients++;

		  command = b3InitSyncBodyInfoCommand(sm);
		  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
			statusType = b3GetStatusType(statusHandle);
#if 0
			if (statusType != CMD_BODY_INFO_COMPLETED) {
			PyErr_SetString(SpamError, "b3InitSyncBodyInfoCommand failed.");
			return NULL;
		}
#endif
	  }

  }
  return PyInt_FromLong(freeIndex);
}

static PyObject* pybullet_disconnectPhysicsServer(PyObject* self,
                                                  PyObject* args,
													PyObject *keywds) {

	int physicsClientId = 0;
	b3PhysicsClientHandle sm=0;
	static char *kwlist[] = { "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist,&physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	

  {
    b3DisconnectSharedMemory(sm);
    sm = 0;
  }

  sPhysicsClients1[physicsClientId] = 0;
  sPhysicsClientsGUI[physicsClientId] = 0;
  sNumPhysicsClients--;

  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject* pybullet_saveWorld(PyObject* self, PyObject* args,PyObject *keywds) {
	const char* worldFileName = "";

	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	static char *kwlist[] = { "worldFileName","physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|i", kwlist, &worldFileName,&physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
   

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

     
	PyErr_SetString(SpamError, "Cannot execute saveWorld command.");
	return NULL;
    
}

#define MAX_SDF_BODIES 512
static PyObject* pybullet_loadBullet(PyObject* self, PyObject* args,PyObject *keywds)
{
	const char* bulletFileName = "";
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;
	int i,numBodies;
	int bodyIndicesOut[MAX_SDF_BODIES];
	PyObject* pylist = 0;
	b3PhysicsClientHandle sm = 0;

	  	int physicsClientId = 0;
	static char *kwlist[] = { "bulletFileName","physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|i", kwlist, &bulletFileName,&physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	

	command = b3LoadBulletCommandInit(sm, bulletFileName);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_BULLET_LOADING_COMPLETED)
	{
		PyErr_SetString(SpamError, "Couldn't load .bullet file.");
		return NULL;
	}

	numBodies =
      b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES);
  if (numBodies > MAX_SDF_BODIES) {
    PyErr_SetString(SpamError, "loadBullet exceeds body capacity");
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

static PyObject* pybullet_saveBullet(PyObject* self, PyObject* args, PyObject* keywds)
{
	const char* bulletFileName = "";
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;
	b3PhysicsClientHandle sm = 0;

	  int physicsClientId = 0;
	static char *kwlist[] = { "bulletFileName","physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|i", kwlist, &bulletFileName,&physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	
	command = b3SaveBulletCommandInit(sm, bulletFileName);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_BULLET_SAVING_COMPLETED)
	{
		PyErr_SetString(SpamError, "Couldn't save .bullet file.");
		return NULL;
	}
	Py_INCREF(Py_None);
	return Py_None;
}



static PyObject* pybullet_loadMJCF(PyObject* self, PyObject* args, PyObject* keywds)
{
	const char* mjcfFileName = "";
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;
	b3PhysicsClientHandle sm = 0;
	int numBodies = 0;
	int i;
	int bodyIndicesOut[MAX_SDF_BODIES];
	PyObject* pylist = 0;
	int physicsClientId = 0;
	static char *kwlist[] = { "mjcfFileName","physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|i", kwlist, &mjcfFileName,&physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}


	command = b3LoadMJCFCommandInit(sm, mjcfFileName);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_MJCF_LOADING_COMPLETED)
	{
		PyErr_SetString(SpamError, "Couldn't load .mjcf file.");
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

static PyObject* pybullet_setPhysicsEngineParameter(PyObject* self, PyObject* args, PyObject *keywds)
{
	double fixedTimeStep = -1;
	int numSolverIterations = -1;
	int useSplitImpulse = -1;
	double splitImpulsePenetrationThreshold = -1;
	int numSubSteps = -1;
	int collisionFilterMode = -1;
	double contactBreakingThreshold = -1;
	
	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char *kwlist[] = { "fixedTimeStep", "numSolverIterations","useSplitImpulse","splitImpulsePenetrationThreshold", "numSubSteps","collisionFilterMode", "contactBreakingThreshold", "physicsClientId", NULL };

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|diidiidi", kwlist,&fixedTimeStep,&numSolverIterations,&useSplitImpulse,&splitImpulsePenetrationThreshold,&numSubSteps,
									 &collisionFilterMode, &contactBreakingThreshold, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
		b3SharedMemoryStatusHandle statusHandle;

		if (numSolverIterations >= 0)
		{
			b3PhysicsParamSetNumSolverIterations(command, numSolverIterations);
		}

		if (collisionFilterMode>=0)
		{
			b3PhysicsParamSetCollisionFilterMode(command, collisionFilterMode);
		}
		if (numSubSteps >= 0)
		{
			b3PhysicsParamSetNumSubSteps(command, numSubSteps);
		}
		if (fixedTimeStep >= 0)
		{
			b3PhysicsParamSetTimeStep(command, fixedTimeStep);
		}
		if (useSplitImpulse >= 0)
		{
			b3PhysicsParamSetUseSplitImpulse(command,useSplitImpulse);
		}
		if (splitImpulsePenetrationThreshold >= 0)
		{
			b3PhysicsParamSetSplitImpulsePenetrationThreshold(command, splitImpulsePenetrationThreshold);
		}
		if (contactBreakingThreshold>=0)
		{
			b3PhysicsParamSetContactBreakingThreshold(command,contactBreakingThreshold);
		}


		//ret = b3PhysicsParamSetRealTimeSimulation(command, enableRealTimeSimulation);

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	}
#if 0
	b3SharedMemoryCommandHandle	b3InitPhysicsParamCommand(b3PhysicsClientHandle physClient);
	int	b3PhysicsParamSetGravity(b3SharedMemoryCommandHandle commandHandle, double gravx, double gravy, double gravz);
	int	b3PhysicsParamSetTimeStep(b3SharedMemoryCommandHandle commandHandle, double timeStep);
	int	b3PhysicsParamSetDefaultContactERP(b3SharedMemoryCommandHandle commandHandle, double defaultContactERP);
	int	b3PhysicsParamSetNumSubSteps(b3SharedMemoryCommandHandle commandHandle, int numSubSteps);
	int b3PhysicsParamSetRealTimeSimulation(b3SharedMemoryCommandHandle commandHandle, int enableRealTimeSimulation);
	int b3PhysicsParamSetNumSolverIterations(b3SharedMemoryCommandHandle commandHandle, int numSolverIterations);
#endif

	Py_INCREF(Py_None);
	return Py_None;
}


// Load a robot from a URDF file (universal robot description format)
// function can be called without arguments and will default
// to position (0,0,1) with orientation(0,0,0,1)
// els(x,y,z) or
// loadURDF(pos_x, pos_y, pos_z, orn_x, orn_y, orn_z, orn_w)
static PyObject* pybullet_loadURDF(PyObject* self, PyObject* args, PyObject *keywds) 
{
  int physicsClientId = 0;
	
  static char *kwlist[] = { "fileName", "basePosition", "baseOrientation", "useMaximalCoordinates","useFixedBase","physicsClientId", NULL };

  static char *kwlistBackwardCompatible4[] = { "fileName", "startPosX", "startPosY", "startPosZ", NULL };
  static char *kwlistBackwardCompatible8[] = { "fileName", "startPosX", "startPosY", "startPosZ", "startOrnX", "startOrnY","startOrnZ","startOrnW", NULL };


  int bodyIndex = -1;
  const char* urdfFileName = "";

  double startPosX = 0.0;
  double startPosY = 0.0;
  double startPosZ = 0.0;
  double startOrnX = 0.0;
  double startOrnY = 0.0;
  double startOrnZ = 0.0;
  double startOrnW = 1.0;
  int useMaximalCoordinates = 0;
  int useFixedBase = 0;

  int backwardsCompatibilityArgs = 0;
b3PhysicsClientHandle sm=0;
  if (PyArg_ParseTupleAndKeywords(args, keywds, "sddd", kwlistBackwardCompatible4, &urdfFileName, &startPosX,
	  &startPosY, &startPosZ))
  {
	  backwardsCompatibilityArgs = 1;
  }
  else
  {
	  PyErr_Clear();
  }


  if (PyArg_ParseTupleAndKeywords(args, keywds, "sddddddd", kwlistBackwardCompatible8,&urdfFileName, &startPosX,
	  &startPosY, &startPosZ, &startOrnX, &startOrnY,&startOrnZ, &startOrnW))
  {
	  backwardsCompatibilityArgs = 1;
  }
  else
  {
	  PyErr_Clear();
  }

  

  if (!backwardsCompatibilityArgs)
  {
	  PyObject* basePosObj = 0;
	  PyObject* baseOrnObj = 0;
	  double basePos[3];
	  double baseOrn[4];
	

	  if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|OOiii", kwlist, &urdfFileName, &basePosObj, &baseOrnObj, &useMaximalCoordinates,&useFixedBase,&physicsClientId))
	  {

		  return NULL;
	  }
	  else
	  {
		  if (basePosObj)
		  {
			  if (!pybullet_internalSetVectord(basePosObj, basePos))
			  {
				  PyErr_SetString(SpamError, "Cannot convert basePosition.");
				  return NULL;
			  }
			  startPosX = basePos[0];
			  startPosY = basePos[1];
			  startPosZ = basePos[2];

		  }
		  if (baseOrnObj)
		  {
			  if (!pybullet_internalSetVector4d(baseOrnObj, baseOrn))
			  {
				  PyErr_SetString(SpamError, "Cannot convert baseOrientation.");
				  return NULL;
			  }
			  startOrnX = baseOrn[0];
			  startOrnY = baseOrn[1];
			  startOrnZ = baseOrn[2];
			  startOrnW = baseOrn[3];
		  }
	  }
  }

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
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
	if (useMaximalCoordinates)
	{
		b3LoadUrdfCommandSetUseMultiBody(command, 0);
	}
	if (useFixedBase)
	{
		b3LoadUrdfCommandSetUseFixedBase(command, 1);
	}

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




static PyObject* pybullet_loadSDF(PyObject* self, PyObject* args, PyObject *keywds) {
  const char* sdfFileName = "";
  int numBodies = 0;
  int i;
  int bodyIndicesOut[MAX_SDF_BODIES];
  PyObject* pylist = 0;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  b3SharedMemoryCommandHandle commandHandle;
b3PhysicsClientHandle sm = 0;

    int physicsClientId = 0;
	static char *kwlist[] = { "sdfFileName","physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|i", kwlist, &sdfFileName,&physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
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
static PyObject* pybullet_resetSimulation(PyObject* self, PyObject* args, PyObject* keywds) {

  	int physicsClientId = 0;
	static char *kwlist[] = { "physicsClientId", NULL };
	b3PhysicsClientHandle sm = 0;
	
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist,&physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
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
//this method is obsolete, use pybullet_setJointMotorControl2 instead
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
  int physicsClientId = 0;
  b3PhysicsClientHandle sm;
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
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


static PyObject* pybullet_setJointMotorControl2(PyObject* self, PyObject* args, PyObject* keywds) 
{
  int bodyIndex, jointIndex, controlMode;

  double targetPosition = 0.0;
  double targetVelocity = 0.0;
  double force = 100000.0;
  double kp = 0.1;
  double kd = 1.0;
b3PhysicsClientHandle sm = 0;

  int physicsClientId = 0;
	static char *kwlist[] = { "bodyIndex", "jointIndex", "controlMode", "targetPosition", "targetVelocity"
								, "force", "positionGain", "velocityGain", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iii|dddddi", kwlist,&bodyIndex, &jointIndex, &controlMode,
                          &targetPosition, &targetVelocity,&force, &kp, &kd, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

  {
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
        b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
        break;
      }

      case CONTROL_MODE_TORQUE: {
        b3JointControlSetDesiredForceTorque(commandHandle, info.m_uIndex,
                                            force);
        break;
      }

      case CONTROL_MODE_POSITION_VELOCITY_PD: {
        b3JointControlSetDesiredPosition(commandHandle, info.m_qIndex,
                                         targetPosition);
        b3JointControlSetKp(commandHandle, info.m_uIndex, kp);
        b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
                                         targetVelocity);
        b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
        b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
        break;
      }
      default: {}
    };

    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);

    Py_INCREF(Py_None);
    return Py_None;
  }
//  PyErr_SetString(SpamError, "Error parsing arguments in setJointControl.");
//  return NULL;
}

static PyObject* pybullet_setRealTimeSimulation(PyObject* self,
                                                PyObject* args,
												PyObject* keywds) {
 
    int enableRealTimeSimulation = 0;
    int ret;
	b3PhysicsClientHandle sm = 0;

	 int physicsClientId = 0;
	static char *kwlist[] = { "enableRealTimeSimulation", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,&enableRealTimeSimulation, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	

  {
 

    b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
    b3SharedMemoryStatusHandle statusHandle;

    ret =
        b3PhysicsParamSetRealTimeSimulation(command, enableRealTimeSimulation);

    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
    // ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
  }

  Py_INCREF(Py_None);
  return Py_None;
}



static PyObject* pybullet_setInternalSimFlags(PyObject* self,
                                                PyObject* args, PyObject* keywds) {
 
  int flags = 0;
  int ret;
   int physicsClientId = 0;
b3PhysicsClientHandle sm = 0;

	static char *kwlist[] = { "flags", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,&flags, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	

  {
   
    b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
    b3SharedMemoryStatusHandle statusHandle;

    ret =
        b3PhysicsParamSetInternalSimFlags(command, flags);

    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
    // ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
  }

  Py_INCREF(Py_None);
  return Py_None;
}

// Set the gravity of the world with (x, y, z) arguments
static PyObject* pybullet_setGravity(PyObject* self, PyObject* args, PyObject* keywds) {
 
  {
    double gravX = 0.0;
    double gravY = 0.0;
    double gravZ = -10.0;
    int ret;
	b3PhysicsClientHandle sm  = 0;
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;

	int physicsClientId = 0;
	static char *kwlist[] = { "gravX", "gravY", "gravZ", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ddd|i", kwlist,&gravX, &gravY, &gravZ, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	

    command = b3InitPhysicsParamCommand(sm);
    
    ret = b3PhysicsParamSetGravity(command, gravX, gravY, gravZ);
    // ret = b3PhysicsParamSetTimeStep(command,  timeStep);
    statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
    // ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
  }

  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject* pybullet_setTimeStep(PyObject* self, PyObject* args, PyObject* keywds) 
{
	double timeStep = 0.001;
	int ret;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	static char *kwlist[] = { "timeStep", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "d|i", kwlist,&timeStep, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	{

	b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
	b3SharedMemoryStatusHandle statusHandle;

	ret = b3PhysicsParamSetTimeStep(command, timeStep);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	

	Py_INCREF(Py_None);
	return Py_None;
}
}

static PyObject *
pybullet_setDefaultContactERP(PyObject* self, PyObject* args,PyObject* keywds)
{  
	double defaultContactERP=0.005;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	static char *kwlist[] = { "defaultContactERP", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "d|i", kwlist,&defaultContactERP, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	{
	    int ret;
       
        b3SharedMemoryStatusHandle statusHandle;
        
		b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
        ret = b3PhysicsParamSetDefaultContactERP(command,  defaultContactERP);

        statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
    }
    
    Py_INCREF(Py_None);
    return Py_None;
}

static int pybullet_internalGetBaseVelocity(
	int bodyIndex, double baseLinearVelocity[3], double baseAngularVelocity[3],b3PhysicsClientHandle sm) {
	baseLinearVelocity[0] = 0.;
	baseLinearVelocity[1] = 0.;
	baseLinearVelocity[2] = 0.;

	baseAngularVelocity[0] = 0.;
	baseAngularVelocity[1] = 0.;
	baseAngularVelocity[2] = 0.;

	if (0 == sm) {
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return 0;
	}

	{
		{
			b3SharedMemoryCommandHandle cmd_handle =
				b3RequestActualStateCommandInit(sm, bodyIndex);
			b3SharedMemoryStatusHandle status_handle =
				b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

			const int status_type = b3GetStatusType(status_handle);
			const double* actualStateQdot;
			// const double* jointReactionForces[];


			if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED) {
				PyErr_SetString(SpamError, "getBaseVelocity failed.");
				return 0;
			}

			b3GetStatusActualState(
				status_handle, 0 /* body_unique_id */,
				0 /* num_degree_of_freedom_q */, 0 /* num_degree_of_freedom_u */,
				0 /*root_local_inertial_frame*/, 0,
				&actualStateQdot, 0 /* joint_reaction_forces */);

			// printf("joint reaction forces=");
			// for (i=0; i < (sizeof(jointReactionForces)/sizeof(double)); i++) {
			//   printf("%f ", jointReactionForces[i]);
			// }
			// now, position x,y,z = actualStateQ[0],actualStateQ[1],actualStateQ[2]
			// and orientation x,y,z,w =
			// actualStateQ[3],actualStateQ[4],actualStateQ[5],actualStateQ[6]
			baseLinearVelocity[0] = actualStateQdot[0];
			baseLinearVelocity[1] = actualStateQdot[1];
			baseLinearVelocity[2] = actualStateQdot[2];

			baseAngularVelocity[0] = actualStateQdot[3];
			baseAngularVelocity[1] = actualStateQdot[4];
			baseAngularVelocity[2] = actualStateQdot[5];
			
		}
	}
	return 1;
}

// Internal function used to get the base position and orientation
// Orientation is returned in quaternions
static int pybullet_internalGetBasePositionAndOrientation(
    int bodyIndex, double basePosition[3], double baseOrientation[4],b3PhysicsClientHandle sm) {
  basePosition[0] = 0.;
  basePosition[1] = 0.;
  basePosition[2] = 0.;

  baseOrientation[0] = 0.;
  baseOrientation[1] = 0.;
  baseOrientation[2] = 0.;
  baseOrientation[3] = 1.;

  if (0 == sm) {
	  PyErr_SetString(SpamError, "Not connected to physics server.");
	  return 0;
  }

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
                                                        PyObject* args, PyObject* keywds) {
  int bodyUniqueId = -1;
  double basePosition[3];
  double baseOrientation[4];
  PyObject* pylistPos;
  PyObject* pylistOrientation;
	b3PhysicsClientHandle sm  = 0;

  int physicsClientId = 0;
	static char *kwlist[] = { "bodyUniqueId", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,&bodyUniqueId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	
  if (0 == pybullet_internalGetBasePositionAndOrientation(
               bodyUniqueId, basePosition, baseOrientation,sm)) {
    PyErr_SetString(SpamError,
                    "GetBasePositionAndOrientation failed.");
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


static PyObject* pybullet_getBaseVelocity(PyObject* self,
	PyObject* args, PyObject* keywds) {
	int bodyUniqueId = -1;
	double baseLinearVelocity[3];
	double baseAngularVelocity[3];
	PyObject* pylistLinVel=0;
	PyObject* pylistAngVel=0;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	static char *kwlist[] = { "bodyUniqueId", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,&bodyUniqueId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	
	if (0 == pybullet_internalGetBaseVelocity(
		bodyUniqueId, baseLinearVelocity, baseAngularVelocity,sm)) {
		PyErr_SetString(SpamError,
			"getBaseVelocity failed.");
		return NULL;
	}

	{
		PyObject* item;
		int i;
		int num = 3;
		pylistLinVel = PyTuple_New(num);
		for (i = 0; i < num; i++) {
			item = PyFloat_FromDouble(baseLinearVelocity[i]);
			PyTuple_SetItem(pylistLinVel, i, item);
		}
	}

	{
		PyObject* item;
		int i;
		int num = 3;
		pylistAngVel = PyTuple_New(num);
		for (i = 0; i < num; i++) {
			item = PyFloat_FromDouble(baseAngularVelocity[i]);
			PyTuple_SetItem(pylistAngVel, i, item);
		}
	}

	{
		PyObject* pylist;
		pylist = PyTuple_New(2);
		PyTuple_SetItem(pylist, 0, pylistLinVel);
		PyTuple_SetItem(pylist, 1, pylistAngVel);
		return pylist;
	}
}
static PyObject* pybullet_getNumBodies(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	static char *kwlist[] = { "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist,&physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
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


static PyObject* pybullet_getBodyUniqueId(PyObject* self, PyObject* args, PyObject* keywds)
{
int physicsClientId = 0;
int serialIndex=-1;
b3PhysicsClientHandle sm  = 0;

	static char *kwlist[] = { "serialIndex", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,&serialIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	

	{

    int bodyUniqueId = -1;
    bodyUniqueId = b3GetBodyUniqueId(sm, serialIndex);

#if PY_MAJOR_VERSION >= 3
    return PyLong_FromLong(bodyUniqueId);
#else
    return PyInt_FromLong(bodyUniqueId);
#endif
  }
}

static PyObject* pybullet_getBodyInfo(PyObject* self, PyObject* args, PyObject* keywds)
{
	
	{
    int bodyUniqueId= -1;
 	b3PhysicsClientHandle sm = 0;

    	int physicsClientId = 0;
	static char *kwlist[] = { "bodyUniqueId", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,&bodyUniqueId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
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


static PyObject* pybullet_getConstraintInfo(PyObject* self, PyObject* args, PyObject* keywds)
{
    
    {
        int constraintUniqueId= -1;
        b3PhysicsClientHandle sm = 0;
        
        int physicsClientId = 0;
        static char *kwlist[] = { "constraintUniqueId", "physicsClientId", NULL };
        if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,&constraintUniqueId, &physicsClientId))
        {
            return NULL;
        }
        sm = getPhysicsClient(physicsClientId);
        if (sm == 0)
        {
            PyErr_SetString(SpamError, "Not connected to physics server.");
            return NULL;
        }
        
        {
            struct b3UserConstraint constraintInfo;
            
            if (b3GetUserConstraintInfo(sm,constraintUniqueId, &constraintInfo))
            {
                PyObject* pyListConstraintInfo = PyTuple_New(11);

				PyTuple_SetItem(pyListConstraintInfo,0,PyLong_FromLong(constraintInfo.m_parentBodyIndex));
                PyTuple_SetItem(pyListConstraintInfo,1,PyLong_FromLong(constraintInfo.m_parentJointIndex));
                PyTuple_SetItem(pyListConstraintInfo,2,PyLong_FromLong(constraintInfo.m_childBodyIndex));
                PyTuple_SetItem(pyListConstraintInfo,3,PyLong_FromLong(constraintInfo.m_childJointIndex));
                PyTuple_SetItem(pyListConstraintInfo,4,PyLong_FromLong(constraintInfo.m_jointType));
                
                {
                    PyObject* axisObj = PyTuple_New(3);
                    PyTuple_SetItem(axisObj,0,PyFloat_FromDouble(constraintInfo.m_jointAxis[0]));
                    PyTuple_SetItem(axisObj,1,PyFloat_FromDouble(constraintInfo.m_jointAxis[1]));
                    PyTuple_SetItem(axisObj,2,PyFloat_FromDouble(constraintInfo.m_jointAxis[2]));
                    PyTuple_SetItem(pyListConstraintInfo,5,axisObj);
                }
                {
                    PyObject* parentFramePositionObj = PyTuple_New(3);
                    PyTuple_SetItem(parentFramePositionObj,0,PyFloat_FromDouble(constraintInfo.m_parentFrame[0]));
                    PyTuple_SetItem(parentFramePositionObj,1,PyFloat_FromDouble(constraintInfo.m_parentFrame[1]));
                    PyTuple_SetItem(parentFramePositionObj,2,PyFloat_FromDouble(constraintInfo.m_parentFrame[2]));
                    PyTuple_SetItem(pyListConstraintInfo,6,parentFramePositionObj);
                }
                {
                    PyObject* childFramePositionObj = PyTuple_New(3);
                    PyTuple_SetItem(childFramePositionObj,0,PyFloat_FromDouble(constraintInfo.m_childFrame[0]));
                    PyTuple_SetItem(childFramePositionObj,1,PyFloat_FromDouble(constraintInfo.m_childFrame[1]));
                    PyTuple_SetItem(childFramePositionObj,2,PyFloat_FromDouble(constraintInfo.m_childFrame[2]));
                    PyTuple_SetItem(pyListConstraintInfo,7,childFramePositionObj);
                }
				{
                    PyObject* parentFrameOrientationObj = PyTuple_New(4);                
					PyTuple_SetItem(parentFrameOrientationObj,0,PyFloat_FromDouble(constraintInfo.m_parentFrame[3]));
					PyTuple_SetItem(parentFrameOrientationObj,1,PyFloat_FromDouble(constraintInfo.m_parentFrame[4]));
					PyTuple_SetItem(parentFrameOrientationObj,2,PyFloat_FromDouble(constraintInfo.m_parentFrame[5]));
					PyTuple_SetItem(parentFrameOrientationObj,3,PyFloat_FromDouble(constraintInfo.m_parentFrame[6]));
                    PyTuple_SetItem(pyListConstraintInfo,8,parentFrameOrientationObj);
				}
								{
                    PyObject* childFrameOrientation = PyTuple_New(4);                
					PyTuple_SetItem(childFrameOrientation,0,PyFloat_FromDouble(constraintInfo.m_childFrame[3]));
					PyTuple_SetItem(childFrameOrientation,1,PyFloat_FromDouble(constraintInfo.m_childFrame[4]));
					PyTuple_SetItem(childFrameOrientation,2,PyFloat_FromDouble(constraintInfo.m_childFrame[5]));
					PyTuple_SetItem(childFrameOrientation,3,PyFloat_FromDouble(constraintInfo.m_childFrame[6]));
                    PyTuple_SetItem(pyListConstraintInfo,9,childFrameOrientation);
				}
				PyTuple_SetItem(pyListConstraintInfo,10,PyFloat_FromDouble(constraintInfo.m_maxAppliedForce));

                return pyListConstraintInfo;
            } else
            {
                PyErr_SetString(SpamError, "Couldn't get user constraint info");
                return NULL;
            }
        }
    }
    
    PyErr_SetString(SpamError, "error in getConstraintInfo.");
    return NULL;
}


static PyObject* pybullet_getNumConstraints(PyObject* self, PyObject* args, PyObject* keywds)
{
    int numConstraints = 0;
    int physicsClientId = 0;
    b3PhysicsClientHandle sm = 0;
    static char *kwlist[] = { "physicsClientId", NULL };
    if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
    {
        return NULL;
    }
    sm = getPhysicsClient(physicsClientId);
    if (sm == 0)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }
    
    numConstraints = b3GetNumUserConstraints(sm);
    
#if PY_MAJOR_VERSION >= 3
    return PyLong_FromLong(numConstraints);
#else
    return PyInt_FromLong(numConstraints);
#endif
    

}



// Return the number of joints in an object based on
// body index; body index is based on order of sequence
// the object is loaded into simulation
static PyObject* pybullet_getNumJoints(PyObject* self, PyObject* args, PyObject* keywds) 
{
	int bodyUniqueId = -1;
    int numJoints = 0;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char *kwlist[] = { "bodyUniqueId", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,&bodyUniqueId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
  
    numJoints = b3GetNumJoints(sm, bodyUniqueId);

#if PY_MAJOR_VERSION >= 3
    return PyLong_FromLong(numJoints);
#else
    return PyInt_FromLong(numJoints);
#endif
 
}

// Initalize all joint positions given a list of values
static PyObject* pybullet_resetJointState(PyObject* self, PyObject* args, PyObject* keywds) {
 
{
    int bodyUniqueId;
    int jointIndex;
    double targetValue;
	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char *kwlist[] = { "bodyUniqueId", "jointIndex", "targetValue","physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iid|i", kwlist,&bodyUniqueId, &jointIndex, &targetValue, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	

    {
      b3SharedMemoryCommandHandle commandHandle;
      b3SharedMemoryStatusHandle statusHandle;
      int numJoints;

      numJoints = b3GetNumJoints(sm, bodyUniqueId);
      if ((jointIndex >= numJoints) || (jointIndex < 0)) {
        PyErr_SetString(SpamError, "Joint index out-of-range.");
        return NULL;
      }

      commandHandle = b3CreatePoseCommandInit(sm, bodyUniqueId);

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




static PyObject* pybullet_resetBaseVelocity(PyObject* self, PyObject* args, PyObject *keywds)
{
	static char *kwlist[] = { "objectUniqueId", "linearVelocity", "angularVelocity","physicsClientId",  NULL };

	
	{
		int bodyIndex=0;
		PyObject* linVelObj=0;
		PyObject* angVelObj=0;
		double linVel[3] = { 0, 0, 0 };
		double angVel[3] = { 0, 0, 0 };
		int  physicsClientId = 0;
		b3PhysicsClientHandle sm = 0;

		if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|OOi", kwlist, &bodyIndex, &linVelObj, &angVelObj,&physicsClientId))
		{
			return NULL;
		}

		sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}


		if (linVelObj || angVelObj)
		{

			b3SharedMemoryCommandHandle commandHandle;
			b3SharedMemoryStatusHandle statusHandle;

			commandHandle = b3CreatePoseCommandInit(sm, bodyIndex);

			if (linVelObj)
			{
				pybullet_internalSetVectord(linVelObj, linVel);
				b3CreatePoseCommandSetBaseLinearVelocity(commandHandle, linVel);
			}

			if (angVelObj)
			{
				pybullet_internalSetVectord(angVelObj, angVel);
				b3CreatePoseCommandSetBaseAngularVelocity(commandHandle, angVel);
			}

			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
			Py_INCREF(Py_None);
			return Py_None;
		}
		else
		{
			PyErr_SetString(SpamError, "expected at least linearVelocity and/or angularVelocity.");
			return NULL;
		}
	}
	PyErr_SetString(SpamError, "error in resetJointState.");
	return NULL;
}



// Reset the position and orientation of the base/root link, position [x,y,z]
// and orientation quaternion [x,y,z,w]
static PyObject* pybullet_resetBasePositionAndOrientation(PyObject* self,
                                                          PyObject* args, PyObject* keywds) {

  {
    int bodyUniqueId;
    PyObject* posObj;
    PyObject* ornObj;
    double pos[3];
    double orn[4];  // as a quaternion
	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char *kwlist[] = { "bodyUniqueId", "posObj", "ornObj", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iOO|i", kwlist,&bodyUniqueId, &posObj, &ornObj, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	
	{
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

      commandHandle = b3CreatePoseCommandInit(sm, bodyUniqueId);

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
static PyObject* pybullet_getJointInfo(PyObject* self, PyObject* args,PyObject* keywds) {
  PyObject* pyListJointInfo;

  struct b3JointInfo info;

  int bodyUniqueId = -1;
  int jointIndex = -1;
  int jointInfoSize = 8;  // size of struct b3JointInfo
b3PhysicsClientHandle sm = 0;
int physicsClientId = 0;
	static char *kwlist[] = { "bodyUniqueId", "jointIndex", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|i", kwlist,&bodyUniqueId, &jointIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	

  {
	{
      // printf("body index = %d, joint index =%d\n", bodyIndex, jointIndex);

      pyListJointInfo = PyTuple_New(jointInfoSize);

      if (b3GetJointInfo(sm, bodyUniqueId, jointIndex, &info)) {
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

static PyObject* pybullet_getJointState(PyObject* self, PyObject* args,PyObject* keywds) {
  PyObject* pyListJointForceTorque;
  PyObject* pyListJointState;


  struct b3JointSensorState sensorState;

  int bodyUniqueId = -1;
  int jointIndex = -1;
  int sensorStateSize = 4;  // size of struct b3JointSensorState
  int forceTorqueSize = 6;  // size of force torque list from b3JointSensorState
  int j;

	b3PhysicsClientHandle sm = 0;
  int physicsClientId = 0;
	static char *kwlist[] = { "bodyUniqueId", "jointIndex","physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|i", kwlist,&bodyUniqueId, &jointIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	

  {
  
	{
	  int status_type = 0;
	  b3SharedMemoryCommandHandle cmd_handle;
	  b3SharedMemoryStatusHandle status_handle;


      if (bodyUniqueId < 0) {
        PyErr_SetString(SpamError, "getJointState failed; invalid bodyIndex");
        return NULL;
      }
      if (jointIndex < 0) {
        PyErr_SetString(SpamError, "getJointState failed; invalid jointIndex");
        return NULL;
      }

      
      cmd_handle =
          b3RequestActualStateCommandInit(sm, bodyUniqueId);
      status_handle =
          b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

      status_type = b3GetStatusType(status_handle);
      if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED) {
        PyErr_SetString(SpamError, "getJointState failed.");
        return NULL;
      }

      pyListJointState = PyTuple_New(sensorStateSize);
      pyListJointForceTorque = PyTuple_New(forceTorqueSize);

      if (b3GetJointState(sm, status_handle, jointIndex, &sensorState))
	  {
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
	  } else
	  {
		PyErr_SetString(SpamError, "getJointState failed (2).");
		return NULL;
	  }
    }
  } 

  Py_INCREF(Py_None);
  return Py_None;
}

static PyObject* pybullet_getLinkState(PyObject* self, PyObject* args,PyObject* keywds) {
  PyObject* pyLinkState;
  PyObject* pyLinkStateWorldPosition;
  PyObject* pyLinkStateWorldOrientation;
  PyObject* pyLinkStateLocalInertialPosition;
  PyObject* pyLinkStateLocalInertialOrientation;
  PyObject* pyLinkStateWorldLinkFramePosition;
  PyObject* pyLinkStateWorldLinkFrameOrientation;

  struct b3LinkState linkState;

  int bodyUniqueId = -1;
  int linkIndex = -1;
  int i;
b3PhysicsClientHandle sm = 0;

  int physicsClientId = 0;
	static char *kwlist[] = { "bodyUniqueId", "linkIndex", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|i", kwlist, &bodyUniqueId, &linkIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	
 {
	{
	int status_type = 0;
	b3SharedMemoryCommandHandle cmd_handle;
	b3SharedMemoryStatusHandle status_handle;

      if (bodyUniqueId < 0) {
        PyErr_SetString(SpamError, "getLinkState failed; invalid bodyIndex");
        return NULL;
      }
      if (linkIndex < 0) {
        PyErr_SetString(SpamError, "getLinkState failed; invalid linkIndex");
        return NULL;
      }


      cmd_handle =
          b3RequestActualStateCommandInit(sm, bodyUniqueId);
      status_handle =
          b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

      status_type = b3GetStatusType(status_handle);
      if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED) {
        PyErr_SetString(SpamError, "getLinkState failed.");
        return NULL;
      }

      if (b3GetLinkState(sm, status_handle, linkIndex, &linkState))
	  {
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

		pyLinkStateWorldLinkFramePosition = PyTuple_New(3);
		  for (i = 0; i < 3; ++i) {
			PyTuple_SetItem(pyLinkStateWorldLinkFramePosition , i,
							PyFloat_FromDouble(linkState.m_worldLinkFramePosition[i]));
		  }

		pyLinkStateWorldLinkFrameOrientation = PyTuple_New(4);
		  for (i = 0; i < 4; ++i) {
			PyTuple_SetItem(pyLinkStateWorldLinkFrameOrientation, i,
							PyFloat_FromDouble(linkState.m_worldLinkFrameOrientation[i]));
		  }


		  pyLinkState = PyTuple_New(6);
		  PyTuple_SetItem(pyLinkState, 0, pyLinkStateWorldPosition);
		  PyTuple_SetItem(pyLinkState, 1, pyLinkStateWorldOrientation);
		  PyTuple_SetItem(pyLinkState, 2, pyLinkStateLocalInertialPosition);
		  PyTuple_SetItem(pyLinkState, 3, pyLinkStateLocalInertialOrientation);
		  PyTuple_SetItem(pyLinkState, 4, pyLinkStateWorldLinkFramePosition );
		  PyTuple_SetItem(pyLinkState, 5, pyLinkStateWorldLinkFrameOrientation);

		  return pyLinkState;
	  }
    }
  }

  Py_INCREF(Py_None);
  return Py_None;
}


static PyObject* pybullet_readUserDebugParameter(PyObject* self, PyObject* args, PyObject *keywds) 
{

	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int itemUniqueId;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	static char *kwlist[] = { "itemUniqueId", "physicsClientId", NULL };

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,&itemUniqueId, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitUserDebugReadParameter(sm,itemUniqueId);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED) 
	{
		double paramValue=0.f;
		int ok = b3GetStatusDebugParameterValue(statusHandle,&paramValue);
		if (ok)
		{
			PyObject* item = PyFloat_FromDouble(paramValue);
			return item;
		}
	}

	PyErr_SetString(SpamError, "Failed to read parameter.");
	return NULL;

}

	

static PyObject* pybullet_addUserDebugParameter(PyObject* self, PyObject* args, PyObject *keywds) 
{
  
  b3SharedMemoryCommandHandle commandHandle;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;

  char* text;

  
  double rangeMin = 0.f;
  double rangeMax = 1.f;
  double startValue = 0.f;
  int physicsClientId = 0;
  b3PhysicsClientHandle sm=0;
  static char *kwlist[] = { "paramName", "rangeMin", "rangeMax","startValue", "physicsClientId", NULL };


  if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|dddi", kwlist, &text,  &rangeMin,  &rangeMax,&startValue, &physicsClientId))
  {
	  return NULL;
  }
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
  
   commandHandle = b3InitUserDebugAddParameter(sm,text,rangeMin,rangeMax, startValue);

  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  statusType = b3GetStatusType(statusHandle);
  
	if (statusType == CMD_USER_DEBUG_DRAW_COMPLETED) 
	{
		int debugItemUniqueId = b3GetDebugItemUniqueId(statusHandle);
		PyObject* item = PyInt_FromLong(debugItemUniqueId);
		return item;
	}


  PyErr_SetString(SpamError, "Error in addUserDebugParameter.");
  return NULL;
}


static PyObject* pybullet_addUserDebugText(PyObject* self, PyObject* args, PyObject *keywds) 
{
  
  b3SharedMemoryCommandHandle commandHandle;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  int res = 0;

char* text;
  double posXYZ[3];
  double colorRGB[3]={1,1,1};

  
  PyObject* textPositionObj=0;
  PyObject* textColorRGBObj=0;
  double textSize = 1.f;
  double lifeTime = 0.f;
  int physicsClientId = 0;
  b3PhysicsClientHandle sm=0;
  static char *kwlist[] = { "text", "textPosition", "textColorRGB", "textSize", "lifeTime","physicsClientId", NULL };


  if (!PyArg_ParseTupleAndKeywords(args, keywds, "sO|Oddi", kwlist, &text,  &textPositionObj,  &textColorRGBObj,&textSize, &lifeTime,&physicsClientId))
  {
	  return NULL;
  }
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}


  res = pybullet_internalSetVectord(textPositionObj,posXYZ);
  if (!res)
  {
	  PyErr_SetString(SpamError, "Error converting textPositionObj[3]");
	  return NULL;
  }

  if (textColorRGBObj)
  {
	  res = pybullet_internalSetVectord(textColorRGBObj,colorRGB);
	  if (!res)
	  {
		  PyErr_SetString(SpamError, "Error converting textColorRGBObj[3]");
		  return NULL;
	  }
  }

  
   commandHandle = b3InitUserDebugDrawAddText3D(sm,text,posXYZ,colorRGB,textSize,lifeTime);

  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  statusType = b3GetStatusType(statusHandle);
  if (statusType == CMD_USER_DEBUG_DRAW_COMPLETED) 
  {
    int debugItemUniqueId = b3GetDebugItemUniqueId(statusHandle);
	PyObject* item = PyInt_FromLong(debugItemUniqueId);
	return item;
  }

  PyErr_SetString(SpamError, "Error in addUserDebugText.");
	  return NULL;
}


static PyObject* pybullet_addUserDebugLine(PyObject* self, PyObject* args, PyObject *keywds) 
{
  
  b3SharedMemoryCommandHandle commandHandle;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  int res = 0;

 
  double fromXYZ[3];
  double toXYZ[3];
  double colorRGB[3]={1,1,1};

  PyObject* lineFromObj=0;
  PyObject* lineToObj=0;
  PyObject* lineColorRGBObj=0;
  double lineWidth = 1.f;
  double lifeTime = 0.f;
  int physicsClientId = 0;
  b3PhysicsClientHandle sm=0;
  static char *kwlist[] = { "lineFromXYZ", "lineToXYZ", "lineColorRGB", "lineWidth", "lifeTime","physicsClientId", NULL };


  if (!PyArg_ParseTupleAndKeywords(args, keywds, "OO|Oddi", kwlist, &lineFromObj,  &lineToObj,  &lineColorRGBObj,&lineWidth, &lifeTime,&physicsClientId))
  {
	  return NULL;
  }
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

  res = pybullet_internalSetVectord(lineFromObj,fromXYZ);
  if (!res)
  {
	  PyErr_SetString(SpamError, "Error converting lineFrom[3]");
	  return NULL;
  }

  res = pybullet_internalSetVectord(lineToObj,toXYZ);
  if (!res)
  {
	  PyErr_SetString(SpamError, "Error converting lineTo[3]");
	  return NULL;
  }
  if (lineColorRGBObj)
  {
	  res = pybullet_internalSetVectord(lineColorRGBObj,colorRGB);
  }
  
   commandHandle = b3InitUserDebugDrawAddLine3D(sm,fromXYZ,toXYZ,colorRGB,lineWidth,lifeTime);

  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  statusType = b3GetStatusType(statusHandle);
  if (statusType == CMD_USER_DEBUG_DRAW_COMPLETED) 
  {
    int debugItemUniqueId = b3GetDebugItemUniqueId(statusHandle);
	PyObject* item = PyInt_FromLong(debugItemUniqueId);
	return item;
  }

  PyErr_SetString(SpamError, "Error in addUserDebugLine.");
	  return NULL;
}




static PyObject* pybullet_removeUserDebugItem(PyObject* self, PyObject* args, PyObject *keywds) 
{
	 b3SharedMemoryCommandHandle commandHandle;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  int itemUniqueId;
   int physicsClientId = 0;
   b3PhysicsClientHandle sm = 0;

	static char *kwlist[] = { "itemUniqueId", "physicsClientId", NULL };

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,&itemUniqueId, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitUserDebugDrawRemove(sm,itemUniqueId);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_removeAllUserDebugItems(PyObject* self, PyObject* args, PyObject *keywds) 
{
	 b3SharedMemoryCommandHandle commandHandle;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  int physicsClientId = 0;
  b3PhysicsClientHandle sm = 0;
	static char *kwlist[] = { "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist,&physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}


	commandHandle = b3InitUserDebugDrawRemoveAll(sm);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	  

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_startStateLogging(PyObject* self, PyObject* args, PyObject *keywds) 
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	b3PhysicsClientHandle sm = 0;
	int loggingType = -1;
	char* fileName = 0;
	PyObject* objectUniqueIdsObj = 0;

	static char *kwlist[] = { "loggingType", "fileName", "objectUniqueIds", "physicsClientId", NULL };
	int physicsClientId = 0;
		
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "is|Oi", kwlist,
		&loggingType, &fileName, &objectUniqueIdsObj,&physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	{
		b3SharedMemoryCommandHandle commandHandle;
		commandHandle = b3StateLoggingCommandInit(sm);
		
		b3StateLoggingStart(commandHandle,loggingType,fileName);

		if (objectUniqueIdsObj)
		{
			PyObject* seq = PySequence_Fast(objectUniqueIdsObj, "expected a sequence of object unique ids");
			if (seq)
			{
				int len = PySequence_Size(objectUniqueIdsObj);
				int i;
				for ( i=0;i<len;i++)
				{
					int objectUid = pybullet_internalGetFloatFromSequence(seq, i);
					b3StateLoggingAddLoggingObjectUniqueId(commandHandle,objectUid);
				}
				Py_DECREF(seq);
			}
		}

		
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType==CMD_STATE_LOGGING_START_COMPLETED)
		{
			int loggingUniqueId = b3GetStatusLoggingUniqueId(statusHandle);
			PyObject* loggingUidObj = PyInt_FromLong(loggingUniqueId);
			return loggingUidObj;
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_stopStateLogging(PyObject* self, PyObject* args, PyObject *keywds) 
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int loggingId=-1;

	b3PhysicsClientHandle sm = 0;
	static char *kwlist[] = { "loggingId", "physicsClientId", NULL };
		int physicsClientId = 0;
		
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,
		&loggingId,&physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	if (loggingId>=0)
	{
		b3SharedMemoryCommandHandle commandHandle;
		commandHandle = b3StateLoggingCommandInit(sm);
		b3StateLoggingStop(commandHandle, loggingId);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);

	}
	Py_INCREF(Py_None);
	return Py_None;
}



static PyObject* pybullet_rayTest(PyObject* self, PyObject* args, PyObject *keywds) 
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	PyObject* rayFromObj=0;
	PyObject* rayToObj=0;
	double from[3];
	double to[3];
	b3PhysicsClientHandle sm = 0;
	static char *kwlist[] = { "rayFromPosition", "rayToPosition", "physicsClientId", NULL };
		int physicsClientId = 0;
		
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OO|i", kwlist,
		&rayFromObj, &rayToObj,&physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	pybullet_internalSetVectord(rayFromObj,from);
	pybullet_internalSetVectord(rayToObj,to);

	commandHandle = b3CreateRaycastCommandInit(sm, from[0],from[1],from[2],
		to[0],to[1],to[2]);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType==CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED)
	{
		struct b3RaycastInformation raycastInfo;
		PyObject* rayHitsObj = 0;
		int i;
		b3GetRaycastInformation(sm, &raycastInfo);
		
		rayHitsObj = PyTuple_New(raycastInfo.m_numRayHits);
		for (i=0;i<raycastInfo.m_numRayHits;i++)
		{
			PyObject* singleHitObj  = PyTuple_New(5);
			{
				PyObject* ob = PyInt_FromLong(raycastInfo.m_rayHits[i].m_hitObjectUniqueId);
				PyTuple_SetItem(singleHitObj,0,ob);
			}
			{
				PyObject* ob = PyInt_FromLong(raycastInfo.m_rayHits[i].m_hitObjectLinkIndex);
				PyTuple_SetItem(singleHitObj,1,ob);
			}
			{
				PyObject* ob = PyFloat_FromDouble(raycastInfo.m_rayHits[i].m_hitFraction);
				PyTuple_SetItem(singleHitObj,2,ob);
			}
			{
				PyObject* posObj  = PyTuple_New(3);
				int p;
				for (p=0;p<3;p++)
				{
					PyObject* ob = PyFloat_FromDouble(raycastInfo.m_rayHits[i].m_hitPositionWorld[p]);
					PyTuple_SetItem(posObj,p,ob);
				}
				PyTuple_SetItem(singleHitObj,3,posObj);
			}
			{
				PyObject* normalObj  = PyTuple_New(3);
				int p;
				for (p=0;p<3;p++)
				{
					PyObject* ob = PyFloat_FromDouble(raycastInfo.m_rayHits[i].m_hitNormalWorld[p]);
					PyTuple_SetItem(normalObj,p,ob);
				}
				PyTuple_SetItem(singleHitObj,4,normalObj);
			}
			PyTuple_SetItem(rayHitsObj,i,singleHitObj);
		}
		return rayHitsObj;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getMatrixFromQuaternion(PyObject* self, PyObject* args)
{
	PyObject* quatObj;
	double quat[4];
	if (PyArg_ParseTuple(args, "O", &quatObj)) 
	{
		if (pybullet_internalSetVector4d(quatObj,quat))
		{
			///see btMatrix3x3::setRotation
			int i;
			double d = quat[0]*quat[0]+quat[1]*quat[1]+quat[2]*quat[2]+quat[3]*quat[3];
			double s = 2.0 / d;
			double xs = quat[0] * s,   ys = quat[1] * s,   zs = quat[2] * s;
			double wx = quat[3] * xs,  wy = quat[3] * ys,  wz = quat[3] * zs;
			double xx = quat[0] * xs,  xy = quat[0] * ys,  xz = quat[0] * zs;
			double yy = quat[1] * ys,  yz = quat[1] * zs,  zz = quat[2] * zs;
			double mat3x3[9] = {
				1.0 - (yy + zz), xy - wz, xz + wy,
				xy + wz, 1.0 - (xx + zz), yz - wx,
				xz - wy, yz + wx, 1.0 - (xx + yy)};
			PyObject* matObj = PyTuple_New(9);
			for (i=0;i<9;i++)
			{
				PyTuple_SetItem(matObj,i,PyFloat_FromDouble(mat3x3[i]));
			}
			return matObj;
		}
	}
	PyErr_SetString(SpamError, "Couldn't convert quaternion [x,y,z,w].");
	return NULL;
};


static PyObject* pybullet_setVRCameraState(PyObject* self, PyObject* args, PyObject *keywds) 
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	PyObject* rootPosObj=0;
	PyObject* rootOrnObj=0;
	int trackObjectUid=-2;
	double rootPos[3];
	double rootOrn[4];
	
	static char *kwlist[] = {"rootPosition", "rootOrientation", "trackObject", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|OOii", kwlist,&rootPosObj, &rootOrnObj, &trackObjectUid,&physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3SetVRCameraStateCommandInit(sm);

	if (pybullet_internalSetVectord(rootPosObj,rootPos))
	{
		b3SetVRCameraRootPosition(commandHandle,rootPos);
	}
	if (pybullet_internalSetVector4d(rootOrnObj,rootOrn))
	{
		b3SetVRCameraRootOrientation(commandHandle,rootOrn);
	}

	if (trackObjectUid>=-1)
	{
		b3SetVRCameraTrackingObject(commandHandle,trackObjectUid);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	Py_INCREF(Py_None);
	return Py_None;

}


static PyObject* pybullet_getVREvents(PyObject* self, PyObject* args, PyObject *keywds) 
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char *kwlist[] = { "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist,&physicsClientId))
	{
		return NULL;
	}
	
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}


	commandHandle = b3RequestVREventsCommandInit(sm);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType==CMD_REQUEST_VR_EVENTS_DATA_COMPLETED)
	{
		struct b3VREventsData vrEvents;
		PyObject* vrEventsObj;
		int i=0;
		b3GetVREventsData(sm,&vrEvents);
		
		vrEventsObj = PyTuple_New(vrEvents.m_numControllerEvents);
		for (i=0;i<vrEvents.m_numControllerEvents;i++)
		{
			PyObject* vrEventObj = PyTuple_New(7);

			PyTuple_SetItem(vrEventObj,0,PyInt_FromLong(vrEvents.m_controllerEvents[i].m_controllerId));
			{
				PyObject* posObj = PyTuple_New(3);
				PyTuple_SetItem(posObj,0,PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_pos[0]));
				PyTuple_SetItem(posObj,1,PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_pos[1]));
				PyTuple_SetItem(posObj,2,PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_pos[2]));
				PyTuple_SetItem(vrEventObj,1,posObj);
			}
			{
				PyObject* ornObj = PyTuple_New(4);
				PyTuple_SetItem(ornObj,0,PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_orn[0]));
				PyTuple_SetItem(ornObj,1,PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_orn[1]));
				PyTuple_SetItem(ornObj,2,PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_orn[2]));
				PyTuple_SetItem(ornObj,3,PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_orn[3]));
				PyTuple_SetItem(vrEventObj,2,ornObj);
			}

			PyTuple_SetItem(vrEventObj,3,PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_analogAxis));
			PyTuple_SetItem(vrEventObj,4,PyInt_FromLong(vrEvents.m_controllerEvents[i].m_numButtonEvents));
			PyTuple_SetItem(vrEventObj,5,PyInt_FromLong(vrEvents.m_controllerEvents[i].m_numMoveEvents));
			{
				PyObject* buttonsObj = PyTuple_New(MAX_VR_BUTTONS);
				int b;
				for (b=0;b<MAX_VR_BUTTONS;b++)
				{
					PyObject* button = PyInt_FromLong(vrEvents.m_controllerEvents[i].m_buttons[b]);
					PyTuple_SetItem(buttonsObj,b,button);
				}
				PyTuple_SetItem(vrEventObj,6,buttonsObj);
			}
			PyTuple_SetItem(vrEventsObj,i,vrEventObj);
		}
	   return vrEventsObj;
	}

	Py_INCREF(Py_None);
	return Py_None;

}



static PyObject* pybullet_setDebugObjectColor(PyObject* self, PyObject* args, PyObject *keywds)
{
	PyObject* objectColorRGBObj = 0;
	double objectColorRGB[3];

	int objectUniqueId = -1;
	int linkIndex = -2;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm=0;
	static char *kwlist[] = { "objectUniqueId", "linkIndex","objectDebugColorRGB", "physicsClientId", NULL };

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|Oi", kwlist,
		&objectUniqueId, &linkIndex, &objectColorRGBObj,&physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (objectColorRGBObj)
	{
		if (pybullet_internalSetVectord(objectColorRGBObj, objectColorRGB))
		{
			b3SharedMemoryCommandHandle commandHandle = b3InitDebugDrawingCommand(sm);
			b3SetDebugObjectColor(commandHandle, objectUniqueId, linkIndex, objectColorRGB);
			b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		}
	}
	else
	{
		b3SharedMemoryCommandHandle commandHandle = b3InitDebugDrawingCommand(sm);
		b3RemoveDebugObjectColor(commandHandle, objectUniqueId, linkIndex);
		b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	}
	Py_INCREF(Py_None);
	return Py_None;
}
	

static PyObject* pybullet_getVisualShapeData(PyObject* self, PyObject* args, PyObject* keywds) 
{
	int objectUniqueId = -1;
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	struct b3VisualShapeInformation visualShapeInfo;
	int statusType;
	int i;
	PyObject* pyResultList = 0;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char *kwlist[] = { "objectUniqueId", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,&objectUniqueId,&physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	

	{
		

		commandHandle = b3InitRequestVisualShapeInformation(sm, objectUniqueId);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_VISUAL_SHAPE_INFO_COMPLETED)
		{
			b3GetVisualShapeInformation(sm, &visualShapeInfo);
			pyResultList = PyTuple_New(visualShapeInfo.m_numVisualShapes);
			for (i = 0; i < visualShapeInfo.m_numVisualShapes; i++)
			{
				PyObject* visualShapeObList = PyTuple_New(8);
				PyObject* item;
				item = PyInt_FromLong(visualShapeInfo.m_visualShapeData[i].m_objectUniqueId);
				PyTuple_SetItem(visualShapeObList, 0, item);
				
				item = PyInt_FromLong(visualShapeInfo.m_visualShapeData[i].m_linkIndex);
				PyTuple_SetItem(visualShapeObList, 1, item);
				
				item = PyInt_FromLong(visualShapeInfo.m_visualShapeData[i].m_visualGeometryType);
				PyTuple_SetItem(visualShapeObList, 2, item);

				{
					PyObject* vec = PyTuple_New(3);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_dimensions[0]);
					PyTuple_SetItem(vec, 0, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_dimensions[1]);
					PyTuple_SetItem(vec, 1, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_dimensions[2]);
					PyTuple_SetItem(vec, 2, item);
					PyTuple_SetItem(visualShapeObList, 3, vec);
				}
				
				item = PyString_FromString(visualShapeInfo.m_visualShapeData[i].m_meshAssetFileName);
				PyTuple_SetItem(visualShapeObList, 4, item);

				{
					PyObject* vec = PyTuple_New(3);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[0]);
					PyTuple_SetItem(vec, 0, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[1]);
					PyTuple_SetItem(vec, 1, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[2]);
					PyTuple_SetItem(vec, 2, item);
					PyTuple_SetItem(visualShapeObList, 5, vec);
				}

				{
					PyObject* vec = PyTuple_New(4);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[3]);
					PyTuple_SetItem(vec, 0, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[4]);
					PyTuple_SetItem(vec, 1, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[5]);
					PyTuple_SetItem(vec, 2, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[6]);
					PyTuple_SetItem(vec, 3, item);
					PyTuple_SetItem(visualShapeObList, 6, vec);
				}

                {
                    PyObject* rgba = PyTuple_New(4);
                    item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_rgbaColor[0]);
                    PyTuple_SetItem(rgba, 0, item);
                    item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_rgbaColor[1]);
                    PyTuple_SetItem(rgba, 1, item);
                    item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_rgbaColor[2]);
                    PyTuple_SetItem(rgba, 2, item);
                    item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_rgbaColor[3]);
                    PyTuple_SetItem(rgba, 3, item);
                    PyTuple_SetItem(visualShapeObList, 7, rgba);
                }
                

				PyTuple_SetItem(pyResultList, i, visualShapeObList);
			}
			return pyResultList;
		}
		else
		{
			PyErr_SetString(SpamError, "Error receiving visual shape info");
			return NULL;
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_resetVisualShapeData(PyObject* self, PyObject* args,PyObject* keywds)
{
 
    int objectUniqueId = -1;
    int jointIndex = -1;
    int shapeIndex = -1;
    int textureUniqueId = -1;
    b3SharedMemoryCommandHandle commandHandle;
    b3SharedMemoryStatusHandle statusHandle;
    int statusType;
    int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char *kwlist[] = {  "objectUniqueId", "jointIndex", "shapeIndex", "textureUniqueId", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiii|i", kwlist, &objectUniqueId, &jointIndex, &shapeIndex, &textureUniqueId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

  
    {
        
        commandHandle = b3InitUpdateVisualShape(sm, objectUniqueId, jointIndex, shapeIndex, textureUniqueId);
        statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
        statusType = b3GetStatusType(statusHandle);
        if (statusType == CMD_VISUAL_SHAPE_UPDATE_COMPLETED)
        {
        }
        else
        {
            PyErr_SetString(SpamError, "Error resetting visual shape info");
            return NULL;
        }
    }
    
    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject* pybullet_loadTexture(PyObject* self, PyObject* args, PyObject* keywds)
{
    const char* filename = 0;
    b3SharedMemoryCommandHandle commandHandle;
    b3SharedMemoryStatusHandle statusHandle;
    int statusType;
    
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char *kwlist[] = { "textureFilename", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|i", kwlist,&filename, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	
    {
        
        commandHandle = b3InitLoadTexture(sm, filename);
        statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
        statusType = b3GetStatusType(statusHandle);
        if (statusType == CMD_LOAD_TEXTURE_COMPLETED)
        {
        }
        else
        {
            PyErr_SetString(SpamError, "Error loading texture");
            return NULL;
        }
    }
    
    Py_INCREF(Py_None);
    return Py_None;
}


static PyObject* MyConvertContactPoint( struct b3ContactInformation* contactPointPtr)
{
	 /*
     0     int m_contactFlags;
     1     int m_bodyUniqueIdA;
     2     int m_bodyUniqueIdB;
     3     int m_linkIndexA;
     4     int m_linkIndexB;
     5		double m_positionOnAInWS[3];//contact point location on object A,
     in world space coordinates
     6		double m_positionOnBInWS[3];//contact point location on object
     A, in world space coordinates
     7		double m_contactNormalOnBInWS[3];//the separating contact
     normal, pointing from object B towards object A
     8		double m_contactDistance;//negative number is penetration, positive
     is distance.
     9		double m_normalForce;
     */

	int i;

	PyObject* pyResultList = PyTuple_New(contactPointPtr->m_numContactPoints);
    for (i = 0; i < contactPointPtr->m_numContactPoints; i++) {
      PyObject* contactObList = PyTuple_New(10);  // see above 10 fields
      PyObject* item;
      item =
          PyInt_FromLong(contactPointPtr->m_contactPointData[i].m_contactFlags);
      PyTuple_SetItem(contactObList, 0, item);
      item = PyInt_FromLong(
          contactPointPtr->m_contactPointData[i].m_bodyUniqueIdA);
      PyTuple_SetItem(contactObList, 1, item);
      item = PyInt_FromLong(
          contactPointPtr->m_contactPointData[i].m_bodyUniqueIdB);
      PyTuple_SetItem(contactObList, 2, item);
      item =
          PyInt_FromLong(contactPointPtr->m_contactPointData[i].m_linkIndexA);
      PyTuple_SetItem(contactObList, 3, item);
      item =
          PyInt_FromLong(contactPointPtr->m_contactPointData[i].m_linkIndexB);
      PyTuple_SetItem(contactObList, 4, item);

	  {
		  PyObject* posAObj = PyTuple_New(3);

		  item = PyFloat_FromDouble(
			  contactPointPtr->m_contactPointData[i].m_positionOnAInWS[0]);
		  PyTuple_SetItem(posAObj, 0, item);
		  item = PyFloat_FromDouble(
			  contactPointPtr->m_contactPointData[i].m_positionOnAInWS[1]);
		  PyTuple_SetItem(posAObj, 1, item);
		  item = PyFloat_FromDouble(
			  contactPointPtr->m_contactPointData[i].m_positionOnAInWS[2]);
		  PyTuple_SetItem(posAObj, 2, item);

		  PyTuple_SetItem(contactObList, 5, posAObj);
	  }

	  {
		  PyObject* posBObj = PyTuple_New(3);


		  item = PyFloat_FromDouble(
			  contactPointPtr->m_contactPointData[i].m_positionOnBInWS[0]);
		  PyTuple_SetItem(posBObj, 0, item);
		  item = PyFloat_FromDouble(
			  contactPointPtr->m_contactPointData[i].m_positionOnBInWS[1]);
		  PyTuple_SetItem(posBObj, 1, item);
		  item = PyFloat_FromDouble(
			  contactPointPtr->m_contactPointData[i].m_positionOnBInWS[2]);
		  PyTuple_SetItem(posBObj, 2, item);

		  PyTuple_SetItem(contactObList, 6, posBObj);

	  }

	  {
		  PyObject* normalOnB = PyTuple_New(3);
		  item = PyFloat_FromDouble(
			  contactPointPtr->m_contactPointData[i].m_contactNormalOnBInWS[0]);
		  PyTuple_SetItem(normalOnB, 0, item);
		  item = PyFloat_FromDouble(
			  contactPointPtr->m_contactPointData[i].m_contactNormalOnBInWS[1]);
		  PyTuple_SetItem(normalOnB, 1, item);
		  item = PyFloat_FromDouble(
			  contactPointPtr->m_contactPointData[i].m_contactNormalOnBInWS[2]);
		  PyTuple_SetItem(normalOnB, 2, item);
		  PyTuple_SetItem(contactObList, 7, normalOnB);
	  }

      item = PyFloat_FromDouble(
          contactPointPtr->m_contactPointData[i].m_contactDistance);
      PyTuple_SetItem(contactObList, 8, item);
      item = PyFloat_FromDouble(
          contactPointPtr->m_contactPointData[i].m_normalForce);
      PyTuple_SetItem(contactObList, 9, item);

      PyTuple_SetItem(pyResultList, i, contactObList);
    }
    return pyResultList;
}

static PyObject* pybullet_getOverlappingObjects(PyObject* self, PyObject* args, PyObject *keywds)
{
	PyObject* aabbMinOb=0, *aabbMaxOb=0;
	double aabbMin[3];
	double aabbMax[3];
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	struct b3AABBOverlapData overlapData;
	int i;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char *kwlist[] = { "aabbMin", "aabbMax", "physicsClientId",NULL };
	

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OO|i", kwlist,
		&aabbMinOb, &aabbMaxOb,&physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}


	pybullet_internalSetVectord(aabbMinOb, aabbMin);
	pybullet_internalSetVectord(aabbMaxOb, aabbMax);

	commandHandle = b3InitAABBOverlapQuery(sm, aabbMin, aabbMax);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	b3GetAABBOverlapResults(sm, &overlapData);

	if (overlapData.m_numOverlappingObjects)
	{
		PyObject* pyResultList = PyTuple_New(overlapData.m_numOverlappingObjects);
		//For huge amount of overlap, we could use numpy instead (see camera pixel data)
		//What would Python do with huge amount of data? Pass it onto TensorFlow!

		for (i = 0; i < overlapData.m_numOverlappingObjects; i++) {
			PyObject* overlap = PyTuple_New(2);//body unique id and link index

			PyObject* item;
			item =
				PyInt_FromLong(overlapData.m_overlappingObjects[i].m_objectUniqueId);
			PyTuple_SetItem(overlap, 0, item);
			item =
				PyInt_FromLong(overlapData.m_overlappingObjects[i].m_linkIndex);
			PyTuple_SetItem(overlap, 1, item);
			PyTuple_SetItem(pyResultList, i, overlap);
		}

		return pyResultList;
	}

	Py_INCREF(Py_None);
	return Py_None;
}


static PyObject* pybullet_getClosestPointData(PyObject* self, PyObject* args, PyObject *keywds) 
{
  int bodyUniqueIdA = -1;
  int bodyUniqueIdB = -1;
  int linkIndexA = -2;
  int linkIndexB = -2;

  double distanceThreshold = 0.f;

  b3SharedMemoryCommandHandle commandHandle;
  struct b3ContactInformation contactPointData;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;
  int physicsClientId = 0;
  b3PhysicsClientHandle sm = 0;
  static char *kwlist[] = { "bodyA", "bodyB", "distance", "linkIndexA","linkIndexB","physicsClientId", NULL };

 

  if (!PyArg_ParseTupleAndKeywords(args, keywds, "iid|iii", kwlist,
	  &bodyUniqueIdA, &bodyUniqueIdB, &distanceThreshold, &linkIndexA, &linkIndexB,&physicsClientId))
	  return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

  commandHandle = b3InitClosestDistanceQuery(sm);
  b3SetClosestDistanceFilterBodyA(commandHandle, bodyUniqueIdA);
  b3SetClosestDistanceFilterBodyB(commandHandle, bodyUniqueIdB);
  b3SetClosestDistanceThreshold(commandHandle, distanceThreshold);
  if (linkIndexA >= -1)
  {
	  b3SetClosestDistanceFilterLinkA(commandHandle, linkIndexA);
  }
  if (linkIndexB >= -1)
  {
	  b3SetClosestDistanceFilterLinkB(commandHandle, linkIndexB);
  }

  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  statusType = b3GetStatusType(statusHandle);
  if (statusType == CMD_CONTACT_POINT_INFORMATION_COMPLETED) {
   

    b3GetContactPointInformation(sm, &contactPointData);

	return MyConvertContactPoint(&contactPointData);
    
  }

  Py_INCREF(Py_None);
  return Py_None;
}


static PyObject* pybullet_changeUserConstraint(PyObject* self, PyObject* args, PyObject *keywds) 
{
	static char *kwlist[] = { "userConstraintUniqueId","jointChildPivot", "jointChildFrameOrientation","maxForce", "physicsClientId", NULL};
	int userConstraintUniqueId=-1;
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	PyObject* jointChildPivotObj=0;
	PyObject* jointChildFrameOrnObj=0;
	double jointChildPivot[3];
	double jointChildFrameOrn[4];
	double maxForce = -1;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|OOdi", kwlist,&userConstraintUniqueId,&jointChildPivotObj, &jointChildFrameOrnObj,&maxForce, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitChangeUserConstraintCommand(sm,userConstraintUniqueId);

	if (pybullet_internalSetVectord(jointChildPivotObj,jointChildPivot))
	{
		b3InitChangeUserConstraintSetPivotInB(commandHandle, jointChildPivot);
	}
	if (pybullet_internalSetVector4d(jointChildFrameOrnObj,jointChildFrameOrn))
	{
		b3InitChangeUserConstraintSetFrameInB(commandHandle, jointChildFrameOrn);
	}
	if (maxForce>=0)
	{
		b3InitChangeUserConstraintSetMaxForce(commandHandle,maxForce);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	Py_INCREF(Py_None);
	return Py_None;
};


static PyObject* pybullet_removeUserConstraint(PyObject* self, PyObject* args, PyObject *keywds) 
{
	static char *kwlist[] = { "userConstraintUniqueId","physicsClientId", NULL};
	int userConstraintUniqueId=-1;
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,&userConstraintUniqueId,&physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitRemoveUserConstraintCommand(sm,userConstraintUniqueId);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	Py_INCREF(Py_None);
	return Py_None;
};


/*
static PyObject* pybullet_updateUserConstraint(PyObject* self, PyObject* args, PyObject *keywds) 
{
	return NULL;
}
*/

static PyObject* pybullet_enableJointForceTorqueSensor(PyObject* self, PyObject* args, PyObject *keywds)
{
	int bodyUniqueId = -1;
	int jointIndex = -1;
	int enableSensor = 1;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	int numJoints = -1;

	static char *kwlist[] = {"bodyUniqueId", "jointIndex" ,"enableSensor", "physicsClientId" };

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|ii", kwlist, &bodyUniqueId, &jointIndex, &enableSensor,&physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	
	if (bodyUniqueId < 0)
	{
		PyErr_SetString(SpamError, "Error: invalid bodyUniqueId");
		return NULL;
	}
	numJoints = b3GetNumJoints(sm, bodyUniqueId);
	if ((jointIndex < 0) || (jointIndex >= numJoints))
	{
		PyErr_SetString(SpamError, "Error: invalid jointIndex.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle commandHandle;
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;

		commandHandle = b3CreateSensorCommandInit(sm, bodyUniqueId);
		b3CreateSensorEnable6DofJointForceTorqueSensor(commandHandle, jointIndex, enableSensor);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_CLIENT_COMMAND_COMPLETED)
		{
			Py_INCREF(Py_None);
			return Py_None;
		}
	}

	PyErr_SetString(SpamError, "Error creating sensor.");
	return NULL;
}


static PyObject* pybullet_createUserConstraint(PyObject* self, PyObject* args, PyObject *keywds) 
{
  
	b3SharedMemoryCommandHandle commandHandle;
	int parentBodyUniqueId=-1;
	int parentLinkIndex=-1;
	int childBodyUniqueId=-1;
	int childLinkIndex=-1;
	int jointType=ePoint2PointType;
	PyObject* jointAxisObj=0;
	double jointAxis[3]={0,0,0};
	PyObject* parentFramePositionObj = 0;
	double parentFramePosition[3]={0,0,0};
	PyObject* childFramePositionObj = 0;
	double childFramePosition[3]={0,0,0};
	PyObject* parentFrameOrientationObj = 0;
	double parentFrameOrientation[4]={0,0,0,1};
	PyObject* childFrameOrientationObj = 0;
	double childFrameOrientation[4]={0,0,0,1};

	struct b3JointInfo jointInfo;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char *kwlist[] = { "parentBodyUniqueId", "parentLinkIndex",
							  "childBodyUniqueId", "childLinkIndex",
							"jointType",
							"jointAxis",
							"parentFramePosition",
							"childFramePosition",
							"parentFrameOrientation",
							"childFrameOrientation",
							"physicsClientId",
							NULL };

	

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiiiiOOO|OOi", kwlist,&parentBodyUniqueId,&parentLinkIndex,
										&childBodyUniqueId,&childLinkIndex,
										&jointType,&jointAxisObj,
										&parentFramePositionObj,
										&childFramePositionObj,
										&parentFrameOrientationObj,
										&childFrameOrientationObj,
										&physicsClientId
	))
	{
		return NULL;
	}
	
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	pybullet_internalSetVectord(jointAxisObj,jointAxis);
	pybullet_internalSetVectord(parentFramePositionObj,parentFramePosition);
	pybullet_internalSetVectord(childFramePositionObj,childFramePosition);
	pybullet_internalSetVector4d(parentFrameOrientationObj,parentFrameOrientation);
	pybullet_internalSetVector4d(childFrameOrientationObj,childFrameOrientation);
	
	jointInfo.m_jointType = jointType;
	jointInfo.m_parentFrame[0] = parentFramePosition[0];
	jointInfo.m_parentFrame[1] = parentFramePosition[1];
	jointInfo.m_parentFrame[2] = parentFramePosition[2];
	jointInfo.m_parentFrame[3] = parentFrameOrientation[0];
	jointInfo.m_parentFrame[4] = parentFrameOrientation[1];
	jointInfo.m_parentFrame[5] = parentFrameOrientation[2];
	jointInfo.m_parentFrame[6] = parentFrameOrientation[3];

	jointInfo.m_childFrame[0] = childFramePosition[0];
	jointInfo.m_childFrame[1] = childFramePosition[1];
	jointInfo.m_childFrame[2] = childFramePosition[2];
	jointInfo.m_childFrame[3] = childFrameOrientation[0];
	jointInfo.m_childFrame[4] = childFrameOrientation[1];
	jointInfo.m_childFrame[5] = childFrameOrientation[2];
	jointInfo.m_childFrame[6] = childFrameOrientation[3];
	
	jointInfo.m_jointAxis[0] = jointAxis[0];
	jointInfo.m_jointAxis[1] = jointAxis[1];
	jointInfo.m_jointAxis[2] = jointAxis[2];

	commandHandle = b3InitCreateUserConstraintCommand(sm,  parentBodyUniqueId, parentLinkIndex, childBodyUniqueId, childLinkIndex, &jointInfo);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType==CMD_USER_CONSTRAINT_COMPLETED)
	{
		int userConstraintUid = b3GetStatusUserConstraintUniqueId(statusHandle);
		PyObject* ob = PyLong_FromLong(userConstraintUid);
		return ob;
	}


	PyErr_SetString(SpamError, "createConstraint failed.");
	return NULL;
}

static PyObject* pybullet_getContactPointData(PyObject* self, PyObject* args, PyObject *keywds) {
  int bodyUniqueIdA = -1;
  int bodyUniqueIdB = -1;
  
  b3SharedMemoryCommandHandle commandHandle;
  struct b3ContactInformation contactPointData;
  b3SharedMemoryStatusHandle statusHandle;
  int statusType;


  static char *kwlist[] = { "bodyA", "bodyB","physicsClientId", NULL };

	 int physicsClientId = 0;
	 b3PhysicsClientHandle sm = 0;
  if (!PyArg_ParseTupleAndKeywords(args, keywds, "|iii", kwlist,
	  &bodyUniqueIdA, &bodyUniqueIdB,&physicsClientId))
	  return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

  commandHandle = b3InitRequestContactPointInformation(sm);
  b3SetContactFilterBodyA(commandHandle, bodyUniqueIdA);
  b3SetContactFilterBodyB(commandHandle, bodyUniqueIdB);
  //b3SetContactQueryMode(commandHandle, mode);

  statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
  statusType = b3GetStatusType(statusHandle);
  if (statusType == CMD_CONTACT_POINT_INFORMATION_COMPLETED) {
   
    b3GetContactPointInformation(sm, &contactPointData);

	return MyConvertContactPoint(&contactPointData);
    
  }

  Py_INCREF(Py_None);
  return Py_None;
}



/// Render an image from the current timestep of the simulation, width, height are required, other args are optional
// getCameraImage(w, h, view[16], projection[16], lightDir[3], lightColor[3], lightDist, hasShadow, lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff)
static PyObject* pybullet_getCameraImage(PyObject* self, PyObject* args, PyObject *keywds)
{
	/// request an image from a simulated camera, using a software renderer.
	struct b3CameraImageData imageData;
	PyObject* objViewMat = 0, *objProjMat = 0, *lightDirObj = 0, *lightColorObj = 0;
	int width, height;
	float viewMatrix[16];
	float projectionMatrix[16];
	float lightDir[3];
    float lightColor[3];
    float lightDist = 10.0;
    int hasShadow = 0;
    float lightAmbientCoeff = 0.6;
    float lightDiffuseCoeff = 0.35;
    float lightSpecularCoeff = 0.05;
	// inialize cmd
	b3SharedMemoryCommandHandle command;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	// set camera resolution, optionally view, projection matrix, light direction, light color, light distance, shadow
	static char *kwlist[] = { "width", "height", "viewMatrix", "projectionMatrix", "lightDirection", "lightColor", "lightDistance", "shadow", "lightAmbientCoeff", "lightDiffuseCoeff", "lightSpecularCoeff", "physicsClientId", NULL };

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|OOOOfifffi", kwlist, &width, &height, &objViewMat, &objProjMat, &lightDirObj, &lightColorObj, &lightDist, &hasShadow, &lightAmbientCoeff, &lightDiffuseCoeff, &lightSpecularCoeff,&physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	command = b3InitRequestCameraImage(sm);
	b3RequestCameraImageSetPixelResolution(command, width, height);

	// set camera matrices only if set matrix function succeeds
	if (pybullet_internalSetMatrix(objViewMat, viewMatrix) && (pybullet_internalSetMatrix(objProjMat, projectionMatrix)))
	{
		b3RequestCameraImageSetCameraMatrices(command, viewMatrix, projectionMatrix);
	}
	//set light direction only if function succeeds
	if (pybullet_internalSetVector(lightDirObj, lightDir))
	{
		b3RequestCameraImageSetLightDirection(command, lightDir);
	}
    //set light color only if function succeeds
    if (pybullet_internalSetVector(lightColorObj, lightColor))
    {
        b3RequestCameraImageSetLightColor(command, lightColor);
    }

    b3RequestCameraImageSetLightDistance(command, lightDist);
    
    b3RequestCameraImageSetShadow(command, hasShadow);
    
    b3RequestCameraImageSetLightAmbientCoeff(command, lightAmbientCoeff);
    b3RequestCameraImageSetLightDiffuseCoeff(command, lightDiffuseCoeff);
    b3RequestCameraImageSetLightSpecularCoeff(command, lightSpecularCoeff);

	if (b3CanSubmitCommand(sm))
	{
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

			npy_intp rgb_dims[3] = { imageData.m_pixelHeight, imageData.m_pixelWidth,
				bytesPerPixel };
			npy_intp dep_dims[2] = { imageData.m_pixelHeight, imageData.m_pixelWidth };
			npy_intp seg_dims[2] = { imageData.m_pixelHeight, imageData.m_pixelWidth };

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



static PyObject* pybullet_computeViewMatrix(PyObject* self, PyObject* args, PyObject *keywds)
{
	PyObject* camEyeObj = 0;
	PyObject* camTargetPositionObj = 0;
	PyObject* camUpVectorObj = 0;
	float camEye[3];
	float camTargetPosition[3];
	float camUpVector[3];

	// set camera resolution, optionally view, projection matrix, light position
	static char *kwlist[] = { "cameraEyePosition", "cameraTargetPosition", "cameraUpVector",NULL };

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OOO", kwlist, &camEyeObj, &camTargetPositionObj, &camUpVectorObj))
	{
		return NULL;
	}

	if (pybullet_internalSetVector(camEyeObj, camEye) &&
		pybullet_internalSetVector(camTargetPositionObj, camTargetPosition) &&
		pybullet_internalSetVector(camUpVectorObj, camUpVector))
	{
		float viewMatrix[16];
		PyObject* pyResultList=0;
		int i;
		b3ComputeViewMatrixFromPositions(camEye, camTargetPosition, camUpVector, viewMatrix);

		pyResultList = PyTuple_New(16);
		for (i = 0; i < 16; i++)
		{
			PyObject* item = PyFloat_FromDouble(viewMatrix[i]);
			PyTuple_SetItem(pyResultList, i, item);
		}
		return pyResultList;
	}

	PyErr_SetString(SpamError, "Error in computeViewMatrix.");
	return NULL;
}

///compute a view matrix, helper function for b3RequestCameraImageSetCameraMatrices
static PyObject* pybullet_computeViewMatrixFromYawPitchRoll(PyObject* self, PyObject* args, PyObject *keywds)
{
	PyObject* cameraTargetPositionObj = 0;
	float cameraTargetPosition[3];
	float distance, yaw, pitch, roll;
	int upAxisIndex;
	float viewMatrix[16];
	PyObject* pyResultList = 0;
	int i;

	// set camera resolution, optionally view, projection matrix, light position
	static char *kwlist[] = { "cameraTargetPosition", "distance", "yaw", "pitch", "roll", "upAxisIndex" ,NULL };
	
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "Offffi", kwlist, &cameraTargetPositionObj, &distance,&yaw,&pitch,&roll, &upAxisIndex))
	{
		return NULL;
	}

	if (!pybullet_internalSetVector(cameraTargetPositionObj, cameraTargetPosition))
	{
		PyErr_SetString(SpamError, "Cannot convert cameraTargetPosition.");
		return NULL;
	}

	b3ComputeViewMatrixFromYawPitchRoll(cameraTargetPosition, distance, yaw, pitch, roll, upAxisIndex, viewMatrix);

	pyResultList = PyTuple_New(16);
	for (i = 0; i < 16; i++)
	{
		PyObject* item = PyFloat_FromDouble(viewMatrix[i]);
		PyTuple_SetItem(pyResultList, i, item);
	}
	return pyResultList;


}

///compute a projection matrix, helper function for b3RequestCameraImageSetCameraMatrices
static PyObject* pybullet_computeProjectionMatrix(PyObject* self, PyObject* args, PyObject *keywds)
{
	PyObject* pyResultList = 0;
	float left;
	float right;
	float bottom;
	float top;
	float nearVal;
	float farVal;
	float projectionMatrix[16];
	int i;

	// set camera resolution, optionally view, projection matrix, light position
	static char *kwlist[] = { "left", "right", "bottom", "top", "nearVal", "farVal" ,NULL };

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ffffff", kwlist, &left, &right, &bottom, &top, &nearVal, &farVal))
	{
		return NULL;
	}

	
	b3ComputeProjectionMatrix(left, right, bottom, top, nearVal, farVal, projectionMatrix);

	pyResultList = PyTuple_New(16);
	for (i = 0; i < 16; i++)
	{
		PyObject* item = PyFloat_FromDouble(projectionMatrix[i]);
		PyTuple_SetItem(pyResultList, i, item);
	}
	return pyResultList;

}

static PyObject* pybullet_computeProjectionMatrixFOV(PyObject* self, PyObject* args, PyObject *keywds)
{
	float fov,  aspect,  nearVal,  farVal;
	PyObject* pyResultList = 0;
	float projectionMatrix[16];
	int i;

	static char *kwlist[] = { "fov","aspect","nearVal","farVal",NULL };

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ffff", kwlist, &fov, &aspect, &nearVal, &farVal))
	{
		return NULL;
	}
	
	b3ComputeProjectionMatrixFOV(fov, aspect, nearVal, farVal, projectionMatrix);

	pyResultList = PyTuple_New(16);
	for (i = 0; i < 16; i++)
	{
		PyObject* item = PyFloat_FromDouble(projectionMatrix[i]);
		PyTuple_SetItem(pyResultList, i, item);
	}
	return pyResultList;

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
static PyObject* pybullet_renderImageObsolete(PyObject* self, PyObject* args) {
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
  b3PhysicsClientHandle sm;
  int physicsClientId = 0;

  	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
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
          pybullet_internalSetVector(objCameraUp, cameraUp)) 
	  {
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

static PyObject* pybullet_applyExternalForce(PyObject* self, PyObject* args,PyObject* keywds) {
 
  {
    int objectUniqueId=-1, linkIndex=-1, flags;
    double force[3];
    double position[3] = {0.0, 0.0, 0.0};
    PyObject* forceObj=0, *posObj=0;

    b3SharedMemoryCommandHandle command;
    b3SharedMemoryStatusHandle statusHandle;
  	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char *kwlist[] = { "objectUniqueId", "linkIndex",
                            "forceObj", "posObj", "flags", "physicsClientId", NULL };

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiOOi|i", kwlist,&objectUniqueId, &linkIndex,
                            &forceObj, &posObj, &flags, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
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

static PyObject* pybullet_applyExternalTorque(PyObject* self, PyObject* args,PyObject* keywds) {
 
  {
    int objectUniqueId, linkIndex, flags;
    double torque[3];
    PyObject* torqueObj;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char *kwlist[] = { "objectUniqueId", "linkIndex", "torqueObj",
                         "flags", "physicsClientId", NULL };

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiOi|i", kwlist,&objectUniqueId, &linkIndex, &torqueObj,
                         &flags, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	
    {
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


///Inverse Kinematics binding
static PyObject* pybullet_calculateInverseKinematics(PyObject* self,
                                                   PyObject* args, PyObject* keywds) 

 {
        int bodyIndex;
		int endEffectorLinkIndex;
		
        PyObject* targetPosObj=0;
		PyObject* targetOrnObj=0;

		int physicsClientId = 0;
		b3PhysicsClientHandle sm = 0;
		PyObject* lowerLimitsObj = 0;
		PyObject* upperLimitsObj = 0;
		PyObject* jointRangesObj = 0;
		PyObject* restPosesObj = 0;
        PyObject* jointDampingObj = 0;

	static char *kwlist[] = { "bodyIndex", "endEffectorLinkIndex", "targetPosition", "targetOrientation","lowerLimits", "upperLimits", "jointRanges", "restPoses", "physicsClientId", "jointDamping", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiO|OOOOOiO", kwlist,  &bodyIndex, &endEffectorLinkIndex, &targetPosObj,&targetOrnObj,&lowerLimitsObj, &upperLimitsObj, &jointRangesObj, &restPosesObj, &physicsClientId, & jointDampingObj))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	{
		double pos[3];
		double ori[4]={0,1.0,0,0};
        int hasPos =pybullet_internalSetVectord(targetPosObj,pos);
		int hasOrn = pybullet_internalSetVector4d(targetOrnObj,ori);

		int szLowerLimits = lowerLimitsObj ? PySequence_Size(lowerLimitsObj) : 0;
		int szUpperLimits = upperLimitsObj? PySequence_Size(upperLimitsObj): 0;
		int szJointRanges = jointRangesObj? PySequence_Size(jointRangesObj):0;
		int szRestPoses = restPosesObj? PySequence_Size(restPosesObj):0;
        int szJointDamping = jointDampingObj? PySequence_Size(jointDampingObj):0;

		int numJoints = b3GetNumJoints(sm, bodyIndex);


		int hasNullSpace = 0;
        int hasJointDamping = 0;
		double* lowerLimits = 0;
		double* upperLimits = 0;
		double* jointRanges = 0;
		double* restPoses = 0;
        double* jointDamping = 0;
		
		if (numJoints && (szLowerLimits == numJoints) && (szUpperLimits == numJoints) &&
          (szJointRanges == numJoints) && (szRestPoses == numJoints))
		{
			int szInBytes = sizeof(double) * numJoints;
			int i;
			lowerLimits = (double*)malloc(szInBytes);
			upperLimits = (double*)malloc(szInBytes);
			jointRanges = (double*)malloc(szInBytes);
			restPoses = (double*)malloc(szInBytes);

			for (i = 0; i < numJoints; i++) 
			{
			  lowerLimits[i] =
				  pybullet_internalGetFloatFromSequence(lowerLimitsObj, i);
			  upperLimits[i] =
				  pybullet_internalGetFloatFromSequence(upperLimitsObj, i);
			  jointRanges[i] =
				  pybullet_internalGetFloatFromSequence(jointRangesObj, i);
			  restPoses[i] =
				pybullet_internalGetFloatFromSequence(restPosesObj, i);
			}
			hasNullSpace = 1;
		}
        
        if (numJoints && (szJointDamping == numJoints))
        {
            int szInBytes = sizeof(double) * numJoints;
            int i;
            jointDamping = (double*)malloc(szInBytes);
            for (i = 0; i < numJoints; i++)
            {
                jointDamping[i] = pybullet_internalGetFloatFromSequence(jointDampingObj, i);
            }
            hasJointDamping = 1;
        }

		if (hasPos)
		{
			b3SharedMemoryStatusHandle statusHandle;
			int numPos=0;
			int resultBodyIndex;
			int result;

			b3SharedMemoryCommandHandle command = b3CalculateInverseKinematicsCommandInit(sm,bodyIndex);

			if (hasNullSpace)
			{
				if (hasOrn)
				{
					b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(command, numJoints, endEffectorLinkIndex, pos, ori, lowerLimits, upperLimits, jointRanges, restPoses);
				} else
				{
					b3CalculateInverseKinematicsPosWithNullSpaceVel(command,numJoints, endEffectorLinkIndex, pos, lowerLimits, upperLimits, jointRanges, restPoses);
				}
			} else
			{
				if (hasOrn)
				{
					b3CalculateInverseKinematicsAddTargetPositionWithOrientation(command,endEffectorLinkIndex,pos,ori);
				} else
				{
					b3CalculateInverseKinematicsAddTargetPurePosition(command,endEffectorLinkIndex,pos);
				}
			}
            
            if (hasJointDamping)
            {
                b3CalculateInverseKinematicsSetJointDamping(command, numJoints, jointDamping);
            }

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
    
    Py_INCREF(Py_None);
    return Py_None;
}



/// Given an object id, joint positions, joint velocities and joint
/// accelerations,
/// compute the joint forces using Inverse Dynamics
static PyObject* pybullet_calculateInverseDynamics(PyObject* self,
                                                   PyObject* args,PyObject* keywds) 
{
 

  {
    int bodyIndex;
    PyObject* objPositionsQ;
    PyObject* objVelocitiesQdot;
    PyObject* objAccelerations;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char *kwlist[] = { "bodyIndex", "objPositions", "objVelocities", "objAccelerations","physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iOOO|i", kwlist, &bodyIndex, &objPositionsQ,
                         &objVelocitiesQdot, &objAccelerations,&physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0) 
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	
	{
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

    }
  }
  Py_INCREF(Py_None);
  return Py_None;
}

static PyMethodDef SpamMethods[] = {

    {"connect",(PyCFunction)pybullet_connectPhysicsServer, METH_VARARGS|METH_KEYWORDS,
     "Connect to an existing physics server (using shared memory by default)."},

    {"disconnect", (PyCFunction)pybullet_disconnectPhysicsServer, METH_VARARGS|METH_KEYWORDS,
     "Disconnect from the physics server."},

    {"resetSimulation", (PyCFunction)pybullet_resetSimulation, METH_VARARGS|METH_KEYWORDS,
     "Reset the simulation: remove all objects and start from an empty world."},

    {"stepSimulation", (PyCFunction)pybullet_stepSimulation, METH_VARARGS|METH_KEYWORDS,
     "Step the simulation using forward dynamics."},

    {"setGravity", (PyCFunction)pybullet_setGravity, METH_VARARGS|METH_KEYWORDS,
     "Set the gravity acceleration (x,y,z)."},

    {"setTimeStep",(PyCFunction) pybullet_setTimeStep, METH_VARARGS|METH_KEYWORDS,
     "Set the amount of time to proceed at each call to stepSimulation. (unit "
     "is seconds, typically range is 0.01 or 0.001)"},



	{"setDefaultContactERP",  (PyCFunction) pybullet_setDefaultContactERP, METH_VARARGS| METH_KEYWORDS,
        "Set the amount of contact penetration Error Recovery Paramater "
        "(ERP) in each time step. \
		This is an tuning parameter to control resting contact stability. "
		"This value depends on the time step."},
    
	{ "setRealTimeSimulation",(PyCFunction) pybullet_setRealTimeSimulation, METH_VARARGS| METH_KEYWORDS,
	"Enable or disable real time simulation (using the real time clock,"
	" RTC) in the physics server. Expects one integer argument, 0 or 1" },

	{ "setPhysicsEngineParameter", (PyCFunction)pybullet_setPhysicsEngineParameter, METH_VARARGS | METH_KEYWORDS,
	"Set some internal physics engine parameter, such as cfm or erp etc." },

	{ "setInternalSimFlags", (PyCFunction)pybullet_setInternalSimFlags, METH_VARARGS| METH_KEYWORDS,
	"This is for experimental purposes, use at own risk, magic may or not happen"},
	
    {"loadURDF", (PyCFunction) pybullet_loadURDF, METH_VARARGS | METH_KEYWORDS,
     "Create a multibody by loading a URDF file."},

    {"loadSDF", (PyCFunction)pybullet_loadSDF, METH_VARARGS| METH_KEYWORDS,
     "Load multibodies from an SDF file."},

	 { "loadBullet", (PyCFunction)pybullet_loadBullet, METH_VARARGS| METH_KEYWORDS,
	"Restore the full state of the world from a .bullet file." },

	{ "saveBullet", (PyCFunction)pybullet_saveBullet, METH_VARARGS| METH_KEYWORDS,
	"Save the full state of the world to a .bullet file." },

	 { "loadMJCF",(PyCFunction) pybullet_loadMJCF, METH_VARARGS| METH_KEYWORDS,
	"Load multibodies from an MJCF file." },

	  {"createConstraint", (PyCFunction)pybullet_createUserConstraint, METH_VARARGS | METH_KEYWORDS,
		"Create a constraint between two bodies. Returns a (int) unique id, if successfull."
     },

	{"changeConstraint", (PyCFunction)pybullet_changeUserConstraint, METH_VARARGS | METH_KEYWORDS,
     "Change some parameters of an existing constraint, such as the child pivot or child frame orientation, using its unique id."
     },


	{"removeConstraint", (PyCFunction)pybullet_removeUserConstraint, METH_VARARGS | METH_KEYWORDS,
     "Remove a constraint using its unique id."
     },
	
	 { "enableJointForceTorqueSensor", (PyCFunction)pybullet_enableJointForceTorqueSensor, METH_VARARGS | METH_KEYWORDS,
		"Enable or disable a joint force/torque sensor measuring the joint reaction forces."
	 },

	{"saveWorld", (PyCFunction)pybullet_saveWorld, METH_VARARGS| METH_KEYWORDS,
     "Save a approximate Python file to reproduce the current state of the world: saveWorld"
	"(filename). (very preliminary and approximately)"},

	{"getNumBodies", (PyCFunction)pybullet_getNumBodies, METH_VARARGS| METH_KEYWORDS,
     "Get the number of bodies in the simulation."},

	{"getBodyUniqueId", (PyCFunction)pybullet_getBodyUniqueId, METH_VARARGS| METH_KEYWORDS,
     "getBodyUniqueId is used after connecting to server with existing bodies."
	 "Get the unique id of the body, given a integer range [0.. number of bodies)."},

	{"getBodyInfo",(PyCFunction) pybullet_getBodyInfo, METH_VARARGS | METH_KEYWORDS,
     "Get the body info, given a body unique id."},
	
    {"getNumConstraints", (PyCFunction)pybullet_getNumConstraints, METH_VARARGS| METH_KEYWORDS,
        "Get the number of user-created constraints in the simulation."},

    {"getConstraintInfo",(PyCFunction) pybullet_getConstraintInfo, METH_VARARGS | METH_KEYWORDS,
     "Get the user-created constraint info, given a constraint unique id."},
    
	{"getConstraintUniqueId", (PyCFunction)pybullet_getBodyUniqueId, METH_VARARGS| METH_KEYWORDS,
	"Get the unique id of the constraint, given a integer index in range [0.. number of constraints)."},
    
    {"getBasePositionAndOrientation",(PyCFunction) pybullet_getBasePositionAndOrientation,
     METH_VARARGS | METH_KEYWORDS,
     "Get the world position and orientation of the base of the object. "
     "(x,y,z) position vector and (x,y,z,w) quaternion orientation."},

    {"resetBasePositionAndOrientation",
     (PyCFunction) pybullet_resetBasePositionAndOrientation, METH_VARARGS| METH_KEYWORDS,
     "Reset the world position and orientation of the base of the object "
     "instantaneously, not through physics simulation. (x,y,z) position vector "
     "and (x,y,z,w) quaternion orientation."},

	 { "getBaseVelocity", (PyCFunction)pybullet_getBaseVelocity,
		METH_VARARGS| METH_KEYWORDS,
		"Get the linear and angular velocity of the base of the object "
		" in world space coordinates. "
		"(x,y,z) linear velocity vector and (x,y,z) angular velocity vector." },

	{ "resetBaseVelocity", (PyCFunction)pybullet_resetBaseVelocity, METH_VARARGS | METH_KEYWORDS,
	"Reset the linear and/or angular velocity of the base of the object "
	" in world space coordinates. "
	"linearVelocity (x,y,z) and angularVelocity (x,y,z)." },

    {"getNumJoints", (PyCFunction)pybullet_getNumJoints, METH_VARARGS| METH_KEYWORDS,
     "Get the number of joints for an object."},

    {"getJointInfo", (PyCFunction)pybullet_getJointInfo, METH_VARARGS| METH_KEYWORDS,
     "Get the name and type info for a joint on a body."},

    {"getJointState",(PyCFunction) pybullet_getJointState, METH_VARARGS| METH_KEYWORDS,
     "Get the state (position, velocity etc) for a joint on a body."},

    {"getLinkState", (PyCFunction)pybullet_getLinkState, METH_VARARGS| METH_KEYWORDS,
     "Provides extra information such as the Cartesian world coordinates"
     " center of mass (COM) of the link, relative to the world reference"
     " frame."},

    {"resetJointState",(PyCFunction) pybullet_resetJointState, METH_VARARGS| METH_KEYWORDS,
     "Reset the state (position, velocity etc) for a joint on a body "
     "instantaneously, not through physics simulation."},

    {"setJointMotorControl",(PyCFunction) pybullet_setJointMotorControl, METH_VARARGS,
	"This (obsolete) method cannot select non-zero physicsClientId, use setJointMotorControl2 instead."
	 "Set a single joint motor control mode and desired target value. There is "
     "no immediate state change, stepSimulation will process the motors."},

	{"setJointMotorControl2",(PyCFunction) pybullet_setJointMotorControl2, METH_VARARGS| METH_KEYWORDS,
     "Set a single joint motor control mode and desired target value. There is "
     "no immediate state change, stepSimulation will process the motors."},

    {"applyExternalForce",(PyCFunction) pybullet_applyExternalForce, METH_VARARGS| METH_KEYWORDS,
     "for objectUniqueId, linkIndex (-1 for base/root link), apply a force "
     "[x,y,z] at the a position [x,y,z], flag to select FORCE_IN_LINK_FRAME or "
     "WORLD_FRAME coordinates"},

    {"applyExternalTorque", (PyCFunction)pybullet_applyExternalTorque, METH_VARARGS| METH_KEYWORDS,
     "for objectUniqueId, linkIndex (-1 for base/root link) apply a torque "
     "[x,y,z] in Cartesian coordinates, flag to select TORQUE_IN_LINK_FRAME or "
     "WORLD_FRAME coordinates"},

    {"renderImage", pybullet_renderImageObsolete, METH_VARARGS,
     "obsolete, please use getCameraImage and getViewProjectionMatrices instead"
     },

	 { "getCameraImage",(PyCFunction)pybullet_getCameraImage, METH_VARARGS| METH_KEYWORDS,
			"Render an image (given the pixel resolution width, height, camera viewMatrix "
			", projectionMatrix, lightDirection, lightColor, lightDistance, shadow, lightAmbientCoeff, lightDiffuseCoeff, and lightSpecularCoeff), and return the "
			"8-8-8bit RGB pixel data and floating point depth values"
#ifdef PYBULLET_USE_NUMPY
			" as NumPy arrays"
#endif
	 },

	 { "computeViewMatrix", (PyCFunction)pybullet_computeViewMatrix, METH_VARARGS | METH_KEYWORDS,
		"Compute a camera viewmatrix from camera eye,  target position and up vector "
	 },

	{ "computeViewMatrixFromYawPitchRoll",(PyCFunction)pybullet_computeViewMatrixFromYawPitchRoll, METH_VARARGS | METH_KEYWORDS,
		"Compute a camera viewmatrix from camera eye,  target position and up vector "
	},
	
	{ "computeProjectionMatrix", (PyCFunction)pybullet_computeProjectionMatrix, METH_VARARGS | METH_KEYWORDS,
		"Compute a camera projection matrix from screen left/right/bottom/top/near/far values"
	},

	{ "computeProjectionMatrixFOV", (PyCFunction)pybullet_computeProjectionMatrixFOV, METH_VARARGS | METH_KEYWORDS,
		"Compute a camera projection matrix from fov, aspect ratio, near, far values"
	},

    {"getContactPoints", (PyCFunction)pybullet_getContactPointData, METH_VARARGS | METH_KEYWORDS,
     "Return existing contact points after the stepSimulation command. "
     "Optional arguments one or two object unique "
     "ids, that need to be involved in the contact."},

	 {"getClosestPoints", (PyCFunction)pybullet_getClosestPointData, METH_VARARGS | METH_KEYWORDS,
     "Compute the closest points between two objects, if the distance is below a given threshold."
     "Input is two objects unique ids and distance threshold."
     },

	 { "getOverlappingObjects", (PyCFunction)pybullet_getOverlappingObjects, METH_VARARGS | METH_KEYWORDS,
		"Return all the objects that have overlap with a given "
		"axis-aligned bounding box volume (AABB)."
		"Input are two vectors defining the AABB in world space [min_x,min_y,min_z],[max_x,max_y,max_z]."
	 },


	 { "addUserDebugLine", (PyCFunction)pybullet_addUserDebugLine, METH_VARARGS | METH_KEYWORDS,
	 "Add a user debug draw line with lineFrom[3], lineTo[3], lineColorRGB[3], lineWidth, lifeTime. "
	  "A lifeTime of 0 means permanent until removed. Returns a unique id for the user debug item."
	 },


	 { "addUserDebugText", (PyCFunction)pybullet_addUserDebugText, METH_VARARGS | METH_KEYWORDS,
	 "Add a user debug draw line with text, textPosition[3], textSize and lifeTime in seconds "
	 "A lifeTime of 0 means permanent until removed. Returns a unique id for the user debug item."
	  },

	  { "addUserDebugParameter", (PyCFunction)pybullet_addUserDebugParameter, METH_VARARGS | METH_KEYWORDS,
	 "Add a user debug parameter, such as a slider, that can be controlled using a GUI."
	  },
	 { "readUserDebugParameter", (PyCFunction)pybullet_readUserDebugParameter, METH_VARARGS | METH_KEYWORDS,
	 "Read the current value of a user debug parameter, given the user debug item unique id."
	  },

	 { "removeUserDebugItem", (PyCFunction)pybullet_removeUserDebugItem, METH_VARARGS | METH_KEYWORDS,
	 "remove a user debug draw item, giving its unique id"
	  },


	 { "removeAllUserDebugItems", (PyCFunction)pybullet_removeAllUserDebugItems, METH_VARARGS | METH_KEYWORDS,
	 "remove all user debug draw items"
	  },

	  {	"setDebugObjectColor", (PyCFunction)pybullet_setDebugObjectColor, METH_VARARGS | METH_KEYWORDS,
		"Override the wireframe debug drawing color for a particular object unique id / link index."
		"If you ommit the color, the custom color will be removed."
	  },


    {"getVisualShapeData", (PyCFunction)pybullet_getVisualShapeData, METH_VARARGS| METH_KEYWORDS,
	"Return the visual shape information for one object." },
		 
    {"resetVisualShapeData", (PyCFunction)pybullet_resetVisualShapeData, METH_VARARGS| METH_KEYWORDS,
        "Reset part of the visual shape information for one object." },
    
    {"loadTexture", (PyCFunction)pybullet_loadTexture, METH_VARARGS| METH_KEYWORDS,
        "Load texture file." },
    
    {"getQuaternionFromEuler", pybullet_getQuaternionFromEuler, METH_VARARGS,
     "Convert Euler [roll, pitch, yaw] as in URDF/SDF convention, to "
     "quaternion [x,y,z,w]"},

    {"getEulerFromQuaternion", pybullet_getEulerFromQuaternion, METH_VARARGS,
     "Convert quaternion [x,y,z,w] to Euler [roll, pitch, yaw] as in URDF/SDF "
     "convention"},

	 {"getMatrixFromQuaternion", pybullet_getMatrixFromQuaternion,METH_VARARGS,
		"Compute the 3x3 matrix from a quaternion, as a list of 9 values (row-major)"},

    {"calculateInverseDynamics", (PyCFunction)pybullet_calculateInverseDynamics, METH_VARARGS| METH_KEYWORDS,
     "Given an object id, joint positions, joint velocities and joint "
     "accelerations, compute the joint forces using Inverse Dynamics"},
    
    {"calculateInverseKinematics", (PyCFunction)pybullet_calculateInverseKinematics,
        METH_VARARGS| METH_KEYWORDS,
        "Inverse Kinematics bindings: Given an object id, "
        "current joint positions and target position"
        " for the end effector,"
        "compute the inverse kinematics and return the new joint state"},
    
	{	"getVREvents", (PyCFunction)pybullet_getVREvents, METH_VARARGS | METH_KEYWORDS,
		"Get Virtual Reality events, for example to track VR controllers position/buttons"
	  },
	{	"setVRCameraState", (PyCFunction)pybullet_setVRCameraState, METH_VARARGS | METH_KEYWORDS,
		"Set properties of the VR Camera such as its root transform "
		"for teleporting or to track objects (camera inside a vehicle for example)."
	  },
	  {	"startStateLogging", (PyCFunction)pybullet_startStateLogging, METH_VARARGS | METH_KEYWORDS,
		"Start logging of state, such as robot base position, orientation, joint positions etc. "
		"Specify loggingType (STATE_LOG_MINITAUR, STATE_LOG_GENERIC_ROBOT, STATE_LOG_VR_CONTROLLERS etc), "
		"fileName, optional objectUniqueId. Function returns int loggingUniqueId"
	  },
	  {	"stopStateLogging", (PyCFunction)pybullet_stopStateLogging, METH_VARARGS | METH_KEYWORDS,
		"Stop logging of robot state, given a loggingUniqueId."
	  },
	{	"rayTest", (PyCFunction)pybullet_rayTest, METH_VARARGS | METH_KEYWORDS,
				"Cast a ray and return the first object hit, if any. "
				"Takes two arguments (from position [x,y,z] and to position [x,y,z] in Cartesian world coordinates"
	  },


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
  PyModule_AddIntConstant(m, "UDP", eCONNECT_UDP);        // user read

  PyModule_AddIntConstant(m, "JOINT_REVOLUTE", eRevoluteType);        // user read
  PyModule_AddIntConstant(m, "JOINT_PRISMATIC", ePrismaticType);        // user read
  PyModule_AddIntConstant(m, "JOINT_SPHERICAL", eSphericalType);        // user read
  PyModule_AddIntConstant(m, "JOINT_PLANAR", ePlanarType);        // user read
  PyModule_AddIntConstant(m, "JOINT_FIXED", eFixedType);        // user read
  PyModule_AddIntConstant(m, "JOINT_POINT2POINT", ePoint2PointType);        // user read

  PyModule_AddIntConstant(m, "SENSOR_FORCE_TORQUE", eSensorForceTorqueType);        // user read


  PyModule_AddIntConstant(m, "TORQUE_CONTROL", CONTROL_MODE_TORQUE);
  PyModule_AddIntConstant(m, "VELOCITY_CONTROL",
                          CONTROL_MODE_VELOCITY);  // user read
  PyModule_AddIntConstant(m, "POSITION_CONTROL",
                          CONTROL_MODE_POSITION_VELOCITY_PD);  // user read

  PyModule_AddIntConstant(m, "LINK_FRAME", EF_LINK_FRAME);
  PyModule_AddIntConstant(m, "WORLD_FRAME", EF_WORLD_FRAME);

  PyModule_AddIntConstant(m, "CONTACT_REPORT_EXISTING", CONTACT_QUERY_MODE_REPORT_EXISTING_CONTACT_POINTS);
  PyModule_AddIntConstant(m, "CONTACT_RECOMPUTE_CLOSEST", CONTACT_QUERY_MODE_COMPUTE_CLOSEST_POINTS);

  PyModule_AddIntConstant(m, "VR_BUTTON_IS_DOWN", eButtonIsDown);
  PyModule_AddIntConstant(m, "VR_BUTTON_WAS_TRIGGERED", eButtonTriggered);
  PyModule_AddIntConstant(m, "VR_BUTTON_WAS_RELEASED", eButtonReleased);
  
  PyModule_AddIntConstant(m, "VR_MAX_CONTROLLERS", MAX_VR_CONTROLLERS);
  PyModule_AddIntConstant(m, "VR_MAX_BUTTONS", MAX_VR_BUTTONS);

  PyModule_AddIntConstant(m, "STATE_LOGGING_MINITAUR", STATE_LOGGING_MINITAUR);
  PyModule_AddIntConstant(m, "STATE_LOGGING_GENERIC_ROBOT", STATE_LOGGING_GENERIC_ROBOT);
  PyModule_AddIntConstant(m, "STATE_LOGGING_VR_CONTROLLERS", STATE_LOGGING_VR_CONTROLLERS);

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
