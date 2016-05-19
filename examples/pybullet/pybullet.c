#include "../SharedMemory/SharedMemoryInProcessPhysicsC_API.h"
#include "../SharedMemory/PhysicsClientC_API.h"
#include "../SharedMemory/PhysicsDirectC_API.h"
#include "../SharedMemory/SharedMemoryInProcessPhysicsC_API.h"


#ifdef __APPLE__
#include <Python/Python.h>
#else
#include <Python.h>
#endif

enum eCONNECT_METHOD
{
	eCONNECT_GUI=1,
	eCONNECT_DIRECT=2,
	eCONNECT_SHARED_MEMORY=3,
};

static PyObject *SpamError;
static  b3PhysicsClientHandle sm=0;


static PyObject *
pybullet_stepSimulation(PyObject *self, PyObject *args)
{
    if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }

    {
        b3SharedMemoryStatusHandle statusHandle;
        int statusType;

        if (b3CanSubmitCommand(sm))
        {
            statusHandle = b3SubmitClientCommandAndWaitStatus(sm, b3InitStepSimulationCommand(sm));
            statusType = b3GetStatusType(statusHandle);
        } 
    }

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject *
pybullet_connectPhysicsServer(PyObject *self, PyObject *args)
{
    if (0!=sm)
    {
        PyErr_SetString(SpamError, "Already connected to physics server, disconnect first.");
        return NULL;
    }
    
    {
		int method=eCONNECT_GUI;
		if (!PyArg_ParseTuple(args, "i", &method))
		{
			PyErr_SetString(SpamError, "connectPhysicsServer expected argument  eCONNECT_GUI, eCONNECT_DIRECT or eCONNECT_SHARED_MEMORY");
	        return NULL;
		}

		switch (method)
		{
			case eCONNECT_GUI:
			{
				int argc=0;
				char* argv[1]={0};

#ifdef __APPLE__
				sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
#else
				sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
#endif
				break;
			}
			case eCONNECT_DIRECT:
			{
				sm = b3ConnectPhysicsDirect();
				break;
			}
			case eCONNECT_SHARED_MEMORY:
			{
				sm = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
				break;
			}


			default:
				{
					PyErr_SetString(SpamError, "connectPhysicsServer unexpected argument");
					return NULL;
				}
		};
        
	
    }
    
	Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
pybullet_disconnectPhysicsServer(PyObject *self, PyObject *args)
{
    if (0==sm)
    {
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


static PyObject *
pybullet_loadURDF(PyObject* self, PyObject* args)
{
    
	int size= PySequence_Size(args);
	
	int bodyIndex = -1;
	const char* urdfFileName=0;
	float startPosX =0;
    float startPosY =0;
    float startPosZ = 1;
	float startOrnX = 0;
	float startOrnY = 0;
	float startOrnZ = 0;
	float startOwnW = 1;
	printf("size=%d\n", size);
	if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }
	if (size==1)
	{
		if (!PyArg_ParseTuple(args, "s", &urdfFileName))
       	 	return NULL;            
	}
	if (size == 4)
	{
	  if (!PyArg_ParseTuple(args, "sfff", &urdfFileName,
		&startPosX,&startPosY,&startPosZ))
                return NULL;
        }
	if (size==7)
	{
		if (!PyArg_ParseTuple(args, "sfffffff", &urdfFileName,
                &startPosX,startPosY,&startPosZ,
		&startOrnX,&startOrnY,&startOrnZ, &startOwnW))
                return NULL;
	}
    {
        
        b3SharedMemoryStatusHandle statusHandle;
        int statusType;
        b3SharedMemoryCommandHandle command = b3LoadUrdfCommandInit(sm, urdfFileName);
		printf("urdf filename = %s\n", urdfFileName);
        //setting the initial position, orientation and other arguments are optional
        b3LoadUrdfCommandSetStartPosition(command, startPosX,startPosY,startPosZ);
        statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
        statusType = b3GetStatusType(statusHandle);
        if (statusType!=CMD_URDF_LOADING_COMPLETED)
		{
			PyErr_SetString(SpamError, "Cannot load URDF file.");
			return NULL;
		}
        bodyIndex = b3GetStatusBodyIndex(statusHandle);
    }
	return PyLong_FromLong(bodyIndex);
}
static PyObject *
pybullet_resetSimulation(PyObject* self, PyObject* args)
{
    if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }

    {
        b3SharedMemoryStatusHandle statusHandle;
        statusHandle = b3SubmitClientCommandAndWaitStatus(sm, b3InitResetSimulationCommand(sm));
    }
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject *
pybullet_setGravity(PyObject* self, PyObject* args)
{
    if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }

	{
		float gravX=0;
		float gravY=0;
		float gravZ=-10;
		int ret;

		b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
		b3SharedMemoryStatusHandle statusHandle;

		if (!PyArg_ParseTuple(args, "fff", &gravX,&gravY,&gravZ))
		{
			PyErr_SetString(SpamError, "setGravity expected (x,y,z) values.");
			return NULL;
		}
		ret = b3PhysicsParamSetGravity(command,  gravX,gravY, gravZ);
		//ret = b3PhysicsParamSetTimeStep(command,  timeStep);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		//ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
	}

	if (1)
	{
		PyObject *pylist; 
		PyObject *item; 
		int i;
		int num=3;
		pylist = PyTuple_New(num);
		for (i = 0; i < num; i++) 
		{
			item = PyFloat_FromDouble(i);
			PyTuple_SetItem(pylist, i, item);
		}
		return pylist;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject *
pybullet_getNumJoints(PyObject* self, PyObject* args)
{
    if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }

	{
		int bodyIndex = -1;
		int numJoints=0;
		if (!PyArg_ParseTuple(args, "i", &bodyIndex ))
		{
			PyErr_SetString(SpamError, "Expected a body index (integer).");
			return NULL;
		}
		numJoints = b3GetNumJoints(sm,bodyIndex);

#if PY_MAJOR_VERSION >= 3
		return PyLong_FromLong(numJoints);
#else
		return PyInt_FromLong(numJoints);
#endif
	}
}


static PyMethodDef SpamMethods[] = {
    {"loadURDF",  pybullet_loadURDF, METH_VARARGS,
        "Create a multibody by loading a URDF file."},
    
    {"connect",  pybullet_connectPhysicsServer, METH_VARARGS,
        "Connect to an existing physics server (using shared memory by default)."},
	
    {"disconnect",  pybullet_disconnectPhysicsServer, METH_VARARGS,
        "Disconnect from the physics server."},

    {"resetSimulation",  pybullet_resetSimulation, METH_VARARGS,
        "Reset the simulation: remove all objects and start from an empty world."},
	
	{"stepSimulation",  pybullet_stepSimulation, METH_VARARGS,
        "Step the simulation using forward dynamics."},

	{"setGravity",  pybullet_setGravity, METH_VARARGS,
        "Set the gravity acceleration (x,y,z)."},
	
	{"getNumsetGravity",  pybullet_setGravity, METH_VARARGS,
        "Set the gravity acceleration (x,y,z)."},
	{
		"getNumJoints", pybullet_getNumJoints, METH_VARARGS,
        "Get the number of joints for an object."},
	
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

#if PY_MAJOR_VERSION >= 3
    static struct PyModuleDef moduledef = {
        PyModuleDef_HEAD_INIT,
        "pybullet",     /* m_name */
        "Python bindings for Bullet",  /* m_doc */
        -1,                  /* m_size */
        SpamMethods,    /* m_methods */
        NULL,                /* m_reload */
        NULL,                /* m_traverse */
        NULL,                /* m_clear */
        NULL,                /* m_free */
    };
#endif

PyMODINIT_FUNC
#if PY_MAJOR_VERSION >= 3
PyInit_pybullet(void)
#else
initpybullet(void)
#endif
{

    PyObject *m;
#if PY_MAJOR_VERSION >= 3
    m = PyModule_Create(&moduledef);
#else
    m = Py_InitModule3("pybullet",
        SpamMethods, "Python bindings for Bullet");
#endif

#if PY_MAJOR_VERSION >= 3
    if (m == NULL)
        return m;
#else
    if (m == NULL)
        return;
#endif
    
	
	PyModule_AddIntConstant (m, "SHARED_MEMORY", eCONNECT_SHARED_MEMORY); // user read
	PyModule_AddIntConstant (m, "DIRECT", eCONNECT_DIRECT); // user read 
	PyModule_AddIntConstant (m, "GUI", eCONNECT_GUI); // user read 

    SpamError = PyErr_NewException("pybullet.error", NULL, NULL);
    Py_INCREF(SpamError);
    PyModule_AddObject(m, "error", SpamError);
#if PY_MAJOR_VERSION >= 3
	return m;
#endif
}

