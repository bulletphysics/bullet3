#include "../SharedMemory/SharedMemoryInProcessPhysicsC_API.h"
#include "../SharedMemory/PhysicsClientC_API.h"

#ifdef __APPLE__
#include <Python/Python.h>
#else
#include <Python.h>
#endif

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

return PyLong_FromLong(1);
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
        sm = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
    }
    
    return PyLong_FromLong(1);
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
    
    return PyLong_FromLong(1);
}


static PyObject *
pybullet_loadURDF(PyObject* self, PyObject* args)
{
    if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }
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
        printf("urdf filename = %s\n", urdfFileName);
        b3SharedMemoryStatusHandle statusHandle;
        int statusType;
        b3SharedMemoryCommandHandle command = b3LoadUrdfCommandInit(sm, urdfFileName);

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
//        ASSERT_EQ(b3GetStatusType(statusHandle), CMD_RESET_SIMULATION_COMPLETED);
    }
	return PyLong_FromLong(1);

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
    
    {NULL, NULL, 0, NULL}        /* Sentinel */
};


PyMODINIT_FUNC
initpybullet(void)
{

    PyObject *m;
    m = Py_InitModule("pybullet", SpamMethods);
    if (m == NULL)
        return;

    SpamError = PyErr_NewException("pybullet.error", NULL, NULL);
    Py_INCREF(SpamError);
    PyModule_AddObject(m, "error", SpamError);
}


