#include "../SharedMemory/SharedMemoryInProcessPhysicsC_API.h"
#include "../SharedMemory/PhysicsClientC_API.h"

#ifdef __APPLE__
#include <Python/Python.h>
#else
#include <Python.h>
#endif

static PyObject *SpamError;
static  b3PhysicsClientHandle sm;

static PyObject *
spam_step(PyObject *self, PyObject *args)
{

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
spam_loadURDF(PyObject* self, PyObject* args)
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
	printf("urdf filename = %s\n", urdfFileName);
	b3SharedMemoryStatusHandle statusHandle;
            int statusType;
             b3SharedMemoryCommandHandle command = b3LoadUrdfCommandInit(sm, urdfFileName);

            //setting the initial position, orientation and other arguments are optional
            int ret = b3LoadUrdfCommandSetStartPosition(command, startPosX,startPosY,startPosZ);
            statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
                        statusType = b3GetStatusType(statusHandle);
                        if (statusType!=CMD_URDF_LOADING_COMPLETED)
			{
				PyErr_SetString(SpamError, "Cannot load URDF file.");
				return NULL;				
			}
                        bodyIndex = b3GetStatusBodyIndex(statusHandle);
	return PyLong_FromLong(bodyIndex);
}

static PyMethodDef SpamMethods[] = {
    {"step",  spam_step, METH_VARARGS,
     "Step the simulation forward."},
{"loadURDF",  spam_loadURDF, METH_VARARGS,
     "Create a multibody by loading a URDF file."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};


PyMODINIT_FUNC
initpybullet(void)
{

	b3PhysicsClientHandle h;

    PyObject *m;
sm = b3ConnectSharedMemory(SHARED_MEMORY_KEY);

//#ifdef __APPLE__
//sm = b3CreateInProcessPhysicsServerAndConnectMainThread(0,0);
//#else
//sm = b3CreateInProcessPhysicsServerAndConnect(0,0);
//#endif
    m = Py_InitModule("pybullet", SpamMethods);
    if (m == NULL)
        return;

    SpamError = PyErr_NewException("pybullet.error", NULL, NULL);
    Py_INCREF(SpamError);
    PyModule_AddObject(m, "error", SpamError);
}


