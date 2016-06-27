#include "../SharedMemory/PhysicsClientC_API.h"
#include "../SharedMemory/PhysicsDirectC_API.h"
#include "../SharedMemory/SharedMemoryInProcessPhysicsC_API.h"



#ifdef __APPLE__
#include <Python/Python.h>
#else
#include <Python.h>
#endif


#if PY_MAJOR_VERSION >= 3
#define PyInt_FromLong PyLong_FromLong
#define PyString_FromString PyBytes_FromString
#endif 

enum eCONNECT_METHOD
{
	eCONNECT_GUI=1,
	eCONNECT_DIRECT=2,
	eCONNECT_SHARED_MEMORY=3,
};



static PyObject *SpamError;
static  b3PhysicsClientHandle sm=0;

// Step through one timestep of the simulation
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

// Load a URDF file indicating the links and joints of an object
// function can be called without arguments and will default
// to position (0,0,1) with orientation(0,0,0,1)
// els(x,y,z) or
// loadURDF(pos_x, pos_y, pos_z, orn_x, orn_y, orn_z, orn_w)
static PyObject *
pybullet_loadURDF(PyObject* self, PyObject* args)
{
    
	int size= PySequence_Size(args);
	
	int bodyIndex = -1;
	const char* urdfFileName="";
	
	float startPosX =0;
  float startPosY =0;
  float startPosZ = 0;
	float startOrnX = 0;
	float startOrnY = 0;
	float startOrnZ = 0;
	float startOrnW = 1;

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
	if (size==8)
	{
		if (!PyArg_ParseTuple(args, "sfffffff", &urdfFileName,
		&startPosX,&startPosY,&startPosZ,
			&startOrnX,&startOrnY,&startOrnZ, &startOrnW))
                return NULL;
	}
    {
      // printf("(%f, %f, %f) (%f, %f, %f, %f)\n", startPosX,startPosY,startPosZ,startOrnX, startOrnY,startOrnZ, startOrnW);
        
        b3SharedMemoryStatusHandle statusHandle;
        int statusType;
        b3SharedMemoryCommandHandle command = b3LoadUrdfCommandInit(sm, urdfFileName);

        //setting the initial position, orientation and other arguments are optional
        b3LoadUrdfCommandSetStartPosition(command, startPosX,startPosY,startPosZ);
        b3LoadUrdfCommandSetStartOrientation(command, startOrnX, startOrnY,startOrnZ, startOrnW );
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

static float pybullet_internalGetFloatFromSequence(PyObject* seq, int index)
{
    float v = 0.f;
    PyObject* item;
    
    if (PyList_Check(seq))
    {
        item = PyList_GET_ITEM(seq, index);
        v = PyFloat_AsDouble(item);
    }
    else
    {
        item = PyTuple_GET_ITEM(seq,index);
        v = PyFloat_AsDouble(item);
    }
    return v;
}


#define MAX_SDF_BODIES 512

static PyObject*
pybullet_loadSDF(PyObject* self, PyObject* args)
{
	const char* sdfFileName="";
	int size= PySequence_Size(args);
	int numBodies  = 0;
	int i;
	int bodyIndicesOut[MAX_SDF_BODIES];
	PyObject* pylist=0;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle commandHandle;

	if (0==sm)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (size==1)
	{
		if (!PyArg_ParseTuple(args, "s", &sdfFileName))
		return NULL;
	}

	commandHandle = b3LoadSdfCommandInit(sm, sdfFileName);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType!=CMD_SDF_LOADING_COMPLETED)
	{
		PyErr_SetString(SpamError, "Cannot load SDF file.");
		return NULL;
	}

	numBodies = b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES);
	if (numBodies > MAX_SDF_BODIES)
	{
		PyErr_SetString(SpamError, "SDF exceeds body capacity");
		return NULL;
	}

	pylist = PyTuple_New(numBodies);

	if (numBodies >0 && numBodies <= MAX_SDF_BODIES)	
	{
		for (i=0;i<numBodies;i++)
		{
			PyTuple_SetItem(pylist,i,PyInt_FromLong(bodyIndicesOut[i]));
		}
	}
	return pylist;
}

// Reset the simulation to remove all loaded objects
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

static PyObject* pybullet_setJointMotorControl(PyObject* self, PyObject* args)
{
    //todo(erwincoumans): set max forces, kp, kd

    int size;
    int bodyIndex, jointIndex, controlMode;
    double targetValue=0;
    double maxForce=100000;
    double gains=0.1;
    int valid = 0;
     
     
    if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }
    
    size= PySequence_Size(args);
    
    if (size==4)
    {
        
        if (!PyArg_ParseTuple(args, "iiid", &bodyIndex, &jointIndex, &controlMode, &targetValue))
        {
            PyErr_SetString(SpamError, "Error parsing arguments");
            return NULL;
        }
        valid = 1;
    }
    if (size==5)
    {
        
        if (!PyArg_ParseTuple(args, "iiidd", &bodyIndex, &jointIndex, &controlMode, &targetValue, &maxForce))
        {
            PyErr_SetString(SpamError, "Error parsing arguments");
            return NULL;
        }
        valid = 1;
    }
    if (size==6)
    {
        
        if (!PyArg_ParseTuple(args, "iiiddd", &bodyIndex, &jointIndex, &controlMode, &targetValue, &maxForce, &gains))
        {
            PyErr_SetString(SpamError, "Error parsing arguments");
            return NULL;
        }
        valid = 1;
    }
    
    
    if (valid)
    {
        int numJoints;
        b3SharedMemoryCommandHandle commandHandle;
        b3SharedMemoryStatusHandle statusHandle;
        struct b3JointInfo info;

        numJoints = b3GetNumJoints(sm,bodyIndex);
        if ((jointIndex >= numJoints) || (jointIndex < 0))
        {
            PyErr_SetString(SpamError, "Joint index out-of-range.");
            return NULL;
        }
        
        if ((controlMode != CONTROL_MODE_VELOCITY) &&
            (controlMode != CONTROL_MODE_TORQUE) &&
            (controlMode != CONTROL_MODE_POSITION_VELOCITY_PD))
        {
            PyErr_SetString(SpamError, "Illegral control mode.");
            return NULL;
        }
        
        commandHandle = b3JointControlCommandInit2(sm, bodyIndex,controlMode);
         
        b3GetJointInfo(sm, bodyIndex, jointIndex, &info);
        
        switch (controlMode)
        {
            case CONTROL_MODE_VELOCITY:
            {
                b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex, targetValue);
                double kd = gains;
                b3JointControlSetKd(commandHandle,info.m_uIndex,kd);
                b3JointControlSetMaximumForce(commandHandle,info.m_uIndex,maxForce);
                break;
            }

            case CONTROL_MODE_TORQUE:
            {
                b3JointControlSetDesiredForceTorque(commandHandle, info.m_uIndex, targetValue);
                break;
            }
                    
            case CONTROL_MODE_POSITION_VELOCITY_PD:
            {
                b3JointControlSetDesiredPosition( commandHandle, info.m_qIndex, targetValue);
                double kp = gains;
                b3JointControlSetKp(commandHandle,info.m_uIndex,kp);
                b3JointControlSetMaximumForce(commandHandle,info.m_uIndex,maxForce);
                break;
            }
            default:
            {
            }
        };
        
        statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
        
        Py_INCREF(Py_None);
        return Py_None;
    }
    PyErr_SetString(SpamError, "error in setJointControl.");
    return NULL;
}


// Set the gravity of the world with (x, y, z) arguments
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

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject *
pybullet_setTimeStep(PyObject* self, PyObject* args)
{
    if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }
    
    {
        double timeStep=0.001;
        int ret;
        
        b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
        b3SharedMemoryStatusHandle statusHandle;
        
        if (!PyArg_ParseTuple(args, "d", &timeStep))
        {
            PyErr_SetString(SpamError, "setTimeStep expected a single value (double).");
            return NULL;
        }
        ret = b3PhysicsParamSetTimeStep(command,  timeStep);
        statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
        //ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
    }
    
    Py_INCREF(Py_None);
    return Py_None;
}



// Internal function used to get the base position and orientation
// Orientation is returned in quaternions
static void pybullet_internalGetBasePositionAndOrientation(int bodyIndex, double basePosition[3],double baseOrientation[3])
{
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
            int i;
            b3GetStatusActualState(status_handle, 0/* body_unique_id */,
                               0/* num_degree_of_freedom_q */,
                               0/* num_degree_of_freedom_u */, 0 /*root_local_inertial_frame*/,
                               &actualStateQ , 0 /* actual_state_q_dot */,
                               0 /* joint_reaction_forces */);

            // printf("joint reaction forces=");
            // for (i=0; i < (sizeof(jointReactionForces)/sizeof(double)); i++) {
            //   printf("%f ", jointReactionForces[i]);
            // }
            //now, position x,y,z = actualStateQ[0],actualStateQ[1],actualStateQ[2]
            //and orientation x,y,z,w = actualStateQ[3],actualStateQ[4],actualStateQ[5],actualStateQ[6]
            basePosition[0] = actualStateQ[0];
            basePosition[1] = actualStateQ[1];
            basePosition[2] = actualStateQ[2];
            
            baseOrientation[0] = actualStateQ[3];
            baseOrientation[1] = actualStateQ[4];
            baseOrientation[2] = actualStateQ[5];
            baseOrientation[3] = actualStateQ[6];
            
		}
	}
}

// Get the positions (x,y,z) and orientation (x,y,z,w) in quaternion
// values for the base link of your object
// Object is retrieved based on body index, which is the order
// the object was loaded into the simulation (0-based)
static PyObject *
pybullet_getBasePositionAndOrientation(PyObject* self, PyObject* args)
{
	int bodyIndex = -1;
	double basePosition[3];
    double baseOrientation[4];
	PyObject *pylistPos;
    PyObject *pylistOrientation;

    if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }
    
    if (!PyArg_ParseTuple(args, "i", &bodyIndex ))
    {
        PyErr_SetString(SpamError, "Expected a body index (integer).");
        return NULL;
    }

    pybullet_internalGetBasePositionAndOrientation(bodyIndex,basePosition,baseOrientation);
    
    {
    
        PyObject *item; 
        int i;
        int num=3;
        pylistPos = PyTuple_New(num);
        for (i = 0; i < num; i++) 
        {
            item = PyFloat_FromDouble(basePosition[i]);
            PyTuple_SetItem(pylistPos, i, item);
        }
    
    }

    {
        
        PyObject *item; 
        int i;
        int num=4;
        pylistOrientation = PyTuple_New(num);
        for (i = 0; i < num; i++) 
        {
            item = PyFloat_FromDouble(baseOrientation[i]);
            PyTuple_SetItem(pylistOrientation, i, item);
        }
        
    }
    
    {
        PyObject *pylist; 
        pylist = PyTuple_New(2);
        PyTuple_SetItem(pylist,0,pylistPos);
        PyTuple_SetItem(pylist,1,pylistOrientation);
        return pylist;
    }
    
}

// Return the number of joints in an object based on
// body index; body index is based on order of sequence
// the object is loaded into simulation
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

// Initalize all joint positions given a list of values
static PyObject*
pybullet_resetJointState(PyObject* self, PyObject* args)
{
	int size;
	if (0==sm)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}


	size= PySequence_Size(args);
   
	if (size==3)
	{
        int bodyIndex;
        int jointIndex;
        double targetValue;
        
        if (PyArg_ParseTuple(args, "iid", &bodyIndex, &jointIndex, &targetValue))
        {
            b3SharedMemoryCommandHandle commandHandle;
            b3SharedMemoryStatusHandle statusHandle;
            int numJoints;
            
            numJoints = b3GetNumJoints(sm,bodyIndex);
            if ((jointIndex >= numJoints) || (jointIndex < 0))
            {
                PyErr_SetString(SpamError, "Joint index out-of-range.");
                return NULL;
            }
            
            commandHandle = b3CreatePoseCommandInit(sm, bodyIndex);

            b3CreatePoseCommandSetJointPosition(sm, commandHandle,  jointIndex, targetValue);

            statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
            Py_INCREF(Py_None);
            return Py_None;
        }

    }
    PyErr_SetString(SpamError, "error in resetJointState.");
    return NULL;
}

// Reset the position and orientation of the base/root link, position [x,y,z] and orientation quaternion [x,y,z,w]
static PyObject*
pybullet_resetBasePositionAndOrientation(PyObject* self, PyObject* args)
{
    int size;
    if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }
    
    
    size= PySequence_Size(args);
    
    if (size==3)
    {
        int bodyIndex;
        PyObject* posObj;
        PyObject* ornObj;
        double pos[3];
        double orn[4];//as a quaternion
        
        if (PyArg_ParseTuple(args, "iOO", &bodyIndex, &posObj, &ornObj))
        {
            b3SharedMemoryCommandHandle commandHandle;
            b3SharedMemoryStatusHandle statusHandle;
            
            {
                PyObject* seq;
                int len,i;
                seq = PySequence_Fast(posObj, "expected a sequence");
                len = PySequence_Size(posObj);
                if (len==3)
                {
                    for (i = 0; i < 3; i++)
                    {
                        pos[i] = pybullet_internalGetFloatFromSequence(seq,i);
                    }
                } else
                {
                    PyErr_SetString(SpamError, "position needs a 3 coordinates [x,y,z].");
                    Py_DECREF(seq);
                    return NULL;
                }
                 Py_DECREF(seq);
            }

            {
                PyObject* seq;
                int len,i;
                seq = PySequence_Fast(ornObj, "expected a sequence");
                len = PySequence_Size(ornObj);
                if (len==4)
                {
                    for (i = 0; i < 4; i++)
                    {
                        orn[i] = pybullet_internalGetFloatFromSequence(seq,i);
                    }
                } else
                {
                    PyErr_SetString(SpamError, "orientation needs a 4 coordinates, quaternion [x,y,z,w].");
                    Py_DECREF(seq);
                    return NULL;
                }
                Py_DECREF(seq);
            }
    
            commandHandle = b3CreatePoseCommandInit(sm, bodyIndex);
            
            b3CreatePoseCommandSetBasePosition( commandHandle, pos[0],pos[1],pos[2]);
            b3CreatePoseCommandSetBaseOrientation( commandHandle, orn[0],orn[1],orn[2],orn[3]);

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
static PyObject*
pybullet_getJointInfo(PyObject* self, PyObject* args)
{
	  PyObject *pyListJointInfo; 
  
  struct b3JointInfo info;
  
  int bodyIndex = -1;
  int jointIndex = -1;
  int jointInfoSize = 8; //size of struct b3JointInfo
    
  int size= PySequence_Size(args);

	if (0==sm)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}



  if (size==2) // get body index and joint index
  {
   if (PyArg_ParseTuple(args, "ii", &bodyIndex, &jointIndex))
  	{
      
      // printf("body index = %d, joint index =%d\n", bodyIndex, jointIndex);

    b3SharedMemoryCommandHandle cmd_handle =
            b3RequestActualStateCommandInit(sm, bodyIndex);
        b3SharedMemoryStatusHandle status_handle =
                b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);
                  
     pyListJointInfo = PyTuple_New(jointInfoSize);

     b3GetJointInfo(sm, bodyIndex, jointIndex, &info);
     
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
      PyTuple_SetItem(pyListJointInfo, 0, 
      PyInt_FromLong(info.m_jointIndex));
      PyTuple_SetItem(pyListJointInfo, 1, 
        PyString_FromString(info.m_jointName));
      PyTuple_SetItem(pyListJointInfo, 2, 
        PyInt_FromLong(info.m_jointType));    
      PyTuple_SetItem(pyListJointInfo, 3, 
        PyInt_FromLong(info.m_qIndex));
      PyTuple_SetItem(pyListJointInfo, 4, 
        PyInt_FromLong(info.m_uIndex));
      PyTuple_SetItem(pyListJointInfo, 5, 
        PyInt_FromLong(info.m_flags));   
      PyTuple_SetItem(pyListJointInfo, 6, 
        PyFloat_FromDouble(info.m_jointDamping));
      PyTuple_SetItem(pyListJointInfo, 7, 
        PyFloat_FromDouble(info.m_jointFriction));    
      return pyListJointInfo;
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

static PyObject*
pybullet_getJointState(PyObject* self, PyObject* args)
{
	  PyObject *pyListJointForceTorque; 
  PyObject *pyListJointState; 
  PyObject *item;   
  
  struct b3JointInfo info;
  struct b3JointSensorState sensorState;
  
  int bodyIndex = -1;
  int jointIndex = -1;
  int sensorStateSize = 4; // size of struct b3JointSensorState
  int forceTorqueSize = 6; // size of force torque list from b3JointSensorState
  int i, j;
    
    
  int size= PySequence_Size(args);

	if (0==sm)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

  if (size==2) // get body index and joint index
  {
   if (PyArg_ParseTuple(args, "ii", &bodyIndex, &jointIndex))
  	{

  b3SharedMemoryCommandHandle cmd_handle =
          b3RequestActualStateCommandInit(sm, bodyIndex);
      b3SharedMemoryStatusHandle status_handle =
              b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);
                
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
    
    PyTuple_SetItem(pyListJointState, 2, 
      pyListJointForceTorque);
    
    PyTuple_SetItem(pyListJointState, 3, 
      PyFloat_FromDouble(sensorState.m_jointMotorTorque));
    
    return pyListJointState;
    }
  } else
  {
      PyErr_SetString(SpamError, "getJointState expects 2 arguments (objectUniqueId and joint index).");
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
static int pybullet_internalSetMatrix(PyObject* objMat, float matrix[16])
{
    int i, len;
    PyObject* seq;

    seq = PySequence_Fast(objMat, "expected a sequence");
    len = PySequence_Size(objMat);
    if (len==16)
    {
        for (i = 0; i < len; i++)
        {
            matrix[i] = pybullet_internalGetFloatFromSequence(seq,i);
        }
        Py_DECREF(seq);
        return 1;
    }
    Py_DECREF(seq);
    return 0;
}

// Render an image from the current timestep of the simulation
//
// Examples:
//  renderImage() - default image resolution and camera position
//  renderImage(w, h) - image resolution of (w,h), default camera
//  renderImage(w, h, view[16], projection[16]) - set both resolution
//    and initialize camera to the view and projection values
//
// Note if the (w,h) is too small, the objects may not appear based on
// where the camera has been set
//
// TODO(hellojas): fix image is cut off at head
// TODO(hellojas): should we add check to give minimum image resolution
//  to see object based on camera position?
static PyObject* pybullet_renderImage(PyObject* self, PyObject* args)
{
  ///request an image from a simulated camera, using a software renderer.
  struct b3CameraImageData imageData;
  PyObject* objViewMat,* objProjMat;
  int width,  height;
  int size= PySequence_Size(args);
  float viewMatrix[16];
  float projectionMatrix[16];

  // inialize cmd
  b3SharedMemoryCommandHandle command;
  

  if (0==sm)
  {
  	PyErr_SetString(SpamError, "Not connected to physics server.");
  	return NULL;
  }

  command = b3InitRequestCameraImage(sm);

  if (size==2) // only set camera resolution
  {
   if (PyArg_ParseTuple(args, "ii", &width, &height))
  	{
     b3RequestCameraImageSetPixelResolution(command,width,height);
   }
  }

  if (size==4) // set camera resoluation and view and projection matrix
  {
    if (PyArg_ParseTuple(args, "iiOO", &width, &height, &objViewMat, &objProjMat))
    {
      b3RequestCameraImageSetPixelResolution(command,width,height);

      // set camera matrices only if set matrix function succeeds
     if (pybullet_internalSetMatrix(objViewMat, viewMatrix) &&
         (pybullet_internalSetMatrix(objProjMat, projectionMatrix))) 
         {
             b3RequestCameraImageSetCameraMatrices(command, viewMatrix, projectionMatrix);
         }
       }
   }

	if (b3CanSubmitCommand(sm))
	{
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		statusType = b3GetStatusType(statusHandle);
		if (statusType==CMD_CAMERA_IMAGE_COMPLETED)
		{
			PyObject *item2;
			PyObject* pyResultList;//store 4 elements in this result: width, height, rgbData, depth
			PyObject *pylistRGB;
			PyObject* pylistDep;
			int i, j, p;

			b3GetCameraImageData(sm, &imageData);
			//TODO(hellojas): error handling if image size is 0
			pyResultList =  PyTuple_New(4);
			PyTuple_SetItem(pyResultList, 0, PyInt_FromLong(imageData.m_pixelWidth));
			PyTuple_SetItem(pyResultList, 1, PyInt_FromLong(imageData.m_pixelHeight));


			{

				PyObject *item;
				int bytesPerPixel = 4;//Red, Green, Blue, and Alpha each 8 bit values
				int num=bytesPerPixel*imageData.m_pixelWidth*imageData.m_pixelHeight;
				pylistRGB = PyTuple_New(num);
				pylistDep = PyTuple_New(imageData.m_pixelWidth*imageData.m_pixelHeight);

				for (i=0;i<imageData.m_pixelWidth;i++)
				{
					for (j=0;j<imageData.m_pixelHeight;j++)
					{
            // TODO(hellojas): validate depth values make sense
						int depIndex = i+j*imageData.m_pixelWidth;
						item = PyFloat_FromDouble(imageData.m_depthValues[depIndex]);
						PyTuple_SetItem(pylistDep, depIndex, item);
						for (p=0; p<bytesPerPixel; p++)
						{
							int pixelIndex = bytesPerPixel*(i+j*imageData.m_pixelWidth)+p;
							item = PyInt_FromLong(imageData.m_rgbColorData[pixelIndex]);
									PyTuple_SetItem(pylistRGB, pixelIndex, item);
						}
					}
				}
			}

			PyTuple_SetItem(pyResultList, 2,pylistRGB);
			PyTuple_SetItem(pyResultList, 3,pylistDep);
			return pyResultList;
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_applyExternalForce(PyObject* self, PyObject* args)
{
    
   if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }
    {
        int objectUniqueId, linkIndex, flags;
        double force[3];
        double position[3]={0,0,0};
        PyObject* forceObj, *posObj;
        
        b3SharedMemoryCommandHandle command;
        b3SharedMemoryStatusHandle statusHandle;
        int size= PySequence_Size(args);
  
        if (size==5)
        {
            if(!PyArg_ParseTuple(args, "iiOOi", &objectUniqueId, &linkIndex, &forceObj, &posObj, &flags))
            {
                PyErr_SetString(SpamError, "applyBaseForce couldn't parse arguments");
                return NULL;
            }
        } else
        {
            PyErr_SetString(SpamError, "applyBaseForce needs 5 arguments: objectUniqueId, linkIndex (-1 for base/root link), force [x,y,z], position [x,y,z], flags");
            
            return NULL;
        }
        
        {
            PyObject* seq;
            int len,i;
            seq = PySequence_Fast(forceObj, "expected a sequence");
            len = PySequence_Size(forceObj);
            if (len==3)
            {
                for (i = 0; i < 3; i++)
                {
                    force[i] = pybullet_internalGetFloatFromSequence(seq,i);
                }
            } else
            {
                PyErr_SetString(SpamError, "force needs a 3 coordinates [x,y,z].");
                Py_DECREF(seq);
                return NULL;
            }
             Py_DECREF(seq);
        }
        {
            PyObject* seq;
            int len,i;
            seq = PySequence_Fast(posObj, "expected a sequence");
            len = PySequence_Size(posObj);
            if (len==3)
            {
                for (i = 0; i < 3; i++)
                {
                    position[i] = pybullet_internalGetFloatFromSequence(seq,i);
                }
            } else
            {
                PyErr_SetString(SpamError, "position needs a 3 coordinates [x,y,z].");
                Py_DECREF(seq);
                return NULL;
            }
             Py_DECREF(seq);

        }
        if ((flags !=EF_WORLD_FRAME) && (flags != EF_LINK_FRAME))
        {
            PyErr_SetString(SpamError, "flag has to be either WORLD_FRAME or LINK_FRAME");
            return NULL;
        }
        command = b3ApplyExternalForceCommandInit(sm);
        b3ApplyExternalForce(command,objectUniqueId,-1,force,position, flags);
        statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
    }
    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject* pybullet_applyExternalTorque(PyObject* self, PyObject* args)
{
    
    if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }
    {
        int objectUniqueId, linkIndex, flags;
        double torque[3];
        PyObject* torqueObj;
        
        if (PyArg_ParseTuple(args, "iiOi", &objectUniqueId, &linkIndex, &torqueObj, &flags))
        {
            
            PyObject* seq;
            int len,i;
            seq = PySequence_Fast(torqueObj, "expected a sequence");
            len = PySequence_Size(torqueObj);
            if (len==3)
            {
                for (i = 0; i < 3; i++)
                {
                    torque[i] = pybullet_internalGetFloatFromSequence(seq,i);
                }
            } else
            {
                PyErr_SetString(SpamError, "torque needs a 3 coordinates [x,y,z].");
                Py_DECREF(seq);
                return NULL;
            }
            Py_DECREF(seq);
            
            if (linkIndex <-1)
            {
                PyErr_SetString(SpamError, "Invalid link index, has to be -1 or larger");
                return NULL;
            }
            if ((flags !=EF_WORLD_FRAME) && (flags != EF_LINK_FRAME))
            {
                PyErr_SetString(SpamError, "flag has to be either WORLD_FRAME or LINK_FRAME");
                return NULL;
            }
            b3SharedMemoryStatusHandle statusHandle;
            b3SharedMemoryCommandHandle command = b3ApplyExternalForceCommandInit(sm);
            b3ApplyExternalTorque(command,objectUniqueId,-1,torque, flags);
            statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);

        }
    }
    
    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject* pybullet_getQuaternionFromEuler(PyObject* self, PyObject* args)
{
    double rpy[3];
   
    PyObject* eulerObj;
 
    if (PyArg_ParseTuple(args, "O", &eulerObj))
    {
        PyObject* seq;
        int len,i;
        seq = PySequence_Fast(eulerObj, "expected a sequence");
        len = PySequence_Size(eulerObj);
        if (len==3)
        {
            for (i = 0; i < 3; i++)
            {
                rpy[i] = pybullet_internalGetFloatFromSequence(seq,i);
            }
        } else
        {
            PyErr_SetString(SpamError, "Euler angles need a 3 coordinates [roll, pitch, yaw].");
            Py_DECREF(seq);
            return NULL;
        }
        Py_DECREF(seq);
    } else
    {
        PyErr_SetString(SpamError, "Euler angles need a 3 coordinates [roll, pitch, yaw].");
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

            //normalize the quaternion
            double len = sqrt(quat[0]*quat[0]+quat[1]*quat[1]+quat[2]*quat[2]+quat[3]*quat[3]);
            quat[0] /= len;
            quat[1] /= len;
            quat[2] /= len;
            quat[3] /= len;
            {
                PyObject *pylist;
                int i;
                pylist = PyTuple_New(4);
                for (i=0;i<4;i++)
                    PyTuple_SetItem(pylist,i,PyFloat_FromDouble(quat[i]));
                return pylist;
            }
        }
     }
    Py_INCREF(Py_None);
    return Py_None;
    
}
///quaternion <-> euler yaw/pitch/roll convention from URDF/SDF, see Gazebo
///https://github.com/arpg/Gazebo/blob/master/gazebo/math/Quaternion.cc
static PyObject* pybullet_getEulerFromQuaternion(PyObject* self, PyObject* args)
{
    
    double squ;
    double sqx;
    double sqy;
    double sqz;
    
    double quat[4];

    PyObject* quatObj;
    
    if (PyArg_ParseTuple(args, "O", &quatObj))
    {
        PyObject* seq;
        int len,i;
        seq = PySequence_Fast(quatObj, "expected a sequence");
        len = PySequence_Size(quatObj);
        if (len==4)
        {
            for (i = 0; i < 4; i++)
            {
                quat[i] = pybullet_internalGetFloatFromSequence(seq,i);
            }
        } else
        {
            PyErr_SetString(SpamError, "Quaternion need a 4 components [x,y,z,w].");
            Py_DECREF(seq);
            return NULL;
        }
        Py_DECREF(seq);
    } else
    {
         PyErr_SetString(SpamError, "Quaternion need a 4 components [x,y,z,w].");
        return NULL;
    }

    {
        double rpy[3];
        sqx = quat[0] * quat[0];
        sqy = quat[1] * quat[1];
        sqz = quat[2] * quat[2];
        squ = quat[3] * quat[3];
        rpy[0] = atan2(2 * (quat[1]*quat[2] + quat[3]*quat[0]), squ - sqx - sqy + sqz);
        double sarg = -2 * (quat[0]*quat[2] - quat[3] * quat[1]);
        rpy[1] = sarg <= -1.0 ? -0.5*3.141592538 : (sarg >= 1.0 ? 0.5*3.141592538 : asin(sarg));
        rpy[2] = atan2(2 * (quat[0]*quat[1] + quat[3]*quat[2]), squ + sqx - sqy - sqz);
        {
            PyObject *pylist;
            int i;
            pylist = PyTuple_New(3);
            for (i=0;i<3;i++)
                PyTuple_SetItem(pylist,i,PyFloat_FromDouble(rpy[i]));
            return pylist;
        }
    }
    Py_INCREF(Py_None);
    return Py_None;
}


static PyMethodDef SpamMethods[] = {
  
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

    {"setTimeStep",  pybullet_setTimeStep, METH_VARARGS,
        "Set the amount of time to proceed at each call to stepSimulation. (unit is seconds, typically range is 0.01 or 0.001)"},
    
    {"loadURDF",  pybullet_loadURDF, METH_VARARGS,
      "Create a multibody by loading a URDF file."},
      
	{"loadSDF", pybullet_loadSDF, METH_VARARGS,
		"Load multibodies from an SDF file."},
    
	{"getBasePositionAndOrientation",  pybullet_getBasePositionAndOrientation, METH_VARARGS,
        "Get the world position and orientation of the base of the object. (x,y,z) position vector and (x,y,z,w) quaternion orientation."},

    {"resetBasePositionAndOrientation",  pybullet_resetBasePositionAndOrientation, METH_VARARGS,
        "Reset the world position and orientation of the base of the object instantaneously, not through physics simulation. (x,y,z) position vector and (x,y,z,w) quaternion orientation."},

	{"getNumJoints", pybullet_getNumJoints, METH_VARARGS,
        "Get the number of joints for an object."},

    {"getJointInfo", pybullet_getJointInfo, METH_VARARGS,
    "Get the name and type info for a joint on a body."},

    {"getJointState", pybullet_getJointState, METH_VARARGS,
    "Get the state (position, velocity etc) for a joint on a body."},
	  
	{"resetJointState", pybullet_resetJointState, METH_VARARGS,
    "Reset the state (position, velocity etc) for a joint on a body instantaneously, not through physics simulation."},
		
    {"setJointMotorControl", pybullet_setJointMotorControl, METH_VARARGS,
        "Set a single joint motor control mode and desired target value. There is no immediate state change, stepSimulation will process the motors."},
    
    {"applyExternalForce", pybullet_applyExternalForce, METH_VARARGS,
        "for objectUniqueId, linkIndex (-1 for base/root link), apply a force [x,y,z] at the a position [x,y,z], flag to select FORCE_IN_LINK_FRAME or FORCE_IN_WORLD_FRAME coordinates"},
   
    {"applyExternalTorque", pybullet_applyExternalTorque, METH_VARARGS,
        "for objectUniqueId, linkIndex (-1 for base/root link) apply a torque [x,y,z] in Cartesian coordinates, flag to select TORQUE_IN_LINK_FRAME or TORQUE_IN_WORLD_FRAME coordinates"},
    
	{"renderImage", pybullet_renderImage, METH_VARARGS,
	"Render an image (given the pixel resolution width & height and camera view & projection matrices), and return the 8-8-8bit RGB pixel data and floating point depth values"},	

    {"getQuaternionFromEuler", pybullet_getQuaternionFromEuler, METH_VARARGS,
        "Convert Euler [roll, pitch, yaw] as in URDF/SDF convention, to quaternion [x,y,z,w]"},
    
    {"getEulerFromQuaternion", pybullet_getEulerFromQuaternion, METH_VARARGS,
        "Convert quaternion [x,y,z,w] to Euler [roll, pitch, yaw] as in URDF/SDF convention"},
    
    //todo(erwincoumans)
    //saveSnapshot
    //loadSnapshot
    
    ////todo(erwincoumans)
    //collision info
    //raycast info

    {NULL, NULL, 0, NULL}        /* Sentinel */
};

#if PY_MAJOR_VERSION >= 3
    static struct PyModuleDef moduledef = {
        PyModuleDef_HEAD_INIT,
        "pybullet",     /* m_name */
        "Python bindings for Bullet Physics Robotics API (also known as Shared Memory API)",  /* m_doc */
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
    
    PyModule_AddIntConstant (m, "TORQUE_CONTROL", CONTROL_MODE_TORQUE);
    PyModule_AddIntConstant (m, "VELOCITY_CONTROL", CONTROL_MODE_VELOCITY); // user read
    PyModule_AddIntConstant (m, "POSITION_CONTROL", CONTROL_MODE_POSITION_VELOCITY_PD); // user read
    
    PyModule_AddIntConstant (m, "LINK_FRAME", EF_LINK_FRAME);
    PyModule_AddIntConstant (m, "WORLD_FRAME", EF_WORLD_FRAME);
    
    SpamError = PyErr_NewException("pybullet.error", NULL, NULL);
    Py_INCREF(SpamError);
    PyModule_AddObject(m, "error", SpamError);
#if PY_MAJOR_VERSION >= 3
	return m;
#endif
}

