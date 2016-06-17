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
  float startPosZ = 1;
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
	b3SharedMemoryCommandHandle command;

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

	command = b3LoadSdfCommandInit(sm, sdfFileName);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
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

static int pybullet_setJointControl(PyObject* self, PyObject* args)
{
    //todo(erwincoumans): set max forces, kp, kd

    int size;
    
    if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }
    
    size= PySequence_Size(args);
    
    if (size==3)
    {
        int bodyIndex, controlMode;
        PyObject* targetValues;
        if (PyArg_ParseTuple(args, "iiO", &bodyIndex, &controlMode, &targetValues))
        {
            PyObject* seq;
            int numJoints, len;
            seq = PySequence_Fast(targetValues, "expected a sequence");
            len = PySequence_Size(targetValues);
            numJoints = b3GetNumJoints(sm,bodyIndex);
            b3SharedMemoryCommandHandle commandHandle;
            
            if (len!=numJoints)
            {
                PyErr_SetString(SpamError, "Number of control target values doesn't match the number of joints.");
                Py_DECREF(seq);
                return NULL;
            }
            
            if ((controlMode != CONTROL_MODE_VELOCITY) &&
                (controlMode != CONTROL_MODE_TORQUE) &&
                (controlMode != CONTROL_MODE_POSITION_VELOCITY_PD))
            {
                PyErr_SetString(SpamError, "Illegral control mode.");
                Py_DECREF(seq);
                return NULL;
            }
            
            commandHandle = b3JointControlCommandInit(sm, bodyIndex,controlMode);
            
            for (int qIndex=0;qIndex<numJoints;qIndex++)
            {
                float value = pybullet_internalGetFloatFromSequence(seq,qIndex);
            
                switch (controlMode)
                {
                    case CONTROL_MODE_VELOCITY:
                    {
                        b3JointControlSetDesiredVelocity(commandHandle, qIndex, value);
                        break;
                    }

                    case CONTROL_MODE_TORQUE:
                    {
                        b3JointControlSetDesiredForceTorque(commandHandle, qIndex, value);
                        break;
                    }
                        
                    case CONTROL_MODE_POSITION_VELOCITY_PD:
                    {
                        b3JointControlSetDesiredPosition( commandHandle, qIndex, value);
                        break;
                    }
                    default:
                    {
                    
                    }
                };
            }
            Py_DECREF(seq);
            Py_INCREF(Py_None);
            return Py_None;
        }
        
    }
    PyErr_SetString(SpamError, "error in setJointControl.");
    return NULL;
}

                /*
                 ///Set joint control variables such as desired position/angle, desired velocity,
                 ///applied joint forces, dependent on the control mode (CONTROL_MODE_VELOCITY or CONTROL_MODE_TORQUE)
                 b3SharedMemoryCommandHandle  b3JointControlCommandInit(b3PhysicsClientHandle physClient, int controlMode);
                 ///Only use when controlMode is CONTROL_MODE_POSITION_VELOCITY_PD
                 int b3JointControlSetDesiredPosition(b3SharedMemoryCommandHandle commandHandle, int qIndex, double value);
                 int b3JointControlSetKp(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
                 int b3JointControlSetKd(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value);
                 //Only use when controlMode is CONTROL_MODE_VELOCITY
                 int b3JointControlSetDesiredVelocity(b3SharedMemoryCommandHandle commandHandle, int dofIndex, double value); 
                 // find a better name for dof/q/u indices, point to b3JointInfo
                int b3JointControlSetMaximumForce(b3SharedMemoryCommandHandle commandHandle, int dofIndex,  double value);
                ///Only use if when controlMode is CONTROL_MODE_TORQUE,
                int b3JointControlSetDesiredForceTorque(b3SharedMemoryCommandHandle commandHandle, int  dofIndex, double value);
                
                 */
         


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
pybullet_initializeJointPositions(PyObject* self, PyObject* args)
{
	if (0==sm)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
  // TODO(hellojas): initialize all joint positions given a pylist of values

    //
    //
    /*
     ///b3CreatePoseCommandInit will initialize (teleport) the pose of a body/robot. You can individually set the base position,
     ///base orientation and joint angles. This will set all velocities of base and joints to zero.
     ///This is not a robot control command using actuators/joint motors, but manual repositioning the robot.
     b3SharedMemoryCommandHandle b3CreatePoseCommandInit(b3PhysicsClientHandle physClient, int bodyIndex);
     int     b3CreatePoseCommandSetBasePosition(b3SharedMemoryCommandHandle commandHandle, double startPosX,double startPosY,double startPosZ);
     int     b3CreatePoseCommandSetBaseOrientation(b3SharedMemoryCommandHandle commandHandle, double startOrnX,double startOrnY,double startOrnZ, double startOrnW);
     int     b3CreatePoseCommandSetJointPositions(b3SharedMemoryCommandHandle commandHandle, int numJointPositions, const double* jointPositions);
     int     b3CreatePoseCommandSetJointPosition(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle,  int jointIndex, double jointPosition);
     

     */
    //
    //
    
	Py_INCREF(Py_None);
        return Py_None;
}


// CURRENTLY NOT SUPPORTED 
// Initalize a single joint position for a specific body index
// 
// This method skips any physics simulation and 
// teleports all joints to the new positions.

// TODO(hellojas): initializing one joint currently not supported
// static PyObject*
// pybullet_initializeJointPosition(PyObject* self, PyObject* args)
// {
// 	if (0==sm)
// 	{
// 		PyErr_SetString(SpamError, "Not connected to physics server.");
// 		return NULL;
// 	}
//   
//   int i;
//   int bodyIndex = -1;
//   int jointIndex = -1;
//   double jointPos = 0.0;
//   
//   int size= PySequence_Size(args);
// 
//   if (size==3) // get body index, joint index, and joint position value
//   {
//    if (PyArg_ParseTuple(args, "iid", &bodyIndex, &jointIndex, &jointPos))
//     {
//       b3SharedMemoryCommandHandle cmd_handle =  b3CreatePoseCommandInit(sm, bodyIndex);
// 
//       // printf("initializing joint %d at %f\n", jointIndex, jointPos);
//       b3CreatePoseCommandSetJointPosition(sm, cmd_handle, jointIndex, jointPos);
// 
//       b3SharedMemoryStatusHandle status_handle =
//               b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);
// 
//     const int status_type = b3GetStatusType(status_handle);
// 
//     }
//   }
//   
// 	Py_INCREF(Py_None);
//         return Py_None;
// }

static void pybullet_internalGetJointPositions(int bodyIndex, int numJoints, double jointPositions[]) {
  int i, j;
  int numDegreeQ;
  int numDegreeU;
  int arrSizeOfPosAndOrn = 7;
  
  for (i =0;i <numJoints; i++){
    jointPositions[i] = .5;
  }
  
  {
  b3SharedMemoryCommandHandle cmd_handle =
          b3RequestActualStateCommandInit(sm, bodyIndex);
      b3SharedMemoryStatusHandle status_handle =
              b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);
              
      const int status_type = b3GetStatusType(status_handle);            
      const double* actualStateQ;

      b3GetStatusActualState(status_handle, 0/* body_unique_id */,
                         &numDegreeQ ,
                         &numDegreeU/* num_degree_of_freedom_u */, 0 /*root_local_inertial_frame*/,
                         &actualStateQ , 0 /* actual_state_q_dot */,
                         0 /* joint_reaction_forces */);

      
      // printf("actual state Q, size = %lu\n", sizeof(actualStateQ.));
      // printf("numjoints = %d\n", numJoints);
      // printf("numDegreeQ = %d\n", numDegreeQ);
      // printf("numDegreeU = %d\n", numDegreeU);
      // printf("actualStateQ[0] = %f\n",actualStateQ[0]);
      for (j = arrSizeOfPosAndOrn; j < numJoints + arrSizeOfPosAndOrn; j++) {
        jointPositions[j - arrSizeOfPosAndOrn] = actualStateQ[j];
        // printf("%d=%f\n", j, jointPositions[j - arrSizeOfPosAndOrn]);

      }     
    }
}

// Get a list of all joint positions for a given body index
//
// Args:
//  bodyIndex - integer indicating body in simulation
// Returns:
//  pyListJointPos - list of positions for each joint index
//
static PyObject *
pybullet_getJointPositions(PyObject* self, PyObject* args)
{
    if (0==sm)
    {
        PyErr_SetString(SpamError, "Not connected to physics server.");
        return NULL;
    }
    
    int bodyIndex = -1;
    
    if (!PyArg_ParseTuple(args, "i", &bodyIndex ))
    {
        PyErr_SetString(SpamError, "Expected a body index (integer).");
        return NULL;
    }
    
    {
      PyObject *item;   
      PyObject *pyListJointPos; 
      
      int i;
      int numJoints = b3GetNumJoints(sm,bodyIndex);
      double jointPositions[numJoints];
      pyListJointPos = PyTuple_New(numJoints);


      // printf("joint positions size = %lu\n", sizeof(jointPositions)/sizeof(double));
      pybullet_internalGetJointPositions(bodyIndex, numJoints,jointPositions);
      
      for (i =0;i <numJoints; i++){
        item = PyFloat_FromDouble(jointPositions[i]);
        PyTuple_SetItem(pyListJointPos, i, item);
      }
      
      return pyListJointPos;
    }
    
    Py_INCREF(Py_None);
    return Py_None;
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
	if (0==sm)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

  PyObject *pyListJointInfo; 
  
  struct b3JointInfo info;
  
  int bodyIndex = -1;
  int jointIndex = -1;
  int jointInfoSize = 8; //size of struct b3JointInfo
    
  int size= PySequence_Size(args);

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
	if (0==sm)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
  
    
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

    
    // double m_jointPosition;
    // double m_jointVelocity;
    // double m_jointForceTorque[6];  /* note to roboticists: this is NOT the motor torque/force, but the spatial reaction force vector at joint */
    // double m_jointMotorTorque;
    b3GetJointState(sm, status_handle, jointIndex, &sensorState);
    // printf("Joint%d: position=%f velocity=%f motortorque=%f\n", 
    // jointIndex, 
    // sensorState.m_jointPosition,
    // sensorState.m_jointVelocity,
    // sensorState.m_jointMotorTorque);
    
    PyTuple_SetItem(pyListJointState, 0, 
      PyFloat_FromDouble(sensorState.m_jointPosition));
    PyTuple_SetItem(pyListJointState, 1, 
      PyFloat_FromDouble(sensorState.m_jointVelocity));
      
    // joint force torque is list of 6
    /* note to roboticists: this is NOT the motor torque/force, but the spatial reaction force vector at joint */
    // printf(" jointForceTorque = ");
    for (j = 0; j < forceTorqueSize; j++) {
      // printf("%f ", sensorState.m_jointForceTorque[j]);
      PyTuple_SetItem(pyListJointForceTorque, j, 
        PyFloat_FromDouble(sensorState.m_jointForceTorque[j]));
    }
    
    PyTuple_SetItem(pyListJointState, 2, 
      pyListJointForceTorque);
    
    PyTuple_SetItem(pyListJointState, 3, 
      PyFloat_FromDouble(sensorState.m_jointMotorTorque));
    
    return pyListJointState;
    }
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


static PyMethodDef SpamMethods[] = {
  {"loadURDF",  pybullet_loadURDF, METH_VARARGS,
      "Create a multibody by loading a URDF file."},
      
	{"loadSDF", pybullet_loadSDF, METH_VARARGS,
		"Load multibodies from an SDF file."},
  
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

	{"initializeJointPositions", pybullet_initializeJointPositions, METH_VARARGS,
	"Initialize the joint positions for all joints. This method skips any physics simulation and teleports all joints to the new positions."},
	
  // CURRENTLY NOT SUPPORTED
  // {"initializeJointPosition", pybullet_initializeJointPosition, METH_VARARGS,
  // "Initialize the joint position for one joint. This method skips any physics simulation and teleports the joint to the new position."},
  
  {"getJointInfo", pybullet_getJointInfo, METH_VARARGS,
  "Get the joint metadata info for a joint on a body. This includes joint index, name, type, q-index and u-index."},
  
  {"getJointState", pybullet_getJointState, METH_VARARGS,
  "Get the joint metadata info for a joint on a body."},

    {"setJointControl", pybullet_setJointControl, METH_VARARGS,
        "Set the joint control mode and desired target values."},

    
	{"renderImage", pybullet_renderImage, METH_VARARGS,
	"Render an image (given the pixel resolution width & height and camera view & projection matrices), and return the 8-8-8bit RGB pixel data and floating point depth values"},	
	
	{"getBasePositionAndOrientation",  pybullet_getBasePositionAndOrientation, METH_VARARGS,
        "Get the world position and orientation of the base of the object. (x,y,z) position vector and (x,y,z,w) quaternion orientation."},

  {"getJointPositions", pybullet_getJointPositions, METH_VARARGS,
  "Get the all the joint positions for a given body index."},
        
	{"getNumJoints", pybullet_getNumJoints, METH_VARARGS,
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
    
    PyModule_AddIntConstant (m, "TORQUE_CONTROL", CONTROL_MODE_TORQUE);
    PyModule_AddIntConstant (m, "VELOCITY_CONTROL", CONTROL_MODE_VELOCITY); // user read
    PyModule_AddIntConstant (m, "POSITION_CONTROL", CONTROL_MODE_POSITION_VELOCITY_PD); // user read
    
    SpamError = PyErr_NewException("pybullet.error", NULL, NULL);
    Py_INCREF(SpamError);
    PyModule_AddObject(m, "error", SpamError);
#if PY_MAJOR_VERSION >= 3
	return m;
#endif
}

