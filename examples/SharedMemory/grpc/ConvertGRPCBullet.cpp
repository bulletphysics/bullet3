#include "ConvertGRPCBullet.h"
#include "SharedMemory/PhysicsClientC_API.h"
#include "SharedMemory/SharedMemoryCommands.h"
#include <memory>
#include <iostream>
#include <string>
#include <thread>
#include <grpc++/grpc++.h>
#include <grpc/support/log.h>
#include "SharedMemory/grpc/proto/pybullet.grpc.pb.h"
#include "LinearMath/btMinMax.h"

#define ALLOW_GRPC_COMMAND_CONVERSION
#define ALLOW_GRPC_STATUS_CONVERSION

using grpc::Server;
using grpc::ServerAsyncResponseWriter;
using grpc::ServerBuilder;
using grpc::ServerCompletionQueue;
using grpc::ServerContext;
using grpc::Status;
using pybullet_grpc::PyBulletCommand;
using pybullet_grpc::PyBulletStatus;

pybullet_grpc::PyBulletCommand* convertBulletToGRPCCommand(const struct SharedMemoryCommand& clientCmd, pybullet_grpc::PyBulletCommand& grpcCommand)
{
	pybullet_grpc::PyBulletCommand* grpcCmdPtr = 0;

	grpcCommand.set_commandtype(clientCmd.m_type);

	switch (clientCmd.m_type)
	{
		case CMD_RESET_SIMULATION:
		{
			grpcCmdPtr = &grpcCommand;
			break;
		}
		case CMD_REQUEST_KEYBOARD_EVENTS_DATA:
		{
			grpcCmdPtr = &grpcCommand;
			break;
		}

		case CMD_USER_CONSTRAINT:
		{
			grpcCmdPtr = &grpcCommand;
			::pybullet_grpc::UserConstraintCommand* con = grpcCommand.mutable_userconstraintcommand();

			con->set_updateflags(clientCmd.m_updateFlags);
			con->mutable_childframe()->mutable_origin()->set_x(clientCmd.m_userConstraintArguments.m_childFrame[0]);
			con->mutable_childframe()->mutable_origin()->set_y(clientCmd.m_userConstraintArguments.m_childFrame[1]);
			con->mutable_childframe()->mutable_origin()->set_z(clientCmd.m_userConstraintArguments.m_childFrame[2]);
			con->mutable_childframe()->mutable_orientation()->set_x(clientCmd.m_userConstraintArguments.m_childFrame[3]);
			con->mutable_childframe()->mutable_orientation()->set_y(clientCmd.m_userConstraintArguments.m_childFrame[4]);
			con->mutable_childframe()->mutable_orientation()->set_z(clientCmd.m_userConstraintArguments.m_childFrame[5]);
			con->mutable_childframe()->mutable_orientation()->set_w(clientCmd.m_userConstraintArguments.m_childFrame[6]);

			con->mutable_parentframe()->mutable_origin()->set_x(clientCmd.m_userConstraintArguments.m_parentFrame[0]);
			con->mutable_parentframe()->mutable_origin()->set_y(clientCmd.m_userConstraintArguments.m_parentFrame[1]);
			con->mutable_parentframe()->mutable_origin()->set_z(clientCmd.m_userConstraintArguments.m_parentFrame[2]);
			con->mutable_parentframe()->mutable_orientation()->set_x(clientCmd.m_userConstraintArguments.m_parentFrame[3]);
			con->mutable_parentframe()->mutable_orientation()->set_y(clientCmd.m_userConstraintArguments.m_parentFrame[4]);
			con->mutable_parentframe()->mutable_orientation()->set_z(clientCmd.m_userConstraintArguments.m_parentFrame[5]);
			con->mutable_parentframe()->mutable_orientation()->set_w(clientCmd.m_userConstraintArguments.m_parentFrame[6]);

			con->mutable_jointaxis()->set_x(clientCmd.m_userConstraintArguments.m_jointAxis[0]);
			con->mutable_jointaxis()->set_y(clientCmd.m_userConstraintArguments.m_jointAxis[1]);
			con->mutable_jointaxis()->set_z(clientCmd.m_userConstraintArguments.m_jointAxis[2]);

			con->set_childbodyindex(clientCmd.m_userConstraintArguments.m_childBodyIndex);
			con->set_childjointindex(clientCmd.m_userConstraintArguments.m_childJointIndex);
			con->set_parentbodyindex(clientCmd.m_userConstraintArguments.m_parentBodyIndex);
			con->set_parentjointindex(clientCmd.m_userConstraintArguments.m_parentJointIndex);
			con->set_jointtype(clientCmd.m_userConstraintArguments.m_jointType);

			if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_MAX_FORCE)
			{
				con->set_maxappliedforce(clientCmd.m_userConstraintArguments.m_maxAppliedForce);
			}
			if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_GEAR_RATIO)
			{
				con->set_gearratio(clientCmd.m_userConstraintArguments.m_gearRatio);
			}

			if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_GEAR_AUX_LINK)
			{
				con->set_gearauxlink(clientCmd.m_userConstraintArguments.m_gearAuxLink);
			}
			if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_RELATIVE_POSITION_TARGET)
			{
				con->set_relativepositiontarget(clientCmd.m_userConstraintArguments.m_relativePositionTarget);
			}
			if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_ERP)
			{
				con->set_erp(clientCmd.m_userConstraintArguments.m_erp);
			}
			con->set_userconstraintuniqueid(-1);

			break;
		}
		case CMD_STEP_FORWARD_SIMULATION:
		{
			grpcCmdPtr = &grpcCommand;
			break;
		}

		case CMD_LOAD_URDF:
		{
			::pybullet_grpc::LoadUrdfCommand* urdfCmd = grpcCommand.mutable_loadurdfcommand();
			grpcCmdPtr = &grpcCommand;
			urdfCmd->set_filename(clientCmd.m_urdfArguments.m_urdfFileName);
			if (clientCmd.m_updateFlags & URDF_ARGS_INITIAL_POSITION)
			{
				urdfCmd->mutable_initialposition()->set_x(clientCmd.m_urdfArguments.m_initialPosition[0]);
				urdfCmd->mutable_initialposition()->set_y(clientCmd.m_urdfArguments.m_initialPosition[1]);
				urdfCmd->mutable_initialposition()->set_z(clientCmd.m_urdfArguments.m_initialPosition[2]);
			}

			if (clientCmd.m_updateFlags & URDF_ARGS_INITIAL_ORIENTATION)
			{
				urdfCmd->mutable_initialorientation()->set_x(clientCmd.m_urdfArguments.m_initialOrientation[0]);
				urdfCmd->mutable_initialorientation()->set_y(clientCmd.m_urdfArguments.m_initialOrientation[1]);
				urdfCmd->mutable_initialorientation()->set_z(clientCmd.m_urdfArguments.m_initialOrientation[2]);
				urdfCmd->mutable_initialorientation()->set_w(clientCmd.m_urdfArguments.m_initialOrientation[3]);
			}

			if (clientCmd.m_updateFlags & URDF_ARGS_USE_MULTIBODY)
			{
				urdfCmd->set_usemultibody(clientCmd.m_urdfArguments.m_useMultiBody);
			}
			if (clientCmd.m_updateFlags & URDF_ARGS_USE_FIXED_BASE)
			{
				urdfCmd->set_usefixedbase(clientCmd.m_urdfArguments.m_useFixedBase);
			}
			if (clientCmd.m_updateFlags & URDF_ARGS_USE_GLOBAL_SCALING)
			{
				urdfCmd->set_globalscaling(clientCmd.m_urdfArguments.m_globalScaling);
			}
			if (clientCmd.m_updateFlags & URDF_ARGS_HAS_CUSTOM_URDF_FLAGS)
			{
				urdfCmd->set_flags(clientCmd.m_urdfArguments.m_urdfFlags);
			}
			break;
		}

		case CMD_INIT_POSE:
		{
			grpcCmdPtr = &grpcCommand;
			::pybullet_grpc::InitPoseCommand* pose = grpcCommand.mutable_initposecommand();
			{
				pose->set_bodyuniqueid(clientCmd.m_initPoseArgs.m_bodyUniqueId);
				pose->set_updateflags(clientCmd.m_updateFlags);
				int maxQ = 0;
				for (int q = 0; q < MAX_DEGREE_OF_FREEDOM; q++)
				{
					if (clientCmd.m_initPoseArgs.m_hasInitialStateQ[q])
					{
						maxQ = q + 1;
					}
					if (clientCmd.m_initPoseArgs.m_hasInitialStateQdot[q])
					{
						maxQ = q + 1;
					}
				}
				for (int q = 0; q < maxQ; q++)
				{
					pose->add_hasinitialstateq(clientCmd.m_initPoseArgs.m_hasInitialStateQ[q]);
					pose->add_hasinitialstateqdot(clientCmd.m_initPoseArgs.m_hasInitialStateQdot[q]);
					pose->add_initialstateq(clientCmd.m_initPoseArgs.m_initialStateQ[q]);

					pose->add_initialstateqdot(clientCmd.m_initPoseArgs.m_initialStateQdot[q]);
				}
			}
			break;
		}

		case CMD_REQUEST_ACTUAL_STATE:
		{
			grpcCmdPtr = &grpcCommand;

			::pybullet_grpc::RequestActualStateCommand* grpcCmd = grpcCommand.mutable_requestactualstatecommand();
			grpcCmd->set_bodyuniqueid(clientCmd.m_requestActualStateInformationCommandArgument.m_bodyUniqueId);
			if (clientCmd.m_updateFlags & ACTUAL_STATE_COMPUTE_FORWARD_KINEMATICS)
			{
				grpcCmd->set_computeforwardkinematics(true);
			}
			if (clientCmd.m_updateFlags & ACTUAL_STATE_COMPUTE_LINKVELOCITY)
			{
				grpcCmd->set_computelinkvelocities(true);
			}
			break;
		}

		case CMD_SEND_DESIRED_STATE:
		{
			::pybullet_grpc::JointMotorControlCommand* motor = grpcCommand.mutable_jointmotorcontrolcommand();
			motor->set_bodyuniqueid(clientCmd.m_sendDesiredStateCommandArgument.m_bodyUniqueId);
			motor->set_controlmode(clientCmd.m_sendDesiredStateCommandArgument.m_controlMode);
			motor->set_updateflags(clientCmd.m_updateFlags);
			int maxQ = 0;
			for (int q = 0; q < MAX_DEGREE_OF_FREEDOM; q++)
			{
				if (clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[q])
				{
					maxQ = q + 1;
				}
			}

			for (int q = 0; q < maxQ; q++)
			{
				motor->add_desiredstateq(clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQ[q]);
				motor->add_desiredstateqdot(clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateQdot[q]);
				motor->add_desiredstateforcetorque(clientCmd.m_sendDesiredStateCommandArgument.m_desiredStateForceTorque[q]);
				motor->add_kd(clientCmd.m_sendDesiredStateCommandArgument.m_Kp[q]);
				motor->add_kp(clientCmd.m_sendDesiredStateCommandArgument.m_Kp[q]);
				motor->add_maxvelocity(clientCmd.m_sendDesiredStateCommandArgument.m_rhsClamp[q]);
				motor->add_hasdesiredstateflags(clientCmd.m_sendDesiredStateCommandArgument.m_hasDesiredStateFlags[q]);
			}
			grpcCmdPtr = &grpcCommand;
			//b3JointControlCommandInit2Internal
			break;
		}

		case CMD_SEND_PHYSICS_SIMULATION_PARAMETERS:
		{
			grpcCmdPtr = &grpcCommand;
			::pybullet_grpc::PhysicsSimulationParametersCommand* grpcCmd = grpcCommand.mutable_setphysicssimulationparameterscommand();
			grpcCmd->set_updateflags(clientCmd.m_updateFlags);
			::pybullet_grpc::PhysicsSimulationParameters* params = grpcCmd->mutable_params();

			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DELTA_TIME)
			{
				params->set_deltatime(clientCmd.m_physSimParamArgs.m_deltaTime);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_GRAVITY)
			{
				::pybullet_grpc::vec3* grav = params->mutable_gravityacceleration();
				grav->set_x(clientCmd.m_physSimParamArgs.m_gravityAcceleration[0]);
				grav->set_y(clientCmd.m_physSimParamArgs.m_gravityAcceleration[1]);
				grav->set_z(clientCmd.m_physSimParamArgs.m_gravityAcceleration[2]);
			}

			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_NUM_SOLVER_ITERATIONS)
			{
				params->set_numsolveriterations(clientCmd.m_physSimParamArgs.m_numSolverIterations);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_NUM_SIMULATION_SUB_STEPS)
			{
				params->set_numsimulationsubsteps(clientCmd.m_physSimParamArgs.m_numSimulationSubSteps);
			}

			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_REAL_TIME_SIMULATION)
			{
				params->set_userealtimesimulation(clientCmd.m_physSimParamArgs.m_useRealTimeSimulation);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DEFAULT_CONTACT_ERP)
			{
				params->set_defaultcontacterp(clientCmd.m_physSimParamArgs.m_defaultContactERP);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_INTERNAL_SIMULATION_FLAGS)
			{
				params->set_internalsimflags(clientCmd.m_physSimParamArgs.m_internalSimFlags);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_USE_SPLIT_IMPULSE)
			{
				params->set_usesplitimpulse(clientCmd.m_physSimParamArgs.m_useSplitImpulse);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_SPLIT_IMPULSE_PENETRATION_THRESHOLD)
			{
				params->set_splitimpulsepenetrationthreshold(clientCmd.m_physSimParamArgs.m_splitImpulsePenetrationThreshold);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_COLLISION_FILTER_MODE)
			{
				params->set_collisionfiltermode(clientCmd.m_physSimParamArgs.m_collisionFilterMode);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_CONTACT_BREAKING_THRESHOLD)
			{
				params->set_contactbreakingthreshold(clientCmd.m_physSimParamArgs.m_contactBreakingThreshold);
			}

			if (clientCmd.m_updateFlags & SIM_PARAM_ENABLE_CONE_FRICTION)
			{
				params->set_enableconefriction(clientCmd.m_physSimParamArgs.m_enableConeFriction);
			}

			if (clientCmd.m_updateFlags & SIM_PARAM_ENABLE_FILE_CACHING)
			{
				params->set_enablefilecaching(clientCmd.m_physSimParamArgs.m_enableFileCaching);
			}

			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_RESTITUTION_VELOCITY_THRESHOLD)
			{
				params->set_restitutionvelocitythreshold(clientCmd.m_physSimParamArgs.m_restitutionVelocityThreshold);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DEFAULT_NON_CONTACT_ERP)
			{
				params->set_defaultnoncontacterp(clientCmd.m_physSimParamArgs.m_defaultNonContactERP);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DEFAULT_FRICTION_ERP)
			{
				params->set_frictionerp(clientCmd.m_physSimParamArgs.m_frictionERP);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DETERMINISTIC_OVERLAPPING_PAIRS)
			{
				params->set_deterministicoverlappingpairs(clientCmd.m_physSimParamArgs.m_deterministicOverlappingPairs);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_CCD_ALLOWED_PENETRATION)
			{
				params->set_allowedccdpenetration(clientCmd.m_physSimParamArgs.m_allowedCcdPenetration);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_JOINT_FEEDBACK_MODE)
			{
				params->set_jointfeedbackmode(clientCmd.m_physSimParamArgs.m_jointFeedbackMode);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DEFAULT_GLOBAL_CFM)
			{
				params->set_defaultglobalcfm(clientCmd.m_physSimParamArgs.m_defaultGlobalCFM);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_DEFAULT_FRICTION_CFM)
			{
				params->set_frictioncfm(clientCmd.m_physSimParamArgs.m_frictionCFM);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_SOLVER_RESIDULAL_THRESHOLD)
			{
				params->set_solverresidualthreshold(clientCmd.m_physSimParamArgs.m_solverResidualThreshold);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_UPDATE_CONTACT_SLOP)
			{
				params->set_contactslop(clientCmd.m_physSimParamArgs.m_contactSlop);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_ENABLE_SAT)
			{
				params->set_enablesat(clientCmd.m_physSimParamArgs.m_enableSAT);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_CONSTRAINT_SOLVER_TYPE)
			{
				params->set_constraintsolvertype(clientCmd.m_physSimParamArgs.m_constraintSolverType);
			}
			if (clientCmd.m_updateFlags & SIM_PARAM_CONSTRAINT_MIN_SOLVER_ISLAND_SIZE)
			{
				params->set_minimumsolverislandsize(clientCmd.m_physSimParamArgs.m_minimumSolverIslandSize);
			}

			break;
		}

		case CMD_REQUEST_BODY_INFO:
		{
			grpcCmdPtr = &grpcCommand;
			::pybullet_grpc::RequestBodyInfoCommand* grpcCmd = grpcCommand.mutable_requestbodyinfocommand();
			grpcCmd->set_bodyuniqueid(clientCmd.m_sdfRequestInfoArgs.m_bodyUniqueId);
			break;
		}

		case CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS:
		{
			grpcCmdPtr = &grpcCommand;
			break;
		}

		case CMD_SYNC_BODY_INFO:
		{
			grpcCmdPtr = &grpcCommand;
			break;
		}
		case CMD_REQUEST_INTERNAL_DATA:
		{
			grpcCmdPtr = &grpcCommand;
			break;
		}

		case CMD_CONFIGURE_OPENGL_VISUALIZER:
		{
			grpcCmdPtr = &grpcCommand;
			::pybullet_grpc::ConfigureOpenGLVisualizerCommand* vizCmd = grpcCommand.mutable_configureopenglvisualizercommand();

			vizCmd->set_updateflags(clientCmd.m_updateFlags);
			vizCmd->set_cameradistance(clientCmd.m_configureOpenGLVisualizerArguments.m_cameraDistance);
			vizCmd->set_camerapitch(clientCmd.m_configureOpenGLVisualizerArguments.m_cameraPitch);
			vizCmd->set_camerayaw(clientCmd.m_configureOpenGLVisualizerArguments.m_cameraYaw);
			vizCmd->set_setflag(clientCmd.m_configureOpenGLVisualizerArguments.m_setFlag);
			vizCmd->set_setenabled(clientCmd.m_configureOpenGLVisualizerArguments.m_setEnabled);
			::pybullet_grpc::vec3* targetPos = vizCmd->mutable_cameratargetposition();
			targetPos->set_x(clientCmd.m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[0]);
			targetPos->set_y(clientCmd.m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[1]);
			targetPos->set_z(clientCmd.m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[2]);
			break;
		}

		case CMD_REQUEST_CAMERA_IMAGE_DATA:
		{
			grpcCmdPtr = &grpcCommand;
			::pybullet_grpc::RequestCameraImageCommand* cam = grpcCommand.mutable_requestcameraimagecommand();

			cam->set_updateflags(clientCmd.m_updateFlags);
			cam->set_startpixelindex(clientCmd.m_requestPixelDataArguments.m_startPixelIndex);

			if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_PIXEL_WIDTH_HEIGHT)
			{
				cam->set_pixelwidth(clientCmd.m_requestPixelDataArguments.m_pixelWidth);
				cam->set_pixelheight(clientCmd.m_requestPixelDataArguments.m_pixelHeight);
			}
			if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_SPECULAR_COEFF)
			{
				cam->set_lightspecularcoeff(clientCmd.m_requestPixelDataArguments.m_lightSpecularCoeff);
			}

			if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_HAS_FLAGS)
			{
				cam->set_cameraflags(clientCmd.m_requestPixelDataArguments.m_flags);
			}
			if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_SHADOW)
			{
				cam->set_hasshadow(clientCmd.m_requestPixelDataArguments.m_hasShadow);
			}
			if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_AMBIENT_COEFF)
			{
				cam->set_lightambientcoeff(clientCmd.m_requestPixelDataArguments.m_lightAmbientCoeff);
			}
			if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_DIFFUSE_COEFF)
			{
				cam->set_lightdiffusecoeff(clientCmd.m_requestPixelDataArguments.m_lightDiffuseCoeff);
			}
			if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_LIGHT_DISTANCE)
			{
				cam->set_lightdistance(clientCmd.m_requestPixelDataArguments.m_lightDistance);
			}

			if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_LIGHT_COLOR)
			{
				::pybullet_grpc::vec3* lightColor = cam->mutable_lightcolor();
				lightColor->set_x(clientCmd.m_requestPixelDataArguments.m_lightColor[0]);
				lightColor->set_y(clientCmd.m_requestPixelDataArguments.m_lightColor[1]);
				lightColor->set_z(clientCmd.m_requestPixelDataArguments.m_lightColor[2]);
			}
			if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_LIGHT_DIRECTION)
			{
				::pybullet_grpc::vec3* lightDir = cam->mutable_lightdirection();
				lightDir->set_x(clientCmd.m_requestPixelDataArguments.m_lightDirection[0]);
				lightDir->set_y(clientCmd.m_requestPixelDataArguments.m_lightDirection[1]);
				lightDir->set_z(clientCmd.m_requestPixelDataArguments.m_lightDirection[2]);
			}

			if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_HAS_CAMERA_MATRICES)
			{
				::pybullet_grpc::matrix4x4* projMat = cam->mutable_projectionmatrix();
				::pybullet_grpc::matrix4x4* viewMat = cam->mutable_viewmatrix();
				for (int i = 0; i < 16; i++)
				{
					projMat->add_elems(clientCmd.m_requestPixelDataArguments.m_projectionMatrix[i]);
					viewMat->add_elems(clientCmd.m_requestPixelDataArguments.m_viewMatrix[i]);
				}
			}

			if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_HAS_PROJECTIVE_TEXTURE_MATRICES)
			{
				::pybullet_grpc::matrix4x4* projectiveProjMat = cam->mutable_projectivetextureprojectionmatrix();
				::pybullet_grpc::matrix4x4* projectiveViewMat = cam->mutable_projectivetextureviewmatrix();
				for (int i = 0; i < 16; i++)
				{
					projectiveProjMat->add_elems(clientCmd.m_requestPixelDataArguments.m_projectiveTextureProjectionMatrix[i]);
					projectiveViewMat->add_elems(clientCmd.m_requestPixelDataArguments.m_projectiveTextureViewMatrix[i]);
				}
			}

			break;
		}

		default:
		{
			//printf("convertBulletToGRPCCommand: Unknown command\n");
			//assert(0);
		}
	};

	if (0 == grpcCmdPtr)
	{
		grpcCmdPtr = &grpcCommand;
		//printf("Warning: slow fallback of convertBulletToGRPCCommand (%d)", clientCmd.m_type);
		//convert an unknown command as binary blob
		int sz = sizeof(SharedMemoryCommand);
		if (sz > 0)
		{
			grpcCommand.add_unknowncommandbinaryblob((const char*)&clientCmd, sz);
		}
	}
	btAssert(grpcCmdPtr);
	return grpcCmdPtr;
}

SharedMemoryCommand* convertGRPCToBulletCommand(const PyBulletCommand& grpcCommand, SharedMemoryCommand& cmd)
{
	SharedMemoryCommand* cmdPtr = 0;

	cmd.m_type = grpcCommand.commandtype();
	if (cmd.m_type == CMD_INVALID)
	{
		//derive it from the contents instead
		if (grpcCommand.has_changedynamicscommand())
		{
			cmd.m_type = CMD_CHANGE_DYNAMICS_INFO;
		}
		if (grpcCommand.has_resetsimulationcommand())
		{
			cmd.m_type = CMD_RESET_SIMULATION;
		}
		if (grpcCommand.has_loadurdfcommand())
		{
			cmd.m_type = CMD_LOAD_URDF;
		}
		if (grpcCommand.has_loadsdfcommand())
		{
			cmd.m_type = CMD_LOAD_SDF;
		}
		if (grpcCommand.has_loadmjcfcommand())
		{
			cmd.m_type = CMD_LOAD_MJCF;
		}
		if (grpcCommand.has_changedynamicscommand())
		{
			cmd.m_type = CMD_CHANGE_DYNAMICS_INFO;
		}
		if (grpcCommand.has_getdynamicscommand())
		{
			cmd.m_type = CMD_GET_DYNAMICS_INFO;
		}
		if (grpcCommand.has_initposecommand())
		{
			cmd.m_type = CMD_INIT_POSE;
		}
		if (grpcCommand.has_requestactualstatecommand())
		{
			cmd.m_type = CMD_REQUEST_ACTUAL_STATE;
		}
		if (grpcCommand.has_stepsimulationcommand())
		{
			cmd.m_type = CMD_STEP_FORWARD_SIMULATION;
		}
	}

	int sz = grpcCommand.unknowncommandbinaryblob_size();
	if (sz)
	{
		if (sz == 1)
		{
			const char* data = grpcCommand.unknowncommandbinaryblob().Get(0).c_str();
			int numBytes = grpcCommand.unknowncommandbinaryblob().Get(0).size();

			btAssert(sizeof(SharedMemoryCommand) == numBytes);
			if (sizeof(SharedMemoryCommand) == numBytes)
			{
				memcpy(&cmd, data, numBytes);
			}
			//printf("slow fallback on command type %d\n", cmd.m_type);
		}
		else
		{
			printf("Error: Ignore unexpected unknowncommandbinaryblob\n");
		}
		cmdPtr = &cmd;
	}

	if (cmdPtr == 0)
	{
		switch (cmd.m_type)
		{
			case CMD_RESET_SIMULATION:
			{
				cmdPtr = &cmd;
				break;
			}

			case CMD_REQUEST_KEYBOARD_EVENTS_DATA:
			{
				cmdPtr = &cmd;
				b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
				b3RequestKeyboardEventsCommandInit2(commandHandle);
				break;
			}

			case CMD_USER_CONSTRAINT:
			{
				SharedMemoryCommand& clientCmd = cmd;
				cmdPtr = &cmd;
				b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;

				const ::pybullet_grpc::UserConstraintCommand* con = &grpcCommand.userconstraintcommand();

				clientCmd.m_updateFlags = con->updateflags();
				clientCmd.m_userConstraintArguments.m_childFrame[0] = con->childframe().origin().x();
				clientCmd.m_userConstraintArguments.m_childFrame[1] = con->childframe().origin().y();
				clientCmd.m_userConstraintArguments.m_childFrame[2] = con->childframe().origin().z();
				clientCmd.m_userConstraintArguments.m_childFrame[3] = con->childframe().orientation().x();
				clientCmd.m_userConstraintArguments.m_childFrame[4] = con->childframe().orientation().y();
				clientCmd.m_userConstraintArguments.m_childFrame[5] = con->childframe().orientation().z();
				clientCmd.m_userConstraintArguments.m_childFrame[6] = con->childframe().orientation().w();

				clientCmd.m_userConstraintArguments.m_parentFrame[0] = con->parentframe().origin().x();
				clientCmd.m_userConstraintArguments.m_parentFrame[1] = con->parentframe().origin().y();
				clientCmd.m_userConstraintArguments.m_parentFrame[2] = con->parentframe().origin().z();
				clientCmd.m_userConstraintArguments.m_parentFrame[3] = con->parentframe().orientation().x();
				clientCmd.m_userConstraintArguments.m_parentFrame[4] = con->parentframe().orientation().y();
				clientCmd.m_userConstraintArguments.m_parentFrame[5] = con->parentframe().orientation().z();
				clientCmd.m_userConstraintArguments.m_parentFrame[6] = con->parentframe().orientation().w();

				clientCmd.m_userConstraintArguments.m_jointAxis[0] = con->jointaxis().x();
				clientCmd.m_userConstraintArguments.m_jointAxis[1] = con->jointaxis().y();
				clientCmd.m_userConstraintArguments.m_jointAxis[2] = con->jointaxis().z();

				clientCmd.m_userConstraintArguments.m_childBodyIndex = con->childbodyindex();
				clientCmd.m_userConstraintArguments.m_childJointIndex = con->childjointindex();
				clientCmd.m_userConstraintArguments.m_parentBodyIndex = con->parentbodyindex();
				clientCmd.m_userConstraintArguments.m_parentJointIndex = con->parentjointindex();
				clientCmd.m_userConstraintArguments.m_jointType = con->jointtype();

				if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_MAX_FORCE)
				{
					clientCmd.m_userConstraintArguments.m_maxAppliedForce = con->maxappliedforce();
				}
				if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_GEAR_RATIO)
				{
					clientCmd.m_userConstraintArguments.m_gearRatio = con->gearratio();
				}

				if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_GEAR_AUX_LINK)
				{
					clientCmd.m_userConstraintArguments.m_gearAuxLink = con->gearauxlink();
				}
				if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_RELATIVE_POSITION_TARGET)
				{
					clientCmd.m_userConstraintArguments.m_relativePositionTarget = con->relativepositiontarget();
				}
				if (clientCmd.m_updateFlags & USER_CONSTRAINT_CHANGE_ERP)
				{
					clientCmd.m_userConstraintArguments.m_erp = con->erp();
				}

				break;
			}

			case CMD_STEP_FORWARD_SIMULATION:
			{
				cmdPtr = &cmd;
				b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
				b3InitStepSimulationCommand2(commandHandle);

				break;
			}

			case CMD_LOAD_URDF:
			{
				btAssert(grpcCommand.has_loadurdfcommand());
				if (grpcCommand.has_loadurdfcommand())
				{
					const ::pybullet_grpc::LoadUrdfCommand& grpcCmd = grpcCommand.loadurdfcommand();

					std::string fileName = grpcCmd.filename();
					if (fileName.length())
					{
						cmdPtr = &cmd;
						b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
						b3LoadUrdfCommandInit2(commandHandle, fileName.c_str());

						if (grpcCmd.has_initialposition())
						{
							const ::pybullet_grpc::vec3& pos = grpcCmd.initialposition();
							b3LoadUrdfCommandSetStartPosition(commandHandle, pos.x(), pos.y(), pos.z());
						}
						if (grpcCmd.has_initialorientation())
						{
							const ::pybullet_grpc::quat4& orn = grpcCmd.initialorientation();
							b3LoadUrdfCommandSetStartOrientation(commandHandle, orn.x(), orn.y(), orn.z(), orn.w());
						}
						if (grpcCmd.hasUseMultiBody_case() == ::pybullet_grpc::LoadUrdfCommand::HasUseMultiBodyCase::kUseMultiBody)
						{
							b3LoadUrdfCommandSetUseMultiBody(commandHandle, grpcCmd.usemultibody());
						}
						if (grpcCmd.hasGlobalScaling_case() == ::pybullet_grpc::LoadUrdfCommand::HasGlobalScalingCase::kGlobalScaling)
						{
							b3LoadUrdfCommandSetGlobalScaling(commandHandle, grpcCmd.globalscaling());
						}
						if (grpcCmd.hasUseFixedBase_case() == ::pybullet_grpc::LoadUrdfCommand::HasUseFixedBaseCase::kUseFixedBase)
						{
							b3LoadUrdfCommandSetUseFixedBase(commandHandle, grpcCmd.usefixedbase());
						}
						if (grpcCmd.flags())
						{
							b3LoadUrdfCommandSetFlags(commandHandle, grpcCmd.flags());
						}
					}
				}
				break;
			}

			case CMD_INIT_POSE:
			{
				btAssert(grpcCommand.has_initposecommand());
				if (grpcCommand.has_initposecommand())
				{
					cmdPtr = &cmd;
					b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;

					const ::pybullet_grpc::InitPoseCommand& grpcCmd = grpcCommand.initposecommand();
					b3CreatePoseCommandInit2(commandHandle, grpcCmd.bodyuniqueid());
					cmd.m_updateFlags = grpcCmd.updateflags();
					double initialQ[MAX_DEGREE_OF_FREEDOM] = {0};
					double initialQdot[MAX_DEGREE_OF_FREEDOM] = {0};
					int hasInitialQ[MAX_DEGREE_OF_FREEDOM] = {0};
					int hasInitialQdot[MAX_DEGREE_OF_FREEDOM] = {0};

					if (grpcCmd.initialstateq_size() == grpcCmd.hasinitialstateq_size())
					{
						int numInitial = btMin(grpcCmd.initialstateq_size(), MAX_DEGREE_OF_FREEDOM);
						for (int i = 0; i < numInitial; i++)
						{
							initialQ[i] = grpcCmd.initialstateq(i);
							hasInitialQ[i] = grpcCmd.hasinitialstateq(i);
						}
						if (numInitial)
						{
							b3CreatePoseCommandSetQ(commandHandle, numInitial, initialQ, hasInitialQ);
						}
					}
					else
					{
						printf("Error: if (grpcCmd.initialstateq_size() != grpcCmd.hasinitialstateq_size())\n");
					}
					if (grpcCmd.initialstateqdot_size() == grpcCmd.hasinitialstateqdot_size())
					{
						int numInitial = btMin(grpcCmd.initialstateqdot_size(), MAX_DEGREE_OF_FREEDOM);
						for (int i = 0; i < numInitial; i++)
						{
							initialQdot[i] = grpcCmd.initialstateqdot(i);
							hasInitialQdot[i] = grpcCmd.hasinitialstateqdot(i);
						}
						if (numInitial)
						{
							b3CreatePoseCommandSetQdots(commandHandle, numInitial, initialQdot, hasInitialQdot);
						}
					}
					else
					{
						printf("Error: (grpcCmd.initialstateqdot_size() != grpcCmd.hasinitialstateqdot_size())\n");
					}
				}
				break;
			}

			case CMD_REQUEST_ACTUAL_STATE:
			{
				btAssert(grpcCommand.has_requestactualstatecommand());
				if (grpcCommand.has_requestactualstatecommand())
				{
					cmdPtr = &cmd;
					b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
					const ::pybullet_grpc::RequestActualStateCommand& grpcCmd = grpcCommand.requestactualstatecommand();
					b3RequestActualStateCommandInit2(commandHandle, grpcCmd.bodyuniqueid());
					if (grpcCmd.computeforwardkinematics())
					{
						b3RequestActualStateCommandComputeForwardKinematics(commandHandle, grpcCmd.computeforwardkinematics());
					}
					if (grpcCmd.computelinkvelocities())
					{
						b3RequestActualStateCommandComputeLinkVelocity(commandHandle, grpcCmd.computelinkvelocities());
					}
				}
				break;
			}

			case CMD_SEND_DESIRED_STATE:
			{
				cmdPtr = &cmd;
				b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;

				const ::pybullet_grpc::JointMotorControlCommand* motor = &grpcCommand.jointmotorcontrolcommand();
				b3JointControlCommandInit2Internal(commandHandle, motor->bodyuniqueid(), motor->controlmode());

				int maxQ = motor->hasdesiredstateflags_size();
				for (int q = 0; q < maxQ; q++)
				{
					if (motor->hasdesiredstateflags(q) & SIM_DESIRED_STATE_HAS_Q)
					{
						b3JointControlSetDesiredPosition(commandHandle, q, motor->desiredstateq(q));
					}
					if (motor->hasdesiredstateflags(q) & SIM_DESIRED_STATE_HAS_QDOT)
					{
						b3JointControlSetDesiredVelocity(commandHandle, q, motor->desiredstateqdot(q));
					}
					if (motor->hasdesiredstateflags(q) & SIM_DESIRED_STATE_HAS_KD)
					{
						b3JointControlSetKd(commandHandle, q, motor->kd(q));
					}
					if (motor->hasdesiredstateflags(q) & SIM_DESIRED_STATE_HAS_KP)
					{
						b3JointControlSetKd(commandHandle, q, motor->kp(q));
					}
					if (motor->hasdesiredstateflags(q) & SIM_DESIRED_STATE_HAS_MAX_FORCE)
					{
						b3JointControlSetMaximumForce(commandHandle, q, motor->desiredstateforcetorque(q));
					}
					if (motor->hasdesiredstateflags(q) & SIM_DESIRED_STATE_HAS_RHS_CLAMP)
					{
						b3JointControlSetMaximumVelocity(commandHandle, q, motor->maxvelocity(q));
					}
				}

				//b3JointControlCommandInit2Internal
				break;
			}

			case CMD_SEND_PHYSICS_SIMULATION_PARAMETERS:
			{
				cmdPtr = &cmd;
				b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
				b3InitPhysicsParamCommand2(commandHandle);

				int updateFlags = grpcCommand.setphysicssimulationparameterscommand().updateflags();
				const ::pybullet_grpc::PhysicsSimulationParameters* params = &grpcCommand.setphysicssimulationparameterscommand().params();

				if (updateFlags & SIM_PARAM_UPDATE_DELTA_TIME)
				{
					b3PhysicsParamSetTimeStep(commandHandle, params->deltatime());
				}
				if (updateFlags & SIM_PARAM_UPDATE_GRAVITY)
				{
					const ::pybullet_grpc::vec3* grav = &params->gravityacceleration();
					b3PhysicsParamSetGravity(commandHandle, grav->x(), grav->y(), grav->z());
				}

				if (updateFlags & SIM_PARAM_UPDATE_NUM_SOLVER_ITERATIONS)
				{
					b3PhysicsParamSetNumSolverIterations(commandHandle, params->numsolveriterations());
				}
				if (updateFlags & SIM_PARAM_UPDATE_NUM_SIMULATION_SUB_STEPS)
				{
					b3PhysicsParamSetNumSubSteps(commandHandle, params->numsimulationsubsteps());
				}

				if (updateFlags & SIM_PARAM_UPDATE_REAL_TIME_SIMULATION)
				{
					b3PhysicsParamSetRealTimeSimulation(commandHandle, params->userealtimesimulation());
				}
				if (updateFlags & SIM_PARAM_UPDATE_DEFAULT_CONTACT_ERP)
				{
					b3PhysicsParamSetDefaultContactERP(commandHandle, params->defaultcontacterp());
				}
				if (updateFlags & SIM_PARAM_UPDATE_INTERNAL_SIMULATION_FLAGS)
				{
					b3PhysicsParamSetInternalSimFlags(commandHandle, params->internalsimflags());
				}
				if (updateFlags & SIM_PARAM_UPDATE_USE_SPLIT_IMPULSE)
				{
					b3PhysicsParamSetUseSplitImpulse(commandHandle, params->usesplitimpulse());
				}
				if (updateFlags & SIM_PARAM_UPDATE_SPLIT_IMPULSE_PENETRATION_THRESHOLD)
				{
					b3PhysicsParamSetSplitImpulsePenetrationThreshold(commandHandle, params->splitimpulsepenetrationthreshold());
				}
				if (updateFlags & SIM_PARAM_UPDATE_COLLISION_FILTER_MODE)
				{
					b3PhysicsParamSetCollisionFilterMode(commandHandle, params->collisionfiltermode());
				}
				if (updateFlags & SIM_PARAM_UPDATE_CONTACT_BREAKING_THRESHOLD)
				{
					b3PhysicsParamSetContactBreakingThreshold(commandHandle, params->contactbreakingthreshold());
				}

				if (updateFlags & SIM_PARAM_ENABLE_CONE_FRICTION)
				{
					b3PhysicsParamSetEnableConeFriction(commandHandle, params->enableconefriction());
				}

				if (updateFlags & SIM_PARAM_ENABLE_FILE_CACHING)
				{
					b3PhysicsParamSetEnableFileCaching(commandHandle, params->enablefilecaching());
				}

				if (updateFlags & SIM_PARAM_UPDATE_RESTITUTION_VELOCITY_THRESHOLD)
				{
					b3PhysicsParamSetRestitutionVelocityThreshold(commandHandle, params->restitutionvelocitythreshold());
				}
				if (updateFlags & SIM_PARAM_UPDATE_DEFAULT_NON_CONTACT_ERP)
				{
					b3PhysicsParamSetDefaultNonContactERP(commandHandle, params->defaultnoncontacterp());
				}
				if (updateFlags & SIM_PARAM_UPDATE_DEFAULT_FRICTION_ERP)
				{
					b3PhysicsParamSetDefaultFrictionERP(commandHandle, params->frictionerp());
				}
				if (updateFlags & SIM_PARAM_UPDATE_DETERMINISTIC_OVERLAPPING_PAIRS)
				{
					b3PhysicsParameterSetDeterministicOverlappingPairs(commandHandle, params->deterministicoverlappingpairs());
				}
				if (updateFlags & SIM_PARAM_UPDATE_CCD_ALLOWED_PENETRATION)
				{
					b3PhysicsParameterSetAllowedCcdPenetration(commandHandle, params->allowedccdpenetration());
				}
				if (updateFlags & SIM_PARAM_UPDATE_JOINT_FEEDBACK_MODE)
				{
					b3PhysicsParameterSetJointFeedbackMode(commandHandle, params->jointfeedbackmode());
				}
				if (updateFlags & SIM_PARAM_UPDATE_DEFAULT_GLOBAL_CFM)
				{
					b3PhysicsParamSetDefaultGlobalCFM(commandHandle, params->defaultglobalcfm());
				}
				if (updateFlags & SIM_PARAM_UPDATE_DEFAULT_FRICTION_CFM)
				{
					b3PhysicsParamSetDefaultFrictionCFM(commandHandle, params->frictioncfm());
				}
				if (updateFlags & SIM_PARAM_UPDATE_SOLVER_RESIDULAL_THRESHOLD)
				{
					b3PhysicsParamSetSolverResidualThreshold(commandHandle, params->solverresidualthreshold());
				}
				if (updateFlags & SIM_PARAM_UPDATE_CONTACT_SLOP)
				{
					b3PhysicsParamSetContactSlop(commandHandle, params->contactslop());
				}
				if (updateFlags & SIM_PARAM_ENABLE_SAT)
				{
					b3PhysicsParameterSetEnableSAT(commandHandle, params->enablesat());
				}
				if (updateFlags & SIM_PARAM_CONSTRAINT_SOLVER_TYPE)
				{
					b3PhysicsParameterSetConstraintSolverType(commandHandle, params->constraintsolvertype());
				}
				if (updateFlags & SIM_PARAM_CONSTRAINT_MIN_SOLVER_ISLAND_SIZE)
				{
					b3PhysicsParameterSetMinimumSolverIslandSize(commandHandle, params->minimumsolverislandsize());
				}

				break;
			}

			case CMD_REQUEST_BODY_INFO:
			{
				cmdPtr = &cmd;
				b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
				cmd.m_sdfRequestInfoArgs.m_bodyUniqueId = grpcCommand.requestbodyinfocommand().bodyuniqueid();
				break;
			}

			case CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS:
			{
				cmdPtr = &cmd;
				break;
			}

			case CMD_SYNC_BODY_INFO:
			{
				cmdPtr = &cmd;
				b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
				break;
			}

			case CMD_REQUEST_INTERNAL_DATA:
			{
				cmdPtr = &cmd;
				b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
				break;
			}

			case CMD_CONFIGURE_OPENGL_VISUALIZER:
			{
				btAssert(grpcCommand.has_configureopenglvisualizercommand());
				if (grpcCommand.has_configureopenglvisualizercommand())
				{
					cmdPtr = &cmd;
					b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
					const ::pybullet_grpc::ConfigureOpenGLVisualizerCommand& grpcCmd = grpcCommand.configureopenglvisualizercommand();

					b3InitConfigureOpenGLVisualizer2(commandHandle);
					cmd.m_updateFlags = grpcCmd.updateflags();
					cmd.m_configureOpenGLVisualizerArguments.m_cameraDistance = grpcCmd.cameradistance();
					cmd.m_configureOpenGLVisualizerArguments.m_cameraPitch = grpcCmd.camerapitch();
					cmd.m_configureOpenGLVisualizerArguments.m_cameraYaw = grpcCmd.camerayaw();
					cmd.m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[0] = grpcCmd.cameratargetposition().x();
					cmd.m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[1] = grpcCmd.cameratargetposition().y();
					cmd.m_configureOpenGLVisualizerArguments.m_cameraTargetPosition[2] = grpcCmd.cameratargetposition().z();
					cmd.m_configureOpenGLVisualizerArguments.m_setEnabled = grpcCmd.setenabled();
					cmd.m_configureOpenGLVisualizerArguments.m_setFlag = grpcCmd.setflag();
				}
				break;
			}

			case CMD_REQUEST_CAMERA_IMAGE_DATA:
			{
				cmdPtr = &cmd;
				b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
				SharedMemoryCommand& clientCmd = cmd;

				const ::pybullet_grpc::RequestCameraImageCommand* cam = &grpcCommand.requestcameraimagecommand();

				b3InitRequestCameraImage2(commandHandle);
				clientCmd.m_updateFlags = cam->updateflags();
				clientCmd.m_requestPixelDataArguments.m_startPixelIndex = cam->startpixelindex();

				if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_PIXEL_WIDTH_HEIGHT)
				{
					clientCmd.m_requestPixelDataArguments.m_pixelWidth = cam->pixelwidth();
					clientCmd.m_requestPixelDataArguments.m_pixelHeight = cam->pixelheight();
				}
				if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_SPECULAR_COEFF)
				{
					clientCmd.m_requestPixelDataArguments.m_lightSpecularCoeff = cam->lightspecularcoeff();
				}

				if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_HAS_FLAGS)
				{
					clientCmd.m_requestPixelDataArguments.m_flags = cam->cameraflags();
				}
				if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_SHADOW)
				{
					clientCmd.m_requestPixelDataArguments.m_hasShadow = cam->hasshadow();
				}
				if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_AMBIENT_COEFF)
				{
					clientCmd.m_requestPixelDataArguments.m_lightAmbientCoeff = cam->lightambientcoeff();
				}
				if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_DIFFUSE_COEFF)
				{
					clientCmd.m_requestPixelDataArguments.m_lightDiffuseCoeff = cam->lightdiffusecoeff();
				}
				if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_LIGHT_DISTANCE)
				{
					clientCmd.m_requestPixelDataArguments.m_lightDistance = cam->lightdistance();
				}

				if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_LIGHT_COLOR)
				{
					const ::pybullet_grpc::vec3* lightColor = &cam->lightcolor();
					clientCmd.m_requestPixelDataArguments.m_lightColor[0] = lightColor->x();
					clientCmd.m_requestPixelDataArguments.m_lightColor[1] = lightColor->y();
					clientCmd.m_requestPixelDataArguments.m_lightColor[2] = lightColor->z();
				}
				if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_SET_LIGHT_DIRECTION)
				{
					const ::pybullet_grpc::vec3* lightDir = &cam->lightdirection();
					clientCmd.m_requestPixelDataArguments.m_lightDirection[0] = lightDir->x();
					clientCmd.m_requestPixelDataArguments.m_lightDirection[1] = lightDir->y();
					clientCmd.m_requestPixelDataArguments.m_lightDirection[2] = lightDir->z();
				}

				if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_HAS_CAMERA_MATRICES)
				{
					const ::pybullet_grpc::matrix4x4* projMat = &cam->projectionmatrix();
					const ::pybullet_grpc::matrix4x4* viewMat = &cam->viewmatrix();
					for (int i = 0; i < 16; i++)
					{
						clientCmd.m_requestPixelDataArguments.m_projectionMatrix[i] = projMat->elems(i);
						clientCmd.m_requestPixelDataArguments.m_viewMatrix[i] = viewMat->elems(i);
					}
				}

				if (clientCmd.m_updateFlags & REQUEST_PIXEL_ARGS_HAS_PROJECTIVE_TEXTURE_MATRICES)
				{
					const ::pybullet_grpc::matrix4x4* projectiveProjMat = &cam->projectivetextureprojectionmatrix();
					const ::pybullet_grpc::matrix4x4* projectiveViewMat = &cam->projectivetextureviewmatrix();
					for (int i = 0; i < 16; i++)
					{
						clientCmd.m_requestPixelDataArguments.m_projectiveTextureProjectionMatrix[i] = projectiveProjMat->elems(i);
						clientCmd.m_requestPixelDataArguments.m_projectiveTextureViewMatrix[i] = projectiveViewMat->elems(i);
					}
				}

				break;
			}

#ifdef ALLOW_GRPC_COMMAND_CONVERSION

			case CMD_LOAD_SDF:
			{
				btAssert(grpcCommand.has_loadsdfcommand());
				if (grpcCommand.has_loadsdfcommand())
				{
					const ::pybullet_grpc::LoadSdfCommand& grpcCmd = grpcCommand.loadsdfcommand();

					std::string fileName = grpcCmd.filename();
					if (fileName.length())
					{
						cmdPtr = &cmd;
						b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
						b3LoadSdfCommandInit2(commandHandle, fileName.c_str());

						if (grpcCmd.hasUseMultiBody_case() == ::pybullet_grpc::LoadSdfCommand::HasUseMultiBodyCase::kUseMultiBody)
						{
							b3LoadSdfCommandSetUseMultiBody(commandHandle, grpcCmd.usemultibody());
						}
						if (grpcCmd.hasGlobalScaling_case() == ::pybullet_grpc::LoadSdfCommand::HasGlobalScalingCase::kGlobalScaling)
						{
							b3LoadSdfCommandSetUseGlobalScaling(commandHandle, grpcCmd.globalscaling());
						}
					}
				}
				break;
			}
			case CMD_LOAD_MJCF:
			{
				btAssert(grpcCommand.has_loadmjcfcommand());
				if (grpcCommand.has_loadmjcfcommand())
				{
					const pybullet_grpc::LoadMjcfCommand& grpcCmd = grpcCommand.loadmjcfcommand();

					std::string fileName = grpcCmd.filename();
					if (fileName.length())
					{
						cmdPtr = &cmd;
						b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
						b3LoadMJCFCommandInit2(commandHandle, fileName.c_str());

						if (grpcCmd.flags())
						{
							b3LoadMJCFCommandSetFlags(commandHandle, grpcCmd.flags());
						}
					}
				}
				break;
			}

			case CMD_CHANGE_DYNAMICS_INFO:
			{
				btAssert(grpcCommand.has_changedynamicscommand());
				if (grpcCommand.has_changedynamicscommand())
				{
					cmdPtr = &cmd;
					b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
					const ::pybullet_grpc::ChangeDynamicsCommand& grpcCmd = grpcCommand.changedynamicscommand();
					int bodyUniqueId = grpcCmd.bodyuniqueid();
					int linkIndex = grpcCmd.linkindex();
					b3InitChangeDynamicsInfo2(commandHandle);
					if (grpcCmd.hasMass_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasMassCase::kMass)
					{
						b3ChangeDynamicsInfoSetMass(commandHandle, bodyUniqueId, linkIndex, grpcCmd.mass());
					}
					if (grpcCmd.hasLateralFriction_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasLateralFrictionCase::kLateralFriction)
					{
						b3ChangeDynamicsInfoSetLateralFriction(commandHandle, bodyUniqueId, linkIndex, grpcCmd.lateralfriction());
					}
					if (grpcCmd.hasSpinningFriction_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasSpinningFrictionCase::kSpinningFriction)
					{
						b3ChangeDynamicsInfoSetSpinningFriction(commandHandle, bodyUniqueId, linkIndex, grpcCmd.spinningfriction());
					}
					if (grpcCmd.hasRollingFriction_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasRollingFrictionCase::kRollingFriction)
					{
						b3ChangeDynamicsInfoSetRollingFriction(commandHandle, bodyUniqueId, linkIndex, grpcCmd.rollingfriction());
					}

					if (grpcCmd.hasRestitution_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasRestitutionCase::kRestitution)
					{
						b3ChangeDynamicsInfoSetRestitution(commandHandle, bodyUniqueId, linkIndex, grpcCmd.restitution());
					}
					if (grpcCmd.haslinearDamping_case() == ::pybullet_grpc::ChangeDynamicsCommand::HaslinearDampingCase::kLinearDamping)
					{
						b3ChangeDynamicsInfoSetLinearDamping(commandHandle, bodyUniqueId, grpcCmd.lineardamping());
					}

					if (grpcCmd.hasangularDamping_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasangularDampingCase::kAngularDamping)
					{
						b3ChangeDynamicsInfoSetAngularDamping(commandHandle, bodyUniqueId, grpcCmd.angulardamping());
					}

					bool hasContactDamping = grpcCmd.hasContactDamping_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasContactDampingCase::kContactDamping;
					bool hasContactStiffness = grpcCmd.hasContactStiffness_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasContactStiffnessCase::kContactStiffness;
					if (hasContactDamping && hasContactStiffness)
					{
						b3ChangeDynamicsInfoSetContactStiffnessAndDamping(commandHandle, bodyUniqueId, linkIndex, grpcCmd.contactstiffness(), grpcCmd.contactdamping());
					}
					if (grpcCmd.hasLocalInertiaDiagonal_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasLocalInertiaDiagonalCase::kLocalInertiaDiagonal)
					{
						double localInertiaDiag[3] = {grpcCmd.localinertiadiagonal().x(), grpcCmd.localinertiadiagonal().y(), grpcCmd.localinertiadiagonal().z()};
						b3ChangeDynamicsInfoSetLocalInertiaDiagonal(commandHandle, bodyUniqueId, linkIndex, localInertiaDiag);
					}

					if (grpcCmd.hasFrictionAnchor_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasFrictionAnchorCase::kFrictionAnchor)
					{
						b3ChangeDynamicsInfoSetFrictionAnchor(commandHandle, bodyUniqueId, linkIndex, grpcCmd.frictionanchor());
					}
					if (grpcCmd.hasccdSweptSphereRadius_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasccdSweptSphereRadiusCase::kCcdSweptSphereRadius)
					{
						b3ChangeDynamicsInfoSetCcdSweptSphereRadius(commandHandle, bodyUniqueId, linkIndex, grpcCmd.ccdsweptsphereradius());
					}

					if (grpcCmd.hasContactProcessingThreshold_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasContactProcessingThresholdCase::kContactProcessingThreshold)
					{
						b3ChangeDynamicsInfoSetContactProcessingThreshold(commandHandle, bodyUniqueId, linkIndex, grpcCmd.contactprocessingthreshold());
					}

					if (grpcCmd.hasActivationState_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasActivationStateCase::kActivationState)
					{
						b3ChangeDynamicsInfoSetActivationState(commandHandle, bodyUniqueId, grpcCmd.activationstate());
					}
				}
				break;
			}
			case CMD_GET_DYNAMICS_INFO:
			{
				btAssert(grpcCommand.has_getdynamicscommand());
				if (grpcCommand.has_getdynamicscommand())
				{
					cmdPtr = &cmd;
					b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
					const ::pybullet_grpc::GetDynamicsCommand& grpcCmd = grpcCommand.getdynamicscommand();
					b3GetDynamicsInfoCommandInit2(commandHandle, grpcCmd.bodyuniqueid(), grpcCmd.linkindex());
				}
				break;
			}

#endif  //ALLOW_GRPC_COMMAND_CONVERSION
			default:
			{
				printf("unknown convertGRPCToBulletCommand");
				assert(0);
			}
		}
	}

	if (0 == cmdPtr)
	{
		printf("Error converting convertGRPCToBulletCommand");
	}
	btAssert(cmdPtr);

	return cmdPtr;
}

bool convertGRPCToStatus(const PyBulletStatus& grpcReply, SharedMemoryStatus& serverStatus, char* bufferServerToClient, int bufferSizeInBytes)
{
	bool converted = false;

	serverStatus.m_type = grpcReply.statustype();

	int sz = grpcReply.binaryblob().size();
	if (sz > 0)
	{
		if (sz == 1)
		{
			const char* data = grpcReply.binaryblob().Get(0).c_str();
			int numBytes = grpcReply.binaryblob().Get(0).size();
			//printf("copied binary blob of %d bytes\n", numBytes);
			memcpy(bufferServerToClient, data, numBytes);
			serverStatus.m_numDataStreamBytes = numBytes;
		}
		else
		{
			printf("Ignore unexpected binary blobs\n");
		}
	}
#ifdef ALLOW_GRPC_STATUS_CONVERSION
	switch (grpcReply.statustype())
	{
		case CMD_ACTUAL_STATE_UPDATE_COMPLETED:
		{
			converted = true;
			const ::pybullet_grpc::SendActualStateStatus* stat = &grpcReply.actualstatestatus();
			serverStatus.m_sendActualStateArgs.m_bodyUniqueId = stat->bodyuniqueid();
			int numLinks = stat->numlinks();
			serverStatus.m_sendActualStateArgs.m_numLinks = numLinks;

			int numDegreeOfFreedomQ = stat->numdegreeoffreedomq();
			int numDegreeOfFreedomU = stat->numdegreeoffreedomu();
			serverStatus.m_sendActualStateArgs.m_numDegreeOfFreedomQ = numDegreeOfFreedomQ;
			serverStatus.m_sendActualStateArgs.m_numDegreeOfFreedomU = numDegreeOfFreedomU;

			serverStatus.m_sendActualStateArgs.m_stateDetails = (SendActualStateSharedMemoryStorage*)bufferServerToClient;

			for (int i = 0; i < numDegreeOfFreedomQ; i++)
			{
				serverStatus.m_sendActualStateArgs.m_stateDetails->m_actualStateQ[i] = stat->actualstateq(i);
			}
			for (int i = 0; i < numDegreeOfFreedomU; i++)
			{
				serverStatus.m_sendActualStateArgs.m_stateDetails->m_actualStateQdot[i] = stat->actualstateqdot(i);
			}
			for (int i = 0; i < 7; i++)
			{
				serverStatus.m_sendActualStateArgs.m_rootLocalInertialFrame[i] = stat->rootlocalinertialframe(i);
			}
			for (int i = 0; i < numLinks * 7; i++)
			{
				serverStatus.m_sendActualStateArgs.m_stateDetails->m_linkLocalInertialFrames[i] = stat->linklocalinertialframes(i);
			}
			for (int i = 0; i < numLinks * 6; i++)
			{
				serverStatus.m_sendActualStateArgs.m_stateDetails->m_jointReactionForces[i] = stat->jointreactionforces(i);
			}
			for (int i = 0; i < numLinks; i++)
			{
				serverStatus.m_sendActualStateArgs.m_stateDetails->m_jointMotorForce[i] = stat->jointmotorforce(i);
			}
			for (int i = 0; i < numLinks * 7; i++)
			{
				serverStatus.m_sendActualStateArgs.m_stateDetails->m_linkState[i] = stat->linkstate(i);
			}
			for (int i = 0; i < numLinks * 6; i++)
			{
				serverStatus.m_sendActualStateArgs.m_stateDetails->m_linkWorldVelocities[i] = stat->linkworldvelocities(i);
			}

			break;
		}

		case CMD_REQUEST_KEYBOARD_EVENTS_DATA_COMPLETED:
		{
			converted = true;
			const ::pybullet_grpc::KeyboardEventsStatus* keys = &grpcReply.keyboardeventsstatus();

			serverStatus.m_sendKeyboardEvents.m_numKeyboardEvents = keys->keyboardevents_size();

			for (int i = 0; i < serverStatus.m_sendKeyboardEvents.m_numKeyboardEvents; i++)
			{
				const ::pybullet_grpc::KeyboardEvent* key = &keys->keyboardevents(i);
				serverStatus.m_sendKeyboardEvents.m_keyboardEvents[i].m_keyCode = key->keycode();
				serverStatus.m_sendKeyboardEvents.m_keyboardEvents[i].m_keyState = key->keystate();
			}

			break;
		}

		case CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS_COMPLETED:
		{
			const ::pybullet_grpc::PhysicsSimulationParameters* params = &grpcReply.requestphysicssimulationparametersstatus();
			serverStatus.m_simulationParameterResultArgs.m_allowedCcdPenetration = params->allowedccdpenetration();
			serverStatus.m_simulationParameterResultArgs.m_collisionFilterMode = params->collisionfiltermode();
			serverStatus.m_simulationParameterResultArgs.m_constraintSolverType = params->constraintsolvertype();
			serverStatus.m_simulationParameterResultArgs.m_contactBreakingThreshold = params->contactbreakingthreshold();
			serverStatus.m_simulationParameterResultArgs.m_contactSlop = params->contactslop();
			serverStatus.m_simulationParameterResultArgs.m_defaultContactERP = params->defaultcontacterp();
			serverStatus.m_simulationParameterResultArgs.m_defaultGlobalCFM = params->defaultglobalcfm();
			serverStatus.m_simulationParameterResultArgs.m_defaultNonContactERP = params->defaultnoncontacterp();
			serverStatus.m_simulationParameterResultArgs.m_deltaTime = params->deltatime();
			serverStatus.m_simulationParameterResultArgs.m_deterministicOverlappingPairs = params->deterministicoverlappingpairs();
			serverStatus.m_simulationParameterResultArgs.m_enableConeFriction = params->enableconefriction();
			serverStatus.m_simulationParameterResultArgs.m_enableFileCaching = params->enablefilecaching();
			serverStatus.m_simulationParameterResultArgs.m_enableSAT = params->enablesat();
			serverStatus.m_simulationParameterResultArgs.m_frictionCFM = params->frictioncfm();
			serverStatus.m_simulationParameterResultArgs.m_frictionERP = params->frictionerp();
			serverStatus.m_simulationParameterResultArgs.m_gravityAcceleration[0] = params->gravityacceleration().x();
			serverStatus.m_simulationParameterResultArgs.m_gravityAcceleration[1] = params->gravityacceleration().y();
			serverStatus.m_simulationParameterResultArgs.m_gravityAcceleration[2] = params->gravityacceleration().z();
			serverStatus.m_simulationParameterResultArgs.m_internalSimFlags = params->internalsimflags();
			serverStatus.m_simulationParameterResultArgs.m_jointFeedbackMode = params->jointfeedbackmode();
			serverStatus.m_simulationParameterResultArgs.m_minimumSolverIslandSize = params->minimumsolverislandsize();
			serverStatus.m_simulationParameterResultArgs.m_numSimulationSubSteps = params->numsimulationsubsteps();
			serverStatus.m_simulationParameterResultArgs.m_numSolverIterations = params->numsolveriterations();
			serverStatus.m_simulationParameterResultArgs.m_restitutionVelocityThreshold = params->restitutionvelocitythreshold();
			serverStatus.m_simulationParameterResultArgs.m_solverResidualThreshold = params->solverresidualthreshold();
			serverStatus.m_simulationParameterResultArgs.m_splitImpulsePenetrationThreshold = params->splitimpulsepenetrationthreshold();
			serverStatus.m_simulationParameterResultArgs.m_useRealTimeSimulation = params->userealtimesimulation();
			serverStatus.m_simulationParameterResultArgs.m_useSplitImpulse = params->usesplitimpulse();

			converted = true;
			break;
		}
		case CMD_BODY_INFO_COMPLETED:
		{
			converted = true;
			serverStatus.m_dataStreamArguments.m_bodyUniqueId = grpcReply.requestbodyinfostatus().bodyuniqueid();
			serverStatus.m_dataStreamArguments.m_bodyName[0] = 0;
			serverStatus.m_dataStreamArguments.m_bulletFileName[0] = 0;
			if (grpcReply.requestbodyinfostatus().bodyname().length())
			{
				strcpy(serverStatus.m_dataStreamArguments.m_bodyName, grpcReply.requestbodyinfostatus().bodyname().c_str());
			}

			break;
		}
		case CMD_SYNC_BODY_INFO_COMPLETED:
		{
			serverStatus.m_sdfLoadedArgs.m_numBodies = grpcReply.syncbodiesstatus().bodyuniqueids_size();
			for (int i = 0; i < serverStatus.m_sdfLoadedArgs.m_numBodies; i++)
			{
				serverStatus.m_sdfLoadedArgs.m_bodyUniqueIds[i] = grpcReply.syncbodiesstatus().bodyuniqueids(i);
			}
			serverStatus.m_sdfLoadedArgs.m_numUserConstraints = grpcReply.syncbodiesstatus().userconstraintuniqueids_size();
			for (int i = 0; i < serverStatus.m_sdfLoadedArgs.m_numUserConstraints; i++)
			{
				serverStatus.m_sdfLoadedArgs.m_userConstraintUniqueIds[i] = grpcReply.syncbodiesstatus().userconstraintuniqueids(i);
			}

			converted = true;
			break;
		}
		case CMD_CLIENT_COMMAND_COMPLETED:
		{
			converted = true;
			break;
		}
		case CMD_REQUEST_INTERNAL_DATA_COMPLETED:
		{
			converted = true;
			break;
		}

		case CMD_URDF_LOADING_COMPLETED:
		{
			converted = true;
			serverStatus.m_dataStreamArguments.m_bodyUniqueId = grpcReply.urdfstatus().bodyuniqueid();
			if (grpcReply.urdfstatus().bodyname().length() > 0 && grpcReply.urdfstatus().bodyname().length() < (MAX_FILENAME_LENGTH - 1))
			{
				strcpy(serverStatus.m_dataStreamArguments.m_bodyName, grpcReply.urdfstatus().bodyname().c_str());
			}
			else
			{
				serverStatus.m_dataStreamArguments.m_bodyName[0] = 0;
			}
			if (grpcReply.urdfstatus().filename().length() > 0 && grpcReply.urdfstatus().filename().length() < (MAX_FILENAME_LENGTH - 1))
			{
				strcpy(serverStatus.m_dataStreamArguments.m_bulletFileName, grpcReply.urdfstatus().filename().c_str());
			}
			else
			{
				serverStatus.m_dataStreamArguments.m_bulletFileName[0] = 0;
			}

			break;
		}
		case CMD_SDF_LOADING_COMPLETED:
		{
			converted = true;
      const ::pybullet_grpc::SdfLoadedStatus* stat = &grpcReply.sdfstatus();
      int numBodies = stat->bodyuniqueids_size();
      if (numBodies > MAX_SDF_BODIES)
			{
				printf("SDF exceeds body capacity: %d > %d", numBodies, MAX_SDF_BODIES);
			}
      serverStatus.m_sdfLoadedArgs.m_numBodies = numBodies;
      for (int i = 0; i < numBodies; i++)
			{
        serverStatus.m_sdfLoadedArgs.m_bodyUniqueIds[i] = stat->bodyuniqueids(i);
			}
			break;
		}
		case CMD_DESIRED_STATE_RECEIVED_COMPLETED:
		{
			converted = true;
			break;
		}
		case CMD_USER_CONSTRAINT_COMPLETED:
		{
			const ::pybullet_grpc::UserConstraintStatus* con = &grpcReply.userconstraintstatus();
			serverStatus.m_userConstraintResultArgs.m_userConstraintUniqueId = con->userconstraintuniqueid();
			serverStatus.m_userConstraintResultArgs.m_maxAppliedForce = con->maxappliedforce();
			converted = true;
			break;
		}
		case CMD_STEP_FORWARD_SIMULATION_COMPLETED:
		{
			converted = true;
			break;
		}
		case CMD_RESET_SIMULATION_COMPLETED:
		{
			converted = true;
			break;
		}
		case CMD_CAMERA_IMAGE_COMPLETED:
		{
			converted = true;
			const ::pybullet_grpc::RequestCameraImageStatus* cam = &grpcReply.requestcameraimagestatus();
			serverStatus.m_sendPixelDataArguments.m_imageWidth = cam->imagewidth();
			serverStatus.m_sendPixelDataArguments.m_imageHeight = cam->imageheight();
			serverStatus.m_sendPixelDataArguments.m_numPixelsCopied = cam->numpixelscopied();
			serverStatus.m_sendPixelDataArguments.m_numRemainingPixels = cam->numremainingpixels();
			serverStatus.m_sendPixelDataArguments.m_startingPixelIndex = cam->startingpixelindex();
			break;
		}
		case CMD_GET_DYNAMICS_INFO_COMPLETED:
		{
			converted = true;
			const ::pybullet_grpc::GetDynamicsStatus* stat = &grpcReply.getdynamicsstatus();
			serverStatus.m_dynamicsInfo.m_mass = stat->mass();
			serverStatus.m_dynamicsInfo.m_lateralFrictionCoeff = stat->lateralfriction();
			serverStatus.m_dynamicsInfo.m_spinningFrictionCoeff = stat->spinningfriction();
			serverStatus.m_dynamicsInfo.m_rollingFrictionCoeff = stat->rollingfriction();
			serverStatus.m_dynamicsInfo.m_restitution = stat->restitution();
			serverStatus.m_dynamicsInfo.m_linearDamping = stat->lineardamping();
			serverStatus.m_dynamicsInfo.m_angularDamping = stat->angulardamping();
			serverStatus.m_dynamicsInfo.m_contactStiffness = stat->contactstiffness();
			serverStatus.m_dynamicsInfo.m_contactDamping = stat->contactdamping();
			serverStatus.m_dynamicsInfo.m_localInertialDiagonal[0] = stat->localinertiadiagonal().x();
			serverStatus.m_dynamicsInfo.m_localInertialDiagonal[1] = stat->localinertiadiagonal().y();
			serverStatus.m_dynamicsInfo.m_localInertialDiagonal[2] = stat->localinertiadiagonal().z();
			serverStatus.m_dynamicsInfo.m_frictionAnchor = stat->frictionanchor();
			serverStatus.m_dynamicsInfo.m_ccdSweptSphereRadius = stat->ccdsweptsphereradius();
			serverStatus.m_dynamicsInfo.m_contactProcessingThreshold = stat->contactprocessingthreshold();
			serverStatus.m_dynamicsInfo.m_activationState = stat->activationstate();
			break;
		}

		default:
		{
#endif  //ALLOW_GRPC_STATUS_CONVERSION
			if (grpcReply.unknownstatusbinaryblob_size() > 0)
			{
				if (grpcReply.unknownstatusbinaryblob_size() == 1)
				{
					//printf("convertStatusToGRPC: slow fallback status (%d), slow fallback", grpcReply.statustype());

					const char* data = grpcReply.unknownstatusbinaryblob().Get(0).c_str();
					int numBytes = grpcReply.unknownstatusbinaryblob().Get(0).size();

					btAssert(sizeof(SharedMemoryStatus) == numBytes);
					if (sizeof(SharedMemoryStatus) == numBytes)
					{
						memcpy(&serverStatus, data, numBytes);
					}
					//printf("slow fallback on command type %d\n", serverStatus.m_type);
					btAssert(grpcReply.statustype() == serverStatus.m_type);
					converted = true;
				}
				else
				{
					printf("unexpected unknownstatusbinaryblob_size\n");
				}
			}
			else
			{
				printf("unknown status and no slow fallback in convertStatusToGRPC %d\n", grpcReply.statustype());
			}

#ifdef ALLOW_GRPC_STATUS_CONVERSION
		}
	};
#endif  //ALLOW_GRPC_STATUS_CONVERSION
	return converted;
}

bool convertStatusToGRPC(const SharedMemoryStatus& serverStatus, char* bufferServerToClient, int bufferSizeInBytes, PyBulletStatus& grpcReply)
{
	bool converted = false;
	grpcReply.set_statustype(serverStatus.m_type);

	if (serverStatus.m_numDataStreamBytes)
	{
		grpcReply.add_binaryblob(bufferServerToClient, serverStatus.m_numDataStreamBytes);
	}

#ifdef ALLOW_GRPC_STATUS_CONVERSION
	switch (serverStatus.m_type)
	{
		case CMD_REQUEST_KEYBOARD_EVENTS_DATA_COMPLETED:
		{
			converted = true;
			::pybullet_grpc::KeyboardEventsStatus* keys = grpcReply.mutable_keyboardeventsstatus();

			for (int i = 0; i < serverStatus.m_sendKeyboardEvents.m_numKeyboardEvents; i++)
			{
				::pybullet_grpc::KeyboardEvent* key = keys->add_keyboardevents();
				key->set_keycode(serverStatus.m_sendKeyboardEvents.m_keyboardEvents[i].m_keyCode);
				key->set_keystate(serverStatus.m_sendKeyboardEvents.m_keyboardEvents[i].m_keyState);
			}

			break;
		}
		case CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS_COMPLETED:
		{
			::pybullet_grpc::PhysicsSimulationParameters* params = grpcReply.mutable_requestphysicssimulationparametersstatus();
			params->set_allowedccdpenetration(serverStatus.m_simulationParameterResultArgs.m_allowedCcdPenetration);
			params->set_collisionfiltermode(serverStatus.m_simulationParameterResultArgs.m_collisionFilterMode);
			params->set_constraintsolvertype(serverStatus.m_simulationParameterResultArgs.m_constraintSolverType);
			params->set_contactbreakingthreshold(serverStatus.m_simulationParameterResultArgs.m_contactBreakingThreshold);
			params->set_contactslop(serverStatus.m_simulationParameterResultArgs.m_contactSlop);
			params->set_defaultcontacterp(serverStatus.m_simulationParameterResultArgs.m_defaultContactERP);
			params->set_defaultglobalcfm(serverStatus.m_simulationParameterResultArgs.m_defaultGlobalCFM);
			params->set_defaultnoncontacterp(serverStatus.m_simulationParameterResultArgs.m_defaultNonContactERP);
			params->set_deltatime(serverStatus.m_simulationParameterResultArgs.m_deltaTime);
			params->set_deterministicoverlappingpairs(serverStatus.m_simulationParameterResultArgs.m_deterministicOverlappingPairs);
			params->set_enableconefriction(serverStatus.m_simulationParameterResultArgs.m_enableConeFriction);
			params->set_enablefilecaching(serverStatus.m_simulationParameterResultArgs.m_enableFileCaching);
			params->set_enablesat(serverStatus.m_simulationParameterResultArgs.m_enableSAT);
			params->set_frictioncfm(serverStatus.m_simulationParameterResultArgs.m_frictionCFM);
			params->set_frictionerp(serverStatus.m_simulationParameterResultArgs.m_frictionERP);
			::pybullet_grpc::vec3* grav = params->mutable_gravityacceleration();
			grav->set_x(serverStatus.m_simulationParameterResultArgs.m_gravityAcceleration[0]);
			grav->set_y(serverStatus.m_simulationParameterResultArgs.m_gravityAcceleration[1]);
			grav->set_z(serverStatus.m_simulationParameterResultArgs.m_gravityAcceleration[2]);
			params->set_internalsimflags(serverStatus.m_simulationParameterResultArgs.m_internalSimFlags);
			params->set_jointfeedbackmode(serverStatus.m_simulationParameterResultArgs.m_jointFeedbackMode);
			params->set_minimumsolverislandsize(serverStatus.m_simulationParameterResultArgs.m_minimumSolverIslandSize);
			params->set_numsimulationsubsteps(serverStatus.m_simulationParameterResultArgs.m_numSimulationSubSteps);
			params->set_numsolveriterations(serverStatus.m_simulationParameterResultArgs.m_numSolverIterations);
			params->set_restitutionvelocitythreshold(serverStatus.m_simulationParameterResultArgs.m_restitutionVelocityThreshold);
			params->set_solverresidualthreshold(serverStatus.m_simulationParameterResultArgs.m_solverResidualThreshold);
			params->set_splitimpulsepenetrationthreshold(serverStatus.m_simulationParameterResultArgs.m_splitImpulsePenetrationThreshold);
			params->set_userealtimesimulation(serverStatus.m_simulationParameterResultArgs.m_useRealTimeSimulation);
			params->set_usesplitimpulse(serverStatus.m_simulationParameterResultArgs.m_useSplitImpulse);

			converted = true;
			break;
		}
		case CMD_BODY_INFO_COMPLETED:
		{
			converted = true;
			::pybullet_grpc::RequestBodyInfoStatus* stat = grpcReply.mutable_requestbodyinfostatus();
			stat->set_bodyuniqueid(serverStatus.m_dataStreamArguments.m_bodyUniqueId);
			stat->set_bodyname(serverStatus.m_dataStreamArguments.m_bodyName);
			break;
		}
		case CMD_SYNC_BODY_INFO_COMPLETED:
		{
			::pybullet_grpc::SyncBodiesStatus* stat = grpcReply.mutable_syncbodiesstatus();

			for (int i = 0; i < serverStatus.m_sdfLoadedArgs.m_numBodies; i++)
			{
				stat->add_bodyuniqueids(serverStatus.m_sdfLoadedArgs.m_bodyUniqueIds[i]);
			}
			for (int i = 0; i < serverStatus.m_sdfLoadedArgs.m_numUserConstraints; i++)
			{
				stat->add_userconstraintuniqueids(serverStatus.m_sdfLoadedArgs.m_userConstraintUniqueIds[i]);
			}

			converted = true;
			break;
		}

		case CMD_REQUEST_INTERNAL_DATA_COMPLETED:
		{
			converted = true;
			break;
		}
		case CMD_URDF_LOADING_COMPLETED:
		{
			converted = true;
			::pybullet_grpc::LoadUrdfStatus* stat = grpcReply.mutable_urdfstatus();
			b3SharedMemoryStatusHandle statusHandle = (b3SharedMemoryStatusHandle)&serverStatus;
			int objectUniqueId = b3GetStatusBodyIndex(statusHandle);
			stat->set_bodyuniqueid(objectUniqueId);

			break;
		}
		case CMD_SDF_LOADING_COMPLETED:
		{
			converted = true;
			int bodyIndicesOut[MAX_SDF_BODIES];
			::pybullet_grpc::SdfLoadedStatus* stat = grpcReply.mutable_sdfstatus();
			b3SharedMemoryStatusHandle statusHandle = (b3SharedMemoryStatusHandle)&serverStatus;
			int numBodies = b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES);
			if (numBodies > MAX_SDF_BODIES)
			{
				printf("SDF exceeds body capacity: %d > %d", numBodies, MAX_SDF_BODIES);
			}
			for (int i = 0; i < numBodies; i++)
			{
				stat->add_bodyuniqueids(bodyIndicesOut[i]);
			}
			break;
		}
		case CMD_MJCF_LOADING_COMPLETED:
		{
			converted = true;
			int bodyIndicesOut[MAX_SDF_BODIES];
			::pybullet_grpc::MjcfLoadedStatus* stat = grpcReply.mutable_mjcfstatus();
			b3SharedMemoryStatusHandle statusHandle = (b3SharedMemoryStatusHandle)&serverStatus;
			int numBodies = b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES);
			if (numBodies > MAX_SDF_BODIES)
			{
				printf("MJCF exceeds body capacity: %d > %d", numBodies, MAX_SDF_BODIES);
			}
			for (int i = 0; i < numBodies; i++)
			{
				stat->add_bodyuniqueids(bodyIndicesOut[i]);
			}
			break;
		}
		case CMD_GET_DYNAMICS_INFO_COMPLETED:
		{
			converted = true;
			b3DynamicsInfo info;
			b3SharedMemoryStatusHandle statusHandle = (b3SharedMemoryStatusHandle)&serverStatus;
			if (b3GetDynamicsInfo(statusHandle, &info))
			{
				::pybullet_grpc::GetDynamicsStatus* stat = grpcReply.mutable_getdynamicsstatus();

				stat->set_mass(info.m_mass);
				stat->set_lateralfriction(info.m_lateralFrictionCoeff);
				stat->set_spinningfriction(info.m_spinningFrictionCoeff);
				stat->set_rollingfriction(info.m_rollingFrictionCoeff);
				stat->set_restitution(info.m_restitution);
				stat->set_lineardamping(info.m_linearDamping);
				stat->set_angulardamping(info.m_angularDamping);

				stat->set_contactstiffness(info.m_contactStiffness);
				stat->set_contactdamping(info.m_contactDamping);

				pybullet_grpc::vec3* localInertia = stat->mutable_localinertiadiagonal();
				localInertia->set_x(info.m_localInertialDiagonal[0]);
				localInertia->set_y(info.m_localInertialDiagonal[1]);
				localInertia->set_z(info.m_localInertialDiagonal[2]);

				stat->set_frictionanchor(info.m_frictionAnchor);
				stat->set_ccdsweptsphereradius(info.m_ccdSweptSphereRadius);
				stat->set_contactprocessingthreshold(info.m_contactProcessingThreshold);

				stat->set_activationstate(info.m_activationState);
			}
			break;
		}
		case CMD_ACTUAL_STATE_UPDATE_COMPLETED:
		{
			converted = true;
			SharedMemoryStatus* status = (SharedMemoryStatus*)&serverStatus;
      status->m_sendActualStateArgs.m_stateDetails = (SendActualStateSharedMemoryStorage*)bufferServerToClient;
			b3SharedMemoryStatusHandle statusHandle = (b3SharedMemoryStatusHandle)status;

			int bodyUniqueId;
			int numLinks;

			int numDegreeOfFreedomQ;
			int numDegreeOfFreedomU;

			const double* rootLocalInertialFramePtr = 0;
			const double* actualStateQptr = 0;
			const double* actualStateQdotPtr = 0;
			const double* jointReactionForcesPtr = 0;

			const double* linkLocalInertialFrames = 0;
			const double* jointMotorForces = 0;
			const double* linkStates = 0;
			const double* linkWorldVelocities = 0;

			if (b3GetStatusActualState2(
					statusHandle, &bodyUniqueId, &numLinks,
					&numDegreeOfFreedomQ,
					&numDegreeOfFreedomU,
					&rootLocalInertialFramePtr,
					&actualStateQptr,
					&actualStateQdotPtr,
					&jointReactionForcesPtr,
					&linkLocalInertialFrames,
					&jointMotorForces,
					&linkStates,
					&linkWorldVelocities))
			{
				::pybullet_grpc::SendActualStateStatus* stat = grpcReply.mutable_actualstatestatus();
				stat->set_bodyuniqueid(bodyUniqueId);
				stat->set_numlinks(numLinks);

				stat->set_numdegreeoffreedomq(numDegreeOfFreedomQ);
				stat->set_numdegreeoffreedomu(numDegreeOfFreedomU);
				for (int i = 0; i < numDegreeOfFreedomQ; i++)
				{
					stat->add_actualstateq(actualStateQptr[i]);
				}
				for (int i = 0; i < numDegreeOfFreedomU; i++)
				{
					stat->add_actualstateqdot(actualStateQdotPtr[i]);
				}
				for (int i = 0; i < 7; i++)
				{
					stat->add_rootlocalinertialframe(rootLocalInertialFramePtr[i]);
				}
				for (int i = 0; i < numLinks * 7; i++)
				{
					stat->add_linklocalinertialframes(linkLocalInertialFrames[i]);
				}
				for (int i = 0; i < numLinks * 6; i++)
				{
					stat->add_jointreactionforces(jointReactionForcesPtr[i]);
				}
				for (int i = 0; i < numLinks; i++)
				{
					stat->add_jointmotorforce(jointMotorForces[i]);
				}
				for (int i = 0; i < numLinks * 7; i++)
				{
					stat->add_linkstate(linkStates[i]);
				}
				for (int i = 0; i < numLinks * 6; i++)
				{
					stat->add_linkworldvelocities(linkWorldVelocities[i]);
				}
			}
			break;
		}
		case CMD_CLIENT_COMMAND_COMPLETED:
		{
			converted = true;
			//no action needed?
			break;
		}
		case CMD_DESIRED_STATE_RECEIVED_COMPLETED:
		{
			converted = true;
			break;
		}
		case CMD_USER_CONSTRAINT_COMPLETED:
		{
			::pybullet_grpc::UserConstraintStatus* con = grpcReply.mutable_userconstraintstatus();
			con->set_userconstraintuniqueid(serverStatus.m_userConstraintResultArgs.m_userConstraintUniqueId);
			con->set_maxappliedforce(serverStatus.m_userConstraintResultArgs.m_maxAppliedForce);
			converted = true;
			break;
		}
		case CMD_STEP_FORWARD_SIMULATION_COMPLETED:
		{
			converted = true;
			break;
		}
		case CMD_RESET_SIMULATION_COMPLETED:
		{
			converted = true;
			break;
		}
		case CMD_CAMERA_IMAGE_COMPLETED:
		{
			converted = true;
			::pybullet_grpc::RequestCameraImageStatus* cam = grpcReply.mutable_requestcameraimagestatus();
			cam->set_imagewidth(serverStatus.m_sendPixelDataArguments.m_imageWidth);
			cam->set_imageheight(serverStatus.m_sendPixelDataArguments.m_imageHeight);
			cam->set_numpixelscopied(serverStatus.m_sendPixelDataArguments.m_numPixelsCopied);
			cam->set_numremainingpixels(serverStatus.m_sendPixelDataArguments.m_numRemainingPixels);
			cam->set_startingpixelindex(serverStatus.m_sendPixelDataArguments.m_startingPixelIndex);
			break;
		}
			/*
	case CMD_USER_CONSTRAINT_INFO_COMPLETED:
	{
	}
	case CMD_USER_CONSTRAINT_REQUEST_STATE_COMPLETED:
	{
	}
	*/

		default:
		{
#endif  //ALLOW_GRPC_STATUS_CONVERSION \
	//printf("convertStatusToGRPC: unknown status (%d), slow fallback", serverStatus.m_type);
			int sz = sizeof(SharedMemoryStatus);
			grpcReply.add_unknownstatusbinaryblob((const char*)&serverStatus, sz);
			converted = true;
#ifdef ALLOW_GRPC_STATUS_CONVERSION
		}
	}
#endif  //ALLOW_GRPC_STATUS_CONVERSION
	return converted;
}
