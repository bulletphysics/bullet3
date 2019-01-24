#pragma once

#include <vector>
#include <fstream>
#include "Shape.h"
#ifdef USE_JSON
#include "json/json.h"
#endif
#include "MathUtil.h"

class cKinTree
{
public:
	// description of the joint tree representing an articulated figure
	enum eJointType
	{
		eJointTypeRevolute,
		eJointTypePlanar,
		eJointTypePrismatic,
		eJointTypeFixed,
		eJointTypeSpherical,
		eJointTypeNone,
		eJointTypeMax
	};

	enum eJointDesc
	{
		eJointDescType,
		eJointDescParent,
		eJointDescAttachX,
		eJointDescAttachY,
		eJointDescAttachZ,
		eJointDescAttachThetaX, // euler angles order rot(Z) * rot(Y) * rot(X)
		eJointDescAttachThetaY,
		eJointDescAttachThetaZ,
		eJointDescLimLow0,
		eJointDescLimLow1,
		eJointDescLimLow2,
		eJointDescLimHigh0,
		eJointDescLimHigh1,
		eJointDescLimHigh2,
		eJointDescTorqueLim,
		eJointDescForceLim,
		eJointDescIsEndEffector,
		eJointDescDiffWeight,
		eJointDescParamOffset,
		eJointDescMax
	};
	typedef Eigen::Matrix<double, 1, eJointDescMax> tJointDesc;

	enum eBodyParam
	{
		eBodyParamShape,
		eBodyParamMass,
		eBodyParamColGroup, // 0 collides with nothing and 1 collides with everything
		eBodyParamEnableFallContact,
		eBodyParamAttachX,
		eBodyParamAttachY,
		eBodyParamAttachZ,
		eBodyParamAttachThetaX, // Euler angles order XYZ
		eBodyParamAttachThetaY,
		eBodyParamAttachThetaZ,
		eBodyParam0,
		eBodyParam1,
		eBodyParam2,
		eBodyColorR,
		eBodyColorG,
		eBodyColorB,
		eBodyColorA,
		eBodyParamMax
	};
	typedef Eigen::Matrix<double, 1, eBodyParamMax> tBodyDef;

	enum eDrawShape
	{
		eDrawShapeShape,
		eDrawShapeParentJoint,
		eDrawShapeAttachX,
		eDrawShapeAttachY,
		eDrawShapeAttachZ,
		eDrawShapeAttachThetaX, // Euler angles order XYZ
		eDrawShapeAttachThetaY,
		eDrawShapeAttachThetaZ,
		eDrawShapeParam0,
		eDrawShapeParam1,
		eDrawShapeParam2,
		eDrawShapeColorR,
		eDrawShapeColorG,
		eDrawShapeColorB,
		eDrawShapeColorA,
		eDrawShapeMeshID,
		eDrawShapeParamMax
	};
	typedef Eigen::Matrix<double, 1, eDrawShapeParamMax> tDrawShapeDef;

	static const int gInvalidJointID;
	static const int gPosDim;
	static const int gRotDim;
	static const int gRootDim;

	static bool HasValidRoot(const Eigen::MatrixXd& joint_mat);
	static tVector GetRootPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state);
	static void SetRootPos(const Eigen::MatrixXd& joint_mat, const tVector& pos, Eigen::VectorXd& out_state);
	static tQuaternion GetRootRot(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state);
	static void SetRootRot(const Eigen::MatrixXd& joint_mat, const tQuaternion& rot, Eigen::VectorXd& out_state);
	static tVector GetRootVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& vel);
	static void SetRootVel(const Eigen::MatrixXd& joint_mat, const tVector& vel, Eigen::VectorXd& out_vel);
	static tVector GetRootAngVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& vel);
	static void SetRootAngVel(const Eigen::MatrixXd& joint_mat, const tVector& ang_vel, Eigen::VectorXd& out_vel);

	static tVector CalcJointWorldPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tVector LocalToWorldPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int parent_id, const tVector& attach_pt);
	static tQuaternion CalcJointWorldRot(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static void CalcJointWorldTheta(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id, tVector& out_axis, double& out_theta);
	
	static tVector CalcJointWorldVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int joint_id);
	static tVector CalcWorldVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int parent_id, const tVector& attach_pt);
	static tVector CalcJointWorldAngularVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int joint_id);
	static tVector CalcWorldAngularVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int parent_id, const tVector& attach_pt);

	static int GetNumDof(const Eigen::MatrixXd& joint_mat);
	static void ApplyStep(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& step, Eigen::VectorXd& out_pose);

	static Eigen::VectorXi FindJointChain(const Eigen::MatrixXd& joint_mat, int joint_beg, int joint_end);
	static bool IsAncestor(const Eigen::MatrixXd& joint_mat, int child_joint, int ancestor_joint, int& out_len);
	static double CalcChainLength(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXi& chain);

	static void CalcAABB(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, tVector& out_min, tVector& out_max);

	static int GetParamOffset(const Eigen::MatrixXd& joint_mat, int joint_id);
	static int GetParamSize(const Eigen::MatrixXd& joint_mat, int joint_id);
	static int GetJointParamSize(eJointType joint_type);
	static void GetJointParams(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int j, Eigen::VectorXd& out_params);
	static void SetJointParams(const Eigen::MatrixXd& joint_mat, int j, const Eigen::VectorXd& params, Eigen::VectorXd& out_state);
	static eJointType GetJointType(const Eigen::MatrixXd& joint_mat, int joint_id);
	static int GetParent(const Eigen::MatrixXd& joint_mat, int joint_id);
	static bool HasParent(const Eigen::MatrixXd& joint_mat, int joint_id);
	static bool IsRoot(const Eigen::MatrixXd& joint_mat, int joint_id);
	static bool IsJointActuated(const Eigen::MatrixXd& joint_mat, int joint_id);
	static double GetTorqueLimit(const Eigen::MatrixXd& joint_mat, int joint_id);
	static double GetForceLimit(const Eigen::MatrixXd& joint_mat, int joint_id);
	static bool IsEndEffector(const Eigen::MatrixXd& joint_mat, int joint_id);
	
	static tVector GetJointLimLow(const Eigen::MatrixXd& joint_mat, int joint_id);
	static tVector GetJointLimHigh(const Eigen::MatrixXd& joint_mat, int joint_id);
	static double GetJointDiffWeight(const Eigen::MatrixXd& joint_mat, int joint_id);
	
	static double CalcLinkLength(const Eigen::MatrixXd& joint_mat, int joint_id);
	static tVector GetAttachPt(const Eigen::MatrixXd& joint_mat, int joint_id);
	static tVector GetAttachTheta(const Eigen::MatrixXd& joint_mat, int joint_id);

	// calculates the longest chain in the subtree of each joint
	static void CalcMaxSubChainLengths(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_lengths);
	static void CalcSubTreeMasses(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, Eigen::VectorXd& out_masses);
	
	static tMatrix BuildAttachTrans(const Eigen::MatrixXd& joint_mat, int joint_id);
	static tMatrix ChildParentTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix ParentChildTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix JointWorldTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix WorldJointTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);

#ifdef USE_JSON
	static bool Load(const Json::Value& root, Eigen::MatrixXd& out_joint_mat);
	static bool ParseBodyDef(const Json::Value& root, tBodyDef& out_def);
	static bool ParseDrawShapeDef(const Json::Value& root, tDrawShapeDef& out_def);
	static std::string BuildJointMatJson(const Eigen::MatrixXd& joint_mat);
	static std::string BuildJointJson(int id, const tJointDesc& joint_desc);
	static bool ParseJoint(const Json::Value& root, tJointDesc& out_joint_desc);
#endif

	static int GetNumJoints(const Eigen::MatrixXd& joint_mat);
	static int GetRoot(const Eigen::MatrixXd& joint_mat);
	static void FindChildren(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXi& out_children);

	static bool LoadBodyDefs(const std::string& char_file, Eigen::MatrixXd& out_body_defs);
	
	static bool LoadDrawShapeDefs(const std::string& char_file, Eigen::MatrixXd& out_draw_defs);
	
	static cShape::eShape GetBodyShape(const Eigen::MatrixXd& body_defs, int part_id);
	static tVector GetBodyAttachPt(const Eigen::MatrixXd& body_defs, int part_id);
	static tVector GetBodyAttachTheta(const Eigen::MatrixXd& body_defs, int part_id);
	static void GetBodyRotation(const Eigen::MatrixXd& body_defs, int part_id, tVector& out_axis, double& out_theta);
	static double GetBodyMass(const Eigen::MatrixXd& body_defs, int part_id);
	static int GetBodyColGroup(const Eigen::MatrixXd& body_defs, int part_id);
	static bool GetBodyEnableFallContact(const Eigen::MatrixXd& body_defs, int part_id);
	static void SetBodyEnableFallContact(int part_id, bool enable, Eigen::MatrixXd& out_body_defs);
	static tVector GetBodySize(const Eigen::MatrixXd& body_defs, int part_id);
	static tVector GetBodyColor(const Eigen::MatrixXd& body_defs, int part_id);
	static double CalcTotalMass(const Eigen::MatrixXd& body_defs);
	static bool IsValidBody(const Eigen::MatrixXd& body_defs, int part_id);
	static tVector GetBodyLocalCoM(const Eigen::MatrixXd& body_defs, int part_id);

	static int GetDrawShapeParentJoint(const tDrawShapeDef& shape);
	static tVector GetDrawShapeAttachPt(const tDrawShapeDef& shape);
	static tVector GetDrawShapeAttachTheta(const tDrawShapeDef& shape);
	static void GetDrawShapeRotation(const tDrawShapeDef& shape, tVector& out_axis, double& out_theta);
	static tVector GetDrawShapeColor(const tDrawShapeDef& shape);
	static int GetDrawShapeMeshID(const tDrawShapeDef& shape);

	static tVector CalcBodyPartPos(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, const Eigen::VectorXd& state, int part_id);
	static tVector CalcBodyPartVel(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int part_id);
	static void CalcBodyPartRotation(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, const Eigen::VectorXd& state, int part_id, tVector& out_axis, double& out_theta);
	static tMatrix BodyWorldTrans(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, const Eigen::VectorXd& state, int part_id);
	static tMatrix BodyJointTrans(const Eigen::MatrixXd& body_defs, int part_id);

	static tJointDesc BuildJointDesc(eJointType joint_type, int parent_id, const tVector& attach_pt);
	static tJointDesc BuildJointDesc();
	static tBodyDef BuildBodyDef();
	static tDrawShapeDef BuildDrawShapeDef();

	static void BuildDefaultPose(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose);
	static void BuildDefaultVel(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_vel);

	static void CalcPoseDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1, Eigen::VectorXd& out_diff);
	static tVector CalcRootPosDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1);
	static tQuaternion CalcRootRotDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1);
	static double CalcPoseErr(const Eigen::MatrixXd& joint_mat, int joint_id, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1);
	static double CalcRootPosErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1);
	static double CalcRootRotErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1);

	static void CalcVelDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& vel0, const Eigen::VectorXd& vel1, Eigen::VectorXd& out_diff);
	static tVector CalcRootVelDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1);
	static tVector CalcRootAngVelDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1);
	static double CalcVelErr(const Eigen::MatrixXd& joint_mat, int joint_id, const Eigen::VectorXd& vel0, const Eigen::VectorXd& vel1);
	static double CalcRootVelErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& vel0, const Eigen::VectorXd& vel1);
	static double CalcRootAngVelErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& vel0, const Eigen::VectorXd& vel1);

	static void CalcVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1, double dt, Eigen::VectorXd& out_vel);

	static void PostProcessPose(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose);
	static void LerpPoses(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1, double lerp, Eigen::VectorXd& out_pose);
	static void VelToPoseDiff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const Eigen::VectorXd& vel, Eigen::VectorXd& out_pose_diff);

	static double CalcHeading(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose);
	static tQuaternion CalcHeadingRot(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose);
	static tMatrix BuildHeadingTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose);
	static tMatrix BuildOriginTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose);
	static void NormalizePoseHeading(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose);
	static void NormalizePoseHeading(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose, Eigen::VectorXd& out_vel);

	

protected:
	static bool ParseJointType(const std::string& type_str, eJointType& out_joint_type);
	static void PostProcessJointMat(Eigen::MatrixXd& out_joint_mat);

	static tMatrix ChildParentTransRoot(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix ChildParentTransRevolute(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix ChildParentTransPlanar(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix ChildParentTransFixed(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix ChildParentTransPrismatic(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix ChildParentTransSpherical(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);

	static void BuildDefaultPoseRoot(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose);
	static void BuildDefaultPoseRevolute(Eigen::VectorXd& out_pose);
	static void BuildDefaultPosePrismatic(Eigen::VectorXd& out_pose);
	static void BuildDefaultPosePlanar(Eigen::VectorXd& out_pose);
	static void BuildDefaultPoseFixed(Eigen::VectorXd& out_pose);
	static void BuildDefaultPoseSpherical(Eigen::VectorXd& out_pose);

	static void BuildDefaultVelRoot(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_pose);
	static void BuildDefaultVelRevolute(Eigen::VectorXd& out_pose);
	static void BuildDefaultVelPrismatic(Eigen::VectorXd& out_pose);
	static void BuildDefaultVelPlanar(Eigen::VectorXd& out_pose);
	static void BuildDefaultVelFixed(Eigen::VectorXd& out_pose);
	static void BuildDefaultVelSpherical(Eigen::VectorXd& out_pose);

	static void CalcJointPoseDiff(const Eigen::MatrixXd& joint_mat, int joint_id, const Eigen::VectorXd& pose0, const Eigen::VectorXd& pose1, Eigen::VectorXd& out_diff);
	static void CalcJointVelDiff(const Eigen::MatrixXd& joint_mat, int joint_id, const Eigen::VectorXd& vel0, const Eigen::VectorXd& vel1, Eigen::VectorXd& out_diff);
};
