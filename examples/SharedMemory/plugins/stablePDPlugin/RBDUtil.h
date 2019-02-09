#pragma once

#include <vector>
#include <fstream>

#include "RBDModel.h"
#include "KinTree.h"

class cRBDUtil
{
public:
	static void SolveInvDyna(const cRBDModel& model, const Eigen::VectorXd& acc, Eigen::VectorXd& out_tau);

	static void SolveForDyna(const cRBDModel& model, const Eigen::VectorXd& tau, Eigen::VectorXd& out_acc);
	static void SolveForDyna(const cRBDModel& model, const Eigen::VectorXd& tau, const Eigen::VectorXd& total_force, Eigen::VectorXd& out_acc);

	static void BuildMassMat(const cRBDModel& model, Eigen::MatrixXd& out_mass_mat);
	static void BuildMassMat(const cRBDModel& model, Eigen::MatrixXd& inertia_buffer, Eigen::MatrixXd& out_mass_mat);
	static void BuildBiasForce(const cRBDModel& model, Eigen::VectorXd& out_bias_force);

	static void CalcGravityForce(const cRBDModel& model, Eigen::VectorXd& out_g_force);

	static void BuildEndEffectorJacobian(const cRBDModel& model, int joint_id, Eigen::MatrixXd& out_J);
	static void BuildEndEffectorJacobian(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int joint_id, Eigen::MatrixXd& out_J);
	static Eigen::MatrixXd MultJacobianEndEff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& vel, const Eigen::MatrixXd& J, int joint_id);

	static void BuildJacobian(const cRBDModel& model, Eigen::MatrixXd& out_J);
	static Eigen::MatrixXd ExtractEndEffJacobian(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& J, int joint_id);

	static void BuildCOMJacobian(const cRBDModel& model, Eigen::MatrixXd& out_J);
	static void BuildCOMJacobian(const cRBDModel& model, const Eigen::MatrixXd& J, Eigen::MatrixXd& out_J);
	static void BuildJacobianDot(const cRBDModel& model, Eigen::MatrixXd& out_J_dot);

	static cSpAlg::tSpVec BuildCOMVelProdAcc(const cRBDModel& model);
	static cSpAlg::tSpVec BuildCOMVelProdAccAux(const cRBDModel& model, const Eigen::MatrixXd& Jd);
	static cSpAlg::tSpVec CalcVelProdAcc(const cRBDModel& model, const Eigen::MatrixXd& Jd, int joint_id);

	static cSpAlg::tSpVec CalcJointWorldVel(const cRBDModel& model, int joint_id);
	static cSpAlg::tSpVec CalcJointWorldVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int joint_id);
	static cSpAlg::tSpVec CalcWorldVel(const cRBDModel& model, int joint_id);
	static cSpAlg::tSpVec CalcWorldVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int joint_id);


	static tVector CalcCoMPos(const cRBDModel& model);
	static tVector CalcCoMVel(const cRBDModel& model);
	static tVector CalcCoMVel(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, const Eigen::VectorXd& pose, const Eigen::VectorXd& vel);
	static void CalcCoM(const cRBDModel& model, tVector& out_com, tVector& out_vel);
	static void CalcCoM(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, const Eigen::VectorXd& pose, const Eigen::VectorXd& vel,
						tVector& out_com, tVector& out_vel);

	static cSpAlg::tSpTrans BuildParentChildTransform(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j);
	static cSpAlg::tSpTrans BuildChildParentTransform(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j);
	
	// extract a cSpAlg::tSpTrans from a matrix presentating a stack of transforms
	static void CalcWorldJointTransforms(const cRBDModel& model, Eigen::MatrixXd& out_trans_arr);
	static bool IsConstJointSubspace(const Eigen::MatrixXd& joint_mat, int j);
	static Eigen::MatrixXd BuildJointSubspace(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j);

	static cSpAlg::tSpMat BuildMomentInertia(const Eigen::MatrixXd& body_defs, int part_id);

protected:

	static Eigen::MatrixXd BuildJointSubspaceRoot(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose);
	static Eigen::MatrixXd BuildJointSubspaceRevolute(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j);
	static Eigen::MatrixXd BuildJointSubspacePrismatic(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j);
	static Eigen::MatrixXd BuildJointSubspacePlanar(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j);
	static Eigen::MatrixXd BuildJointSubspaceFixed(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j);
	static Eigen::MatrixXd BuildJointSubspaceSpherical(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j);

	static cSpAlg::tSpVec BuildCj(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot, int j);
	static cSpAlg::tSpVec BuildCjRoot(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot, int j);
	static cSpAlg::tSpVec BuildCjRevolute(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q_dot, int j);
	static cSpAlg::tSpVec BuildCjPrismatic(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q_dot, int j);
	static cSpAlg::tSpVec BuildCjPlanar(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q_dot, int j);
	static cSpAlg::tSpVec BuildCjFixed(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q_dot, int j);
	static cSpAlg::tSpVec BuildCjSpherical(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q_dot, int j);

	static cSpAlg::tSpMat BuildMomentInertiaBox(const Eigen::MatrixXd& body_defs, int part_id);
	static cSpAlg::tSpMat BuildMomentInertiaCapsule(const Eigen::MatrixXd& body_defs, int part_id);
	static cSpAlg::tSpMat BuildMomentInertiaSphere(const Eigen::MatrixXd& body_defs, int part_id);
	static cSpAlg::tSpMat BuildMomentInertiaCylinder(const Eigen::MatrixXd& body_defs, int part_id);

	// builds the spatial inertial matrix in the coordinate frame of the parent joint
	static cSpAlg::tSpMat BuildInertiaSpatialMat(const Eigen::MatrixXd& body_defs, int part_id);
};