#include "RBDUtil.h"
#include <iostream>

#define _USE_MATH_DEFINES
#include "math.h"
void cRBDUtil::SolveInvDyna(const cRBDModel& model, const Eigen::VectorXd& acc, Eigen::VectorXd& out_tau)
{
	const Eigen::MatrixXd& joint_mat = model.GetJointMat();
	const Eigen::MatrixXd& body_defs = model.GetBodyDefs();
	const tVector& gravity = model.GetGravity();
	const Eigen::VectorXd& pose = model.GetPose();
	const Eigen::VectorXd& vel = model.GetVel();

	assert(joint_mat.rows() == body_defs.rows());
	assert(pose.rows() == vel.rows());
	assert(pose.rows() == acc.rows());
	assert(cKinTree::GetNumDof(joint_mat) == pose.rows());

	cSpAlg::tSpVec vel0 = cSpAlg::tSpVec::Zero();
	cSpAlg::tSpVec acc0 = cSpAlg::BuildSV(tVector::Zero(), -gravity);

	int num_joints = cKinTree::GetNumJoints(joint_mat);
	Eigen::MatrixXd vels = Eigen::MatrixXd(num_joints, gSpVecSize);
	Eigen::MatrixXd accs = Eigen::MatrixXd(num_joints, gSpVecSize);
	Eigen::MatrixXd fs = Eigen::MatrixXd(num_joints, gSpVecSize);

	for (int j = 0; j < num_joints; ++j)
	{
		if (cKinTree::IsValidBody(body_defs, j))
		{
			cSpAlg::tSpTrans parent_child_trans = model.GetSpParentChildTrans(j);
			cSpAlg::tSpTrans world_child_trans = model.GetSpWorldJointTrans(j);

			const Eigen::Block<const Eigen::MatrixXd> S = model.GetJointSubspace(j);
			
			Eigen::VectorXd q;
			Eigen::VectorXd dq;
			Eigen::VectorXd ddq;
			cKinTree::GetJointParams(joint_mat, pose, j, q);
			cKinTree::GetJointParams(joint_mat, vel, j, dq);
			cKinTree::GetJointParams(joint_mat, acc, j, ddq);

			cSpAlg::tSpVec cj = BuildCj(joint_mat, q, dq, j);
			cSpAlg::tSpVec vj = cSpAlg::tSpVec::Zero();
			if (S.cols() > 0)
			{
				vj = S * dq;
			}

			cSpAlg::tSpMat I = BuildInertiaSpatialMat(body_defs, j);

			cSpAlg::tSpVec vel_p;
			cSpAlg::tSpVec acc_p;
			if (cKinTree::HasParent(joint_mat, j))
			{
				int parent_id = cKinTree::GetParent(joint_mat, j);
				vel_p = vels.row(parent_id);
				acc_p = accs.row(parent_id);
			}
			else
			{
				vel_p = vel0;
				acc_p = acc0;
			}

			cSpAlg::tSpVec Sddq = cSpAlg::tSpVec::Zero();
			if (S.cols() > 0)
			{
				Sddq = S * ddq;
			}

			cSpAlg::tSpVec curr_vel = cSpAlg::ApplyTransM(parent_child_trans, vel_p) + vj;
			cSpAlg::tSpVec curr_acc = cSpAlg::ApplyTransM(parent_child_trans, acc_p) + Sddq + cj + cSpAlg::CrossM(curr_vel, vj);
			cSpAlg::tSpVec curr_f = I * curr_acc + cSpAlg::CrossF(curr_vel, I * curr_vel);
			
			vels.row(j) = curr_vel;
			accs.row(j) = curr_acc;
			fs.row(j) = curr_f;
		}
	}

	out_tau = Eigen::VectorXd::Zero(pose.size());
	for (int j = num_joints - 1; j >= 0; --j)
	{
		if (cKinTree::IsValidBody(body_defs, j))
		{
			cSpAlg::tSpVec curr_f = fs.row(j);
			const Eigen::Block<const Eigen::MatrixXd> S = model.GetJointSubspace(j);
			Eigen::VectorXd curr_tau = S.transpose() * curr_f;

			cKinTree::SetJointParams(joint_mat, j, curr_tau, out_tau);
			if (cKinTree::HasParent(joint_mat, j))
			{
				int parent_id = cKinTree::GetParent(joint_mat, j);
				cSpAlg::tSpTrans child_parent_trans = model.GetSpChildParentTrans(j);
				fs.row(parent_id) += cSpAlg::ApplyTransF(child_parent_trans, curr_f);
			}
		}
	}
}

void cRBDUtil::SolveForDyna(const cRBDModel& model, const Eigen::VectorXd& tau, Eigen::VectorXd& out_acc)
{
	Eigen::VectorXd total_force = Eigen::VectorXd::Zero(model.GetNumDof());
	SolveForDyna(model, tau, total_force, out_acc);
}

void cRBDUtil::SolveForDyna(const cRBDModel& model, const Eigen::VectorXd& tau, const Eigen::VectorXd& total_force, Eigen::VectorXd& out_acc)
{
	Eigen::VectorXd C;
	Eigen::MatrixXd H;
	BuildBiasForce(model, C);
	BuildMassMat(model, H);
	out_acc = H.ldlt().solve(tau + total_force - C);
}


void cRBDUtil::BuildMassMat(const cRBDModel& model, Eigen::MatrixXd& out_mass_mat)
{
	const int svs = gSpVecSize;
	int num_joints = model.GetNumJoints();
	Eigen::MatrixXd Is = Eigen::MatrixXd::Zero(num_joints * svs, svs);
	BuildMassMat(model, Is, out_mass_mat);
}

void cRBDUtil::BuildMassMat(const cRBDModel& model, Eigen::MatrixXd& inertia_buffer, Eigen::MatrixXd& out_mass_mat)
{
	// use composite-rigid-body algorithm
	const Eigen::MatrixXd& joint_mat = model.GetJointMat();
	const Eigen::MatrixXd& body_defs = model.GetBodyDefs();
	const Eigen::VectorXd& pose = model.GetPose();
	Eigen::MatrixXd& H = out_mass_mat;

	int dim = model.GetNumDof();
	int num_joints = model.GetNumJoints();
	H.setZero(dim, dim);
	const int svs = gSpVecSize;

	Eigen::MatrixXd child_parent_mats_F = Eigen::MatrixXd(svs * num_joints, svs);
	Eigen::MatrixXd parent_child_mats_M = Eigen::MatrixXd(svs * num_joints, svs);
	Eigen::MatrixXd& Is = inertia_buffer;
	for (int j = 0; j < num_joints; ++j)
	{
		if (cKinTree::IsValidBody(body_defs, j))
		{
			Is.block(j * svs, 0, svs, svs) = BuildInertiaSpatialMat(body_defs, j);
		}

		cSpAlg::tSpTrans child_parent_trans = model.GetSpChildParentTrans(j);
		cSpAlg::tSpMat child_parent_mat = cSpAlg::BuildSpatialMatF(child_parent_trans);
		cSpAlg::tSpMat parent_child_mat = cSpAlg::BuildSpatialMatM(cSpAlg::InvTrans(child_parent_trans));
		child_parent_mats_F.block(j * svs, 0, svs, svs) = child_parent_mat;
		parent_child_mats_M.block(j * svs, 0, svs, svs) = parent_child_mat;
	}

	for (int j = num_joints - 1; j >= 0; --j)
	{
		if (cKinTree::IsValidBody(body_defs, j))
		{
			Eigen::Block< Eigen::MatrixXd, -1, -1, false> curr_I = Is.block(j * svs, 0, svs, svs);
			int parent_id = cKinTree::GetParent(joint_mat, j);
			if (cKinTree::HasParent(joint_mat, j))
			{
				cSpAlg::tSpTrans child_parent_trans = model.GetSpChildParentTrans(j);
				cSpAlg::tSpMat child_parent_mat = child_parent_mats_F.block(j * svs, 0, svs, svs);
				cSpAlg::tSpMat parent_child_mat = parent_child_mats_M.block(j * svs, 0, svs, svs);
				Is.block(parent_id * svs, 0, svs, svs) += child_parent_mat * curr_I * parent_child_mat;
			}

			const Eigen::Block<const Eigen::MatrixXd> S = model.GetJointSubspace(j);
			int dim = cKinTree::GetParamSize(joint_mat, j);
			if (dim > 0)
			{
				int offset = cKinTree::GetParamOffset(joint_mat, j);
				Eigen::MatrixXd F = curr_I * S;
				H.block(offset, offset, dim, dim) = S.transpose() * F;

				int curr_id = j;
				while (cKinTree::HasParent(joint_mat, curr_id))
				{
					cSpAlg::tSpMat child_parent_mat = child_parent_mats_F.block(curr_id * svs, 0, svs, svs);
					F = child_parent_mat * F;

					curr_id = cKinTree::GetParent(joint_mat, curr_id);
					int curr_offset = cKinTree::GetParamOffset(joint_mat, curr_id);
					int curr_dim = cKinTree::GetParamSize(joint_mat, curr_id);

					if (curr_dim > 0)
					{
						const Eigen::Block<const Eigen::MatrixXd> curr_S = model.GetJointSubspace(curr_id);
						H.block(offset, curr_offset, dim, curr_dim) = F.transpose() * curr_S;
						H.block(curr_offset, offset, curr_dim, dim) = H.block(offset, curr_offset, dim, curr_dim).transpose();
					}
				}
			}
		}
	}
}

void cRBDUtil::BuildEndEffectorJacobian(const cRBDModel& model, int joint_id, Eigen::MatrixXd& out_J)
{
	// jacobian in world coordinates
	const Eigen::MatrixXd& joint_mat = model.GetJointMat();
	const Eigen::VectorXd& pose = model.GetPose();
	
	int num_dofs = cKinTree::GetNumDof(joint_mat);
	out_J = Eigen::MatrixXd::Zero(gSpVecSize, num_dofs);

	int curr_id = joint_id;
	cSpAlg::tSpTrans curr_trans = cSpAlg::BuildTrans();
	while (curr_id != cKinTree::gInvalidJointID)
	{
		int offset = cKinTree::GetParamOffset(joint_mat, curr_id);
		int size = cKinTree::GetParamSize(joint_mat, curr_id);
		const Eigen::MatrixXd S = model.GetJointSubspace(curr_id);

		out_J.block(0, offset, gSpVecSize, size) = cSpAlg::ApplyTransM(curr_trans, S);

		int parent_id = cKinTree::GetParent(joint_mat, curr_id);
		cSpAlg::tSpTrans parent_child_trans = model.GetSpParentChildTrans(curr_id);
		curr_trans = cSpAlg::CompTrans(curr_trans, parent_child_trans);
		curr_id = parent_id;
	}

	out_J = cSpAlg::ApplyInvTransM(curr_trans, out_J);
}

void cRBDUtil::BuildEndEffectorJacobian(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int joint_id, Eigen::MatrixXd& out_J)
{
	// jacobian in world coordinates
	int num_dofs = cKinTree::GetNumDof(joint_mat);
	out_J = Eigen::MatrixXd::Zero(gSpVecSize, num_dofs);

	int curr_id = joint_id;
	cSpAlg::tSpTrans curr_trans = cSpAlg::BuildTrans();
	while (curr_id != cKinTree::gInvalidJointID)
	{
		int offset = cKinTree::GetParamOffset(joint_mat, curr_id);
		int size = cKinTree::GetParamSize(joint_mat, curr_id);
		Eigen::MatrixXd S = BuildJointSubspace(joint_mat, pose, curr_id);

		S = cSpAlg::ApplyTransM(curr_trans, S);
		out_J.block(0, offset, gSpVecSize, size) = S;

		int parent_id = cKinTree::GetParent(joint_mat, curr_id);
		cSpAlg::tSpTrans parent_child_trans = BuildParentChildTransform(joint_mat, pose, curr_id);
		curr_trans = cSpAlg::CompTrans(curr_trans, parent_child_trans);
		curr_id = parent_id;
	}

	out_J = cSpAlg::ApplyInvTransM(curr_trans, out_J);
}

Eigen::MatrixXd cRBDUtil::MultJacobianEndEff(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q, const Eigen::MatrixXd& J, int joint_id)
{
	// multiplies q by the jacobian of the end effector (joint_id)
	int curr_id = joint_id;
	cSpAlg::tSpVec joint_vel = cSpAlg::tSpVec::Zero();
	while (curr_id != cKinTree::gInvalidJointID)
	{
		int offset = cKinTree::GetParamOffset(joint_mat, curr_id);
		int size = cKinTree::GetParamSize(joint_mat, curr_id);
		Eigen::VectorXd curr_q;
		cKinTree::GetJointParams(joint_mat, q, curr_id, curr_q);
		const Eigen::Block<const Eigen::MatrixXd> curr_J = J.block(0, offset, gSpVecSize, size);
		
		joint_vel += curr_J * curr_q;
		curr_id = cKinTree::GetParent(joint_mat, curr_id);
	}
	return joint_vel;
}

void cRBDUtil::BuildJacobian(const cRBDModel& model, Eigen::MatrixXd& out_J)
{
	const Eigen::MatrixXd& joint_mat = model.GetJointMat();
	const Eigen::VectorXd& pose = model.GetPose();

	int num_dofs = model.GetNumDof();
	out_J = Eigen::MatrixXd::Zero(gSpVecSize, num_dofs);

	int num_joints = cKinTree::GetNumJoints(joint_mat);
	for (int j = 0; j < num_joints; ++j)
	{
		cSpAlg::tSpTrans world_joint_trans = model.GetSpWorldJointTrans(j);

		int offset = cKinTree::GetParamOffset(joint_mat, j);
		int size = cKinTree::GetParamSize(joint_mat, j);
		const Eigen::MatrixXd S = model.GetJointSubspace(j);

		out_J.block(0, offset, gSpVecSize, size) = cSpAlg::ApplyInvTransM(world_joint_trans, S);
	}
}

Eigen::MatrixXd cRBDUtil::ExtractEndEffJacobian(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& J, int joint_id)
{
	int curr_id = joint_id;
	Eigen::MatrixXd J_end_eff = Eigen::MatrixXd::Zero(J.rows(), J.cols());

	while (curr_id != cKinTree::gInvalidJointID)
	{
		int offset = cKinTree::GetParamOffset(joint_mat, curr_id);
		int size = cKinTree::GetParamSize(joint_mat, curr_id);
		const Eigen::Block<const Eigen::MatrixXd> curr_J = J.block(0, offset, gSpVecSize, size);

		J_end_eff.block(0, offset, gSpVecSize, size) = curr_J;
		curr_id = cKinTree::GetParent(joint_mat, curr_id);
	}
	return J_end_eff;
}


void cRBDUtil::BuildCOMJacobian(const cRBDModel& model, Eigen::MatrixXd& out_J)
{
	// coord frame for jacobian has origin at the com
	const Eigen::MatrixXd& joint_mat = model.GetJointMat();
	const Eigen::MatrixXd& body_defs = model.GetBodyDefs();
	const Eigen::VectorXd& pose = model.GetPose();

	Eigen::MatrixXd J;
	BuildJacobian(model, J);
	BuildCOMJacobian(model, J, out_J);
}

void cRBDUtil::BuildCOMJacobian(const cRBDModel& model, const Eigen::MatrixXd& J, Eigen::MatrixXd& out_J)
{
	const Eigen::MatrixXd& joint_mat = model.GetJointMat();
	const Eigen::MatrixXd& body_defs = model.GetBodyDefs();

	int num_dofs = cKinTree::GetNumDof(joint_mat);
	out_J = Eigen::MatrixXd::Zero(gSpVecSize, num_dofs);
	double total_mass = cKinTree::CalcTotalMass(body_defs);

	int num_joints = cKinTree::GetNumJoints(joint_mat);
	for (int j = num_joints - 1; j >= 0; --j)
	{
		if (cKinTree::IsValidBody(body_defs, j))
		{
			double mass = cKinTree::GetBodyMass(body_defs, j);
			double mass_frac = mass / total_mass;

			cSpAlg::tSpTrans world_child_trans = model.GetSpWorldJointTrans(j);
			tMatrix body_joint_mat = cKinTree::BodyJointTrans(body_defs, j);
			cSpAlg::tSpTrans body_world_trans = cSpAlg::CompTrans(cSpAlg::InvTrans(world_child_trans), cSpAlg::MatToTrans(body_joint_mat));
			tMatrix body_world_mat = cSpAlg::TransToMat(body_world_trans);

			tVector body_pos = body_world_mat.col(3);
			body_pos[3] = 0;
			cSpAlg::tSpTrans body_pos_trans = cSpAlg::BuildTrans(body_pos);

			int curr_id = j;
			while (curr_id != cKinTree::gInvalidJointID)
			{
				int offset = cKinTree::GetParamOffset(joint_mat, curr_id);
				int size = cKinTree::GetParamSize(joint_mat, curr_id);
				
				const Eigen::MatrixXd& J_block = J.block(0, offset, gSpVecSize, size);
				Eigen::Block<Eigen::MatrixXd, -1, -1, false> J_com_block = out_J.block(0, offset, gSpVecSize, size);
				J_com_block += mass_frac * cSpAlg::ApplyTransM(body_pos_trans, J_block);

				curr_id = cKinTree::GetParent(joint_mat, curr_id);
			}
		}
	}
}


void cRBDUtil::BuildJacobianDot(const cRBDModel& model, Eigen::MatrixXd& out_J_dot)
{
	// for comput the velocity product acceleration
	const Eigen::MatrixXd& joint_mat = model.GetJointMat();
	const Eigen::VectorXd& pose = model.GetPose();
	const Eigen::VectorXd& vel = model.GetVel();

	int num_dofs = cKinTree::GetNumDof(joint_mat);
	int num_joints = cKinTree::GetNumJoints(joint_mat);
	out_J_dot = Eigen::MatrixXd(gSpVecSize, num_dofs);
	Eigen::MatrixXd Sqs(gSpVecSize, num_joints);

	for (int j = 0; j < num_joints; ++j)
	{
		cSpAlg::tSpTrans world_child_trans = model.GetSpWorldJointTrans(j);
		Eigen::MatrixXd S = model.GetJointSubspace(j);
		S = cSpAlg::ApplyInvTransM(world_child_trans, S);
		Eigen::VectorXd dq;
		cKinTree::GetJointParams(joint_mat, vel, j, dq);
		cSpAlg::tSpVec Sq = S * dq;

		cSpAlg::tSpVec parent_Sq = cSpAlg::tSpVec::Zero();
		int parent_id = cKinTree::GetParent(joint_mat, j);
		if (parent_id != cKinTree::gInvalidJointID)
		{
			parent_Sq = Sqs.col(parent_id);
		}
		Sqs.col(j) = parent_Sq + Sq;

		int offset = cKinTree::GetParamOffset(joint_mat, j);
		int size = cKinTree::GetParamSize(joint_mat, j);
		out_J_dot.block(0, offset, gSpVecSize, size) = cSpAlg::CrossMs(parent_Sq, S);
	}
}

cSpAlg::tSpVec cRBDUtil::BuildCOMVelProdAcc(const cRBDModel& model)
{
	Eigen::MatrixXd Jd;
	BuildJacobianDot(model, Jd);
	return BuildCOMVelProdAccAux(model, Jd);
}

cSpAlg::tSpVec cRBDUtil::BuildCOMVelProdAccAux(const cRBDModel& model, const Eigen::MatrixXd& Jd)
{
	const Eigen::MatrixXd& joint_mat = model.GetJointMat();
	const Eigen::MatrixXd& body_defs = model.GetBodyDefs();
	const Eigen::VectorXd& pose = model.GetPose();
	const Eigen::VectorXd& vel = model.GetVel();
	const tVector& gravity = model.GetGravity();

	// coord frame origin at com
	int num_dofs = cKinTree::GetNumDof(joint_mat);
	int num_joints = cKinTree::GetNumJoints(joint_mat);
	
	double total_mass = cKinTree::CalcTotalMass(body_defs);
	cSpAlg::tSpVec com_vpa = cSpAlg::tSpVec::Zero();
	for (int j = 0; j < num_joints; ++j)
	{
		if (cKinTree::IsValidBody(body_defs, j))
		{
			double mass = cKinTree::GetBodyMass(body_defs, j);
			double mass_frac = mass / total_mass;

			cSpAlg::tSpVec vpa = CalcVelProdAcc(model, Jd, j);
			com_vpa += mass_frac * vpa;
		}
	}

	tVector com = CalcCoMPos(model);
	cSpAlg::tSpTrans trans = cSpAlg::BuildTrans(com);
	com_vpa = cSpAlg::ApplyTransM(trans, com_vpa);

	return com_vpa;
}

cSpAlg::tSpVec cRBDUtil::CalcVelProdAcc(const cRBDModel& model, const Eigen::MatrixXd& Jd, int joint_id)
{
	const Eigen::MatrixXd& joint_mat = model.GetJointMat();
	const Eigen::VectorXd& pose = model.GetPose();
	const Eigen::VectorXd& vel = model.GetVel();
	const tVector& gravity = model.GetGravity();

	int curr_id = joint_id;
	cSpAlg::tSpVec acc = cSpAlg::BuildSV(tVector::Zero(), -gravity);

	while (curr_id != cKinTree::gInvalidJointID)
	{
		int offset = cKinTree::GetParamOffset(joint_mat, curr_id);
		int size = cKinTree::GetParamSize(joint_mat, curr_id);
		Eigen::VectorXd q;
		Eigen::VectorXd dq;
		cKinTree::GetJointParams(joint_mat, pose, curr_id, q);
		cKinTree::GetJointParams(joint_mat, vel, curr_id, dq);
		const Eigen::Block<const Eigen::MatrixXd> curr_Jd = Jd.block(0, offset, gSpVecSize, size);

		cSpAlg::tSpVec cj = BuildCj(joint_mat, q, dq, curr_id);
		if (cj.squaredNorm() > 0)
		{
			cSpAlg::tSpTrans world_joint_trans = model.GetSpWorldJointTrans(curr_id);
			cj = cSpAlg::ApplyInvTransM(world_joint_trans, cj);
		}

		acc += curr_Jd * dq + cj;
		curr_id = cKinTree::GetParent(joint_mat, curr_id);
	}

	return acc;
}

cSpAlg::tSpVec cRBDUtil::CalcJointWorldVel(const cRBDModel& model, int joint_id)
{
	cSpAlg::tSpVec joint_vel = CalcJointWorldVel(model.GetJointMat(), model.GetPose(), model.GetVel(), joint_id);
	return joint_vel;
}

cSpAlg::tSpVec cRBDUtil::CalcJointWorldVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int joint_id)
{
	cSpAlg::tSpVec joint_vel = CalcWorldVel(joint_mat, state, vel, joint_id);
	return joint_vel;
}

cSpAlg::tSpVec cRBDUtil::CalcWorldVel(const cRBDModel& model, int joint_id)
{
	return CalcWorldVel(model.GetJointMat(), model.GetPose(), model.GetVel(), joint_id);
}

cSpAlg::tSpVec cRBDUtil::CalcWorldVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const Eigen::VectorXd& vel, int joint_id)
{
	Eigen::MatrixXd J;
	BuildEndEffectorJacobian(joint_mat, pose, joint_id, J);
	cSpAlg::tSpVec sv = J * vel;
	return sv;
}

tVector cRBDUtil::CalcCoMPos(const cRBDModel& model)
{
	tVector com;
	tVector com_vel;
	Eigen::VectorXd vel = Eigen::VectorXd::Zero(model.GetNumDof());
	CalcCoM(model, com, com_vel);
	return com;
}

tVector cRBDUtil::CalcCoMVel(const cRBDModel& model)
{
	tVector com;
	tVector com_vel;
	CalcCoM(model, com, com_vel);
	return com_vel;
}

tVector cRBDUtil::CalcCoMVel(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, const Eigen::VectorXd& pose, const Eigen::VectorXd& vel)
{
	tVector com;
	tVector com_vel;
	CalcCoM(joint_mat, body_defs, pose, vel, com, com_vel);
	return com_vel;
}

void cRBDUtil::CalcCoM(const cRBDModel& model, tVector& out_com, tVector& out_vel)
{
	const Eigen::MatrixXd& joint_mat = model.GetJointMat();
	const Eigen::MatrixXd& body_defs = model.GetBodyDefs();
	const Eigen::VectorXd& pose = model.GetPose();
	const Eigen::VectorXd& vel = model.GetVel();

	int num_joints = cKinTree::GetNumJoints(joint_mat);
	out_com.setZero();
	out_vel.setZero();
	double total_mass = 0;

	for (int j = 0; j < num_joints; ++j)
	{
		if (cKinTree::IsValidBody(body_defs, j))
		{
			tVector local_com = cKinTree::GetBodyLocalCoM(body_defs, j);
			cSpAlg::tSpTrans world_joint_trans = model.GetSpWorldJointTrans(j);

			tMatrix joint_world_mat = cSpAlg::TransToMat(cSpAlg::InvTrans(world_joint_trans));
			tMatrix body_joint_mat = cKinTree::BodyJointTrans(body_defs, j);
			tMatrix body_world_mat = joint_world_mat * body_joint_mat;

			tVector attach_pt = cKinTree::GetBodyAttachPt(body_defs, j);
			attach_pt[3] = 1;
			attach_pt = body_joint_mat * attach_pt;

			local_com[3] = 1;
			tVector world_com = body_world_mat * local_com;
			world_com[3] = 0;

			cSpAlg::tSpTrans com_trans = cSpAlg::BuildTrans(world_com);
			cSpAlg::tSpVec sv = CalcWorldVel(model, j);
			sv = cSpAlg::ApplyTransM(com_trans, sv);

			tVector com_vel = cSpAlg::GetV(sv);

			double m = cKinTree::GetBodyMass(body_defs, j);
			out_com += m * world_com;
			out_vel += m * com_vel;
			total_mass += m;
		}
	}

	assert(total_mass > 0);
	out_com /= total_mass;
	out_vel /= total_mass;
}

void cRBDUtil::CalcCoM(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, const Eigen::VectorXd& pose, const Eigen::VectorXd& vel,
						tVector& out_com, tVector& out_vel)
{
	int num_joints = cKinTree::GetNumJoints(joint_mat);
	out_com.setZero();
	out_vel.setZero();
	double total_mass = 0;

	for (int j = 0; j < num_joints; ++j)
	{
		if (cKinTree::IsValidBody(body_defs, j))
		{
			tVector local_com = cKinTree::GetBodyLocalCoM(body_defs, j);
			tMatrix joint_world_mat = cKinTree::JointWorldTrans(joint_mat, pose, j);
			tMatrix body_joint_mat = cKinTree::BodyJointTrans(body_defs, j);
			tMatrix body_world_mat = joint_world_mat * body_joint_mat;

			tVector attach_pt = cKinTree::GetBodyAttachPt(body_defs, j);
			attach_pt[3] = 1;
			attach_pt = body_joint_mat * attach_pt;

			local_com[3] = 1;
			tVector world_com = body_world_mat * local_com;
			world_com[3] = 0;

			cSpAlg::tSpTrans com_trans = cSpAlg::BuildTrans(world_com);
			cSpAlg::tSpVec sv = CalcWorldVel(joint_mat, pose, vel, j);
			sv = cSpAlg::ApplyTransM(com_trans, sv);

			tVector com_vel = cSpAlg::GetV(sv);

			double m = cKinTree::GetBodyMass(body_defs, j);
			out_com += m * world_com;
			out_vel += m * com_vel;
			total_mass += m;
		}
	}

	assert(total_mass > 0);
	out_com /= total_mass;
	out_vel /= total_mass;
}

cSpAlg::tSpMat cRBDUtil::BuildMomentInertia(const Eigen::MatrixXd& body_defs, int part_id)
{
	// inertia tensor of shape centered at the com
	assert(cKinTree::IsValidBody(body_defs, part_id));
	cShape::eShape shape = cKinTree::GetBodyShape(body_defs, part_id);

	cSpAlg::tSpMat I;
	switch (shape)
	{
	case cShape::eShapeBox:
		I = BuildMomentInertiaBox(body_defs, part_id);
		break;
	case cShape::eShapeCapsule:
		I = BuildMomentInertiaCapsule(body_defs, part_id);
		break;
	case cShape::eShapeSphere:
		I = BuildMomentInertiaSphere(body_defs, part_id);
		break;
	case cShape::eShapeCylinder:
		I = BuildMomentInertiaCylinder(body_defs, part_id);
		break;
	default:
		assert(false); // unsupported shape
		break;
	}

	return I;
}

cSpAlg::tSpMat cRBDUtil::BuildMomentInertiaBox(const Eigen::MatrixXd& body_defs, int part_id)
{
	const cKinTree::tBodyDef& def = body_defs.row(part_id);
	double mass = cKinTree::GetBodyMass(body_defs, part_id);
	double sx = def(cKinTree::eBodyParam0);
	double sy = def(cKinTree::eBodyParam1);
	double sz = def(cKinTree::eBodyParam2);

	double x = mass / 12.0 * (sy * sy + sz * sz);
	double y = mass / 12.0 * (sx * sx + sz * sz);
	double z = mass / 12.0 * (sx * sx + sy * sy);

	cSpAlg::tSpMat I = cSpAlg::tSpMat::Zero();
	I(0, 0) = x;
	I(1, 1) = y;
	I(2, 2) = z;
	I(3, 3) = mass;
	I(4, 4) = mass;
	I(5, 5) = mass;

	return I;
}

cSpAlg::tSpMat cRBDUtil::BuildMomentInertiaCapsule(const Eigen::MatrixXd& body_defs, int part_id)
{
	const cKinTree::tBodyDef& def = body_defs.row(part_id);
	double mass = cKinTree::GetBodyMass(body_defs, part_id);
	double r = 0.5 * def(cKinTree::eBodyParam0);
	double h = def(cKinTree::eBodyParam1);

	double c_vol = M_PI * r * r * h;
	double hs_vol = M_PI * 2.0 / 3.0 * r * r * r;
	double density = mass / (c_vol + 2 * hs_vol);
	double cm = c_vol * density;
	double hsm = hs_vol * density;

	double x = cm*(0.25 * r * r + (1.0 / 12.0) * h * h) +
				2 * hsm *(0.4 * r * r + (3.0 / 8) * r * h + 0.5 * h * h);
	double y = (0.5 * cm + 0.8 * hsm) * r * r;
	double z = x;

	cSpAlg::tSpMat I = cSpAlg::tSpMat::Zero();
	I(0, 0) = x;
	I(1, 1) = y;
	I(2, 2) = z;
	I(3, 3) = mass;
	I(4, 4) = mass;
	I(5, 5) = mass;

	return I;
}

cSpAlg::tSpMat cRBDUtil::BuildMomentInertiaSphere(const Eigen::MatrixXd& body_defs, int part_id)
{
	const cKinTree::tBodyDef& def = body_defs.row(part_id);
	double mass = cKinTree::GetBodyMass(body_defs, part_id);
	double r = 0.5 * def(cKinTree::eBodyParam0);

	double x = 0.4 * mass * r * r;
	double y = x;
	double z = x;

	cSpAlg::tSpMat I = cSpAlg::tSpMat::Zero();
	I(0, 0) = x;
	I(1, 1) = y;
	I(2, 2) = z;
	I(3, 3) = mass;
	I(4, 4) = mass;
	I(5, 5) = mass;

	return I;
}

cSpAlg::tSpMat cRBDUtil::BuildMomentInertiaCylinder(const Eigen::MatrixXd& body_defs, int part_id)
{
	const cKinTree::tBodyDef& def = body_defs.row(part_id);
	double mass = cKinTree::GetBodyMass(body_defs, part_id);
	double r = 0.5 * def(cKinTree::eBodyParam0);
	double h = def(cKinTree::eBodyParam1);

	double c_vol = M_PI * r * r * h;
	double hs_vol = M_PI * 2.0 / 3.0 * r * r * r;

	double x = mass / 12 * (3 * r * r + h * h);
	double y = mass * r * r / 2;
	double z = x;

	cSpAlg::tSpMat I = cSpAlg::tSpMat::Zero();
	I(0, 0) = x;
	I(1, 1) = y;
	I(2, 2) = z;
	I(3, 3) = mass;
	I(4, 4) = mass;
	I(5, 5) = mass;

	return I;
}

cSpAlg::tSpMat cRBDUtil::BuildInertiaSpatialMat(const Eigen::MatrixXd& body_defs, int part_id)
{
	cSpAlg::tSpMat Ic = BuildMomentInertia(body_defs, part_id);
	tMatrix body_joint = cKinTree::BodyJointTrans(body_defs, part_id);
	cSpAlg::tSpTrans X = cSpAlg::MatToTrans(body_joint);
	cSpAlg::tSpMat Io = cSpAlg::BuildSpatialMatF(X) * Ic * cSpAlg::BuildSpatialMatM(cSpAlg::InvTrans(X));
	return Io;
}

void cRBDUtil::CalcWorldJointTransforms(const cRBDModel& model, Eigen::MatrixXd& out_trans_arr)
{
	const Eigen::MatrixXd& joint_mat = model.GetJointMat();
	const Eigen::VectorXd& pose = model.GetPose();

	int num_joints = cKinTree::GetNumJoints(joint_mat);
	out_trans_arr.resize(num_joints * gSVTransRows, gSVTransCols);

	for (int j = 0; j < num_joints; ++j)
	{
		int row_idx = j * gSVTransRows;
		int parent_id = cKinTree::GetParent(joint_mat, j);

		cSpAlg::tSpTrans parent_child_trans = model.GetSpParentChildTrans(j);
		cSpAlg::tSpTrans world_parent_trans = cSpAlg::BuildTrans();

		if (parent_id != cKinTree::gInvalidJointID)
		{
			world_parent_trans = cSpAlg::GetTrans(out_trans_arr, parent_id);
		}

		cSpAlg::tSpTrans world_child_trans = cSpAlg::CompTrans(parent_child_trans, world_parent_trans);
		out_trans_arr.block(row_idx, 0, gSVTransRows, gSVTransCols) = world_child_trans;
	}
}

cSpAlg::tSpTrans cRBDUtil::BuildParentChildTransform(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j)
{
	cSpAlg::tSpTrans trans = BuildChildParentTransform(joint_mat, pose, j);
	trans = cSpAlg::InvTrans(trans);
	return trans;
}

cSpAlg::tSpTrans cRBDUtil::BuildChildParentTransform(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j)
{
	tMatrix m = cKinTree::ChildParentTrans(joint_mat, pose, j);
	cSpAlg::tSpTrans trans = cSpAlg::MatToTrans(m);
	return trans;
}

bool cRBDUtil::IsConstJointSubspace(const Eigen::MatrixXd& joint_mat, int j)
{
	bool is_root = cKinTree::IsRoot(joint_mat, j);
	bool is_const = !is_root;
	return is_const;
}

Eigen::MatrixXd cRBDUtil::BuildJointSubspace(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j)
{
	cKinTree::eJointType j_type = cKinTree::GetJointType(joint_mat, j);
	bool is_root = cKinTree::IsRoot(joint_mat, j);
	Eigen::MatrixXd S;

	if (is_root)
	{
		S = BuildJointSubspaceRoot(joint_mat, pose);
	}
	else
	{
		switch (j_type)
		{
		case cKinTree::eJointTypeRevolute:
			S = BuildJointSubspaceRevolute(joint_mat, pose, j);
			break;
		case cKinTree::eJointTypePrismatic:
			S = BuildJointSubspacePrismatic(joint_mat, pose, j);
			break;
		case cKinTree::eJointTypePlanar:
			S = BuildJointSubspacePlanar(joint_mat, pose, j);
			break;
		case cKinTree::eJointTypeFixed:
			S = BuildJointSubspaceFixed(joint_mat, pose, j);
			break;
		case cKinTree::eJointTypeSpherical:
			S = BuildJointSubspaceSpherical(joint_mat, pose, j);
			break;
		default:
			assert(false); // unsupported joint type;
			break;
		}
	}
	return S;
}

Eigen::MatrixXd cRBDUtil::BuildJointSubspaceRoot(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose)
{
	int dim = cKinTree::gRootDim;
	int pos_dim = cKinTree::gPosDim;
	int rot_dim = cKinTree::gRotDim;

	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(gSpVecSize, dim);
	tQuaternion quat = cKinTree::GetRootRot(joint_mat, pose);
	tMatrix E = cMathUtil::RotateMat(quat);

	S.block(3, 0, 3, pos_dim) = E.block(0, 0, 3, pos_dim).transpose();
	//S.block(0, pos_dim, 3, rot_dim).setIdentity();
	S.block(0, pos_dim, 3, rot_dim - 1) = E.block(0, 0, 3, rot_dim - 1).transpose();
	return S;
}

Eigen::MatrixXd cRBDUtil::BuildJointSubspaceRevolute(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j)
{
	int dim = cKinTree::GetJointParamSize(cKinTree::eJointTypeRevolute);
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(gSpVecSize, dim);
	S(2, 0) = 1;
	return S;
}

Eigen::MatrixXd cRBDUtil::BuildJointSubspacePrismatic(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j)
{
	int dim = cKinTree::GetJointParamSize(cKinTree::eJointTypePrismatic);
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(gSpVecSize, dim);

	S(3, 0) = 1;
	return S;
}

Eigen::MatrixXd cRBDUtil::BuildJointSubspacePlanar(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j)
{
	int dim = cKinTree::GetJointParamSize(cKinTree::eJointTypePlanar);
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(gSpVecSize, dim);
	S(3, 0) = 1;
	S(4, 1) = 1;
	S(2, 2) = 1;
	return S;
}

Eigen::MatrixXd cRBDUtil::BuildJointSubspaceFixed(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j)
{
	int dim = cKinTree::GetJointParamSize(cKinTree::eJointTypeFixed);
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(gSpVecSize, dim);
	return S;
}

Eigen::MatrixXd cRBDUtil::BuildJointSubspaceSpherical(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, int j)
{
	int dim = cKinTree::GetJointParamSize(cKinTree::eJointTypeSpherical);
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(gSpVecSize, dim);
	S(0, 0) = 1;
	S(1, 1) = 1;
	S(2, 2) = 1;
	return S;
}

cSpAlg::tSpVec cRBDUtil::BuildCj(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot, int j)
{
	cKinTree::eJointType j_type = cKinTree::GetJointType(joint_mat, j);
	bool is_root = cKinTree::IsRoot(joint_mat, j);
	cSpAlg::tSpVec cj;

	if (is_root)
	{
		cj = BuildCjRoot(joint_mat, q, q_dot, j);
	}
	else
	{
		switch (j_type)
		{
		case cKinTree::eJointTypeRevolute:
			cj = BuildCjRevolute(joint_mat, q_dot, j);
			break;
		case cKinTree::eJointTypePrismatic:
			cj = BuildCjPrismatic(joint_mat, q_dot, j);
			break;
		case cKinTree::eJointTypePlanar:
			cj = BuildCjPlanar(joint_mat, q_dot, j);
			break;
		case cKinTree::eJointTypeFixed:
			cj = BuildCjFixed(joint_mat, q_dot, j);
			break;
		case cKinTree::eJointTypeSpherical:
			cj = BuildCjSpherical(joint_mat, q_dot, j);
			break;
		default:
			assert(false); // unsupported joint type;
			break;
		}
	}
	return cj;
}

cSpAlg::tSpVec cRBDUtil::BuildCjRoot(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot, int j)
{
	tQuaternion quat = cKinTree::GetRootRot(joint_mat, q);
	tVector vel_lin = cKinTree::GetRootVel(joint_mat, q_dot);
	tVector vel_ang = cKinTree::GetRootAngVel(joint_mat, q_dot);
	vel_ang[3] = 0;

	Eigen::VectorXd joint_params;
	cKinTree::GetJointParams(joint_mat, q_dot, j, joint_params);
	
	tMatrix vel_dquat_mat = cMathUtil::BuildQuaternionDiffMat(quat);
	tVector dq_vec = vel_dquat_mat * vel_ang;
	tQuaternion dquat = cMathUtil::VecToQuat(dq_vec);
	
	tMatrix mat = tMatrix::Identity();
	mat(0, 0) = 4 * (quat.w() * dquat.w() + quat.x() * dquat.x());
	mat(1, 1) = 4 * (quat.w() * dquat.w() + quat.y() * dquat.y());
	mat(2, 2) = 4 * (quat.w() * dquat.w() + quat.z() * dquat.z());

	mat(1, 0) = 2 * (dquat.x() * quat.y() + quat.x() * dquat.y()
				- dquat.w() * quat.z() - quat.w() * dquat.z());
	mat(0, 1) = 2 * (dquat.x() * quat.y() + quat.x() * dquat.y()
				+ dquat.w() * quat.z() + quat.w() * dquat.z());

	mat(2, 0) = 2 * (dquat.x() * quat.z() + quat.x() * dquat.z()
				+ dquat.w() * quat.y() + quat.w() * dquat.y());
	mat(0, 2) = 2 * (dquat.x() * quat.z() + quat.x() * dquat.z()
				- dquat.w() * quat.y() - quat.w() * dquat.y());

	mat(2, 1) = 2 * (dquat.y() * quat.z() + quat.y() * dquat.z()
				- dquat.w() * quat.x() - quat.w() * dquat.x());
	mat(1, 2) = 2 * (dquat.y() * quat.z() + quat.y() * dquat.z()
				+ dquat.w() * quat.x() + quat.w() * dquat.x());

	cSpAlg::tSpVec cj = cSpAlg::tSpVec::Zero();
	cj.segment(3, 3) = (mat * vel_lin).segment(0, 3);

	return cj;
}

cSpAlg::tSpVec cRBDUtil::BuildCjRevolute(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q_dot, int j)
{
	return cSpAlg::tSpVec::Zero();
}

cSpAlg::tSpVec cRBDUtil::BuildCjPrismatic(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q_dot, int j)
{
	return cSpAlg::tSpVec::Zero();
}

cSpAlg::tSpVec cRBDUtil::BuildCjPlanar(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q_dot, int j)
{
	return cSpAlg::tSpVec::Zero();
}

cSpAlg::tSpVec cRBDUtil::BuildCjFixed(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q_dot, int j)
{
	return cSpAlg::tSpVec::Zero();
}

cSpAlg::tSpVec cRBDUtil::BuildCjSpherical(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& q_dot, int j)
{
	return cSpAlg::tSpVec::Zero();
}

void cRBDUtil::BuildBiasForce(const cRBDModel& model, Eigen::VectorXd& out_bias_force)
{
	Eigen::VectorXd acc = Eigen::VectorXd::Zero(model.GetNumDof());
	SolveInvDyna(model, acc, out_bias_force);
}

void cRBDUtil::CalcGravityForce(const cRBDModel& model, Eigen::VectorXd& out_g_force)
{
	const Eigen::MatrixXd& joint_mat = model.GetJointMat();
	const Eigen::MatrixXd& body_defs = model.GetBodyDefs();
	const tVector& gravity = model.GetGravity();
	const Eigen::VectorXd& pose = model.GetPose();

	assert(joint_mat.rows() == body_defs.rows());
	assert(cKinTree::GetNumDof(joint_mat) == pose.rows());

	cSpAlg::tSpVec acc0 = cSpAlg::BuildSV(tVector::Zero(), gravity);

	int num_joints = cKinTree::GetNumJoints(joint_mat);
	Eigen::MatrixXd fs = Eigen::MatrixXd(num_joints, gSpVecSize);

	for (int j = 0; j < num_joints; ++j)
	{
		if (cKinTree::IsValidBody(body_defs, j))
		{
			cSpAlg::tSpTrans world_child_trans = model.GetSpWorldJointTrans(j);
			cSpAlg::tSpMat I = BuildInertiaSpatialMat(body_defs, j);
			cSpAlg::tSpVec curr_acc = cSpAlg::ApplyTransM(world_child_trans, acc0);
			cSpAlg::tSpVec curr_f = I * curr_acc;
			fs.row(j) = curr_f;
		}
	}

	out_g_force = Eigen::VectorXd::Zero(pose.size());
	for (int j = num_joints - 1; j >= 0; --j)
	{
		if (cKinTree::IsValidBody(body_defs, j))
		{
			cSpAlg::tSpVec curr_f = fs.row(j);
			const Eigen::Block<const Eigen::MatrixXd> S = model.GetJointSubspace(j);
			Eigen::VectorXd curr_tau = S.transpose() * curr_f;

			cKinTree::SetJointParams(joint_mat, j, curr_tau, out_g_force);
			if (cKinTree::HasParent(joint_mat, j))
			{
				int parent_id = cKinTree::GetParent(joint_mat, j);
				cSpAlg::tSpTrans child_parent_trans = model.GetSpChildParentTrans(j);
				fs.row(parent_id) += cSpAlg::ApplyTransF(child_parent_trans, curr_f);
			}
		}
	}
}
