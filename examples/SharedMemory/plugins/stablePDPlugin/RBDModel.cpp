#include "RBDModel.h"
#include "RBDUtil.h"
#include "KinTree.h"
#include "LinearMath/btQuickprof.h"
cRBDModel::cRBDModel()
{
}

cRBDModel::~cRBDModel()
{
}

void cRBDModel::Init(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, const tVector& gravity)
{
	assert(joint_mat.rows() == body_defs.rows());
	mGravity = gravity;
	mJointMat = joint_mat;
	mBodyDefs = body_defs;

	int num_dofs = GetNumDof();
	int num_joints = GetNumJoints();
	const int svs = gSpVecSize;

	mPose = Eigen::VectorXd::Zero(num_dofs);
	mVel = Eigen::VectorXd::Zero(num_dofs);

	tMatrix trans_mat;
	InitJointSubspaceArr();
	mChildParentMatArr = Eigen::MatrixXd::Zero(num_joints * trans_mat.rows(), trans_mat.cols());
	mSpWorldJointTransArr = Eigen::MatrixXd::Zero(num_joints * gSVTransRows, gSVTransCols);
	mMassMat = Eigen::MatrixXd::Zero(num_dofs, num_dofs);
	mBiasForce = Eigen::VectorXd::Zero(num_dofs);
	mInertiaBuffer = Eigen::MatrixXd::Zero(num_joints * svs, svs);
}

void cRBDModel::Update(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel)
{
	SetPose(pose);
	SetVel(vel);

	{
		BT_PROFILE("rbdModel::UpdateJointSubspaceArr");
		UpdateJointSubspaceArr();
	}
	{
		BT_PROFILE("rbdModel::UpdateChildParentMatArr");
		UpdateChildParentMatArr();
	}
	{
		BT_PROFILE("rbdModel::UpdateSpWorldTrans");
		UpdateSpWorldTrans();
	}
	{
		BT_PROFILE("UpdateMassMat");
		UpdateMassMat();
	}
	{
		BT_PROFILE("UpdateBiasForce");
		UpdateBiasForce();
	}
}

int cRBDModel::GetNumDof() const
{
	return cKinTree::GetNumDof(mJointMat);
}

int cRBDModel::GetNumJoints() const
{
	return cKinTree::GetNumJoints(mJointMat);
}

void cRBDModel::SetGravity(const tVector& gravity)
{
	mGravity = gravity;
}



const tVector& cRBDModel::GetGravity() const
{
	return mGravity;
}

const Eigen::MatrixXd& cRBDModel::GetJointMat() const
{
	return mJointMat;
}

const Eigen::MatrixXd& cRBDModel::GetBodyDefs() const
{
	return mBodyDefs;
}

const Eigen::VectorXd& cRBDModel::GetPose() const
{
	return mPose;
}

const Eigen::VectorXd& cRBDModel::GetVel() const
{
	return mVel;
}

int cRBDModel::GetParent(int j) const
{
	return cKinTree::GetParent(mJointMat, j);
}

const Eigen::MatrixXd& cRBDModel::GetMassMat() const
{
	return mMassMat;
}

const Eigen::VectorXd& cRBDModel::GetBiasForce() const
{
	return mBiasForce;
}

Eigen::MatrixXd& cRBDModel::GetInertiaBuffer()
{
	return mInertiaBuffer;
}

tMatrix cRBDModel::GetChildParentMat(int j) const
{
	assert(j >= 0 && j < GetNumJoints());
	tMatrix trans;
	int r = static_cast<int>(trans.rows());
	int c = static_cast<int>(trans.cols());
	trans = mChildParentMatArr.block(j * r, 0, r, c);
	return trans;
}

tMatrix cRBDModel::GetParentChildMat(int j) const
{
	tMatrix child_parent_trans = GetChildParentMat(j);
	tMatrix parent_child_trans = cMathUtil::InvRigidMat(child_parent_trans);
	return parent_child_trans;
}

cSpAlg::tSpTrans cRBDModel::GetSpChildParentTrans(int j) const
{
	tMatrix mat = GetChildParentMat(j);
	return cSpAlg::MatToTrans(mat);
}

cSpAlg::tSpTrans cRBDModel::GetSpParentChildTrans(int j) const
{
	tMatrix mat = GetParentChildMat(j);
	return cSpAlg::MatToTrans(mat);
}

tMatrix cRBDModel::GetWorldJointMat(int j) const
{
	cSpAlg::tSpTrans trans = GetSpWorldJointTrans(j);
	return cSpAlg::TransToMat(trans);
}

tMatrix cRBDModel::GetJointWorldMat(int j) const
{
	cSpAlg::tSpTrans trans = GetSpJointWorldTrans(j);
	return cSpAlg::TransToMat(trans);
}

cSpAlg::tSpTrans cRBDModel::GetSpWorldJointTrans(int j) const
{
	assert(j >= 0 && j < GetNumJoints());
	cSpAlg::tSpTrans trans = cSpAlg::GetTrans(mSpWorldJointTransArr, j);
	return trans;
}

cSpAlg::tSpTrans cRBDModel::GetSpJointWorldTrans(int j) const
{
	cSpAlg::tSpTrans world_joint_trans = GetSpWorldJointTrans(j);
	return cSpAlg::InvTrans(world_joint_trans);
}

const Eigen::Block<const Eigen::MatrixXd> cRBDModel::GetJointSubspace(int j) const
{
	assert(j >= 0 && j < GetNumJoints());
	int offset = cKinTree::GetParamOffset(mJointMat, j);
	int dim = cKinTree::GetParamSize(mJointMat, j);
	int r = static_cast<int>(mJointSubspaceArr.rows());
	return mJointSubspaceArr.block(0, offset, r, dim);
}

tVector cRBDModel::CalcJointWorldPos(int j) const
{
	cSpAlg::tSpTrans world_joint_trans = GetSpWorldJointTrans(j);
	tVector r = cSpAlg::GetRad(world_joint_trans);
	return r;
}

void cRBDModel::SetPose(const Eigen::VectorXd& pose)
{
	mPose = pose;
}

void cRBDModel::SetVel(const Eigen::VectorXd& vel)
{
	mVel = vel;
}

void cRBDModel::InitJointSubspaceArr()
{
	int num_dofs = GetNumDof();
	int num_joints = GetNumJoints();
	mJointSubspaceArr = Eigen::MatrixXd(gSpVecSize, num_dofs);
	for (int j = 0; j < num_joints; ++j)
	{
		int offset = cKinTree::GetParamOffset(mJointMat, j);
		int dim = cKinTree::GetParamSize(mJointMat, j);
		int r = static_cast<int>(mJointSubspaceArr.rows());
		mJointSubspaceArr.block(0, offset, r, dim) = cRBDUtil::BuildJointSubspace(mJointMat, mPose, j);
	}
}

void cRBDModel::UpdateJointSubspaceArr()
{
	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		bool const_subspace = cRBDUtil::IsConstJointSubspace(mJointMat, j);
		if (!const_subspace)
		{
			int offset = cKinTree::GetParamOffset(mJointMat, j);
			int dim = cKinTree::GetParamSize(mJointMat, j);
			int r = static_cast<int>(mJointSubspaceArr.rows());
			mJointSubspaceArr.block(0, offset, r, dim) = cRBDUtil::BuildJointSubspace(mJointMat, mPose, j);
		}
	}
}

void cRBDModel::UpdateChildParentMatArr()
{
	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		tMatrix child_parent_trans = cKinTree::ChildParentTrans(mJointMat, mPose, j);
		int r = static_cast<int>(child_parent_trans.rows());
		int c = static_cast<int>(child_parent_trans.cols());
		mChildParentMatArr.block(j * r, 0, r, c) = child_parent_trans;
	}
}

void cRBDModel::UpdateSpWorldTrans()
{
	cRBDUtil::CalcWorldJointTransforms(*this, mSpWorldJointTransArr);
}

void cRBDModel::UpdateMassMat()
{
	cRBDUtil::BuildMassMat(*this, mInertiaBuffer, mMassMat);
}

void cRBDModel::UpdateBiasForce()
{
	cRBDUtil::BuildBiasForce(*this, mBiasForce);
}