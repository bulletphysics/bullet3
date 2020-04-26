#include "SpAlg.h"
#include <iostream>


cSpAlg::tSpVec cSpAlg::ConvertCoordM(const tSpVec& m0, const tVector& origin0, const tVector& origin1)
{
	return ConvertCoordM(m0, origin0, origin1, tMatrix::Identity());
}

cSpAlg::tSpVec cSpAlg::ConvertCoordM(const tSpVec& m0, const tVector& origin0, const tMatrix& R0,
												const tVector& origin1, const tMatrix& R1)
{
	tMatrix R = R1 * R0.transpose();
	return ConvertCoordM(m0, origin0, origin1, R);
}


cSpAlg::tSpVec cSpAlg::ConvertCoordM(const tSpVec& m0, const tVector& origin0, const tVector& origin1, const tMatrix& R)
{
	tSpTrans X = BuildTrans(R, origin1 - origin0);
	return ApplyTransM(X, m0);
}

cSpAlg::tSpVec cSpAlg::ConvertCoordF(const tSpVec& f0, const tVector& origin0, const tVector& origin1)
{
	return ConvertCoordF(f0, origin0, origin1, tMatrix::Identity());
}

cSpAlg::tSpVec cSpAlg::ConvertCoordF(const tSpVec& f0, const tVector& origin0, const tMatrix& R0,
	const tVector& origin1, const tMatrix& R1)
{
	tMatrix R = R1 * R0.transpose();
	return ConvertCoordF(f0, origin0, origin1, R);
}

cSpAlg::tSpVec cSpAlg::ConvertCoordF(const tSpVec& f0, const tVector& origin0,
	const tVector& origin1, const tMatrix& R)
{
	tSpTrans X = BuildTrans(R, origin1 - origin0);
	return ApplyTransF(X, f0);
}


cSpAlg::tSpVec cSpAlg::CrossM(const tSpVec& sv, const tSpVec& m)
{
	tVector sv_o = GetOmega(sv);
	tVector sv_v = GetV(sv);
	tVector m_o = GetOmega(m);
	tVector m_v = GetV(m);

	tVector o = sv_o.cross3(m_o);
	tVector v = sv_v.cross3(m_o) + sv_o.cross3(m_v);

	return BuildSV(o, v);
}

Eigen::MatrixXd cSpAlg::CrossMs(const tSpVec& sv, const Eigen::MatrixXd& ms)
{
	assert(ms.rows() == gSpVecSize);
	Eigen::MatrixXd result(gSpVecSize, ms.cols());
	for (int i = 0; i < ms.cols(); ++i)
	{
		const tSpVec& curr_m = ms.col(i);
		result.col(i) = CrossM(sv, curr_m);
	}
	return result;
}

cSpAlg::tSpVec cSpAlg::CrossF(const tSpVec& sv, const tSpVec& f)
{
	tVector sv_o = GetOmega(sv);
	tVector sv_v = GetV(sv);
	tVector f_o = GetOmega(f);
	tVector f_v = GetV(f);

	tVector o = sv_o.cross3(f_o) + sv_v.cross3(f_v);
	tVector v = sv_o.cross3(f_v);

	return BuildSV(o, v);
}

Eigen::MatrixXd cSpAlg::CrossFs(const tSpVec& sv, const Eigen::MatrixXd& fs)
{
	assert(fs.rows() == gSpVecSize);
	Eigen::MatrixXd result(gSpVecSize, fs.cols());
	for (int i = 0; i < fs.cols(); ++i)
	{
		const tSpVec& curr_f = fs.col(i);
		result.col(i) = CrossF(sv, curr_f);
	}
	return result;
}

cSpAlg::tSpVec cSpAlg::BuildSV(const tVector& v)
{
	return BuildSV(tVector::Zero(), v);
}

cSpAlg::tSpVec cSpAlg::BuildSV(const tVector& o, const tVector& v)
{
	tSpVec sv;
	SetOmega(o, sv);
	SetV(v, sv);
	return sv;
}

tVector cSpAlg::GetOmega(const tSpVec& sv)
{
	tVector o = tVector::Zero();
	o.block(0, 0, 3, 1) = sv.block(0, 0, 3, 1);
	return o;
}

void cSpAlg::SetOmega(const tVector& o, tSpVec& out_sv)
{
	out_sv.block(0, 0, 3, 1) = o.block(0, 0, 3, 1);
}

tVector cSpAlg::GetV(const tSpVec& sv)
{
	tVector v = tVector::Zero();
	v.block(0, 0, 3, 1) = sv.block(3, 0, 3, 1);
	return v;
}

void cSpAlg::SetV(const tVector& v, tSpVec& out_sv)
{
	out_sv.block(3, 0, 3, 1) = v.block(0, 0, 3, 1);
}

cSpAlg::tSpTrans cSpAlg::BuildTrans()
{
	return BuildTrans(tMatrix::Identity(), tVector::Zero());
}

cSpAlg::tSpTrans cSpAlg::BuildTrans(const tMatrix& E, const tVector& r)
{
	tSpTrans X;
	SetRad(r, X);
	SetRot(E, X);
	return X;
}

cSpAlg::tSpTrans cSpAlg::BuildTrans(const tVector& r)
{
	return BuildTrans(tMatrix::Identity(), r);
}

cSpAlg::tSpTrans cSpAlg::MatToTrans(const tMatrix& mat)
{
	tMatrix E = mat;
	tVector r = mat.col(3);
	r[3] = 0;
	r = -E.transpose() * r;
	return BuildTrans(E, r);
}

tMatrix cSpAlg::TransToMat(const tSpTrans& X)
{
	tMatrix E = GetRot(X);
	tVector r = GetRad(X);
	tMatrix m = E;
	m.col(3) = -E * r;
	m(3, 3) = 1;
	return m;
}

cSpAlg::tSpMat cSpAlg::BuildSpatialMatM(const tSpTrans& X)
{
	tSpMat m = tSpMat::Zero();
	tMatrix E = GetRot(X);
	tVector r = GetRad(X);
	tMatrix Er = E * cMathUtil::CrossMat(r);

	m.block(0, 0, 3, 3) = E.block(0, 0, 3, 3);
	m.block(3, 3, 3, 3) = E.block(0, 0, 3, 3);
	m.block(3, 0, 3, 3) = -Er.block(0, 0, 3, 3);
	return m;
}

cSpAlg::tSpMat cSpAlg::BuildSpatialMatF(const tSpTrans& X)
{
	tSpMat m = tSpMat::Zero();
	tMatrix E = GetRot(X);
	tVector r = GetRad(X);
	tMatrix Er = E * cMathUtil::CrossMat(r);

	m.block(0, 0, 3, 3) = E.block(0, 0, 3, 3);
	m.block(3, 3, 3, 3) = E.block(0, 0, 3, 3);
	m.block(0, 3, 3, 3) = -Er.block(0, 0, 3, 3);
	return m;
}


cSpAlg::tSpTrans cSpAlg::InvTrans(const tSpTrans& X)
{
	tMatrix E = GetRot(X);
	tVector r = GetRad(X);
	tSpTrans inv_X = BuildTrans(E.transpose(), -E * r);
	return inv_X;
}

tMatrix cSpAlg::GetRot(const tSpTrans& X)
{
	tMatrix E = tMatrix::Zero();
	E.block(0, 0, 3, 3) = X.block(0, 0, 3, 3);
	return E;
}

void cSpAlg::SetRot(const tMatrix& E, tSpTrans& out_X)
{
	out_X.block(0, 0, 3, 3) = E.block(0, 0, 3, 3);
}

tVector cSpAlg::GetRad(const tSpTrans& X)
{
	tVector r = tVector::Zero();
	r.block(0, 0, 3, 1) = X.block(0, 3, 3, 1);
	return r;
}

void cSpAlg::SetRad(const tVector& r, tSpTrans& out_X)
{
	out_X.block(0, 3, 3, 1) = r.block(0, 0, 3, 1);
}

cSpAlg::tSpVec cSpAlg::ApplyTransM(const tSpTrans& X, const tSpVec& sv)
{
	tMatrix E = GetRot(X);
	tVector r = GetRad(X);
	tVector o0 = GetOmega(sv);
	tVector v0 = GetV(sv);

	tVector o1 = E * o0;
	tVector v1 = E * (v0 - r.cross3(o0));

	tSpVec new_vec = BuildSV(o1, v1);
	return new_vec;
}

cSpAlg::tSpVec cSpAlg::ApplyTransF(const tSpTrans& X, const tSpVec& sv)
{
	tMatrix E = GetRot(X);
	tVector r = GetRad(X);
	tVector o0 = GetOmega(sv);
	tVector v0 = GetV(sv);

	tVector o1 = E * (o0 - r.cross3(v0));
	tVector v1 = E * v0;

	tSpVec new_vec = BuildSV(o1, v1);
	return new_vec;
}

Eigen::MatrixXd cSpAlg::ApplyTransM(const tSpTrans& X, const Eigen::MatrixXd& sm)
{
	assert(sm.rows() == gSpVecSize);
	Eigen::MatrixXd result(gSpVecSize, sm.cols());
	for (int i = 0; i < sm.cols(); ++i)
	{
		const tSpVec& curr_sv = sm.col(i);
		result.col(i) = ApplyTransM(X, curr_sv);
	}
	return result;
}

Eigen::MatrixXd cSpAlg::ApplyTransF(const tSpTrans& X, const Eigen::MatrixXd& sm)
{
	assert(sm.rows() == gSpVecSize);
	Eigen::MatrixXd result(gSpVecSize, sm.cols());
	for (int i = 0; i < sm.cols(); ++i)
	{
		const tSpVec& curr_sv = sm.col(i);
		result.col(i) = ApplyTransF(X, curr_sv);
	}
	return result;
}

cSpAlg::tSpVec cSpAlg::ApplyInvTransM(const tSpTrans& X, const tSpVec& sv)
{
	tMatrix E = GetRot(X);
	tVector r = GetRad(X);
	tVector o0 = GetOmega(sv);
	tVector v0 = GetV(sv);

	tVector o1 = E.transpose() * o0;
	tVector v1 = E.transpose() * v0 + r.cross3(E.transpose() * o0);

	tSpVec new_vec = BuildSV(o1, v1);
	return new_vec;
}

cSpAlg::tSpVec cSpAlg::ApplyInvTransF(const tSpTrans& X, const tSpVec& sv)
{
	tMatrix E = GetRot(X);
	tVector r = GetRad(X);
	tVector o0 = GetOmega(sv);
	tVector v0 = GetV(sv);

	tVector o1 = E.transpose() * o0 + r.cross3(E.transpose() * v0);
	tVector v1 = E.transpose() * v0;

	tSpVec new_vec = BuildSV(o1, v1);
	return new_vec;
}

Eigen::MatrixXd cSpAlg::ApplyInvTransM(const tSpTrans& X, const Eigen::MatrixXd& sm)
{
	assert(sm.rows() == gSpVecSize);
	Eigen::MatrixXd result(sm.rows(), sm.cols());
	for (int i = 0; i < sm.cols(); ++i)
	{
		const tSpVec& curr_sv = sm.col(i);
		result.col(i) = ApplyInvTransM(X, curr_sv);
	}
	return result;
}

Eigen::MatrixXd cSpAlg::ApplyInvTransF(const tSpTrans& X, const Eigen::MatrixXd& sm)
{
	assert(sm.rows() == gSpVecSize);
	Eigen::MatrixXd result(sm.rows(), sm.cols());
	for (int i = 0; i < sm.cols(); ++i)
	{
		const tSpVec& curr_sv = sm.col(i);
		result.col(i) = ApplyInvTransF(X, curr_sv);
	}
	return result;
}

cSpAlg::tSpTrans cSpAlg::CompTrans(const tSpTrans& X0, const tSpTrans& X1)
{
	tMatrix E0 = GetRot(X0);
	tMatrix E1 = GetRot(X1);
	tVector r0 = GetRad(X0);
	tVector r1 = GetRad(X1);

	tSpTrans X = BuildTrans(E0 * E1, r1 + E1.transpose() * r0);
	return X;
}

cSpAlg::tSpTrans cSpAlg::GetTrans(const Eigen::MatrixXd& trans_arr, int j)
{
	assert(trans_arr.rows() >= gSVTransRows);
	assert((trans_arr.rows() % gSVTransRows) == 0);
	assert(trans_arr.cols() == gSVTransCols);

	int row_idx = j * gSVTransRows;
	assert(row_idx <= trans_arr.rows() - gSVTransRows);

	tSpTrans X = trans_arr.block(row_idx, 0, gSVTransRows, gSVTransCols);
	return X;
}
