#include "MathUtil.h"
#include <time.h>
#define _USE_MATH_DEFINES
#include <math.h>


int cMathUtil::Clamp(int val, int min, int max)
{
	return std::max(min, std::min(val, max));
}

void cMathUtil::Clamp(const Eigen::VectorXd& min, const Eigen::VectorXd& max, Eigen::VectorXd& out_vec)
{
	out_vec = out_vec.cwiseMin(max).cwiseMax(min);
}

double cMathUtil::Clamp(double val, double min, double max)
{
	return std::max(min, std::min(val, max));
}

double cMathUtil::Saturate(double val)
{
	return Clamp(val, 0.0, 1.0);
}

double cMathUtil::Lerp(double t, double val0, double val1)
{
	return (1 - t) * val0 + t * val1;
}

double cMathUtil::NormalizeAngle(double theta)
{
	// normalizes theta to be between [-pi, pi]
	double norm_theta = fmod(theta, 2 * M_PI);
	if (norm_theta > M_PI)
	{
		norm_theta = -2 * M_PI + norm_theta;
	}
	else if (norm_theta < -M_PI)
	{
		norm_theta = 2 * M_PI + norm_theta;
	}
	return norm_theta;
}


double cMathUtil::SmoothStep(double t)
{
	double val = t * t * t * (t * (t * 6 - 15) + 10);
	return val;
}


tMatrix cMathUtil::TranslateMat(const tVector& trans)
{
	tMatrix mat = tMatrix::Identity();
	mat(0, 3) = trans[0];
	mat(1, 3) = trans[1];
	mat(2, 3) = trans[2];
	return mat;
}

tMatrix cMathUtil::ScaleMat(double scale)
{
	return ScaleMat(tVector::Ones() * scale);
}

tMatrix cMathUtil::ScaleMat(const tVector& scale)
{
	tMatrix mat = tMatrix::Identity();
	mat(0, 0) = scale[0];
	mat(1, 1) = scale[1];
	mat(2, 2) = scale[2];
	return mat;
}

tMatrix cMathUtil::RotateMat(const tVector& euler)
{
	double x = euler[0];
	double y = euler[1];
	double z = euler[2];

	double x_s = std::sin(x);
	double x_c = std::cos(x);
	double y_s = std::sin(y);
	double y_c = std::cos(y);
	double z_s = std::sin(z);
	double z_c = std::cos(z);

	tMatrix mat = tMatrix::Identity();
	mat(0, 0) = y_c * z_c;
	mat(1, 0) = y_c * z_s;
	mat(2, 0) = -y_s;

	mat(0, 1) = x_s * y_s * z_c - x_c * z_s;
	mat(1, 1) = x_s * y_s * z_s + x_c * z_c;
	mat(2, 1) = x_s * y_c;

	mat(0, 2) = x_c * y_s * z_c + x_s * z_s;
	mat(1, 2) = x_c * y_s * z_s - x_s * z_c;
	mat(2, 2) = x_c * y_c;

	return mat;
}

tMatrix cMathUtil::RotateMat(const tVector& axis, double theta)
{
	assert(std::abs(axis.squaredNorm() - 1) < 0.0001);
	
	double c = std::cos(theta);
	double s = std::sin(theta);
	double x = axis[0];
	double y = axis[1];
	double z = axis[2];

	tMatrix mat;
	mat <<	c + x * x * (1 - c),		x * y * (1 - c) - z * s,	x * z * (1 - c) + y * s,	0,
			y * x * (1 - c) + z * s,	c + y * y * (1 - c),		y * z * (1 - c) - x * s,	0,
			z * x * (1 - c) - y * s,	z * y * (1 - c) + x * s,	c + z * z * (1 - c),		0,
			0,							0,							0,							1;

	return mat;
}

tMatrix cMathUtil::RotateMat(const tQuaternion& q)
{
	tMatrix mat = tMatrix::Identity();

	double sqw = q.w() * q.w();
	double sqx = q.x()*  q.x();
	double sqy = q.y() * q.y();
	double sqz = q.z() * q.z();
	double invs = 1 / (sqx + sqy + sqz + sqw);

	mat(0, 0) = (sqx - sqy - sqz + sqw) * invs;
	mat(1, 1) = (-sqx + sqy - sqz + sqw) * invs;
	mat(2, 2) = (-sqx - sqy + sqz + sqw) * invs;

	double tmp1 = q.x()*q.y();
	double tmp2 = q.z()*q.w();
	mat(1, 0) = 2.0 * (tmp1 + tmp2) * invs;
	mat(0, 1) = 2.0 * (tmp1 - tmp2) * invs;

	tmp1 = q.x()*q.z();
	tmp2 = q.y()*q.w();
	mat(2, 0) = 2.0 * (tmp1 - tmp2) * invs;
	mat(0, 2) = 2.0 * (tmp1 + tmp2) * invs;

	tmp1 = q.y()*q.z();
	tmp2 = q.x()*q.w();
	mat(2, 1) = 2.0 * (tmp1 + tmp2) * invs;
	mat(1, 2) = 2.0 * (tmp1 - tmp2) * invs;

	return mat;
}

tMatrix cMathUtil::CrossMat(const tVector& a)
{
	tMatrix m;
	m << 0,		-a[2],	a[1],	0,
		 a[2],	0,		-a[0],	0,
		 -a[1],	a[0],	0,		0,
		 0,		0,		0,		1;
	return m;
}

tMatrix cMathUtil::InvRigidMat(const tMatrix& mat)
{
	tMatrix inv_mat = tMatrix::Zero();
	inv_mat.block(0, 0, 3, 3) = mat.block(0, 0, 3, 3).transpose();
	inv_mat.col(3) = -inv_mat * mat.col(3);
	inv_mat(3, 3) = 1;
	return inv_mat;
}

tVector cMathUtil::GetRigidTrans(const tMatrix& mat)
{
	return tVector(mat(0, 3), mat(1, 3), mat(2, 3), 0);
}

tVector cMathUtil::InvEuler(const tVector& euler)
{
	tMatrix inv_mat = cMathUtil::RotateMat(tVector(1, 0, 0, 0), -euler[0])
					* cMathUtil::RotateMat(tVector(0, 1, 0, 0), -euler[1])
					* cMathUtil::RotateMat(tVector(0, 0, 1, 0), -euler[2]);
	tVector inv_euler = cMathUtil::RotMatToEuler(inv_mat);
	return inv_euler;
}

void cMathUtil::RotMatToAxisAngle(const tMatrix& mat, tVector& out_axis, double& out_theta)
{
	double c = (mat(0, 0) + mat(1, 1) + mat(2, 2) - 1) * 0.5;
	c = cMathUtil::Clamp(c, -1.0, 1.0);

	out_theta = std::acos(c);
	if (std::abs(out_theta) < 0.00001)
	{
		out_axis = tVector(0, 0, 1, 0);
	}
	else
	{
		double m21 = mat(2, 1) - mat(1, 2);
		double m02 = mat(0, 2) - mat(2, 0);
		double m10 = mat(1, 0) - mat(0, 1);
		double denom = std::sqrt(m21 * m21 + m02 * m02 + m10 * m10);
		out_axis[0] = m21 / denom;
		out_axis[1] = m02 / denom;
		out_axis[2] = m10 / denom;
		out_axis[3] = 0;
	}
}

tVector cMathUtil::RotMatToEuler(const tMatrix& mat)
{
	tVector euler;
	euler[0] = std::atan2(mat(2, 1), mat(2, 2));
	euler[1] = std::atan2(-mat(2, 0), std::sqrt(mat(2, 1) * mat(2, 1) + mat(2, 2) * mat(2, 2)));
	euler[2] = std::atan2(mat(1, 0), mat(0, 0)); 
	euler[3] = 0;
	return euler;
}

tQuaternion cMathUtil::RotMatToQuaternion(const tMatrix& mat)
{
	double tr = mat(0, 0) + mat(1, 1) + mat(2, 2);
	tQuaternion q;

	if (tr > 0) {
		double S = sqrt(tr + 1.0) * 2; // S=4*qw 
		q.w() = 0.25 * S;
		q.x() = (mat(2, 1) - mat(1, 2)) / S;
		q.y() = (mat(0, 2) - mat(2, 0)) / S;
		q.z() = (mat(1, 0) - mat(0, 1)) / S;
	}
	else if ((mat(0, 0) > mat(1, 1) && (mat(0, 0) > mat(2, 2)))) {
		double S = sqrt(1.0 + mat(0, 0) - mat(1, 1) - mat(2, 2)) * 2; // S=4*qx 
		q.w() = (mat(2, 1) - mat(1, 2)) / S;
		q.x() = 0.25 * S;
		q.y() = (mat(0, 1) + mat(1, 0)) / S;
		q.z() = (mat(0, 2) + mat(2, 0)) / S;
	}
	else if (mat(1, 1) > mat(2, 2)) {
		double S = sqrt(1.0 + mat(1, 1) - mat(0, 0) - mat(2, 2)) * 2; // S=4*qy
		q.w() = (mat(0, 2) - mat(2, 0)) / S;
		q.x() = (mat(0, 1) + mat(1, 0)) / S;
		q.y() = 0.25 * S;
		q.z() = (mat(1, 2) + mat(2, 1)) / S;
	}
	else {
		double S = sqrt(1.0 + mat(2, 2) - mat(0, 0) - mat(1, 1)) * 2; // S=4*qz
		q.w() = (mat(1, 0) - mat(0, 1)) / S;
		q.x() = (mat(0, 2) + mat(2, 0)) / S;
		q.y() = (mat(1, 2) + mat(2, 1)) / S;
		q.z() = 0.25 * S;
	}

	return q;
}

void cMathUtil::EulerToAxisAngle(const tVector& euler, tVector& out_axis, double& out_theta)
{
	double x = euler[0];
	double y = euler[1];
	double z = euler[2];

	double x_s = std::sin(x);
	double x_c = std::cos(x);
	double y_s = std::sin(y);
	double y_c = std::cos(y);
	double z_s = std::sin(z);
	double z_c = std::cos(z);

	double c = (y_c * z_c + x_s * y_s * z_s + x_c * z_c + x_c * y_c - 1) * 0.5;
	c = Clamp(c, -1.0, 1.0);

	out_theta = std::acos(c);
	if (std::abs(out_theta) < 0.00001)
	{
		out_axis = tVector(0, 0, 1, 0);
	}
	else
	{
		double m21 = x_s * y_c - x_c * y_s * z_s + x_s * z_c;
		double m02 = x_c * y_s * z_c + x_s * z_s + y_s;
		double m10 = y_c * z_s - x_s * y_s * z_c + x_c * z_s;
		double denom = std::sqrt(m21 * m21 + m02 * m02 + m10 * m10);
		out_axis[0] = m21 / denom;
		out_axis[1] = m02 / denom;
		out_axis[2] = m10 / denom;
		out_axis[3] = 0;
	}
}


tMatrix cMathUtil::DirToRotMat(const tVector& dir, const tVector& up)
{
	tVector x = up.cross3(dir);
	double x_norm = x.norm();
	if (x_norm == 0)
	{
		x_norm = 1;
		x = (dir.dot(up) >= 0) ? tVector(1, 0, 0, 0) : tVector(-1, 0, 0, 0);
	}
	x /= x_norm;

	tVector y = dir.cross3(x).normalized();
	tVector z = dir;

	tMatrix mat = tMatrix::Identity();
	mat.block(0, 0, 3, 1) = x.segment(0, 3);
	mat.block(0, 1, 3, 1) = y.segment(0, 3);
	mat.block(0, 2, 3, 1) = z.segment(0, 3);

	return mat;
}

void cMathUtil::DeltaRot(const tVector& axis0, double theta0, const tVector& axis1, double theta1,
							tVector& out_axis, double& out_theta)
{
	tMatrix R0 = RotateMat(axis0, theta0);
	tMatrix R1 = RotateMat(axis1, theta1);
	tMatrix M = DeltaRot(R0, R1);
	RotMatToAxisAngle(M, out_axis, out_theta);
}

tMatrix cMathUtil::DeltaRot(const tMatrix& R0, const tMatrix& R1)
{
	return R1 * R0.transpose();
}

tQuaternion cMathUtil::EulerToQuaternion(const tVector& euler)
{
	tVector axis;
	double theta;
	EulerToAxisAngle(euler, axis, theta);
	return AxisAngleToQuaternion(axis, theta);
}


tQuaternion cMathUtil::AxisAngleToQuaternion(const tVector& axis, double theta)
{
	// axis must be normalized
	double c = std::cos(theta / 2);
	double s = std::sin(theta / 2);
	tQuaternion q;
	q.w() = c;
	q.x() = s * axis[0];
	q.y() = s * axis[1];
	q.z() = s * axis[2];
	return q;
}

void cMathUtil::QuaternionToAxisAngle(const tQuaternion& q, tVector& out_axis, double& out_theta)
{
	out_theta = 0;
	out_axis = tVector(0, 0, 1, 0);

	tQuaternion q1 = q;
	if (q1.w() > 1)
	{
		q1.normalize();
	}

	double sin_theta = std::sqrt(1 - q1.w() * q1.w());
	if (sin_theta > 0.000001)
	{
		out_theta = 2 * std::acos(q1.w());
		out_theta = cMathUtil::NormalizeAngle(out_theta);
		out_axis = tVector(q1.x(), q1.y(), q1.z(), 0) / sin_theta;
	}
}

tMatrix cMathUtil::BuildQuaternionDiffMat(const tQuaternion& q)
{
	tMatrix mat;
	mat << -0.5 * q.x(), -0.5 * q.y(), -0.5 * q.z(), 0,
		0.5 * q.w(), -0.5 * q.z(), 0.5 * q.y(), 0,
		0.5 * q.z(), 0.5 * q.w(), -0.5 * q.x(), 0,
		-0.5 * q.y(), 0.5 * q.x(), 0.5 * q.w(), 0;
	return mat;
}

tVector cMathUtil::CalcQuaternionVel(const tQuaternion& q0, const tQuaternion& q1, double dt)
{
	tQuaternion q_diff = cMathUtil::QuatDiff(q0, q1);
	tVector axis;
	double theta;
	QuaternionToAxisAngle(q_diff, axis, theta);
	return (theta / dt) * axis;
}

tVector cMathUtil::CalcQuaternionVelRel(const tQuaternion& q0, const tQuaternion& q1, double dt)
{
	// calculate relative rotational velocity in the coordinate frame of q0
	tQuaternion q_diff = q0.conjugate() * q1;
	tVector axis;
	double theta;
	QuaternionToAxisAngle(q_diff, axis, theta);
	return (theta / dt) * axis;
}

tQuaternion cMathUtil::VecToQuat(const tVector& v)
{
	return tQuaternion(v[0], v[1], v[2], v[3]);
}

tVector cMathUtil::QuatToVec(const tQuaternion& q)
{
	return tVector(q.w(), q.x(), q.y(), q.z());
}

tQuaternion cMathUtil::QuatDiff(const tQuaternion& q0, const tQuaternion& q1)
{
	return q1 * q0.conjugate();
}

double cMathUtil::QuatDiffTheta(const tQuaternion& q0, const tQuaternion& q1)
{
	tQuaternion dq = QuatDiff(q0, q1);
	return QuatTheta(dq);
}

double cMathUtil::QuatTheta(const tQuaternion& dq)
{
	double theta = 0;
	tQuaternion q1 = dq;
	if (q1.w() > 1)
	{
		q1.normalize();
	}

	double sin_theta = std::sqrt(1 - q1.w() * q1.w());
	if (sin_theta > 0.0001)
	{
		theta = 2 * std::acos(q1.w());
		theta = cMathUtil::NormalizeAngle(theta);
	}
	return theta;
}

tQuaternion cMathUtil::VecDiffQuat(const tVector& v0, const tVector& v1)
{
	return tQuaternion::FromTwoVectors(v0.segment(0, 3), v1.segment(0, 3));
}

tVector cMathUtil::QuatRotVec(const tQuaternion& q, const tVector& dir)
{
	tVector rot_dir = tVector::Zero();
	rot_dir.segment(0, 3)  = q * dir.segment(0, 3);
	return rot_dir;
}

tQuaternion cMathUtil::MirrorQuaternion(const tQuaternion& q, eAxis axis)
{
	tQuaternion mirror_q;
	mirror_q.w() = q.w();
	mirror_q.x() = (axis == eAxisX) ? q.x() : -q.x();
	mirror_q.y() = (axis == eAxisY) ? q.y() : -q.y();
	mirror_q.z() = (axis == eAxisZ) ? q.z() : -q.z();
	return mirror_q;
}

double cMathUtil::Sign(double val)
{
	return SignAux<double>(val);
}

int cMathUtil::Sign(int val)
{
	return SignAux<int>(val);
}

double cMathUtil::AddAverage(double avg0, int count0, double avg1, int count1)
{
	double total = count0 + count1;
	return (count0 / total) * avg0 + (count1 / total) * avg1;
}

tVector cMathUtil::AddAverage(const tVector& avg0, int count0, const tVector& avg1, int count1)
{
	double total = count0 + count1;
	return (count0 / total) * avg0 + (count1 / total) * avg1 ;
}

void cMathUtil::AddAverage(const Eigen::VectorXd& avg0, int count0, const Eigen::VectorXd& avg1, int count1, Eigen::VectorXd& out_result)
{
	double total = count0 + count1;
	out_result = (count0 / total) * avg0 + (count1 / total) * avg1;
}

void cMathUtil::CalcSoftmax(const Eigen::VectorXd& vals, double temp, Eigen::VectorXd& out_prob)
{
	assert(out_prob.size() == vals.size());
	int num_vals = static_cast<int>(vals.size());
	double sum = 0;
	double max_val = vals.maxCoeff();
	for (int i = 0; i < num_vals; ++i)
	{
		double val = vals[i];
		val = std::exp((val - max_val) / temp);
		out_prob[i] = val;
		sum += val;
	}

	out_prob /= sum;
}

double cMathUtil::EvalGaussian(const Eigen::VectorXd& mean, const Eigen::VectorXd& covar, const Eigen::VectorXd& sample)
{
	assert(mean.size() == covar.size());
	assert(sample.size() == covar.size());

	Eigen::VectorXd diff = sample - mean;
	double exp_val = diff.dot(diff.cwiseQuotient(covar));
	double likelihood = std::exp(-0.5 * exp_val);

	double partition = CalcGaussianPartition(covar);
	likelihood /= partition;
	return likelihood;
}

double cMathUtil::EvalGaussian(double mean, double covar, double sample)
{
	double diff = sample - mean;
	double exp_val = diff * diff / covar;
	double norm = 1 / std::sqrt(2 * M_PI * covar);
	double likelihood = norm * std::exp(-0.5 * exp_val);
	return likelihood;
}

double cMathUtil::CalcGaussianPartition(const Eigen::VectorXd& covar)
{
	int data_size = static_cast<int>(covar.size());
	double det = covar.prod();
	double partition = std::sqrt(std::pow(2 * M_PI, data_size) * det);
	return partition;
}

double cMathUtil::EvalGaussianLogp(const Eigen::VectorXd& mean, const Eigen::VectorXd& covar, const Eigen::VectorXd& sample)
{
	int data_size = static_cast<int>(covar.size());

	Eigen::VectorXd diff = sample - mean;
	double logp = -0.5 * diff.dot(diff.cwiseQuotient(covar));
	double det = covar.prod();
	logp += -0.5 * (data_size * std::log(2 * M_PI) + std::log(det));

	return logp;
}

double cMathUtil::EvalGaussianLogp(double mean, double covar, double sample)
{
	double diff = sample - mean;
	double logp = -0.5 * diff * diff / covar;
	logp += -0.5 * (std::log(2 * M_PI) + std::log(covar));
	return logp;
}

double cMathUtil::Sigmoid(double x)
{
	return Sigmoid(x, 1, 0);
}

double cMathUtil::Sigmoid(double x, double gamma, double bias)
{
	double exp = -gamma * (x + bias);
	double val = 1 / (1 + std::exp(exp));
	return val;
}


tVector cMathUtil::CalcBarycentric(const tVector& p, const tVector& a, const tVector& b, const tVector& c)
{
	tVector v0 = b - a;
	tVector v1 = c - a;
	tVector v2 = p - a;

	double d00 = v0.dot(v0);
	double d01 = v0.dot(v1);
	double d11 = v1.dot(v1);
	double d20 = v2.dot(v0);
	double d21 = v2.dot(v1);
	double denom = d00 * d11 - d01 * d01;
	double v = (d11 * d20 - d01 * d21) / denom;
	double w = (d00 * d21 - d01 * d20) / denom;
	double u = 1.0f - v - w;

	return tVector(u, v, w, 0);
}

bool cMathUtil::ContainsAABB(const tVector& pt, const tVector& aabb_min, const tVector& aabb_max)
{
	bool contains = pt[0] >= aabb_min[0] && pt[1] >= aabb_min[1] && pt[2] >= aabb_min[2]
					&& pt[0] <= aabb_max[0] && pt[1] <= aabb_max[1] && pt[2] <= aabb_max[2];
	return contains;
}

bool cMathUtil::ContainsAABB(const tVector& aabb_min0, const tVector& aabb_max0, const tVector& aabb_min1, const tVector& aabb_max1)
{
	return ContainsAABB(aabb_min0, aabb_min1, aabb_max1) && ContainsAABB(aabb_max0, aabb_min1, aabb_max1);
}

bool cMathUtil::ContainsAABBXZ(const tVector& pt, const tVector& aabb_min, const tVector& aabb_max)
{
	bool contains = pt[0] >= aabb_min[0] && pt[2] >= aabb_min[2]
		&& pt[0] <= aabb_max[0] && pt[2] <= aabb_max[2];
	return contains;
}

bool cMathUtil::ContainsAABBXZ(const tVector& aabb_min0, const tVector& aabb_max0, const tVector& aabb_min1, const tVector& aabb_max1)
{
	return ContainsAABBXZ(aabb_min0, aabb_min1, aabb_max1) && ContainsAABBXZ(aabb_max0, aabb_min1, aabb_max1);
}

void cMathUtil::CalcAABBIntersection(const tVector& aabb_min0, const tVector& aabb_max0, const tVector& aabb_min1, const tVector& aabb_max1,
									tVector& out_min, tVector& out_max)
{
	out_min = aabb_min0.cwiseMax(aabb_min1);
	out_max = aabb_max0.cwiseMin(aabb_max1);
	if (out_min[0] > out_max[0])
	{
		out_min[0] = 0;
		out_max[0] = 0;
	}
	if (out_min[1] > out_max[1])
	{
		out_min[1] = 0;
		out_max[1] = 0;
	}
	if (out_min[2] > out_max[2])
	{
		out_min[2] = 0;
		out_max[2] = 0;
	}
}

void cMathUtil::CalcAABBUnion(const tVector& aabb_min0, const tVector& aabb_max0, const tVector& aabb_min1, const tVector& aabb_max1,
								tVector& out_min, tVector& out_max)
{
	out_min = aabb_min0.cwiseMin(aabb_min1);
	out_max = aabb_max0.cwiseMax(aabb_max1);
}

bool cMathUtil::IntersectAABB(const tVector& aabb_min0, const tVector& aabb_max0, const tVector& aabb_min1, const tVector& aabb_max1)
{
	tVector center0 = 0.5 * (aabb_max0 + aabb_min0);
	tVector center1 = 0.5 * (aabb_max1 + aabb_min1);
	tVector size0 = aabb_max0 - aabb_min0;
	tVector size1 = aabb_max1 - aabb_min1;
	tVector test_len = 0.5 * (size0 + size1);
	tVector delta = center1 - center0;
	bool overlap = (std::abs(delta[0]) <= test_len[0]) && (std::abs(delta[1]) <= test_len[1]) && (std::abs(delta[2]) <= test_len[2]);
	return overlap;
}

bool cMathUtil::IntersectAABBXZ(const tVector& aabb_min0, const tVector& aabb_max0, const tVector& aabb_min1, const tVector& aabb_max1)
{
	tVector center0 = 0.5 * (aabb_max0 + aabb_min0);
	tVector center1 = 0.5 * (aabb_max1 + aabb_min1);
	tVector size0 = aabb_max0 - aabb_min0;
	tVector size1 = aabb_max1 - aabb_min1;
	tVector test_len = 0.5 * (size0 + size1);
	tVector delta = center1 - center0;
	bool overlap = (std::abs(delta[0]) <= test_len[0]) && (std::abs(delta[2]) <= test_len[2]);
	return overlap;
}

bool cMathUtil::CheckNextInterval(double delta, double curr_val, double int_size)
{
	double pad = 0.001 * delta;
	int curr_count = static_cast<int>(std::floor((curr_val + pad) / int_size));
	int prev_count = static_cast<int>(std::floor((curr_val + pad - delta) / int_size));
	bool new_action = (curr_count != prev_count);
	return new_action;
}



void cMathUtil::QuatSwingTwistDecomposition(const tQuaternion& q, const tVector& dir, tQuaternion& out_swing, tQuaternion& out_twist)
{
	assert(std::abs(dir.norm() - 1) < 0.000001);
	assert(std::abs(q.norm() - 1) < 0.000001);

	tVector q_axis = tVector(q.x(), q.y(), q.z(), 0);
	double p = q_axis.dot(dir);
	tVector twist_axis = p * dir;
	out_twist = tQuaternion(q.w(), twist_axis[0], twist_axis[1], twist_axis[2]);
	out_twist.normalize();
	out_swing = q * out_twist.conjugate();
}

tQuaternion cMathUtil::ProjectQuat(const tQuaternion& q, const tVector& dir)
{
	assert(std::abs(dir.norm() - 1) < 0.00001);
	tVector ref_axis = tVector::Zero();
	int min_idx = 0;
	dir.cwiseAbs().minCoeff(&min_idx);
	ref_axis[min_idx] = 1;

	tVector rot_dir0 = dir.cross3(ref_axis);
	tVector rot_dir1 = cMathUtil::QuatRotVec(q, rot_dir0);
	rot_dir1 -= rot_dir1.dot(dir) * dir;

	double dir1_norm = rot_dir1.norm();
	tQuaternion p_rot = tQuaternion::Identity();
	if (dir1_norm > 0.0001)
	{
		rot_dir1 /= dir1_norm;
		p_rot = cMathUtil::VecDiffQuat(rot_dir0, rot_dir1);
	}
	return p_rot;
}

void cMathUtil::ButterworthFilter(double dt, double cutoff, Eigen::VectorXd& out_x)
{
	double sampling_rate = 1 / dt;
	int n = static_cast<int>(out_x.size());

	double wc = std::tan(cutoff * M_PI / sampling_rate);
	double k1 = std::sqrt(2.0) * wc;
	double k2 = wc * wc;
	double a = k2 / (1 + k1 + k2);
	double b = 2 * a;
	double c = a;
	double k3 = b / k2;
	double d = -2 * a + k3;
	double e = 1 - (2 * a) - k3;

	double xm2 = out_x[0];
	double xm1 = out_x[0];
	double ym2 = out_x[0];
	double ym1 = out_x[0];

	for (int s = 0; s < n; ++s) 
	{
		double x = out_x[s];
		double y = a * x + b * xm1 + c * xm2 + d * ym1 + e * ym2;

		out_x[s] = y;
		xm2 = xm1;
		xm1 = x;
		ym2 = ym1;
		ym1 = y;
	}

	double yp2 = out_x[n - 1];
	double yp1 = out_x[n - 1];
	double zp2 = out_x[n - 1];
	double zp1 = out_x[n - 1];

	for (int t = n - 1; t >= 0; --t) 
	{
		double y = out_x[t];
		double z = a * y + b * yp1 + c * yp2 + d * zp1 + e * zp2;

		out_x[t] = z;
		yp2 = yp1;
		yp1 = y;
		zp2 = zp1;
		zp1 = z;
	}
}