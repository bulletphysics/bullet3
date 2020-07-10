#pragma once

#include "MathUtil.h"

#define gSpVecSize 6
#define gSVTransRows 3
#define gSVTransCols 4


// spatial algebra util
class cSpAlg
{
public:
	

	typedef Eigen::Matrix<double, gSVTransRows, gSVTransCols> tSpTrans;
	typedef Eigen::Matrix<double, gSpVecSize, 1> tSpVec;
	typedef Eigen::Matrix<double, gSpVecSize, gSpVecSize> tSpMat;

	static tSpVec ConvertCoordM(const tSpVec& m0, const tVector& origin0, const tVector& origin1);
	// rows of R shoold be the basis for the coordinate frame centered ar origin
	static tSpVec ConvertCoordM(const tSpVec& m0, const tVector& origin0, const tMatrix& R0, const tVector& origin1, const tMatrix& R1);
	// R is a rotation from coord frame 0 to coord frame 1
	static tSpVec ConvertCoordM(const tSpVec& m0, const tVector& origin0, const tVector& origin1, const tMatrix& R);

	static tSpVec ConvertCoordF(const tSpVec& m0, const tVector& origin0, const tVector& origin1);
	static tSpVec ConvertCoordF(const tSpVec& m0, const tVector& origin0, const tMatrix& R0, const tVector& origin1, const tMatrix& R1);
	static tSpVec ConvertCoordF(const tSpVec& m0, const tVector& origin0, const tVector& origin1, const tMatrix& R);

	static tSpVec CrossM(const tSpVec& sv, const tSpVec& m);
	static Eigen::MatrixXd CrossMs(const tSpVec& sv, const Eigen::MatrixXd& ms);
	static tSpVec CrossF(const tSpVec& sv, const tSpVec& f);
	static Eigen::MatrixXd CrossFs(const tSpVec& sv, const Eigen::MatrixXd& fs);

	// Spatial Vector methods
	static tSpVec BuildSV(const tVector& v);
	static tSpVec BuildSV(const tVector& o, const tVector& v);
	static tVector GetOmega(const tSpVec& sv);
	static void SetOmega(const tVector& o, tSpVec& out_sv);
	static tVector GetV(const tSpVec& sv);
	static void SetV(const tVector& v, tSpVec& out_sv);

	// tSpTrans methods
	static tSpTrans BuildTrans();
	static tSpTrans BuildTrans(const tMatrix& E, const tVector& r);
	static tSpTrans BuildTrans(const tVector& r);

	static tSpTrans MatToTrans(const tMatrix& mat);
	static tMatrix TransToMat(const tSpTrans& X);
	static tSpMat BuildSpatialMatM(const tSpTrans& X);
	static tSpMat BuildSpatialMatF(const tSpTrans& X);
	static tSpTrans InvTrans(const tSpTrans& X);
	static tMatrix GetRot(const tSpTrans& X);
	static void SetRot(const tMatrix& E, tSpTrans& out_X);
	static tVector GetRad(const tSpTrans& X);
	static void SetRad(const tVector& r, tSpTrans& out_X);

	static tSpVec ApplyTransM(const tSpTrans& X, const tSpVec& sv);
	static tSpVec ApplyTransF(const tSpTrans& X, const tSpVec& sv);
	static Eigen::MatrixXd ApplyTransM(const tSpTrans& X, const Eigen::MatrixXd& sm);
	static Eigen::MatrixXd ApplyTransF(const tSpTrans& X, const Eigen::MatrixXd& sm);
	static tSpVec ApplyInvTransM(const tSpTrans& X, const tSpVec& sv);
	static tSpVec ApplyInvTransF(const tSpTrans& X, const tSpVec& sv);
	static Eigen::MatrixXd ApplyInvTransM(const tSpTrans& X, const Eigen::MatrixXd& sm);
	static Eigen::MatrixXd ApplyInvTransF(const tSpTrans& X, const Eigen::MatrixXd& sm);
	static tSpTrans CompTrans(const tSpTrans& X0, const tSpTrans& X1);
	
	// extract a tSpTrans from a matrix presentating a stack of transforms
	static tSpTrans GetTrans(const Eigen::MatrixXd& trans_arr, int j);
};
