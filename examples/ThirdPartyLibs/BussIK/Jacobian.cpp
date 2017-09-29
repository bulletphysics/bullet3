/*
*
* Inverse Kinematics software, with several solvers including
* Selectively Damped Least Squares Method
* Damped Least Squares Method
* Pure Pseudoinverse Method
* Jacobian Transpose Method
*
*
* Author: Samuel R. Buss, sbuss@ucsd.edu.
* Web page: http://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/index.html
*
*
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*
*
*/

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <iostream>
using namespace std;


#include "Jacobian.h"

void Arrow(const VectorR3& tail, const VectorR3& head);

//extern RestPositionOn;
//extern VectorR3 target1[];

// Optimal damping values have to be determined in an ad hoc manner  (Yuck!)
const double Jacobian::DefaultDampingLambda = 0.6;		// Optimal for the "Y" shape (any lower gives jitter)
//const double Jacobian::DefaultDampingLambda = 1.1;			// Optimal for the DLS "double Y" shape (any lower gives jitter)
// const double Jacobian::DefaultDampingLambda = 0.7;			// Optimal for the DLS "double Y" shape with distance clamping (lower gives jitter)

const double Jacobian::PseudoInverseThresholdFactor = 0.01;
const double Jacobian::MaxAngleJtranspose = 30.0*DegreesToRadians;
const double Jacobian::MaxAnglePseudoinverse = 5.0*DegreesToRadians;
const double Jacobian::MaxAngleDLS = 45.0*DegreesToRadians;
const double Jacobian::MaxAngleSDLS = 45.0*DegreesToRadians;
const double Jacobian::BaseMaxTargetDist = 0.4;

Jacobian::Jacobian(Tree* tree)
{
	m_tree = tree;
	m_nEffector = tree->GetNumEffector();
	nJoint = tree->GetNumJoint();
	nRow = 3 * m_nEffector; // Include only the linear part

	nCol = nJoint;

	Jend.SetSize(nRow, nCol);				// The Jocobian matrix
	Jend.SetZero();
	Jtarget.SetSize(nRow, nCol);			// The Jacobian matrix based on target positions
	Jtarget.SetZero();
	SetJendActive();

	U.SetSize(nRow, nRow);				// The U matrix for SVD calculations
	w .SetLength(Min(nRow, nCol));
	V.SetSize(nCol, nCol);				// The V matrix for SVD calculations

	dS.SetLength(nRow);			// (Target positions) - (End effector positions)
	dTheta.SetLength(nCol);		// Changes in joint angles
	dPreTheta.SetLength(nCol);

	// Used by Jacobian transpose method & DLS & SDLS
	dT1.SetLength(nRow);			// Linearized change in end effector positions based on dTheta

	// Used by the Selectively Damped Least Squares Method
	//dT.SetLength(nRow);
	dSclamp.SetLength(m_nEffector);
	errorArray.SetLength(m_nEffector);
	Jnorms.SetSize(m_nEffector, nCol);		// Holds the norms of the active J matrix

	Reset();
}


Jacobian::Jacobian(bool useAngularJacobian,int nDof)
{
	
	m_tree = 0;
	m_nEffector = 1;

	if (useAngularJacobian)
	{
		nRow = 2 * 3 * m_nEffector; // Include both linear and angular part
	} else
	{
		nRow = 3 * m_nEffector; // Include only the linear part
	}

	nCol = nDof;

	Jend.SetSize(nRow, nCol);				// The Jocobian matrix
	Jend.SetZero();
	Jtarget.SetSize(nRow, nCol);			// The Jacobian matrix based on target positions
	Jtarget.SetZero();
	SetJendActive();

	U.SetSize(nRow, nRow);				// The U matrix for SVD calculations
	w .SetLength(Min(nRow, nCol));
	V.SetSize(nCol, nCol);				// The V matrix for SVD calculations

	dS.SetLength(nRow);			// (Target positions) - (End effector positions)
	dTheta.SetLength(nCol);		// Changes in joint angles
	dPreTheta.SetLength(nCol);

	// Used by Jacobian transpose method & DLS & SDLS
	dT1.SetLength(nRow);			// Linearized change in end effector positions based on dTheta

	// Used by the Selectively Damped Least Squares Method
	//dT.SetLength(nRow);
	dSclamp.SetLength(m_nEffector);
	errorArray.SetLength(m_nEffector);
	Jnorms.SetSize(m_nEffector, nCol);		// Holds the norms of the active J matrix

	Reset();
}

void Jacobian::Reset() 
{
	// Used by Damped Least Squares Method
	DampingLambda = DefaultDampingLambda;
	DampingLambdaSq = Square(DampingLambda);
	// DampingLambdaSDLS = 1.5*DefaultDampingLambda;
	
	dSclamp.Fill(HUGE_VAL);
}

// Compute the deltaS vector, dS, (the error in end effector positions
// Compute the J and K matrices (the Jacobians)
void Jacobian::ComputeJacobian(VectorR3* targets)
{
	if (m_tree==0)
		return;

	// Traverse tree to find all end effectors
	VectorR3 temp;
	Node* n = m_tree->GetRoot();
	while ( n ) {	
		if ( n->IsEffector() ) {
			int i = n->GetEffectorNum();
			const VectorR3& targetPos = targets[i];

			// Compute the delta S value (differences from end effectors to target positions.
			temp = targetPos;
			temp -= n->GetS();
			dS.SetTriple(i, temp);

			// Find all ancestors (they will usually all be joints)
			// Set the corresponding entries in the Jacobians J, K.
			Node* m = m_tree->GetParent(n);
			while ( m ) {
				int j = m->GetJointNum();
				assert ( 0 <=i && i<m_nEffector && 0<=j && j<nJoint );
				if ( m->IsFrozen() ) {
					Jend.SetTriple(i, j, VectorR3::Zero);
					Jtarget.SetTriple(i, j, VectorR3::Zero);
				}
				else {
					temp = m->GetS();			// joint pos.
					temp -= n->GetS();			// -(end effector pos. - joint pos.)
					temp *= m->GetW();			// cross product with joint rotation axis
					Jend.SetTriple(i, j, temp);	
					temp = m->GetS();			// joint pos.
					temp -= targetPos;		// -(target pos. - joint pos.)
					temp *= m->GetW();			// cross product with joint rotation axis
					Jtarget.SetTriple(i, j, temp);	
				}
				m = m_tree->GetParent( m );
			}
		}
		n = m_tree->GetSuccessor( n );
	}
}

void Jacobian::SetJendTrans(MatrixRmn& J)
{
    Jend.SetSize(J.GetNumRows(), J.GetNumColumns());
    Jend.LoadAsSubmatrix(J);
}

void Jacobian::SetDeltaS(VectorRn& S)
{
    dS.Set(S);
}

// The delta theta values have been computed in dTheta array
// Apply the delta theta values to the joints
// Nothing is done about joint limits for now.
void Jacobian::UpdateThetas() 
{
	// Traverse the tree to find all joints
	// Update the joint angles
	Node* n = m_tree->GetRoot();
	while ( n ) {	
		if ( n->IsJoint() ) {
			int i = n->GetJointNum();
			n->AddToTheta( dTheta[i] );

		}
		n = m_tree->GetSuccessor( n );
	}
	
	// Update the positions and rotation axes of all joints/effectors
	m_tree->Compute();
}

void Jacobian::UpdateThetaDot()
{
	if (m_tree==0)
		return;

    // Traverse the tree to find all joints
    // Update the joint angles
    Node* n = m_tree->GetRoot();
    while ( n ) {
        if ( n->IsJoint() ) {
            int i = n->GetJointNum();
            n->UpdateTheta( dTheta[i] );
            
        }
        n = m_tree->GetSuccessor( n );
    }
    
    // Update the positions and rotation axes of all joints/effectors
    m_tree->Compute();
}

void Jacobian::CalcDeltaThetas() 
{
	switch (CurrentUpdateMode) {
		case JACOB_Undefined:
			ZeroDeltaThetas();
			break;
		case JACOB_JacobianTranspose:
			CalcDeltaThetasTranspose();
			break;
		case JACOB_PseudoInverse:
			CalcDeltaThetasPseudoinverse();
			break;
		case JACOB_DLS:
			CalcDeltaThetasDLS();
			break;
		case JACOB_SDLS:
			CalcDeltaThetasSDLS();
			break;
	}
}

void Jacobian::ZeroDeltaThetas()
{
	dTheta.SetZero();
}

// Find the delta theta values using inverse Jacobian method
// Uses a greedy method to decide scaling factor
void Jacobian::CalcDeltaThetasTranspose()
{
	const MatrixRmn& J = ActiveJacobian();

	J.MultiplyTranspose( dS, dTheta );

	// Scale back the dTheta values greedily 
	J.Multiply ( dTheta, dT1 );						// dT = J * dTheta
	double alpha = Dot(dS,dT1) / dT1.NormSq();
	assert ( alpha>0.0 );
	// Also scale back to be have max angle change less than MaxAngleJtranspose
	double maxChange = dTheta.MaxAbs();
	double beta = MaxAngleJtranspose/maxChange;
	dTheta *= Min(alpha, beta);

}

void Jacobian::CalcDeltaThetasPseudoinverse() 
{	
	MatrixRmn& J = const_cast<MatrixRmn&>(ActiveJacobian());

	// Compute Singular Value Decomposition 
	//	This an inefficient way to do Pseudoinverse, but it is convenient since we need SVD anyway

	J.ComputeSVD( U, w, V );
	
	// Next line for debugging only
    assert(J.DebugCheckSVD(U, w , V));

	// Calculate response vector dTheta that is the DLS solution.
	//	Delta target values are the dS values
	//  We multiply by Moore-Penrose pseudo-inverse of the J matrix
	double pseudoInverseThreshold = PseudoInverseThresholdFactor*w.MaxAbs();

	long diagLength = w.GetLength();
	double* wPtr = w.GetPtr();
	dTheta.SetZero();
	for ( long i=0; i<diagLength; i++ ) {		
		double dotProdCol = U.DotProductColumn( dS, i );		// Dot product with i-th column of U
		double alpha = *(wPtr++);
		if ( fabs(alpha)>pseudoInverseThreshold ) {
			alpha = 1.0/alpha;
			MatrixRmn::AddArrayScale(V.GetNumRows(), V.GetColumnPtr(i), 1, dTheta.GetPtr(), 1, dotProdCol*alpha );
		}
	}

	// Scale back to not exceed maximum angle changes
	double maxChange = dTheta.MaxAbs();
	if ( maxChange>MaxAnglePseudoinverse ) {
		dTheta *= MaxAnglePseudoinverse/maxChange;
	}

}

void Jacobian::CalcDeltaThetasDLSwithNullspace(const VectorRn& desiredV)
{	
	const MatrixRmn& J = ActiveJacobian();

	MatrixRmn::MultiplyTranspose(J, J, U);		// U = J * (J^T)
	U.AddToDiagonal( DampingLambdaSq );
	
	// Use the next four lines instead of the succeeding two lines for the DLS method with clamped error vector e.
	// CalcdTClampedFromdS();
	// VectorRn dTextra(3*m_nEffector);
	// U.Solve( dT, &dTextra );
	// J.MultiplyTranspose( dTextra, dTheta );
	
	// Use these two lines for the traditional DLS method
	U.Solve( dS, &dT1 );
	J.MultiplyTranspose( dT1, dTheta );
    
    // Compute JInv in damped least square form
    MatrixRmn UInv(U.GetNumRows(),U.GetNumColumns());
    U.ComputeInverse(UInv);
    assert(U.DebugCheckInverse(UInv));
    MatrixRmn JInv(J.GetNumColumns(), J.GetNumRows());
    MatrixRmn::TransposeMultiply(J, UInv, JInv);
    
    // Compute null space projection
    MatrixRmn JInvJ(J.GetNumColumns(), J.GetNumColumns());
    MatrixRmn::Multiply(JInv, J, JInvJ);
    MatrixRmn P(J.GetNumColumns(), J.GetNumColumns());
    P.SetIdentity();
    P -= JInvJ;
    
    // Compute null space velocity
    VectorRn nullV(J.GetNumColumns());
    P.Multiply(desiredV, nullV);
	
	// Compute residual
	VectorRn residual(J.GetNumRows());
	J.Multiply(nullV, residual);
	// TODO: Use residual to set the null space term coefficient adaptively.
	//printf("residual: %f\n", residual.Norm());
    
    // Add null space velocity
    dTheta += nullV;
    
	// Scale back to not exceed maximum angle changes
	double maxChange = dTheta.MaxAbs();
	if ( maxChange>MaxAngleDLS ) {
		dTheta *= MaxAngleDLS/maxChange;
	}
}

void Jacobian::CalcDeltaThetasDLS()
{
    const MatrixRmn& J = ActiveJacobian();
    
    MatrixRmn::MultiplyTranspose(J, J, U);		// U = J * (J^T)
    U.AddToDiagonal( DampingLambdaSq );
    
    // Use the next four lines instead of the succeeding two lines for the DLS method with clamped error vector e.
    // CalcdTClampedFromdS();
    // VectorRn dTextra(3*m_nEffector);
    // U.Solve( dT, &dTextra );
    // J.MultiplyTranspose( dTextra, dTheta );
    
    // Use these two lines for the traditional DLS method
    U.Solve( dS, &dT1 );
    J.MultiplyTranspose( dT1, dTheta );
    
    // Scale back to not exceed maximum angle changes
    double maxChange = dTheta.MaxAbs();
    if ( maxChange>MaxAngleDLS ) {
        dTheta *= MaxAngleDLS/maxChange;
    }
}

void Jacobian::CalcDeltaThetasDLS2(const VectorRn& dVec)
{
    const MatrixRmn& J = ActiveJacobian();
    
    U.SetSize(J.GetNumColumns(), J.GetNumColumns());
    MatrixRmn::TransposeMultiply(J, J, U);
    U.AddToDiagonal( dVec );
    
    dT1.SetLength(J.GetNumColumns());
    J.MultiplyTranspose(dS, dT1);
    U.Solve(dT1, &dTheta);
    
    // Scale back to not exceed maximum angle changes
    double maxChange = dTheta.MaxAbs();
    if ( maxChange>MaxAngleDLS ) {
        dTheta *= MaxAngleDLS/maxChange;
    }
}


void Jacobian::CalcDeltaThetasDLSwithSVD() 
{	
	const MatrixRmn& J = ActiveJacobian();

	// Compute Singular Value Decomposition 
	//	This an inefficient way to do DLS, but it is convenient since we need SVD anyway

	J.ComputeSVD( U, w, V );
	
	// Next line for debugging only
    assert(J.DebugCheckSVD(U, w , V));

	// Calculate response vector dTheta that is the DLS solution.
	//	Delta target values are the dS values
	//  We multiply by DLS inverse of the J matrix
	long diagLength = w.GetLength();
	double* wPtr = w.GetPtr();
	dTheta.SetZero();
	for ( long i=0; i<diagLength; i++ ) {		
		double dotProdCol = U.DotProductColumn( dS, i );		// Dot product with i-th column of U
		double alpha = *(wPtr++);
		alpha = alpha/(Square(alpha)+DampingLambdaSq);
		MatrixRmn::AddArrayScale(V.GetNumRows(), V.GetColumnPtr(i), 1, dTheta.GetPtr(), 1, dotProdCol*alpha );
	}

	// Scale back to not exceed maximum angle changes
	double maxChange = dTheta.MaxAbs();
	if ( maxChange>MaxAngleDLS ) {
		dTheta *= MaxAngleDLS/maxChange;
	}
}


void Jacobian::CalcDeltaThetasSDLS() 
{	
	const MatrixRmn& J = ActiveJacobian();

	// Compute Singular Value Decomposition 

	J.ComputeSVD( U, w, V );

	// Next line for debugging only
    assert(J.DebugCheckSVD(U, w , V));

	// Calculate response vector dTheta that is the SDLS solution.
	//	Delta target values are the dS values
	int nRows = J.GetNumRows();
	int numEndEffectors = m_tree->GetNumEffector();		// Equals the number of rows of J divided by three
	int nCols = J.GetNumColumns();
	dTheta.SetZero();

	// Calculate the norms of the 3-vectors in the Jacobian
	long i;
	const double *jx = J.GetPtr();
	double *jnx = Jnorms.GetPtr();
	for ( i=nCols*numEndEffectors; i>0; i-- ) {
		double accumSq = Square(*(jx++));
		accumSq += Square(*(jx++));
		accumSq += Square(*(jx++));
		*(jnx++) = sqrt(accumSq);
	}

	// Clamp the dS values
	CalcdTClampedFromdS();

	// Loop over each singular vector
	for ( i=0; i<nRows; i++ ) {

		double wiInv = w[i];
		if ( NearZero(wiInv,1.0e-10) ) {
			continue;
		}
		wiInv = 1.0/wiInv;

		double N = 0.0;						// N is the quasi-1-norm of the i-th column of U
		double alpha = 0.0;					// alpha is the dot product of dT and the i-th column of U

		const double *dTx = dT1.GetPtr();
		const double *ux = U.GetColumnPtr(i);
		long j;
		for ( j=numEndEffectors; j>0; j-- ) {
			double tmp;
			alpha += (*ux)*(*(dTx++));
			tmp = Square( *(ux++) );
			alpha += (*ux)*(*(dTx++));
			tmp += Square(*(ux++));
			alpha += (*ux)*(*(dTx++));
			tmp += Square(*(ux++));
			N += sqrt(tmp);
		}

		// M is the quasi-1-norm of the response to angles changing according to the i-th column of V
		//		Then is multiplied by the wiInv value.
		double M = 0.0;
		double *vx = V.GetColumnPtr(i);
		jnx = Jnorms.GetPtr();
		for ( j=nCols; j>0; j-- ) {
			double accum=0.0;
			for ( long k=numEndEffectors; k>0; k-- ) {
				accum += *(jnx++);
			}
			M += fabs((*(vx++)))*accum;
		}
		M *= fabs(wiInv);
	
		double gamma = MaxAngleSDLS;
		if ( N<M ) {
			gamma *= N/M;				// Scale back maximum permissable joint angle
		}

		// Calculate the dTheta from pure pseudoinverse considerations
		double scale = alpha*wiInv;			// This times i-th column of V is the psuedoinverse response
		dPreTheta.LoadScaled( V.GetColumnPtr(i), scale );
		// Now rescale the dTheta values.
		double max = dPreTheta.MaxAbs();
		double rescale = (gamma)/(gamma+max);
		dTheta.AddScaled(dPreTheta,rescale);
		/*if ( gamma<max) {
			dTheta.AddScaled( dPreTheta, gamma/max );
		}
		else {
			dTheta += dPreTheta;
		}*/
	}

	// Scale back to not exceed maximum angle changes
	double maxChange = dTheta.MaxAbs();
	if ( maxChange>MaxAngleSDLS ) {
		dTheta *= MaxAngleSDLS/(MaxAngleSDLS+maxChange);
		//dTheta *= MaxAngleSDLS/maxChange;
	}
}

void Jacobian::CalcdTClampedFromdS() 
{
	long len = dS.GetLength();
	long j = 0;
	for ( long i=0; i<len; i+=3, j++ ) {
		double normSq = Square(dS[i])+Square(dS[i+1])+Square(dS[i+2]);
		if ( normSq>Square(dSclamp[j]) ) {
			double factor = dSclamp[j]/sqrt(normSq);
			dT1[i] = dS[i]*factor;
			dT1[i+1] = dS[i+1]*factor;
			dT1[i+2] = dS[i+2]*factor;
		}
		else {
			dT1[i] = dS[i];
			dT1[i+1] = dS[i+1];
			dT1[i+2] = dS[i+2];
		}
	}
}

double Jacobian::UpdateErrorArray(VectorR3* targets)
{
	double totalError = 0.0;

	// Traverse tree to find all end effectors
	VectorR3 temp;
	Node* n = m_tree->GetRoot();
	while ( n ) {	
		if ( n->IsEffector() ) {
			int i = n->GetEffectorNum();
			const VectorR3& targetPos = targets[i];
			temp = targetPos;
			temp -= n->GetS();
			double err = temp.Norm();
			errorArray[i] = err;
			totalError += err;
		}
		n = m_tree->GetSuccessor( n );
	}
	return totalError;
}

void Jacobian::UpdatedSClampValue(VectorR3* targets)
{
	// Traverse tree to find all end effectors
	VectorR3 temp;
	Node* n = m_tree->GetRoot();
	while ( n ) {	
		if ( n->IsEffector() ) {
			int i = n->GetEffectorNum();
			const VectorR3& targetPos = targets[i];

			// Compute the delta S value (differences from end effectors to target positions.
			// While we are at it, also update the clamping values in dSclamp;
			temp = targetPos;
			temp -= n->GetS();
			double normSi = sqrt(Square(dS[i])+Square(dS[i+1])+Square(dS[i+2]));
			double changedDist = temp.Norm()-normSi;
			if ( changedDist>0.0 ) {
				dSclamp[i] = BaseMaxTargetDist + changedDist;
			}
			else {
				dSclamp[i] = BaseMaxTargetDist;
			}
		}
		n = m_tree->GetSuccessor( n );
	}
}



void Jacobian::CompareErrors( const Jacobian& j1, const Jacobian& j2, double* weightedDist1, double* weightedDist2 )
{
	const VectorRn& e1 = j1.errorArray;
	const VectorRn& e2 = j2.errorArray;
	double ret1 = 0.0;
	double ret2 = 0.0;
	int len = e1.GetLength();
	for ( long i=0; i<len; i++ ) {
		double v1 = e1[i];
		double v2 = e2[i];
		if ( v1<v2 ) {
			ret1 += v1/v2;
			ret2 += 1.0;
		}
		else if ( v1 != 0.0 ) {
			ret1 += 1.0;
			ret2 += v2/v1;
		}
		else {
			ret1 += 0.0;
			ret2 += 0.0;
		}
	}
	*weightedDist1 = ret1;
	*weightedDist2 = ret2;
}

void Jacobian::CountErrors( const Jacobian& j1, const Jacobian& j2, int* numBetter1, int* numBetter2, int* numTies )
{
	const VectorRn& e1 = j1.errorArray;
	const VectorRn& e2 = j2.errorArray;
	int b1=0, b2=0, tie=0;
	int len = e1.GetLength();
	for ( long i=0; i<len; i++ ) {
		double v1 = e1[i];
		double v2 = e2[i];
		if ( v1<v2 ) {
			b1++;
		}
		else if ( v2<v1 ) {
			b2++;
		}
		else {
			tie++;
		}
	}
	*numBetter1 = b1;
	*numBetter2 = b2;
	*numTies = tie;
}

/* THIS VERSION IS NOT AS GOOD.  DO NOT USE!
void Jacobian::CalcDeltaThetasSDLSrev2() 
{	
	const MatrixRmn& J = ActiveJacobian();

	// Compute Singular Value Decomposition 

	J.ComputeSVD( U, w, V );
	
	// Next line for debugging only
    assert(J.DebugCheckSVD(U, w , V));

	// Calculate response vector dTheta that is the SDLS solution.
	//	Delta target values are the dS values
	int nRows = J.GetNumRows();
	int numEndEffectors = tree->GetNumEffector();		// Equals the number of rows of J divided by three
	int nCols = J.GetNumColumns();
	dTheta.SetZero();

	// Calculate the norms of the 3-vectors in the Jacobian
	long i;
	const double *jx = J.GetPtr();
	double *jnx = Jnorms.GetPtr();
	for ( i=nCols*numEndEffectors; i>0; i-- ) {
		double accumSq = Square(*(jx++));
		accumSq += Square(*(jx++));
		accumSq += Square(*(jx++));
		*(jnx++) = sqrt(accumSq);
	}

	// Loop over each singular vector
	for ( i=0; i<nRows; i++ ) {

		double wiInv = w[i];
		if ( NearZero(wiInv,1.0e-10) ) {
			continue;
		}

		double N = 0.0;						// N is the quasi-1-norm of the i-th column of U
		double alpha = 0.0;					// alpha is the dot product of dS and the i-th column of U

		const double *dSx = dS.GetPtr();
		const double *ux = U.GetColumnPtr(i);
		long j;
		for ( j=numEndEffectors; j>0; j-- ) {
			double tmp;
			alpha += (*ux)*(*(dSx++));
			tmp = Square( *(ux++) );
			alpha += (*ux)*(*(dSx++));
			tmp += Square(*(ux++));
			alpha += (*ux)*(*(dSx++));
			tmp += Square(*(ux++));
			N += sqrt(tmp);
		}

		// P is the quasi-1-norm of the response to angles changing according to the i-th column of V
		double P = 0.0;
		double *vx = V.GetColumnPtr(i);
		jnx = Jnorms.GetPtr();
		for ( j=nCols; j>0; j-- ) {
			double accum=0.0;
			for ( long k=numEndEffectors; k>0; k-- ) {
				accum += *(jnx++);
			}
			P += fabs((*(vx++)))*accum;
		}
	
		double lambda = 1.0;
		if ( N<P ) {
			lambda -= N/P;				// Scale back maximum permissable joint angle
		}
		lambda *= lambda;
		lambda *= DampingLambdaSDLS;

		// Calculate the dTheta from pure pseudoinverse considerations
		double scale = alpha*wiInv/(Square(wiInv)+Square(lambda));			// This times i-th column of V is the SDLS response
		MatrixRmn::AddArrayScale(nCols, V.GetColumnPtr(i), 1, dTheta.GetPtr(), 1, scale );
	}

	// Scale back to not exceed maximum angle changes
	double maxChange = dTheta.MaxAbs();
	if ( maxChange>MaxAngleSDLS ) {
		dTheta *= MaxAngleSDLS/maxChange;
	}
} */




