/*
*
* Mathematics Subpackage (VrMath)
*
*
* Author: Samuel R. Buss, sbuss@ucsd.edu.
* Web page: http://math.ucsd.edu/~sbuss/MathCG
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

//
// MatrixRmn:  Matrix over reals  (Variable dimensional vector)
//
//    Not very sophisticated yet.  Needs more functionality
//		To do: better handling of resizing.
//

#ifndef MATRIX_RMN_H
#define MATRIX_RMN_H

#include <math.h>
#include <assert.h>
#include "LinearR3.h"
#include "VectorRn.h"

class MatrixRmn
{
public:
	MatrixRmn();                            // Null constructor
	MatrixRmn(long numRows, long numCols);  // Constructor with length
	~MatrixRmn();                           // Destructor

	void SetSize(long numRows, long numCols);
	long GetNumRows() const { return NumRows; }
	long GetNumColumns() const { return NumCols; }
	void SetZero();

	// Return entry in row i and column j.
	double Get(long i, long j) const;
	void GetTriple(long i, long j, VectorR3* retValue) const;

	// Use GetPtr to get pointer into the array (efficient)
	// Is friendly in that anyone can change the array contents (be careful!)
	// The entries are in column order!!!
	// Use this with care.  You may call GetRowStride and GetColStride to navigate
	//			within the matrix.  I do not expect these values to ever change.
	const double* GetPtr() const;
	double* GetPtr();
	const double* GetPtr(long i, long j) const;
	double* GetPtr(long i, long j);
	const double* GetColumnPtr(long j) const;
	double* GetColumnPtr(long j);
	const double* GetRowPtr(long i) const;
	double* GetRowPtr(long i);
	long GetRowStride() const { return NumRows; }  // Step size (stride) along a row
	long GetColStride() const { return 1; }        // Step size (stide) along a column

	void Set(long i, long j, double val);
	void SetTriple(long i, long c, const VectorR3& u);

	void SetIdentity();
	void SetDiagonalEntries(double d);
	void SetDiagonalEntries(const VectorRn& d);
	void SetSuperDiagonalEntries(double d);
	void SetSuperDiagonalEntries(const VectorRn& d);
	void SetSubDiagonalEntries(double d);
	void SetSubDiagonalEntries(const VectorRn& d);
	void SetColumn(long i, const VectorRn& d);
	void SetRow(long i, const VectorRn& d);
	void SetSequence(const VectorRn& d, long startRow, long startCol, long deltaRow, long deltaCol);

	// Loads matrix in as a sub-matrix.  (i,j) is the base point. Defaults to (0,0).
	// The "Tranpose" versions load the transpose of A.
	void LoadAsSubmatrix(const MatrixRmn& A);
	void LoadAsSubmatrix(long i, long j, const MatrixRmn& A);
	void LoadAsSubmatrixTranspose(const MatrixRmn& A);
	void LoadAsSubmatrixTranspose(long i, long j, const MatrixRmn& A);

	// Norms
	double FrobeniusNormSq() const;
	double FrobeniusNorm() const;

	// Operations on VectorRn's
	void Multiply(const VectorRn& v, VectorRn& result) const;           // result = (this)*(v)
	void MultiplyTranspose(const VectorRn& v, VectorRn& result) const;  // Equivalent to mult by row vector on left
	double DotProductColumn(const VectorRn& v, long colNum) const;      // Returns dot product of v with i-th column

	// Operations on MatrixRmn's
	MatrixRmn& operator*=(double);
	MatrixRmn& operator/=(double d)
	{
		assert(d != 0.0);
		*this *= (1.0 / d);
		return *this;
	}
	MatrixRmn& AddScaled(const MatrixRmn& B, double factor);
	MatrixRmn& operator+=(const MatrixRmn& B);
	MatrixRmn& operator-=(const MatrixRmn& B);
	static MatrixRmn& Multiply(const MatrixRmn& A, const MatrixRmn& B, MatrixRmn& dst);           // Sets dst = A*B.
	static MatrixRmn& MultiplyTranspose(const MatrixRmn& A, const MatrixRmn& B, MatrixRmn& dst);  // Sets dst = A*(B-tranpose).
	static MatrixRmn& TransposeMultiply(const MatrixRmn& A, const MatrixRmn& B, MatrixRmn& dst);  // Sets dst = (A-transpose)*B.

	// Miscellaneous operation
	MatrixRmn& AddToDiagonal(double d);  // Adds d to each diagonal
	MatrixRmn& AddToDiagonal(const VectorRn& dVec);

	// Solving systems of linear equations
	void Solve(const VectorRn& b, VectorRn* x, MatrixRmn& AugMat) const;  // Solves the equation   (*this)*x = b;    Uses row operations.  Assumes *this is invertible.

	// Row Echelon Form and Reduced Row Echelon Form routines
	// Row echelon form here allows non-negative entries (instead of 1's) in the positions of lead variables.
	void ConvertToRefNoFree();                   // Converts the matrix in place to row echelon form -- assumption is no free variables will be found
	void ConvertToRef(int numVars);              // Converts the matrix in place to row echelon form -- numVars is number of columns to work with.
	void ConvertToRef(int numVars, double eps);  // Same, but eps is the measure of closeness to zero

	// Givens transformation
	static void CalcGivensValues(double a, double b, double* c, double* s);
	void PostApplyGivens(double c, double s, long idx);              // Applies Givens transform to columns idx and idx+1.
	void PostApplyGivens(double c, double s, long idx1, long idx2);  // Applies Givens transform to columns idx1 and idx2.

	// Singular value decomposition
	void ComputeSVD(MatrixRmn& U, VectorRn& w, MatrixRmn& V) const;
	// Good for debugging SVD computations (I recommend this be used for any new application to check for bugs/instability).
	bool DebugCheckSVD(const MatrixRmn& U, const VectorRn& w, const MatrixRmn& V) const;
	// Compute inverse of a matrix, the result is written in R
	void ComputeInverse(MatrixRmn& R) const;
	// Debug matrix inverse computation
	bool DebugCheckInverse(const MatrixRmn& MInv) const;

	// Some useful routines for experts who understand the inner workings of these classes.
	inline static double DotArray(long length, const double* ptrA, long strideA, const double* ptrB, long strideB);
	inline static void CopyArrayScale(long length, const double* from, long fromStride, double* to, long toStride, double scale);
	inline static void AddArrayScale(long length, const double* from, long fromStride, double* to, long toStride, double scale);

private:
	long NumRows;    // Number of rows
	long NumCols;    // Number of columns
	double* x;       // Array of vector entries - stored in column order
	long AllocSize;  // Allocated size of the x array


	// Internal helper routines for SVD calculations
	static void CalcBidiagonal(MatrixRmn& U, MatrixRmn& V, VectorRn& w, VectorRn& superDiag);
	void ConvertBidiagToDiagonal(MatrixRmn& U, MatrixRmn& V, VectorRn& w, VectorRn& superDiag) const;
	static void SvdHouseholder(double* basePt,
							   long colLength, long numCols, long colStride, long rowStride,
							   double* retFirstEntry);
	void ExpandHouseholders(long numXforms, int numZerosSkipped, const double* basePt, long colStride, long rowStride);
	static bool UpdateBidiagIndices(long* firstDiagIdx, long* lastBidiagIdx, VectorRn& w, VectorRn& superDiag, double eps);
	static void ApplyGivensCBTD(double cosine, double sine, double* a, double* b, double* c, double* d);
	static void ApplyGivensCBTD(double cosine, double sine, double* a, double* b, double* c,
								double d, double* e, double* f);
	static void ClearRowWithDiagonalZero(long firstBidiagIdx, long lastBidiagIdx,
										 MatrixRmn& U, double* wPtr, double* sdPtr, double eps);
	static void ClearColumnWithDiagonalZero(long endIdx, MatrixRmn& V, double* wPtr, double* sdPtr, double eps);
	bool DebugCalcBidiagCheck(const MatrixRmn& U, const VectorRn& w, const VectorRn& superDiag, const MatrixRmn& V) const;
};

inline MatrixRmn::MatrixRmn()
{
	NumRows = 0;
	NumCols = 0;
	x = 0;
	AllocSize = 0;
}

inline MatrixRmn::MatrixRmn(long numRows, long numCols)
{
	NumRows = 0;
	NumCols = 0;
	x = 0;
	AllocSize = 0;
	SetSize(numRows, numCols);
}

inline MatrixRmn::~MatrixRmn()
{
	delete[] x;
}

// Resize.
// If the array space is decreased, the information about the allocated length is lost.
inline void MatrixRmn::SetSize(long numRows, long numCols)
{
	assert(numRows > 0 && numCols > 0);
	long newLength = numRows * numCols;
	if (newLength > AllocSize)
	{
		delete[] x;
		AllocSize = Max(newLength, AllocSize << 1);
		x = new double[AllocSize];
	}
	NumRows = numRows;
	NumCols = numCols;
}

// Zero out the entire vector
inline void MatrixRmn::SetZero()
{
	double* target = x;
	for (long i = NumRows * NumCols; i > 0; i--)
	{
		*(target++) = 0.0;
	}
}

// Return entry in row i and column j.
inline double MatrixRmn::Get(long i, long j) const
{
	assert(i < NumRows && j < NumCols);
	return *(x + j * NumRows + i);
}

// Return a VectorR3 out of a column.  Starts at row 3*i, in column j.
inline void MatrixRmn::GetTriple(long i, long j, VectorR3* retValue) const
{
	long ii = 3 * i;
	assert(0 <= i && ii + 2 < NumRows && 0 <= j && j < NumCols);
	retValue->Load(x + j * NumRows + ii);
}

// Get a pointer to the (0,0) entry.
// The entries are in column order.
// This version gives read-only pointer
inline const double* MatrixRmn::GetPtr() const
{
	return x;
}

// Get a pointer to the (0,0) entry.
// The entries are in column order.
inline double* MatrixRmn::GetPtr()
{
	return x;
}

// Get a pointer to the (i,j) entry.
// The entries are in column order.
// This version gives read-only pointer
inline const double* MatrixRmn::GetPtr(long i, long j) const
{
	assert(0 <= i && i < NumRows && 0 <= j && j < NumCols);
	return (x + j * NumRows + i);
}

// Get a pointer to the (i,j) entry.
// The entries are in column order.
// This version gives pointer to writable data
inline double* MatrixRmn::GetPtr(long i, long j)
{
	assert(i < NumRows && j < NumCols);
	return (x + j * NumRows + i);
}

// Get a pointer to the j-th column.
// The entries are in column order.
// This version gives read-only pointer
inline const double* MatrixRmn::GetColumnPtr(long j) const
{
	assert(0 <= j && j < NumCols);
	return (x + j * NumRows);
}

// Get a pointer to the j-th column.
// This version gives pointer to writable data
inline double* MatrixRmn::GetColumnPtr(long j)
{
	assert(0 <= j && j < NumCols);
	return (x + j * NumRows);
}

/// Get a pointer to the i-th row
// The entries are in column order.
// This version gives read-only pointer
inline const double* MatrixRmn::GetRowPtr(long i) const
{
	assert(0 <= i && i < NumRows);
	return (x + i);
}

// Get a pointer to the i-th row
// This version gives pointer to writable data
inline double* MatrixRmn::GetRowPtr(long i)
{
	assert(0 <= i && i < NumRows);
	return (x + i);
}

// Set the (i,j) entry of the matrix
inline void MatrixRmn::Set(long i, long j, double val)
{
	assert(i < NumRows && j < NumCols);
	*(x + j * NumRows + i) = val;
}

// Set the i-th triple in the j-th column to u's three values
inline void MatrixRmn::SetTriple(long i, long j, const VectorR3& u)
{
	long ii = 3 * i;
	assert(0 <= i && ii + 2 < NumRows && 0 <= j && j < NumCols);
	u.Dump(x + j * NumRows + ii);
}

// Set to be equal to the identity matrix
inline void MatrixRmn::SetIdentity()
{
	assert(NumRows == NumCols);
	SetZero();
	SetDiagonalEntries(1.0);
}

inline MatrixRmn& MatrixRmn::operator*=(double mult)
{
	double* aPtr = x;
	for (long i = NumRows * NumCols; i > 0; i--)
	{
		(*(aPtr++)) *= mult;
	}
	return (*this);
}

inline MatrixRmn& MatrixRmn::AddScaled(const MatrixRmn& B, double factor)
{
	assert(NumRows == B.NumRows && NumCols == B.NumCols);
	double* aPtr = x;
	double* bPtr = B.x;
	for (long i = NumRows * NumCols; i > 0; i--)
	{
		(*(aPtr++)) += (*(bPtr++)) * factor;
	}
	return (*this);
}

inline MatrixRmn& MatrixRmn::operator+=(const MatrixRmn& B)
{
	assert(NumRows == B.NumRows && NumCols == B.NumCols);
	double* aPtr = x;
	double* bPtr = B.x;
	for (long i = NumRows * NumCols; i > 0; i--)
	{
		(*(aPtr++)) += *(bPtr++);
	}
	return (*this);
}

inline MatrixRmn& MatrixRmn::operator-=(const MatrixRmn& B)
{
	assert(NumRows == B.NumRows && NumCols == B.NumCols);
	double* aPtr = x;
	double* bPtr = B.x;
	for (long i = NumRows * NumCols; i > 0; i--)
	{
		(*(aPtr++)) -= *(bPtr++);
	}
	return (*this);
}

inline double MatrixRmn::FrobeniusNormSq() const
{
	double* aPtr = x;
	double result = 0.0;
	for (long i = NumRows * NumCols; i > 0; i--)
	{
		result += Square(*(aPtr++));
	}
	return result;
}

// Helper routine to calculate dot product
inline double MatrixRmn::DotArray(long length, const double* ptrA, long strideA, const double* ptrB, long strideB)
{
	double result = 0.0;
	for (; length > 0; length--)
	{
		result += (*ptrA) * (*ptrB);
		ptrA += strideA;
		ptrB += strideB;
	}
	return result;
}

// Helper routine: copies and scales an array (src and dest may be equal, or overlap)
inline void MatrixRmn::CopyArrayScale(long length, const double* from, long fromStride, double* to, long toStride, double scale)
{
	for (; length > 0; length--)
	{
		*to = (*from) * scale;
		from += fromStride;
		to += toStride;
	}
}

// Helper routine: adds a scaled array
//	fromArray = toArray*scale.
inline void MatrixRmn::AddArrayScale(long length, const double* from, long fromStride, double* to, long toStride, double scale)
{
	for (; length > 0; length--)
	{
		*to += (*from) * scale;
		from += fromStride;
		to += toStride;
	}
}

#endif  //MATRIX_RMN_H
