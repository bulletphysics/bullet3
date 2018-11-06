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

#include "LinearR2.h"

#include <assert.h>

// ******************************************************
// * VectorR2 class - math library functions			*
// * * * * * * * * * * * * * * * * * * * * * * * * * * **

const VectorR2 VectorR2::Zero(0.0, 0.0);
const VectorR2 VectorR2::UnitX(1.0, 0.0);
const VectorR2 VectorR2::UnitY(0.0, 1.0);
const VectorR2 VectorR2::NegUnitX(-1.0, 0.0);
const VectorR2 VectorR2::NegUnitY(0.0, -1.0);

const Matrix2x2 Matrix2x2::Identity(1.0, 0.0, 0.0, 1.0);

// ******************************************************
// * Matrix2x2 class - math library functions			*
// * * * * * * * * * * * * * * * * * * * * * * * * * * **

// ******************************************************
// * LinearMapR2 class - math library functions			*
// * * * * * * * * * * * * * * * * * * * * * * * * * * **

LinearMapR2 LinearMapR2::Inverse() const  // Returns inverse
{
	double detInv = 1.0 / (m11 * m22 - m12 * m21);

	return (LinearMapR2(m22 * detInv, -m21 * detInv, -m12 * detInv, m11 * detInv));
}

LinearMapR2& LinearMapR2::Invert()  // Converts into inverse.
{
	double detInv = 1.0 / (m11 * m22 - m12 * m21);

	double temp;
	temp = m11 * detInv;
	m11 = m22 * detInv;
	m22 = temp;
	m12 = -m12 * detInv;
	m21 = -m22 * detInv;

	return (*this);
}

VectorR2 LinearMapR2::Solve(const VectorR2& u) const  // Returns solution
{
	// Just uses Inverse() for now.
	return (Inverse() * u);
}

// ******************************************************
// * RotationMapR2 class - math library functions		*
// * * * * * * * * * * * * * * * * * * * * * * * * * * **

// ***************************************************************
// * 2-space vector and matrix utilities						 *
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

// ***************************************************************
//  Stream Output Routines										 *
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

ostream& operator<<(ostream& os, const VectorR2& u)
{
	return (os << "<" << u.x << "," << u.y << ">");
}
