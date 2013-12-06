/*
  FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
  Copyright (C) 2009. Rama Hoetzlein, http://www.rchoetzlein.com

  ZLib license
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#include "vector.h"
#include "matrix.h"

Vector3DF &Vector3DF::operator*= (const MatrixF &op)
{
	double *m = op.GetDataF ();
	double xa, ya, za;
	xa = x * (*m++);	ya = x * (*m++);	za = x * (*m++);	m++;
	xa += y * (*m++);	ya += y * (*m++);	za += y * (*m++);	m++;
	xa += z * (*m++);	ya += z * (*m++);	za += z * (*m++);	m++;
	xa += (*m++);		ya += (*m++);		za += (*m++);
	x = (float) xa; y = (float) ya; z = (float) za;
	return *this;
}

Vector3DF &Vector3DF::operator*= (const Matrix4F &op)
{
	float xa, ya, za;
	xa = x * op.data[0] + y * op.data[4] + z * op.data[8] + op.data[12];
	ya = x * op.data[1] + y * op.data[5] + z * op.data[9] + op.data[13];
	za = x * op.data[2] + y * op.data[6] + z * op.data[10] + op.data[14];
	x = xa; y = ya; z = za;
	return *this;
}

Vector4DF &Vector4DF::operator*= (const MatrixF &op)
{
	double *m = op.GetDataF ();
	double xa, ya, za, wa;
	xa = x * (*m++);	ya = x * (*m++);	za = x * (*m++);	wa = x * (*m++);
	xa += y * (*m++);	ya += y * (*m++);	za += y * (*m++);	wa += y * (*m++);
	xa += z * (*m++);	ya += z * (*m++);	za += z * (*m++);	wa += z * (*m++);
	xa += w * (*m++);	ya += w * (*m++);	za += w * (*m++);	wa += w * (*m++);
	x = xa; y = ya; z = za; w = wa;
	return *this;
}

Vector4DF &Vector4DF::operator*= (const Matrix4F &op)
{
	double xa, ya, za, wa;
	xa = x * op.data[0] + y * op.data[4] + z * op.data[8] + w * op.data[12];
	ya = x * op.data[1] + y * op.data[5] + z * op.data[9] + w * op.data[13];
	za = x * op.data[2] + y * op.data[6] + z * op.data[10] + w * op.data[14];
	wa = x * op.data[3] + y * op.data[7] + z * op.data[11] + w * op.data[15];
	x = xa; y = ya; z = za; w = wa;
	return *this;
}

