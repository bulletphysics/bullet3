/*
CDTestFramework http://codercorner.com
Copyright (c) 2007-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "stdafx.h"
#include "Camera.h"

static const float gCamSpeed = 1.0f;
//static Point gEye(3.0616338f, 1.1985892f, 2.5769043f);
//static Point gDir(-0.66853905,-0.14004262,-0.73037237);
static Point gEye(240, 205, 205);
static Point gDir(-1,-1,-1);
static Point gN;
static float gFOV = 60.0f;

const Point& GetCameraPos()
{
	return gEye;
}

const Point& GetCameraDir()
{
	return gDir;
}

void MoveCameraForward()
{
	gEye += gDir * gCamSpeed;
}

void MoveCameraBackward()
{
	gEye -= gDir * gCamSpeed;
}

void MoveCameraRight()
{
	gEye -= gN * gCamSpeed;
}

void MoveCameraLeft()
{
	gEye += gN * gCamSpeed;
}

	static const float NxPiF32	= 3.141592653589793f;

	float degToRad(float a)
		{
		return (float)0.01745329251994329547 * a;
		}

	class NxQuat
		{
		public:
		NxQuat(){}

		NxQuat(const float angle, const Point & axis)
		{
		x = axis.x;
		y = axis.y;
		z = axis.z;

		const float i_length =  1.0f / sqrtf( x*x + y*y + z*z );
		x = x * i_length;
		y = y * i_length;
		z = z * i_length;

		float Half = degToRad(angle * 0.5f);

		w = cosf(Half);
		const float sin_theta_over_two = sinf(Half );
		x = x * sin_theta_over_two;
		y = y * sin_theta_over_two;
		z = z * sin_theta_over_two;
		}

	void NxQuat::multiply(const NxQuat& left, const Point& right)
		{
		float a,b,c,d;

		a = - left.x*right.x - left.y*right.y - left.z *right.z;
		b =   left.w*right.x + left.y*right.z - right.y*left.z;
		c =   left.w*right.y + left.z*right.x - right.z*left.x;
		d =   left.w*right.z + left.x*right.y - right.x*left.y;

		w = a;
		x = b;
		y = c;
		z = d;
		}

	void NxQuat::rotate(Point & v) const
		{
		NxQuat myInverse;
		myInverse.x = -x;
		myInverse.y = -y;
		myInverse.z = -z;
		myInverse.w =  w;

		NxQuat left;
		left.multiply(*this,v);
		v.x = left.w*myInverse.x + myInverse.w*left.x + left.y*myInverse.z - myInverse.y*left.z;
		v.y = left.w*myInverse.y + myInverse.w*left.y + left.z*myInverse.x - myInverse.z*left.x;
		v.z = left.w*myInverse.z + myInverse.w*left.z + left.x*myInverse.y - myInverse.x*left.y;
		}

		float x,y,z,w;
		};

void RotateCamera(int dx, int dy)
{
	gDir = gDir.Normalize();
	gN = gDir ^ Point(0,1,0);

	NxQuat qx(NxPiF32 * dx * 20/ 180.0f, Point(0,1,0));
	qx.rotate(gDir);
	NxQuat qy(NxPiF32 * dy * 20/ 180.0f, gN);
	qy.rotate(gDir);
}

void SetupCameraMatrix()
{
	glLoadIdentity();
	gluPerspective(gFOV, ((float)glutGet(GLUT_WINDOW_WIDTH))/((float)glutGet(GLUT_WINDOW_HEIGHT)), 1.0f, 10000.0f);
	gluLookAt(gEye.x, gEye.y, gEye.z, gEye.x + gDir.x, gEye.y + gDir.y, gEye.z + gDir.z, 0.0f, 1.0f, 0.0f);
}

Point ComputeWorldRay(int xs, int ys)
{
	GLint viewPort[4];
	GLdouble modelMatrix[16];
	GLdouble projMatrix[16];
	glGetIntegerv(GL_VIEWPORT, viewPort);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
	ys = viewPort[3] - ys - 1;
	GLdouble wx0, wy0, wz0;
	gluUnProject((GLdouble) xs, (GLdouble) ys, 0.0, modelMatrix, projMatrix, viewPort, &wx0, &wy0, &wz0);
	GLdouble wx1, wy1, wz1;
	gluUnProject((GLdouble) xs, (GLdouble) ys, 1.0, modelMatrix, projMatrix, viewPort, &wx1, &wy1, &wz1);
	Point tmp(float(wx1-wx0), float(wy1-wy0), float(wz1-wz0));
	tmp.Normalize();
	return tmp;
}
