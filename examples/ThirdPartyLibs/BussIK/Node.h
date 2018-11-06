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

#ifndef _CLASS_NODE
#define _CLASS_NODE

#include "LinearR3.h"

enum Purpose
{
	JOINT,
	EFFECTOR
};

class VectorR3;

class Node
{
	friend class Tree;

public:
	Node(const VectorR3&, const VectorR3&, double, Purpose, double minTheta = -PI, double maxTheta = PI, double restAngle = 0.);

	void PrintNode();
	void InitNode();

	const VectorR3& GetAttach() const { return attach; }

	double GetTheta() const { return theta; }
	double AddToTheta(double& delta)
	{
		//double orgTheta = theta;
		theta += delta;
#if 0
		if (theta < minTheta)
			theta = minTheta;
		if (theta > maxTheta)
			theta = maxTheta;
		double actualDelta = theta - orgTheta;
		delta = actualDelta;
#endif
		return theta;
	}

	double UpdateTheta(double& delta)
	{
		theta = delta;
		return theta;
	}

	const VectorR3& GetS() const { return s; }
	const VectorR3& GetW() const { return w; }

	double GetMinTheta() const { return minTheta; }
	double GetMaxTheta() const { return maxTheta; }
	double GetRestAngle() const { return restAngle; };
	void SetTheta(double newTheta) { theta = newTheta; }
	void ComputeS(void);
	void ComputeW(void);

	bool IsEffector() const { return purpose == EFFECTOR; }
	bool IsJoint() const { return purpose == JOINT; }
	int GetEffectorNum() const { return seqNumEffector; }
	int GetJointNum() const { return seqNumJoint; }

	bool IsFrozen() const { return freezed; }
	void Freeze() { freezed = true; }
	void UnFreeze() { freezed = false; }

	//private:
	bool freezed;        // Is this node frozen?
	int seqNumJoint;     // sequence number if this node is a joint
	int seqNumEffector;  // sequence number if this node is an effector
	double size;         // size
	Purpose purpose;     // joint / effector / both
	VectorR3 attach;     // attachment point
	VectorR3 r;          // relative position vector
	VectorR3 v;          // rotation axis
	double theta;        // joint angle (radian)
	double minTheta;     // lower limit of joint angle
	double maxTheta;     // upper limit of joint angle
	double restAngle;    // rest position angle
	VectorR3 s;          // GLobal Position
	VectorR3 w;          // Global rotation axis
	Node* left;          // left child
	Node* right;         // right sibling
	Node* realparent;    // pointer to real parent
};

#endif