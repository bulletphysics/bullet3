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

#include <math.h>

#include "LinearR3.h"
#include "MathMisc.h"
#include "Node.h"

extern int RotAxesOn;

Node::Node(const VectorR3& attach, const VectorR3& v, double size, Purpose purpose, double minTheta, double maxTheta, double restAngle)
{
	Node::freezed = false;
	Node::size = size;
	Node::purpose = purpose;
	seqNumJoint = -1;
	seqNumEffector = -1;
	Node::attach = attach;  // Global attachment point when joints are at zero angle
	r.Set(0.0, 0.0, 0.0);   // r will be updated when this node is inserted into tree
	Node::v = v;            // Rotation axis when joints at zero angles
	theta = 0.0;
	Node::minTheta = minTheta;
	Node::maxTheta = maxTheta;
	Node::restAngle = restAngle;
	left = right = realparent = 0;
}

// Compute the global position of a single node
void Node::ComputeS(void)
{
	Node* y = this->realparent;
	Node* w = this;
	s = r;  // Initialize to local (relative) position
	while (y)
	{
		s.Rotate(y->theta, y->v);
		y = y->realparent;
		w = w->realparent;
		s += w->r;
	}
}

// Compute the global rotation axis of a single node
void Node::ComputeW(void)
{
	Node* y = this->realparent;
	w = v;  // Initialize to local rotation axis
	while (y)
	{
		w.Rotate(y->theta, y->v);
		y = y->realparent;
	}
}

void Node::PrintNode()
{
	cerr << "Attach : (" << attach << ")\n";
	cerr << "r : (" << r << ")\n";
	cerr << "s : (" << s << ")\n";
	cerr << "w : (" << w << ")\n";
	cerr << "realparent : " << realparent->seqNumJoint << "\n";
}

void Node::InitNode()
{
	theta = 0.0;
}