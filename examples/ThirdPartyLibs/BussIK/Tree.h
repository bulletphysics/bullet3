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

#include "LinearR3.h"
#include "Node.h"

#ifndef _CLASS_TREE
#define _CLASS_TREE

class Tree
{
public:
	Tree();

	int GetNumNode() const { return nNode; }
	int GetNumEffector() const { return nEffector; }
	int GetNumJoint() const { return nJoint; }
	void InsertRoot(Node*);
	void InsertLeftChild(Node* parent, Node* child);
	void InsertRightSibling(Node* parent, Node* child);

	// Accessors based on node numbers
	Node* GetJoint(int);
	Node* GetEffector(int);
	const VectorR3& GetEffectorPosition(int);

	// Accessors for tree traversal
	Node* GetRoot() const { return root; }
	Node* GetSuccessor(const Node*) const;
	Node* GetParent(const Node* node) const { return node->realparent; }

	void Compute();

	void Print();
	void Init();
	void UnFreeze();

private:
	Node* root;
	int nNode;  // nNode = nEffector + nJoint
	int nEffector;
	int nJoint;
	void SetSeqNum(Node*);
	Node* SearchJoint(Node*, int);
	Node* SearchEffector(Node*, int);
	void ComputeTree(Node*);

	void PrintTree(Node*);
	void InitTree(Node*);
	void UnFreezeTree(Node*);
};

inline Node* Tree::GetSuccessor(const Node* node) const
{
	if (node->left)
	{
		return node->left;
	}
	while (true)
	{
		if (node->right)
		{
			return (node->right);
		}
		node = node->realparent;
		if (!node)
		{
			return 0;  // Back to root, finished traversal
		}
	}
}

#endif