/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_MULTIBODY_CONSTRAINT_H
#define BT_MULTIBODY_CONSTRAINT_H

#include "LinearMath/btScalar.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btMultiBody.h"

class btMultiBody;

class btMultiBodyConstraint
{
protected:

	btMultiBody*	m_bodyA;
    btMultiBody*	m_bodyB;
    int				m_linkA;
    int				m_linkB;

    int				m_num_rows;
    int				m_jac_size_A;
    int				m_jac_size_both;
    int				m_pos_offset;

    // data block laid out as follows:
    // cached impulses. (one per row.)
    // jacobians. (interleaved, row1 body1 then row1 body2 then row2 body 1 etc)
    // positions. (one per row.)
    btAlignedObjectArray<btScalar> m_data;

public:

	btMultiBodyConstraint(btMultiBody* bodyA,btMultiBody* bodyB,int linkA, int linkB, int numRows)
		:m_bodyA(bodyA),
		m_bodyB(bodyB),
		m_linkA(linkA),
		m_linkB(linkB),
		m_num_rows(numRows)
	{
		m_jac_size_A = (6 + bodyA->getNumLinks());
		m_jac_size_both = (m_jac_size_A + (bodyB ? 6 + bodyB->getNumLinks() : 0));
		m_pos_offset = ((1 + m_jac_size_both)*m_num_rows);
		m_data.resize((2 + m_jac_size_both) * m_num_rows);
	}
	virtual int getIslandIdA() const
	{
		return 0;
	}
	virtual int getIslandIdB() const
	{
		return 0;
	}

	virtual void update()=0;

	// current constraint position
    // constraint is pos >= 0 for unilateral, or pos = 0 for bilateral
    // NOTE: position ignored for friction rows.
    btScalar getPosition(int row) const 
	{ 
		return m_data[m_pos_offset + row]; 
	}

    void setPosition(int row, btScalar pos) 
	{ 
		m_data[m_pos_offset + row] = pos; 
	}

	// jacobian blocks.
    // each of size 6 + num_links. (jacobian2 is null if no body2.)
    // format: 3 'omega' coefficients, 3 'v' coefficients, then the 'qdot' coefficients.
    btScalar* jacobianA(int row) 
	{ 
		return &m_data[m_num_rows + row * m_jac_size_both]; 
	}
    const btScalar* jacobianA(int row) const 
	{ 
		return &m_data[m_num_rows + (row * m_jac_size_both)]; 
	}
    btScalar* jacobianB(int row) 
	{ 
		return &m_data[m_num_rows + (row * m_jac_size_both) + m_jac_size_A]; 
	}
    const btScalar* jacobianB(int row) const 
	{ 
		return &m_data[m_num_rows + (row * m_jac_size_both) + m_jac_size_A]; 
	}


};

#endif //BT_MULTIBODY_CONSTRAINT_H

