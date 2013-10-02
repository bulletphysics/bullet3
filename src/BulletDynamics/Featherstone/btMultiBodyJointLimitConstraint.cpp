
#include "btMultiBodyJointLimitConstraint.h"
#include "btMultiBody.h"
#include "btMultiBodyLinkCollider.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

btMultiBodyJointLimitConstraint::btMultiBodyJointLimitConstraint(btMultiBody* body, int link, btScalar lower, btScalar upper)
	:btMultiBodyConstraint(body,body,link,link,2),
	m_lowerBound(lower),
	m_upperBound(upper)
{
	// the jacobians never change, so may as well
    // initialize them here
        
    // note: we rely on the fact that jacobians are
    // always initialized to zero by the Constraint ctor

    // row 0: the lower bound
    jacobianA(0)[6 + link] = 1;

    // row 1: the upper bound
    jacobianB(1)[6 + link] = -1;
}
btMultiBodyJointLimitConstraint::~btMultiBodyJointLimitConstraint()
{
}

int btMultiBodyJointLimitConstraint::getIslandIdA() const
{
	return m_bodyA->getLinkCollider(0)->getIslandTag();
}

int btMultiBodyJointLimitConstraint::getIslandIdB() const
{
	return m_bodyB->getLinkCollider(0)->getIslandTag();
}

void btMultiBodyJointLimitConstraint::update()
{
    // only positions need to be updated -- jacobians and force
    // directions were set in the ctor and never change.
    
    // row 0: the lower bound
    setPosition(0, m_bodyA->getJointPos(m_linkA) - m_lowerBound);

    // row 1: the upper bound
    setPosition(1, m_upperBound - m_bodyA->getJointPos(m_linkA));
}