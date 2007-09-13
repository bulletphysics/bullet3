/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#include "OdeTypedJoint.h"
#include "OdeSolverBody.h"
#include "OdeMacros.h"
#include <stdio.h>

void OdeTypedJoint::GetInfo1(Info1 *info)
{
    int joint_type = m_constraint->getConstraintType();
    switch (joint_type)
    {
    case POINT2POINT_CONSTRAINT_TYPE:
    {
        OdeP2PJoint p2pjoint(m_constraint,m_index,m_swapBodies,m_body0,m_body1);
        p2pjoint.GetInfo1(info);
    }
    break;
    case D6_CONSTRAINT_TYPE:
    {
        OdeD6Joint d6joint(m_constraint,m_index,m_swapBodies,m_body0,m_body1);
        d6joint.GetInfo1(info);
    }
    break;
    };
}

void OdeTypedJoint::GetInfo2(Info2 *info)
{
    int joint_type = m_constraint->getConstraintType();
    switch (joint_type)
    {
    case POINT2POINT_CONSTRAINT_TYPE:
    {
        OdeP2PJoint p2pjoint(m_constraint,m_index,m_swapBodies,m_body0,m_body1);
        p2pjoint.GetInfo2(info);
    }
    break;
    case D6_CONSTRAINT_TYPE:
    {
        OdeD6Joint d6joint(m_constraint,m_index,m_swapBodies,m_body0,m_body1);
        d6joint.GetInfo2(info);
    }
    break;
    };
}


OdeP2PJoint::OdeP2PJoint(
    btTypedConstraint * constraint,
    int index,bool swap,OdeSolverBody* body0,OdeSolverBody* body1):
        OdeTypedJoint(constraint,index,swap,body0,body1)
{
}


void OdeP2PJoint::GetInfo1(Info1 *info)
{
    info->m = 3;
    info->nub = 3;
}


void OdeP2PJoint::GetInfo2(Info2 *info)
{

    btPoint2PointConstraint * p2pconstraint = this->getP2PConstraint();

    //retrieve matrices
    btTransform body0_trans;
    if (m_body0)
    {
        body0_trans = m_body0->m_originalBody->getCenterOfMassTransform();
    }
//    btScalar body0_mat[12];
//    body0_mat[0] = body0_trans.getBasis()[0][0];
//    body0_mat[1] = body0_trans.getBasis()[0][1];
//    body0_mat[2] = body0_trans.getBasis()[0][2];
//    body0_mat[4] = body0_trans.getBasis()[1][0];
//    body0_mat[5] = body0_trans.getBasis()[1][1];
//    body0_mat[6] = body0_trans.getBasis()[1][2];
//    body0_mat[8] = body0_trans.getBasis()[2][0];
//    body0_mat[9] = body0_trans.getBasis()[2][1];
//    body0_mat[10] = body0_trans.getBasis()[2][2];

    btTransform body1_trans;

    if (m_body1)
    {
        body1_trans = m_body1->m_originalBody->getCenterOfMassTransform();
    }
//    btScalar body1_mat[12];
//    body1_mat[0] = body1_trans.getBasis()[0][0];
//    body1_mat[1] = body1_trans.getBasis()[0][1];
//    body1_mat[2] = body1_trans.getBasis()[0][2];
//    body1_mat[4] = body1_trans.getBasis()[1][0];
//    body1_mat[5] = body1_trans.getBasis()[1][1];
//    body1_mat[6] = body1_trans.getBasis()[1][2];
//    body1_mat[8] = body1_trans.getBasis()[2][0];
//    body1_mat[9] = body1_trans.getBasis()[2][1];
//    body1_mat[10] = body1_trans.getBasis()[2][2];




    // anchor points in global coordinates with respect to body PORs.


    int s = info->rowskip;

    // set jacobian
    info->J1l[0] = 1;
    info->J1l[s+1] = 1;
    info->J1l[2*s+2] = 1;


    btVector3 a1,a2;

    a1 = body0_trans.getBasis()*p2pconstraint->getPivotInA();
    //dMULTIPLY0_331 (a1, body0_mat,m_constraint->m_pivotInA);
    dCROSSMAT (info->J1a,a1,s,-,+);
    if (m_body1)
    {
        info->J2l[0] = -1;
        info->J2l[s+1] = -1;
        info->J2l[2*s+2] = -1;
        a2 = body1_trans.getBasis()*p2pconstraint->getPivotInB();
        //dMULTIPLY0_331 (a2,body1_mat,m_constraint->m_pivotInB);
        dCROSSMAT (info->J2a,a2,s,+,-);
    }


    // set right hand side
    btScalar k = info->fps * info->erp;
    if (m_body1)
    {
        for (int j=0; j<3; j++)
        {
            info->c[j] = k * (a2[j] + body1_trans.getOrigin()[j] -
                              a1[j] - body0_trans.getOrigin()[j]);
        }
    }
    else
    {
        for (int j=0; j<3; j++)
        {
            info->c[j] = k * (p2pconstraint->getPivotInB()[j] - a1[j] -
                              body0_trans.getOrigin()[j]);
        }
    }
}


///////////////////limit motor support

/*! \pre testLimitValue must be called on limot*/
int bt_get_limit_motor_info2(
	btRotationalLimitMotor * limot,
	btRigidBody * body0, btRigidBody * body1,
	BU_Joint::Info2 *info, int row, btVector3 ax1, int rotational)
{


    int srow = row * info->rowskip;

    // if the joint is powered, or has joint limits, add in the extra row
    int powered = limot->m_enableMotor;
    int limit = limot->m_currentLimit;

    if (powered || limit)
    {
        btScalar *J1 = rotational ? info->J1a : info->J1l;
        btScalar *J2 = rotational ? info->J2a : info->J2l;

        J1[srow+0] = ax1[0];
        J1[srow+1] = ax1[1];
        J1[srow+2] = ax1[2];
        if (body1)
        {
            J2[srow+0] = -ax1[0];
            J2[srow+1] = -ax1[1];
            J2[srow+2] = -ax1[2];
        }

        // linear limot torque decoupling step:
        //
        // if this is a linear limot (e.g. from a slider), we have to be careful
        // that the linear constraint forces (+/- ax1) applied to the two bodies
        // do not create a torque couple. in other words, the points that the
        // constraint force is applied at must lie along the same ax1 axis.
        // a torque couple will result in powered or limited slider-jointed free
        // bodies from gaining angular momentum.
        // the solution used here is to apply the constraint forces at the point
        // halfway between the body centers. there is no penalty (other than an
        // extra tiny bit of computation) in doing this adjustment. note that we
        // only need to do this if the constraint connects two bodies.

        btVector3 ltd;	// Linear Torque Decoupling vector (a torque)
        if (!rotational && body1)
        {
            btVector3 c;
            c[0]=btScalar(0.5)*(body1->getCenterOfMassPosition()[0]
            				-body0->getCenterOfMassPosition()[0]);
            c[1]=btScalar(0.5)*(body1->getCenterOfMassPosition()[1]
            				-body0->getCenterOfMassPosition()[1]);
            c[2]=btScalar(0.5)*(body1->getCenterOfMassPosition()[2]
            				-body0->getCenterOfMassPosition()[2]);

			ltd = c.cross(ax1);

            info->J1a[srow+0] = ltd[0];
            info->J1a[srow+1] = ltd[1];
            info->J1a[srow+2] = ltd[2];
            info->J2a[srow+0] = ltd[0];
            info->J2a[srow+1] = ltd[1];
            info->J2a[srow+2] = ltd[2];
        }

        // if we're limited low and high simultaneously, the joint motor is
        // ineffective

        if (limit && (limot->m_loLimit == limot->m_hiLimit)) powered = 0;

        if (powered)
        {
            info->cfm[row] = 0.0f;//limot->m_normalCFM;
            if (! limit)
            {
                info->c[row] = limot->m_targetVelocity;
                info->lo[row] = -limot->m_maxMotorForce;
                info->hi[row] = limot->m_maxMotorForce;
            }
        }

        if (limit)
        {
            btScalar k = info->fps * limot->m_ERP;
            info->c[row] = -k * limot->m_currentLimitError;
            info->cfm[row] = 0.0f;//limot->m_stopCFM;

            if (limot->m_loLimit == limot->m_hiLimit)
            {
                // limited low and high simultaneously
                info->lo[row] = -dInfinity;
                info->hi[row] = dInfinity;
            }
            else
            {
                if (limit == 1)
                {
                    // low limit
                    info->lo[row] = 0;
                    info->hi[row] = SIMD_INFINITY;
                }
                else
                {
                    // high limit
                    info->lo[row] = -SIMD_INFINITY;
                    info->hi[row] = 0;
                }

                // deal with bounce
                if (limot->m_bounce > 0)
                {
                    // calculate joint velocity
                    btScalar vel;
                    if (rotational)
                    {
                        vel = body0->getAngularVelocity().dot(ax1);
                        if (body1)
                            vel -= body1->getAngularVelocity().dot(ax1);
                    }
                    else
                    {
                        vel = body0->getLinearVelocity().dot(ax1);
                        if (body1)
                            vel -= body1->getLinearVelocity().dot(ax1);
                    }

                    // only apply bounce if the velocity is incoming, and if the
                    // resulting c[] exceeds what we already have.
                    if (limit == 1)
                    {
                        // low limit
                        if (vel < 0)
                        {
                            btScalar newc = -limot->m_bounce* vel;
                            if (newc > info->c[row]) info->c[row] = newc;
                        }
                    }
                    else
                    {
                        // high limit - all those computations are reversed
                        if (vel > 0)
                        {
                            btScalar newc = -limot->m_bounce * vel;
                            if (newc < info->c[row]) info->c[row] = newc;
                        }
                    }
                }
            }
        }
        return 1;
    }
    else return 0;
}


///////////////////OdeD6Joint





OdeD6Joint::OdeD6Joint(
    btTypedConstraint * constraint,
    int index,bool swap,OdeSolverBody* body0,OdeSolverBody* body1):
        OdeTypedJoint(constraint,index,swap,body0,body1)
{
}


void OdeD6Joint::GetInfo1(Info1 *info)
{
	btGeneric6DofConstraint * d6constraint = this->getD6Constraint();
	//prepare constraint
	d6constraint->calculateTransforms();
    info->m = 3;
    info->nub = 3;

    //test angular limits
    for (int i=0;i<3 ;i++ )
    {
    	//if(i==2) continue;
		if(d6constraint->testAngularLimitMotor(i))
		{
			info->m++;
		}
    }


}


int OdeD6Joint::setLinearLimits(Info2 *info)
{

    btGeneric6DofConstraint * d6constraint = this->getD6Constraint();

    //retrieve matrices
    btTransform body0_trans;
    if (m_body0)
    {
        body0_trans = m_body0->m_originalBody->getCenterOfMassTransform();
    }

    btTransform body1_trans;

    if (m_body1)
    {
        body1_trans = m_body1->m_originalBody->getCenterOfMassTransform();
    }

    // anchor points in global coordinates with respect to body PORs.

    int s = info->rowskip;

    // set jacobian
    info->J1l[0] = 1;
    info->J1l[s+1] = 1;
    info->J1l[2*s+2] = 1;


    btVector3 a1,a2;

    a1 = body0_trans.getBasis()*d6constraint->getFrameOffsetA().getOrigin();
    //dMULTIPLY0_331 (a1, body0_mat,m_constraint->m_pivotInA);
    dCROSSMAT (info->J1a,a1,s,-,+);
    if (m_body1)
    {
        info->J2l[0] = -1;
        info->J2l[s+1] = -1;
        info->J2l[2*s+2] = -1;
        a2 = body1_trans.getBasis()*d6constraint->getFrameOffsetB().getOrigin();

        //dMULTIPLY0_331 (a2,body1_mat,m_constraint->m_pivotInB);
        dCROSSMAT (info->J2a,a2,s,+,-);
    }


    // set right hand side
    btScalar k = info->fps * info->erp;
    if (m_body1)
    {
        for (int j=0; j<3; j++)
        {
            info->c[j] = k * (a2[j] + body1_trans.getOrigin()[j] -
                              a1[j] - body0_trans.getOrigin()[j]);
        }
    }
    else
    {
        for (int j=0; j<3; j++)
        {
            info->c[j] = k * (d6constraint->getCalculatedTransformB().getOrigin()[j] - a1[j] -
                              body0_trans.getOrigin()[j]);
        }
    }

    return 3;

}

int OdeD6Joint::setAngularLimits(Info2 *info, int row_offset)
{
	btGeneric6DofConstraint * d6constraint = this->getD6Constraint();
	int row = row_offset;
	//solve angular limits
    for (int i=0;i<3 ;i++ )
    {
    	//if(i==2) continue;
		if(d6constraint->getRotationalLimitMotor(i)->needApplyTorques())
		{
			btVector3 axis = d6constraint->getAxis(i);
			row += bt_get_limit_motor_info2(
				d6constraint->getRotationalLimitMotor(i),
				m_body0->m_originalBody,m_body1->m_originalBody,info,row,axis,1);
		}
    }

    return row;
}

void OdeD6Joint::GetInfo2(Info2 *info)
{
    int row = setLinearLimits(info);
    setAngularLimits(info, row);
}

