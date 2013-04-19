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


#include "b3TypedConstraint.h"
//#include "Bullet3Common/btSerializer.h"


#define DEFAULT_DEBUGDRAW_SIZE b3Scalar(0.3f)



b3TypedConstraint::b3TypedConstraint(btTypedConstraintType type, int rbA,int rbB)
:btTypedObject(type),
m_userConstraintType(-1),
m_userConstraintId(-1),
m_breakingImpulseThreshold(SIMD_INFINITY),
m_isEnabled(true),
m_needsFeedback(false),
m_overrideNumSolverIterations(-1),
m_rbA(rbA),
m_rbB(rbB),
m_appliedImpulse(b3Scalar(0.)),
m_dbgDrawSize(DEFAULT_DEBUGDRAW_SIZE),
m_jointFeedback(0)
{
}




b3Scalar b3TypedConstraint::getMotorFactor(b3Scalar pos, b3Scalar lowLim, b3Scalar uppLim, b3Scalar vel, b3Scalar timeFact)
{
	if(lowLim > uppLim)
	{
		return b3Scalar(1.0f);
	}
	else if(lowLim == uppLim)
	{
		return b3Scalar(0.0f);
	}
	b3Scalar lim_fact = b3Scalar(1.0f);
	b3Scalar delta_max = vel / timeFact;
	if(delta_max < b3Scalar(0.0f))
	{
		if((pos >= lowLim) && (pos < (lowLim - delta_max)))
		{
			lim_fact = (lowLim - pos) / delta_max;
		}
		else if(pos  < lowLim)
		{
			lim_fact = b3Scalar(0.0f);
		}
		else
		{
			lim_fact = b3Scalar(1.0f);
		}
	}
	else if(delta_max > b3Scalar(0.0f))
	{
		if((pos <= uppLim) && (pos > (uppLim - delta_max)))
		{
			lim_fact = (uppLim - pos) / delta_max;
		}
		else if(pos  > uppLim)
		{
			lim_fact = b3Scalar(0.0f);
		}
		else
		{
			lim_fact = b3Scalar(1.0f);
		}
	}
	else
	{
			lim_fact = b3Scalar(0.0f);
	}
	return lim_fact;
}



void btAngularLimit::set(b3Scalar low, b3Scalar high, b3Scalar _softness, b3Scalar _biasFactor, b3Scalar _relaxationFactor)
{
	m_halfRange = (high - low) / 2.0f;
	m_center = btNormalizeAngle(low + m_halfRange);
	m_softness =  _softness;
	m_biasFactor = _biasFactor;
	m_relaxationFactor = _relaxationFactor;
}

void btAngularLimit::test(const b3Scalar angle)
{
	m_correction = 0.0f;
	m_sign = 0.0f;
	m_solveLimit = false;

	if (m_halfRange >= 0.0f)
	{
		b3Scalar deviation = btNormalizeAngle(angle - m_center);
		if (deviation < -m_halfRange)
		{
			m_solveLimit = true;
			m_correction = - (deviation + m_halfRange);
			m_sign = +1.0f;
		}
		else if (deviation > m_halfRange)
		{
			m_solveLimit = true;
			m_correction = m_halfRange - deviation;
			m_sign = -1.0f;
		}
	}
}


b3Scalar btAngularLimit::getError() const
{
	return m_correction * m_sign;
}

void btAngularLimit::fit(b3Scalar& angle) const
{
	if (m_halfRange > 0.0f)
	{
		b3Scalar relativeAngle = btNormalizeAngle(angle - m_center);
		if (!btEqual(relativeAngle, m_halfRange))
		{
			if (relativeAngle > 0.0f)
			{
				angle = getHigh();
			}
			else
			{
				angle = getLow();
			}
		}
	}
}

b3Scalar btAngularLimit::getLow() const
{
	return btNormalizeAngle(m_center - m_halfRange);
}

b3Scalar btAngularLimit::getHigh() const
{
	return btNormalizeAngle(m_center + m_halfRange);
}
