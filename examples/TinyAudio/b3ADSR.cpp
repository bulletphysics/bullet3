#include "b3ADSR.h"
//ADSR mostly copied/reimplemented from Stk, see
//http://github.com/thestk/stk

//! ADSR envelope states.


enum
{
	ADSR_ATTACK,  /*!< Attack */
	ADSR_DECAY,   /*!< Decay */
	ADSR_SUSTAIN, /*!< Sustain */
	ADSR_RELEASE, /*!< Release */
	ADSR_IDLE     /*!< Before attack / after release */
};

b3ADSR::b3ADSR()
{
	m_target = 0.0;
	m_value = 0.0;
	m_attackRate = 0.001;
	m_decayRate = 0.00001;
	m_releaseRate = 0.0005;
	m_sustainLevel = 0.5;
	m_state = ADSR_IDLE;
	m_autoKeyOff = false;
}

b3ADSR::~b3ADSR()
{
}

double b3ADSR::tick()
{
	switch (m_state)
	{
		case ADSR_ATTACK:
			m_value += m_attackRate;
			if (m_value >= m_target)
			{
				m_value = m_target;
				m_target = m_sustainLevel;
				m_state = ADSR_DECAY;
			}
			break;

		case ADSR_DECAY:
			if (m_value > m_sustainLevel)
			{
				m_value -= m_decayRate;
				if (m_value <= m_sustainLevel)
				{
					m_value = m_sustainLevel;
					m_state = ADSR_SUSTAIN;
					if (m_autoKeyOff)
					{
						keyOff();
					}
				}
			}
			else
			{
				m_value += m_decayRate;  // attack target < sustain level
				if (m_value >= m_sustainLevel)
				{
					m_value = m_sustainLevel;
					m_state = ADSR_SUSTAIN;
					if (m_autoKeyOff)
					{
						keyOff();
					}

				}
			}
			break;

		case ADSR_RELEASE:
			m_value -= m_releaseRate;
			if (m_value <= 0.0)
			{
				m_value = 0.0;
				m_state = ADSR_IDLE;
			}
	}

	return m_value;
}

bool b3ADSR::isIdle() const
{
	return m_state == ADSR_IDLE;
}

void b3ADSR::keyOn(bool autoKeyOff)
{
	m_autoKeyOff = autoKeyOff;
	if (m_target <= 0.0)
		m_target = 1.0;
	if (m_attackRate==1)
	{
		m_value = 1.0;
	}
	m_state = ADSR_ATTACK;
}

void b3ADSR::keyOff()
{
	m_autoKeyOff = false;
	m_target = 0.0;
	m_state = ADSR_RELEASE;

}
