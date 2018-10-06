#ifndef B3_ADSR_H
#define B3_ADSR_H

class b3ADSR
{
	int m_state;
	double m_value;
	double m_target;
	double m_attackRate;
	double m_decayRate;
	double m_releaseRate;
	double m_releaseTime;
	double m_sustainLevel;
	bool m_autoKeyOff;

public:
	b3ADSR();
	virtual ~b3ADSR();

	double tick();
	bool isIdle() const;
	void keyOn(bool autoKeyOff);
	void keyOff();

	void setValues(double attack, double decay, double sustain, double release)
	{
		m_attackRate = attack;
		m_decayRate = decay;
		m_sustainLevel = sustain;
		m_releaseRate = release;
	}
};

#endif  //B3_ADSR_H