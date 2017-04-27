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

public:

	b3ADSR();
	virtual ~b3ADSR();

	double tick();
	bool isIdle() const;
	void keyOn();
	void keyOff();
};

#endif //B3_ADSR_H