#ifndef B3_SOUND_ENGINE_H
#define B3_SOUND_ENGINE_H

#include "Bullet3Common/b3Scalar.h"
#include "b3Sound_C_Api.h"

struct b3SoundMessage
{
	int m_type;//B3_SOUND_SOURCE_TYPE
	double m_amplitude;

	double m_frequency;
	int m_wavId;

	double m_attackRate;
	double m_decayRate;
	double m_sustainLevel;
	double m_releaseRate;
	bool m_autoKeyOff;

	b3SoundMessage()
		:m_type(B3_SOUND_SOURCE_SINE_OSCILLATOR),
		m_amplitude(0.5),
		m_frequency(440),
		m_wavId(-1),
		m_attackRate(0.001),
		m_decayRate(0.00001),
		m_sustainLevel(0.5),
		m_releaseRate(0.0005),
		m_autoKeyOff(false)
	{
	}
};

class b3SoundEngine
{
	struct b3SoundEngineInternalData* m_data;
	
	public:
		
	b3SoundEngine();
	virtual ~b3SoundEngine();

	void init(int maxNumSoundSources, bool useRealTimeDac);
	void exit();

	int getAvailableSoundSource();
	void startSound(int soundSourceIndex, b3SoundMessage msg);
	void releaseSound(int soundSourceIndex);

	int loadWavFile(const char* fileName);

	double getSampleRate() const;

};

#endif //B3_SOUND_ENGINE_H
