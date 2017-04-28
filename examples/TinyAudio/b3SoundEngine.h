#ifndef B3_SOUND_ENGINE_H
#define B3_SOUND_ENGINE_H

#include "Bullet3Common/b3Scalar.h"

struct b3SoundMessage
{
	int m_type;//B3_SOUND_SOURCE_TYPE
	double m_amplitude;

	double m_frequency;
	int m_wavId;

	double m_attack;
	double m_decay;
	double m_sustain;
	double m_release;
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
