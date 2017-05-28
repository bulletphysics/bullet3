#ifndef B3_SOUND_SOURCE_H
#define B3_SOUND_SOURCE_H

#include "b3Sound_C_Api.h"


class b3SoundSource
{
	struct b3SoundSourceInternalData* m_data;

public:

	b3SoundSource();
	virtual ~b3SoundSource();
	
	virtual bool computeSamples(double *sampleBuffer, int numSamples, double sampleRate);

	int getNumOscillators() const;
	void setOscillatorType(int oscillatorIndex, int type);
	void setOscillatorFrequency(int oscillatorIndex, double frequency);
	void setOscillatorAmplitude(int oscillatorIndex, double amplitude);
	void setOscillatorPhase(int oscillatorIndex, double phase);
	void setADSR( double attackRate, double decayRate, double sustainLevel, double releaseRate);

	bool setWavFile(int oscillatorIndex, class b3ReadWavFile* wavFilePtr, int sampleRate);

	void startSound(bool autoKeyOff);
	void stopSound();

	bool isAvailable() const;
};

#endif //B3_SOUND_SOURCE_H
