#ifndef B3_SOUND_SOURCE_H
#define B3_SOUND_SOURCE_H

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

	bool setWavFile(int oscillatorIndex, const char* fileName, int sampleRate);

	void startSound();
	void stopSound();

};

#endif //B3_SOUND_SOURCE_H
