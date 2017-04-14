#ifndef B3_SOUND_SOURCE_H
#define B3_SOUND_SOURCE_H

class b3SoundSource
{
	struct b3SoundSourceInternalData* m_data;

public:

	b3SoundSource();
	virtual ~b3SoundSource();
	
	virtual bool computeSamples(double *sampleBuffer, int numSamples, double sampleRate);
};

#endif //B3_SOUND_SOURCE_H
