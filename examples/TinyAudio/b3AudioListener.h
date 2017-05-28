#ifndef B3_AUDIO_LISTENER_H
#define B3_AUDIO_LISTENER_H

class b3SoundSource;


class b3AudioListener
{
	struct b3AudioListenerInternalData* m_data;

public:
	b3AudioListener();
	virtual ~b3AudioListener();
	
	static int tick(void *outputBuffer, void *inputBuffer1, unsigned int nBufferFrames,
         double streamTime, unsigned int status, void *dataPointer);

	int addSoundSource(b3SoundSource* source);
	void removeSoundSource(b3SoundSource* source);

	b3AudioListenerInternalData* getTickData();
	const b3AudioListenerInternalData* getTickData() const;

	double getSampleRate() const;
	void setSampleRate(double sampleRate);

};         	

#endif //B3_AUDIO_LISTENER_H