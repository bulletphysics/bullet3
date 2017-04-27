#ifndef B3_SOUND_ENGINE_H
#define B3_SOUND_ENGINE_H

#include "Bullet3Common/b3Scalar.h"

class b3SoundEngine
{
	struct b3SoundEngineInternalData* m_data;
	
	public:
		
	b3SoundEngine();
	virtual ~b3SoundEngine();

	void init();
	void exit();

	//int createListener();

	void addSoundSource(class b3SoundSource* source);
	void removeSoundSource(class b3SoundSource* source);

	int loadWavFile(const char* fileName);

	double getSampleRate() const;

};

#endif //B3_SOUND_ENGINE_H
