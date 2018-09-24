#include "TinyAudioExample.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3Common/b3HashMap.h"

#include "b3SoundEngine.h"
#include "b3SoundSource.h"
#include <string>

///very basic hashable string implementation, compatible with b3HashMap
struct MyHashString
{
	std::string m_string;
	unsigned int m_hash;

	B3_FORCE_INLINE unsigned int getHash() const
	{
		return m_hash;
	}

	MyHashString(const char* name)
		: m_string(name)
	{
		/* magic numbers from http://www.isthe.com/chongo/tech/comp/fnv/ */
		static const unsigned int InitialFNV = 2166136261u;
		static const unsigned int FNVMultiple = 16777619u;

		/* Fowler / Noll / Vo (FNV) Hash */
		unsigned int hash = InitialFNV;

		for (int i = 0; m_string[i]; i++)
		{
			hash = hash ^ (m_string[i]); /* xor  the low 8 bits */
			hash = hash * FNVMultiple;   /* multiply by the magic number */
		}
		m_hash = hash;
	}

	bool equals(const MyHashString& other) const
	{
		return (m_string == other.m_string);
	}
};

double base_frequency = 440.0;
double base_pitch = 69.0;

double MidiPitch2Frequency(double incoming_note)
{
	return base_frequency * pow(2.0, (incoming_note - base_pitch) / 12.0);
}

double FrequencytoMidiPitch(double incoming_frequency)
{
	return base_pitch + (12.0 * log(incoming_frequency / base_frequency) / log(2));
}

class TinyAudioExample : public CommonExampleInterface
{
	GUIHelperInterface* m_guiHelper;

	b3SoundEngine m_soundEngine;
	int m_wavId;

	b3HashMap<MyHashString, int> m_keyToSoundSource;

public:
	TinyAudioExample(struct GUIHelperInterface* helper)
		: m_guiHelper(helper)
	{
	}

	virtual ~TinyAudioExample()
	{
	}

	virtual void initPhysics()
	{
		int numSoundSources = 32;
		bool useRealTimeDac = true;

		m_soundEngine.init(numSoundSources, useRealTimeDac);

		m_wavId = m_soundEngine.loadWavFile("wav/xylophone.rosewood.ff.C5B5_1.wav");
		int sampleRate = m_soundEngine.getSampleRate();
	}

	virtual void exitPhysics()
	{
		m_soundEngine.exit();
	}

	virtual void renderScene()
	{
	}

	virtual void stepSimulation(float deltaTime)
	{
	}

	virtual void physicsDebugDraw(int debugFlags)
	{
	}
	virtual bool mouseMoveCallback(float x, float y)
	{
		return false;
	}
	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}

	virtual bool keyboardCallback(int key, int state)
	{
		if (key >= 'a' && key <= 'z')
		{
			char keyStr[2];
			keyStr[0] = (char)key;
			keyStr[1] = 0;
			MyHashString hs(keyStr);

			if (state)
			{
				int soundSourceIndex = m_soundEngine.getAvailableSoundSource();
				if (soundSourceIndex >= 0)
				{
					int note = key - (97 - 58);
					double freq = MidiPitch2Frequency(note);

					b3SoundMessage msg;
					msg.m_type = B3_SOUND_SOURCE_SINE_OSCILLATOR;
					msg.m_frequency = freq;
					msg.m_amplitude = 1;

					msg.m_type = B3_SOUND_SOURCE_WAV_FILE;
					msg.m_wavId = m_wavId;
					msg.m_attackRate = 1;
					msg.m_sustainLevel = 1;
					msg.m_releaseRate = 0.001;

					m_soundEngine.startSound(soundSourceIndex, msg);
					m_keyToSoundSource.insert(hs, soundSourceIndex);
					//printf("soundSourceIndex:%d\n", soundSourceIndex);

#if 0
					b3SoundSource* soundSource = this->m_soundSourcesPool[soundSourceIndex];

					soundSource->setOscillatorFrequency(0, freq );
					soundSource->setOscillatorFrequency(1, freq );
					soundSource->startSound();
					
					{
						int* soundSourceIndexPtr = m_keyToSoundSource[hs];
						if (soundSourceIndexPtr)
						{
							int newIndex = *soundSourceIndexPtr;
							printf("just inserted: %d\n", newIndex);
						}
					}
#endif
				}
			}
			else
			{
				int* soundSourceIndexPtr = m_keyToSoundSource[hs];
				if (soundSourceIndexPtr)
				{
					int soundSourceIndex = *soundSourceIndexPtr;
					//printf("releaseSound: %d\n", soundSourceIndex);
					m_soundEngine.releaseSound(soundSourceIndex);
				}
#if 0
					if (soundSourceIndex>=0)
					{
						printf("releasing %d\n", soundSourceIndex);
						b3SoundSource* soundSource = this->m_soundSourcesPool[soundSourceIndex];
						soundSource->stopSound();
					}
				}
#endif
			}
		}

		return false;
	}

	void resetCamera()
	{
		float dist = 4;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
};

CommonExampleInterface* TinyAudioExampleCreateFunc(CommonExampleOptions& options)
{
	return new TinyAudioExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(TinyAudioExampleCreateFunc)
