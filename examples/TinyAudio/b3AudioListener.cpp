#include "b3AudioListener.h"
#include "b3SoundSource.h"
#include "Bullet3Common/b3Logging.h"
#include "b3WriteWavFile.h"

template <class T>
inline const T& MyMin(const T& a, const T& b)
{
	return a < b ? a : b;
}
#define MAX_SOUND_SOURCES 128
#define B3_SAMPLE_RATE 48000

struct b3AudioListenerInternalData
{
	int m_numControlTicks;
	double m_sampleRate;

	b3SoundSource* m_soundSources[MAX_SOUND_SOURCES];

	b3WriteWavFile m_wavOut2;
	bool m_writeWavOut;

	b3AudioListenerInternalData()
		: m_numControlTicks(64),
		  m_sampleRate(B3_SAMPLE_RATE),
		  m_writeWavOut(false)
	{
		for (int i = 0; i < MAX_SOUND_SOURCES; i++)
		{
			m_soundSources[i] = 0;
		}
	}
};

b3AudioListener::b3AudioListener()
{
	m_data = new b3AudioListenerInternalData();
	if (m_data->m_writeWavOut)
	{
		m_data->m_wavOut2.setWavFile("bulletAudio2.wav", B3_SAMPLE_RATE, 2, false);
	}
}

b3AudioListener::~b3AudioListener()
{
	if (m_data->m_writeWavOut)
	{
		m_data->m_wavOut2.closeWavFile();
	}

	delete m_data;
}

int b3AudioListener::addSoundSource(b3SoundSource* source)
{
	int soundIndex = -1;

	for (int i = 0; i < MAX_SOUND_SOURCES; i++)
	{
		if (m_data->m_soundSources[i] == 0)
		{
			m_data->m_soundSources[i] = source;
			soundIndex = i;
			break;
		}
	}
	return soundIndex;
}

void b3AudioListener::removeSoundSource(b3SoundSource* source)
{
	for (int i = 0; i < MAX_SOUND_SOURCES; i++)
	{
		if (m_data->m_soundSources[i] == source)
		{
			m_data->m_soundSources[i] = 0;
		}
	}
}

b3AudioListenerInternalData* b3AudioListener::getTickData()
{
	return m_data;
}

const b3AudioListenerInternalData* b3AudioListener::getTickData() const
{
	return m_data;
}

double b3AudioListener::getSampleRate() const
{
	return m_data->m_sampleRate;
}

void b3AudioListener::setSampleRate(double sampleRate)
{
	m_data->m_sampleRate = sampleRate;
}

int b3AudioListener::tick(void* outputBuffer, void* inputBuffer1, unsigned int nBufferFrames,
						  double streamTime, unsigned int status, void* dataPointer)
{
	B3_PROFILE("b3AudioListener::tick");

	b3AudioListenerInternalData* data = (b3AudioListenerInternalData*)dataPointer;
	register double outs[2], *samples = (double*)outputBuffer;
	register double tempOuts[2];
	int counter, nTicks = (int)nBufferFrames;
	bool done = false;

	int numSamples = 0;

	while (nTicks > 0 && !done)
	{
		counter = MyMin(nTicks, data->m_numControlTicks);
		bool newsynth = true;
		if (newsynth)
		{
			for (int i = 0; i < counter; i++)
			{
				outs[0] = 0.;
				outs[1] = 0.;
				//make_sound_double(outs,1);
				float numActiveSources = 0;

				for (int i = 0; i < MAX_SOUND_SOURCES; i++)
				{
					if (data->m_soundSources[i])
					{
						tempOuts[0] = 0;
						tempOuts[1] = 0;

						if (data->m_soundSources[i]->computeSamples(tempOuts, 1, data->m_sampleRate))
						{
							numActiveSources++;
							//simple mixer
							outs[0] += tempOuts[0];
							outs[1] += tempOuts[1];
						}
					}
				}

				//soft-clipping of sounds
				outs[0] = tanh(outs[0]);
				outs[1] = tanh(outs[1]);

				*samples++ = outs[0];
				*samples++ = outs[1];
				numSamples++;
			}
			nTicks -= counter;
		}
		if (nTicks == 0)
			break;
	}

	//logging to wav file
	if (data->m_writeWavOut && numSamples)
	{
		data->m_wavOut2.tick((double*)outputBuffer, numSamples);
	}
	return 0;
}
