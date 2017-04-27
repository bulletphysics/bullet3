#include "b3SoundEngine.h"

#include "RtAudio.h"

#include "b3AudioListener.h"
#include "b3SoundSource.h"
#include "Bullet3Common/b3AlignedObjectArray.h"

// The default real-time audio input and output buffer size.  If
// clicks are occuring in the input and/or output sound stream, a
// larger buffer size may help.  Larger buffer sizes, however, produce
// more latency.
//const unsigned int RT_BUFFER_SIZE = 1024;
const unsigned int RT_BUFFER_SIZE = 256;

struct b3SoundEngineInternalData
{
	b3AudioListener m_listener;
	RtAudio m_dac;

	b3AlignedObjectArray<b3SoundSource*> m_soundSources;

};

b3SoundEngine::b3SoundEngine()
{
	m_data = new b3SoundEngineInternalData();
}

b3SoundEngine::~b3SoundEngine()
{
	exit();
	delete m_data;
}

void b3SoundEngine::init()
{

	RtAudioFormat format = ( sizeof(double) == 8 ) ? RTAUDIO_FLOAT64 : RTAUDIO_FLOAT32;
	RtAudio::StreamParameters parameters;
	parameters.deviceId = m_data->m_dac.getDefaultOutputDevice();
	parameters.nChannels = 2;

	unsigned int bufferFrames = RT_BUFFER_SIZE;
	double sampleRate = m_data->m_listener.getSampleRate();

	m_data->m_dac.openStream( &parameters, NULL, format, (unsigned int)sampleRate, &bufferFrames, &b3AudioListener::tick,
	(void *)m_data->m_listener.getTickData());

	m_data->m_dac.startStream();
}

void b3SoundEngine::exit()
{
	m_data->m_dac.closeStream();

	for (int i=0;i<m_data->m_soundSources.size();i++)
	{
		m_data->m_listener.removeSoundSource(m_data->m_soundSources[i]);
		delete m_data->m_soundSources[i];
	}
	m_data->m_soundSources.clear();
}


void b3SoundEngine::addSoundSource(b3SoundSource* source)
{
	m_data->m_soundSources.push_back(source);
	m_data->m_listener.addSoundSource(source);
}

void b3SoundEngine::removeSoundSource(b3SoundSource* source)
{
	m_data->m_soundSources.remove(source);
}

int b3SoundEngine::loadWavFile(const char* fileName)
{
	return 0;
}

double b3SoundEngine::getSampleRate() const
{
	return m_data->m_listener.getSampleRate();
}

	