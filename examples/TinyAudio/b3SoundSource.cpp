#include "b3SoundSource.h"

#define MY2PI (2.*3.14159265)
#include <math.h>

#include "../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3FileUtils.h"
#include "b3ReadWavFile.h"
#include "b3ADSR.h"

struct b3SoundOscillator
{
	int m_type;
	double m_frequency;
	double m_amplitude;
	double m_phase;
	
	b3WavTicker m_wavTicker;


	double sampleSineWaveForm(double sampleRate)
	{
		while (m_phase >= MY2PI)
			m_phase -= MY2PI;

		double z =  sinf(m_phase);
		double sample = m_amplitude*z;

		m_phase += MY2PI * (1./sampleRate) * m_frequency;
		return sample;
	}

	double sampleSawWaveForm(double sampleRate)
	{
		while (m_phase >= MY2PI)
			m_phase -= MY2PI;

		double z =  2.*(m_phase)/MY2PI-1.;
		double sample = m_amplitude*z;

		m_phase += MY2PI * (1./sampleRate) * m_frequency;
		return sample;
	}

	void reset()
	{
		m_phase = 0;
	}

	b3SoundOscillator()
		:m_type(0),
		m_frequency(442.),
		m_amplitude(1),
		m_phase(0)
	{
		
	}
};
#define MAX_OSCILLATORS 2

struct b3SoundSourceInternalData
{
	b3SoundOscillator m_oscillators[MAX_OSCILLATORS];
	b3ADSR m_envelope;
	b3ReadWavFile m_wavFile;
};

b3SoundSource::b3SoundSource()
{
	m_data = new b3SoundSourceInternalData();
}

b3SoundSource::~b3SoundSource()
{
	delete m_data;
}

bool b3SoundSource::computeSamples(double* sampleBuffer, int numSamples, double sampleRate)
{
	
	double* outputSamples = sampleBuffer;

	for (int i=0;i<numSamples;i++)
	{
		double samples[MAX_OSCILLATORS] ={0};

		double env = m_data->m_envelope.tick();
		if (env)
		{
		for (int osc=0;osc<MAX_OSCILLATORS;osc++)
		{
			if (m_data->m_oscillators[osc].m_type == 0)
			{
				samples[osc] +=  env * m_data->m_oscillators[osc].sampleSineWaveForm(sampleRate);
			}

			if (m_data->m_oscillators[osc].m_type == 1)
			{
				samples[osc] += env * m_data->m_oscillators[osc].sampleSawWaveForm(sampleRate);
			}

			if (m_data->m_oscillators[osc].m_type == 128)
			{
				int frame = 0;
				double data = m_data->m_oscillators[osc].m_amplitude * m_data->m_wavFile.tick(frame,&m_data->m_oscillators[osc].m_wavTicker);
				
				samples[osc] += data;
			}
	
		}	
		}
		//sample *= 1./double(MAX_OSCILLATORS);

		double sampleLeft = samples[0];
		double sampleRight = samples[1];

		*outputSamples++ = sampleRight;
		*outputSamples++ = sampleLeft ;
	}

/*	if (m_data->m_flags & looping)
	{
		for (int osc=0;osc<MAX_OSCILLATORS;osc++)
		{
			if (m_data->m_oscillators[osc].m_waveIn.isFinished())
				m_data->m_oscillators[osc].m_waveIn.reset();
		}
	}
	*/
	return true;
//	return false;
}

int b3SoundSource::getNumOscillators() const
{
	return MAX_OSCILLATORS;
}
void b3SoundSource::setOscillatorType(int oscillatorIndex, int type)
{
	m_data->m_oscillators[oscillatorIndex].m_type = type;
}
void b3SoundSource::setOscillatorFrequency(int oscillatorIndex, double frequency)
{
	m_data->m_oscillators[oscillatorIndex].m_frequency = frequency;
}
void b3SoundSource::setOscillatorAmplitude(int oscillatorIndex, double amplitude)
{
	m_data->m_oscillators[oscillatorIndex].m_amplitude = amplitude;
}
void b3SoundSource::setOscillatorPhase(int oscillatorIndex, double phase)
{
	m_data->m_oscillators[oscillatorIndex].m_phase = phase;
}

void b3SoundSource::startSound()
{
	if (m_data->m_envelope.isIdle())
	{
		for (int osc=0;osc<MAX_OSCILLATORS;osc++)
		{
			static int reset = 0;
			printf("reset %d!\n",reset++);
			if (m_data->m_oscillators[osc].m_wavTicker.finished_)
			{
				m_data->m_oscillators[osc].reset();
				//test reverse playback of wav
				m_data->m_oscillators[osc].m_wavTicker.rate_ *= -1;
				if (m_data->m_oscillators[osc].m_wavTicker.rate_<0)
				{
					m_data->m_oscillators[osc].m_wavTicker.time_ = m_data->m_wavFile.getNumFrames()-1.;
				} else
				{
					m_data->m_oscillators[osc].m_wavTicker.time_ = 0.f;
				}

				m_data->m_oscillators[osc].m_wavTicker.finished_ = false;

			}
		}
	}
	m_data->m_envelope.keyOn();
}

void b3SoundSource::stopSound()
{
	m_data->m_envelope.keyOff();
}


bool b3SoundSource::setWavFile(int oscillatorIndex, const char* fileName, int sampleRate)
{
	char resourcePath[1024];
		
	if (b3ResourcePath::findResourcePath(fileName,resourcePath,1024))
	{

		m_data->m_wavFile.getWavInfo(resourcePath);
		m_data->m_wavFile.resize();
		m_data->m_wavFile.read(0,true);
		m_data->m_wavFile.normalize(1);
		m_data->m_oscillators[oscillatorIndex].m_wavTicker = m_data->m_wavFile.createWavTicker(sampleRate);

//		waveIn.openFile(resourcePath);
		double rate = 1.0;
	//	rate = waveIn.getFileRate() / stkSampleRate; 
	//	waveIn.setRate( rate );
	//	waveIn.ignoreSampleRateChange();
		// Find out how many channels we have.
	//	int channels = waveIn.channelsOut();
	//	m_data->m_oscillators[oscillatorIndex].m_frames.resize( 1, channels );
		m_data->m_oscillators[oscillatorIndex].m_type = 128;
		return true;
	}
	return false;
}