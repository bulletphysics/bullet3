#include "b3SoundSource.h"

#define MY2PI (2.*3.14159265)
#include <math.h>

struct b3SoundOscillator
{
	int m_type;
	double m_amplitude;
	double m_phase;
	double m_frequency;

	double sampleWaveForm(double sampleRate)
	{
		while (m_phase >= MY2PI)
			m_phase -= MY2PI;

		double z = sinf(m_phase);
		double sample = m_amplitude*z;

		m_phase += MY2PI * (1./sampleRate) * m_frequency;
		return sample;
	}

	b3SoundOscillator()
		:m_phase(0),
		m_amplitude(0.8),
		m_frequency(442.)
	{

	}
};

struct b3SoundSourceInternalData
{
	b3SoundOscillator m_oscillator;
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
		double sample = m_data->m_oscillator.sampleWaveForm(sampleRate);
		double sampleLeft = sample;
		double sampleRight = sample;

		*outputSamples++ = sampleLeft;
		*outputSamples++ = sampleRight;

	}

	return true;
//	return false;
}