#ifndef SDF_AUDIO_TYPES_H
#define SDF_AUDIO_TYPES_H

#include <string>

///See audio_source element in http://sdformat.org/spec?ver=1.6&elem=link
struct SDFAudioSource
{
	enum
	{
		SDFAudioSourceValid = 1,
		SDFAudioSourceLooping = 2,
	};

	int m_flags;  //repeat mode (0 = no repeat, 1 = loop forever)

	std::string m_uri;  //media filename of the sound, .wav file
	double m_pitch;     //1 = regular rate, -1 play in reverse
	double m_gain;      //normalized volume in range [0..1] where 0 is silent, 1 is most loud

	double m_attackRate;
	double m_decayRate;
	double m_sustainLevel;
	double m_releaseRate;

	double m_collisionForceThreshold;  //force that will trigger the audio, in Newton. If < 0, audio source is invalid

	int m_userIndex;

	SDFAudioSource()
		: m_flags(0),
		  m_pitch(1),
		  m_gain(1),
		  m_attackRate(0.0001),
		  m_decayRate(0.00001),
		  m_sustainLevel(0.5),
		  m_releaseRate(0.0005),
		  m_collisionForceThreshold(0.5),
		  m_userIndex(-1)
	{
	}
};

#endif  //SDF_AUDIO_TYPES_H
