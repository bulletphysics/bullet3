#include "TinyAudioExample.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"

#include "RtAudio.h"
#include "b3AudioListener.h"
#include "b3SoundSource.h"

class TinyAudioExample : public CommonExampleInterface
{
	b3AudioListener m_listener;
	b3SoundSource m_soundA;
	RtAudio m_dac;
	GUIHelperInterface* m_guiHelper;
	int m_soundIndexA;

public:
	TinyAudioExample(struct GUIHelperInterface* helper)
		:m_guiHelper(helper)
	{
	}
	
	virtual ~TinyAudioExample()
	{
	}

	virtual void initPhysics()
	{
		m_soundIndexA = m_listener.addSoundSource(&m_soundA);
		RtAudioFormat format = ( sizeof(double) == 8 ) ? RTAUDIO_FLOAT64 : RTAUDIO_FLOAT32;
		RtAudio::StreamParameters parameters;
		parameters.deviceId = 1;// dac.getDefaultOutputDevice();
		parameters.nChannels = 2;

		// The default real-time audio input and output buffer size.  If
		// clicks are occuring in the input and/or output sound stream, a
		// larger buffer size may help.  Larger buffer sizes, however, produce
		// more latency.
		const unsigned int RT_BUFFER_SIZE = 512;

		unsigned int bufferFrames = RT_BUFFER_SIZE;
		int sampleRate = m_listener.getSampleRate();

		m_dac.openStream( &parameters, NULL, format, (unsigned int)sampleRate, &bufferFrames, &b3AudioListener::tick,
		(void *)m_listener.getTickData());

		// Install an interrupt handler function.
		//	(void) signal( SIGINT, finish );

		m_dac.startStream();

	}  
	
	virtual void exitPhysics()
	{
		m_dac.closeStream();
		m_listener.removeSoundSource(m_soundIndexA);
	}

	virtual void renderScene()
	{
	}
	
	virtual void	stepSimulation(float deltaTime)
	{
	}

	virtual void	physicsDebugDraw(int debugFlags)
	{
	}
	virtual bool	mouseMoveCallback(float x,float y)
	{
		return false;
	}
	virtual bool	mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}
	virtual bool	keyboardCallback(int key, int state)
	{
		return false;
	}

	void resetCamera()
	{
		float dist = 4;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}

};

CommonExampleInterface*    TinyAudioExampleCreateFunc(CommonExampleOptions& options)
{
	return new TinyAudioExample(options.m_guiHelper);

}


B3_STANDALONE_EXAMPLE(TinyAudioExampleCreateFunc)
