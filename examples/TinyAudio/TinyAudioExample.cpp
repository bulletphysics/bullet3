#include "TinyAudioExample.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"



#include "b3SoundEngine.h"
#include "b3SoundSource.h"




class TinyAudioExample : public CommonExampleInterface
{
	
	GUIHelperInterface* m_guiHelper;

	
	b3SoundEngine m_soundEngine;
	b3SoundSource* m_soundSource;
	
public:
	TinyAudioExample(struct GUIHelperInterface* helper)
		:m_guiHelper(helper),
		m_soundSource(0)
	{
	}
	
	virtual ~TinyAudioExample()
	{
	}

	virtual void initPhysics()
	{
		m_soundEngine.init();
		int sampleRate = m_soundEngine.getSampleRate();

		m_soundSource = new b3SoundSource();
		m_soundSource->setWavFile(1,"wav/xylophone.rosewood.ff.C5B5_1.wav", sampleRate);
		m_soundSource->setWavFile(0,"wav/xylophone.rosewood.ff.C5B5_1.wav", sampleRate);
		m_soundSource->setOscillatorAmplitude(0,1);
		m_soundSource->setOscillatorAmplitude(1,1);
		m_soundEngine.addSoundSource(m_soundSource);

	}  
	
	virtual void exitPhysics()
	{
		m_soundEngine.exit();
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
		if (key=='v' || key=='b')
		{

				if (state)
				{
					if (key=='b')
					{
						m_soundSource->setOscillatorFrequency(0, 442);
						m_soundSource->setOscillatorFrequency(1, 442);
					}
					if (key=='v')
					{
						m_soundSource->setOscillatorFrequency(0, 2*442);
						m_soundSource->setOscillatorFrequency(1, 2*442);
					}
				}


				if (state==1)
				{
					m_soundSource->startSound();
					

				}
				else
				{
					m_soundSource->stopSound();
				}
		}


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
