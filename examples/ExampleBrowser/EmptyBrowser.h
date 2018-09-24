#ifndef EMPTY_BROWSER
#define EMPTY_BROWSER

#include "ExampleBrowserInterface.h"
#include "EmptyExample.h"

class EmptyBrowser : public ExampleBrowserInterface
{
public:
	EmptyExample m_emptyExample;

	virtual CommonExampleInterface* getCurrentExample()
	{
		return &m_emptyExample;
	}

	EmptyBrowser(class ExampleEntries* examples)
	{
	}

	virtual bool init(int /*argc*/, char* argv[])
	{
		return true;
	}

	virtual void update(float deltaTime)
	{
		m_emptyExample.stepSimulation(deltaTime);
	}

	virtual bool requestedExit()
	{
		return false;
	}
};

#endif  //EMPTY_BROWSER
