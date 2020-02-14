#ifndef EXAMPLE_BROWSER_GUI_H
#define EXAMPLE_BROWSER_GUI_H

#include "../CommonInterfaces/CommonExampleInterface.h"

class ExampleBrowserInterface
{
public:
	virtual ~ExampleBrowserInterface() {}

	virtual CommonExampleInterface* getCurrentExample() = 0;

	virtual bool init(int argc, char* argv[]) = 0;

	virtual void update(float deltaTime) = 0;

	virtual void updateGraphics() = 0;

	virtual bool requestedExit() = 0;

	virtual void setSharedMemoryInterface(class SharedMemoryInterface* sharedMem) = 0;
};

#endif  //EXAMPLE_BROWSER_GUI_H