#ifndef OPENGL_BROWSER_GUI_H
#define OPENGL_BROWSER_GUI_H

#include "ExampleBrowserInterface.h"

class OpenGLExampleBrowser : public ExampleBrowserInterface
{
public:

	OpenGLExampleBrowser(class ExampleEntries* examples);
	virtual ~OpenGLExampleBrowser();
	
	virtual CommonExampleInterface* getCurrentExample();
	
	virtual bool init(int argc, char* argv[]);

	virtual void update(float deltaTime);

	virtual bool requestedExit();
	
};

#endif //OPENGL_BROWSER_GUI_H
