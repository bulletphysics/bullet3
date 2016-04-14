
//#define EXAMPLE_CONSOLE_ONLY
#ifdef EXAMPLE_CONSOLE_ONLY
	#include "EmptyBrowser.h"
	typedef EmptyBrowser DefaultBrowser;
#else
	#include "OpenGLExampleBrowser.h"
	typedef OpenGLExampleBrowser DefaultBrowser;
#endif //EXAMPLE_CONSOLE_ONLY

#include "Bullet3Common/b3CommandLineArgs.h"
#include "../Utils/b3Clock.h"

#include "ExampleEntries.h"
#include "Bullet3Common/b3Logging.h"



int main(int argc, char* argv[])
{
	b3CommandLineArgs args(argc,argv);
	b3Clock clock;
	
	
	ExampleEntries examples;
	examples.initExampleEntries();

	ExampleBrowserInterface* exampleBrowser = new DefaultBrowser(&examples);
	bool init = exampleBrowser->init(argc,argv);
	clock.reset();
	if (init)
	{
		do 
		{
			float deltaTimeInSeconds = clock.getTimeMicroseconds()/1000000.f;
			clock.reset();
			exampleBrowser->update(deltaTimeInSeconds);

		} while (!exampleBrowser->requestedExit());
	}
	delete exampleBrowser;
	
	
	return 0;
}
