#include "OpenGLExampleBrowser.h"

#include "Bullet3Common/b3CommandLineArgs.h"
#include "../Utils/b3Clock.h"

#include "ExampleEntries.h"
#include "Bullet3Common/b3Logging.h"

#include "../Importers/ImportObjDemo/ImportObjExample.h"
#include "../Importers/ImportBsp/ImportBspExample.h"
#include "../Importers/ImportColladaDemo/ImportColladaSetup.h"
#include "../Importers/ImportSTLDemo/ImportSTLSetup.h"
#include "../Importers/ImportURDFDemo/ImportURDFSetup.h"
#include "../Importers/ImportSDFDemo/ImportSDFSetup.h"
#include "../Importers/ImportSTLDemo/ImportSTLSetup.h"
#include "../Importers/ImportBullet/SerializeSetup.h"

#include "LinearMath/btAlignedAllocator.h"

static double gMinUpdateTimeMicroSecs = 1000.;


static bool interrupted=false;
static OpenGLExampleBrowser* sExampleBrowser=0;

#ifndef _WIN32
#include <signal.h>
#include <err.h>
#include <unistd.h>
static void cleanup(int signo)
{

    if (interrupted) {  // this is the second time, we're hanging somewhere
      //  if (example) {
      //      example->abort();
      //  }

        b3Printf("Aborting and deleting SharedMemoryCommon object");
        sleep(1);
        delete sExampleBrowser;
	sExampleBrowser = 0;
        errx(EXIT_FAILURE, "aborted example on signal %d", signo);
    } else
    {
	b3Printf("no action");
    }
    interrupted = true;
    warnx("caught signal %d", signo);
}
#endif//_WIN32

int main(int argc, char* argv[])
{

#ifndef _WIN32
    struct sigaction action;
    memset(&action, 0x0, sizeof(action));
    action.sa_handler = cleanup;
    static const int signos[] = { SIGHUP, SIGINT, SIGQUIT, SIGABRT, SIGSEGV, SIGPIPE, SIGTERM };
    for (int ii(0); ii < sizeof(signos) / sizeof(*signos); ++ii) {
        if (0 != sigaction(signos[ii], &action, NULL)) {
            err(EXIT_FAILURE, "signal %d", signos[ii]);
        }
    }
#endif

	{
		b3CommandLineArgs args(argc, argv);
		b3Clock clock;
		args.GetCmdLineArgument("minUpdateTimeMicroSecs",gMinUpdateTimeMicroSecs);

		ExampleEntriesAll examples;
		examples.initExampleEntries();

		OpenGLExampleBrowser* exampleBrowser = new OpenGLExampleBrowser(&examples);
		sExampleBrowser = exampleBrowser;//for <CTRL-C> etc, cleanup shared memory
		bool init = exampleBrowser->init(argc, argv);
		exampleBrowser->registerFileImporter(".urdf", ImportURDFCreateFunc);
		exampleBrowser->registerFileImporter(".sdf", ImportSDFCreateFunc);
		exampleBrowser->registerFileImporter(".obj", ImportObjCreateFunc);
		exampleBrowser->registerFileImporter(".stl", ImportSTLCreateFunc);
		exampleBrowser->registerFileImporter(".bullet", SerializeBulletCreateFunc);


		clock.reset();
		if (init)
		{
			do
			{
				float deltaTimeInSeconds = clock.getTimeMicroseconds() / 1000000.f;
				if (deltaTimeInSeconds > 0.1)
				{
					deltaTimeInSeconds = 0.1;
				}
				if (deltaTimeInSeconds < (gMinUpdateTimeMicroSecs/1e6))
				{
					b3Clock::usleep(gMinUpdateTimeMicroSecs/10.);
				} else
				{
					clock.reset();
					exampleBrowser->update(deltaTimeInSeconds);
				}
			} while (!exampleBrowser->requestedExit() && !interrupted);
		}
		delete exampleBrowser;

	}
	
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	int numBytesLeaked = btDumpMemoryLeaks();
	btAssert(numBytesLeaked==0);
#endif//BT_DEBUG_MEMORY_ALLOCATIONS
	
	return 0;
}
