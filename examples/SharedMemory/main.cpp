/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "PhysicsServerExample.h"
#include "PhysicsClientExample.h"
#include "Bullet3Common/b3CommandLineArgs.h"

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "SharedMemoryCommon.h"


#include <stdlib.h>

int gSharedMemoryKey = -1;

static SharedMemoryCommon*    example = NULL;
static bool interrupted = false;

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
        delete example;
        errx(EXIT_FAILURE, "aborted example on signal %d", signo);
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
    
	b3CommandLineArgs args(argc, argv);
	
	DummyGUIHelper noGfx;

	CommonExampleOptions options(&noGfx);

	args.GetCmdLineArgument("shared_memory_key", gSharedMemoryKey);
	
  	if (args.CheckCmdLineFlag("client"))
    {
        example = (SharedMemoryCommon*)PhysicsClientCreateFunc(options);
    }else
    {
        example = (SharedMemoryCommon*)PhysicsServerCreateFunc(options);
    }
	
	
	example->initPhysics();
	while (example->isConnected() && !(example->wantsTermination() || interrupted))
	{
		example->stepSimulation(1.f/60.f);
	}	

	example->exitPhysics();

	delete example;

	return 0;
}

