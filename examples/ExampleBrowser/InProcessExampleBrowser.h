#ifndef IN_PROCESS_EXAMPLE_BROWSER_H
#define IN_PROCESS_EXAMPLE_BROWSER_H

struct btInProcessExampleBrowserInternalData;

btInProcessExampleBrowserInternalData* btCreateInProcessExampleBrowser(int argc,char** argv2);

bool btIsExampleBrowserTerminated(btInProcessExampleBrowserInternalData* data);

void btShutDownExampleBrowser(btInProcessExampleBrowserInternalData* data);

class SharedMemoryInterface* btGetSharedMemoryInterface(btInProcessExampleBrowserInternalData* data);


#endif //IN_PROCESS_EXAMPLE_BROWSER_H
