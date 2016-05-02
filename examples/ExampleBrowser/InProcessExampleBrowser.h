#ifndef IN_PROCESS_EXAMPLE_BROWSER_H
#define IN_PROCESS_EXAMPLE_BROWSER_H

struct btInProcessExampleBrowserInternalData;

btInProcessExampleBrowserInternalData* btCreateInProcessExampleBrowser(int argc,char** argv2);

bool btIsExampleBrowserTerminated(btInProcessExampleBrowserInternalData* data);

void btShutDownExampleBrowser(btInProcessExampleBrowserInternalData* data);

class SharedMemoryInterface* btGetSharedMemoryInterface(btInProcessExampleBrowserInternalData* data);


///////////////////////


struct btInProcessExampleBrowserMainThreadInternalData;

btInProcessExampleBrowserMainThreadInternalData* btCreateInProcessExampleBrowserMainThread(int argc,char** argv2);

bool btIsExampleBrowserMainThreadTerminated(btInProcessExampleBrowserMainThreadInternalData* data);

void btUpdateInProcessExampleBrowserMainThread(btInProcessExampleBrowserMainThreadInternalData* data);

void btShutDownExampleBrowserMainThread(btInProcessExampleBrowserMainThreadInternalData* data);

class SharedMemoryInterface* btGetSharedMemoryInterfaceMainThread(btInProcessExampleBrowserMainThreadInternalData* data);


//////////////////////

#endif //IN_PROCESS_EXAMPLE_BROWSER_H
