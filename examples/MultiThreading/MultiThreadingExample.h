#ifndef MULTI_THREADING_EXAMPLE_H
#define MULTI_THREADING_EXAMPLE_H

enum EnumMultiThreadingExampleTypes
{
	SINGLE_SIM_THREAD = 0,
};

class CommonExampleInterface* MultiThreadingExampleCreateFunc(struct CommonExampleOptions& options);

#endif  //MULTI_THREADING_EXAMPLE_H
