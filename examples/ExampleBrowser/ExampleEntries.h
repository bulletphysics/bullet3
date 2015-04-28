
#ifndef EXAMPLE_ENTRIES_H
#define EXAMPLE_ENTRIES_H

#include "../CommonInterfaces/ExampleInterface.h"




class ExampleEntries
{

	struct ExampleEntriesInternalData* m_data;

public:

	ExampleEntries();
	virtual ~ExampleEntries();

	static void registerExampleEntry(int menuLevel, const char* name,const char* description, ExampleInterface::CreateFunc* createFunc, int option=0);
	
	void initExampleEntries();
	
	int getNumRegisteredExamples();

	ExampleInterface::CreateFunc* getExampleCreateFunc(int index);

	const char* getExampleName(int index);
	
	const char* getExampleDescription(int index);

	int	getExampleOption(int index);

};



#endif //EXAMPLE_ENTRIES_H
