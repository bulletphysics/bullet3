
#ifndef EXAMPLE_ENTRIES_H
#define EXAMPLE_ENTRIES_H

#include "../CommonInterfaces/CommonExampleInterface.h"




class ExampleEntries
{

	struct ExampleEntriesInternalData* m_data;

public:

	ExampleEntries();
	virtual ~ExampleEntries();

	static void registerExampleEntry(int menuLevel, const char* name,const char* description, CommonExampleInterface::CreateFunc* createFunc, int option=0);
	
	void initExampleEntries();

	void initOpenCLExampleEntries();
	
	int getNumRegisteredExamples();

	CommonExampleInterface::CreateFunc* getExampleCreateFunc(int index);

	const char* getExampleName(int index);
	
	const char* getExampleDescription(int index);

	int	getExampleOption(int index);

};



#endif //EXAMPLE_ENTRIES_H
