
#ifndef EXAMPLE_ENTRIES_H
#define EXAMPLE_ENTRIES_H

#include "../CommonInterfaces/CommonExampleInterface.h"



class ExampleEntries
{

public:

        virtual ~ExampleEntries() {}


        virtual void initExampleEntries()=0;

        virtual void initOpenCLExampleEntries()=0;

        virtual int getNumRegisteredExamples()=0;

        virtual CommonExampleInterface::CreateFunc* getExampleCreateFunc(int index)=0;

        virtual const char* getExampleName(int index)=0;

        virtual const char* getExampleDescription(int index)=0;

        virtual int     getExampleOption(int index)=0;

};


class ExampleEntriesAll : public ExampleEntries
{

	struct ExampleEntriesInternalData* m_data;

public:

	ExampleEntriesAll();
	virtual ~ExampleEntriesAll();

	static void registerExampleEntry(int menuLevel, const char* name,const char* description, CommonExampleInterface::CreateFunc* createFunc, int option=0);
	
	virtual void initExampleEntries();

	virtual void initOpenCLExampleEntries();
	
	virtual int getNumRegisteredExamples();

	virtual CommonExampleInterface::CreateFunc* getExampleCreateFunc(int index);

	virtual const char* getExampleName(int index);
	
	virtual const char* getExampleDescription(int index);

	virtual int	getExampleOption(int index);

};



#endif //EXAMPLE_ENTRIES_H
