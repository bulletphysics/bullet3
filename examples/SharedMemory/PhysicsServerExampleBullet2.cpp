
#include "PhysicsServerExampleBullet2.h"
#include "PhysicsServerExample.h"
#include "PhysicsServerCommandProcessor.h"
#include "../CommonInterfaces/CommonExampleInterface.h"

struct Bullet2CommandProcessorCreation : public CommandProcessorCreationInterface
{
	virtual class CommandProcessorInterface* createCommandProcessor()
	{
		PhysicsServerCommandProcessor* proc = new PhysicsServerCommandProcessor;
		return proc;
	}

	virtual void deleteCommandProcessor(CommandProcessorInterface* proc)
	{
		delete proc;
	}
};


static Bullet2CommandProcessorCreation sBullet2CommandCreator;

CommonExampleInterface*    PhysicsServerCreateFuncBullet2(struct CommonExampleOptions& options)
{
	options.m_commandProcessorCreation = &sBullet2CommandCreator;
	
	CommonExampleInterface* example = PhysicsServerCreateFuncInternal(options);
	return example;

}

B3_STANDALONE_EXAMPLE(PhysicsServerCreateFuncBullet2)


