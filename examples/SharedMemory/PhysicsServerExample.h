#ifndef PHYSICS_SERVER_EXAMPLE_H
#define PHYSICS_SERVER_EXAMPLE_H

enum PhysicsServerOptions
{
	PHYSICS_SERVER_ENABLE_COMMAND_LOGGING = 1,
	PHYSICS_SERVER_REPLAY_FROM_COMMAND_LOG = 2,
	PHYSICS_SERVER_USE_RTC_CLOCK = 4,
};

///Don't use PhysicsServerCreateFuncInternal directly
///Use PhysicsServerCreateFuncBullet2 instead, or initialize options.m_commandProcessor
class CommonExampleInterface* PhysicsServerCreateFuncInternal(struct CommonExampleOptions& options);

#endif  //PHYSICS_SERVER_EXAMPLE_H
