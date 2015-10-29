#ifndef PHYSICS_SERVER_EXAMPLE_H
#define PHYSICS_SERVER_EXAMPLE_H

enum PhysicsServerOptions
{
	PHYSICS_SERVER_ENABLE_COMMAND_LOGGING=1,
};

class CommonExampleInterface*    PhysicsServerCreateFunc(struct CommonExampleOptions& options);

#endif //PHYSICS_SERVER_EXAMPLE_H


