#ifndef PHYSICS_CLIENT_EXAMPLE_H
#define PHYSICS_CLIENT_EXAMPLE_H

enum ClientExampleOptions
{
	eCLIENTEXAMPLE_LOOPBACK = 1,
	eCLIENTEXAMPLE_DIRECT = 2,
	eCLIENTEXAMPLE_SERVER = 3,
};

class CommonExampleInterface* PhysicsClientCreateFunc(struct CommonExampleOptions& options);

#endif  //PHYSICS_CLIENT_EXAMPLE_H
