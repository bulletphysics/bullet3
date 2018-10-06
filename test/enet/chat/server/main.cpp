/* server.cpp */
#include <stdio.h>
#include <enet/enet.h>

int main(int argc, char *argv[])
{
	ENetAddress address;
	ENetHost *server;
	ENetEvent event;
	int serviceResult;

	puts("Starting server");

	if (enet_initialize() != 0)
	{
		puts("Error initialising enet");
		exit(EXIT_FAILURE);
	}

	/* Bind the server to the default localhost.     */
	/* A specific host address can be specified by   */
	/* enet_address_set_host (& address, "x.x.x.x"); */
	address.host = ENET_HOST_ANY;
	/* Bind the server to port 1234. */
	address.port = 1234;

	server = enet_host_create(&address,
							  32, /* number of clients */
							  2,  /* number of channels */
							  0,  /* Any incoming bandwith */
							  0); /* Any outgoing bandwith */

	if (server == NULL)
	{
		puts("Could not create server host");
		exit(EXIT_FAILURE);
	}

	while (true)
	{
		serviceResult = 1;

		/* Keep doing host_service until no events are left */
		while (serviceResult > 0)
		{
			/* Wait up to 1000 milliseconds for an event. */
			serviceResult = enet_host_service(server, &event, 1000);

			if (serviceResult > 0)
			{
				switch (event.type)
				{
					case ENET_EVENT_TYPE_CONNECT:
						printf("A new client connected from %x:%u.\n",
							   event.peer->address.host,
							   event.peer->address.port);

						/* Store any relevant client information here. */
						event.peer->data = (void *)"Client information";

						break;

					case ENET_EVENT_TYPE_RECEIVE:
						printf(
							"A packet of length %lu containing '%s' was "
							"received from %s on channel %u.\n",
							event.packet->dataLength,
							event.packet->data,
							event.peer->data,
							event.channelID);

						/* Tell all clients about this message */
						enet_host_broadcast(server, 0, event.packet);

						break;

					case ENET_EVENT_TYPE_DISCONNECT:
						printf("%s disconnected.\n", event.peer->data);

						/* Reset the peer's client information. */

						event.peer->data = NULL;

						break;
					case ENET_EVENT_TYPE_NONE:
						break;
				}
			}
			else if (serviceResult > 0)
			{
				puts("Error with servicing the server");
				exit(EXIT_FAILURE);
			}
		}
	}

	enet_host_destroy(server);
	enet_deinitialize();

	return 0;
}
