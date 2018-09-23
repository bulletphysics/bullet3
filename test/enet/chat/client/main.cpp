
/* client.cpp */
#include <stdio.h>
#include <string.h>
#include <enet/enet.h>

int main(int argc, char *argv[])
{
	ENetHost *client;
	ENetAddress address;
	ENetPeer *peer;
	ENetEvent event;
	char message[1024];
	int serviceResult;

	puts("Starting client");

	if (enet_initialize() != 0)
	{
		fprintf(stderr, "Error initialising enet");
		exit(EXIT_FAILURE);
	}

	client = enet_host_create(NULL,       /* create a client host */
							  1,          /* number of clients */
							  2,          /* number of channels */
							  57600 / 8,  /* incoming bandwith */
							  14400 / 8); /* outgoing bandwith */

	if (client == NULL)
	{
		fprintf(stderr, "Could not create client host");
		exit(EXIT_FAILURE);
	}

	enet_address_set_host(&address, "localhost");
	address.port = 1234;

	peer = enet_host_connect(client,
							 &address, /* address to connect to */
							 2,        /* number of channels */
							 0);       /* user data supplied to
						  the receiving host */

	if (peer == NULL)
	{
		fprintf(stderr,
				"No available peers for initiating an ENet "
				"connection.\n");
		exit(EXIT_FAILURE);
	}

	/* Try to connect to server within 5 seconds */
	if (enet_host_service(client, &event, 5000) > 0 &&
		event.type == ENET_EVENT_TYPE_CONNECT)
	{
		puts("Connection to server succeeded.");
	}
	else
	{
		/* Either the 5 seconds are up or a disconnect event was */
		/* received. Reset the peer in the event the 5 seconds   */
		/* had run out without any significant event.            */
		enet_peer_reset(peer);

		fprintf(stderr, "Connection to server failed.");
		exit(EXIT_FAILURE);
	}

	while (true)
	{
		serviceResult = 1;

		/* Keep doing host_service until no events are left */
		while (serviceResult > 0)
		{
			serviceResult = enet_host_service(client, &event, 0);

			if (serviceResult > 0)
			{
				switch (event.type)
				{
					case ENET_EVENT_TYPE_CONNECT:
						printf("A new client connected from %x:%u.\n",
							   event.peer->address.host,
							   event.peer->address.port);

						event.peer->data = (void *)"New User";
						break;

					case ENET_EVENT_TYPE_RECEIVE:
						printf(
							"A packet of length %u containing '%s' was "
							"received from %s on channel %u.\n",
							event.packet->dataLength,
							event.packet->data,
							event.peer->data,
							event.channelID);

						/* Clean up the packet now that we're done using it.
						> */
						enet_packet_destroy(event.packet);

						break;

					case ENET_EVENT_TYPE_DISCONNECT:
						printf("%s disconnected.\n", event.peer->data);

						break;
				}
			}
			else if (serviceResult > 0)
			{
				puts("Error with servicing the client");
				exit(EXIT_FAILURE);
			}
		}

		printf("Say> ");
#ifdef _WIN32
		gets_s(message, 1024);
#else
		fgets(message, 1024, stdin);
#endif
		if (strcmp(message, "exit") == 0 ||
			strcmp(message, "quit") == 0)
		{
			break;
		}

		if (strlen(message) > 0)
		{
			ENetPacket *packet = enet_packet_create(message, strlen(message) + 1, ENET_PACKET_FLAG_RELIABLE);
			enet_peer_send(peer, 0, packet);
		}
	}

	enet_peer_disconnect(peer, 0);

	/* Allow up to 3 seconds for the disconnect to succeed */
	/* and drop any packets received packets */
	while (enet_host_service(client, &event, 3000) > 0)
	{
		switch (event.type)
		{
			case ENET_EVENT_TYPE_RECEIVE:
				enet_packet_destroy(event.packet);
				break;

			case ENET_EVENT_TYPE_DISCONNECT:
				puts("Disconnection succeeded.");
				break;
		}
	}

	enet_host_destroy(client);
	enet_deinitialize();

	return 0;
}
