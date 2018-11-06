
#include <stdio.h>
#include <enet/enet.h>
#include <string.h>

ENetPeer* mypeers[2] = {0, 0};
ENetAddress clientAddresses[2];
int numpeers = 0;

int main(int argc, char* argv[])
{
	fprintf(stderr, "starting enet dedicated server\n");

	if (enet_initialize() != 0)
	{
		fprintf(stderr, "An error occurred while initializing ENet.\n");
		return EXIT_FAILURE;
	}
	atexit(enet_deinitialize);

	ENetAddress address;
	ENetHost* server;
	/* Bind the server to the default localhost.     */
	/* A specific host address can be specified by   */
	/* enet_address_set_host (& address, "x.x.x.x"); */
	address.host = ENET_HOST_ANY;
	/* Bind the server to port 1234. */
	address.port = 1234;
	server = enet_host_create(&address /* the address to bind the server host to */,
							  32 /* allow up to 32 clients and/or outgoing connections */,
							  2 /* allow up to 2 channels to be used, 0 and 1 */,
							  0 /* assume any amount of incoming bandwidth */,
							  0 /* assume any amount of outgoing bandwidth */);
	if (server == NULL)
	{
		fprintf(stderr,
				"An error occurred while trying to create an ENet server host.\n");
		exit(EXIT_FAILURE);
	}

	ENetEvent event;

	/* Wait up to 10000000 milliseconds for an event. */
	while (enet_host_service(server, &event, 10000000) > 0)
	{
		switch (event.type)
		{
			case ENET_EVENT_TYPE_CONNECT:
				char clientname[1024];
				enet_address_get_host(&event.peer->address, clientname, 1024);
				printf("A new client connected from %s:%u.\n",
					   clientname,
					   event.peer->address.port);
				/* Store any relevant client information here. */
				event.peer->data = (char*)"Client information";
				if (numpeers < 2)
				{
					clientAddresses[numpeers] = event.peer->address;
					mypeers[numpeers] = event.peer;
				}
				numpeers++;
				if (numpeers == 2)
				{
					printf("exchanging addresses for NAT punchthrough\n");
					//exchange the address info
					for (int i = 0; i < 2; i++)
					{
						int sz = sizeof(ENetAddress);
						/* Create a reliable packet of size 7 containing "packet\0" */
						ENetPacket* packet = enet_packet_create(&clientAddresses[i],
																sz,
																ENET_PACKET_FLAG_RELIABLE);
						enet_peer_send(mypeers[1 - i], 0, packet);
					}
					//prepare for the next pair of clients to connect/NAT punchthrough
					numpeers = 0;
				}

				break;
			case ENET_EVENT_TYPE_RECEIVE:
				printf("A packet of length %u containing %s was received from %s on channel %u.\n",
					   event.packet->dataLength,
					   event.packet->data,
					   event.peer->data,
					   event.channelID);
				/* Clean up the packet now that we're done using it. */
				enet_packet_destroy(event.packet);

				break;

			case ENET_EVENT_TYPE_DISCONNECT:
				printf("%s disconnected.\n", event.peer->data);
				/* Reset the peer's client information. */
				event.peer->data = NULL;
		}
	}

	enet_host_destroy(server);
	printf("server exited, press <enter> key\n");
	getchar();

	return 0;
}
