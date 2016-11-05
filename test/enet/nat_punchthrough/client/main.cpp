





#include <stdio.h>
#include <enet/enet.h>
#include <string.h>


int main(int argc, char* argv[])
{
	printf("starting client (and server)\n");

    if (enet_initialize () != 0)
    {
        fprintf (stderr, "An error occurred while initializing ENet.\n");
        return EXIT_FAILURE;
    }
    atexit (enet_deinitialize);

	ENetAddress selfaddress;
	selfaddress.host = ENET_HOST_ANY;
	/* Bind the server to port 1111. */
	selfaddress.port = 1111;

	ENetHost * client=0;
	while (!client)
	{
		client = enet_host_create (&selfaddress/* create a client host */,
					32 /* only 32 connections */,
					2 /* allow up 2 channels to be used, 0 and 1 */,
					0/*57600 / 8  56K modem with 56 Kbps downstream bandwidth */,
					0 /* 14400 / 8  56K modem with 14 Kbps upstream bandwidth */);
		if (client == NULL)
		{
			selfaddress.port++;
		}
	}
	if (client == NULL)
	{
		fprintf (stderr, 
				 "An error occurred while trying to create an ENet client host.\n");
		exit (EXIT_FAILURE);
	}



	ENetAddress dedicatedserveraddress;
	ENetEvent event;
	ENetPeer* dedicatedpeer=0;
	ENetPeer* natpeer=0;

	/* Connect to some.server.net:1234. */
	enet_address_set_host (& dedicatedserveraddress, "localhost");
	dedicatedserveraddress.port = 1234;
	/* Initiate the connection, allocating the two channels 0 and 1. */
	dedicatedpeer = enet_host_connect (client, & dedicatedserveraddress, 2, 0);    
	if (dedicatedpeer == NULL)
	{
	   fprintf (stderr, "No available peers for initiating an ENet connection.\n");
	   exit (EXIT_FAILURE);
	}
	/* Wait up to 5 seconds for the connection attempt to succeed. */
	if (enet_host_service (client, & event, 5000) > 0 &&
		event.type == ENET_EVENT_TYPE_CONNECT)
	{
		char servername[1024];
		enet_address_get_host(&dedicatedserveraddress,servername, 1024);
		char serverinfo[1024];

		sprintf(serverinfo,"Connection to %s:%d succeeded", servername,dedicatedserveraddress.port);
		puts (serverinfo);

		/////....

		/* Wait up to 1000 milliseconds for an event. */
		while (enet_host_service (client, & event, 1000000000) > 0)
		{
			if (natpeer)
			{
					/* Create a reliable packet of size 7 containing "packet\0" */
					ENetPacket * packet = enet_packet_create ("packet", 
															  strlen ("packet") + 1, 
															  ENET_PACKET_FLAG_RELIABLE);
					/* Extend the packet so and append the string "foo", so it now */
					/* contains "packetfoo\0"                                      */
					enet_packet_resize (packet, strlen ("packetfoo") + 1);
					strcpy ((char*)& packet -> data [strlen ("packet")], "foo");
					/* Send the packet to the peer over channel id 0. */
					/* One could also broadcast the packet by         */
					/* enet_host_broadcast (host, 0, packet);         */
					enet_peer_send (natpeer, 0, packet);
			}
			switch (event.type)
			{
			case ENET_EVENT_TYPE_CONNECT:
				printf ("A new client connected from %x:%u.\n", 
						event.peer -> address.host,
						event.peer -> address.port);
				/* Store any relevant client information here. */
				event.peer -> data = (char*)"Client information";
				break;
			case ENET_EVENT_TYPE_RECEIVE:
				printf ("A packet of length %u containing %s was received from %s on channel %u.\n",
						event.packet -> dataLength,
						event.packet -> data,
						event.peer -> data,
						event.channelID);
				/* Clean up the packet now that we're done using it. */

				if (event.packet->dataLength==sizeof(ENetAddress))
				{
					ENetAddress* address = (ENetAddress*)event.packet->data;
					printf("received other client's address from server, connecting...\n");
					natpeer = enet_host_connect (client, address, 2, 0);    
					if (natpeer== NULL)
					{
						fprintf (stderr, "No available peers for initiating an ENet connection.\n");
						exit (EXIT_FAILURE);
					}
					/* Wait up to 5 seconds for the connection attempt to succeed. */
					if (enet_host_service (client, & event, 5000) > 0 &&
						event.type == ENET_EVENT_TYPE_CONNECT)
					{
						puts ("Connection to natpeer succeeded.");
					} else
					{
						enet_peer_reset (natpeer);
						puts ("Connection to natpeer failed.");
						natpeer=0;
						exit(0);
					}

				}

				enet_packet_destroy (event.packet);
				break;
       
			case ENET_EVENT_TYPE_DISCONNECT:
				printf ("%s disconected.\n", event.peer -> data);
				/* Reset the peer's client information. */
				event.peer -> data = NULL;
			}
		}
		/* One could just use enet_host_service() instead. */
		enet_host_flush (client);//host);
	}
	else
	{
		/* Either the 5 seconds are up or a disconnect event was */
		/* received. Reset the peer in the event the 5 seconds   */
		/* had run out without any significant event.            */
		enet_peer_reset (dedicatedpeer);
		puts ("Connection to some.server.net:1234 failed.");
	}


	



	enet_host_destroy(client);
	
	return 0;
}












