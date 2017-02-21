
#include "PassiveSocket.h"       // Include header for active socket object definition

#define MAX_PACKET 4096 

int main(int argc, char **argv)
{
    CPassiveSocket socket;
    CActiveSocket *pClient = NULL;

    //--------------------------------------------------------------------------
    // Initialize our socket object 
    //--------------------------------------------------------------------------
    socket.Initialize();

    socket.Listen("localhost", 6667);

    while (true)
    {
        if ((pClient = socket.Accept()) != NULL)
        {
			int clientPort = socket.GetClientPort();
			printf("connected from %s:%d\n", socket.GetClientAddr(),clientPort);
            //----------------------------------------------------------------------
            // Receive request from the client.
            //----------------------------------------------------------------------
            while (1)
			{
				//printf("try receive\n");
				bool receivedData = false;
				int recBytes = 0;
				recBytes = pClient->Receive(MAX_PACKET);
				if (recBytes)
				{
					char* msg = (char*) pClient->GetData();
					msg[recBytes]=0;
					printf("received message [%s]\n",msg);
					//------------------------------------------------------------------
					// Send response to client and close connection to the client.
					//------------------------------------------------------------------
					pClient->Send( pClient->GetData(), pClient->GetBytesReceived() );
					receivedData = true;
					if (strncmp(msg,"stop",4)==0)
					{
						printf("Stop request received\n");
						break;
					}
				}
				if (!receivedData)
				{
					printf("Didn't receive data.\n");
					break;
				} 
			}
			printf("Disconnecting client.\n");
			pClient->Close();
            delete pClient;
        }
    }

    //-----------------------------------------------------------------------------
    // Receive request from the client.
    //-----------------------------------------------------------------------------
    socket.Close();

    return 1;
}
