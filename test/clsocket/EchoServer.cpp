
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

    socket.Listen("127.0.0.1", 6789);

    while (true)
    {
        if ((pClient = socket.Accept()) != NULL)
        {
            //----------------------------------------------------------------------
            // Receive request from the client.
            //----------------------------------------------------------------------
            if (pClient->Receive(MAX_PACKET))
            {
                //------------------------------------------------------------------
                // Send response to client and close connection to the client.
                //------------------------------------------------------------------
                pClient->Send( pClient->GetData(), pClient->GetBytesReceived() );
                pClient->Close();
            }

            delete pClient;
        }
    }

    //-----------------------------------------------------------------------------
    // Receive request from the client.
    //-----------------------------------------------------------------------------
    socket.Close();

    return 1;
}
