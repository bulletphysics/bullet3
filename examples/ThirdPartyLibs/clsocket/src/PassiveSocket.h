/*---------------------------------------------------------------------------*/
/*                                                                           */
/* Socket.h - Passive Socket Decleration.                                    */
/*                                                                           */
/* Author : Mark Carrier (mark@carrierlabs.com)                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/* Copyright (c) 2007-2009 CarrierLabs, LLC.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * 4. The name "CarrierLabs" must not be used to
 *    endorse or promote products derived from this software without
 *    prior written permission. For written permission, please contact
 *    mark@carrierlabs.com.
 *
 * THIS SOFTWARE IS PROVIDED BY MARK CARRIER ``AS IS'' AND ANY
 * EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL MARK CARRIER OR
 * ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *----------------------------------------------------------------------------*/
#ifndef __PASSIVESOCKET_H__
#define __PASSIVESOCKET_H__
#include "ActiveSocket.h"

/// Provides a platform independent class to create a passive socket.
/// A passive socket is used to create a "listening" socket.  This type
/// of object would be used when an application needs to wait for
/// inbound connections.  Support for CSimpleSocket::SocketTypeTcp,
/// CSimpleSocket::SocketTypeUdp, and CSimpleSocket::SocketTypeRaw is handled
/// in a similar fashion.  The big difference is that the method
/// CPassiveSocket::Accept should not be called on the latter two socket
/// types.
class CPassiveSocket : public CSimpleSocket {
public:
    CPassiveSocket(CSocketType type = SocketTypeTcp);
    virtual ~CPassiveSocket() {
        Close();
    };

    /// Extracts the first connection request on the queue of pending
    /// connections and creates a newly connected socket.  Used with
    /// CSocketType CSimpleSocket::SocketTypeTcp.  It is the responsibility of
    /// the caller to delete the returned object when finished.
    ///  @return if successful a pointer to a newly created CActiveSocket object
    ///          will be returned and the internal error condition of the CPassiveSocket
    ///          object will be CPassiveSocket::SocketSuccess.  If an error condition was encountered
    ///          the NULL will be returned and one of the following error conditions will be set:
    ///    CPassiveSocket::SocketEwouldblock, CPassiveSocket::SocketInvalidSocket,
    ///    CPassiveSocket::SocketConnectionAborted, CPassiveSocket::SocketInterrupted
    ///    CPassiveSocket::SocketProtocolError, CPassiveSocket::SocketFirewallError
    virtual CActiveSocket *Accept(void);

    /// Bind to a multicast group on  a specified interface, multicast group, and port
    ///
    ///  @param pInterface - interface on which to bind.
    ///  @param pGroup - multicast group address to bind.
    ///  @param nPort - port on which multicast
    ///  @return true if able to bind to interface and multicast group.
    ///      If not successful, the false is returned and one of the following error
    ///      condiitions will be set: CPassiveSocket::SocketAddressInUse, CPassiveSocket::SocketProtocolError,
    ///      CPassiveSocket::SocketInvalidSocket.  The following socket errors are for Linux/Unix
    ///      derived systems only: CPassiveSocket::SocketInvalidSocketBuffer
    bool BindMulticast(const char *pInterface, const char *pGroup, uint16 nPort);

    /// Create a listening socket at local ip address 'x.x.x.x' or 'localhost'
    /// if pAddr is NULL on port nPort.
    ///
    ///  @param pAddr specifies the IP address on which to listen.
    ///  @param nPort specifies the port on which to listen.
    ///  @param nConnectionBacklog specifies connection queue backlog (default 30,000)
    ///  @return true if a listening socket was created.
    ///      If not successful, the false is returned and one of the following error
    ///      conditions will be set: CPassiveSocket::SocketAddressInUse, CPassiveSocket::SocketProtocolError,
    ///      CPassiveSocket::SocketInvalidSocket.  The following socket errors are for Linux/Unix
    ///      derived systems only: CPassiveSocket::SocketInvalidSocketBuffer
    virtual bool Listen(const char *pAddr, uint16 nPort, int32 nConnectionBacklog = 30000);

    /// Attempts to send a block of data on an established connection.
    /// @param pBuf block of data to be sent.
    /// @param bytesToSend size of data block to be sent.
    /// @return number of bytes actually sent, return of zero means the
    /// connection has been shutdown on the other side, and a return of -1
    /// means that an error has occurred.  If an error was signaled then one
    /// of the following error codes will be set: CPassiveSocket::SocketInvalidSocket,
    /// CPassiveSocket::SocketEwouldblock, SimpleSocket::SocketConnectionReset,
    /// CPassiveSocket::SocketInvalidSocketBuffer, CPassiveSocket::SocketInterrupted,
    /// CPassiveSocket::SocketProtocolError, CPassiveSocket::SocketNotconnected
    /// <br>\b Note: This function is used only for a socket of type
    /// CSimpleSocket::SocketTypeUdp
    virtual int32 Send(const uint8 *pBuf, size_t bytesToSend);

private:
    struct ip_mreq  m_stMulticastRequest;   /// group address for multicast

};

#endif // __PASSIVESOCKET_H__
