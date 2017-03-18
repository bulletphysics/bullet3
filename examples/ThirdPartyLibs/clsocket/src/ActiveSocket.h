/*---------------------------------------------------------------------------*/
/*                                                                           */
/* ActiveSocket.h - Active Socket Decleration                                */
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
#ifndef __ACTIVESOCKET_H__
#define __ACTIVESOCKET_H__

#include "SimpleSocket.h"

class CPassiveSocket;

/// Provides a platform independent class to create an active socket.
/// An active socket is used to create a socket which connects to a server.
/// This type of object would be used when an application needs to send/receive
/// data from a server.
class CActiveSocket : public CSimpleSocket {
public:
    friend class CPassiveSocket;

    CActiveSocket(CSocketType type = SocketTypeTcp);
    virtual ~CActiveSocket() {
        Close();
    };

    /// Established a connection to the address specified by pAddr.
    /// Connection-based protocol sockets (CSocket::SocketTypeTcp) may
    /// successfully call Open() only once, however; connectionless protocol
    /// sockets (CSocket::SocketTypeUdp) may use Open() multiple times to
    /// change their association.
    ///  @param pAddr specifies the destination address to connect.
    ///  @param nPort specifies the destination port.
    ///  @return true if successful connection made, otherwise false.
    virtual bool Open(const char *pAddr, uint16 nPort);

private:
    /// Utility function used to create a TCP connection, called from Open().
    ///  @return true if successful connection made, otherwise false.
    bool ConnectTCP(const char *pAddr, uint16 nPort);

    /// Utility function used to create a UDP connection, called from Open().
    ///  @return true if successful connection made, otherwise false.
    bool ConnectUDP(const char *pAddr, uint16 nPort);

    /// Utility function used to create a RAW connection, called from Open().
    ///  @return true if successful connection made, otherwise false.
    bool ConnectRAW(const char *pAddr, uint16 nPort);

private:
    struct hostent *m_pHE;
};

#endif /*  __ACTIVESOCKET_H__  */

