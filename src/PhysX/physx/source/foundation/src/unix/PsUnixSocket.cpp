//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "foundation/PxIntrinsics.h"

#include "PsSocket.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#if !PX_PS4
#include <netdb.h>
#include <arpa/inet.h>
#else
#include <ps4/PsPS4Socket.h>
#endif
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#define INVALID_SOCKET -1

#ifndef SOMAXCONN
#define SOMAXCONN 5
#endif

namespace physx
{
namespace shdfnd
{

const uint32_t Socket::DEFAULT_BUFFER_SIZE = 32768;

class SocketImpl
{
  public:
	SocketImpl(bool isBlocking);
	virtual ~SocketImpl();

	bool connect(const char* host, uint16_t port, uint32_t timeout);
	bool listen(uint16_t port);
	bool accept(bool block);
	void disconnect();

	void setBlocking(bool blocking);

	virtual uint32_t write(const uint8_t* data, uint32_t length);
	virtual bool flush();
	uint32_t read(uint8_t* data, uint32_t length);

	PX_FORCE_INLINE bool isBlocking() const
	{
		return mIsBlocking;
	}
	PX_FORCE_INLINE bool isConnected() const
	{
		return mIsConnected;
	}
	PX_FORCE_INLINE const char* getHost() const
	{
		return mHost;
	}
	PX_FORCE_INLINE uint16_t getPort() const
	{
		return mPort;
	}

  protected:
	bool nonBlockingTimeout() const;

	int32_t mSocket;
	int32_t mListenSocket;
	const char* mHost;
	uint16_t mPort;
	bool mIsConnected;
	bool mIsBlocking;
	bool mListenMode;
};

void socketSetBlockingInternal(int32_t socket, bool blocking);

SocketImpl::SocketImpl(bool isBlocking)
: mSocket(INVALID_SOCKET)
, mListenSocket(INVALID_SOCKET)
, mHost(NULL)
, mPort(0)
, mIsConnected(false)
, mIsBlocking(isBlocking)
, mListenMode(false)
{
}

SocketImpl::~SocketImpl()
{
}

bool SocketImpl::connect(const char* host, uint16_t port, uint32_t timeout)
{
	sockaddr_in socketAddress;
	intrinsics::memSet(&socketAddress, 0, sizeof(sockaddr_in));
	socketAddress.sin_family = AF_INET;
	socketAddress.sin_port = htons(port);

#if PX_PS4
	socketAddress.sin_addr.s_addr = resolveName(host, timeout);
#else
	// get host
	hostent* hp = gethostbyname(host);
	if(!hp)
	{
		in_addr a;
		a.s_addr = inet_addr(host);
		hp = gethostbyaddr(reinterpret_cast<const char*>(&a), sizeof(in_addr), AF_INET);
		if(!hp)
			return false;
	}
	intrinsics::memCopy(&socketAddress.sin_addr, hp->h_addr_list[0], hp->h_length);
#endif
	// connect
	mSocket = socket(AF_INET, SOCK_STREAM, 0);
	if(mSocket == INVALID_SOCKET)
		return false;

	socketSetBlockingInternal(mSocket, false);

	int connectRet = ::connect(mSocket, reinterpret_cast<sockaddr*>(&socketAddress), sizeof(socketAddress));
	if(connectRet < 0)
	{
		if(errno != EINPROGRESS)
		{
			disconnect();
			return false;
		}

		// Setup select function call to monitor the connect call.
		fd_set writefs;
		fd_set exceptfs;
		FD_ZERO(&writefs);
		FD_ZERO(&exceptfs);
		FD_SET(mSocket, &writefs);
		FD_SET(mSocket, &exceptfs);
		timeval timeout_;
		timeout_.tv_sec = timeout / 1000;
		timeout_.tv_usec = (timeout % 1000) * 1000;
		int selret = ::select(mSocket + 1, NULL, &writefs, &exceptfs, &timeout_);
		int excepted = FD_ISSET(mSocket, &exceptfs);
		int canWrite = FD_ISSET(mSocket, &writefs);
		if(selret != 1 || excepted || !canWrite)
		{
			disconnect();
			return false;
		}

		// check if we are really connected, above code seems to return
		// true if host is a unix machine even if the connection was
		// not accepted.
		char buffer;
		if(recv(mSocket, &buffer, 0, 0) < 0)
		{
			if(errno != EWOULDBLOCK)
			{
				disconnect();
				return false;
			}
		}
	}

	socketSetBlockingInternal(mSocket, mIsBlocking);

#if PX_APPLE_FAMILY
	int noSigPipe = 1;
	setsockopt(mSocket, SOL_SOCKET, SO_NOSIGPIPE, &noSigPipe, sizeof(int));
#endif

	mIsConnected = true;
	mPort = port;
	mHost = host;
	return true;
}

bool SocketImpl::listen(uint16_t port)
{
	mListenSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(mListenSocket == INVALID_SOCKET)
		return false;

	// enable address reuse: "Address already in use" error message
	int yes = 1;
	if(setsockopt(mListenSocket, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1)
		return false;

	mListenMode = true;

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = INADDR_ANY;
	intrinsics::memSet(addr.sin_zero, '\0', sizeof addr.sin_zero);

	return bind(mListenSocket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != -1 &&
	       ::listen(mListenSocket, SOMAXCONN) != -1;
}

bool SocketImpl::accept(bool block)
{
	if(mIsConnected || !mListenMode)
		return false;

	// set the listen socket to be non-blocking.
	socketSetBlockingInternal(mListenSocket, block);
	int32_t clientSocket = ::accept(mListenSocket, 0, 0);
	if(clientSocket == INVALID_SOCKET)
		return false;

	mSocket = clientSocket;
	mIsConnected = true;
	socketSetBlockingInternal(mSocket, mIsBlocking); // force the mode to whatever the user set

	return mIsConnected;
}

void SocketImpl::disconnect()
{
	if(mListenSocket != INVALID_SOCKET)
	{
		close(mListenSocket);
		mListenSocket = INVALID_SOCKET;
	}
	if(mSocket != INVALID_SOCKET)
	{
		if(mIsConnected)
		{
			socketSetBlockingInternal(mSocket, true);
			shutdown(mSocket, SHUT_RDWR);
		}
		close(mSocket);
		mSocket = INVALID_SOCKET;
	}
	mIsConnected = false;
	mListenMode = false;
	mPort = 0;
	mHost = NULL;
}

bool SocketImpl::nonBlockingTimeout() const
{
	return !mIsBlocking && errno == EWOULDBLOCK;
}

#if !PX_PS4
void socketSetBlockingInternal(int32_t socket, bool blocking)
{
	int mode = fcntl(socket, F_GETFL, 0);
	if(!blocking)
		mode |= O_NONBLOCK;
	else
		mode &= ~O_NONBLOCK;
	fcntl(socket, F_SETFL, mode);
}
#endif

// should be cross-platform from here down

void SocketImpl::setBlocking(bool blocking)
{
	if(blocking != mIsBlocking)
	{
		mIsBlocking = blocking;
		if(isConnected())
			socketSetBlockingInternal(mSocket, blocking);
	}
}

bool SocketImpl::flush()
{
	return true;
}

uint32_t SocketImpl::write(const uint8_t* data, uint32_t length)
{
	if(length == 0)
		return 0;

	int sent = send(mSocket, reinterpret_cast<const char*>(data), int32_t(length), 0);

	if(sent <= 0 && !nonBlockingTimeout())
		disconnect();

	return uint32_t(sent > 0 ? sent : 0);
}

uint32_t SocketImpl::read(uint8_t* data, uint32_t length)
{
	if(length == 0)
		return 0;

	int32_t received = recv(mSocket, reinterpret_cast<char*>(data), int32_t(length), 0);

	if(received <= 0 && !nonBlockingTimeout())
		disconnect();

	return uint32_t(received > 0 ? received : 0);
}

class BufferedSocketImpl : public SocketImpl
{
  public:
	BufferedSocketImpl(bool isBlocking) : SocketImpl(isBlocking), mBufferPos(0)
	{
	}
	virtual ~BufferedSocketImpl()
	{
	}
	bool flush();
	uint32_t write(const uint8_t* data, uint32_t length);

  private:
	uint32_t mBufferPos;
	uint8_t mBuffer[Socket::DEFAULT_BUFFER_SIZE];
};

bool BufferedSocketImpl::flush()
{
	uint32_t totalBytesWritten = 0;

	while(totalBytesWritten < mBufferPos && mIsConnected)
		totalBytesWritten += int32_t(SocketImpl::write(mBuffer + totalBytesWritten, mBufferPos - totalBytesWritten));

	bool ret = (totalBytesWritten == mBufferPos);
	mBufferPos = 0;
	return ret;
}

uint32_t BufferedSocketImpl::write(const uint8_t* data, uint32_t length)
{
	uint32_t bytesWritten = 0;
	while(mBufferPos + length >= Socket::DEFAULT_BUFFER_SIZE)
	{
		uint32_t currentChunk = Socket::DEFAULT_BUFFER_SIZE - mBufferPos;
		intrinsics::memCopy(mBuffer + mBufferPos, data + bytesWritten, currentChunk);
		bytesWritten += uint32_t(currentChunk); // for the user, this is consumed even if we fail to shove it down a
		// non-blocking socket

		uint32_t sent = SocketImpl::write(mBuffer, Socket::DEFAULT_BUFFER_SIZE);
		mBufferPos = Socket::DEFAULT_BUFFER_SIZE - sent;

		if(sent < Socket::DEFAULT_BUFFER_SIZE) // non-blocking or error
		{
			if(sent) // we can reasonably hope this is rare
				intrinsics::memMove(mBuffer, mBuffer + sent, mBufferPos);

			return bytesWritten;
		}
		length -= currentChunk;
	}

	if(length > 0)
	{
		intrinsics::memCopy(mBuffer + mBufferPos, data + bytesWritten, length);
		bytesWritten += length;
		mBufferPos += length;
	}

	return bytesWritten;
}

Socket::Socket(bool inIsBuffering, bool isBlocking)
{
	if(inIsBuffering)
	{
		void* mem = PX_ALLOC(sizeof(BufferedSocketImpl), "BufferedSocketImpl");
		mImpl = PX_PLACEMENT_NEW(mem, BufferedSocketImpl)(isBlocking);
	}
	else
	{
		void* mem = PX_ALLOC(sizeof(SocketImpl), "SocketImpl");
		mImpl = PX_PLACEMENT_NEW(mem, SocketImpl)(isBlocking);
	}
}

Socket::~Socket()
{
	mImpl->flush();
	mImpl->disconnect();
	mImpl->~SocketImpl();
	PX_FREE(mImpl);
}

bool Socket::connect(const char* host, uint16_t port, uint32_t timeout)
{
	return mImpl->connect(host, port, timeout);
}

bool Socket::listen(uint16_t port)
{
	return mImpl->listen(port);
}

bool Socket::accept(bool block)
{
	return mImpl->accept(block);
}

void Socket::disconnect()
{
	mImpl->disconnect();
}

bool Socket::isConnected() const
{
	return mImpl->isConnected();
}

const char* Socket::getHost() const
{
	return mImpl->getHost();
}

uint16_t Socket::getPort() const
{
	return mImpl->getPort();
}

bool Socket::flush()
{
	if(!mImpl->isConnected())
		return false;
	return mImpl->flush();
}

uint32_t Socket::write(const uint8_t* data, uint32_t length)
{
	if(!mImpl->isConnected())
		return 0;
	return mImpl->write(data, length);
}

uint32_t Socket::read(uint8_t* data, uint32_t length)
{
	if(!mImpl->isConnected())
		return 0;
	return mImpl->read(data, length);
}

void Socket::setBlocking(bool blocking)
{
	mImpl->setBlocking(blocking);
}

bool Socket::isBlocking() const
{
	return mImpl->isBlocking();
}

} // namespace shdfnd
} // namespace physx
