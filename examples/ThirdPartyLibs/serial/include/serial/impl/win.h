/*!
 * \file serial/impl/windows.h
 * \author  William Woodall <wjwwood@gmail.com>
 * \author  John Harrison <ash@greaterthaninfinity.com>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The MIT License
 *
 * Copyright (c) 2012 William Woodall, John Harrison
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides a windows implementation of the Serial class interface.
 *
 */

#if defined(_WIN32)

#ifndef SERIAL_IMPL_WINDOWS_H
#define SERIAL_IMPL_WINDOWS_H

#include "serial/serial.h"

#include "windows.h"

namespace serial
{
using std::invalid_argument;
using std::string;
using std::wstring;

using serial::IOException;
using serial::SerialException;

class serial::Serial::SerialImpl
{
public:
	SerialImpl(const string &port,
			   unsigned long baudrate,
			   bytesize_t bytesize,
			   parity_t parity,
			   stopbits_t stopbits,
			   flowcontrol_t flowcontrol);

	virtual ~SerialImpl();

	void
	open();

	void
	close();

	bool
	isOpen() const;

	size_t
	available();

	bool
	waitReadable(uint32_t timeout);

	void
	waitByteTimes(size_t count);

	size_t
	read(uint8_t *buf, size_t size = 1);

	size_t
	write(const uint8_t *data, size_t length);

	void
	flush();

	void
	flushInput();

	void
	flushOutput();

	void
	sendBreak(int duration);

	void
	setBreak(bool level);

	void
	setRTS(bool level);

	void
	setDTR(bool level);

	bool
	waitForChange();

	bool
	getCTS();

	bool
	getDSR();

	bool
	getRI();

	bool
	getCD();

	void
	setPort(const string &port);

	string
	getPort() const;

	void
	setTimeout(Timeout &timeout);

	Timeout
	getTimeout() const;

	void
	setBaudrate(unsigned long baudrate);

	unsigned long
	getBaudrate() const;

	void
	setBytesize(bytesize_t bytesize);

	bytesize_t
	getBytesize() const;

	void
	setParity(parity_t parity);

	parity_t
	getParity() const;

	void
	setStopbits(stopbits_t stopbits);

	stopbits_t
	getStopbits() const;

	void
	setFlowcontrol(flowcontrol_t flowcontrol);

	flowcontrol_t
	getFlowcontrol() const;

	void
	readLock();

	void
	readUnlock();

	void
	writeLock();

	void
	writeUnlock();

protected:
	void reconfigurePort();

private:
	wstring port_;  // Path to the file descriptor
	HANDLE fd_;

	bool is_open_;

	Timeout timeout_;         // Timeout for read operations
	unsigned long baudrate_;  // Baudrate

	parity_t parity_;            // Parity
	bytesize_t bytesize_;        // Size of the bytes
	stopbits_t stopbits_;        // Stop Bits
	flowcontrol_t flowcontrol_;  // Flow Control

	// Mutex used to lock the read functions
	HANDLE read_mutex;
	// Mutex used to lock the write functions
	HANDLE write_mutex;
};

}  // namespace serial

#endif  // SERIAL_IMPL_WINDOWS_H

#endif  // if defined(_WIN32)
