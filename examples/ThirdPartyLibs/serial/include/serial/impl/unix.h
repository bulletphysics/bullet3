/*!
 * \file serial/impl/unix.h
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
 * This provides a unix based pimpl for the Serial class. This implementation is
 * based off termios.h and uses select for multiplexing the IO ports.
 *
 */

#if !defined(_WIN32)

#ifndef SERIAL_IMPL_UNIX_H
#define SERIAL_IMPL_UNIX_H

#include "serial/serial.h"

#include <pthread.h>

namespace serial {

using std::size_t;
using std::string;
using std::invalid_argument;

using serial::SerialException;
using serial::IOException;

class MillisecondTimer {
public:
  MillisecondTimer(const uint32_t millis);         
  int64_t remaining();

private:
  static timespec timespec_now();
  timespec expiry;
};

class serial::Serial::SerialImpl {
public:
  SerialImpl (const string &port,
              unsigned long baudrate,
              bytesize_t bytesize,
              parity_t parity,
              stopbits_t stopbits,
              flowcontrol_t flowcontrol);

  virtual ~SerialImpl ();

  void
  open ();

  void
  close ();

  bool
  isOpen () const;

  size_t
  available ();

  bool
  waitReadable (uint32_t timeout);

  void
  waitByteTimes (size_t count);

  size_t
  read (uint8_t *buf, size_t size = 1);

  size_t
  write (const uint8_t *data, size_t length);

  void
  flush ();

  void
  flushInput ();

  void
  flushOutput ();

  void
  sendBreak (int duration);

  void
  setBreak (bool level);

  void
  setRTS (bool level);

  void
  setDTR (bool level);

  bool
  waitForChange ();

  bool
  getCTS ();

  bool
  getDSR ();

  bool
  getRI ();

  bool
  getCD ();

  void
  setPort (const string &port);

  string
  getPort () const;

  void
  setTimeout (Timeout &timeout);

  Timeout
  getTimeout () const;

  void
  setBaudrate (unsigned long baudrate);

  unsigned long
  getBaudrate () const;

  void
  setBytesize (bytesize_t bytesize);

  bytesize_t
  getBytesize () const;

  void
  setParity (parity_t parity);

  parity_t
  getParity () const;

  void
  setStopbits (stopbits_t stopbits);

  stopbits_t
  getStopbits () const;

  void
  setFlowcontrol (flowcontrol_t flowcontrol);

  flowcontrol_t
  getFlowcontrol () const;

  void
  readLock ();

  void
  readUnlock ();

  void
  writeLock ();

  void
  writeUnlock ();

protected:
  void reconfigurePort ();

private:
  string port_;               // Path to the file descriptor
  int fd_;                    // The current file descriptor

  bool is_open_;
  bool xonxoff_;
  bool rtscts_;

  Timeout timeout_;           // Timeout for read operations
  unsigned long baudrate_;    // Baudrate
  uint32_t byte_time_ns_;     // Nanoseconds to transmit/receive a single byte

  parity_t parity_;           // Parity
  bytesize_t bytesize_;       // Size of the bytes
  stopbits_t stopbits_;       // Stop Bits
  flowcontrol_t flowcontrol_; // Flow Control

  // Mutex used to lock the read functions
  pthread_mutex_t read_mutex;
  // Mutex used to lock the write functions
  pthread_mutex_t write_mutex;
};

}

#endif // SERIAL_IMPL_UNIX_H

#endif // !defined(_WIN32)
