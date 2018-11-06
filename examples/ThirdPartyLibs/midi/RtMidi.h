/**********************************************************************/
/*! \class RtMidi
    \brief An abstract base class for realtime MIDI input/output.

    This class implements some common functionality for the realtime
    MIDI input/output subclasses RtMidiIn and RtMidiOut.

    RtMidi WWW site: http://music.mcgill.ca/~gary/rtmidi/

    RtMidi: realtime MIDI i/o C++ classes
    Copyright (c) 2003-2012 Gary P. Scavone

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation files
    (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software,
    and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    Any person wishing to distribute modifications to the Software is
    asked to send the modifications to the original developer so that
    they can be incorporated into the canonical version.  This is,
    however, not a binding provision of this license.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
    ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
    CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
    WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/**********************************************************************/

/*!
  \file RtMidi.h
 */

// RtMidi: Version 2.0.1

#ifndef RTMIDI_H
#define RTMIDI_H

#include "RtError.h"
#include <string>
#include <vector>

class RtMidi
{
public:
	//! MIDI API specifier arguments.
	enum Api
	{
		UNSPECIFIED, /*!< Search for a working compiled API. */
		MACOSX_CORE, /*!< Macintosh OS-X Core Midi API. */
		LINUX_ALSA,  /*!< The Advanced Linux Sound Architecture API. */
		UNIX_JACK,   /*!< The Jack Low-Latency MIDI Server API. */
		WINDOWS_MM,  /*!< The Microsoft Multimedia MIDI API. */
		WINDOWS_KS,  /*!< The Microsoft Kernel Streaming MIDI API. */
		RTMIDI_DUMMY /*!< A compilable but non-functional API. */
	};

	//! A static function to determine the available compiled MIDI APIs.
	/*!
    The values returned in the std::vector can be compared against
    the enumerated list values.  Note that there can be more than one
    API compiled for certain operating systems.
  */
	static void getCompiledApi(std::vector<RtMidi::Api> &apis);

	//! Pure virtual openPort() function.
	virtual void openPort(unsigned int portNumber = 0, const std::string portName = std::string("RtMidi")) = 0;

	//! Pure virtual openVirtualPort() function.
	virtual void openVirtualPort(const std::string portName = std::string("RtMidi")) = 0;

	//! Pure virtual getPortCount() function.
	virtual unsigned int getPortCount() = 0;

	//! Pure virtual getPortName() function.
	virtual std::string getPortName(unsigned int portNumber = 0) = 0;

	//! Pure virtual closePort() function.
	virtual void closePort(void) = 0;

	//! A basic error reporting function for RtMidi classes.
	static void error(RtError::Type type, std::string errorString);

protected:
	RtMidi(){};
	virtual ~RtMidi(){};
};

/**********************************************************************/
/*! \class RtMidiIn
    \brief A realtime MIDI input class.

    This class provides a common, platform-independent API for
    realtime MIDI input.  It allows access to a single MIDI input
    port.  Incoming MIDI messages are either saved to a queue for
    retrieval using the getMessage() function or immediately passed to
    a user-specified callback function.  Create multiple instances of
    this class to connect to more than one MIDI device at the same
    time.  With the OS-X and Linux ALSA MIDI APIs, it is also possible
    to open a virtual input port to which other MIDI software clients
    can connect.

    by Gary P. Scavone, 2003-2012.
*/
/**********************************************************************/

// **************************************************************** //
//
// RtMidiIn and RtMidiOut class declarations.
//
// RtMidiIn / RtMidiOut are "controllers" used to select an available
// MIDI input or output interface.  They present common APIs for the
// user to call but all functionality is implemented by the classes
// MidiInApi, MidiOutApi and their subclasses.  RtMidiIn and RtMidiOut
// each create an instance of a MidiInApi or MidiOutApi subclass based
// on the user's API choice.  If no choice is made, they attempt to
// make a "logical" API selection.
//
// **************************************************************** //

class MidiInApi;
class MidiOutApi;

class RtMidiIn : public RtMidi
{
public:
	//! User callback function type definition.
	typedef void (*RtMidiCallback)(double timeStamp, std::vector<unsigned char> *message, void *userData);

	//! Default constructor that allows an optional api, client name and queue size.
	/*!
    An assert will be fired if a MIDI system initialization
    error occurs.  The queue size defines the maximum number of
    messages that can be held in the MIDI queue (when not using a
    callback function).  If the queue size limit is reached,
    incoming messages will be ignored.

    If no API argument is specified and multiple API support has been
    compiled, the default order of use is JACK, ALSA (Linux) and CORE,
    Jack (OS-X).
  */
	RtMidiIn(RtMidi::Api api = UNSPECIFIED,
			 const std::string clientName = std::string("RtMidi Input Client"),
			 unsigned int queueSizeLimit = 100);

	//! If a MIDI connection is still open, it will be closed by the destructor.
	~RtMidiIn(void);

	//! Returns the MIDI API specifier for the current instance of RtMidiIn.
	RtMidi::Api getCurrentApi(void);

	//! Open a MIDI input connection.
	/*!
    An optional port number greater than 0 can be specified.
    Otherwise, the default or first port found is opened.
  */
	void openPort(unsigned int portNumber = 0, const std::string portName = std::string("RtMidi Input"));

	//! Create a virtual input port, with optional name, to allow software connections (OS X and ALSA only).
	/*!
    This function creates a virtual MIDI input port to which other
    software applications can connect.  This type of functionality
    is currently only supported by the Macintosh OS-X and Linux ALSA
    APIs (the function does nothing for the other APIs).
  */
	void openVirtualPort(const std::string portName = std::string("RtMidi Input"));

	//! Set a callback function to be invoked for incoming MIDI messages.
	/*!
    The callback function will be called whenever an incoming MIDI
    message is received.  While not absolutely necessary, it is best
    to set the callback function before opening a MIDI port to avoid
    leaving some messages in the queue.
  */
	void setCallback(RtMidiCallback callback, void *userData = 0);

	//! Cancel use of the current callback function (if one exists).
	/*!
    Subsequent incoming MIDI messages will be written to the queue
    and can be retrieved with the \e getMessage function.
  */
	void cancelCallback();

	//! Close an open MIDI connection (if one exists).
	void closePort(void);

	//! Return the number of available MIDI input ports.
	unsigned int getPortCount();

	//! Return a string identifier for the specified MIDI input port number.
	/*!
    An empty string is returned if an invalid port specifier is provided.
  */
	std::string getPortName(unsigned int portNumber = 0);

	//! Specify whether certain MIDI message types should be queued or ignored during input.
	/*!
    o      By default, MIDI timing and active sensing messages are ignored
    during message input because of their relative high data rates.
    MIDI sysex messages are ignored by default as well.  Variable
    values of "true" imply that the respective message type will be
    ignored.
  */
	void ignoreTypes(bool midiSysex = true, bool midiTime = true, bool midiSense = true);

	//! Fill the user-provided vector with the data bytes for the next available MIDI message in the input queue and return the event delta-time in seconds.
	/*!
    This function returns immediately whether a new message is
    available or not.  A valid message is indicated by a non-zero
    vector size.  An assert is fired if an error occurs during
    message retrieval or an input connection was not previously
    established.
  */
	double getMessage(std::vector<unsigned char> *message);

protected:
	void openMidiApi(RtMidi::Api api, const std::string clientName, unsigned int queueSizeLimit);
	MidiInApi *rtapi_;
};

/**********************************************************************/
/*! \class RtMidiOut
    \brief A realtime MIDI output class.

    This class provides a common, platform-independent API for MIDI
    output.  It allows one to probe available MIDI output ports, to
    connect to one such port, and to send MIDI bytes immediately over
    the connection.  Create multiple instances of this class to
    connect to more than one MIDI device at the same time.  With the
    OS-X and Linux ALSA MIDI APIs, it is also possible to open a
    virtual port to which other MIDI software clients can connect.

    by Gary P. Scavone, 2003-2012.
*/
/**********************************************************************/

class RtMidiOut : public RtMidi
{
public:
	//! Default constructor that allows an optional client name.
	/*!
    An exception will be thrown if a MIDI system initialization error occurs.

    If no API argument is specified and multiple API support has been
    compiled, the default order of use is JACK, ALSA (Linux) and CORE,
    Jack (OS-X).
  */
	RtMidiOut(RtMidi::Api api = UNSPECIFIED,
			  const std::string clientName = std::string("RtMidi Output Client"));

	//! The destructor closes any open MIDI connections.
	~RtMidiOut(void);

	//! Returns the MIDI API specifier for the current instance of RtMidiOut.
	RtMidi::Api getCurrentApi(void);

	//! Open a MIDI output connection.
	/*!
      An optional port number greater than 0 can be specified.
      Otherwise, the default or first port found is opened.  An
      exception is thrown if an error occurs while attempting to make
      the port connection.
  */
	void openPort(unsigned int portNumber = 0, const std::string portName = std::string("RtMidi Output"));

	//! Close an open MIDI connection (if one exists).
	void closePort(void);

	//! Create a virtual output port, with optional name, to allow software connections (OS X and ALSA only).
	/*!
      This function creates a virtual MIDI output port to which other
      software applications can connect.  This type of functionality
      is currently only supported by the Macintosh OS-X and Linux ALSA
      APIs (the function does nothing with the other APIs).  An
      exception is thrown if an error occurs while attempting to create
      the virtual port.
  */
	void openVirtualPort(const std::string portName = std::string("RtMidi Output"));

	//! Return the number of available MIDI output ports.
	unsigned int getPortCount(void);

	//! Return a string identifier for the specified MIDI port type and number.
	/*!
      An empty string is returned if an invalid port specifier is provided.
  */
	std::string getPortName(unsigned int portNumber = 0);

	//! Immediately send a single message out an open MIDI output port.
	/*!
      An exception is thrown if an error occurs during output or an
      output connection was not previously established.
  */
	void sendMessage(std::vector<unsigned char> *message);

protected:
	void openMidiApi(RtMidi::Api api, const std::string clientName);
	MidiOutApi *rtapi_;
};

// **************************************************************** //
//
// MidiInApi / MidiOutApi class declarations.
//
// Subclasses of MidiInApi and MidiOutApi contain all API- and
// OS-specific code necessary to fully implement the RtMidi API.
//
// Note that MidiInApi and MidiOutApi are abstract base classes and
// cannot be explicitly instantiated.  RtMidiIn and RtMidiOut will
// create instances of a MidiInApi or MidiOutApi subclass.
//
// **************************************************************** //

class MidiInApi
{
public:
	MidiInApi(unsigned int queueSizeLimit);
	virtual ~MidiInApi(void);
	virtual RtMidi::Api getCurrentApi(void) = 0;
	virtual void openPort(unsigned int portNumber, const std::string portName) = 0;
	virtual void openVirtualPort(const std::string portName) = 0;
	virtual void closePort(void) = 0;
	void setCallback(RtMidiIn::RtMidiCallback callback, void *userData);
	void cancelCallback(void);
	virtual unsigned int getPortCount(void) = 0;
	virtual std::string getPortName(unsigned int portNumber) = 0;
	virtual void ignoreTypes(bool midiSysex, bool midiTime, bool midiSense);
	double getMessage(std::vector<unsigned char> *message);

	// A MIDI structure used internally by the class to store incoming
	// messages.  Each message represents one and only one MIDI message.
	struct MidiMessage
	{
		std::vector<unsigned char> bytes;
		double timeStamp;

		// Default constructor.
		MidiMessage()
			: bytes(0), timeStamp(0.0) {}
	};

	struct MidiQueue
	{
		unsigned int front;
		unsigned int back;
		unsigned int size;
		unsigned int ringSize;
		MidiMessage *ring;

		// Default constructor.
		MidiQueue()
			: front(0), back(0), size(0), ringSize(0) {}
	};

	// The RtMidiInData structure is used to pass private class data to
	// the MIDI input handling function or thread.
	struct RtMidiInData
	{
		MidiQueue queue;
		MidiMessage message;
		unsigned char ignoreFlags;
		bool doInput;
		bool firstMessage;
		void *apiData;
		bool usingCallback;
		void *userCallback;
		void *userData;
		bool continueSysex;

		// Default constructor.
		RtMidiInData()
			: ignoreFlags(7), doInput(false), firstMessage(true), apiData(0), usingCallback(false), userCallback(0), userData(0), continueSysex(false) {}
	};

protected:
	virtual void initialize(const std::string &clientName) = 0;
	RtMidiInData inputData_;

	void *apiData_;
	bool connected_;
	std::string errorString_;
};

class MidiOutApi
{
public:
	MidiOutApi(void);
	virtual ~MidiOutApi(void);
	virtual RtMidi::Api getCurrentApi(void) = 0;
	virtual void openPort(unsigned int portNumber, const std::string portName) = 0;
	virtual void openVirtualPort(const std::string portName) = 0;
	virtual void closePort(void) = 0;
	virtual unsigned int getPortCount(void) = 0;
	virtual std::string getPortName(unsigned int portNumber) = 0;
	virtual void sendMessage(std::vector<unsigned char> *message) = 0;

protected:
	virtual void initialize(const std::string &clientName) = 0;

	void *apiData_;
	bool connected_;
	std::string errorString_;
};

// **************************************************************** //
//
// Inline RtMidiIn and RtMidiOut definitions.
//
// **************************************************************** //

inline RtMidi::Api RtMidiIn ::getCurrentApi(void) { return rtapi_->getCurrentApi(); }
inline void RtMidiIn ::openPort(unsigned int portNumber, const std::string portName) { return rtapi_->openPort(portNumber, portName); }
inline void RtMidiIn ::openVirtualPort(const std::string portName) { return rtapi_->openVirtualPort(portName); }
inline void RtMidiIn ::closePort(void) { return rtapi_->closePort(); }
inline void RtMidiIn ::setCallback(RtMidiCallback callback, void *userData) { return rtapi_->setCallback(callback, userData); }
inline void RtMidiIn ::cancelCallback(void) { return rtapi_->cancelCallback(); }
inline unsigned int RtMidiIn ::getPortCount(void) { return rtapi_->getPortCount(); }
inline std::string RtMidiIn ::getPortName(unsigned int portNumber) { return rtapi_->getPortName(portNumber); }
inline void RtMidiIn ::ignoreTypes(bool midiSysex, bool midiTime, bool midiSense) { return rtapi_->ignoreTypes(midiSysex, midiTime, midiSense); }
inline double RtMidiIn ::getMessage(std::vector<unsigned char> *message) { return rtapi_->getMessage(message); }

inline RtMidi::Api RtMidiOut ::getCurrentApi(void) { return rtapi_->getCurrentApi(); }
inline void RtMidiOut ::openPort(unsigned int portNumber, const std::string portName) { return rtapi_->openPort(portNumber, portName); }
inline void RtMidiOut ::openVirtualPort(const std::string portName) { return rtapi_->openVirtualPort(portName); }
inline void RtMidiOut ::closePort(void) { return rtapi_->closePort(); }
inline unsigned int RtMidiOut ::getPortCount(void) { return rtapi_->getPortCount(); }
inline std::string RtMidiOut ::getPortName(unsigned int portNumber) { return rtapi_->getPortName(portNumber); }
inline void RtMidiOut ::sendMessage(std::vector<unsigned char> *message) { return rtapi_->sendMessage(message); }

// **************************************************************** //
//
// MidiInApi and MidiOutApi subclass prototypes.
//
// **************************************************************** //

#if !defined(__LINUX_ALSA__) && !defined(__UNIX_JACK__) && !defined(__MACOSX_CORE__) && !defined(__WINDOWS_MM__) && !defined(__WINDOWS_KS__)
#define __RTMIDI_DUMMY__
#endif

#if defined(__MACOSX_CORE__)

class MidiInCore : public MidiInApi
{
public:
	MidiInCore(const std::string clientName, unsigned int queueSizeLimit);
	~MidiInCore(void);
	RtMidi::Api getCurrentApi(void) { return RtMidi::MACOSX_CORE; };
	void openPort(unsigned int portNumber, const std::string portName);
	void openVirtualPort(const std::string portName);
	void closePort(void);
	unsigned int getPortCount(void);
	std::string getPortName(unsigned int portNumber);

protected:
	void initialize(const std::string &clientName);
};

class MidiOutCore : public MidiOutApi
{
public:
	MidiOutCore(const std::string clientName);
	~MidiOutCore(void);
	RtMidi::Api getCurrentApi(void) { return RtMidi::MACOSX_CORE; };
	void openPort(unsigned int portNumber, const std::string portName);
	void openVirtualPort(const std::string portName);
	void closePort(void);
	unsigned int getPortCount(void);
	std::string getPortName(unsigned int portNumber);
	void sendMessage(std::vector<unsigned char> *message);

protected:
	void initialize(const std::string &clientName);
};

#endif

#if defined(__UNIX_JACK__)

class MidiInJack : public MidiInApi
{
public:
	MidiInJack(const std::string clientName, unsigned int queueSizeLimit);
	~MidiInJack(void);
	RtMidi::Api getCurrentApi(void) { return RtMidi::UNIX_JACK; };
	void openPort(unsigned int portNumber, const std::string portName);
	void openVirtualPort(const std::string portName);
	void closePort(void);
	unsigned int getPortCount(void);
	std::string getPortName(unsigned int portNumber);

protected:
	void initialize(const std::string &clientName);
};

class MidiOutJack : public MidiOutApi
{
public:
	MidiOutJack(const std::string clientName);
	~MidiOutJack(void);
	RtMidi::Api getCurrentApi(void) { return RtMidi::UNIX_JACK; };
	void openPort(unsigned int portNumber, const std::string portName);
	void openVirtualPort(const std::string portName);
	void closePort(void);
	unsigned int getPortCount(void);
	std::string getPortName(unsigned int portNumber);
	void sendMessage(std::vector<unsigned char> *message);

protected:
	void initialize(const std::string &clientName);
};

#endif

#if defined(__LINUX_ALSA__)

class MidiInAlsa : public MidiInApi
{
public:
	MidiInAlsa(const std::string clientName, unsigned int queueSizeLimit);
	~MidiInAlsa(void);
	RtMidi::Api getCurrentApi(void) { return RtMidi::LINUX_ALSA; };
	void openPort(unsigned int portNumber, const std::string portName);
	void openVirtualPort(const std::string portName);
	void closePort(void);
	unsigned int getPortCount(void);
	std::string getPortName(unsigned int portNumber);

protected:
	void initialize(const std::string &clientName);
};

class MidiOutAlsa : public MidiOutApi
{
public:
	MidiOutAlsa(const std::string clientName);
	~MidiOutAlsa(void);
	RtMidi::Api getCurrentApi(void) { return RtMidi::LINUX_ALSA; };
	void openPort(unsigned int portNumber, const std::string portName);
	void openVirtualPort(const std::string portName);
	void closePort(void);
	unsigned int getPortCount(void);
	std::string getPortName(unsigned int portNumber);
	void sendMessage(std::vector<unsigned char> *message);

protected:
	void initialize(const std::string &clientName);
};

#endif

#if defined(__WINDOWS_MM__)

class MidiInWinMM : public MidiInApi
{
public:
	MidiInWinMM(const std::string clientName, unsigned int queueSizeLimit);
	~MidiInWinMM(void);
	RtMidi::Api getCurrentApi(void) { return RtMidi::WINDOWS_MM; };
	void openPort(unsigned int portNumber, const std::string portName);
	void openVirtualPort(const std::string portName);
	void closePort(void);
	unsigned int getPortCount(void);
	std::string getPortName(unsigned int portNumber);

protected:
	void initialize(const std::string &clientName);
};

class MidiOutWinMM : public MidiOutApi
{
public:
	MidiOutWinMM(const std::string clientName);
	~MidiOutWinMM(void);
	RtMidi::Api getCurrentApi(void) { return RtMidi::WINDOWS_MM; };
	void openPort(unsigned int portNumber, const std::string portName);
	void openVirtualPort(const std::string portName);
	void closePort(void);
	unsigned int getPortCount(void);
	std::string getPortName(unsigned int portNumber);
	void sendMessage(std::vector<unsigned char> *message);

protected:
	void initialize(const std::string &clientName);
};

#endif

#if defined(__WINDOWS_KS__)

class MidiInWinKS : public MidiInApi
{
public:
	MidiInWinKS(const std::string clientName, unsigned int queueSizeLimit);
	~MidiInWinKS(void);
	RtMidi::Api getCurrentApi(void) { return RtMidi::WINDOWS_KS; };
	void openPort(unsigned int portNumber, const std::string portName);
	void openVirtualPort(const std::string portName);
	void closePort(void);
	unsigned int getPortCount(void);
	std::string getPortName(unsigned int portNumber);

protected:
	void initialize(const std::string &clientName);
};

class MidiOutWinKS : public MidiOutApi
{
public:
	MidiOutWinKS(const std::string clientName);
	~MidiOutWinKS(void);
	RtMidi::Api getCurrentApi(void) { return RtMidi::WINDOWS_KS; };
	void openPort(unsigned int portNumber, const std::string portName);
	void openVirtualPort(const std::string portName);
	void closePort(void);
	unsigned int getPortCount(void);
	std::string getPortName(unsigned int portNumber);
	void sendMessage(std::vector<unsigned char> *message);

protected:
	void initialize(const std::string &clientName);
};

#endif

#if defined(__RTMIDI_DUMMY__)

class MidiInDummy : public MidiInApi
{
public:
	MidiInDummy(const std::string clientName, unsigned int queueSizeLimit) : MidiInApi(queueSizeLimit)
	{
		errorString_ = "MidiInDummy: This class provides no functionality.";
		RtMidi::error(RtError::WARNING, errorString_);
	};
	RtMidi::Api getCurrentApi(void) { return RtMidi::RTMIDI_DUMMY; };
	void openPort(unsigned int portNumber, const std::string portName){};
	void openVirtualPort(const std::string portName){};
	void closePort(void){};
	unsigned int getPortCount(void) { return 0; };
	std::string getPortName(unsigned int portNumber) { return ""; };

protected:
	void initialize(const std::string &clientName){};
};

class MidiOutDummy : public MidiOutApi
{
public:
	MidiOutDummy(const std::string clientName)
	{
		errorString_ = "MidiOutDummy: This class provides no functionality.";
		RtMidi::error(RtError::WARNING, errorString_);
	};
	RtMidi::Api getCurrentApi(void) { return RtMidi::RTMIDI_DUMMY; };
	void openPort(unsigned int portNumber, const std::string portName){};
	void openVirtualPort(const std::string portName){};
	void closePort(void){};
	unsigned int getPortCount(void) { return 0; };
	std::string getPortName(unsigned int portNumber) { return ""; };
	void sendMessage(std::vector<unsigned char> *message){};

protected:
	void initialize(const std::string &clientName){};
};

#endif

#endif