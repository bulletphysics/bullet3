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

// RtMidi: Version 2.0.1

#include "RtMidi.h"
#include <sstream>

//*********************************************************************//
//  RtMidi Definitions
//*********************************************************************//

void RtMidi :: getCompiledApi( std::vector<RtMidi::Api> &apis ) throw()
{
  apis.clear();

  // The order here will control the order of RtMidi's API search in
  // the constructor.
#if defined(__MACOSX_CORE__)
  apis.push_back( MACOSX_CORE );
#endif
#if defined(__LINUX_ALSA__)
  apis.push_back( LINUX_ALSA );
#endif
#if defined(__UNIX_JACK__)
  apis.push_back( UNIX_JACK );
#endif
#if defined(__WINDOWS_MM__)
  apis.push_back( WINDOWS_MM );
#endif
#if defined(__WINDOWS_KS__)
  apis.push_back( WINDOWS_KS );
#endif
#if defined(__RTMIDI_DUMMY__)
  apis.push_back( RTMIDI_DUMMY );
#endif
}

void RtMidi :: error( RtError::Type type, std::string errorString )
{
  if (type == RtError::WARNING) {
    std::cerr << '\n' << errorString << "\n\n";
  }
  else if (type == RtError::DEBUG_WARNING) {
#if defined(__RTMIDI_DEBUG__)
    std::cerr << '\n' << errorString << "\n\n";
#endif
  }
  else {
    std::cerr << '\n' << errorString << "\n\n";
    throw RtError( errorString, type );
  }
}

//*********************************************************************//
//  RtMidiIn Definitions
//*********************************************************************//

void RtMidiIn :: openMidiApi( RtMidi::Api api, const std::string clientName, unsigned int queueSizeLimit )
{
  if ( rtapi_ )
    delete rtapi_;
  rtapi_ = 0;

#if defined(__UNIX_JACK__)
  if ( api == UNIX_JACK )
    rtapi_ = new MidiInJack( clientName, queueSizeLimit );
#endif
#if defined(__LINUX_ALSA__)
  if ( api == LINUX_ALSA )
    rtapi_ = new MidiInAlsa( clientName, queueSizeLimit );
#endif
#if defined(__WINDOWS_MM__)
  if ( api == WINDOWS_MM )
    rtapi_ = new MidiInWinMM( clientName, queueSizeLimit );
#endif
#if defined(__WINDOWS_KS__)
  if ( api == WINDOWS_KS )
    rtapi_ = new MidiInWinKS( clientName, queueSizeLimit );
#endif
#if defined(__MACOSX_CORE__)
  if ( api == MACOSX_CORE )
    rtapi_ = new MidiInCore( clientName, queueSizeLimit );
#endif
#if defined(__RTMIDI_DUMMY__)
  if ( api == RTMIDI_DUMMY )
    rtapi_ = new MidiInDummy( clientName, queueSizeLimit );
#endif
}

RtMidiIn :: RtMidiIn( RtMidi::Api api, const std::string clientName, unsigned int queueSizeLimit )
{
  rtapi_ = 0;

  if ( api != UNSPECIFIED ) {
    // Attempt to open the specified API.
    openMidiApi( api, clientName, queueSizeLimit );
    if ( rtapi_ ) return;

    // No compiled support for specified API value.  Issue a debug
    // warning and continue as if no API was specified.
    RtMidi::error( RtError::WARNING, "RtMidiIn: no compiled support for specified API argument!" );
  }

  // Iterate through the compiled APIs and return as soon as we find
  // one with at least one port or we reach the end of the list.
  std::vector< RtMidi::Api > apis;
  getCompiledApi( apis );
  for ( unsigned int i=0; i<apis.size(); i++ ) {
    openMidiApi( apis[i], clientName, queueSizeLimit );
    if ( rtapi_->getPortCount() ) break;
  }

  if ( rtapi_ ) return;

  // It should not be possible to get here because the preprocessor
  // definition __RTMIDI_DUMMY__ is automatically defined if no
  // API-specific definitions are passed to the compiler. But just in
  // case something weird happens, we'll print out an error message.
  RtMidi::error( RtError::WARNING, "RtMidiIn: no compiled API support found ... critical error!!" );
}

RtMidiIn :: ~RtMidiIn() throw()
{
  delete rtapi_;
}


//*********************************************************************//
//  RtMidiOut Definitions
//*********************************************************************//

void RtMidiOut :: openMidiApi( RtMidi::Api api, const std::string clientName )
{
  if ( rtapi_ )
    delete rtapi_;
  rtapi_ = 0;

#if defined(__UNIX_JACK__)
  if ( api == UNIX_JACK )
    rtapi_ = new MidiOutJack( clientName );
#endif
#if defined(__LINUX_ALSA__)
  if ( api == LINUX_ALSA )
    rtapi_ = new MidiOutAlsa( clientName );
#endif
#if defined(__WINDOWS_MM__)
  if ( api == WINDOWS_MM )
    rtapi_ = new MidiOutWinMM( clientName );
#endif
#if defined(__WINDOWS_KS__)
  if ( api == WINDOWS_KS )
    rtapi_ = new MidiOutWinKS( clientName );
#endif
#if defined(__MACOSX_CORE__)
  if ( api == MACOSX_CORE )
    rtapi_ = new MidiOutCore( clientName );
#endif
#if defined(__RTMIDI_DUMMY__)
  if ( api == RTMIDI_DUMMY )
    rtapi_ = new MidiOutDummy( clientName );
#endif
}

RtMidiOut :: RtMidiOut( RtMidi::Api api, const std::string clientName )
{
  rtapi_ = 0;

  if ( api != UNSPECIFIED ) {
    // Attempt to open the specified API.
    openMidiApi( api, clientName );
    if ( rtapi_ ) return;

    // No compiled support for specified API value.  Issue a debug
    // warning and continue as if no API was specified.
    RtMidi::error( RtError::WARNING, "RtMidiOut: no compiled support for specified API argument!" );
  }

  // Iterate through the compiled APIs and return as soon as we find
  // one with at least one port or we reach the end of the list.
  std::vector< RtMidi::Api > apis;
  getCompiledApi( apis );
  for ( unsigned int i=0; i<apis.size(); i++ ) {
    openMidiApi( apis[i], clientName );
    if ( rtapi_->getPortCount() ) break;
  }

  if ( rtapi_ ) return;

  // It should not be possible to get here because the preprocessor
  // definition __RTMIDI_DUMMY__ is automatically defined if no
  // API-specific definitions are passed to the compiler. But just in
  // case something weird happens, we'll print out an error message.
  RtMidi::error( RtError::WARNING, "RtMidiOut: no compiled API support found ... critical error!!" );
}

RtMidiOut :: ~RtMidiOut() throw()
{
  delete rtapi_;
}

//*********************************************************************//
//  Common MidiInApi Definitions
//*********************************************************************//

MidiInApi :: MidiInApi( unsigned int queueSizeLimit )
  : apiData_( 0 ), connected_( false )
{
  // Allocate the MIDI queue.
  inputData_.queue.ringSize = queueSizeLimit;
  if ( inputData_.queue.ringSize > 0 )
    inputData_.queue.ring = new MidiMessage[ inputData_.queue.ringSize ];
}

MidiInApi :: ~MidiInApi( void )
{
  // Delete the MIDI queue.
  if ( inputData_.queue.ringSize > 0 ) delete [] inputData_.queue.ring;
}

void MidiInApi :: setCallback( RtMidiIn::RtMidiCallback callback, void *userData )
{
  if ( inputData_.usingCallback ) {
    errorString_ = "MidiInApi::setCallback: a callback function is already set!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return;
  }

  if ( !callback ) {
    errorString_ = "RtMidiIn::setCallback: callback function value is invalid!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return;
  }

  inputData_.userCallback = (void *) callback;
  inputData_.userData = userData;
  inputData_.usingCallback = true;
}

void MidiInApi :: cancelCallback()
{
  if ( !inputData_.usingCallback ) {
    errorString_ = "RtMidiIn::cancelCallback: no callback function was set!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return;
  }

  inputData_.userCallback = 0;
  inputData_.userData = 0;
  inputData_.usingCallback = false;
}

void MidiInApi :: ignoreTypes( bool midiSysex, bool midiTime, bool midiSense )
{
  inputData_.ignoreFlags = 0;
  if ( midiSysex ) inputData_.ignoreFlags = 0x01;
  if ( midiTime ) inputData_.ignoreFlags |= 0x02;
  if ( midiSense ) inputData_.ignoreFlags |= 0x04;
}

double MidiInApi :: getMessage( std::vector<unsigned char> *message )
{
  message->clear();

  if ( inputData_.usingCallback ) {
    errorString_ = "RtMidiIn::getNextMessage: a user callback is currently set for this port.";
    RtMidi::error( RtError::WARNING, errorString_ );
    return 0.0;
  }

  if ( inputData_.queue.size == 0 ) return 0.0;

  // Copy queued message to the vector pointer argument and then "pop" it.
  std::vector<unsigned char> *bytes = &(inputData_.queue.ring[inputData_.queue.front].bytes);
  message->assign( bytes->begin(), bytes->end() );
  double deltaTime = inputData_.queue.ring[inputData_.queue.front].timeStamp;
  inputData_.queue.size--;
  inputData_.queue.front++;
  if ( inputData_.queue.front == inputData_.queue.ringSize )
    inputData_.queue.front = 0;

  return deltaTime;
}

//*********************************************************************//
//  Common MidiOutApi Definitions
//*********************************************************************//

MidiOutApi :: MidiOutApi( void )
  : apiData_( 0 ), connected_( false )
{
}

MidiOutApi :: ~MidiOutApi( void )
{
}

// *************************************************** //
//
// OS/API-specific methods.
//
// *************************************************** //

#if defined(__MACOSX_CORE__)

// The CoreMIDI API is based on the use of a callback function for
// MIDI input.  We convert the system specific time stamps to delta
// time values.

// OS-X CoreMIDI header files.
#include <CoreMIDI/CoreMIDI.h>
#include <CoreAudio/HostTime.h>
#include <CoreServices/CoreServices.h>

// A structure to hold variables related to the CoreMIDI API
// implementation.
struct CoreMidiData {
  MIDIClientRef client;
  MIDIPortRef port;
  MIDIEndpointRef endpoint;
  MIDIEndpointRef destinationId;
  unsigned long long lastTime;
  MIDISysexSendRequest sysexreq;
};

//*********************************************************************//
//  API: OS-X
//  Class Definitions: MidiInCore
//*********************************************************************//

void midiInputCallback( const MIDIPacketList *list, void *procRef, void *srcRef )
{
  MidiInApi::RtMidiInData *data = static_cast<MidiInApi::RtMidiInData *> (procRef);
  CoreMidiData *apiData = static_cast<CoreMidiData *> (data->apiData);

  unsigned char status;
  unsigned short nBytes, iByte, size;
  unsigned long long time;

  bool& continueSysex = data->continueSysex;
  MidiInApi::MidiMessage& message = data->message;

  const MIDIPacket *packet = &list->packet[0];
  for ( unsigned int i=0; i<list->numPackets; ++i ) {

    // My interpretation of the CoreMIDI documentation: all message
    // types, except sysex, are complete within a packet and there may
    // be several of them in a single packet.  Sysex messages can be
    // broken across multiple packets and PacketLists but are bundled
    // alone within each packet (these packets do not contain other
    // message types).  If sysex messages are split across multiple
    // MIDIPacketLists, they must be handled by multiple calls to this
    // function.

    nBytes = packet->length;
    if ( nBytes == 0 ) continue;

    // Calculate time stamp.

    if ( data->firstMessage ) {
      message.timeStamp = 0.0;
      data->firstMessage = false;
    }
    else {
      time = packet->timeStamp;
      if ( time == 0 ) { // this happens when receiving asynchronous sysex messages
        time = AudioGetCurrentHostTime();
      }
      time -= apiData->lastTime;
      time = AudioConvertHostTimeToNanos( time );
      if ( !continueSysex )
        message.timeStamp = time * 0.000000001;
    }
    apiData->lastTime = packet->timeStamp;
    if ( apiData->lastTime == 0 ) { // this happens when receiving asynchronous sysex messages
      apiData->lastTime = AudioGetCurrentHostTime();
    }
    //std::cout << "TimeStamp = " << packet->timeStamp << std::endl;

    iByte = 0;
    if ( continueSysex ) {
      // We have a continuing, segmented sysex message.
      if ( !( data->ignoreFlags & 0x01 ) ) {
        // If we're not ignoring sysex messages, copy the entire packet.
        for ( unsigned int j=0; j<nBytes; ++j )
          message.bytes.push_back( packet->data[j] );
      }
      continueSysex = packet->data[nBytes-1] != 0xF7;

      if ( !continueSysex ) {
        // If not a continuing sysex message, invoke the user callback function or queue the message.
        if ( data->usingCallback ) {
          RtMidiIn::RtMidiCallback callback = (RtMidiIn::RtMidiCallback) data->userCallback;
          callback( message.timeStamp, &message.bytes, data->userData );
        }
        else {
          // As long as we haven't reached our queue size limit, push the message.
          if ( data->queue.size < data->queue.ringSize ) {
            data->queue.ring[data->queue.back++] = message;
            if ( data->queue.back == data->queue.ringSize )
              data->queue.back = 0;
            data->queue.size++;
          }
          else
            std::cerr << "\nMidiInCore: message queue limit reached!!\n\n";
        }
        message.bytes.clear();
      }
    }
    else {
      while ( iByte < nBytes ) {
        size = 0;
        // We are expecting that the next byte in the packet is a status byte.
        status = packet->data[iByte];
        if ( !(status & 0x80) ) break;
        // Determine the number of bytes in the MIDI message.
        if ( status < 0xC0 ) size = 3;
        else if ( status < 0xE0 ) size = 2;
        else if ( status < 0xF0 ) size = 3;
        else if ( status == 0xF0 ) {
          // A MIDI sysex
          if ( data->ignoreFlags & 0x01 ) {
            size = 0;
            iByte = nBytes;
          }
          else size = nBytes - iByte;
          continueSysex = packet->data[nBytes-1] != 0xF7;
        }
        else if ( status == 0xF1 ) {
            // A MIDI time code message
           if ( data->ignoreFlags & 0x02 ) {
            size = 0;
            iByte += 2;
           }
           else size = 2;
        }
        else if ( status == 0xF2 ) size = 3;
        else if ( status == 0xF3 ) size = 2;
        else if ( status == 0xF8 && ( data->ignoreFlags & 0x02 ) ) {
          // A MIDI timing tick message and we're ignoring it.
          size = 0;
          iByte += 1;
        }
        else if ( status == 0xFE && ( data->ignoreFlags & 0x04 ) ) {
          // A MIDI active sensing message and we're ignoring it.
          size = 0;
          iByte += 1;
        }
        else size = 1;

        // Copy the MIDI data to our vector.
        if ( size ) {
          message.bytes.assign( &packet->data[iByte], &packet->data[iByte+size] );
          if ( !continueSysex ) {
            // If not a continuing sysex message, invoke the user callback function or queue the message.
            if ( data->usingCallback ) {
              RtMidiIn::RtMidiCallback callback = (RtMidiIn::RtMidiCallback) data->userCallback;
              callback( message.timeStamp, &message.bytes, data->userData );
            }
            else {
              // As long as we haven't reached our queue size limit, push the message.
              if ( data->queue.size < data->queue.ringSize ) {
                data->queue.ring[data->queue.back++] = message;
                if ( data->queue.back == data->queue.ringSize )
                  data->queue.back = 0;
                data->queue.size++;
              }
              else
                std::cerr << "\nMidiInCore: message queue limit reached!!\n\n";
            }
            message.bytes.clear();
          }
          iByte += size;
        }
      }
    }
    packet = MIDIPacketNext(packet);
  }
}

MidiInCore :: MidiInCore( const std::string clientName, unsigned int queueSizeLimit ) : MidiInApi( queueSizeLimit )
{
  initialize( clientName );
}

MidiInCore :: ~MidiInCore( void )
{
  // Close a connection if it exists.
  closePort();

  // Cleanup.
  CoreMidiData *data = static_cast<CoreMidiData *> (apiData_);
  MIDIClientDispose( data->client );
  if ( data->endpoint ) MIDIEndpointDispose( data->endpoint );
  delete data;
}

void MidiInCore :: initialize( const std::string& clientName )
{
  // Set up our client.
  MIDIClientRef client;
  OSStatus result = MIDIClientCreate( CFStringCreateWithCString( NULL, clientName.c_str(), kCFStringEncodingASCII ), NULL, NULL, &client );
  if ( result != noErr ) {
    errorString_ = "MidiInCore::initialize: error creating OS-X MIDI client object.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Save our api-specific connection information.
  CoreMidiData *data = (CoreMidiData *) new CoreMidiData;
  data->client = client;
  data->endpoint = 0;
  apiData_ = (void *) data;
  inputData_.apiData = (void *) data;
}

void MidiInCore :: openPort( unsigned int portNumber, const std::string portName )
{
  if ( connected_ ) {
    errorString_ = "MidiInCore::openPort: a valid connection already exists!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return;
  }

  unsigned int nSrc = MIDIGetNumberOfSources();
  if (nSrc < 1) {
    errorString_ = "MidiInCore::openPort: no MIDI input sources found!";
    RtMidi::error( RtError::NO_DEVICES_FOUND, errorString_ );
  }

  std::ostringstream ost;
  if ( portNumber >= nSrc ) {
    ost << "MidiInCore::openPort: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::INVALID_PARAMETER, errorString_ );
  }

  MIDIPortRef port;
  CoreMidiData *data = static_cast<CoreMidiData *> (apiData_);
  OSStatus result = MIDIInputPortCreate( data->client, 
                                         CFStringCreateWithCString( NULL, portName.c_str(), kCFStringEncodingASCII ),
                                         midiInputCallback, (void *)&inputData_, &port );
  if ( result != noErr ) {
    MIDIClientDispose( data->client );
    errorString_ = "MidiInCore::openPort: error creating OS-X MIDI input port.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Get the desired input source identifier.
  MIDIEndpointRef endpoint = MIDIGetSource( portNumber );
  if ( endpoint == 0 ) {
    MIDIPortDispose( port );
    MIDIClientDispose( data->client );
    errorString_ = "MidiInCore::openPort: error getting MIDI input source reference.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Make the connection.
  result = MIDIPortConnectSource( port, endpoint, NULL );
  if ( result != noErr ) {
    MIDIPortDispose( port );
    MIDIClientDispose( data->client );
    errorString_ = "MidiInCore::openPort: error connecting OS-X MIDI input port.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Save our api-specific port information.
  data->port = port;

  connected_ = true;
}

void MidiInCore :: openVirtualPort( const std::string portName )
{
  CoreMidiData *data = static_cast<CoreMidiData *> (apiData_);

  // Create a virtual MIDI input destination.
  MIDIEndpointRef endpoint;
  OSStatus result = MIDIDestinationCreate( data->client,
                                           CFStringCreateWithCString( NULL, portName.c_str(), kCFStringEncodingASCII ),
                                           midiInputCallback, (void *)&inputData_, &endpoint );
  if ( result != noErr ) {
    errorString_ = "MidiInCore::openVirtualPort: error creating virtual OS-X MIDI destination.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Save our api-specific connection information.
  data->endpoint = endpoint;
}

void MidiInCore :: closePort( void )
{
  if ( connected_ ) {
    CoreMidiData *data = static_cast<CoreMidiData *> (apiData_);
    MIDIPortDispose( data->port );
    connected_ = false;
  }
}

unsigned int MidiInCore :: getPortCount()
{
  return MIDIGetNumberOfSources();
}

// This function was submitted by Douglas Casey Tucker and apparently
// derived largely from PortMidi.
CFStringRef EndpointName( MIDIEndpointRef endpoint, bool isExternal )
{
  CFMutableStringRef result = CFStringCreateMutable( NULL, 0 );
  CFStringRef str;

  // Begin with the endpoint's name.
  str = NULL;
  MIDIObjectGetStringProperty( endpoint, kMIDIPropertyName, &str );
  if ( str != NULL ) {
    CFStringAppend( result, str );
    CFRelease( str );
  }

  MIDIEntityRef entity = NULL;
  MIDIEndpointGetEntity( endpoint, &entity );
  if ( entity == 0 )
    // probably virtual
    return result;

  if ( CFStringGetLength( result ) == 0 ) {
    // endpoint name has zero length -- try the entity
    str = NULL;
    MIDIObjectGetStringProperty( entity, kMIDIPropertyName, &str );
    if ( str != NULL ) {
      CFStringAppend( result, str );
      CFRelease( str );
    }
  }
  // now consider the device's name
  MIDIDeviceRef device = 0;
  MIDIEntityGetDevice( entity, &device );
  if ( device == 0 )
    return result;

  str = NULL;
  MIDIObjectGetStringProperty( device, kMIDIPropertyName, &str );
  if ( CFStringGetLength( result ) == 0 ) {
      CFRelease( result );
      return str;
  }
  if ( str != NULL ) {
    // if an external device has only one entity, throw away
    // the endpoint name and just use the device name
    if ( isExternal && MIDIDeviceGetNumberOfEntities( device ) < 2 ) {
      CFRelease( result );
      return str;
    } else {
      if ( CFStringGetLength( str ) == 0 ) {
        CFRelease( str );
        return result;
      }
      // does the entity name already start with the device name?
      // (some drivers do this though they shouldn't)
      // if so, do not prepend
        if ( CFStringCompareWithOptions( result, /* endpoint name */
             str /* device name */,
             CFRangeMake(0, CFStringGetLength( str ) ), 0 ) != kCFCompareEqualTo ) {
        // prepend the device name to the entity name
        if ( CFStringGetLength( result ) > 0 )
          CFStringInsert( result, 0, CFSTR(" ") );
        CFStringInsert( result, 0, str );
      }
      CFRelease( str );
    }
  }
  return result;
}

// This function was submitted by Douglas Casey Tucker and apparently
// derived largely from PortMidi.
static CFStringRef ConnectedEndpointName( MIDIEndpointRef endpoint )
{
  CFMutableStringRef result = CFStringCreateMutable( NULL, 0 );
  CFStringRef str;
  OSStatus err;
  int i;

  // Does the endpoint have connections?
  CFDataRef connections = NULL;
  int nConnected = 0;
  bool anyStrings = false;
  err = MIDIObjectGetDataProperty( endpoint, kMIDIPropertyConnectionUniqueID, &connections );
  if ( connections != NULL ) {
    // It has connections, follow them
    // Concatenate the names of all connected devices
    nConnected = CFDataGetLength( connections ) / sizeof(MIDIUniqueID);
    if ( nConnected ) {
      const SInt32 *pid = (const SInt32 *)(CFDataGetBytePtr(connections));
      for ( i=0; i<nConnected; ++i, ++pid ) {
        MIDIUniqueID id = EndianS32_BtoN( *pid );
        MIDIObjectRef connObject;
        MIDIObjectType connObjectType;
        err = MIDIObjectFindByUniqueID( id, &connObject, &connObjectType );
        if ( err == noErr ) {
          if ( connObjectType == kMIDIObjectType_ExternalSource  ||
              connObjectType == kMIDIObjectType_ExternalDestination ) {
            // Connected to an external device's endpoint (10.3 and later).
            str = EndpointName( (MIDIEndpointRef)(connObject), true );
          } else {
            // Connected to an external device (10.2) (or something else, catch-
            str = NULL;
            MIDIObjectGetStringProperty( connObject, kMIDIPropertyName, &str );
          }
          if ( str != NULL ) {
            if ( anyStrings )
              CFStringAppend( result, CFSTR(", ") );
            else anyStrings = true;
            CFStringAppend( result, str );
            CFRelease( str );
          }
        }
      }
    }
    CFRelease( connections );
  }
  if ( anyStrings )
    return result;

  // Here, either the endpoint had no connections, or we failed to obtain names 
  return EndpointName( endpoint, false );
}

std::string MidiInCore :: getPortName( unsigned int portNumber )
{
  CFStringRef nameRef;
  MIDIEndpointRef portRef;
  std::ostringstream ost;
  char name[128];

  std::string stringName;
  if ( portNumber >= MIDIGetNumberOfSources() ) {
    ost << "MidiInCore::getPortName: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::WARNING, errorString_ );
    //RtMidi::error( RtError::INVALID_PARAMETER, errorString_ );
    return stringName;
  }

  portRef = MIDIGetSource( portNumber );
  nameRef = ConnectedEndpointName(portRef);
  CFStringGetCString( nameRef, name, sizeof(name), 0);
  CFRelease( nameRef );

  return stringName = name;
}

//*********************************************************************//
//  API: OS-X
//  Class Definitions: MidiOutCore
//*********************************************************************//

MidiOutCore :: MidiOutCore( const std::string clientName ) : MidiOutApi()
{
  initialize( clientName );
}

MidiOutCore :: ~MidiOutCore( void )
{
  // Close a connection if it exists.
  closePort();

  // Cleanup.
  CoreMidiData *data = static_cast<CoreMidiData *> (apiData_);
  MIDIClientDispose( data->client );
  if ( data->endpoint ) MIDIEndpointDispose( data->endpoint );
  delete data;
}

void MidiOutCore :: initialize( const std::string& clientName )
{
  // Set up our client.
  MIDIClientRef client;
  OSStatus result = MIDIClientCreate( CFStringCreateWithCString( NULL, clientName.c_str(), kCFStringEncodingASCII ), NULL, NULL, &client );
  if ( result != noErr ) {
    errorString_ = "MidiOutCore::initialize: error creating OS-X MIDI client object.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Save our api-specific connection information.
  CoreMidiData *data = (CoreMidiData *) new CoreMidiData;
  data->client = client;
  data->endpoint = 0;
  apiData_ = (void *) data;
}

unsigned int MidiOutCore :: getPortCount()
{
  return MIDIGetNumberOfDestinations();
}

std::string MidiOutCore :: getPortName( unsigned int portNumber )
{
  CFStringRef nameRef;
  MIDIEndpointRef portRef;
  std::ostringstream ost;
  char name[128];

  std::string stringName;
  if ( portNumber >= MIDIGetNumberOfDestinations() ) {
    ost << "MidiOutCore::getPortName: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::WARNING, errorString_ );
    return stringName;
    //RtMidi::error( RtError::INVALID_PARAMETER, errorString_ );
  }

  portRef = MIDIGetDestination( portNumber );
  nameRef = ConnectedEndpointName(portRef);
  CFStringGetCString( nameRef, name, sizeof(name), 0);
  CFRelease( nameRef );
  
  return stringName = name;
}

void MidiOutCore :: openPort( unsigned int portNumber, const std::string portName )
{
  if ( connected_ ) {
    errorString_ = "MidiOutCore::openPort: a valid connection already exists!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return;
  }

  unsigned int nDest = MIDIGetNumberOfDestinations();
  if (nDest < 1) {
    errorString_ = "MidiOutCore::openPort: no MIDI output destinations found!";
    RtMidi::error( RtError::NO_DEVICES_FOUND, errorString_ );
  }

  std::ostringstream ost;
  if ( portNumber >= nDest ) {
    ost << "MidiOutCore::openPort: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::INVALID_PARAMETER, errorString_ );
  }

  MIDIPortRef port;
  CoreMidiData *data = static_cast<CoreMidiData *> (apiData_);
  OSStatus result = MIDIOutputPortCreate( data->client, 
                                          CFStringCreateWithCString( NULL, portName.c_str(), kCFStringEncodingASCII ),
                                          &port );
  if ( result != noErr ) {
    MIDIClientDispose( data->client );
    errorString_ = "MidiOutCore::openPort: error creating OS-X MIDI output port.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Get the desired output port identifier.
  MIDIEndpointRef destination = MIDIGetDestination( portNumber );
  if ( destination == 0 ) {
    MIDIPortDispose( port );
    MIDIClientDispose( data->client );
    errorString_ = "MidiOutCore::openPort: error getting MIDI output destination reference.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Save our api-specific connection information.
  data->port = port;
  data->destinationId = destination;
  connected_ = true;
}

void MidiOutCore :: closePort( void )
{
  if ( connected_ ) {
    CoreMidiData *data = static_cast<CoreMidiData *> (apiData_);
    MIDIPortDispose( data->port );
    connected_ = false;
  }
}

void MidiOutCore :: openVirtualPort( std::string portName )
{
  CoreMidiData *data = static_cast<CoreMidiData *> (apiData_);

  if ( data->endpoint ) {
    errorString_ = "MidiOutCore::openVirtualPort: a virtual output port already exists!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return;
  }

  // Create a virtual MIDI output source.
  MIDIEndpointRef endpoint;
  OSStatus result = MIDISourceCreate( data->client,
                                      CFStringCreateWithCString( NULL, portName.c_str(), kCFStringEncodingASCII ),
                                      &endpoint );
  if ( result != noErr ) {
    errorString_ = "MidiOutCore::initialize: error creating OS-X virtual MIDI source.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Save our api-specific connection information.
  data->endpoint = endpoint;
}

char *sysexBuffer = 0;

void sysexCompletionProc( MIDISysexSendRequest * sreq )
{
  //std::cout << "Completed SysEx send\n";
 delete sysexBuffer;
 sysexBuffer = 0;
}

void MidiOutCore :: sendMessage( std::vector<unsigned char> *message )
{
  // We use the MIDISendSysex() function to asynchronously send sysex
  // messages.  Otherwise, we use a single CoreMidi MIDIPacket.
  unsigned int nBytes = message->size();
  if ( nBytes == 0 ) {
    errorString_ = "MidiOutCore::sendMessage: no data in message argument!";      
    RtMidi::error( RtError::WARNING, errorString_ );
    return;
  }

  //  unsigned int packetBytes, bytesLeft = nBytes;
  //  unsigned int messageIndex = 0;
  MIDITimeStamp timeStamp = AudioGetCurrentHostTime();
  CoreMidiData *data = static_cast<CoreMidiData *> (apiData_);
  OSStatus result;

  if ( message->at(0) == 0xF0 ) {

    while ( sysexBuffer != 0 ) usleep( 1000 ); // sleep 1 ms

   sysexBuffer = new char[nBytes];
   if ( sysexBuffer == NULL ) {
     errorString_ = "MidiOutCore::sendMessage: error allocating sysex message memory!";
     RtMidi::error( RtError::MEMORY_ERROR, errorString_ );
   }

   // Copy data to buffer.
   for ( unsigned int i=0; i<nBytes; ++i ) sysexBuffer[i] = message->at(i);

   data->sysexreq.destination = data->destinationId;
   data->sysexreq.data = (Byte *)sysexBuffer;
   data->sysexreq.bytesToSend = nBytes;
   data->sysexreq.complete = 0;
   data->sysexreq.completionProc = sysexCompletionProc;
   data->sysexreq.completionRefCon = &(data->sysexreq);

   result = MIDISendSysex( &(data->sysexreq) );
   if ( result != noErr ) {
     errorString_ = "MidiOutCore::sendMessage: error sending MIDI to virtual destinations.";
     RtMidi::error( RtError::WARNING, errorString_ );
   }
   return;
  }
  else if ( nBytes > 3 ) {
   errorString_ = "MidiOutCore::sendMessage: message format problem ... not sysex but > 3 bytes?";
   RtMidi::error( RtError::WARNING, errorString_ );
   return;
  }

  MIDIPacketList packetList;
  MIDIPacket *packet = MIDIPacketListInit( &packetList );
  packet = MIDIPacketListAdd( &packetList, sizeof(packetList), packet, timeStamp, nBytes, (const Byte *) &message->at( 0 ) );
  if ( !packet ) {
    errorString_ = "MidiOutCore::sendMessage: could not allocate packet list";      
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Send to any destinations that may have connected to us.
  if ( data->endpoint ) {
    result = MIDIReceived( data->endpoint, &packetList );
    if ( result != noErr ) {
      errorString_ = "MidiOutCore::sendMessage: error sending MIDI to virtual destinations.";
      RtMidi::error( RtError::WARNING, errorString_ );
    }
  }

  // And send to an explicit destination port if we're connected.
  if ( connected_ ) {
    result = MIDISend( data->port, data->destinationId, &packetList );
    if ( result != noErr ) {
      errorString_ = "MidiOutCore::sendMessage: error sending MIDI message to port.";
      RtMidi::error( RtError::WARNING, errorString_ );
    }
  }

}

#endif  // __MACOSX_CORE__


//*********************************************************************//
//  API: LINUX ALSA SEQUENCER
//*********************************************************************//

// API information found at:
//   - http://www.alsa-project.org/documentation.php#Library

#if defined(__LINUX_ALSA__)

// The ALSA Sequencer API is based on the use of a callback function for
// MIDI input.
//
// Thanks to Pedro Lopez-Cabanillas for help with the ALSA sequencer
// time stamps and other assorted fixes!!!

// If you don't need timestamping for incoming MIDI events, define the
// preprocessor definition AVOID_TIMESTAMPING to save resources
// associated with the ALSA sequencer queues.

#include <pthread.h>
#include <sys/time.h>

// ALSA header file.
#include <alsa/asoundlib.h>

// Global sequencer instance created when first In/Out object is
// created, then destroyed when last In/Out is deleted.
static snd_seq_t *s_seq = NULL;

// Variable to keep track of how many ports are open.
static unsigned int s_numPorts = 0;

// The client name to use when creating the sequencer, which is
// currently set on the first call to createSequencer.
static std::string s_clientName = "RtMidi Client";

// A structure to hold variables related to the ALSA API
// implementation.
struct AlsaMidiData {
  snd_seq_t *seq;
  unsigned int portNum;
  int vport;
  snd_seq_port_subscribe_t *subscription;
  snd_midi_event_t *coder;
  unsigned int bufferSize;
  unsigned char *buffer;
  pthread_t thread;
  pthread_t dummy_thread_id;
  unsigned long long lastTime;
  int queue_id; // an input queue is needed to get timestamped events
  int trigger_fds[2];
};

#define PORT_TYPE( pinfo, bits ) ((snd_seq_port_info_get_capability(pinfo) & (bits)) == (bits))

snd_seq_t* createSequencer( const std::string& clientName )
{
  // Set up the ALSA sequencer client.
  if ( s_seq == NULL ) {
    int result = snd_seq_open(&s_seq, "default", SND_SEQ_OPEN_DUPLEX, SND_SEQ_NONBLOCK);
    if ( result < 0 ) {
      s_seq = NULL;
    }
    else {
      // Set client name, use current name if given string is empty.
      if ( clientName != "" ) {
        s_clientName = clientName;
      }
      snd_seq_set_client_name( s_seq, s_clientName.c_str() );
    }
  }

  // Increment port count.
  s_numPorts++;

  return s_seq;
}

void freeSequencer ( void )
{
  s_numPorts--;
  if ( s_numPorts == 0 && s_seq != NULL ) {
    snd_seq_close( s_seq );
    s_seq = NULL;
  }
}

//*********************************************************************//
//  API: LINUX ALSA
//  Class Definitions: MidiInAlsa
//*********************************************************************//

extern "C" void *alsaMidiHandler( void *ptr )
{
  MidiInApi::RtMidiInData *data = static_cast<MidiInApi::RtMidiInData *> (ptr);
  AlsaMidiData *apiData = static_cast<AlsaMidiData *> (data->apiData);

  long nBytes;
  unsigned long long time, lastTime;
  bool continueSysex = false;
  bool doDecode = false;
  MidiInApi::MidiMessage message;
  int poll_fd_count;
  struct pollfd *poll_fds;

  snd_seq_event_t *ev;
  int result;
  apiData->bufferSize = 32;
  result = snd_midi_event_new( 0, &apiData->coder );
  if ( result < 0 ) {
    data->doInput = false;
    std::cerr << "\nMidiInAlsa::alsaMidiHandler: error initializing MIDI event parser!\n\n";
    return 0;
  }
  unsigned char *buffer = (unsigned char *) malloc( apiData->bufferSize );
  if ( buffer == NULL ) {
    data->doInput = false;
    snd_midi_event_free( apiData->coder );
    apiData->coder = 0;
    std::cerr << "\nMidiInAlsa::alsaMidiHandler: error initializing buffer memory!\n\n";
    return 0;
  }
  snd_midi_event_init( apiData->coder );
  snd_midi_event_no_status( apiData->coder, 1 ); // suppress running status messages

  poll_fd_count = snd_seq_poll_descriptors_count( apiData->seq, POLLIN ) + 1;
  poll_fds = (struct pollfd*)alloca( poll_fd_count * sizeof( struct pollfd ));
  snd_seq_poll_descriptors( apiData->seq, poll_fds + 1, poll_fd_count - 1, POLLIN );
  poll_fds[0].fd = apiData->trigger_fds[0];
  poll_fds[0].events = POLLIN;

  while ( data->doInput ) {

    if ( snd_seq_event_input_pending( apiData->seq, 1 ) == 0 ) {
      // No data pending
      if ( poll( poll_fds, poll_fd_count, -1) >= 0 ) {
        if ( poll_fds[0].revents & POLLIN ) {
          bool dummy;
          int res = read( poll_fds[0].fd, &dummy, sizeof(dummy) );
          (void) res;
        }
      }
      continue;
    }

    // If here, there should be data.
    result = snd_seq_event_input( apiData->seq, &ev );
    if ( result == -ENOSPC ) {
      std::cerr << "\nMidiInAlsa::alsaMidiHandler: MIDI input buffer overrun!\n\n";
      continue;
    }
    else if ( result <= 0 ) {
      std::cerr << "MidiInAlsa::alsaMidiHandler: unknown MIDI input error!\n";
      continue;
    }

    // This is a bit weird, but we now have to decode an ALSA MIDI
    // event (back) into MIDI bytes.  We'll ignore non-MIDI types.
    if ( !continueSysex ) message.bytes.clear();

    doDecode = false;
    switch ( ev->type ) {

		case SND_SEQ_EVENT_PORT_SUBSCRIBED:
#if defined(__RTMIDI_DEBUG__)
      std::cout << "MidiInAlsa::alsaMidiHandler: port connection made!\n";
#endif
      break;

		case SND_SEQ_EVENT_PORT_UNSUBSCRIBED:
#if defined(__RTMIDI_DEBUG__)
      std::cerr << "MidiInAlsa::alsaMidiHandler: port connection has closed!\n";
      std::cout << "sender = " << (int) ev->data.connect.sender.client << ":"
                << (int) ev->data.connect.sender.port
                << ", dest = " << (int) ev->data.connect.dest.client << ":"
                << (int) ev->data.connect.dest.port
                << std::endl;
#endif
      break;

    case SND_SEQ_EVENT_QFRAME: // MIDI time code
      if ( !( data->ignoreFlags & 0x02 ) ) doDecode = true;
      break;

    case SND_SEQ_EVENT_TICK: // MIDI timing tick
      if ( !( data->ignoreFlags & 0x02 ) ) doDecode = true;
      break;

    case SND_SEQ_EVENT_SENSING: // Active sensing
      if ( !( data->ignoreFlags & 0x04 ) ) doDecode = true;
      break;

		case SND_SEQ_EVENT_SYSEX:
      if ( (data->ignoreFlags & 0x01) ) break;
      if ( ev->data.ext.len > apiData->bufferSize ) {
        apiData->bufferSize = ev->data.ext.len;
        free( buffer );
        buffer = (unsigned char *) malloc( apiData->bufferSize );
        if ( buffer == NULL ) {
          data->doInput = false;
          std::cerr << "\nMidiInAlsa::alsaMidiHandler: error resizing buffer memory!\n\n";
          break;
        }
      }

    default:
      doDecode = true;
    }

    if ( doDecode ) {

      nBytes = snd_midi_event_decode( apiData->coder, buffer, apiData->bufferSize, ev );
      if ( nBytes > 0 ) {
        // The ALSA sequencer has a maximum buffer size for MIDI sysex
        // events of 256 bytes.  If a device sends sysex messages larger
        // than this, they are segmented into 256 byte chunks.  So,
        // we'll watch for this and concatenate sysex chunks into a
        // single sysex message if necessary.
        if ( !continueSysex )
          message.bytes.assign( buffer, &buffer[nBytes] );
        else
          message.bytes.insert( message.bytes.end(), buffer, &buffer[nBytes] );

        continueSysex = ( ( ev->type == SND_SEQ_EVENT_SYSEX ) && ( message.bytes.back() != 0xF7 ) );
        if ( !continueSysex ) {

          // Calculate the time stamp:
          message.timeStamp = 0.0;

          // Method 1: Use the system time.
          //(void)gettimeofday(&tv, (struct timezone *)NULL);
          //time = (tv.tv_sec * 1000000) + tv.tv_usec;

          // Method 2: Use the ALSA sequencer event time data.
          // (thanks to Pedro Lopez-Cabanillas!).
          time = ( ev->time.time.tv_sec * 1000000 ) + ( ev->time.time.tv_nsec/1000 );
          lastTime = time;
          time -= apiData->lastTime;
          apiData->lastTime = lastTime;
          if ( data->firstMessage == true )
            data->firstMessage = false;
          else
            message.timeStamp = time * 0.000001;
        }
        else {
#if defined(__RTMIDI_DEBUG__)
          std::cerr << "\nMidiInAlsa::alsaMidiHandler: event parsing error or not a MIDI event!\n\n";
#endif
        }
      }
    }

    snd_seq_free_event( ev );
    if ( message.bytes.size() == 0 || continueSysex ) continue;

    if ( data->usingCallback ) {
      RtMidiIn::RtMidiCallback callback = (RtMidiIn::RtMidiCallback) data->userCallback;
      callback( message.timeStamp, &message.bytes, data->userData );
    }
    else {
      // As long as we haven't reached our queue size limit, push the message.
      if ( data->queue.size < data->queue.ringSize ) {
        data->queue.ring[data->queue.back++] = message;
        if ( data->queue.back == data->queue.ringSize )
          data->queue.back = 0;
        data->queue.size++;
      }
      else
        std::cerr << "\nMidiInAlsa: message queue limit reached!!\n\n";
    }
  }

  if ( buffer ) free( buffer );
  snd_midi_event_free( apiData->coder );
  apiData->coder = 0;
  apiData->thread = apiData->dummy_thread_id;
  return 0;
}

MidiInAlsa :: MidiInAlsa( const std::string clientName, unsigned int queueSizeLimit ) : MidiInApi( queueSizeLimit )
{
  initialize( clientName );
}

MidiInAlsa :: ~MidiInAlsa()
{
  // Close a connection if it exists.
  closePort();

  // Shutdown the input thread.
  AlsaMidiData *data = static_cast<AlsaMidiData *> (apiData_);
  if ( inputData_.doInput ) {
    inputData_.doInput = false;
    int res = write( data->trigger_fds[1], &inputData_.doInput, sizeof(inputData_.doInput) );
    (void) res;
    if ( !pthread_equal(data->thread, data->dummy_thread_id) )
      pthread_join( data->thread, NULL );
  }

  // Cleanup.
  close ( data->trigger_fds[0] );
  close ( data->trigger_fds[1] );
  if ( data->vport >= 0 ) snd_seq_delete_port( data->seq, data->vport );
#ifndef AVOID_TIMESTAMPING
  snd_seq_free_queue( data->seq, data->queue_id );
#endif
  freeSequencer();
  delete data;
}

void MidiInAlsa :: initialize( const std::string& clientName )
{
  snd_seq_t* seq = createSequencer( clientName );
  if ( seq == NULL ) {
    s_seq = NULL;
    errorString_ = "MidiInAlsa::initialize: error creating ALSA sequencer client object.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Save our api-specific connection information.
  AlsaMidiData *data = (AlsaMidiData *) new AlsaMidiData;
  data->seq = seq;
  data->portNum = -1;
  data->vport = -1;
  data->subscription = 0;
  data->dummy_thread_id = pthread_self();
  data->thread = data->dummy_thread_id;
  data->trigger_fds[0] = -1;
  data->trigger_fds[1] = -1;
  apiData_ = (void *) data;
  inputData_.apiData = (void *) data;

   if ( pipe(data->trigger_fds) == -1 ) {
    errorString_ = "MidiInAlsa::initialize: error creating pipe objects.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Create the input queue
#ifndef AVOID_TIMESTAMPING
  data->queue_id = snd_seq_alloc_named_queue(s_seq, "RtMidi Queue");
  // Set arbitrary tempo (mm=100) and resolution (240)
  snd_seq_queue_tempo_t *qtempo;
  snd_seq_queue_tempo_alloca(&qtempo);
  snd_seq_queue_tempo_set_tempo(qtempo, 600000);
  snd_seq_queue_tempo_set_ppq(qtempo, 240);
  snd_seq_set_queue_tempo(data->seq, data->queue_id, qtempo);
  snd_seq_drain_output(data->seq);
#endif
}

// This function is used to count or get the pinfo structure for a given port number.
unsigned int portInfo( snd_seq_t *seq, snd_seq_port_info_t *pinfo, unsigned int type, int portNumber )
{
  snd_seq_client_info_t *cinfo;
  int client;
  int count = 0;
  snd_seq_client_info_alloca( &cinfo );

  snd_seq_client_info_set_client( cinfo, -1 );
  while ( snd_seq_query_next_client( seq, cinfo ) >= 0 ) {
    client = snd_seq_client_info_get_client( cinfo );
    if ( client == 0 ) continue;
    // Reset query info
    snd_seq_port_info_set_client( pinfo, client );
    snd_seq_port_info_set_port( pinfo, -1 );
    while ( snd_seq_query_next_port( seq, pinfo ) >= 0 ) {
      unsigned int atyp = snd_seq_port_info_get_type( pinfo );
      if ( ( atyp & SND_SEQ_PORT_TYPE_MIDI_GENERIC ) == 0 ) continue;
      unsigned int caps = snd_seq_port_info_get_capability( pinfo );
      if ( ( caps & type ) != type ) continue;
      if ( count == portNumber ) return 1;
      ++count;
    }
  }

  // If a negative portNumber was used, return the port count.
  if ( portNumber < 0 ) return count;
  return 0;
}

unsigned int MidiInAlsa :: getPortCount()
{
  snd_seq_port_info_t *pinfo;
  snd_seq_port_info_alloca( &pinfo );

  AlsaMidiData *data = static_cast<AlsaMidiData *> (apiData_);
  return portInfo( data->seq, pinfo, SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ, -1 );
}

std::string MidiInAlsa :: getPortName( unsigned int portNumber )
{
  snd_seq_client_info_t *cinfo;
  snd_seq_port_info_t *pinfo;
  snd_seq_client_info_alloca( &cinfo );
  snd_seq_port_info_alloca( &pinfo );

  std::string stringName;
  AlsaMidiData *data = static_cast<AlsaMidiData *> (apiData_);
  if ( portInfo( data->seq, pinfo, SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ, (int) portNumber ) ) {
    int cnum = snd_seq_port_info_get_client( pinfo );
    snd_seq_get_any_client_info( data->seq, cnum, cinfo );
    std::ostringstream os;
    os << snd_seq_client_info_get_name( cinfo );
    os << " ";                                    // GO: These lines added to make sure devices are listed
    os << snd_seq_port_info_get_client( pinfo );  // GO: with full portnames added to ensure individual device names
    os << ":";
    os << snd_seq_port_info_get_port( pinfo );
    stringName = os.str();
    return stringName;
  }

  // If we get here, we didn't find a match.
  errorString_ = "MidiInAlsa::getPortName: error looking for port name!";
  RtMidi::error( RtError::WARNING, errorString_ );
  return stringName;
  //RtMidi::error( RtError::INVALID_PARAMETER, errorString_ );
}

void MidiInAlsa :: openPort( unsigned int portNumber, const std::string portName )
{
  if ( connected_ ) {
    errorString_ = "MidiInAlsa::openPort: a valid connection already exists!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return;
  }

  unsigned int nSrc = this->getPortCount();
  if (nSrc < 1) {
    errorString_ = "MidiInAlsa::openPort: no MIDI input sources found!";
    RtMidi::error( RtError::NO_DEVICES_FOUND, errorString_ );
  }

  snd_seq_port_info_t *pinfo;
  snd_seq_port_info_alloca( &pinfo );
  std::ostringstream ost;
  AlsaMidiData *data = static_cast<AlsaMidiData *> (apiData_);
  if ( portInfo( data->seq, pinfo, SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ, (int) portNumber ) == 0 ) {
    ost << "MidiInAlsa::openPort: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::INVALID_PARAMETER, errorString_ );
  }


  snd_seq_addr_t sender, receiver;
  sender.client = snd_seq_port_info_get_client( pinfo );
  sender.port = snd_seq_port_info_get_port( pinfo );
  receiver.client = snd_seq_client_id( data->seq );
  if ( data->vport < 0 ) {
    snd_seq_port_info_set_client( pinfo, 0 );
    snd_seq_port_info_set_port( pinfo, 0 );
    snd_seq_port_info_set_capability( pinfo,
                                      SND_SEQ_PORT_CAP_WRITE |
                                      SND_SEQ_PORT_CAP_SUBS_WRITE );
    snd_seq_port_info_set_type( pinfo,
                                SND_SEQ_PORT_TYPE_MIDI_GENERIC |
                                SND_SEQ_PORT_TYPE_APPLICATION );
    snd_seq_port_info_set_midi_channels(pinfo, 16);
#ifndef AVOID_TIMESTAMPING
    snd_seq_port_info_set_timestamping(pinfo, 1);
    snd_seq_port_info_set_timestamp_real(pinfo, 1);    
    snd_seq_port_info_set_timestamp_queue(pinfo, data->queue_id);
#endif
    snd_seq_port_info_set_name(pinfo,  portName.c_str() );
    data->vport = snd_seq_create_port(data->seq, pinfo);
  
    if ( data->vport < 0 ) {
      errorString_ = "MidiInAlsa::openPort: ALSA error creating input port.";
      RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    }
  }

  receiver.port = data->vport;

  if ( !data->subscription ) {
    // Make subscription
    if (snd_seq_port_subscribe_malloc( &data->subscription ) < 0) {
      errorString_ = "MidiInAlsa::openPort: ALSA error allocation port subscription.";
      RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    }
    snd_seq_port_subscribe_set_sender(data->subscription, &sender);
    snd_seq_port_subscribe_set_dest(data->subscription, &receiver);
    if ( snd_seq_subscribe_port(data->seq, data->subscription) ) {
      snd_seq_port_subscribe_free( data->subscription );
      data->subscription = 0;
      errorString_ = "MidiInAlsa::openPort: ALSA error making port connection.";
      RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    }
  }

  if ( inputData_.doInput == false ) {
    // Start the input queue
#ifndef AVOID_TIMESTAMPING
    snd_seq_start_queue( data->seq, data->queue_id, NULL );
    snd_seq_drain_output( data->seq );
#endif
    // Start our MIDI input thread.
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setschedpolicy(&attr, SCHED_OTHER);

    inputData_.doInput = true;
    int err = pthread_create(&data->thread, &attr, alsaMidiHandler, &inputData_);
    pthread_attr_destroy(&attr);
    if ( err ) {
      snd_seq_unsubscribe_port( data->seq, data->subscription );
      snd_seq_port_subscribe_free( data->subscription );
      data->subscription = 0;
      inputData_.doInput = false;
      errorString_ = "MidiInAlsa::openPort: error starting MIDI input thread!";
      RtMidi::error( RtError::THREAD_ERROR, errorString_ );
    }
  }

  connected_ = true;
}

void MidiInAlsa :: openVirtualPort( std::string portName )
{
  AlsaMidiData *data = static_cast<AlsaMidiData *> (apiData_);
  if ( data->vport < 0 ) {
    snd_seq_port_info_t *pinfo;
    snd_seq_port_info_alloca( &pinfo );
    snd_seq_port_info_set_capability( pinfo,
				      SND_SEQ_PORT_CAP_WRITE |
				      SND_SEQ_PORT_CAP_SUBS_WRITE );
    snd_seq_port_info_set_type( pinfo,
				SND_SEQ_PORT_TYPE_MIDI_GENERIC |
				SND_SEQ_PORT_TYPE_APPLICATION );
    snd_seq_port_info_set_midi_channels(pinfo, 16);
#ifndef AVOID_TIMESTAMPING
    snd_seq_port_info_set_timestamping(pinfo, 1);
    snd_seq_port_info_set_timestamp_real(pinfo, 1);    
    snd_seq_port_info_set_timestamp_queue(pinfo, data->queue_id);
#endif
    snd_seq_port_info_set_name(pinfo, portName.c_str());
    data->vport = snd_seq_create_port(data->seq, pinfo);

    if ( data->vport < 0 ) {
      errorString_ = "MidiInAlsa::openVirtualPort: ALSA error creating virtual port.";
      RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    }
  }

  if ( inputData_.doInput == false ) {
    // Wait for old thread to stop, if still running
    if ( !pthread_equal(data->thread, data->dummy_thread_id) )
      pthread_join( data->thread, NULL );

    // Start the input queue
#ifndef AVOID_TIMESTAMPING
    snd_seq_start_queue( data->seq, data->queue_id, NULL );
    snd_seq_drain_output( data->seq );
#endif
    // Start our MIDI input thread.
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setschedpolicy(&attr, SCHED_OTHER);

    inputData_.doInput = true;
    int err = pthread_create(&data->thread, &attr, alsaMidiHandler, &inputData_);
    pthread_attr_destroy(&attr);
    if ( err ) {
      if ( data->subscription ) {
        snd_seq_unsubscribe_port( data->seq, data->subscription );
        snd_seq_port_subscribe_free( data->subscription );
        data->subscription = 0;
      }
      inputData_.doInput = false;
      errorString_ = "MidiInAlsa::openPort: error starting MIDI input thread!";
      RtMidi::error( RtError::THREAD_ERROR, errorString_ );
    }
  }
}

void MidiInAlsa :: closePort( void )
{
  AlsaMidiData *data = static_cast<AlsaMidiData *> (apiData_);

  if ( connected_ ) {
    if ( data->subscription ) {
      snd_seq_unsubscribe_port( data->seq, data->subscription );
      snd_seq_port_subscribe_free( data->subscription );
      data->subscription = 0;
    }
    // Stop the input queue
#ifndef AVOID_TIMESTAMPING
    snd_seq_stop_queue( data->seq, data->queue_id, NULL );
    snd_seq_drain_output( data->seq );
#endif
    connected_ = false;
  }

  // Stop thread to avoid triggering the callback, while the port is intended to be closed
  if ( inputData_.doInput ) {
    inputData_.doInput = false;
    int res = write( data->trigger_fds[1], &inputData_.doInput, sizeof(inputData_.doInput) );
    (void) res;
    if ( !pthread_equal(data->thread, data->dummy_thread_id) )
      pthread_join( data->thread, NULL );
  }
}

//*********************************************************************//
//  API: LINUX ALSA
//  Class Definitions: MidiOutAlsa
//*********************************************************************//

MidiOutAlsa :: MidiOutAlsa( const std::string clientName ) : MidiOutApi()
{
  initialize( clientName );
}

MidiOutAlsa :: ~MidiOutAlsa()
{
  // Close a connection if it exists.
  closePort();

  // Cleanup.
  AlsaMidiData *data = static_cast<AlsaMidiData *> (apiData_);
  if ( data->vport >= 0 ) snd_seq_delete_port( data->seq, data->vport );
  if ( data->coder ) snd_midi_event_free( data->coder );
  if ( data->buffer ) free( data->buffer );
  freeSequencer();
  delete data;
}

void MidiOutAlsa :: initialize( const std::string& clientName )
{
  snd_seq_t* seq = createSequencer( clientName );
  if ( seq == NULL ) {
    s_seq = NULL;
    errorString_ = "MidiOutAlsa::initialize: error creating ALSA sequencer client object.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
	}

  // Save our api-specific connection information.
  AlsaMidiData *data = (AlsaMidiData *) new AlsaMidiData;
  data->seq = seq;
  data->portNum = -1;
  data->vport = -1;
  data->bufferSize = 32;
  data->coder = 0;
  data->buffer = 0;
  int result = snd_midi_event_new( data->bufferSize, &data->coder );
  if ( result < 0 ) {
    delete data;
    errorString_ = "MidiOutAlsa::initialize: error initializing MIDI event parser!\n\n";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }
  data->buffer = (unsigned char *) malloc( data->bufferSize );
  if ( data->buffer == NULL ) {
    delete data;
    errorString_ = "MidiOutAlsa::initialize: error allocating buffer memory!\n\n";
    RtMidi::error( RtError::MEMORY_ERROR, errorString_ );
  }
  snd_midi_event_init( data->coder );
  apiData_ = (void *) data;
}

unsigned int MidiOutAlsa :: getPortCount()
{
	snd_seq_port_info_t *pinfo;
	snd_seq_port_info_alloca( &pinfo );

  AlsaMidiData *data = static_cast<AlsaMidiData *> (apiData_);
  return portInfo( data->seq, pinfo, SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE, -1 );
}

std::string MidiOutAlsa :: getPortName( unsigned int portNumber )
{
  snd_seq_client_info_t *cinfo;
  snd_seq_port_info_t *pinfo;
  snd_seq_client_info_alloca( &cinfo );
  snd_seq_port_info_alloca( &pinfo );

  std::string stringName;
  AlsaMidiData *data = static_cast<AlsaMidiData *> (apiData_);
  if ( portInfo( data->seq, pinfo, SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE, (int) portNumber ) ) {
    int cnum = snd_seq_port_info_get_client(pinfo);
    snd_seq_get_any_client_info( data->seq, cnum, cinfo );
    std::ostringstream os;
    os << snd_seq_client_info_get_name(cinfo);
    os << ":";
    os << snd_seq_port_info_get_port(pinfo);
    stringName = os.str();
    return stringName;
  }

  // If we get here, we didn't find a match.
  errorString_ = "MidiOutAlsa::getPortName: error looking for port name!";
  //RtMidi::error( RtError::INVALID_PARAMETER, errorString_ );
  RtMidi::error( RtError::WARNING, errorString_ );
  return stringName;
}

void MidiOutAlsa :: openPort( unsigned int portNumber, const std::string portName )
{
  if ( connected_ ) {
    errorString_ = "MidiOutAlsa::openPort: a valid connection already exists!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return;
  }

  unsigned int nSrc = this->getPortCount();
  if (nSrc < 1) {
    errorString_ = "MidiOutAlsa::openPort: no MIDI output sources found!";
    RtMidi::error( RtError::NO_DEVICES_FOUND, errorString_ );
  }

	snd_seq_port_info_t *pinfo;
	snd_seq_port_info_alloca( &pinfo );
  std::ostringstream ost;
  AlsaMidiData *data = static_cast<AlsaMidiData *> (apiData_);
  if ( portInfo( data->seq, pinfo, SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE, (int) portNumber ) == 0 ) {
    ost << "MidiOutAlsa::openPort: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::INVALID_PARAMETER, errorString_ );
  }

  snd_seq_addr_t sender, receiver;
  receiver.client = snd_seq_port_info_get_client( pinfo );
  receiver.port = snd_seq_port_info_get_port( pinfo );
  sender.client = snd_seq_client_id( data->seq );

  if ( data->vport < 0 ) {
    data->vport = snd_seq_create_simple_port( data->seq, portName.c_str(),
                                              SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ,
                                              SND_SEQ_PORT_TYPE_MIDI_GENERIC|SND_SEQ_PORT_TYPE_APPLICATION );
    if ( data->vport < 0 ) {
      errorString_ = "MidiOutAlsa::openPort: ALSA error creating output port.";
      RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    }
  }

  sender.port = data->vport;

  // Make subscription
  if (snd_seq_port_subscribe_malloc( &data->subscription ) < 0) {
    snd_seq_port_subscribe_free( data->subscription );
    errorString_ = "MidiOutAlsa::openPort: error allocation port subscribtion.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }
  snd_seq_port_subscribe_set_sender(data->subscription, &sender);
  snd_seq_port_subscribe_set_dest(data->subscription, &receiver);
  snd_seq_port_subscribe_set_time_update(data->subscription, 1);
  snd_seq_port_subscribe_set_time_real(data->subscription, 1);
  if ( snd_seq_subscribe_port(data->seq, data->subscription) ) {
    snd_seq_port_subscribe_free( data->subscription );
    errorString_ = "MidiOutAlsa::openPort: ALSA error making port connection.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  connected_ = true;
}

void MidiOutAlsa :: closePort( void )
{
  if ( connected_ ) {
    AlsaMidiData *data = static_cast<AlsaMidiData *> (apiData_);
    snd_seq_unsubscribe_port( data->seq, data->subscription );
    snd_seq_port_subscribe_free( data->subscription );
    connected_ = false;
  }
}

void MidiOutAlsa :: openVirtualPort( std::string portName )
{
  AlsaMidiData *data = static_cast<AlsaMidiData *> (apiData_);
  if ( data->vport < 0 ) {
    data->vport = snd_seq_create_simple_port( data->seq, portName.c_str(),
                                              SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ,
                                              SND_SEQ_PORT_TYPE_MIDI_GENERIC|SND_SEQ_PORT_TYPE_APPLICATION );

    if ( data->vport < 0 ) {
      errorString_ = "MidiOutAlsa::openVirtualPort: ALSA error creating virtual port.";
      RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    }
  }
}

void MidiOutAlsa :: sendMessage( std::vector<unsigned char> *message )
{
  int result;
  AlsaMidiData *data = static_cast<AlsaMidiData *> (apiData_);
  unsigned int nBytes = message->size();
  if ( nBytes > data->bufferSize ) {
    data->bufferSize = nBytes;
    result = snd_midi_event_resize_buffer ( data->coder, nBytes);
    if ( result != 0 ) {
      errorString_ = "MidiOutAlsa::sendMessage: ALSA error resizing MIDI event buffer.";
      RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    }
    free (data->buffer);
    data->buffer = (unsigned char *) malloc( data->bufferSize );
    if ( data->buffer == NULL ) {
    errorString_ = "MidiOutAlsa::initialize: error allocating buffer memory!\n\n";
    RtMidi::error( RtError::MEMORY_ERROR, errorString_ );
    }
  }

  snd_seq_event_t ev;
  snd_seq_ev_clear(&ev);
  snd_seq_ev_set_source(&ev, data->vport);
  snd_seq_ev_set_subs(&ev);
  snd_seq_ev_set_direct(&ev);
  for ( unsigned int i=0; i<nBytes; ++i ) data->buffer[i] = message->at(i);
  result = snd_midi_event_encode( data->coder, data->buffer, (long)nBytes, &ev );
  if ( result < (int)nBytes ) {
    errorString_ = "MidiOutAlsa::sendMessage: event parsing error!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return;
  }

  // Send the event.
  result = snd_seq_event_output(data->seq, &ev);
  if ( result < 0 ) {
    errorString_ = "MidiOutAlsa::sendMessage: error sending MIDI message to port.";
    RtMidi::error( RtError::WARNING, errorString_ );
  }
  snd_seq_drain_output(data->seq);
}

#endif // __LINUX_ALSA__


//*********************************************************************//
//  API: Windows Multimedia Library (MM)
//*********************************************************************//

// API information deciphered from:
//  - http://msdn.microsoft.com/library/default.asp?url=/library/en-us/multimed/htm/_win32_midi_reference.asp

// Thanks to Jean-Baptiste Berruchon for the sysex code.

#if defined(__WINDOWS_MM__)

// The Windows MM API is based on the use of a callback function for
// MIDI input.  We convert the system specific time stamps to delta
// time values.

// Windows MM MIDI header files.
#include <windows.h>
#include <mmsystem.h>

#define  RT_SYSEX_BUFFER_SIZE 1024
#define  RT_SYSEX_BUFFER_COUNT 4

// A structure to hold variables related to the CoreMIDI API
// implementation.
struct WinMidiData {
  HMIDIIN inHandle;    // Handle to Midi Input Device
  HMIDIOUT outHandle;  // Handle to Midi Output Device
  DWORD lastTime;
  MidiInApi::MidiMessage message;
  LPMIDIHDR sysexBuffer[RT_SYSEX_BUFFER_COUNT];
};

//*********************************************************************//
//  API: Windows MM
//  Class Definitions: MidiInWinMM
//*********************************************************************//

static void CALLBACK midiInputCallback( HMIDIIN hmin,
                                        UINT inputStatus, 
                                        DWORD_PTR instancePtr,
                                        DWORD_PTR midiMessage,
                                        DWORD timestamp )
{
  if ( inputStatus != MIM_DATA && inputStatus != MIM_LONGDATA && inputStatus != MIM_LONGERROR ) return;

  //MidiInApi::RtMidiInData *data = static_cast<MidiInApi::RtMidiInData *> (instancePtr);
  MidiInApi::RtMidiInData *data = (MidiInApi::RtMidiInData *)instancePtr;
  WinMidiData *apiData = static_cast<WinMidiData *> (data->apiData);

  // Calculate time stamp.
  if ( data->firstMessage == true ) {
    apiData->message.timeStamp = 0.0;
    data->firstMessage = false;
  }
  else apiData->message.timeStamp = (double) ( timestamp - apiData->lastTime ) * 0.001;
  apiData->lastTime = timestamp;

  if ( inputStatus == MIM_DATA ) { // Channel or system message

    // Make sure the first byte is a status byte.
    unsigned char status = (unsigned char) (midiMessage & 0x000000FF);
    if ( !(status & 0x80) ) return;

    // Determine the number of bytes in the MIDI message.
    unsigned short nBytes = 1;
    if ( status < 0xC0 ) nBytes = 3;
    else if ( status < 0xE0 ) nBytes = 2;
    else if ( status < 0xF0 ) nBytes = 3;
    else if ( status == 0xF1 ) {
      if ( data->ignoreFlags & 0x02 ) return;
      else nBytes = 2;
    }
    else if ( status == 0xF2 ) nBytes = 3;
    else if ( status == 0xF3 ) nBytes = 2;
    else if ( status == 0xF8 && (data->ignoreFlags & 0x02) ) {
      // A MIDI timing tick message and we're ignoring it.
      return;
    }
    else if ( status == 0xFE && (data->ignoreFlags & 0x04) ) {
      // A MIDI active sensing message and we're ignoring it.
      return;
    }

    // Copy bytes to our MIDI message.
    unsigned char *ptr = (unsigned char *) &midiMessage;
    for ( int i=0; i<nBytes; ++i ) apiData->message.bytes.push_back( *ptr++ );
  }
  else { // Sysex message ( MIM_LONGDATA or MIM_LONGERROR )
    MIDIHDR *sysex = ( MIDIHDR *) midiMessage; 
    if ( !( data->ignoreFlags & 0x01 ) && inputStatus != MIM_LONGERROR ) {  
      // Sysex message and we're not ignoring it
      for ( int i=0; i<(int)sysex->dwBytesRecorded; ++i )
        apiData->message.bytes.push_back( sysex->lpData[i] );
    }

    // The WinMM API requires that the sysex buffer be requeued after
    // input of each sysex message.  Even if we are ignoring sysex
    // messages, we still need to requeue the buffer in case the user
    // decides to not ignore sysex messages in the future.  However,
    // it seems that WinMM calls this function with an empty sysex
    // buffer when an application closes and in this case, we should
    // avoid requeueing it, else the computer suddenly reboots after
    // one or two minutes.
	if ( apiData->sysexBuffer[sysex->dwUser]->dwBytesRecorded > 0 ) {
    //if ( sysex->dwBytesRecorded > 0 ) {
      MMRESULT result = midiInAddBuffer( apiData->inHandle, apiData->sysexBuffer[sysex->dwUser], sizeof(MIDIHDR) );
      if ( result != MMSYSERR_NOERROR )
        std::cerr << "\nRtMidiIn::midiInputCallback: error sending sysex to Midi device!!\n\n";

      if ( data->ignoreFlags & 0x01 ) return;
    }
    else return;
  }

  if ( data->usingCallback ) {
    RtMidiIn::RtMidiCallback callback = (RtMidiIn::RtMidiCallback) data->userCallback;
    callback( apiData->message.timeStamp, &apiData->message.bytes, data->userData );
  }
  else {
    // As long as we haven't reached our queue size limit, push the message.
    if ( data->queue.size < data->queue.ringSize ) {
      data->queue.ring[data->queue.back++] = apiData->message;
      if ( data->queue.back == data->queue.ringSize )
        data->queue.back = 0;
      data->queue.size++;
    }
    else
      std::cerr << "\nRtMidiIn: message queue limit reached!!\n\n";
  }

  // Clear the vector for the next input message.
  apiData->message.bytes.clear();
}

MidiInWinMM :: MidiInWinMM( const std::string clientName, unsigned int queueSizeLimit ) : MidiInApi( queueSizeLimit )
{
  initialize( clientName );
}

MidiInWinMM :: ~MidiInWinMM()
{
  // Close a connection if it exists.
  closePort();

  // Cleanup.
  WinMidiData *data = static_cast<WinMidiData *> (apiData_);
  delete data;
}

void MidiInWinMM :: initialize( const std::string& /*clientName*/ )
{
  // We'll issue a warning here if no devices are available but not
  // throw an error since the user can plugin something later.
  unsigned int nDevices = midiInGetNumDevs();
  if ( nDevices == 0 ) {
    errorString_ = "MidiInWinMM::initialize: no MIDI input devices currently available.";
    RtMidi::error( RtError::WARNING, errorString_ );
  }

  // Save our api-specific connection information.
  WinMidiData *data = (WinMidiData *) new WinMidiData;
  apiData_ = (void *) data;
  inputData_.apiData = (void *) data;
  data->message.bytes.clear();  // needs to be empty for first input message
}

void MidiInWinMM :: openPort( unsigned int portNumber, const std::string /*portName*/ )
{
  if ( connected_ ) {
    errorString_ = "MidiInWinMM::openPort: a valid connection already exists!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return;
  }

  unsigned int nDevices = midiInGetNumDevs();
  if (nDevices == 0) {
    errorString_ = "MidiInWinMM::openPort: no MIDI input sources found!";
    RtMidi::error( RtError::NO_DEVICES_FOUND, errorString_ );
  }

  std::ostringstream ost;
  if ( portNumber >= nDevices ) {
    ost << "MidiInWinMM::openPort: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::INVALID_PARAMETER, errorString_ );
  }

  WinMidiData *data = static_cast<WinMidiData *> (apiData_);
  MMRESULT result = midiInOpen( &data->inHandle,
                                portNumber,
                                (DWORD_PTR)&midiInputCallback,
                                (DWORD_PTR)&inputData_,
                                CALLBACK_FUNCTION );
  if ( result != MMSYSERR_NOERROR ) {
    errorString_ = "MidiInWinMM::openPort: error creating Windows MM MIDI input port.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Allocate and init the sysex buffers.
  for ( int i=0; i<RT_SYSEX_BUFFER_COUNT; ++i ) {
    data->sysexBuffer[i] = (MIDIHDR*) new char[ sizeof(MIDIHDR) ];
    data->sysexBuffer[i]->lpData = new char[ RT_SYSEX_BUFFER_SIZE ];
    data->sysexBuffer[i]->dwBufferLength = RT_SYSEX_BUFFER_SIZE;
    data->sysexBuffer[i]->dwUser = i; // We use the dwUser parameter as buffer indicator
    data->sysexBuffer[i]->dwFlags = 0;

    result = midiInPrepareHeader( data->inHandle, data->sysexBuffer[i], sizeof(MIDIHDR) );
    if ( result != MMSYSERR_NOERROR ) {
      midiInClose( data->inHandle );
      errorString_ = "MidiInWinMM::openPort: error starting Windows MM MIDI input port (PrepareHeader).";
      RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    }

    // Register the buffer.
    result = midiInAddBuffer( data->inHandle, data->sysexBuffer[i], sizeof(MIDIHDR) );
    if ( result != MMSYSERR_NOERROR ) {
      midiInClose( data->inHandle );
      errorString_ = "MidiInWinMM::openPort: error starting Windows MM MIDI input port (AddBuffer).";
      RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    }
  }

  result = midiInStart( data->inHandle );
  if ( result != MMSYSERR_NOERROR ) {
    midiInClose( data->inHandle );
    errorString_ = "MidiInWinMM::openPort: error starting Windows MM MIDI input port.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  connected_ = true;
}

void MidiInWinMM :: openVirtualPort( std::string portName )
{
  // This function cannot be implemented for the Windows MM MIDI API.
  errorString_ = "MidiInWinMM::openVirtualPort: cannot be implemented in Windows MM MIDI API!";
  RtMidi::error( RtError::WARNING, errorString_ );
}

void MidiInWinMM :: closePort( void )
{
  if ( connected_ ) {
    WinMidiData *data = static_cast<WinMidiData *> (apiData_);
    midiInReset( data->inHandle );
    midiInStop( data->inHandle );

    for ( int i=0; i<RT_SYSEX_BUFFER_COUNT; ++i ) {
      int result = midiInUnprepareHeader(data->inHandle, data->sysexBuffer[i], sizeof(MIDIHDR));
      delete [] data->sysexBuffer[i]->lpData;
      delete [] data->sysexBuffer[i];
      if ( result != MMSYSERR_NOERROR ) {
        midiInClose( data->inHandle );
        errorString_ = "MidiInWinMM::openPort: error closing Windows MM MIDI input port (midiInUnprepareHeader).";
        RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
      }
    }

    midiInClose( data->inHandle );
    connected_ = false;
  }
}

unsigned int MidiInWinMM :: getPortCount()
{
  return midiInGetNumDevs();
}

std::string MidiInWinMM :: getPortName( unsigned int portNumber )
{
  std::string stringName;
  unsigned int nDevices = midiInGetNumDevs();
  if ( portNumber >= nDevices ) {
    std::ostringstream ost;
    ost << "MidiInWinMM::getPortName: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    //RtMidi::error( RtError::INVALID_PARAMETER, errorString_ );
    RtMidi::error( RtError::WARNING, errorString_ );
    return stringName;
  }

  MIDIINCAPS deviceCaps;
  midiInGetDevCaps( portNumber, &deviceCaps, sizeof(MIDIINCAPS));

#if defined( UNICODE ) || defined( _UNICODE )
  int length = WideCharToMultiByte(CP_UTF8, 0, deviceCaps.szPname, -1, NULL, 0, NULL, NULL);
  stringName.assign( length, 0 );
  length = WideCharToMultiByte(CP_UTF8, 0, deviceCaps.szPname, wcslen(deviceCaps.szPname), &stringName[0], length, NULL, NULL);
#else
  stringName = std::string( deviceCaps.szPname );
#endif

  // Next lines added to add the portNumber to the name so that 
  // the device's names are sure to be listed with individual names
  // even when they have the same brand name
  std::ostringstream os;
  os << " ";
  os << portNumber;
  stringName += os.str();

  return stringName;
}

//*********************************************************************//
//  API: Windows MM
//  Class Definitions: MidiOutWinMM
//*********************************************************************//

MidiOutWinMM :: MidiOutWinMM( const std::string clientName ) : MidiOutApi()
{
  initialize( clientName );
}

MidiOutWinMM :: ~MidiOutWinMM()
{
  // Close a connection if it exists.
  closePort();

  // Cleanup.
  WinMidiData *data = static_cast<WinMidiData *> (apiData_);
  delete data;
}

void MidiOutWinMM :: initialize( const std::string& /*clientName*/ )
{
  // We'll issue a warning here if no devices are available but not
  // throw an error since the user can plug something in later.
  unsigned int nDevices = midiOutGetNumDevs();
  if ( nDevices == 0 ) {
    errorString_ = "MidiOutWinMM::initialize: no MIDI output devices currently available.";
    RtMidi::error( RtError::WARNING, errorString_ );
  }

  // Save our api-specific connection information.
  WinMidiData *data = (WinMidiData *) new WinMidiData;
  apiData_ = (void *) data;
}

unsigned int MidiOutWinMM :: getPortCount()
{
  return midiOutGetNumDevs();
}

std::string MidiOutWinMM :: getPortName( unsigned int portNumber )
{
  std::string stringName;
  unsigned int nDevices = midiOutGetNumDevs();
  if ( portNumber >= nDevices ) {
    std::ostringstream ost;
    ost << "MidiOutWinMM::getPortName: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    //RtMidi::error( RtError::INVALID_PARAMETER, errorString_ );
    RtMidi::error( RtError::WARNING, errorString_ );
    return stringName;
  }

  MIDIOUTCAPS deviceCaps;
  midiOutGetDevCaps( portNumber, &deviceCaps, sizeof(MIDIOUTCAPS));

#if defined( UNICODE ) || defined( _UNICODE )
  int length = WideCharToMultiByte(CP_UTF8, 0, deviceCaps.szPname, -1, NULL, 0, NULL, NULL);
  stringName.assign( length, 0 );
  length = WideCharToMultiByte(CP_UTF8, 0, deviceCaps.szPname, wcslen(deviceCaps.szPname), &stringName[0], length, NULL, NULL);
#else
  stringName = std::string( deviceCaps.szPname );
#endif

  return stringName;
}

void MidiOutWinMM :: openPort( unsigned int portNumber, const std::string /*portName*/ )
{
  if ( connected_ ) {
    errorString_ = "MidiOutWinMM::openPort: a valid connection already exists!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return;
  }

  unsigned int nDevices = midiOutGetNumDevs();
  if (nDevices < 1) {
    errorString_ = "MidiOutWinMM::openPort: no MIDI output destinations found!";
    RtMidi::error( RtError::NO_DEVICES_FOUND, errorString_ );
  }

  std::ostringstream ost;
  if ( portNumber >= nDevices ) {
    ost << "MidiOutWinMM::openPort: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::INVALID_PARAMETER, errorString_ );
  }

  WinMidiData *data = static_cast<WinMidiData *> (apiData_);
  MMRESULT result = midiOutOpen( &data->outHandle,
                                 portNumber,
                                 (DWORD)NULL,
                                 (DWORD)NULL,
                                 CALLBACK_NULL );
  if ( result != MMSYSERR_NOERROR ) {
    errorString_ = "MidiOutWinMM::openPort: error creating Windows MM MIDI output port.";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  connected_ = true;
}

void MidiOutWinMM :: closePort( void )
{
  if ( connected_ ) {
    WinMidiData *data = static_cast<WinMidiData *> (apiData_);
    midiOutReset( data->outHandle );
    midiOutClose( data->outHandle );
    connected_ = false;
  }
}

void MidiOutWinMM :: openVirtualPort( std::string portName )
{
  // This function cannot be implemented for the Windows MM MIDI API.
  errorString_ = "MidiOutWinMM::openVirtualPort: cannot be implemented in Windows MM MIDI API!";
  RtMidi::error( RtError::WARNING, errorString_ );
}

void MidiOutWinMM :: sendMessage( std::vector<unsigned char> *message )
{
  unsigned int nBytes = static_cast<unsigned int>(message->size());
  if ( nBytes == 0 ) {
    errorString_ = "MidiOutWinMM::sendMessage: message argument is empty!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return;
  }

  MMRESULT result;
  WinMidiData *data = static_cast<WinMidiData *> (apiData_);
  if ( message->at(0) == 0xF0 ) { // Sysex message

    // Allocate buffer for sysex data.
    char *buffer = (char *) malloc( nBytes );
    if ( buffer == NULL ) {
      errorString_ = "MidiOutWinMM::sendMessage: error allocating sysex message memory!";
      RtMidi::error( RtError::MEMORY_ERROR, errorString_ );
    }

    // Copy data to buffer.
    for ( unsigned int i=0; i<nBytes; ++i ) buffer[i] = message->at(i);

    // Create and prepare MIDIHDR structure.
    MIDIHDR sysex;
    sysex.lpData = (LPSTR) buffer;
    sysex.dwBufferLength = nBytes;
    sysex.dwFlags = 0;
    result = midiOutPrepareHeader( data->outHandle,  &sysex, sizeof(MIDIHDR) ); 
    if ( result != MMSYSERR_NOERROR ) {
      free( buffer );
      errorString_ = "MidiOutWinMM::sendMessage: error preparing sysex header.";
      RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    }

    // Send the message.
    result = midiOutLongMsg( data->outHandle, &sysex, sizeof(MIDIHDR) );
    if ( result != MMSYSERR_NOERROR ) {
      free( buffer );
      errorString_ = "MidiOutWinMM::sendMessage: error sending sysex message.";
      RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    }

    // Unprepare the buffer and MIDIHDR.
    while ( MIDIERR_STILLPLAYING == midiOutUnprepareHeader( data->outHandle, &sysex, sizeof (MIDIHDR) ) ) Sleep( 1 );
    free( buffer );

  }
  else { // Channel or system message.

    // Make sure the message size isn't too big.
    if ( nBytes > 3 ) {
      errorString_ = "MidiOutWinMM::sendMessage: message size is greater than 3 bytes (and not sysex)!";
      RtMidi::error( RtError::WARNING, errorString_ );
      return;
    }

    // Pack MIDI bytes into double word.
    DWORD packet;
    unsigned char *ptr = (unsigned char *) &packet;
    for ( unsigned int i=0; i<nBytes; ++i ) {
      *ptr = message->at(i);
      ++ptr;
    }

    // Send the message immediately.
    result = midiOutShortMsg( data->outHandle, packet );
    if ( result != MMSYSERR_NOERROR ) {
      errorString_ = "MidiOutWinMM::sendMessage: error sending MIDI message.";
      RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    }
  }
}

#endif  // __WINDOWS_MM__

// *********************************************************************//
// API: WINDOWS Kernel Streaming
//
// Written by Sebastien Alaiwan, 2012.
//
// NOTE BY GARY: much of the KS-specific code below probably should go in a separate file.
//
// *********************************************************************//

#if defined(__WINDOWS_KS__)

#include <string>
#include <vector>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <windows.h>
#include <setupapi.h>
#include <mmsystem.h>

#include "ks.h"
#include "ksmedia.h"

#define INSTANTIATE_GUID(a) GUID const a = { STATIC_ ## a }

INSTANTIATE_GUID(GUID_NULL);
INSTANTIATE_GUID(KSPROPSETID_Pin);
INSTANTIATE_GUID(KSPROPSETID_Connection);
INSTANTIATE_GUID(KSPROPSETID_Topology);
INSTANTIATE_GUID(KSINTERFACESETID_Standard);
INSTANTIATE_GUID(KSMEDIUMSETID_Standard);
INSTANTIATE_GUID(KSDATAFORMAT_TYPE_MUSIC);
INSTANTIATE_GUID(KSDATAFORMAT_SUBTYPE_MIDI);
INSTANTIATE_GUID(KSDATAFORMAT_SPECIFIER_NONE);

#undef INSTANTIATE_GUID

typedef std::basic_string<TCHAR> tstring;

inline bool IsValid(HANDLE handle)
{
  return handle != NULL && handle != INVALID_HANDLE_VALUE;
}

class ComException : public std::runtime_error
{
private:
  static std::string MakeString(std::string const& s, HRESULT hr)
  {
    std::stringstream ss;
    ss << "(error 0x" << std::hex << hr << ")";
    return s + ss.str();
  }

public:
  ComException(std::string const& s, HRESULT hr) :
    std::runtime_error(MakeString(s, hr))
  {
  }
};

template<typename TFilterType>
class CKsEnumFilters
{
public:
  ~CKsEnumFilters()
  {
    DestroyLists();
  }

  void EnumFilters(GUID const* categories, size_t numCategories)
  {
    DestroyLists();

    if (categories == 0)
      throw std::runtime_error("CKsEnumFilters: invalid argument");

    // Get a handle to the device set specified by the guid
    HDEVINFO hDevInfo = ::SetupDiGetClassDevs(&categories[0], NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (!IsValid(hDevInfo))
      throw std::runtime_error("CKsEnumFilters: no devices found");

    // Loop through members of the set and get details for each
    for (int iClassMember=0;;iClassMember++) {
      try {
        SP_DEVICE_INTERFACE_DATA DID;
        DID.cbSize = sizeof(DID);
        DID.Reserved = 0;

        bool fRes = ::SetupDiEnumDeviceInterfaces(hDevInfo, NULL, &categories[0], iClassMember, &DID);
        if (!fRes)
          break;

        // Get filter friendly name
        HKEY hRegKey = ::SetupDiOpenDeviceInterfaceRegKey(hDevInfo, &DID, 0, KEY_READ);
        if (hRegKey == INVALID_HANDLE_VALUE)
          throw std::runtime_error("CKsEnumFilters: interface has no registry");

        char friendlyName[256];
        DWORD dwSize = sizeof friendlyName;
        LONG lval = ::RegQueryValueEx(hRegKey, TEXT("FriendlyName"), NULL, NULL, (LPBYTE)friendlyName, &dwSize);
        ::RegCloseKey(hRegKey);
        if (lval != ERROR_SUCCESS)
          throw std::runtime_error("CKsEnumFilters: interface has no friendly name");

        // Get details for the device registered in this class
        DWORD const cbItfDetails = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA) + MAX_PATH * sizeof(WCHAR);
        std::vector<BYTE> buffer(cbItfDetails);

        SP_DEVICE_INTERFACE_DETAIL_DATA* pDevInterfaceDetails = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA*>(&buffer[0]);
        pDevInterfaceDetails->cbSize = sizeof(*pDevInterfaceDetails);

        SP_DEVINFO_DATA DevInfoData;
        DevInfoData.cbSize = sizeof(DevInfoData);
        DevInfoData.Reserved = 0;

        fRes = ::SetupDiGetDeviceInterfaceDetail(hDevInfo, &DID, pDevInterfaceDetails, cbItfDetails, NULL, &DevInfoData);
        if (!fRes)
          throw std::runtime_error("CKsEnumFilters: could not get interface details");

        // check additional category guids which may (or may not) have been supplied
        for (size_t i=1; i < numCategories; ++i) {
          SP_DEVICE_INTERFACE_DATA DIDAlias;
          DIDAlias.cbSize = sizeof(DIDAlias);
          DIDAlias.Reserved = 0;

          fRes = ::SetupDiGetDeviceInterfaceAlias(hDevInfo, &DID, &categories[i], &DIDAlias);
          if (!fRes)
            throw std::runtime_error("CKsEnumFilters: could not get interface alias");

          // Check if the this interface alias is enabled.
          if (!DIDAlias.Flags || (DIDAlias.Flags & SPINT_REMOVED))
            throw std::runtime_error("CKsEnumFilters: interface alias is not enabled");
        }

        std::auto_ptr<TFilterType> pFilter(new TFilterType(pDevInterfaceDetails->DevicePath, friendlyName));

        pFilter->Instantiate();
        pFilter->FindMidiPins();
        pFilter->Validate();

        m_Filters.push_back(pFilter.release());
      }
      catch (std::runtime_error const& e) {
      }
    }

    ::SetupDiDestroyDeviceInfoList(hDevInfo);
  }

private:
  void DestroyLists()
  {
    for (size_t i=0;i < m_Filters.size();++i)
      delete m_Filters[i];
    m_Filters.clear();
  }

public:
  // TODO: make this private.
  std::vector<TFilterType*> m_Filters;
};

class CKsObject
{
public:
  CKsObject(HANDLE handle) : m_handle(handle)
  {
  }

protected:
  HANDLE m_handle;

  void SetProperty(REFGUID guidPropertySet, ULONG nProperty, void* pvValue, ULONG cbValue)
  {
    KSPROPERTY ksProperty;
    memset(&ksProperty, 0, sizeof ksProperty);
    ksProperty.Set = guidPropertySet;
    ksProperty.Id = nProperty;
    ksProperty.Flags = KSPROPERTY_TYPE_SET;

    HRESULT hr = DeviceIoControlKsProperty(ksProperty, pvValue, cbValue);
    if (FAILED(hr))
      throw ComException("CKsObject::SetProperty: could not set property", hr);
  }

private:

  HRESULT DeviceIoControlKsProperty(KSPROPERTY& ksProperty, void* pvValue, ULONG cbValue)
  {
    ULONG ulReturned;
    return ::DeviceIoControl(
             m_handle,
             IOCTL_KS_PROPERTY,
             &ksProperty,
             sizeof(ksProperty),
             pvValue,
             cbValue,
             &ulReturned,
             NULL);
  }
};

class CKsPin;

class CKsFilter : public CKsObject
{
  friend class CKsPin;

public:
  CKsFilter(tstring const& name, std::string const& sFriendlyName);
  virtual ~CKsFilter();

  virtual void Instantiate();

  template<typename T>
  T GetPinProperty(ULONG nPinId, ULONG nProperty)
  {
    ULONG ulReturned = 0;
    T value;

    KSP_PIN ksPProp;
    ksPProp.Property.Set = KSPROPSETID_Pin;
    ksPProp.Property.Id = nProperty;
    ksPProp.Property.Flags = KSPROPERTY_TYPE_GET;
    ksPProp.PinId = nPinId;
    ksPProp.Reserved = 0;

    HRESULT hr = ::DeviceIoControl(
      m_handle,
      IOCTL_KS_PROPERTY,
      &ksPProp,
      sizeof(KSP_PIN),
      &value,
      sizeof(value),
      &ulReturned,
      NULL);
    if (FAILED(hr))
      throw ComException("CKsFilter::GetPinProperty: failed to retrieve property", hr);

    return value;
  }

  void GetPinPropertyMulti(ULONG nPinId, REFGUID guidPropertySet, ULONG nProperty, PKSMULTIPLE_ITEM* ppKsMultipleItem)
  {
    HRESULT hr;

    KSP_PIN ksPProp;
    ksPProp.Property.Set = guidPropertySet;
    ksPProp.Property.Id = nProperty;
    ksPProp.Property.Flags = KSPROPERTY_TYPE_GET;
    ksPProp.PinId = nPinId;
    ksPProp.Reserved = 0;

    ULONG cbMultipleItem = 0;
    hr = ::DeviceIoControl(m_handle,
        IOCTL_KS_PROPERTY,
        &ksPProp.Property,
        sizeof(KSP_PIN),
        NULL,
        0,
        &cbMultipleItem,
        NULL);
    if (FAILED(hr))
      throw ComException("CKsFilter::GetPinPropertyMulti: cannot get property", hr);

    *ppKsMultipleItem = (PKSMULTIPLE_ITEM) new BYTE[cbMultipleItem];

    ULONG ulReturned = 0;
    hr = ::DeviceIoControl(
        m_handle,
        IOCTL_KS_PROPERTY,
        &ksPProp,
        sizeof(KSP_PIN),
        (PVOID)*ppKsMultipleItem,
        cbMultipleItem,
        &ulReturned,
        NULL);
    if (FAILED(hr))
      throw ComException("CKsFilter::GetPinPropertyMulti: cannot get property", hr);
  }

  std::string const& GetFriendlyName() const
  {
    return m_sFriendlyName;
  }

protected:

  std::vector<CKsPin*> m_Pins; // this list owns the pins.

  std::vector<CKsPin*> m_RenderPins;
  std::vector<CKsPin*> m_CapturePins;

private:
  std::string const m_sFriendlyName; // friendly name eg "Virus TI Synth"
  tstring const m_sName; // Filter path, eg "\\?\usb#vid_133e&pid_0815...\vtimidi02"
};

class CKsPin : public CKsObject
{
public:
  CKsPin(CKsFilter* pFilter, ULONG nId);
  virtual ~CKsPin();

  virtual void Instantiate();

  void ClosePin();

  void SetState(KSSTATE ksState);

  void WriteData(KSSTREAM_HEADER* pKSSTREAM_HEADER, OVERLAPPED* pOVERLAPPED);
  void ReadData(KSSTREAM_HEADER* pKSSTREAM_HEADER, OVERLAPPED* pOVERLAPPED);

  KSPIN_DATAFLOW GetDataFlow() const
  {
    return m_DataFlow;
  }

  bool IsSink() const
  {
    return m_Communication == KSPIN_COMMUNICATION_SINK
      || m_Communication == KSPIN_COMMUNICATION_BOTH;
  }


protected:
  PKSPIN_CONNECT m_pKsPinConnect;    // creation parameters of pin
  CKsFilter* const m_pFilter;

  ULONG m_cInterfaces;
  PKSIDENTIFIER m_pInterfaces;
  PKSMULTIPLE_ITEM m_pmiInterfaces;

  ULONG m_cMediums;
  PKSIDENTIFIER m_pMediums;
  PKSMULTIPLE_ITEM m_pmiMediums;

  ULONG m_cDataRanges;
  PKSDATARANGE m_pDataRanges;
  PKSMULTIPLE_ITEM m_pmiDataRanges;

  KSPIN_DATAFLOW m_DataFlow;
  KSPIN_COMMUNICATION m_Communication;
};

CKsFilter::CKsFilter(tstring const& sName, std::string const& sFriendlyName) :
  CKsObject(INVALID_HANDLE_VALUE),
  m_sFriendlyName(sFriendlyName),
  m_sName(sName)
{
  if (sName.empty())
    throw std::runtime_error("CKsFilter::CKsFilter: name can't be empty");
}

CKsFilter::~CKsFilter()
{
  for (size_t i=0;i < m_Pins.size();++i)
    delete m_Pins[i];

  if (IsValid(m_handle))
    ::CloseHandle(m_handle);
}

void CKsFilter::Instantiate()
{
  m_handle = CreateFile(
    m_sName.c_str(),
    GENERIC_READ | GENERIC_WRITE,
    0,
    NULL,
    OPEN_EXISTING,
    FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
    NULL);

  if (!IsValid(m_handle))
  {
    DWORD const dwError = GetLastError();
    throw ComException("CKsFilter::Instantiate: can't open driver", HRESULT_FROM_WIN32(dwError));
  }
}

CKsPin::CKsPin(CKsFilter* pFilter, ULONG PinId) :
  CKsObject(INVALID_HANDLE_VALUE),
  m_pKsPinConnect(NULL),
  m_pFilter(pFilter)
{
  m_Communication = m_pFilter->GetPinProperty<KSPIN_COMMUNICATION>(PinId, KSPROPERTY_PIN_COMMUNICATION);
  m_DataFlow = m_pFilter->GetPinProperty<KSPIN_DATAFLOW>(PinId, KSPROPERTY_PIN_DATAFLOW);

  // Interfaces
  m_pFilter->GetPinPropertyMulti(
      PinId,
      KSPROPSETID_Pin,
      KSPROPERTY_PIN_INTERFACES,
      &m_pmiInterfaces);

  m_cInterfaces = m_pmiInterfaces->Count;
  m_pInterfaces = (PKSPIN_INTERFACE)(m_pmiInterfaces + 1);

  // Mediums
  m_pFilter->GetPinPropertyMulti(
      PinId,
      KSPROPSETID_Pin,
      KSPROPERTY_PIN_MEDIUMS,
      &m_pmiMediums);

  m_cMediums = m_pmiMediums->Count;
  m_pMediums = (PKSPIN_MEDIUM)(m_pmiMediums + 1);

  // Data ranges
  m_pFilter->GetPinPropertyMulti(
      PinId,
      KSPROPSETID_Pin,
      KSPROPERTY_PIN_DATARANGES,
      &m_pmiDataRanges);

  m_cDataRanges = m_pmiDataRanges->Count;
  m_pDataRanges = (PKSDATARANGE)(m_pmiDataRanges + 1);
}

CKsPin::~CKsPin()
{
  ClosePin();

  delete[] (BYTE*)m_pKsPinConnect;
  delete[] (BYTE*)m_pmiDataRanges;
  delete[] (BYTE*)m_pmiInterfaces;
  delete[] (BYTE*)m_pmiMediums;
}

void CKsPin::ClosePin()
{
  if (IsValid(m_handle)) {
    SetState(KSSTATE_STOP);
    ::CloseHandle(m_handle);
  }
  m_handle = INVALID_HANDLE_VALUE;
}

void CKsPin::SetState(KSSTATE ksState)
{
  SetProperty(KSPROPSETID_Connection, KSPROPERTY_CONNECTION_STATE, &ksState, sizeof(ksState));
}

void CKsPin::Instantiate()
{
  if (!m_pKsPinConnect)
    throw std::runtime_error("CKsPin::Instanciate: abstract pin");

  DWORD const dwResult = KsCreatePin(m_pFilter->m_handle, m_pKsPinConnect, GENERIC_WRITE | GENERIC_READ, &m_handle);
  if (dwResult != ERROR_SUCCESS)
    throw ComException("CKsMidiCapFilter::CreateRenderPin: Pin instanciation failed", HRESULT_FROM_WIN32(dwResult));
}

void CKsPin::WriteData(KSSTREAM_HEADER* pKSSTREAM_HEADER, OVERLAPPED* pOVERLAPPED)
{
  DWORD cbWritten;
  BOOL fRes = ::DeviceIoControl(
    m_handle,
    IOCTL_KS_WRITE_STREAM,
    NULL,
    0,
    pKSSTREAM_HEADER,
    pKSSTREAM_HEADER->Size,
    &cbWritten,
    pOVERLAPPED);
  if (!fRes) {
    DWORD const dwError = GetLastError();
    if (dwError != ERROR_IO_PENDING)
      throw ComException("CKsPin::WriteData: DeviceIoControl failed", HRESULT_FROM_WIN32(dwError));
  }
}

void CKsPin::ReadData(KSSTREAM_HEADER* pKSSTREAM_HEADER, OVERLAPPED* pOVERLAPPED)
{
  DWORD cbReturned;
  BOOL fRes = ::DeviceIoControl(
    m_handle,
    IOCTL_KS_READ_STREAM,
    NULL,
    0,
    pKSSTREAM_HEADER,
    pKSSTREAM_HEADER->Size,
    &cbReturned,
    pOVERLAPPED);
  if (!fRes) {
    DWORD const dwError = GetLastError();
    if (dwError != ERROR_IO_PENDING)
      throw ComException("CKsPin::ReadData: DeviceIoControl failed", HRESULT_FROM_WIN32(dwError));
  }
}

class CKsMidiFilter : public CKsFilter
{
public:
  void FindMidiPins();

protected:
  CKsMidiFilter(tstring const& sPath, std::string const& sFriendlyName);
};

class CKsMidiPin : public CKsPin
{
public:
  CKsMidiPin(CKsFilter* pFilter, ULONG nId);
};

class CKsMidiRenFilter : public CKsMidiFilter
{
public:
  CKsMidiRenFilter(tstring const& sPath, std::string const& sFriendlyName);
  CKsMidiPin* CreateRenderPin();

  void Validate()
  {
    if (m_RenderPins.empty())
      throw std::runtime_error("Could not find a MIDI render pin");
  }
};

class CKsMidiCapFilter : public CKsMidiFilter
{
public:
  CKsMidiCapFilter(tstring const& sPath, std::string const& sFriendlyName);
  CKsMidiPin* CreateCapturePin();

  void Validate()
  {
    if (m_CapturePins.empty())
      throw std::runtime_error("Could not find a MIDI capture pin");
  }
};

CKsMidiFilter::CKsMidiFilter(tstring const& sPath, std::string const& sFriendlyName) :
  CKsFilter(sPath, sFriendlyName)
{
}

void CKsMidiFilter::FindMidiPins()
{
  ULONG numPins = GetPinProperty<ULONG>(0, KSPROPERTY_PIN_CTYPES);

  for (ULONG iPin = 0; iPin < numPins; ++iPin) {
    try {
      KSPIN_COMMUNICATION com = GetPinProperty<KSPIN_COMMUNICATION>(iPin, KSPROPERTY_PIN_COMMUNICATION);
      if (com != KSPIN_COMMUNICATION_SINK && com != KSPIN_COMMUNICATION_BOTH)
        throw std::runtime_error("Unknown pin communication value");

      m_Pins.push_back(new CKsMidiPin(this, iPin));
    }
    catch (std::runtime_error const&) {
      // pin instanciation has failed, continue to the next pin.
    }
  }

  m_RenderPins.clear();
  m_CapturePins.clear();

  for (size_t i = 0; i < m_Pins.size(); ++i) {
    CKsPin* const pPin = m_Pins[i];

    if (pPin->IsSink()) {
      if (pPin->GetDataFlow() == KSPIN_DATAFLOW_IN)
        m_RenderPins.push_back(pPin);
      else
        m_CapturePins.push_back(pPin);
    }
  }

  if (m_RenderPins.empty() && m_CapturePins.empty())
    throw std::runtime_error("No valid pins found on the filter.");
}

CKsMidiRenFilter::CKsMidiRenFilter(tstring const& sPath, std::string const& sFriendlyName) :
  CKsMidiFilter(sPath, sFriendlyName)
{
}

CKsMidiPin* CKsMidiRenFilter::CreateRenderPin()
{
  if (m_RenderPins.empty())
    throw std::runtime_error("Could not find a MIDI render pin");

  CKsMidiPin* pPin = (CKsMidiPin*)m_RenderPins[0];
  pPin->Instantiate();
  return pPin;
}

CKsMidiCapFilter::CKsMidiCapFilter(tstring const& sPath, std::string const& sFriendlyName) :
  CKsMidiFilter(sPath, sFriendlyName)
{
}

CKsMidiPin* CKsMidiCapFilter::CreateCapturePin()
{
  if (m_CapturePins.empty())
    throw std::runtime_error("Could not find a MIDI capture pin");

  CKsMidiPin* pPin = (CKsMidiPin*)m_CapturePins[0];
  pPin->Instantiate();
  return pPin;
}

CKsMidiPin::CKsMidiPin(CKsFilter* pFilter, ULONG nId) :
  CKsPin(pFilter, nId)
{
  DWORD const cbPinCreateSize = sizeof(KSPIN_CONNECT) + sizeof(KSDATAFORMAT);
  m_pKsPinConnect = (PKSPIN_CONNECT) new BYTE[cbPinCreateSize];

  m_pKsPinConnect->Interface.Set = KSINTERFACESETID_Standard;
  m_pKsPinConnect->Interface.Id = KSINTERFACE_STANDARD_STREAMING;
  m_pKsPinConnect->Interface.Flags = 0;
  m_pKsPinConnect->Medium.Set = KSMEDIUMSETID_Standard;
  m_pKsPinConnect->Medium.Id = KSMEDIUM_TYPE_ANYINSTANCE;
  m_pKsPinConnect->Medium.Flags = 0;
  m_pKsPinConnect->PinId = nId;
  m_pKsPinConnect->PinToHandle = NULL;
  m_pKsPinConnect->Priority.PriorityClass = KSPRIORITY_NORMAL;
  m_pKsPinConnect->Priority.PrioritySubClass = 1;

  // point m_pDataFormat to just after the pConnect struct
  KSDATAFORMAT* m_pDataFormat = (KSDATAFORMAT*)(m_pKsPinConnect + 1);
  m_pDataFormat->FormatSize = sizeof(KSDATAFORMAT);
  m_pDataFormat->Flags = 0;
  m_pDataFormat->SampleSize = 0;
  m_pDataFormat->Reserved = 0;
  m_pDataFormat->MajorFormat = GUID(KSDATAFORMAT_TYPE_MUSIC);
  m_pDataFormat->SubFormat = GUID(KSDATAFORMAT_SUBTYPE_MIDI);
  m_pDataFormat->Specifier = GUID(KSDATAFORMAT_SPECIFIER_NONE);

  bool hasStdStreamingInterface = false;
  bool hasStdStreamingMedium = false;

  for ( ULONG i = 0; i < m_cInterfaces; i++ ) {
    if (m_pInterfaces[i].Set == KSINTERFACESETID_Standard
        && m_pInterfaces[i].Id == KSINTERFACE_STANDARD_STREAMING)
      hasStdStreamingInterface = true;
  }

  for (ULONG i = 0; i < m_cMediums; i++) {
    if (m_pMediums[i].Set == KSMEDIUMSETID_Standard
        && m_pMediums[i].Id == KSMEDIUM_STANDARD_DEVIO)
      hasStdStreamingMedium = true;
  }

  if (!hasStdStreamingInterface) // No standard streaming interfaces on the pin
    throw std::runtime_error("CKsMidiPin::CKsMidiPin: no standard streaming interface");

  if (!hasStdStreamingMedium) // No standard streaming mediums on the pin
    throw std::runtime_error("CKsMidiPin::CKsMidiPin: no standard streaming medium");

  bool hasMidiDataRange = false;

  BYTE const* pDataRangePtr = reinterpret_cast<BYTE const*>(m_pDataRanges);

  for (ULONG i = 0; i < m_cDataRanges; ++i) {
    KSDATARANGE const* pDataRange = reinterpret_cast<KSDATARANGE const*>(pDataRangePtr);

    if (pDataRange->SubFormat == KSDATAFORMAT_SUBTYPE_MIDI) {
      hasMidiDataRange = true;
      break;
    }

    pDataRangePtr += pDataRange->FormatSize;
  }

  if (!hasMidiDataRange) // No MIDI dataranges on the pin
    throw std::runtime_error("CKsMidiPin::CKsMidiPin: no MIDI datarange");
}


struct WindowsKsData
{
  WindowsKsData() : m_pPin(NULL), m_Buffer(1024), m_hInputThread(NULL)
  {
    memset(&overlapped, 0, sizeof(OVERLAPPED));
    m_hExitEvent = ::CreateEvent(NULL, FALSE, FALSE, NULL);
    overlapped.hEvent = ::CreateEvent(NULL, FALSE, FALSE, NULL);
    m_hInputThread = NULL;
  }

  ~WindowsKsData()
  {
    ::CloseHandle(overlapped.hEvent);
    ::CloseHandle(m_hExitEvent);
  }

  OVERLAPPED overlapped;
  CKsPin* m_pPin;
  std::vector<unsigned char> m_Buffer;
  std::auto_ptr<CKsEnumFilters<CKsMidiCapFilter> > m_pCaptureEnum;
  std::auto_ptr<CKsEnumFilters<CKsMidiRenFilter> > m_pRenderEnum;
  HANDLE m_hInputThread;
  HANDLE m_hExitEvent;
};

// *********************************************************************//
// API: WINDOWS Kernel Streaming
// Class Definitions: MidiInWinKS
// *********************************************************************//

DWORD WINAPI midiKsInputThread(VOID* pUser)
{
  MidiInApi::RtMidiInData* data = static_cast<MidiInApi::RtMidiInData*>(pUser);
  WindowsKsData* apiData = static_cast<WindowsKsData*>(data->apiData);

  HANDLE hEvents[] = { apiData->overlapped.hEvent, apiData->m_hExitEvent };

  while ( true ) {
    KSSTREAM_HEADER packet;
    memset(&packet, 0, sizeof packet);
    packet.Size = sizeof(KSSTREAM_HEADER);
    packet.PresentationTime.Time = 0;
    packet.PresentationTime.Numerator = 1;
    packet.PresentationTime.Denominator = 1;
    packet.Data = &apiData->m_Buffer[0];
    packet.DataUsed = 0;
    packet.FrameExtent = apiData->m_Buffer.size();
    apiData->m_pPin->ReadData(&packet, &apiData->overlapped);

    DWORD dwRet = ::WaitForMultipleObjects(2, hEvents, FALSE, INFINITE);

    if ( dwRet == WAIT_OBJECT_0 ) {
      // parse packet
      unsigned char* pData = (unsigned char*)packet.Data;
      unsigned int iOffset = 0;

      while ( iOffset < packet.DataUsed ) {
        KSMUSICFORMAT* pMusic = (KSMUSICFORMAT*)&pData[iOffset];
        iOffset += sizeof(KSMUSICFORMAT);

        MidiInApi::MidiMessage message;
        message.timeStamp = 0;
        for(size_t i=0;i < pMusic->ByteCount;++i)
          message.bytes.push_back(pData[iOffset+i]);

        if ( data->usingCallback ) {
          RtMidiIn::RtMidiCallback callback = (RtMidiIn::RtMidiCallback)data->userCallback;
          callback(message.timeStamp, &message.bytes, data->userData);
        }
        else {
          // As long as we haven't reached our queue size limit, push the message.
          if ( data->queue.size < data->queue.ringSize ) {
            data->queue.ring[data->queue.back++] = message;
            if(data->queue.back == data->queue.ringSize)
              data->queue.back = 0;
            data->queue.size++;
          }
          else
            std::cerr << "\nRtMidiIn: message queue limit reached!!\n\n";
        }

        iOffset += pMusic->ByteCount;

        // re-align on 32 bits
        if ( iOffset % 4 != 0 )
          iOffset += (4 - iOffset % 4);
      }
    }
    else
      break;
  }
  return 0;
}

MidiInWinKS :: MidiInWinKS( const std::string clientName, unsigned int queueSizeLimit ) : MidiInApi( queueSizeLimit )
{
  initialize( clientName );
}

void MidiInWinKS :: initialize( const std::string& clientName )
{
  WindowsKsData* data = new WindowsKsData;
  apiData_ = (void*)data;
  inputData_.apiData = data;

  GUID const aguidEnumCats[] =
  {
    { STATIC_KSCATEGORY_AUDIO }, { STATIC_KSCATEGORY_CAPTURE }
  };
  data->m_pCaptureEnum.reset(new CKsEnumFilters<CKsMidiCapFilter> );
  data->m_pCaptureEnum->EnumFilters(aguidEnumCats, 2);
}

MidiInWinKS :: ~MidiInWinKS()
{
  WindowsKsData* data = static_cast<WindowsKsData*>(apiData_);
  try {
    if ( data->m_pPin )
      closePort();
  }
  catch(...) {
  }

  delete data;
}

void MidiInWinKS :: openPort( unsigned int portNumber, const std::string portName )
{
  WindowsKsData* data = static_cast<WindowsKsData*>(apiData_);

  if ( portNumber < 0 || portNumber >= data->m_pCaptureEnum->m_Filters.size() ) {
    std::stringstream ost;
    ost << "MidiInWinKS::openPort: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::WARNING, errorString_ );
  }

  CKsMidiCapFilter* pFilter = data->m_pCaptureEnum->m_Filters[portNumber];
  data->m_pPin = pFilter->CreateCapturePin();

  if ( data->m_pPin == NULL ) {
    std::stringstream ost;
    ost << "MidiInWinKS::openPort: KS error opening port (could not create pin)";
    errorString_ = ost.str();
    RtMidi::error( RtError::WARNING, errorString_ );
  }

  data->m_pPin->SetState(KSSTATE_RUN);

  DWORD threadId;
  data->m_hInputThread = ::CreateThread(NULL, 0, &midiKsInputThread, &inputData_, 0, &threadId);
  if ( data->m_hInputThread == NULL ) {
    std::stringstream ost;
    ost << "MidiInWinKS::initialize: Could not create input thread : Windows error " << GetLastError() << std::endl;;
    errorString_ = ost.str();
    RtMidi::error( RtError::WARNING, errorString_ );
  }

  connected_ = true;
}

void MidiInWinKS :: openVirtualPort( const std::string portName )
{
  // This function cannot be implemented for the Windows KS MIDI API.
  errorString_ = "MidiInWinKS::openVirtualPort: cannot be implemented in Windows KS MIDI API!";
  RtMidi::error( RtError::WARNING, errorString_ );
}

unsigned int MidiInWinKS :: getPortCount()
{
  WindowsKsData* data = static_cast<WindowsKsData*>(apiData_);
  return (unsigned int)data->m_pCaptureEnum->m_Filters.size();
}

std::string MidiInWinKS :: getPortName(unsigned int portNumber)
{
  WindowsKsData* data = static_cast<WindowsKsData*>(apiData_);

  if(portNumber < 0 || portNumber >= data->m_pCaptureEnum->m_Filters.size()) {
    std::stringstream ost;
    ost << "MidiInWinKS::getPortName: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::WARNING, errorString_ );
  }

  CKsMidiCapFilter* pFilter = data->m_pCaptureEnum->m_Filters[portNumber];
  return pFilter->GetFriendlyName();
}

void MidiInWinKS :: closePort()
{
  WindowsKsData* data = static_cast<WindowsKsData*>(apiData_);
  connected_ = false;

  if(data->m_hInputThread) {
    ::SignalObjectAndWait(data->m_hExitEvent, data->m_hInputThread, INFINITE, FALSE);
    ::CloseHandle(data->m_hInputThread);
  }

  if(data->m_pPin) {
    data->m_pPin->SetState(KSSTATE_PAUSE);
    data->m_pPin->SetState(KSSTATE_STOP);
    data->m_pPin->ClosePin();
    data->m_pPin = NULL;
  }
}

// *********************************************************************//
// API: WINDOWS Kernel Streaming
// Class Definitions: MidiOutWinKS
// *********************************************************************//

MidiOutWinKS :: MidiOutWinKS( const std::string clientName ) : MidiOutApi()
{
  initialize( clientName );
}

void MidiOutWinKS :: initialize( const std::string& clientName )
{
  WindowsKsData* data = new WindowsKsData;

  data->m_pPin = NULL;
  data->m_pRenderEnum.reset(new CKsEnumFilters<CKsMidiRenFilter> );
  GUID const aguidEnumCats[] =
  {
    { STATIC_KSCATEGORY_AUDIO }, { STATIC_KSCATEGORY_RENDER }
  };
  data->m_pRenderEnum->EnumFilters(aguidEnumCats, 2);

  apiData_ = (void*)data;
}

MidiOutWinKS :: ~MidiOutWinKS()
{
  // Close a connection if it exists.
  closePort();

  // Cleanup.
  WindowsKsData* data = static_cast<WindowsKsData*>(apiData_);
  delete data;
}

void MidiOutWinKS :: openPort( unsigned int portNumber, const std::string portName )
{
  WindowsKsData* data = static_cast<WindowsKsData*>(apiData_);

  if(portNumber < 0 || portNumber >= data->m_pRenderEnum->m_Filters.size()) {
    std::stringstream ost;
    ost << "MidiOutWinKS::openPort: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::WARNING, errorString_ );
  }

  CKsMidiRenFilter* pFilter = data->m_pRenderEnum->m_Filters[portNumber];
  data->m_pPin = pFilter->CreateRenderPin();

  if(data->m_pPin == NULL) {
    std::stringstream ost;
    ost << "MidiOutWinKS::openPort: KS error opening port (could not create pin)";
    errorString_ = ost.str();
    RtMidi::error( RtError::WARNING, errorString_ );
  }

  data->m_pPin->SetState(KSSTATE_RUN);
  connected_ = true;
}

void MidiOutWinKS :: openVirtualPort( const std::string portName )
{
  // This function cannot be implemented for the Windows KS MIDI API.
  errorString_ = "MidiOutWinKS::openVirtualPort: cannot be implemented in Windows KS MIDI API!";
  RtMidi::error( RtError::WARNING, errorString_ );
}

unsigned int MidiOutWinKS :: getPortCount()
{
  WindowsKsData* data = static_cast<WindowsKsData*>(apiData_);

  return (unsigned int)data->m_pRenderEnum->m_Filters.size();
}

std::string MidiOutWinKS :: getPortName( unsigned int portNumber )
{
  WindowsKsData* data = static_cast<WindowsKsData*>(apiData_);

  if ( portNumber < 0 || portNumber >= data->m_pRenderEnum->m_Filters.size() ) {
    std::stringstream ost;
    ost << "MidiOutWinKS::getPortName: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::WARNING, errorString_ );
  }

  CKsMidiRenFilter* pFilter = data->m_pRenderEnum->m_Filters[portNumber];
  return pFilter->GetFriendlyName();
}

void MidiOutWinKS :: closePort()
{
  WindowsKsData* data = static_cast<WindowsKsData*>(apiData_);
  connected_ = false;

  if ( data->m_pPin ) {
    data->m_pPin->SetState(KSSTATE_PAUSE);
    data->m_pPin->SetState(KSSTATE_STOP);
    data->m_pPin->ClosePin();
    data->m_pPin = NULL;
  }
}

void MidiOutWinKS :: sendMessage(std::vector<unsigned char>* pMessage)
{
  std::vector<unsigned char> const& msg = *pMessage;
  WindowsKsData* data = static_cast<WindowsKsData*>(apiData_);
  size_t iNumMidiBytes = msg.size();
  size_t pos = 0;

  // write header
  KSMUSICFORMAT* pKsMusicFormat = reinterpret_cast<KSMUSICFORMAT*>(&data->m_Buffer[pos]);
  pKsMusicFormat->TimeDeltaMs = 0;
  pKsMusicFormat->ByteCount = iNumMidiBytes;
  pos += sizeof(KSMUSICFORMAT);

  // write MIDI bytes
  if ( pos + iNumMidiBytes > data->m_Buffer.size() ) {
    std::stringstream ost;
    ost << "KsMidiInput::Write: MIDI buffer too small. Required " << pos + iNumMidiBytes << " bytes, only has " << data->m_Buffer.size();
    errorString_ = ost.str();
    RtMidi::error( RtError::WARNING, errorString_ );
  }

  if ( data->m_pPin == NULL ) {
    std::stringstream ost;
    ost << "MidiOutWinKS::sendMessage: port is not open";
    errorString_ = ost.str();
    RtMidi::error( RtError::WARNING, errorString_ );
  }

  memcpy(&data->m_Buffer[pos], &msg[0], iNumMidiBytes);
  pos += iNumMidiBytes;

  KSSTREAM_HEADER packet;
  memset(&packet, 0, sizeof packet);
  packet.Size = sizeof(packet);
  packet.PresentationTime.Time = 0;
  packet.PresentationTime.Numerator = 1;
  packet.PresentationTime.Denominator = 1;
  packet.Data = const_cast<unsigned char*>(&data->m_Buffer[0]);
  packet.DataUsed = ((pos+3)/4)*4;
  packet.FrameExtent = data->m_Buffer.size();

  data->m_pPin->WriteData(&packet, NULL);
}

#endif  // __WINDOWS_KS__

//*********************************************************************//
//  API: UNIX JACK
//
//  Written primarily by Alexander Svetalkin, with updates for delta
//  time by Gary Scavone, April 2011.
//
//  *********************************************************************//

#if defined(__UNIX_JACK__)

// JACK header files
#include <jack/jack.h>
#include <jack/midiport.h>
#include <jack/ringbuffer.h>

#define JACK_RINGBUFFER_SIZE 16384 // Default size for ringbuffer

struct JackMidiData {
  jack_client_t *client;
  jack_port_t *port;
  jack_ringbuffer_t *buffSize;
  jack_ringbuffer_t *buffMessage;
  jack_time_t lastTime;
  MidiInApi :: RtMidiInData *rtMidiIn;
  };

//*********************************************************************//
//  API: JACK
//  Class Definitions: MidiInJack
//*********************************************************************//

int jackProcessIn( jack_nframes_t nframes, void *arg )
{
  JackMidiData *jData = (JackMidiData *) arg;
  MidiInApi :: RtMidiInData *rtData = jData->rtMidiIn;
  jack_midi_event_t event;
  jack_time_t long long time;

  // Is port created?
  if ( jData->port == NULL ) return 0;
  void *buff = jack_port_get_buffer( jData->port, nframes );

  // We have midi events in buffer
  int evCount = jack_midi_get_event_count( buff );
  if ( evCount > 0 ) {
    MidiInApi::MidiMessage message;
    message.bytes.clear();

    jack_midi_event_get( &event, buff, 0 );

    for (unsigned int i = 0; i < event.size; i++ )
      message.bytes.push_back( event.buffer[i] );

    // Compute the delta time.
    time = jack_get_time();
    if ( rtData->firstMessage == true )
      rtData->firstMessage = false;
    else
      message.timeStamp = ( time - jData->lastTime ) * 0.000001;

    jData->lastTime = time;

    if ( !rtData->continueSysex ) {
      if ( rtData->usingCallback ) {
        RtMidiIn::RtMidiCallback callback = (RtMidiIn::RtMidiCallback) rtData->userCallback;
        callback( message.timeStamp, &message.bytes, rtData->userData );
      }
      else {
        // As long as we haven't reached our queue size limit, push the message.
        if ( rtData->queue.size < rtData->queue.ringSize ) {
          rtData->queue.ring[rtData->queue.back++] = message;
          if ( rtData->queue.back == rtData->queue.ringSize )
            rtData->queue.back = 0;
          rtData->queue.size++;
        }
        else
          std::cerr << "\nMidiInJack: message queue limit reached!!\n\n";
      }
    }
  }

  return 0;
}

MidiInJack :: MidiInJack( const std::string clientName, unsigned int queueSizeLimit ) : MidiInApi( queueSizeLimit )
{
  initialize( clientName );
}

void MidiInJack :: initialize( const std::string& clientName )
{
  JackMidiData *data = new JackMidiData;
  apiData_ = (void *) data;

  // Initialize JACK client
  if (( data->client = jack_client_open( clientName.c_str(), JackNullOption, NULL )) == 0) {
    errorString_ = "MidiInJack::initialize: JACK server not running?";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    return;
  }

  data->rtMidiIn = &inputData_;
  data->port = NULL;

  jack_set_process_callback( data->client, jackProcessIn, data );
  jack_activate( data->client );
}

MidiInJack :: ~MidiInJack()
{
  JackMidiData *data = static_cast<JackMidiData *> (apiData_);
  closePort();

  jack_client_close( data->client );
}

void MidiInJack :: openPort( unsigned int portNumber, const std::string portName )
{
  JackMidiData *data = static_cast<JackMidiData *> (apiData_);

  // Creating new port
  if ( data->port == NULL)
    data->port = jack_port_register( data->client, portName.c_str(),
                                     JACK_DEFAULT_MIDI_TYPE, JackPortIsInput, 0 );

  if ( data->port == NULL) {
    errorString_ = "MidiInJack::openVirtualPort: JACK error creating virtual port";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Connecting to the output
  std::string name = getPortName( portNumber );
  jack_connect( data->client, name.c_str(), jack_port_name( data->port ) );
}

void MidiInJack :: openVirtualPort( const std::string portName )
{
  JackMidiData *data = static_cast<JackMidiData *> (apiData_);

  if ( data->port == NULL )
    data->port = jack_port_register( data->client, portName.c_str(),
                                     JACK_DEFAULT_MIDI_TYPE, JackPortIsInput, 0 );

  if ( data->port == NULL ) {
    errorString_ = "MidiInJack::openVirtualPort: JACK error creating virtual port";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }
}

unsigned int MidiInJack :: getPortCount()
{
  int count = 0;
  JackMidiData *data = static_cast<JackMidiData *> (apiData_);

  // List of available ports
  const char **ports = jack_get_ports( data->client, NULL, JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput );

  if ( ports == NULL ) return 0;
  while ( ports[count] != NULL )
    count++;

  free( ports );

  return count;
}

std::string MidiInJack :: getPortName( unsigned int portNumber )
{
  JackMidiData *data = static_cast<JackMidiData *> (apiData_);
  std::ostringstream ost;
  std::string retStr("");

  // List of available ports
  const char **ports = jack_get_ports( data->client, NULL,
                                       JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput );

  // Check port validity
  if ( ports == NULL ) {
    errorString_ = "MidiInJack::getPortName: no ports available!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return retStr;
  }

  if ( ports[portNumber] == NULL ) {
    ost << "MidiInJack::getPortName: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::WARNING, errorString_ );
  }
  else retStr.assign( ports[portNumber] );

  free( ports );

  return retStr;
}

void MidiInJack :: closePort()
{
  JackMidiData *data = static_cast<JackMidiData *> (apiData_);

  if ( data->port == NULL ) return;
  jack_port_unregister( data->client, data->port );
  data->port = NULL;
}

//*********************************************************************//
//  API: JACK
//  Class Definitions: MidiOutJack
//*********************************************************************//

// Jack process callback
int jackProcessOut( jack_nframes_t nframes, void *arg )
{
  JackMidiData *data = (JackMidiData *) arg;
  jack_midi_data_t *midiData;
  int space;

  // Is port created?
  if ( data->port == NULL ) return 0;

  void *buff = jack_port_get_buffer( data->port, nframes );
  jack_midi_clear_buffer( buff );

  while ( jack_ringbuffer_read_space( data->buffSize ) > 0 ) {
    jack_ringbuffer_read( data->buffSize, (char *) &space, (size_t) sizeof(space) );
    midiData = jack_midi_event_reserve( buff, 0, space );

    jack_ringbuffer_read( data->buffMessage, (char *) midiData, (size_t) space );
  }

  return 0;
}

MidiOutJack :: MidiOutJack( const std::string clientName ) : MidiOutApi()
{
  initialize( clientName );
}

void MidiOutJack :: initialize( const std::string& clientName )
{
  JackMidiData *data = new JackMidiData;

  data->port = NULL;

  // Initialize JACK client
  if (( data->client = jack_client_open( clientName.c_str(), JackNullOption, NULL )) == 0) {
    errorString_ = "MidiOutJack::initialize: JACK server not running?";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
    return;
  }

  jack_set_process_callback( data->client, jackProcessOut, data );
  data->buffSize = jack_ringbuffer_create( JACK_RINGBUFFER_SIZE );
  data->buffMessage = jack_ringbuffer_create( JACK_RINGBUFFER_SIZE );
  jack_activate( data->client );

  apiData_ = (void *) data;
}

MidiOutJack :: ~MidiOutJack()
{
  JackMidiData *data = static_cast<JackMidiData *> (apiData_);
  closePort();

  // Cleanup
  jack_client_close( data->client );
  jack_ringbuffer_free( data->buffSize );
  jack_ringbuffer_free( data->buffMessage );

  delete data;
}

void MidiOutJack :: openPort( unsigned int portNumber, const std::string portName )
{
  JackMidiData *data = static_cast<JackMidiData *> (apiData_);

  // Creating new port
  if ( data->port == NULL )
    data->port = jack_port_register( data->client, portName.c_str(),
      JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0 );

  if ( data->port == NULL ) {
    errorString_ = "MidiOutJack::openVirtualPort: JACK error creating virtual port";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }

  // Connecting to the output
  std::string name = getPortName( portNumber );
  jack_connect( data->client, jack_port_name( data->port ), name.c_str() );
}

void MidiOutJack :: openVirtualPort( const std::string portName )
{
  JackMidiData *data = static_cast<JackMidiData *> (apiData_);

  if ( data->port == NULL )
    data->port = jack_port_register( data->client, portName.c_str(),
      JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0 );

  if ( data->port == NULL ) {
    errorString_ = "MidiOutJack::openVirtualPort: JACK error creating virtual port";
    RtMidi::error( RtError::DRIVER_ERROR, errorString_ );
  }
}

unsigned int MidiOutJack :: getPortCount()
{
  int count = 0;
  JackMidiData *data = static_cast<JackMidiData *> (apiData_);

  // List of available ports
  const char **ports = jack_get_ports( data->client, NULL,
    JACK_DEFAULT_MIDI_TYPE, JackPortIsInput );

  if ( ports == NULL ) return 0;
  while ( ports[count] != NULL )
    count++;

  free( ports );

  return count;
}

std::string MidiOutJack :: getPortName( unsigned int portNumber )
{
  JackMidiData *data = static_cast<JackMidiData *> (apiData_);
  std::ostringstream ost;
  std::string retStr("");

  // List of available ports
  const char **ports = jack_get_ports( data->client, NULL,
    JACK_DEFAULT_MIDI_TYPE, JackPortIsInput );

  // Check port validity
  if ( ports == NULL) {
    errorString_ = "MidiOutJack::getPortName: no ports available!";
    RtMidi::error( RtError::WARNING, errorString_ );
    return retStr;
  }

  if ( ports[portNumber] == NULL) {
    ost << "MidiOutJack::getPortName: the 'portNumber' argument (" << portNumber << ") is invalid.";
    errorString_ = ost.str();
    RtMidi::error( RtError::WARNING, errorString_ );
  }
  else retStr.assign( ports[portNumber] );

  free( ports );

  return retStr;
}

void MidiOutJack :: closePort()
{
  JackMidiData *data = static_cast<JackMidiData *> (apiData_);

  if ( data->port == NULL ) return;
  jack_port_unregister( data->client, data->port );
  data->port = NULL;
}

void MidiOutJack :: sendMessage( std::vector<unsigned char> *message )
{
  int nBytes = message->size();
  JackMidiData *data = static_cast<JackMidiData *> (apiData_);

  // Write full message to buffer
  jack_ringbuffer_write( data->buffMessage, ( const char * ) &( *message )[0],
                         message->size() );
  jack_ringbuffer_write( data->buffSize, ( char * ) &nBytes, sizeof( nBytes ) );
}

#endif  // __UNIX_JACK__
