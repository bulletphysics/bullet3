/*
  FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
  Copyright (C) 2009. Rama Hoetzlein, http://www.rchoetzlein.com

  ZLib license
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/



//#define MEMORY_DEBUG						// Overloads the new and delete operators

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "mdebug.h"

#ifdef _MSC_VER
	#include <windows.h>
	#include <io.h>
	#include <fcntl.h>	
	#include <conio.h>
#endif

CDebug debug;
CError error;

//------------------------------------------------- DEBUG CLASS
CDebug::CDebug ()
{
	m_OutName = "debug.txt";	
	m_bStarted = false;
	m_bToFile = true;
	m_bToSysbox = true;
	m_bToCons = false;	
	m_OutFile = NULL;	
	m_OutCons = NULL;
	Start ();
}

CDebug::~CDebug (void)
{	
	Stop ();	
}

void CDebug::Start ()
{
	if ( m_bStarted ) Stop ();
	
	if ( m_bToFile ) {
		char fn[200];

#ifdef _MSC_VER
		strcpy_s ( fn, 200, m_OutName.c_str () );
		fopen_s ( &m_OutFile, fn, "w+t" );
#else 
        strncpy ( fn, m_OutName.c_str(), 200 );
        m_OutFile = fopen( fn, "w+" );
#endif 

		if ( m_OutFile == 0x0 ) {
			exit ( EXIT_FAILURE );		// error: Cannot create debug file
		}
		m_bStarted = true;
		Print ( "debug", "CDebug started to file.\n");
	}		
	m_bStarted = true;	
}

void CDebug::Exit ( int code )
{
	if ( !m_bToSysbox ) {
		#ifdef _MSC_VER
			_getch();
		#endif
	}
	exit ( code );	
}

void CDebug::Stop ()
{
	if ( m_bStarted ) {
		if ( m_bToFile ) {
			Print ( "debug", "CDebug stopped.");
			fclose ( m_OutFile );			
		}
	}
	m_bStarted = false;
}

void CDebug::SendToFile ( char* fname )
{
	if (m_bStarted) Stop ();
	m_bToFile = true;
	m_OutName = fname;
}

void CDebug::SendToConsole ( bool tf )
{
	m_bToCons = tf;
	if ( tf ) {
		#ifdef _MSC_VER
			AllocConsole ();
			long lStdHandle = (long) GetStdHandle( STD_OUTPUT_HANDLE );
			int hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);
			m_OutCons = _fdopen( hConHandle, "w" );
		#endif
	}
}

void CDebug::SendToSysbox ( bool tf )
{
	m_bToSysbox = tf;
}

void CDebug::Print ( std::string subsys, std::string msg )
{
	if ( m_bStarted ) {
		if ( m_bToFile ) {
			fprintf ( m_OutFile, "%s: %s\n", subsys.c_str(), msg.c_str() );
			fflush ( m_OutFile);
		}
		if ( m_bToCons ) {
			#ifdef _MSC_VER
				fprintf ( m_OutCons, "%s: %s\n", subsys.c_str(), msg.c_str() );
				fflush ( m_OutCons );
			#else
				printf ( "%s: %s\n", subsys.c_str(), msg.c_str() );
			#endif
		}				
	}
}

void CDebug::Print (char* msg)
{
	if ( m_bStarted ) {
		if ( m_bToFile ) {
			fprintf ( m_OutFile, "%s", msg );
			fflush ( m_OutFile );
		}
		if ( m_bToCons ) {
			#ifdef _MSC_VER
				fprintf ( m_OutCons, "%s", msg );
				fflush ( m_OutCons );
			#else
				printf ( "%s", msg );
			#endif
		}				
	}	
}

void CDebug::Print ( std::string msg )
{
	if ( m_bStarted ) {
		if ( m_bToFile ) {
			fprintf ( m_OutFile, "%s", msg.c_str() );
			fflush ( m_OutFile );
		}
		if ( m_bToCons ) {
			#ifdef _MSC_VER
				fprintf ( m_OutCons, "%s", msg.c_str() );
				fflush ( m_OutCons );
			#else
				printf ( "%s", msg.c_str() );
			#endif
		}				
	}	
}

void CDebug::PrintF ( std::string substr, char* format, ... )
{
	// Note: This is the >only< way to do this. There is no general way to
	// pass on all the arguments from one ellipsis function to another.
	// The function vfprintf was specially designed to allow this.

	if ( m_bStarted ) {
		if ( m_bToFile ) {
			va_list argptr;
			va_start (argptr, format);
			fprintf ( m_OutFile, "%s: ", substr.c_str() );
 			vfprintf ( m_OutFile, format, argptr);
			va_end (argptr);
			fflush ( m_OutFile );
		}
		if ( m_bToCons ) {
			#ifdef _MSC_VER
				va_list argptr;
				va_start (argptr, format);
				fprintf ( m_OutCons, "%s: ", substr.c_str() );
 				vfprintf ( m_OutCons, format, argptr);			
				va_end (argptr);			
				fflush ( m_OutCons );
			#else
				va_list argptr;
				va_start (argptr, format);
				printf ( "%s: ", substr.c_str() );
 				vprintf ( format, argptr);			
				va_end (argptr);			
			#endif
		}	
	}
}


void CDebug::Printf ( char* format, ... )
{
	// Note: This is the >only< way to do this. There is no general way to
	// pass on all the arguments from one ellipsis function to another.
	// The function vfprintf was specially designed to allow this.

	if ( m_bStarted ) {
		if ( m_bToFile ) {
			va_list argptr;
			va_start (argptr, format);			
 			vfprintf ( m_OutFile, format, argptr);
			va_end (argptr);
			fflush ( m_OutFile );
		}
		if ( m_bToCons ) {
			#ifdef _MSC_VER
				va_list argptr;
				va_start (argptr, format);				
 				vfprintf ( m_OutCons, format, argptr);			
				va_end (argptr);			
				fflush ( m_OutCons );
			#else
				va_list argptr;
				va_start (argptr, format);				
 				vprintf ( format, argptr);			
				va_end (argptr);			
			#endif
		}	
	}
}

void CDebug::PrintErr ( std::string errid, std::string subsys, std::string msg, std::string sysbox )
{
	if ( m_bStarted ) {
		if ( m_bToFile ) {
			fprintf ( m_OutFile, "%s: ERROR: %s\n", subsys.c_str(), msg.c_str() );
			fflush ( m_OutFile );
		}
		if ( m_bToCons ) {
			#ifdef _MSC_VER
				fprintf ( m_OutCons, "%s: ERROR: %s\n", subsys.c_str(), msg.c_str() );
				fflush ( m_OutCons );
			#else
				printf ( "%s: ERROR[%s] %s\n", subsys.c_str(), errid.c_str(), msg.c_str() );
			#endif				
		}
		if ( m_bToSysbox ) {			
			char disp[4000];
			char caption[200];
			
			#ifdef _MSC_VER
				// Message boxes for Windows 
				strcpy_s ( caption, 200, "Mint - Error" );
				strcpy_s ( disp, 4000, sysbox.c_str()); 					
				#include <windows.h>
				int hr = MessageBoxA ( 0x0, disp, caption, MB_OK);			
			#else 
				strncpy ( caption, m_ErrorSubsys.c_str(), 200 );
				strncpy ( disp, msg.c_str(), 4000 ); 		
			#endif
		}
	}
}


//-------------------------------------------------------------------- Error Class Code 
// 
// This software is released under the LGPL Open Source Liscense. 
// See the documentation included with this source code for terms of modification, 
// distribution and re-release. 
//
// Original Copyright (C) 2002 Rama C. Hoetzlein, GameX R4
// 

CError::CError ()
{
	m_Errors.clear ();
	m_ErrorID = "";
	m_ErrorSubsys = "";
	m_ErrorMsg = "";
	m_ErrorFunc = "";
	m_ErrorFix = "";
	m_ErrorExtra = "";
}

void CError::Start ()
{
	Start ( "" );
}

void CError::Start ( char* fname )
{
	if ( fname != 0x0 && strlen(fname) > 0 ) {
		// Read error message file. NOT YET IMPLEMENTED
	} else {
		debug.Print ( "error", "No error file loaded." );
	}
	debug.Print ( "error", "Error handler started." );
	m_bStarted = true;
}

void CError::OutputMessage ()
{
	// Create sysbox message
	std::string box_msg;
	box_msg = "Subsystem: " + m_ErrorSubsys + "\n\n";
	box_msg += "Error: " + m_ErrorMsg + "\n";		
	if ( m_ErrorExtra.length() > 0)		box_msg += "Info: " + m_ErrorExtra + "\n";
	if ( m_ErrorFix.length() > 0)		box_msg += "\nFix: " + m_ErrorFix + "\n";	
	if ( m_ErrorID.length() > 0 )		box_msg += "Error ID: " + m_ErrorID + "\n";
	if ( m_ErrorFunc.length() > 0 )		box_msg += "Function: " + m_ErrorFunc + "\n";			
			
	// Error output to debug file 
	debug.PrintErr ( m_ErrorID, m_ErrorSubsys, m_ErrorMsg, box_msg );	
}

void CError::Exit ( int code )
{
	debug.Exit ( code );
}

void CError::Print ( char* msg)
{
	// User-level error (no additional info)
	m_ErrorID = "";
	m_ErrorSubsys = "undef";
	m_ErrorMsg = msg;
	m_ErrorFunc = "";
	m_ErrorFix = "";
	m_ErrorExtra = "";	
	OutputMessage ();
}

void CError::Print ( std::string msg )
{
	// User-level error (no additional info)
	m_ErrorID = "";
	m_ErrorSubsys = "undef";
	m_ErrorMsg = msg;
	m_ErrorFunc = "";
	m_ErrorFix = "";
	m_ErrorExtra = "";	
	OutputMessage ();
}

void CError::Print ( std::string subsys, std::string msg ){
	// Unregistered error
	m_ErrorID = "";
	m_ErrorSubsys = subsys;
	m_ErrorMsg = msg;
	m_ErrorFunc = "";
	m_ErrorFix = "";
	m_ErrorExtra = "";
	OutputMessage ();
}

void CError::PrintF ( std::string subsys, char* msg, ... )
{
	char buf[2000];
	va_list argptr;
	m_ErrorID = "";
	m_ErrorSubsys = subsys;
	va_start(argptr, msg);
    #ifdef _MSC_VER
 		vsprintf_s (buf, 2000, msg, argptr);
    #else
		vsnprintf(buf, 2000, msg, argptr);
    #endif
	va_end (argptr);	
	m_ErrorMsg = buf;
	m_ErrorFunc = "";
	m_ErrorFix = "";
	m_ErrorExtra = "";
	OutputMessage ();
}


void CError::PrintErr ( std::string err )
{
	// Registered error - NOT YET IMPLEMENTED
	// CErrorMsg* msg = m_
}

void CError::PrintErrDX ( std::string err, int result )
{
//
// #ifdef BUILD_DX
// m_ErrorExtra = DXGetErrorString9 ( result );
// #endif
	OutputMessage ();
	m_ErrorExtra = "";
}

void CError::PrintErrGL ( std::string err, int result )
{
	OutputMessage ();
	m_ErrorExtra = "";
}

void CError::PrintErrW ( std::string err,  int result )
{

	OutputMessage ();
	m_ErrorExtra = "";
}


/*
void CError::GetErrorMessage ( ErrorDef::Errors err )
{
	switch (err) {
	//----------------------------------------------------- MATRIX Errors
	case ErrorDef::MatrixIsNull:
		m_strDescription = "Matrix data is null\n";
		m_strFunction = "Matrix::op()";
		m_strFix = "";
		break;
	case ErrorDef::ColOutOfBounds:
		m_strDescription = "Column index out of bounds\n";
		m_strFunction = "Matrix::op()";
		m_strFix = "";
		break;
	case ErrorDef::RowOutOfBounds:
		m_strDescription = "Row index out of bounds\n";
		m_strFunction = "Matrix::op()";
		m_strFix = "";
		break;
	//----------------------------------------------------- WinDX Errors
	case ErrorDef::DXInitFail:		
		m_strDescription = "Initialization of DirectX failed.";
		m_strFunction = "GameX::InitDirect3D";
		m_strFix = "Reinstall DirectX 9.0. Confirm DirectX hardware support.";
		break;
	case ErrorDef::DXCannotGetMode:		
		m_strDescription = "Cannot get current display mode.";
		m_strFunction = "GameX::SysGetCurrentMode";
		m_strFix = "";
		break;
	case ErrorDef::DXCannotSetMode:		
		m_strDescription = "Cannot select display mode.";
		m_strFunction = "GameX::SysSelectMode";
		m_strFix = "Try requesting a different display mode and options.";
		break;
	case ErrorDef::WinCannotCreateClass:		
		m_strDescription = "Cannot create Windows class.";
		m_strFunction = "GameX::SysCreateWindowsClass";
		m_strFix = "Close other applications. Restart system.";
		break;
	case ErrorDef::WinCannotCreateWindow:		
		m_strDescription = "Cannot create Window.";
		m_strFunction = "GameX::SysCreateWindow";
		m_strFix = "Try requesting a different display mode and options.\nClose other applications. Restart system. Possibly unsupported display hardware.";
		break;
	case ErrorDef::DXCannotCreateDevice:	
		m_strDescription = "Cannot create DirectX Device.";
		m_strFunction = "GameX::SysCreateDevice3D";
		m_strFix = "Try requesting a different display mode and options.";
		break;
	case ErrorDef::DXCannotCreateDepthStencils:		
		m_strDescription = "Cannot create DirectX Depth Stencils.";
		m_strFunction = "GameX::SysCreateDepthStencils";
		m_strFix = "Try requesting a different display mode and options.";
		break;
	//---------------------------------------------------------------- File Errors
	case ErrorDef::CannotAppendAndRead:		
		m_strDescription = "Cannot open a file in both APPEND and READ mode.";
		m_strFunction = "File::Open";
		m_strFix = "Use either APPEND, READ or WRITE when opening file.";
		break;
	case ErrorDef::CannotAppendAndWrite:		
		m_strDescription = "Cannot open a file in both APPEND and WRITE mode.";
		m_strFunction = "File::Open";
		m_strFix = "Use either APPEND, READ or WRITE when opening file.";
		break;
	case ErrorDef::FileNotFound:		
		m_strDescription = "File not found.";
		m_strFunction = "File::Open";
		m_strFix = "Use AUTOCREATE if you would like to create a new file.";
		break;
	case ErrorDef::LeaveEofOnlyOnAppend:		
		m_strDescription = "Warning:LEAVEEOF flag can only be used in APPEND mode.";
		m_strFunction = "File::Open";
		m_strFix = "Remove the LEAVEEOF flag or open file in APPEND mode.";
		break;
	case ErrorDef::NoModeSpecified:		
		m_strDescription = "No open mode specified.";
		m_strFunction = "File::Open";
		m_strFix = "You must specify READ, WRITE or APPEND to open a file.";
		break;
	case ErrorDef::NoNotOpenCmd:		
		m_strDescription = "NOTOPEN cannot be used as an open file flag.";
		m_strFunction = "File::Open";
		m_strFix = "Remove the NOTOPEN flag.";
		break;
	case ErrorDef::NoSeqAndRandom:		
		m_strDescription = "Cannot open file in both SEQUENTIAL and RANDOM access modes.";
		m_strFunction = "File::Open";
		m_strFix = "Choose either SEQUENTIAL or RANDOM mode to open file in.";
		break;		
	case ErrorDef::CannotCreateFont:	
		m_strDescription = "Cannot create default font.";
		m_strFunction = "RenderDX::Initialize";
		m_strFix = "DirectX fonts are not supported for some reason.";
		break;		
	case ErrorDef::CannotCreateSprite:		
		m_strDescription = "Cannot create default sprite.";
		m_strFunction = "RenderDX::Initialize";
		m_strFix = "DirectX sprites are not supported for some reason.";
		break;		
	case ErrorDef::CannotAddImage:		
		m_strDescription = "Cannot create image on video memory.";
		m_strFunction = "RenderDX::AddImage";
		m_strFix = "DirectX was unable to create hardware texture for the image.";
		break;		
	case ErrorDef::CannotUpdateImage:
		m_strDescription = "Cannot update image data on video memory.";
		m_strFunction = "RenderDX::UpdateImage";
		m_strFix = "DirectX was unable to modify hardware texture for the image.";
		break;		
	case ErrorDef::CannotOpenFile:
		#if defined(BUILD_VSNET)
			m_strDescription = strerror( errno );
		#else	
			m_strDescription = "";		// should replace with Mac general error code
		#endif
		m_strFunction = "File::Open";
		m_strFix = "";
		break;
	case ErrorDef::ImageLoadError:
		m_strDescription = "Unable to load image.";
		m_strFunction = "ImageX::Load";
		m_strFix = "";
		break;
	default:
		char msg[500];
		sprintf (msg, "%d", (int) err );
		m_strDescription = "Undefined error: ";
		m_strDescription += msg;
		m_strSubsystem = "Undefined error.";
		m_strFix = "Error must be given a description and fix message.";
		break;
	};
}
*/



#ifdef MEMORY_DEBUG

	// User-defined operator new.
	void *operator new( size_t stSize ) 
	{
		void* pvMem = malloc( stSize );
		debug.Printf ( "NEW %p (%d bytes)\n", pvMem, stSize );
		return pvMem;
	}

	// User-defined operator delete.
	void operator delete( void *pvMem )
	{
		debug.Printf ( "DELETE %p\n", pvMem );
		free ( pvMem );	
	}
#endif