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

#ifndef INC_MINT_DEBUG_H
	#define INC_MINT_DEBUG_H

	#include <stdio.h>
	#include <stdlib.h>
	#include <string>
	#include <map>

	

	#ifdef BUILD_VS2005
	 //	#pragma warning( disable : 4101)		// removes warning 4101 - unreferenced local
	 //	#pragma warning( disable : 4244)		// removes warning 4244 - conversion loss of data
		#pragma warning( disable : 4995)		// removes warning 4995 - depricated function
		#pragma warning( disable : 4996)		// removes warning 4996 - depricated function
	 //	#pragma warning( disable : 4018)		// removes warning 4018 - unsigned/signed
	#endif

	class CDebug {
	public:
		CDebug ();
		~CDebug ();	

		// Control functions
		void Start ();
		void Stop ();	
		void SendToConsole ( bool tf );
		void SendToSysbox ( bool tf );
		void SendToFile ( bool tf );
		void SendToFile ( char *filename );
		
		// Output functions
		void Exit ( int code );
		void Print ( std::string subsys, std::string msg );
		void Print ( std::string msg );
		void Print ( char* msg);
		void PrintF ( std::string subsys, char *msg, ... );
		void Printf ( char *msg, ... );
		void PrintErr ( std::string errid, std::string subsys, std::string msg, std::string sysbox );	// Used by Error class

		// Filtering functions
		void AddFilter ( std::string subsys );
		void ClearFilters ();

	private:
		bool							m_bStarted;
		bool							m_bToFile;
		bool							m_bToCons;
		bool							m_bToSysbox;		
		std::string						m_OutName;
		FILE*							m_OutFile;
		FILE*							m_OutCons;
		
		std::map< std::string, bool >	m_Filters;
	};	

	class CErrorMsg {		
		std::string			m_strSubsys;		
		std::string			m_strFunction;
		std::string			m_strMessage;
		std::string			m_strFix;
		std::string			m_strExtraInfo;
		bool				m_bFatal;
		bool				m_bFilter;
	};

	class CError {
	public:
		CError ();

		void Start ();
		void Start ( char* fname );
		void LoadErrors ();	
		void OutputMessage ();
		void Exit ()	{ Exit (EXIT_SUCCESS); }
		void Exit ( int code );

		// Unregistered errors		
		void Print ( std::string subsys, std::string msg );				
		void Print ( std::string msg );
		void Print ( char* msg);
		void PrintF ( std::string subsys, char *msg, ... );				

		// Registered errors		
		void PrintErr ( std::string err );
		void PrintErrDX ( std::string err, int result );		// for DirectX errors
		void PrintErrGL ( std::string err, int result );		// for OpenGL errors
		void PrintErrW ( std::string err,  int result );		// for Windows errors		

		std::string GetErrorSubsys ()		{ return m_ErrorID; }
		std::string GetErrorMessage ()		{ return m_ErrorMsg; }
		std::string GetErrorFunction ()		{ return m_ErrorFunc; }
		std::string GetErrorFix ()			{ return m_ErrorFix; }
		std::string GetErrorExtra ()		{ return m_ErrorExtra; }
	
	private:	

		bool					m_bStarted;

		std::string				m_ErrorID;			// Current error
		std::string				m_ErrorSubsys;
		std::string				m_ErrorMsg;
		std::string				m_ErrorFunc;
		std::string				m_ErrorFix;
		std::string				m_ErrorExtra;		
		bool					m_bFatal;
		
		std::map < std::string, CErrorMsg* >	m_Errors;		// List of registered errors
	};	

	

	extern CError		error;
	extern CDebug		debug;
#endif

