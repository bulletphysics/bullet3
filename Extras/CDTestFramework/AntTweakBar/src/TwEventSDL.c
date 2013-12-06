//  ---------------------------------------------------------------------------
//
//  @file       TwEventSDL.c
//  @brief      Helper: 
//              translate and re-send mouse and keyboard events 
//              from SDL event loop to AntTweakBar
//  
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  note:       AntTweakBar.dll does not need to link with SDL, 
//              it just needs some definitions for its helper functions.
//
//  ---------------------------------------------------------------------------

#include <AntTweakBar.h>

int TW_CALL TwEventSDL12(const void *sdlEvent); // implemented in TwEventSDL12.c
int TW_CALL TwEventSDL13(const void *sdlEvent); // implmeneted in TwEventSDL13.c
#ifdef  __cplusplus
    extern "C" { int TW_CALL TwSetLastError(const char *staticErrorMessage); }
#else
    int TW_CALL TwSetLastError(const char *staticErrorMessage);
#endif  // __cplusplus


//  TwEventSDL returns zero if msg has not been handled or the SDL version 
//  is not supported, and a non-zero value if it has been handled by the 
//  AntTweakBar library.
int TW_CALL TwEventSDL(const void *sdlEvent, unsigned char majorVersion, unsigned char minorVersion)
{
    if (majorVersion < 1 || (majorVersion == 1 && minorVersion < 2))
    {
        static const char *g_ErrBadSDLVersion = "Unsupported SDL version";
        TwSetLastError(g_ErrBadSDLVersion);
        return 0;
    }
    else if (majorVersion == 1 && minorVersion == 2)
        return TwEventSDL12(sdlEvent);
    else // if( majorVersion==1 && minorVersion==3 ) 
        return TwEventSDL13(sdlEvent); // will probably not work for version > 1.3, but give it a chance
}
