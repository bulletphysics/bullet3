//	---------------------------------------------------------------------------
//
//	@file		TwEventSLD.c
//	@brief		Helper: 
//				translate and re-send mouse and keyboard events 
//				from SDL event loop to AntTweakBar
//	
//	@author		Philippe Decaudin - http://www.antisphere.com
//	@date		2006/05/10
//  @license    This file is part of the AntTweakBar library.
//				Copyright © 2005, 2006 Philippe Decaudin.
//              For conditions of distribution and use, see License.txt
//
//	note:		TAB=4
//
//	---------------------------------------------------------------------------

#include <AntTweakBar.h>
#include <SDL/SDL.h>


//	TwEventSDL returns zero if msg has not been handled, 
//	and a non-zero value if it has been handled by the AntTweakBar library.
int TW_CALL TwEventSDL(const void *sdlEvent)
{
	int handled = 0;
	const SDL_Event *event = (const SDL_Event *)sdlEvent;

	if( event==NULL )
		return 0;

	switch( event->type )
	{
	case SDL_KEYDOWN:
		if ( event->key.keysym.unicode!=0 && (event->key.keysym.unicode & 0xFF00)==0 )
		{
			if( (event->key.keysym.unicode & 0xFF)<32 && (event->key.keysym.unicode & 0xFF)!=event->key.keysym.sym )
				handled = TwKeyPressed((event->key.keysym.unicode & 0xFF)+'a'-1, event->key.keysym.mod);
			else
				handled = TwKeyPressed(event->key.keysym.unicode & 0xFF, event->key.keysym.mod);
		}
		else
			handled = TwKeyPressed(event->key.keysym.sym, event->key.keysym.mod);
		break;
	case SDL_MOUSEMOTION:
		handled = TwMouseMotion(event->motion.x, event->motion.y);
		break;
	case SDL_MOUSEBUTTONUP:
	case SDL_MOUSEBUTTONDOWN:
		if( event->type==SDL_MOUSEBUTTONDOWN && (event->button.button==4 || event->button.button==5) )	// mouse wheel
		{
			static int s_WheelPos = 0;
			if( event->button.button==4 )
				++s_WheelPos;
			else
				--s_WheelPos;
			handled = TwMouseWheel(s_WheelPos);
		}
		else
			handled = TwMouseButton((event->type==SDL_MOUSEBUTTONUP)?TW_MOUSE_RELEASED:TW_MOUSE_PRESSED, (TwMouseButtonID)event->button.button);
		break;
	case SDL_VIDEORESIZE:
		// tell the new size to TweakBar
		TwWindowSize(event->resize.w, event->resize.h);
		// do not set 'handled', SDL_VIDEORESIZE may be also processed by the calling application
		break;
	}

	return handled;
}
