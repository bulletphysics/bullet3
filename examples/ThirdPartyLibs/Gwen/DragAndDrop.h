/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_DRAGANDDROP_H
#define GWEN_DRAGANDDROP_H

#include <sstream>

#include "Gwen/Skin.h"
#include "Gwen/Structures.h"

namespace Gwen
{
	namespace DragAndDrop
	{
		extern GWEN_EXPORT Package*	CurrentPackage;
		extern GWEN_EXPORT Gwen::Controls::Base*	SourceControl;
		extern GWEN_EXPORT Gwen::Controls::Base*	HoveredControl;

		bool GWEN_EXPORT Start( Gwen::Controls::Base* pControl, Package* pPackage );

		bool GWEN_EXPORT OnMouseButton( Gwen::Controls::Base* pHoveredControl, int x, int y, bool bDown );
		void GWEN_EXPORT OnMouseMoved( Gwen::Controls::Base* pHoveredControl, int x, int y );

		void GWEN_EXPORT RenderOverlay( Gwen::Controls::Canvas* pCanvas, Skin::Base* skin );

		void GWEN_EXPORT ControlDeleted( Gwen::Controls::Base* pControl );
	}

}
#endif
