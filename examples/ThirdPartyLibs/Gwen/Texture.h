/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_TEXTURE_H
#define GWEN_TEXTURE_H

#include <string>

#include "Gwen/BaseRender.h"
#include "Gwen/TextObject.h"

namespace Gwen
{
	//
	// Texture
	//
	struct Texture
	{
		TextObject	name;
		void*	data;
		int m_intData;
		
		bool	failed;
		int		width;
		int		height;

		Texture()
		{
			data = NULL;
			m_intData = 0;
			width = 4;
			height = 4;
			failed = false;
		}

		~Texture()
		{
		}

		void Load( const TextObject& str, Gwen::Renderer::Base* render )
		{
			name = str;
			render->LoadTexture( this );
		}

		void Release( Gwen::Renderer::Base* render )
		{
			render->FreeTexture( this );
		}
	};

}
#endif
