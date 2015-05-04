/*
	GWEN
	Copyright (c) 2011 Facepunch Studios
	See license in Gwen.h
*/

#ifndef GWEN_RENDERERS_OPENGL_DEBUGFONT_H
#define GWEN_RENDERERS_OPENGL_DEBUGFONT_H

#include "../ThirdPartyLibs/Gwen/Gwen.h"
#include "../ThirdPartyLibs/Gwen/Renderers/OpenGL.h"

void	restoreOpenGLState();
void	saveOpenGLState(int screenWidth, int screenHeight);


namespace Gwen 
{
	namespace Renderer 
	{

		class OpenGL_DebugFont : public Gwen::Renderer::Base
		{
			public:

					struct Vertex
				{
					float x, y, z;
					float u, v;
					unsigned char r, g, b, a;
				};

					
					static const int	MaxVerts = 1024;

				OpenGL_DebugFont();
				~OpenGL_DebugFont();

				void RenderText( Gwen::Font* pFont, Gwen::Point pos, const Gwen::UnicodeString& text );
				Gwen::Point MeasureText( Gwen::Font* pFont, const Gwen::UnicodeString& text );
				
				virtual void Begin();
				virtual void End();

				virtual void SetDrawColor( Gwen::Color color );
				virtual void DrawFilledRect( Gwen::Rect rect );
				void DrawTexturedRect( Gwen::Texture* pTexture, Gwen::Rect rect, float u1, float v1, float u2, float v2 );
		

				void StartClip();
				void EndClip();
			
				void Flush();
				void AddVert( int x, int y, float u = 0.0f , float v = 0.0f );

				virtual void Resize(int width, int height) {}


			protected:

				Gwen::Texture*	m_pFontTexture;
				float			m_fFontScale[2];
				float			m_fLetterSpacing;

			
				Gwen::Color			m_Color;
				int					m_iVertNum;
				Vertex				m_Vertices[ MaxVerts ];

		};

	}
}
#endif
