/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#ifndef GWEN_RENDERERS_DIRECTX9_H
#define GWEN_RENDERERS_DIRECTX9_H
#include "Gwen/Gwen.h"
#include "Gwen/BaseRender.h"

struct VERTEXFORMAT2D
{
	FLOAT x, y, z, rhw;
	DWORD color;
	float u, v;
};

#define D3DFVF_VERTEXFORMAT2D ( D3DFVF_XYZRHW | D3DFVF_DIFFUSE | D3DFVF_TEX1 )

namespace Gwen 
{
	namespace Renderer 
	{

		class GWEN_EXPORT DirectX9 : public Gwen::Renderer::Base
		{
			public:

				DirectX9( IDirect3DDevice9* pDevice );
				~DirectX9();

				virtual void Begin();
				virtual void End();
				virtual void Release();

				virtual void SetDrawColor(Gwen::Color color);

				virtual void DrawLine( int x, int y, int a, int b );
				virtual void DrawFilledRect( Gwen::Rect rect );

				virtual void LoadFont( Gwen::Font* pFont );
				virtual void FreeFont( Gwen::Font* pFont );
				virtual void RenderText( Gwen::Font* pFont, Gwen::Point pos, const Gwen::UnicodeString& text );
				virtual Gwen::Point MeasureText( Gwen::Font* pFont, const Gwen::UnicodeString& text );

				void StartClip();
				void EndClip();

				void DrawTexturedRect( Gwen::Texture* pTexture, Gwen::Rect pTargetRect, float u1=0.0f, float v1=0.0f, float u2=1.0f, float v2=1.0f );
				void LoadTexture( Gwen::Texture* pTexture );
				void FreeTexture( Gwen::Texture* pTexture );

			protected:

				void*				m_pCurrentTexture;
				IDirect3DDevice9*	m_pDevice;
				DWORD				m_Color;

				void Flush();
				void AddVert( int x, int y );
				void AddVert( int x, int y, float u, float v );

				static const int	MaxVerts = 1024;
				VERTEXFORMAT2D  m_pVerts[MaxVerts];
				int				m_iVertNum;

				Gwen::Font::List		m_FontList;

		};

	}
}
#endif
