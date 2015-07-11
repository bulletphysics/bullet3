#pragma once
#ifndef GWEN_SKINS_TEXTURING_H
#define GWEN_SKINS_TEXTURING_H

#include "Gwen/Gwen.h"
#include "Gwen/Texture.h"

namespace Gwen 
{
	namespace Skin
	{
		namespace Texturing
		{
			struct Single
			{
				Single()
				{
					texture = NULL;
				}

				void Init( Texture* pTexture, float x, float y, float w, float h )
				{
					texture = pTexture;

					float texw = texture->width;
					float texh = texture->height;

					uv[0] = x / texw;
					uv[1] = y / texh;
					uv[2] = (x+w) / texw;
					uv[3] = (y+h) / texh;
				}

				void Draw( Gwen::Renderer::Base* render, Gwen::Rect r, const Gwen::Color& col = Gwen::Colors::White )
				{
					render->SetDrawColor( col );	

					render->DrawTexturedRect( texture, r, uv[0], uv[1],uv[2], uv[3]  );
				}

				Texture*	texture;
				float uv[4];
			};

			struct Bordered
			{
				Bordered()
				{
					texture = NULL;
				}

				void Init( Texture* pTexture, float x, float y, float w, float h, Margin in_margin, float DrawMarginScale = 1.0f )
				{
					texture = pTexture;

					margin = in_margin;

					SetRect( 0, x, y, margin.left, margin.top );
					SetRect( 1, x+margin.left, y, w - margin.left - margin.right - 1, margin.top );
					SetRect( 2, (x + w) - margin.right, y, margin.right, margin.top );

					SetRect( 3, x, y+margin.top, margin.left, h - margin.top - margin.bottom - 1 );
					SetRect( 4, x+margin.left,  y+margin.top, w - margin.left - margin.right - 1, h - margin.top - margin.bottom - 1 );
					SetRect( 5, (x + w) - margin.right,  y+margin.top, margin.right, h - margin.top - margin.bottom - 1 );

					SetRect( 6, x, (y+h)-margin.bottom, margin.left, margin.bottom );
					SetRect( 7, x+margin.left, (y+h)-margin.bottom, w - margin.left - margin.right - 1, margin.bottom );
					SetRect( 8, (x + w) - margin.right, (y+h)-margin.bottom, margin.right, margin.bottom );

					margin.left *= DrawMarginScale;
					margin.right *= DrawMarginScale;
					margin.top *= DrawMarginScale;
					margin.bottom *= DrawMarginScale;

					width = w - x;
					height = h - y;
				}

				void SetRect( int iNum, float x, float y, float w, float h )
				{
					float texw = texture->width;
					float texh = texture->height;

					//x -= 1.0f;
					//y -= 1.0f;

					

					rects[iNum].uv[0] = x / texw;
					rects[iNum].uv[1] = y / texh;

					rects[iNum].uv[2] = (x+w) / texw;
					rects[iNum].uv[3] = (y+h) / texh;

				//	rects[iNum].uv[0] += 1.0f / texture->width;
				//	rects[iNum].uv[1] += 1.0f / texture->width;
				}

				void Draw( Gwen::Renderer::Base* render, Gwen::Rect r, const Gwen::Color& col = Gwen::Colors::White )
				{
					render->SetDrawColor( col );	

					if ( r.w < width && r.h < height )
					{
						render->DrawTexturedRect( texture, 
												r, 
												rects[0].uv[0], rects[0].uv[1], rects[8].uv[2], rects[8].uv[3]  );	
						return;
					}						
					 
					DrawRect( render, 0, r.x, r.y, margin.left, margin.top );
					DrawRect( render, 1, r.x + margin.left, r.y, r.w - margin.left - margin.right, margin.top );
					DrawRect( render, 2, (r.x + r.w) - margin.right, r.y, margin.right, margin.top );

					DrawRect( render, 3, r.x, r.y+margin.top, margin.left, r.h - margin.top - margin.bottom );
					DrawRect( render, 4, r.x + margin.left, r.y+margin.top, r.w - margin.left - margin.right, r.h - margin.top - margin.bottom );
					DrawRect( render, 5, (r.x + r.w) - margin.right, r.y+margin.top, margin.right, r.h - margin.top - margin.bottom );

					DrawRect( render, 6, r.x, (r.y+r.h) - margin.bottom, margin.left, margin.bottom );
					DrawRect( render, 7, r.x + margin.left, (r.y+r.h) - margin.bottom, r.w - margin.left - margin.right, margin.bottom );
					DrawRect( render, 8, (r.x + r.w) - margin.right, (r.y+r.h) - margin.bottom, margin.right, margin.bottom );
				}

				void DrawRect( Gwen::Renderer::Base* render, int i, int x, int y, int w, int h )
				{
					render->DrawTexturedRect( texture, 
												Gwen::Rect( x, y, w, h ), 
												rects[i].uv[0], rects[i].uv[1], rects[i].uv[2], rects[i].uv[3]  );
				}

				Texture*	texture;

				struct SubRect
				{
					float uv[4];
				};

				SubRect rects[9];
				Margin margin;

				float width;
				float height;
			};
		}
	}
}
#endif
