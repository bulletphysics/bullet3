
#ifndef __GWEN_OPENGL3_CORE_RENDERER_H
#define __GWEN_OPENGL3_CORE_RENDERER_H

#include "Gwen/Gwen.h"
#include "Gwen/BaseRender.h"
#include "GLPrimitiveRenderer.h"
#include "../OpenGLWindow/OpenGLInclude.h"

struct sth_stash;
#include "fontstash.h"
#include "Gwen/Texture.h"

#include "TwFonts.h"
static float extraSpacing = 0.;  //6f;
#include <assert.h>
#include <math.h>

template <class T>
inline void MyClamp(T& a, const T& lb, const T& ub)
{
	if (a < lb)
	{
		a = lb;
	}
	else if (ub < a)
	{
		a = ub;
	}
}

static GLuint BindFont(const CTexFont* _Font)
{
	GLuint TexID = 0;
	glGenTextures(1, &TexID);
	glBindTexture(GL_TEXTURE_2D, TexID);
	glPixelStorei(GL_UNPACK_SWAP_BYTES, GL_FALSE);
	glPixelStorei(GL_UNPACK_LSB_FIRST, GL_FALSE);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
	glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, _Font->m_TexWidth, _Font->m_TexHeight, 0, GL_RED, GL_UNSIGNED_BYTE, _Font->m_TexBytes);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_2D, 0);

	return TexID;
}

struct MyTextureLoader
{
	virtual ~MyTextureLoader()
	{
	}
	virtual void LoadTexture(Gwen::Texture* pTexture) = 0;
	virtual void FreeTexture(Gwen::Texture* pTexture) = 0;
};

class GwenOpenGL3CoreRenderer : public Gwen::Renderer::Base
{
	GLPrimitiveRenderer* m_primitiveRenderer;
	float m_currentColor[4];
	float m_yOffset;
	sth_stash* m_font;
	float m_screenWidth;
	float m_screenHeight;
	float m_fontScaling;
	float m_retinaScale;
	bool m_useTrueTypeFont;
	const CTexFont* m_currentFont;

	GLuint m_fontTextureId;
	MyTextureLoader* m_textureLoader;

public:
	GwenOpenGL3CoreRenderer(GLPrimitiveRenderer* primRender, sth_stash* font, float screenWidth, float screenHeight, float retinaScale, MyTextureLoader* loader = 0)
		: m_primitiveRenderer(primRender),
		  m_font(font),
		  m_screenWidth(screenWidth),
		  m_screenHeight(screenHeight),
		  m_retinaScale(retinaScale),
		  m_useTrueTypeFont(false),
		  m_textureLoader(loader)
	{
		///only enable true type fonts on Macbook Retina, it looks gorgeous
		if (retinaScale == 2.0f)
		{
			m_useTrueTypeFont = true;
		}
		m_currentColor[0] = 1;
		m_currentColor[1] = 1;
		m_currentColor[2] = 1;
		m_currentColor[3] = 1;

		m_fontScaling = 16.f * m_retinaScale;

		TwGenerateDefaultFonts();

		m_currentFont = g_DefaultNormalFont;
		//m_currentFont = g_DefaultNormalFontAA;

		//m_currentFont = g_DefaultLargeFont;
		m_fontTextureId = BindFont(m_currentFont);
	}

	virtual ~GwenOpenGL3CoreRenderer()
	{
		TwDeleteDefaultFonts();
	}
	virtual void Resize(int width, int height)
	{
		m_screenWidth = width;
		m_screenHeight = height;
	}

	virtual void Begin()
	{
		m_yOffset = 0;
		glEnable(GL_BLEND);
		assert(glGetError() == GL_NO_ERROR);

		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		assert(glGetError() == GL_NO_ERROR);

		assert(glGetError() == GL_NO_ERROR);

		glDisable(GL_DEPTH_TEST);
		assert(glGetError() == GL_NO_ERROR);
		//glColor4ub(255,0,0,255);

		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		//	saveOpenGLState(width,height);//m_glutScreenWidth,m_glutScreenHeight);

		assert(glGetError() == GL_NO_ERROR);

		glDisable(GL_CULL_FACE);

		glDisable(GL_DEPTH_TEST);
		assert(glGetError() == GL_NO_ERROR);

		glEnable(GL_BLEND);

		assert(glGetError() == GL_NO_ERROR);
	}
	virtual void End()
	{
		glDisable(GL_BLEND);
	}

	virtual void StartClip()
	{
		if (m_useTrueTypeFont)
			sth_flush_draw(m_font);
		Gwen::Rect rect = ClipRegion();

		// OpenGL's coords are from the bottom left
		// so we need to translate them here.
		{
			GLint view[4];
			glGetIntegerv(GL_VIEWPORT, &view[0]);
			rect.y = view[3] / m_retinaScale - (rect.y + rect.h);
		}

		glScissor(m_retinaScale * rect.x * Scale(), m_retinaScale * rect.y * Scale(), m_retinaScale * rect.w * Scale(), m_retinaScale * rect.h * Scale());
		glEnable(GL_SCISSOR_TEST);
		//glDisable( GL_SCISSOR_TEST );
	};

	virtual void EndClip()
	{
		if (m_useTrueTypeFont)
			sth_flush_draw(m_font);
		glDisable(GL_SCISSOR_TEST);
	};

	virtual void SetDrawColor(Gwen::Color color)
	{
		m_currentColor[0] = color.r / 256.f;
		m_currentColor[1] = color.g / 256.f;
		m_currentColor[2] = color.b / 256.f;
		m_currentColor[3] = color.a / 256.f;
	}
	virtual void DrawFilledRect(Gwen::Rect rect)
	{
		//		BT_PROFILE("GWEN_DrawFilledRect");
		Translate(rect);

		m_primitiveRenderer->drawRect(rect.x, rect.y + m_yOffset,
									  rect.x + rect.w, rect.y + rect.h + m_yOffset, m_currentColor);

		//		m_primitiveRenderer->drawTexturedRect2a(rect.x, rect.y+m_yOffset,
		//		rect.x+rect.w, rect.y+rect.h+m_yOffset, m_currentColor,0,0,1,1);
		//		m_yOffset+=rect.h+10;
	}

	void RenderText(Gwen::Font* pFont, Gwen::Point rasterPos, const Gwen::UnicodeString& text)
	{
		//		BT_PROFILE("GWEN_RenderText");

		Gwen::String str = Gwen::Utility::UnicodeToString(text);
		const char* unicodeText = (const char*)str.c_str();

		Gwen::Rect r;
		r.x = rasterPos.x;
		r.y = rasterPos.y;
		r.w = 0;
		r.h = 0;

		//
		//printf("str = %s\n",unicodeText);
		//int xpos=0;
		//int ypos=0;
		float dx;

		int measureOnly = 0;

		if (m_useTrueTypeFont)
		{
			float yoffset = 0.f;
			if (m_retinaScale == 2.0f)
			{
				yoffset = -12;
			}
			Translate(r);
			sth_draw_text(m_font,
						  1, m_fontScaling,
						  r.x, r.y + yoffset,
						  unicodeText, &dx, m_screenWidth, m_screenHeight, measureOnly, m_retinaScale);
		}
		else
		{
			//float width = 0.f;
			int pos = 0;
			//float color[]={0.2f,0.2,0.2f,1.f};

			glBindTexture(GL_TEXTURE_2D, m_fontTextureId);
			float width = r.x;

			while (unicodeText[pos])
			{
				int c = unicodeText[pos];
				r.h = m_currentFont->m_CharHeight;
				r.w = m_currentFont->m_CharWidth[c] + extraSpacing;
				Gwen::Rect rect = r;
				Translate(rect);

				m_primitiveRenderer->drawTexturedRect2(rect.x, rect.y + m_yOffset, rect.x + rect.w, rect.y + rect.h + m_yOffset, m_currentColor, m_currentFont->m_CharU0[c], m_currentFont->m_CharV0[c], m_currentFont->m_CharU1[c], m_currentFont->m_CharV1[c]);

				width += r.w;
				r.x = width;
				pos++;
			}
			{
				m_primitiveRenderer->drawTexturedRect3D2Text(false);
			}
			glBindTexture(GL_TEXTURE_2D, 0);
		}
	}
	Gwen::Point MeasureText(Gwen::Font* pFont, const Gwen::UnicodeString& text)
	{
		//		BT_PROFILE("GWEN_MeasureText");
		Gwen::String str = Gwen::Utility::UnicodeToString(text);
		const char* unicodeText = (const char*)str.c_str();

		// printf("str = %s\n",unicodeText);
		int xpos = 0;
		int ypos = 0;

		int measureOnly = 1;
		float dx = 0;
		if (m_useTrueTypeFont)
		{
			sth_draw_text(m_font,
						  1, m_fontScaling,
						  xpos, ypos,
						  unicodeText, &dx, m_screenWidth, m_screenHeight, measureOnly);

			Gwen::Point pt;

			if (m_retinaScale == 2.0f)
			{
				pt.x = dx * Scale() / 2.f;
				pt.y = m_fontScaling / 2 * Scale() + 1;
			}
			else
			{
				pt.x = dx * Scale();
				pt.y = m_fontScaling * Scale() + 1;
			}
			return pt;
		}
		else
		{
			float width = 0.f;
			int pos = 0;
			while (unicodeText[pos])
			{
				width += m_currentFont->m_CharWidth[(int)unicodeText[pos]] + extraSpacing;
				pos++;
			}
			Gwen::Point pt;
			int fontHeight = m_currentFont->m_CharHeight;

			pt.x = width * Scale();
			pt.y = (fontHeight + 2) * Scale();

			return pt;
		}

		return Gwen::Renderer::Base::MeasureText(pFont, text);
	}

	virtual void LoadTexture(Gwen::Texture* pTexture)
	{
		if (m_textureLoader)
			m_textureLoader->LoadTexture(pTexture);
	}
	virtual void FreeTexture(Gwen::Texture* pTexture)
	{
		if (m_textureLoader)
			m_textureLoader->FreeTexture(pTexture);
	}

	virtual void DrawTexturedRect(Gwen::Texture* pTexture, Gwen::Rect rect, float u1 = 0.0f, float v1 = 0.0f, float u2 = 1.0f, float v2 = 1.0f)
	{
		//		BT_PROFILE("DrawTexturedRect");
		Translate(rect);

		//float eraseColor[4] = {0,0,0,0};
		//m_primitiveRenderer->drawRect(rect.x, rect.y+m_yOffset, rect.x+rect.w, rect.y+rect.h+m_yOffset, eraseColor);

		GLint texHandle = (GLint)pTexture->m_intData;
		//if (!texHandle)
		//	return;

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, texHandle);
		//		glDisable(GL_DEPTH_TEST);

		assert(glGetError() == GL_NO_ERROR);

		/*	bool useFiltering = true;
	if (useFiltering)
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	} else
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	}
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
*/

		//glEnable(GL_TEXTURE_2D);

		//	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE );
		static float add = 0.0;
		//add+=1./512.;//0.01;
		float color[4] = {1, 1, 1, 1};

		m_primitiveRenderer->drawTexturedRect(rect.x, rect.y + m_yOffset, rect.x + rect.w, rect.y + rect.h + m_yOffset, color, 0 + add, 0, 1 + add, 1, true);

		assert(glGetError() == GL_NO_ERROR);
	}
};
#endif  //__GWEN_OPENGL3_CORE_RENDERER_H
