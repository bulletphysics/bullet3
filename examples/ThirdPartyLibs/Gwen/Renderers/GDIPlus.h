/*
	GWEN
	Copyright (c) 2011 Facepunch Studios
	See license in Gwen.h
*/

#ifndef GWEN_RENDERERS_GDIPLUS_H
#define GWEN_RENDERERS_GDIPLUS_H

#include "Gwen/Gwen.h"
#include "Gwen/BaseRender.h"

/*

 GDI(plus) is pretty slow for rendering GWEN, because we're
 re-rendering everything on redraw.

 Therefore its usage should be as a test - rather than production.

 // Note: For this to work you should be including

 #include <gdiplus.h>

 // Which we don't do in the header, for the sake of usability

*/

namespace Gwen
{
namespace Renderer
{
class GDIPlus : public Gwen::Renderer::Base
{
public:
	GDIPlus(HWND pHWND);
	~GDIPlus();

	virtual void Begin();
	virtual void End();

	virtual void SetDrawColor(Gwen::Color color);

	virtual void DrawLine(int x, int y, int a, int b);
	virtual void DrawFilledRect(Gwen::Rect rect);

	virtual void LoadFont(Gwen::Font* pFont);
	virtual void FreeFont(Gwen::Font* pFont);
	virtual void RenderText(Gwen::Font* pFont, Gwen::Point pos, const Gwen::UnicodeString& text);
	virtual Gwen::Point MeasureText(Gwen::Font* pFont, const Gwen::UnicodeString& text);

	void StartClip();
	void EndClip();

	void DrawTexturedRect(Gwen::Texture* pTexture, Gwen::Rect pTargetRect, float u1 = 0.0f, float v1 = 0.0f, float u2 = 1.0f, float v2 = 1.0f);
	void LoadTexture(Gwen::Texture* pTexture);
	void FreeTexture(Gwen::Texture* pTexture);

protected:
	int m_iWidth;
	int m_iHeight;

	Gdiplus::Color m_Colour;

	HWND m_HWND;
	HDC m_hDC;
	ULONG_PTR m_gdiplusToken;

	Gdiplus::Graphics* graphics;
};

class GDIPlusBuffered : public GDIPlus
{
public:
	GDIPlusBuffered(HWND pHWND);
	~GDIPlusBuffered();

	virtual void Begin();
	virtual void End();

private:
	void CreateBackbuffer();
	void DestroyBackbuffer();

	Gdiplus::Bitmap* m_Bitmap;
};
}  // namespace Renderer
}  // namespace Gwen
#endif
