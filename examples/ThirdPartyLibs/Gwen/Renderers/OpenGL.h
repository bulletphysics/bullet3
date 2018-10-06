/*
	GWEN
	Copyright (c) 2011 Facepunch Studios
	See license in Gwen.h
*/

#ifndef GWEN_RENDERERS_OPENGL_H
#define GWEN_RENDERERS_OPENGL_H

#include "Gwen/Gwen.h"
#include "Gwen/BaseRender.h"

namespace Gwen
{
namespace Renderer
{
class OpenGL : public Gwen::Renderer::Base
{
public:
	struct Vertex
	{
		float x, y, z;
		float u, v;
		unsigned char r, g, b, a;
	};

	OpenGL();
	~OpenGL();

	virtual void Begin();
	virtual void End();

	virtual void SetDrawColor(Gwen::Color color);
	virtual void DrawFilledRect(Gwen::Rect rect);

	void StartClip();
	void EndClip();

	void DrawTexturedRect(Gwen::Texture* pTexture, Gwen::Rect pTargetRect, float u1 = 0.0f, float v1 = 0.0f, float u2 = 1.0f, float v2 = 1.0f);
	void LoadTexture(Gwen::Texture* pTexture);
	void FreeTexture(Gwen::Texture* pTexture);

protected:
	static const int MaxVerts = 1024;

	void Flush();
	void AddVert(int x, int y, float u = 0.0f, float v = 0.0f);

	Gwen::Color m_Color;
	int m_iVertNum;
	Vertex m_Vertices[MaxVerts];
};

}  // namespace Renderer
}  // namespace Gwen
#endif
