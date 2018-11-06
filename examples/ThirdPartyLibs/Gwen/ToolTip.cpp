/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/ToolTip.h"
#include "Gwen/Utility.h"

using namespace Gwen;
using namespace Gwen::Controls;

namespace ToolTip
{
Base* g_ToolTip = NULL;

void Enable(Controls::Base* pControl)
{
	if (!pControl->GetToolTip())
		return;

	g_ToolTip = pControl;
}

void Disable(Controls::Base* pControl)
{
	if (g_ToolTip == pControl)
	{
		g_ToolTip = NULL;
	}
}

void RenderToolTip(Skin::Base* skin)
{
	if (!g_ToolTip) return;

	Gwen::Renderer::Base* render = skin->GetRender();

	Gwen::Point pOldRenderOffset = render->GetRenderOffset();
	Gwen::Point MousePos = Input::GetMousePosition();
	Gwen::Rect Bounds = g_ToolTip->GetToolTip()->GetBounds();

	Gwen::Rect rOffset = Gwen::Rect(MousePos.x - Bounds.w * 0.5f, MousePos.y - Bounds.h - 10, Bounds.w, Bounds.h);
	rOffset = Utility::ClampRectToRect(rOffset, g_ToolTip->GetCanvas()->GetBounds());

	//Calculate offset on screen bounds
	render->AddRenderOffset(rOffset);
	render->EndClip();

	skin->DrawToolTip(g_ToolTip->GetToolTip());
	g_ToolTip->GetToolTip()->DoRender(skin);

	render->SetRenderOffset(pOldRenderOffset);
}

void ControlDeleted(Controls::Base* pControl)
{
	Disable(pControl);
}
}  // namespace ToolTip
