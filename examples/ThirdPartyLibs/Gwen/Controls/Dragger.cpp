/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Controls/Dragger.h"

using namespace Gwen;
using namespace Gwen::ControlsInternal;

GWEN_CONTROL_CONSTRUCTOR(Dragger)
{
	m_pTarget = NULL;
	SetMouseInputEnabled(true);
	m_bDepressed = false;
}

void Dragger::OnMouseClickLeft(int x, int y, bool bDown)
{
	if (!m_pTarget) return;

	if (bDown)
	{
		m_bDepressed = true;
		m_HoldPos = m_pTarget->CanvasPosToLocal(Gwen::Point(x, y));
		Gwen::MouseFocus = this;
	}
	else
	{
		m_bDepressed = false;

		Gwen::MouseFocus = NULL;
	}
}

void Dragger::OnMouseMoved(int x, int y, int /*deltaX*/, int /*deltaY*/)
{
	if (!m_pTarget) return;
	if (!m_bDepressed) return;

	Gwen::Point p = Gwen::Point(x - m_HoldPos.x, y - m_HoldPos.y);

	// Translate to parent
	if (m_pTarget->GetParent())
		p = m_pTarget->GetParent()->CanvasPosToLocal(p);

	//m_pTarget->SetPosition( p.x, p.y );
	m_pTarget->MoveTo(p.x, p.y);
	onDragged.Call(this);
}

void Dragger::Render(Skin::Base* /*skin*/)
{
	//skin->DrawButton(this,false,false);
}
