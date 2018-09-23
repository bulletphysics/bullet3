/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Controls/ImagePanel.h"
#include "Gwen/Controls/Label.h"
#include "Gwen/Controls/Resizer.h"
#include "Gwen/Controls/ResizableControl.h"

using namespace Gwen;
using namespace Gwen::Controls;
using namespace Gwen::ControlsInternal;

GWEN_CONTROL_CONSTRUCTOR(ResizableControl)
{
	m_bResizable = true;
	m_MinimumSize = Gwen::Point(5, 5);
	m_bClampMovement = false;

	Resizer* resizerBottom = new Resizer(this);
	resizerBottom->Dock(Pos::Bottom);
	resizerBottom->SetResizeDir(Pos::Bottom);
	resizerBottom->SetTarget(this);
	resizerBottom->onResize.Add(this, &ResizableControl::OnResizedInternal);

	Resizer* resizerBottomLeft = new Resizer(resizerBottom);
	resizerBottomLeft->Dock(Pos::Left);
	resizerBottomLeft->SetResizeDir(Pos::Bottom | Pos::Left);
	resizerBottomLeft->SetTarget(this);
	resizerBottomLeft->onResize.Add(this, &ResizableControl::OnResizedInternal);

	Resizer* resizerBottomRight = new Resizer(resizerBottom);
	resizerBottomRight->Dock(Pos::Right);
	resizerBottomRight->SetResizeDir(Pos::Bottom | Pos::Right);
	resizerBottomRight->SetTarget(this);
	resizerBottomRight->onResize.Add(this, &ResizableControl::OnResizedInternal);

	Resizer* resizerTop = new Resizer(this);
	resizerTop->Dock(Pos::Top);
	resizerTop->SetResizeDir(Pos::Top);
	resizerTop->SetTarget(this);
	resizerTop->onResize.Add(this, &ResizableControl::OnResizedInternal);

	Resizer* resizerTopLeft = new Resizer(resizerTop);
	resizerTopLeft->Dock(Pos::Left);
	resizerTopLeft->SetResizeDir(Pos::Top | Pos::Left);
	resizerTopLeft->SetTarget(this);
	resizerTopLeft->onResize.Add(this, &ResizableControl::OnResizedInternal);

	Resizer* resizerTopRight = new Resizer(resizerTop);
	resizerTopRight->Dock(Pos::Right);
	resizerTopRight->SetResizeDir(Pos::Top | Pos::Right);
	resizerTopRight->SetTarget(this);
	resizerTopRight->onResize.Add(this, &ResizableControl::OnResizedInternal);

	Resizer* resizerLeft = new Resizer(this);
	resizerLeft->Dock(Pos::Left);
	resizerLeft->SetResizeDir(Pos::Left);
	resizerLeft->SetTarget(this);
	resizerLeft->onResize.Add(this, &ResizableControl::OnResizedInternal);

	Resizer* resizerRight = new Resizer(this);
	resizerRight->Dock(Pos::Right);
	resizerRight->SetResizeDir(Pos::Right);
	resizerRight->SetTarget(this);
	resizerRight->onResize.Add(this, &ResizableControl::OnResizedInternal);
}

void ResizableControl::DisableResizing()
{
	for (Base::List::iterator it = Children.begin(); it != Children.end(); ++it)
	{
		Resizer* resizer = (*it)->DynamicCastResizer();
		if (!resizer) continue;

		resizer->SetMouseInputEnabled(false);
		resizer->SetHidden(true);
		SetPadding(Padding(resizer->Width(), resizer->Width(), resizer->Width(), resizer->Width()));
	}
}

bool ResizableControl::SetBounds(int x, int y, int w, int h)
{
	Gwen::Point minSize = GetMinimumSize();

	// Clamp Minimum Size
	if (w < minSize.x) w = minSize.x;
	if (h < minSize.y) h = minSize.y;

	// Clamp to parent's window
	Base* pParent = GetParent();
	if (pParent && m_bClampMovement)
	{
		if (x + w > pParent->Width()) x = pParent->Width() - w;
		if (x < 0) x = 0;
		if (y + h > pParent->Height()) y = pParent->Height() - h;
		if (y < 0) y = 0;
	}

	return BaseClass::SetBounds(x, y, w, h);
}

void ResizableControl::OnResizedInternal(Controls::Base* /*pControl*/)
{
	onResize.Call(this);
	OnResized();
}