/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Controls/Slider.h"
#include "Gwen/Controls/VerticalSlider.h"

using namespace Gwen;
using namespace Gwen::Controls;
using namespace Gwen::ControlsInternal;

GWEN_CONTROL_CONSTRUCTOR(VerticalSlider)
{
}

float VerticalSlider::CalculateValue()
{
	return 1 - (float)m_SliderBar->Y() / (float)(Height() - m_SliderBar->Height());
}

void VerticalSlider::UpdateBarFromValue()
{
	m_SliderBar->MoveTo(m_SliderBar->X(), (Height() - m_SliderBar->Height()) * (1 - m_fValue));
}

void VerticalSlider::OnMouseClickLeft(int x, int y, bool bDown)
{
	m_SliderBar->MoveTo(m_SliderBar->X(), CanvasPosToLocal(Gwen::Point(x, y)).y - m_SliderBar->Height() * 0.5);
	m_SliderBar->OnMouseClickLeft(x, y, bDown);
	OnMoved(m_SliderBar);
}

void VerticalSlider::Layout(Skin::Base* /*skin*/)
{
	m_SliderBar->SetSize(Width(), 10);
}

void VerticalSlider::Render(Skin::Base* skin)
{
	skin->DrawSlider(this, false, m_bClampToNotches ? m_iNumNotches : 0, m_SliderBar->Height());
}