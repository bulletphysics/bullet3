/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Controls/Slider.h"
#include "Gwen/Controls/HorizontalSlider.h"

using namespace Gwen;
using namespace Gwen::Controls;
using namespace Gwen::ControlsInternal;

GWEN_CONTROL_CONSTRUCTOR(HorizontalSlider)
{
}

float HorizontalSlider::CalculateValue()
{
	return (float)m_SliderBar->X() / (float)(Width() - m_SliderBar->Width());
}

void HorizontalSlider::UpdateBarFromValue()
{
	m_SliderBar->MoveTo((Width() - m_SliderBar->Width()) * (m_fValue), m_SliderBar->Y());
}

void HorizontalSlider::OnMouseClickLeft(int x, int y, bool bDown)
{
	m_SliderBar->MoveTo(CanvasPosToLocal(Gwen::Point(x, y)).x - m_SliderBar->Width() * 0.5, m_SliderBar->Y());
	m_SliderBar->OnMouseClickLeft(x, y, bDown);
	OnMoved(m_SliderBar);
}

void HorizontalSlider::Layout(Skin::Base* /*skin*/)
{
	m_SliderBar->SetSize(10, Height());
}

void HorizontalSlider::Render(Skin::Base* skin)
{
	skin->DrawSlider(this, true, m_bClampToNotches ? m_iNumNotches : 0, m_SliderBar->Width());
}