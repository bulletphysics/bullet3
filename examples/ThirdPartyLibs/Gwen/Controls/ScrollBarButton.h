/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_SCROLLBARBOTTON_H
#define GWEN_CONTROLS_SCROLLBARBOTTON_H

#include "Gwen/Controls/Button.h"

namespace Gwen
{
namespace ControlsInternal
{
class GWEN_EXPORT ScrollBarButton : public Controls::Button
{
public:
	GWEN_CONTROL(ScrollBarButton, Controls::Button);

	void Render(Skin::Base* skin);

	void SetDirectionUp();
	void SetDirectionDown();
	void SetDirectionLeft();
	void SetDirectionRight();

protected:
	int m_iDirection;
};
}  // namespace ControlsInternal
}  // namespace Gwen
#endif
