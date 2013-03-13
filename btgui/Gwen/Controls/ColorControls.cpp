/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Utility.h"
#include "Gwen/Controls/ColorControls.h"

using namespace Gwen;
using namespace Gwen::Controls;

//Find a place to put these...
Color HSVToColor( float h, float s, float v )
{
	if (h < 0.0f) h += 360.0f;
	if (h > 360.0f) h -= 360.0f;

	s *= 255.0f;
	v *= 255.0f;

	float r, g, b;

	if (!h && !s)
	{
		r = g = b = v;
	}
	double min,max,delta,hue;

	max = v;
	delta = (max * s)/255.0;
	min = max - delta;

	hue = h;
	if (h > 300 || h <= 60)
	{
		r = (int)max;
		if (h > 300)
		{
			g = (int)min;
			hue = (hue - 360.0)/60.0;
			b = (int)((hue * delta - min) * -1);
		}
		else
		{
			b = (int)min;
			hue = hue / 60.0;
			g = (int)(hue * delta + min);
		}
	}
	else if (h > 60 && h < 180)
	{
		g = (int)max;
		if (h < 120)
		{
			b = (int)min;
			hue = (hue/60.0 - 2.0 ) * delta;
			r = (int)(min - hue);
		}
		else
		{
			r = (int)min;
			hue = (hue/60 - 2.0) * delta;
			b = (int)(min + hue);
		}
	}
	else
	{
		b = (int)max;
		if (h < 240)
		{
			r = (int)min;
			hue = (hue/60.0 - 4.0 ) * delta;
			g = (int)(min - hue);
		}
		else
		{
			g = (int)min;
			hue = (hue/60 - 4.0) * delta;
			r = (int)(min + hue);
		}
	}

	return Color( r, g, b, 255);
}

HSV RGBtoHSV( int r, int g, int b )
{
	double min,max,delta,temp;
	min = GwenUtil_Min(r,GwenUtil_Min(g,b));
	max = GwenUtil_Max(r,GwenUtil_Max(g,b));
	delta = max - min;

	HSV hsv;
	hsv.v = (int)max;
	if (!delta)
	{
		hsv.h = hsv.s = 0;
	}
	else
	{
		temp = delta/max;
		hsv.s = (int)(temp*255);

		if (r == (int)max)
		{
			temp = (double)(g-b)/delta;
		}
		else
			if (g == (int)max)
			{
				temp = 2.0 + ((double)(b-r)/delta);
			}
			else
			{
				temp = 4.0 + ((double)(r-g)/delta);
			}
			temp *= 60;
			if (temp < 0)
			{
				temp+=360;
			}
			if (temp == 360)
			{
				temp = 0;
			}
			hsv.h = (int)temp;
	}


	hsv.s /= 255.0f;
	hsv.v /= 255.0f;

	return hsv;
}


GWEN_CONTROL_CONSTRUCTOR( ColorLerpBox )
{
	SetColor( Gwen::Color(255, 128, 0, 255) );
	SetSize( 128, 128 );
	SetMouseInputEnabled( true );
	m_bDepressed = false;
}  

//Find a place to put this? color member?
Gwen::Color LerpColor( Gwen::Color &toColor, Gwen::Color &fromColor, float amount )
{
	Gwen::Color colorDelta = toColor - fromColor;

	colorDelta.r *= amount;
	colorDelta.g *= amount;
	colorDelta.b *= amount;

	Gwen::Color newColor = fromColor + colorDelta;
	return newColor;
}

Gwen::Color ColorLerpBox::GetSelectedColor()
{
	return GetColorAtPos( cursorPos.x, cursorPos.y );
}

void ColorLerpBox::SetColor( Gwen::Color color, bool onlyHue )
{
	HSV hsv = RGBtoHSV(color.r, color.g, color.b);
	m_Hue = hsv.h;
	if ( !onlyHue )
	{
		cursorPos.x = hsv.s * Width();
		cursorPos.y = (1 - hsv.v) * Height();
	}

	onSelectionChanged.Call( this );
}

void ColorLerpBox::OnMouseMoved( int x, int y, int /*deltaX*/, int /*deltaY*/ )
{
	if ( m_bDepressed )
	{
		cursorPos = CanvasPosToLocal( Gwen::Point( x, y ) );
		//Do we have clamp?
		if ( cursorPos.x < 0)
			cursorPos.x = 0;
		if (cursorPos.x > Width())
			cursorPos.x = Width();

		if (cursorPos.y < 0)
			cursorPos.y = 0;
		if (cursorPos.y > Height())
			cursorPos.y = Height();

		onSelectionChanged.Call( this );
	}
}

void ColorLerpBox::OnMouseClickLeft( int x, int y, bool bDown )
{
	m_bDepressed = bDown;
	if ( bDown )
		Gwen::MouseFocus = this;
	else
		Gwen::MouseFocus = NULL;

	OnMouseMoved( x, y, 0, 0);
}

Gwen::Color ColorLerpBox::GetColorAtPos( int x, int y )
{
	float xPercent =     ( (float)x / (float)Width()  );
	float yPercent = 1 - ( (float)y / (float)Height() );

	Gwen::Color result = HSVToColor( m_Hue, xPercent, yPercent );

	result.a = 255;

	return result;
}
void ColorLerpBox::Render( Gwen::Skin::Base* skin )
{
	//Is there any way to move this into skin? Not for now, no idea how we'll "actually" render these
	BaseClass::Render( skin );
	for ( int x = 0; x<Width(); x++)
	{
		for ( int y = 0; y<Height(); y++)
		{
			skin->GetRender()->SetDrawColor( GetColorAtPos( x, y ) );
			skin->GetRender()->DrawPixel( x, y );
		}
	}

	skin->GetRender()->SetDrawColor( Gwen::Color( 0, 0, 0, 255 ) );
	skin->GetRender()->DrawLinedRect( GetRenderBounds() );

	Gwen::Color selected = GetSelectedColor();
	if ( (selected.r + selected.g + selected.b) / 3 < 170 )
		skin->GetRender()->SetDrawColor( Gwen::Color( 255, 255, 255, 255 ) );
	else
		skin->GetRender()->SetDrawColor( Gwen::Color( 0, 0, 0, 255 ) );

	Gwen::Rect testRect = Gwen::Rect( cursorPos.x -3, cursorPos.y -3, 6, 6);

	skin->GetRender()->DrawShavedCornerRect( testRect );
}





GWEN_CONTROL_CONSTRUCTOR( ColorSlider )
{
	SetSize( 32, 128 );
	SetMouseInputEnabled( true );
	m_bDepressed = false;
}

void ColorSlider::Render( Gwen::Skin::Base* skin )
{
	//Is there any way to move this into skin? Not for now, no idea how we'll "actually" render these
	int y = 0;
	for ( y = 0; y < Height(); y++)
	{
		float yPercent = (float)y / (float)Height();
		skin->GetRender()->SetDrawColor( HSVToColor( yPercent * 360, 1, 1 ) );
		skin->GetRender()->DrawFilledRect( Gwen::Rect( 5, y, Width() - 10, 1 ) );
	}

	int drawHeight = m_iSelectedDist - 3;

	//Draw our selectors
	skin->GetRender()->SetDrawColor( Gwen::Color( 0, 0, 0, 255 ));
		skin->GetRender()->DrawFilledRect( Gwen::Rect( 0, drawHeight + 2, Width(), 1));
		skin->GetRender()->DrawFilledRect( Gwen::Rect( 0, drawHeight, 5, 5) );
		skin->GetRender()->DrawFilledRect( Gwen::Rect( Width() - 5, drawHeight, 5, 5) );
	skin->GetRender()->SetDrawColor( Gwen::Color( 255, 255, 255, 255 ) );
		skin->GetRender()->DrawFilledRect( Gwen::Rect( 1, drawHeight + 1, 3, 3 ) );
		skin->GetRender()->DrawFilledRect( Gwen::Rect( Width() - 4, drawHeight + 1, 3, 3 ) );
}

void ColorSlider::OnMouseClickLeft( int x, int y, bool bDown )
{
	m_bDepressed = bDown;
	if ( bDown)
		Gwen::MouseFocus = this;
	else
		Gwen::MouseFocus = NULL;

	OnMouseMoved(x, y, 0, 0);
}

Gwen::Color ColorSlider::GetColorAtHeight( int y )
{
	float yPercent = (float)y / (float)Height();
	return HSVToColor( yPercent * 360, 1, 1);

}
void ColorSlider::OnMouseMoved( int x, int y, int /*deltaX*/, int /*deltaY*/ )
{
	if ( m_bDepressed )
	{
		Gwen::Point cursorPos = CanvasPosToLocal( Gwen::Point( x, y ) );

		if (cursorPos.y < 0)
			cursorPos.y = 0;
		if (cursorPos.y > Height())
			cursorPos.y = Height();

		m_iSelectedDist = cursorPos.y;
		onSelectionChanged.Call( this );
	}
}

void ColorSlider::SetColor( Gwen::Color color)
{
	HSV hsv = RGBtoHSV( color.r, color.g, color.b );

	m_iSelectedDist = hsv.h / 360 * Height();

	onSelectionChanged.Call( this );
}

Gwen::Color ColorSlider::GetSelectedColor()
{
	return GetColorAtHeight( m_iSelectedDist );
}
