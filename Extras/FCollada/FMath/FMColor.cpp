/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FMColor.h"

FMColor::FMColor(const float* components, uint32 componentCount)
{
	switch (componentCount)
	{
	case 1:
		r = (uint8) (components[0] * 255.0f);
		g = 0; b = 0; a = 255;
		break;

	case 2:
		r = (uint8) (components[0] * 255.0f);
		g = (uint8) (components[1] * 255.0f);
		b = 0; a = 255;
		break;

	case 3:
		r = (uint8) (components[0] * 255.0f);
		g = (uint8) (components[1] * 255.0f);
		b = (uint8) (components[2] * 255.0f);
		a = 255;
		break;

	case 4:
		r = (uint8) (components[0] * 255.0f);
		g = (uint8) (components[1] * 255.0f);
		b = (uint8) (components[2] * 255.0f);
		a = (uint8) (components[3] * 255.0f);
		break;
	
	default:
		r = 0; g = 0; b = 0; a = 255;
		break;
	}
}

void FMColor::ToFloats(float* components, uint32 componentCount)
{
	switch (componentCount)
	{
	case 1:
		components[0] = float(r) / 255.0f;
		break;

	case 2:
		components[0] = float(r) / 255.0f;
		components[1] = float(g) / 255.0f;
		break;

	case 3:
		components[0] = float(r) / 255.0f;
		components[1] = float(g) / 255.0f;
		components[2] = float(b) / 255.0f;
		break;

	case 4:
		components[0] = float(r) / 255.0f;
		components[1] = float(g) / 255.0f;
		components[2] = float(b) / 255.0f;
		break;
	
	default:
		for (uint32 i = 0; i < componentCount; ++i) components[i] = 0.0f;
		break;
	}
}

