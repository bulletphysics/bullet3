/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2020 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/// @author Ian Purvis <ian@purvisresearch.com>

#ifndef BT_HEADLESS_PARAMETERS_H
#define BT_HEADLESS_PARAMETERS_H
#include "CommonInterfaces/CommonParameterInterface.h"

struct HeadlessParameters : public CommonParameterInterface
{
public:
	HeadlessParameters() = default;
	~HeadlessParameters() = default;

	void registerButtonParameter(ButtonParams& params);
	void registerComboBox(ComboBoxParams& params);
	void registerSliderFloatParameter(SliderParams& params);
	void removeAllParameters();
	void setSliderValue(int sliderIndex, double sliderValue);
	void syncParameters();
};

#endif  //BT_HEADLESS_PARAMETERS_H
