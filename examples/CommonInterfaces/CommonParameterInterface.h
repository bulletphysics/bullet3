
#ifndef PARAM_INTERFACE_H
#define PARAM_INTERFACE_H

#pragma once

typedef void (*SliderParamChangedCallback)(float newVal, void* userPointer);
#include "LinearMath/btScalar.h"

struct SliderParams
{
	const char* m_name;
	float m_minVal;
	float m_maxVal;
	SliderParamChangedCallback m_callback;
	btScalar* m_paramValuePointer;
	void* m_userPointer;
	bool m_clampToNotches;
	bool m_clampToIntegers;
	bool m_showValues;

	SliderParams(const char* name, btScalar* targetValuePointer)
		: m_name(name),
		  m_minVal(-100),
		  m_maxVal(100),
		  m_callback(0),
		  m_paramValuePointer(targetValuePointer),
		  m_userPointer(0),
		  m_clampToNotches(false),
		  m_clampToIntegers(false),
		  m_showValues(true)
	{
	}
};

typedef void (*ButtonParamChangedCallback)(int buttonId, bool buttonState, void* userPointer);
typedef void (*ComboBoxCallback)(int combobox, const char* item, void* userPointer);

struct ButtonParams
{
	const char* m_name;
	int m_buttonId;
	void* m_userPointer;
	bool m_isTrigger;
	bool m_initialState;

	ButtonParamChangedCallback m_callback;
	ButtonParams(const char* name, int buttonId, bool isTrigger)
		: m_name(name),
		  m_buttonId(buttonId),
		  m_userPointer(0),
		  m_isTrigger(isTrigger),
		  m_initialState(false),
		  m_callback(0)
	{
	}
};

struct ComboBoxParams
{
	int m_comboboxId;
	int m_numItems;
	const char** m_items;
	int m_startItem;
	ComboBoxCallback m_callback;
	void* m_userPointer;

	ComboBoxParams()
		: m_comboboxId(-1),
		  m_numItems(0),
		  m_items(0),
		  m_startItem(0),
		  m_callback(0),
		  m_userPointer(0)
	{
	}
};

struct CommonParameterInterface
{
	virtual ~CommonParameterInterface() {}
	virtual void registerSliderFloatParameter(SliderParams& params) = 0;
	virtual void registerButtonParameter(ButtonParams& params) = 0;
	virtual void registerComboBox(ComboBoxParams& params) = 0;

	virtual void syncParameters() = 0;
	virtual void removeAllParameters() = 0;
	virtual void setSliderValue(int sliderIndex, double sliderValue) = 0;
};

#endif  //PARAM_INTERFACE_H
