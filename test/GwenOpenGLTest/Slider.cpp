#include "UnitTest.h"
#include "Gwen/Controls/RadioButtonController.h"
#include "Gwen/Controls/VerticalSlider.h"
#include "Gwen/Controls/HorizontalSlider.h"

using namespace Gwen;

class Slider : public GUnit
{
public:
	GWEN_CONTROL_INLINE(Slider, GUnit)
	{
		{
			Gwen::Controls::HorizontalSlider* pSlider = new Gwen::Controls::HorizontalSlider(this);
			pSlider->SetPos(10, 10);
			pSlider->SetSize(150, 20);
			pSlider->SetRange(0, 100);
			pSlider->SetValue(25);
			pSlider->onValueChanged.Add(this, &Slider::SliderMoved);
		}

		{
			Gwen::Controls::HorizontalSlider* pSlider = new Gwen::Controls::HorizontalSlider(this);
			pSlider->SetPos(10, 40);
			pSlider->SetSize(150, 20);
			pSlider->SetRange(0, 100);
			pSlider->SetValue(25);
			pSlider->SetNotchCount(10);
			pSlider->SetClampToNotches(true);
			pSlider->onValueChanged.Add(this, &Slider::SliderMoved);
		}

		{
			Gwen::Controls::VerticalSlider* pSlider = new Gwen::Controls::VerticalSlider(this);
			pSlider->SetPos(160, 10);
			pSlider->SetSize(20, 200);
			pSlider->SetRange(0, 100);
			pSlider->SetValue(25);
			pSlider->onValueChanged.Add(this, &Slider::SliderMoved);
		}

		{
			Gwen::Controls::VerticalSlider* pSlider = new Gwen::Controls::VerticalSlider(this);
			pSlider->SetPos(190, 10);
			pSlider->SetSize(20, 200);
			pSlider->SetRange(0, 100);
			pSlider->SetValue(25);
			pSlider->SetNotchCount(10);
			pSlider->SetClampToNotches(true);
			pSlider->onValueChanged.Add(this, &Slider::SliderMoved);
		}
	}

	void SliderMoved(Base* pControl)
	{
		Gwen::Controls::Slider* pSlider = (Gwen::Controls::Slider*)pControl;

		UnitPrint(Utility::Format(L"Slider Value: %.2f", pSlider->GetValue()));
	}
};

DEFINE_UNIT_TEST(Slider, L"Slider");