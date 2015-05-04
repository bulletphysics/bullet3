#include "GwenParameterInterface.h"
#include "gwenInternalData.h"



template<typename T>
struct MySliderEventHandler : public Gwen::Event::Handler
{
	Gwen::Controls::TextBox* m_label;
	Gwen::Controls::Slider* m_pSlider;
	char m_variableName[1024];
	T* m_targetValue;
    bool m_showValue;

	MySliderEventHandler(const char* varName, Gwen::Controls::TextBox* label, Gwen::Controls::Slider* pSlider,T* target)
		:m_label(label),
		m_pSlider(pSlider),
		m_targetValue(target),
        m_showValue(true)
	{
		memcpy(m_variableName,varName,strlen(varName)+1);
	}


	void SliderMoved( Gwen::Controls::Base* pControl )
	{
		Gwen::Controls::Slider* pSlider = (Gwen::Controls::Slider*)pControl;
		//printf("value = %f\n", pSlider->GetValue());//UnitPrint( Utility::Format( L"Slider Value: %.2f", pSlider->GetValue() ) );
		float bla = pSlider->GetValue();
		T v = T(bla);
		SetValue(v);

	}

	void	SetValue(T v)
	{
		if (v < m_pSlider->GetRangeMin())
		{
			printf("?\n");
		}

		if (v > m_pSlider->GetRangeMax())
		{
						printf("?\n");

		}
		m_pSlider->SetValue(v,true);
		(*m_targetValue) = v;
		float val = float(v);//todo: specialize on template type
        if (m_showValue)
        {
            char txt[1024];
            sprintf(txt,"%s : %.3f", m_variableName,val);
            m_label->SetText(txt);
        }

	}
};


struct  GwenParameters
{
	b3AlignedObjectArray<MySliderEventHandler<btScalar>*> m_sliderEventHandlers;
	b3AlignedObjectArray<Gwen::Controls::HorizontalSlider*> m_sliders;
	b3AlignedObjectArray<Gwen::Controls::TextBox*> m_textLabels;
	int m_savedYposition;
};

GwenParameterInterface::GwenParameterInterface(GwenInternalData* gwenInternalData)
:m_gwenInternalData(gwenInternalData)
{
	m_paramInternalData = new GwenParameters;
	m_paramInternalData->m_savedYposition = m_gwenInternalData->m_curYposition;

}

GwenParameterInterface::~GwenParameterInterface()
{

	removeAllParameters();
	delete m_paramInternalData;
}


void GwenParameterInterface::setSliderValue(int sliderIndex, double sliderValue)
{
    int sliderCapped = sliderValue+4;
    sliderCapped /= 8;
    sliderCapped *= 8;
    
    if (sliderIndex>=0 && sliderIndex<m_paramInternalData->m_sliders.size())
    {
        m_paramInternalData->m_sliders[sliderIndex]->GetRangeMin();
        
        m_paramInternalData->m_sliders[sliderIndex]->GetRangeMax();
        float mappedValue =m_paramInternalData->m_sliders[sliderIndex]->GetRangeMin()+
        (m_paramInternalData->m_sliders[sliderIndex]->GetRangeMax()-
         m_paramInternalData->m_sliders[sliderIndex]->GetRangeMin())*sliderCapped/128.f;
        printf("mappedValue = %f\n",mappedValue);
        m_paramInternalData->m_sliders[sliderIndex]->SetValue(mappedValue);
    }
}

#include <stdio.h>
void GwenParameterInterface::registerSliderFloatParameter(SliderParams& params)
{
	Gwen::Controls::TextBox* label = new Gwen::Controls::TextBox(m_gwenInternalData->m_demoPage->GetPage());
	m_paramInternalData->m_textLabels.push_back(label);
	//m_data->m_myControls.push_back(label);
	label->SetText( params.m_name);
	label->SetPos( 10, 10 + 25 );
	label->SetWidth(110);
	label->SetPos(10,m_gwenInternalData->m_curYposition);
	m_gwenInternalData->m_curYposition+=22;

	Gwen::Controls::HorizontalSlider* pSlider = new Gwen::Controls::HorizontalSlider( m_gwenInternalData->m_demoPage->GetPage());
	m_paramInternalData->m_sliders.push_back(pSlider);
	//m_data->m_myControls.push_back(pSlider);
	pSlider->SetPos( 10, m_gwenInternalData->m_curYposition );
	pSlider->SetSize( 100, 20 );
	pSlider->SetRange( params.m_minVal, params.m_maxVal);
	pSlider->SetNotchCount(128);//float(params.m_maxVal-params.m_minVal)/100.f);
	pSlider->SetClampToNotches( params.m_clampToNotches );
	pSlider->SetValue( *params.m_paramValuePointer);//dimensions[i] );
	char labelName[1024];
	sprintf(labelName,"%s",params.m_name);//axisNames[0]);
	MySliderEventHandler<btScalar>* handler = new MySliderEventHandler<btScalar>(labelName,label,pSlider,params.m_paramValuePointer);
    handler->m_showValue = params.m_showValues;
	m_paramInternalData->m_sliderEventHandlers.push_back(handler);

	pSlider->onValueChanged.Add( handler, &MySliderEventHandler<btScalar>::SliderMoved );
	handler->SliderMoved(pSlider);
//	float v = pSlider->GetValue();
	m_gwenInternalData->m_curYposition+=22;
}

void GwenParameterInterface::syncParameters()
{
	for (int i=0;i<m_paramInternalData->m_sliderEventHandlers.size();i++)
	{
		MySliderEventHandler<btScalar>* handler = m_paramInternalData->m_sliderEventHandlers[i];
		handler->m_pSlider->SetValue(*handler->m_targetValue,true);
	}
}

void GwenParameterInterface::removeAllParameters()
{
	for (int i=0;i<m_paramInternalData->m_sliders.size();i++)
	{
		delete m_paramInternalData->m_sliders[i];
	}
	m_paramInternalData->m_sliders.clear();

	for (int i=0;i<m_paramInternalData->m_sliderEventHandlers.size();i++)
	{
		delete m_paramInternalData->m_sliderEventHandlers[i];
	}
	m_paramInternalData->m_sliderEventHandlers.clear();
	
	for (int i=0;i<m_paramInternalData->m_textLabels.size();i++)
	{
		delete m_paramInternalData->m_textLabels[i];
	}
	m_paramInternalData->m_textLabels.clear();
	
	m_gwenInternalData->m_curYposition = this->m_paramInternalData->m_savedYposition;
}