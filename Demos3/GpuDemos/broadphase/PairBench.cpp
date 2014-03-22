//those header files need to be at the top, because of conflict __global and STL
#include "../gwenUserInterface.h"
#include "../gwenInternalData.h"

#include "PairBench.h"
#include "OpenGLWindow/ShapeData.h"
#include "OpenGLWindow/GLInstancingRenderer.h"
#include "Bullet3Common/b3Quaternion.h"
#include "OpenGLWindow/b3gWindowInterface.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuGridBroadphase.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuParallelLinearBvhBroadphase.h"

#include "../GpuDemoInternalData.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "OpenGLWindow/OpenGLInclude.h"
#include "OpenGLWindow/GLInstanceRendererInternalData.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "../../../btgui/Timing/b3Quickprof.h"

#include <string.h>

#include "pairsKernel.h"

//we use an offset, just for testing to make sure there is no assumption in the broadphase that 'index' starts at 0
#define TEST_INDEX_OFFSET 1024

#ifdef B3_USE_MIDI
#include "../../../btgui/MidiTest/RtMidi.h"
bool chooseMidiPort( RtMidiIn *rtmidi )
{
	if (!rtmidi)
		return false;
	  /*

	std::cout << "\nWould you like to open a virtual input port? [y/N] ";

  std::string keyHit;
  std::getline( std::cin, keyHit );
  if ( keyHit == "y" ) {
    rtmidi->openVirtualPort();
    return true;
  }
  */

  std::string portName;
  unsigned int i = 0, nPorts = rtmidi->getPortCount();
  if ( nPorts == 0 ) {
    std::cout << "No input ports available!" << std::endl;
    return false;
  }

  if ( nPorts == 1 ) {
    std::cout << "\nOpening " << rtmidi->getPortName() << std::endl;
  }
  else {
    for ( i=0; i<nPorts; i++ ) {
      portName = rtmidi->getPortName(i);
      std::cout << "  Input port #" << i << ": " << portName << '\n';
    }

    do {
      std::cout << "\nChoose a port number: ";
      std::cin >> i;
    } while ( i >= nPorts );
  }

//  std::getline( std::cin, keyHit );  // used to clear out stdin
  rtmidi->openPort( i );

  return true;
}


#endif

static b3KeyboardCallback oldCallback = 0;

char* gPairBenchFileName = 0;
extern bool useShadowMap;
float maxExtents = -1e30f;
int largeCount = 0;

float timeStepPos = 0.000166666;
float mAmplitude = 251.f;
int dimensions[3]={10,10,10};//initialized with x_dim/y_dim/z_dim
const char* axisNames[3] = {"# x-axis","# y-axis","# z-axis"};
extern bool gReset;

static int curUseLargeAabbOption=0;
const char*	useLargeAabbOptions[] = 
{
	"NoLargeAabb",
	"UseLargeAabb",
};

struct BroadphaseEntry
{
	const char*							m_name;
	b3GpuBroadphaseInterface::CreateFunc*	m_createFunc;
};


static PairBench* sPairDemo = 0;

#define BP_COMBO_INDEX 123

static int curSelectedBroadphase = 0;
static BroadphaseEntry allBroadphases[]=
{
	{"Gpu Grid",b3GpuGridBroadphase::CreateFunc},
	{"Parallel Linear BVH",b3GpuParallelLinearBvhBroadphase::CreateFunc},
	{"CPU Brute Force",b3GpuSapBroadphase::CreateFuncBruteForceCpu},
	{"GPU Brute Force",b3GpuSapBroadphase::CreateFuncBruteForceGpu},
	{"GPU 1-SAP Original",b3GpuSapBroadphase::CreateFuncOriginal},
	{"GPU 1-SAP Barrier",b3GpuSapBroadphase::CreateFuncBarrier},
	{"GPU 1-SAP LDS",b3GpuSapBroadphase::CreateFuncLocalMemory}
};


struct	PairBenchInternalData
{
	b3GpuBroadphaseInterface*	m_broadphaseGPU;
	b3GpuBroadphaseInterface*	m_validationBroadphase;

	cl_kernel	m_moveObjectsKernel;
	cl_kernel	m_sineWaveKernel;
	cl_kernel	m_colorPairsKernel;
	cl_kernel	m_updateAabbSimple;

	GwenUserInterface*	m_gui;
	
	b3OpenCLArray<b3Vector4>*	m_instancePosOrnColor;
	b3OpenCLArray<float>*		m_bodyTimes;
	PairBenchInternalData()
		:m_broadphaseGPU(0),
		m_moveObjectsKernel(0),
		m_sineWaveKernel(0),
		m_colorPairsKernel(0),
		m_instancePosOrnColor(0),
		m_bodyTimes(0),
		m_updateAabbSimple(0)
	{
	}

	int m_oldYposition;

	b3AlignedObjectArray<Gwen::Controls::Base*> m_myControls;
#ifdef B3_USE_MIDI
	RtMidiIn*	m_midiIn;
#endif //B3_USE_MIDI
};


PairBench::PairBench()
:m_instancingRenderer(0),
m_window(0)
{
	m_data = new PairBenchInternalData;
	
	m_data->m_validationBroadphase = 0;
}
PairBench::~PairBench()
{
	delete m_data;
}







static void PairKeyboardCallback(int key, int state)
{
	if (key=='R' && state)
	{
		gReset = true;
	}
	
	//b3DefaultKeyboardCallback(key,state);
	oldCallback(key,state);
}

static inline float parseFloat(const char*& token)
{
  token += strspn(token, " \t");
  float f = (float)atof(token);
  token += strcspn(token, " \t\r");
  return f;
}

enum PairToggleButtons
{
	MY_RESET = 1024,
};


#define PAIRS_CL_PROGRAM_PATH "Demos3/GpuDemos/broadphase/pairsKernel.cl"




struct PairComboBoxHander   :public Gwen::Event::Handler
{
	
	int					m_buttonId;
	int					m_active;

	PairComboBoxHander  (int buttonId)
		:m_buttonId(buttonId),
		m_active(false)
	{
	}

	void onSelect( Gwen::Controls::Base* pControl )
	{
		if (m_active)
		{
			Gwen::Controls::ComboBox* but = (Gwen::Controls::ComboBox*) pControl;

			Gwen::String str = Gwen::Utility::UnicodeToString(	but->GetSelectedItem()->GetText());
		
			int numItems = sizeof(allBroadphases)/sizeof(BroadphaseEntry);

			//find selected item
			for (int i=0;i<numItems;i++)
			{
				if (strcmp(str.c_str(),allBroadphases[i].m_name)==0)
				{
					curSelectedBroadphase=i;
					sPairDemo->deleteBroadphase();
					sPairDemo->createBroadphase(dimensions[0],dimensions[1],dimensions[2]);
					break;
				}
			}
		}

	}

	void onSelectUseLargeAabb( Gwen::Controls::Base* pControl )
	{
		if (m_active)
		{
			Gwen::Controls::ComboBox* but = (Gwen::Controls::ComboBox*) pControl;

			Gwen::String str = Gwen::Utility::UnicodeToString(	but->GetSelectedItem()->GetText());
		
			int numItems = sizeof(useLargeAabbOptions)/sizeof(const char*);

			//find selected item
			for (int i=0;i<numItems;i++)
			{
				if (strcmp(str.c_str(),useLargeAabbOptions[i])==0)
				{
					curUseLargeAabbOption=i;
					sPairDemo->deleteBroadphase();
					sPairDemo->createBroadphase(dimensions[0],dimensions[1],dimensions[2]);
					break;
				}
			}

			

		}
	}

};


template<typename T>
struct MySliderEventHandler : public Gwen::Event::Handler
{
	Gwen::Controls::TextBox* m_label;
	Gwen::Controls::Slider* m_pSlider;
	char m_variableName[1024];
	T* m_targetValue;

	MySliderEventHandler(const char* varName, Gwen::Controls::TextBox* label, Gwen::Controls::Slider* pSlider,T* target)
		:m_label(label),
		m_pSlider(pSlider),
		m_targetValue(target)
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
		int val = int(v);//todo: specialize on template type
		char txt[1024];
		sprintf(txt,"%s : %d", m_variableName,val);
		m_label->SetText(txt);

	}
};

MySliderEventHandler<float>* test = 0;

#ifdef B3_USE_MIDI
//todo: create a mapping from midi channel to variable 'slider' or 'knob'


void PairMidiCallback( double deltatime, std::vector< unsigned char > *message, void *userData )
{
  unsigned int nBytes = message->size();
  if (nBytes==3)
  {
	  if (test && message->at(1)==16)
	  {
		  test->SetValue(message->at(2));
	  }
  }
}

#endif

void	PairBench::initPhysics(const ConstructionInfo& ci)
{
	dimensions[0] = ci.arraySizeX;
	dimensions[1] = ci.arraySizeY;
	dimensions[2] = ci.arraySizeZ;

#ifdef B3_USE_MIDI
	m_data->m_midiIn = new RtMidiIn();
	if (!chooseMidiPort(m_data->m_midiIn))
	{
		delete m_data->m_midiIn;
		m_data->m_midiIn = 0;
	} else
	{
		m_data->m_midiIn->setCallback( &PairMidiCallback,this );
		// Don't ignore sysex, timing, or active sensing messages.
		m_data->m_midiIn->ignoreTypes( false, false, false );
	}
#endif //B3_USE_MIDI
	m_instancingRenderer = ci.m_instancingRenderer;
	sPairDemo = this;
	useShadowMap = false;

	m_data->m_gui = ci.m_gui;

	//remember the old position in the GUI, to restore at exit
	
	GwenInternalData* data = m_data->m_gui->getInternalData();
	m_data->m_oldYposition = data->m_curYposition;



	data->m_curYposition+=40;
	
	{
	

		int startItem = curSelectedBroadphase;
		int numBroadphases = sizeof(allBroadphases)/sizeof(BroadphaseEntry);

		Gwen::Controls::ComboBox* combobox = new Gwen::Controls::ComboBox(data->m_demoPage->GetPage());
		PairComboBoxHander* handler = new PairComboBoxHander(555);
		m_data->m_myControls.push_back(combobox);
		
	
		combobox->onSelection.Add(handler,&PairComboBoxHander::onSelect);
		int ypos = data->m_curYposition;
		combobox->SetPos(10, ypos );
		combobox->SetWidth( 100 );
		
		
		for (int i=0;i<numBroadphases;i++)
		{
			Gwen::Controls::MenuItem* item = combobox->AddItem(Gwen::Utility::StringToUnicode(allBroadphases[i].m_name));
			if (i==startItem)
				combobox->OnItemSelected(item);
		}
		
		handler->m_active = true;

		data->m_curYposition+=22;
	}

	{
		int startItem = curUseLargeAabbOption;
		int numUseLargeAabb = sizeof(useLargeAabbOptions)/sizeof(const char*);

		Gwen::Controls::ComboBox* combobox = new Gwen::Controls::ComboBox(data->m_demoPage->GetPage());
		PairComboBoxHander* handler = new PairComboBoxHander(555);
		m_data->m_myControls.push_back(combobox);
		
	
		combobox->onSelection.Add(handler,&PairComboBoxHander::onSelectUseLargeAabb);
		int ypos = data->m_curYposition;
		combobox->SetPos(10, ypos );
		combobox->SetWidth( 100 );
		
		for (int i=0;i<numUseLargeAabb;i++)
		{
			Gwen::Controls::MenuItem* item = combobox->AddItem(Gwen::Utility::StringToUnicode(useLargeAabbOptions[i]));
			if (i==startItem)
				combobox->OnItemSelected(item);
		}
		
		handler->m_active = true;

		data->m_curYposition+=22;
	}
	data->m_curYposition+=22;

	if (1)
	for (int i=0;i<3;i++)
	{
		{
			Gwen::Controls::TextBox* label = new Gwen::Controls::TextBox(data->m_demoPage->GetPage());
			m_data->m_myControls.push_back(label);
			label->SetText( "Text Label" );
			label->SetPos( 10, 10 + 25 );
			label->SetWidth(100);
			label->SetPos(10,data->m_curYposition);
			data->m_curYposition+=22;
		
			Gwen::Controls::HorizontalSlider* pSlider = new Gwen::Controls::HorizontalSlider( data->m_demoPage->GetPage());
			m_data->m_myControls.push_back(pSlider);
			pSlider->SetPos( 10, data->m_curYposition );
			pSlider->SetSize( 100, 20 );
			pSlider->SetRange( 0, 100 );
			pSlider->SetValue( dimensions[i] );
			char labelName[1024];
			sprintf(labelName,"%s",axisNames[0]);
			MySliderEventHandler<int>* handler = new MySliderEventHandler<int>(labelName,label,pSlider,&dimensions[i]);
			pSlider->onValueChanged.Add( handler, &MySliderEventHandler<int>::SliderMoved );
			handler->SliderMoved(pSlider);
			float v = pSlider->GetValue();
			data->m_curYposition+=22;
		}
	}
	if (1)
	{
		{
			Gwen::Controls::TextBox* label = new Gwen::Controls::TextBox(data->m_demoPage->GetPage());
			m_data->m_myControls.push_back(label);
			const char* labelName = "Scale: ";
			label->SetText( labelName);
			label->SetPos( 10, 10 + 25 );
			label->SetWidth(100);
			label->SetPos(10,data->m_curYposition);
			data->m_curYposition+=22;
		
			Gwen::Controls::HorizontalSlider* pSlider = new Gwen::Controls::HorizontalSlider( data->m_demoPage->GetPage());
			m_data->m_myControls.push_back(pSlider);
			pSlider->SetPos( 10, data->m_curYposition );
			pSlider->SetSize( 100, 20 );
			pSlider->SetRange( 0, 300);
			pSlider->SetValue( mAmplitude );
			
			MySliderEventHandler<float>* handler = new MySliderEventHandler<float>(labelName,label,pSlider,&mAmplitude);
			test = handler;
			pSlider->onValueChanged.Add( handler, &MySliderEventHandler<float>::SliderMoved );
			handler->SliderMoved(pSlider);
			float v = pSlider->GetValue();
			data->m_curYposition+=22;
		}
	}
			//pSlider->onValueChanged.Add( this, &Slider::SliderMoved );


	data->m_curYposition+=22;

	/*m_data->m_gui->registerToggleButton(MY_RESET,"reset");
	sOldCallback = m_data->m_gui->getToggleButtonCallback();
	m_data->m_gui->setToggleButtonCallback(PairButtonCallback);
	*/
	
	int startItem = 0;
	//m_data->m_gui->registerComboBox(BP_COMBO_INDEX,numBroadphases,&mydemonames[0],startItem);

	//sOldComboCallback = m_data->m_gui->getComboBoxCallback();

	//m_data->m_gui->setComboBoxCallback(PairComboBoxCallback);


	initCL(ci.preferredOpenCLDeviceIndex,ci.preferredOpenCLPlatformIndex);

	if (m_clData->m_clContext)
	{
		cl_int err;
		cl_program pairBenchProg=b3OpenCLUtils::compileCLProgramFromString(m_clData->m_clContext,m_clData->m_clDevice,pairsKernelsCL,&err,"",PAIRS_CL_PROGRAM_PATH);
		int errNum=0;
		m_data->m_moveObjectsKernel = b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext,m_clData->m_clDevice,pairsKernelsCL,"moveObjectsKernel",&errNum,pairBenchProg);
		m_data->m_sineWaveKernel = b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext,m_clData->m_clDevice,pairsKernelsCL,"sineWaveKernel",&errNum,pairBenchProg);
		m_data->m_colorPairsKernel = b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext,m_clData->m_clDevice,pairsKernelsCL,"colorPairsKernel2",&errNum,pairBenchProg);
		m_data->m_updateAabbSimple = b3OpenCLUtils::compileCLKernelFromString(m_clData->m_clContext,m_clData->m_clDevice,pairsKernelsCL,"updateAabbSimple",&errNum,pairBenchProg);
		
		//Method for validating the overlapping pairs requires that the
		//reference broadphase does not maintain internal state aside from AABB data.
		//That is, overwriting the AABB state in the broadphase using
		//	b3GpuBroadphaseInterface::getAllAabbsGPU(),
		//	b3GpuBroadphaseInterface::getSmallAabbIndicesGPU(), and
		//	b3GpuBroadphaseInterface::getLargeAabbIndicesGPU()
		//and then calling b3GpuBroadphaseInterface::calculateOverlappingPairs() should 
		//always produce the same result regardless of the current state of the broadphase.
		m_data->m_validationBroadphase = b3GpuParallelLinearBvhBroadphase::CreateFunc(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue);
	}

	if (ci.m_window)
	{
		m_window = ci.m_window;
		oldCallback = ci.m_window->getKeyboardCallback();
		ci.m_window->setKeyboardCallback(PairKeyboardCallback);

	}

	

#ifndef B3_NO_PROFILE
	b3ProfileManager::CleanupMemory();
#endif //B3_NO_PROFILE

	createBroadphase(ci.arraySizeX,ci.arraySizeY,ci.arraySizeZ);

}

void	PairBench::createBroadphase(int arraySizeX, int arraySizeY, int arraySizeZ)
{

	
	m_data->m_broadphaseGPU = (allBroadphases[curSelectedBroadphase].m_createFunc)(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue);
	
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_vertices)/sizeof(int);
	int shapeId = m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int group=1;
	int mask=1;
	int index=TEST_INDEX_OFFSET;

	
	if (gPairBenchFileName)
	{
		
		
		//char* fileName = "32006GPUAABBs.txt";
		char relativeFileName[1024];
		const char* prefix[]={"./data/","../data/","../../data/","../../../data/","../../../../data/"};
		int prefixIndex=-1;
		{
	
			int numPrefixes = sizeof(prefix)/sizeof(char*);

			for (int i=0;i<numPrefixes;i++)
			{
				FILE* f = 0;
				sprintf(relativeFileName,"%s%s",prefix[i],gPairBenchFileName);
				f = fopen(relativeFileName,"rb");
				if (f)
				{
					fseek( f, 0L, SEEK_END );
					int size = ftell( f);
					rewind( f);
					char* buf = (char*)malloc(size);
					
					int actualReadBytes =0;

					while (actualReadBytes<size)
					{	int left =  size-actualReadBytes;
						int chunk = 8192;
						int numPlannedRead= left < chunk? left : chunk;
						actualReadBytes += fread(&buf[actualReadBytes],1,numPlannedRead,f);
					}

					fclose(f);
					

					char pattern[1024];
					pattern[0] = 0x0a;
					pattern[1] = 0;			
					size_t const patlen = strlen(pattern);
  					size_t patcnt = 0;
					char * oriptr;
					char * patloc;

					
					for (oriptr = buf; patloc = strstr(oriptr, pattern); oriptr = patloc + patlen)
					{
						if (patloc)
						{
							*patloc=0;
							const char* token = oriptr;

							b3Vector3 aabbMin;
							b3Vector3 aabbMax;

							aabbMin.x = parseFloat(token);
							aabbMin.y = parseFloat(token);
							aabbMin.z = parseFloat(token);
							aabbMin.w = 0.f;
							aabbMax.x = parseFloat(token);
							aabbMax.y = parseFloat(token);
							aabbMax.z = parseFloat(token);
							aabbMax.w = 0.f;

							aabbMin*=0.1;
							aabbMax*=0.1;

							b3Vector3 extents = aabbMax-aabbMin;
							
							//printf("%s\n", oriptr);

							b3Vector3 position=0.5*(aabbMax+aabbMin);
							b3Quaternion orn(0,0,0,1);
				
							
							b3Vector4 scaling = b3MakeVector4(0.5*extents.x,0.5*extents.y,0.5*extents.z,1);//b3MakeVector4(1,1,1,1);
							
							
							float l = extents.length();
							if (l>500)
							{
								b3Vector4 color=b3MakeVector4(0,1,0,0.1);
								int id = m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
								m_data->m_broadphaseGPU->createLargeProxy(aabbMin,aabbMax,index,group,mask);
							} else
							{
								b3Vector4 color=b3MakeVector4(1,0,0,1);
								int id = m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
								m_data->m_broadphaseGPU->createProxy(aabbMin,aabbMax,index,group,mask);
									index++;
							}

							
						

							patcnt++;
						}
					}
					prefixIndex = i;
					break;
				}
			
			}
			
			if (prefixIndex<0)
			{
				b3Printf("Cannot find %s\n",gPairBenchFileName);
			}
			
		}

		
	}
	else
	{
		for (int i=0;i<arraySizeX;i++)
		{
			for (int j=0;j<arraySizeY;j++)
			{
				for (int k=0;k<arraySizeZ;k++)
				{
					b3Vector3 position=b3MakeVector3(k*3,i*3,j*3);
					b3Quaternion orn(0,0,0,1);
				
					b3Vector4 color=b3MakeVector4(0,1,0,1);
					b3Vector4 scaling=b3MakeVector4(1,1,1,1);
					bool large = false;

					if (curUseLargeAabbOption)
					{
						if (i==0 && j==0 && k==0)
						{
							large = true;
							scaling[0] = 1000;
							scaling[1] = 1000;
							scaling[2] = 1000;
						}
					}
					/*if (j==0)
					{
						large=true;
						scaling[1] = 10000;
					}
					if (k==0)
					{
						large=true;
						scaling[2] = 10000;
					}*/


					int id = m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
					
					
					b3Vector3 aabbMin = position-scaling;
					b3Vector3 aabbMax = position+scaling;
				
					if (large)
					{
						m_data->m_broadphaseGPU->createLargeProxy(aabbMin,aabbMax,index,group,mask);
						
					} else
					{
						m_data->m_broadphaseGPU->createProxy(aabbMin,aabbMax,index,group,mask);
					}
					index++;
				}
			}
		}
	}
	
	float camPos[4]={15.5,12.5,15.5,0};
	m_instancingRenderer->setCameraTargetPosition(camPos);
	if (gPairBenchFileName)
	{
		m_instancingRenderer->setCameraDistance(830);
	} else
	{
		m_instancingRenderer->setCameraDistance(130);
	}

	m_instancingRenderer->writeTransforms();
	m_data->m_broadphaseGPU->writeAabbsToGpu();

}

void	PairBench::deleteBroadphase()
{
	delete m_data->m_broadphaseGPU;
	m_data->m_broadphaseGPU = 0;
	delete m_data->m_instancePosOrnColor;
	m_data->m_instancePosOrnColor = 0;
	delete m_data->m_bodyTimes;
	m_data->m_bodyTimes = 0;

	m_data->m_broadphaseGPU = 0;
	m_instancingRenderer->removeAllInstances();
}

void	PairBench::exitPhysics()
{
	if(m_data->m_validationBroadphase)
	{
		delete m_data->m_validationBroadphase;
		m_data->m_validationBroadphase = 0;
	}
	
#ifdef B3_USE_MIDI
	if (m_data->m_midiIn)
	{
		delete m_data->m_midiIn;
		m_data->m_midiIn = 0;
	}
#endif //B3_USE_MIDI
	m_data->m_gui->getInternalData()->m_curYposition = m_data->m_oldYposition;
	
	for (int i=0;i<m_data->m_myControls.size();i++)
	{
		delete m_data->m_myControls[i];
	}
	
	sPairDemo = 0;

	m_window->setKeyboardCallback(oldCallback);
	exitCL();

}


void PairBench::renderScene()
{
	m_instancingRenderer->renderScene();
}

struct OverlappingPairSortPredicate 
{
	inline bool operator() (const b3Int4& a, const b3Int4& b) const 
	{
		if(a.x != b.x) return (a.x < b.x);
		if(a.y != b.y) return (a.y < b.y);
		if(a.z != b.z) return (a.z < b.z);
		return (a.w < b.w);
	}
};

void PairBench::clientMoveAndDisplay()
{
	//color all objects blue

	bool animate=true;
	int numObjects= 0;
	{
		B3_PROFILE("Num Objects");
		numObjects = m_instancingRenderer->getInternalData()->m_totalNumInstances;
	}
	b3Vector4* positions = 0;
	if (numObjects)
	{
		B3_PROFILE("Sync");
		GLuint vbo = m_instancingRenderer->getInternalData()->m_vbo;
		
			

		int arraySizeInBytes  = numObjects * (3)*sizeof(b3Vector4);

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		cl_bool blocking=  CL_TRUE;
		char* hostPtr= 0;
		{
			B3_PROFILE("glMapBufferRange");
			hostPtr = 	(char*)glMapBufferRange( GL_ARRAY_BUFFER,m_instancingRenderer->getMaxShapeCapacity(),arraySizeInBytes, GL_MAP_WRITE_BIT|GL_MAP_READ_BIT );//GL_READ_WRITE);//GL_WRITE_ONLY
		}
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		positions = (b3Vector4*)hostPtr;
		
		if (m_data->m_instancePosOrnColor && m_data->m_instancePosOrnColor->size() != 3*numObjects)
		{
			delete m_data->m_instancePosOrnColor;
			m_data->m_instancePosOrnColor=0;
		}
		if (!m_data->m_instancePosOrnColor)
		{
			m_data->m_instancePosOrnColor = new b3OpenCLArray<b3Vector4>(m_clData->m_clContext,m_clData->m_clQueue);
			m_data->m_instancePosOrnColor->resize(3*numObjects);
			m_data->m_instancePosOrnColor->copyFromHostPointer(positions,3*numObjects,0);
			m_data->m_bodyTimes = new b3OpenCLArray<float>(m_clData->m_clContext,m_clData->m_clQueue);
			m_data->m_bodyTimes ->resize(numObjects);
			b3AlignedObjectArray<float> tmp;
			tmp.resize(numObjects);
			for (int i=0;i<numObjects;i++)
			{
				tmp[i] = float(i)*(1024.f/numObjects);
			}
			m_data->m_bodyTimes->copyFromHost(tmp);
		}

		if (!gPairBenchFileName)
		{
			if (1)
			{
				if (1)
				{
			
					b3LauncherCL launcher(m_clData->m_clQueue, m_data->m_sineWaveKernel,"m_sineWaveKernel");
					launcher.setBuffer(m_data->m_instancePosOrnColor->getBufferCL() );
					launcher.setBuffer(m_data->m_bodyTimes->getBufferCL() );
				
					launcher.setConst(timeStepPos);
					launcher.setConst(mAmplitude);
					launcher.setConst( numObjects);
					launcher.launch1D( numObjects);
					clFinish(m_clData->m_clQueue);
				}
				else
				{
			
					b3LauncherCL launcher(m_clData->m_clQueue, m_data->m_moveObjectsKernel,"m_moveObjectsKernel");
					launcher.setBuffer(m_data->m_instancePosOrnColor->getBufferCL() );
					launcher.setConst( numObjects);
					launcher.launch1D( numObjects);
					clFinish(m_clData->m_clQueue);
				}
			}
		}
	}

	bool updateOnGpu=true;

	if (1)
	{
		if (updateOnGpu)
		{
			B3_PROFILE("updateOnGpu");
			b3LauncherCL launcher(m_clData->m_clQueue, m_data->m_updateAabbSimple,"m_updateAabbSimple");
				launcher.setBuffer(m_data->m_instancePosOrnColor->getBufferCL() );
				launcher.setConst( numObjects);
				launcher.setBuffer(m_data->m_broadphaseGPU->getAabbBufferWS());
				launcher.launch1D( numObjects);
				clFinish(m_clData->m_clQueue);
		
		} else
		{
			B3_PROFILE("updateOnCpu");
			if (!gPairBenchFileName)
			{
			int allAabbs = m_data->m_broadphaseGPU->getAllAabbsCPU().size();
			

			b3AlignedObjectArray<b3Vector4> posOrnColorsCpu;
			if (m_data->m_instancePosOrnColor)
				m_data->m_instancePosOrnColor->copyToHost(posOrnColorsCpu);
		
		
		
			for (int nodeId=0;nodeId<numObjects;nodeId++)
			{
				{
					b3Vector3 position = posOrnColorsCpu[nodeId];
					
					b3SapAabb orgAabb = m_data->m_broadphaseGPU->getAllAabbsCPU()[nodeId];
					b3Vector3 halfExtents = 0.5f*(orgAabb.m_maxVec-orgAabb.m_minVec);
					int orgNodeIndex = orgAabb.m_minIndices[3];
					int orgBroadphaseIndex = orgAabb.m_signedMaxIndices[3];

					m_data->m_broadphaseGPU->getAllAabbsCPU()[nodeId].m_minVec = position-halfExtents;
					m_data->m_broadphaseGPU->getAllAabbsCPU()[nodeId].m_minIndices[3] = orgNodeIndex;
					m_data->m_broadphaseGPU->getAllAabbsCPU()[nodeId].m_maxVec = position+halfExtents;
					m_data->m_broadphaseGPU->getAllAabbsCPU()[nodeId].m_signedMaxIndices[3]= orgBroadphaseIndex;		
				}
			}
			m_data->m_broadphaseGPU->writeAabbsToGpu();
			}

		
		
		}
	}
	
	int prealloc = 3*1024*1024;
	int maxOverlap = b3Min(prealloc,16*numObjects);
	
	unsigned long dt = 0;
	if (numObjects)
	{
		b3Clock cl;
		dt = cl.getTimeMicroseconds();
		B3_PROFILE("calculateOverlappingPairs");
		int sz = sizeof(b3Int4)*64*numObjects;


		m_data->m_broadphaseGPU->calculateOverlappingPairs(maxOverlap);
		int numPairs = m_data->m_broadphaseGPU->getNumOverlap();
		//printf("numPairs = %d\n", numPairs);
		dt = cl.getTimeMicroseconds()-dt;
		
	}
	
	const bool VALIDATE_BROADPHASE = false;	//Check that overlapping pairs of 2 broadphases are the same
	if(numObjects && VALIDATE_BROADPHASE)
	{
		B3_PROFILE("validate broadphases");
			
		{
			B3_PROFILE("calculateOverlappingPairs m_validationBroadphase");
			//m_data->m_validationBroadphase->getAllAabbsCPU() = m_data->m_broadphaseGPU->getAllAabbsCPU();
			
			m_data->m_validationBroadphase->getAllAabbsGPU().copyFromOpenCLArray( m_data->m_broadphaseGPU->getAllAabbsGPU() );
			m_data->m_validationBroadphase->getSmallAabbIndicesGPU().copyFromOpenCLArray( m_data->m_broadphaseGPU->getSmallAabbIndicesGPU() );
			m_data->m_validationBroadphase->getLargeAabbIndicesGPU().copyFromOpenCLArray( m_data->m_broadphaseGPU->getLargeAabbIndicesGPU() );
			
			m_data->m_validationBroadphase->calculateOverlappingPairs(maxOverlap);
		}
		
		static b3AlignedObjectArray<b3Int4> overlappingPairs;
		static b3AlignedObjectArray<b3Int4> overlappingPairsReference;
		m_data->m_broadphaseGPU->getOverlappingPairsGPU().copyToHost(overlappingPairs);
		m_data->m_validationBroadphase->getOverlappingPairsGPU().copyToHost(overlappingPairsReference);
		
		//Reorder pairs so that (pair.x < pair.y) is always true
		{
			B3_PROFILE("reorder pairs");
			
			for(int i = 0; i < overlappingPairs.size(); ++i)
			{
				b3Int4 pair = overlappingPairs[i];
				if(pair.x > pair.y)
				{
					b3Swap(pair.x, pair.y);
					b3Swap(pair.z, pair.w);
					overlappingPairs[i] = pair;
				}
			}
			for(int i = 0; i < overlappingPairsReference.size(); ++i)
			{
				b3Int4 pair = overlappingPairsReference[i];
				if(pair.x > pair.y)
				{
					b3Swap(pair.x, pair.y);
					b3Swap(pair.z, pair.w);
					overlappingPairsReference[i] = pair;
				}
			}
		}
		
		//
		{
			B3_PROFILE("Sort overlapping pairs from most to least significant bit");
			
			overlappingPairs.quickSort( OverlappingPairSortPredicate() );
			overlappingPairsReference.quickSort( OverlappingPairSortPredicate() );
		}
		
		//Compare
		{
			B3_PROFILE("compare pairs");
			
			int numPairs = overlappingPairs.size();
			int numPairsReference = overlappingPairsReference.size();
			
			bool success = true;
			
			if(numPairs == numPairsReference)
			{
				for(int i = 0; i < numPairsReference; ++i)
				{
					const b3Int4& pairA = overlappingPairs[i];
					const b3Int4& pairB = overlappingPairsReference[i];
					if(  pairA.x != pairB.x
					  || pairA.y != pairB.y
					  || pairA.z != pairB.z
					  || pairA.w != pairB.w ) 
					{
						b3Error("Error: one or more overlappingPairs differs from reference.\n");
						success = false;
						break;
					}
				}
			}
			else 
			{
				b3Error("Error: numPairs %d != numPairsReference %d \n", numPairs, numPairsReference);
				success = false;
			}
			
			printf("Broadphase validation: %d \n", success);
		}
	}
			
	if (m_data->m_gui)
	{
		B3_PROFILE("update Gui");
		int allAabbs = m_data->m_broadphaseGPU->getAllAabbsCPU().size();
		int numOverlap = m_data->m_broadphaseGPU->getNumOverlap();

		float time = dt/1000.f;
		//printf("time = %f\n", time);

		char msg[1024];
		sprintf(msg,"#objects = %d, #overlapping pairs = %d, time = %f ms", allAabbs,numOverlap,time );
		//printf("msg=%s\n",msg);
		m_data->m_gui->setStatusBarMessage(msg,true);
	}


	if (numObjects)
	{
		B3_PROFILE("animate");
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		//color overlapping objects in red

		
		if (m_data->m_broadphaseGPU->getNumOverlap())
		{
			bool colorPairsOnHost = false;
			if (colorPairsOnHost )
			{

			} else
			{
				int numPairs = m_data->m_broadphaseGPU->getNumOverlap();
				cl_mem pairBuf = m_data->m_broadphaseGPU->getOverlappingPairBuffer();

				b3LauncherCL launcher(m_clData->m_clQueue, m_data->m_colorPairsKernel,"m_colorPairsKernel");
				launcher.setBuffer(m_data->m_instancePosOrnColor->getBufferCL() );
				launcher.setConst( numObjects);
				launcher.setBuffer( pairBuf);
				int indexOffset = TEST_INDEX_OFFSET;
				launcher.setConst(indexOffset);
				launcher.setConst( numPairs);
				launcher.launch1D( numPairs);
				clFinish(m_clData->m_clQueue);
			}
		}

		if (numObjects)
		{
			m_data->m_instancePosOrnColor->copyToHostPointer(positions,3*numObjects,0);
		}

		glUnmapBuffer( GL_ARRAY_BUFFER);
		err = glGetError();
		assert(err==GL_NO_ERROR);
	}

}
