

#ifdef __APPLE__
#include "OpenGLWindow/MacOpenGLWindow.h"
#elif defined _WIN32
#include "OpenGLWindow/Win32OpenGLWindow.h"
#elif defined __linux
#include "OpenGLWindow/X11OpenGLWindow.h"
#endif

#include "OpenGLWindow/GLPrimitiveRenderer.h"
#include "OpenGLWindow/GLInstancingRenderer.h"
//#include "OpenGL3CoreRenderer.h"
//#include "b3GpuDynamicsWorld.h"
#include <assert.h>
#include <string.h>
#include "OpenGLWindow/fontstash.h"
#include "OpenGLWindow/opengl_fontstashcallbacks.h"

#include "OpenGLWindow/GwenOpenGL3CoreRenderer.h"
#include "../btgui/Timing/b3Quickprof.h"

#include "Gwen/Gwen.h"
#include "Gwen/Controls/Button.h"
#include "Gwen/Skins/Simple.h"
#include "Gwen/Renderers/OpenGL_DebugFont.h"
#include "Gwen/Controls/MenuStrip.h"
#include "Gwen/Controls/WindowControl.h"
#include "Gwen/Controls/ListBox.h"
#include "Gwen/Controls/VerticalSlider.h"
#include "Gwen/Controls/HorizontalSlider.h"
#include "Gwen/Controls/GroupBox.h"
#include "Gwen/Controls/CheckBox.h"
#include "Gwen/Controls/TreeControl.h"

b3gDefaultOpenGLWindow* window=0;

GLPrimitiveRenderer* primRenderer = 0;
GwenOpenGL3CoreRenderer* pRenderer = 0;

Gwen::Skin::Simple skin;
Gwen::Controls::Canvas* pCanvas =0;
class MyProfileWindow* prof = 0;

int sGlutScreenWidth = 640;
int sGlutScreenHeight = 480;
int sLastmousepos[2] = {0,0};

#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"

cl_context			g_cxMainContext;
cl_command_queue	g_cqCommandQue;



int droidRegular=0;//, droidItalic, droidBold, droidJapanese, dejavu;
extern char OpenSansData[];
sth_stash* stash=0;

sth_stash* initFont(GLPrimitiveRenderer* primRender)
{
	GLint err;

		struct sth_stash* stash = 0;
	int datasize;
	
	float sx,sy,dx,dy,lh;
	GLuint texture;

	OpenGL2RenderCallbacks* renderCallbacks = new OpenGL2RenderCallbacks(primRender);

	stash = sth_create(512,512,renderCallbacks);//256,256);//,1024);//512,512);
    err = glGetError();
    assert(err==GL_NO_ERROR);

	if (!stash)
	{
		fprintf(stderr, "Could not create stash.\n");
		return 0;
	}
	char* data2 = OpenSansData;
	unsigned char* data = (unsigned char*) data2;
	if (!(droidRegular = sth_add_font_from_memory(stash, data)))
	{
		b3Error("error!\n");
	}

    err = glGetError();
    assert(err==GL_NO_ERROR);

	return stash;
}



class MyProfileWindow : public Gwen::Controls::WindowControl
{
	//		Gwen::Controls::TabControl*	m_TabControl;
	Gwen::Controls::ListBox*	m_TextOutput;
	unsigned int				m_iFrames;
	float						m_fLastSecond;

	Gwen::Controls::TreeNode* m_node;
	Gwen::Controls::TreeControl* m_ctrl;

protected:

	void onButtonA( Gwen::Controls::Base* pControl )
	{
	//		OpenTissue::glut::toggleIdle();
	}

	void SliderMoved(Gwen::Controls::Base* pControl )
	{
		Gwen::Controls::Slider* pSlider = (Gwen::Controls::Slider*)pControl;
		//this->m_app->scaleYoungModulus(pSlider->GetValue());
		//	printf("Slider Value: %.2f", pSlider->GetValue() );
	}


	void	OnCheckChangedStiffnessWarping (Gwen::Controls::Base* pControl)
	{
		Gwen::Controls::CheckBox* labeled = (Gwen::Controls::CheckBox* )pControl;
		bool checked = labeled->IsChecked();
		//m_app->m_stiffness_warp_on  = checked;
	}
public:

	void MenuItemSelect(Gwen::Controls::Base* pControl)
	{
		if (Hidden())
		{
			SetHidden(false);
		} else
		{
			SetHidden(true);
		}
	}


	MyProfileWindow (	Gwen::Controls::Base* pParent)
		: Gwen::Controls::WindowControl( pParent )
	{
		
		SetTitle( L"OpenCL info" );

		SetSize( 550, 350 );
		this->SetPos(0,40);

//		this->Dock( Gwen::Pos::Bottom);

		

		{
			m_ctrl = new Gwen::Controls::TreeControl( this );
	
			int numPlatforms = b3OpenCLUtils::getNumPlatforms();
			for (int i=0;i<numPlatforms;i++)
			{
				cl_platform_id platform = b3OpenCLUtils::getPlatform(i);
				b3OpenCLPlatformInfo platformInfo;
				b3OpenCLUtils::getPlatformInfo(platform, &platformInfo);
				cl_device_type deviceType = CL_DEVICE_TYPE_ALL;
				cl_int errNum;
				cl_context context = b3OpenCLUtils::createContextFromPlatform(platform,deviceType,&errNum);
				if (context)
				{
				
				Gwen::UnicodeString strIn = Gwen::Utility::StringToUnicode(platformInfo.m_platformName);
				Gwen::UnicodeString txt = Gwen::Utility::Format( L"Platform %d (",i)+strIn + Gwen::Utility::Format(L")");
				
				m_node = m_ctrl->AddNode(txt);
				int numDev = b3OpenCLUtils::getNumDevices(context);
				for (int j=0;j<numDev;j++)
				{
					Gwen::UnicodeString txt = Gwen::Utility::Format( L"Device %d", j);
					Gwen::Controls::TreeNode* deviceNode = m_node->AddNode( txt );

					cl_device_id device = b3OpenCLUtils::getDevice(context,j);
					b3OpenCLDeviceInfo info;
					b3OpenCLUtils::getDeviceInfo(device,&info);

					Gwen::Controls::TreeNode* node;
					Gwen::UnicodeString strIn;


					switch (info.m_deviceType)
					{
					case CL_DEVICE_TYPE_CPU:
						{
							txt = Gwen::Utility::Format( L"CL_DEVICE_TYPE_CPU");
							node = deviceNode->AddNode( txt );
							break;
						}
					case CL_DEVICE_TYPE_GPU:
						{
							txt = Gwen::Utility::Format( L"CL_DEVICE_TYPE_GPU");
							node = deviceNode->AddNode( txt );
							break;
						}
					case CL_DEVICE_TYPE_ACCELERATOR:
						{
							txt = Gwen::Utility::Format( L"CL_DEVICE_TYPE_ACCELERATOR");
							node = deviceNode->AddNode( txt );
							break;
						}
						

					default:
						{
							txt = Gwen::Utility::Format( L"Unknown device type");
							node = deviceNode->AddNode( txt );
						}
					}

					strIn = Gwen::Utility::StringToUnicode(info.m_deviceName);
					txt = Gwen::Utility::Format( L"CL_DEVICE_NAME: ")+strIn;
					node = deviceNode->AddNode( txt );

					strIn = Gwen::Utility::StringToUnicode(info.m_deviceVendor);
					txt = Gwen::Utility::Format( L"CL_DEVICE_VENDOR: ")+strIn;
					node = deviceNode->AddNode( txt );
					
					strIn = Gwen::Utility::StringToUnicode(info.m_driverVersion);
					txt = Gwen::Utility::Format( L"CL_DRIVER_VERSION: ")+strIn;
					node = deviceNode->AddNode( txt );
					
					txt = Gwen::Utility::Format( L"CL_DEVICE_MAX_COMPUTE_UNITS:%u",info.m_computeUnits);
					node = deviceNode->AddNode( txt );

					txt = Gwen::Utility::Format( L"CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS:%u",info.m_workitemDims);
					node = deviceNode->AddNode( txt );

					txt = Gwen::Utility::Format( L"CL_DEVICE_MAX_WORK_ITEM_SIZES:%u / %u / %u",info.m_workItemSize[0], info.m_workItemSize[1], info.m_workItemSize[2]);
					node = deviceNode->AddNode( txt );

					txt = Gwen::Utility::Format( L"CL_DEVICE_MAX_WORK_GROUP_SIZE:%u",info.m_workgroupSize);
					node = deviceNode->AddNode( txt );

					txt = Gwen::Utility::Format( L"CL_DEVICE_MAX_CLOCK_FREQUENCY:%u MHz",info.m_clockFrequency);
					node = deviceNode->AddNode( txt );

					txt = Gwen::Utility::Format( L"CL_DEVICE_ADDRESS_BITS:%u",info.m_addressBits);
					node = deviceNode->AddNode( txt );

					txt = Gwen::Utility::Format( L"CL_DEVICE_MAX_MEM_ALLOC_SIZE:%u MByte",(unsigned int)(info.m_maxMemAllocSize/ (1024 * 1024)));
					node = deviceNode->AddNode( txt );

					txt = Gwen::Utility::Format( L"CL_DEVICE_GLOBAL_MEM_SIZE:%u MByte",(unsigned int)(info.m_globalMemSize/ (1024 * 1024)));
					node = deviceNode->AddNode( txt );

					txt = Gwen::Utility::Format( L"CL_DEVICE_ERROR_CORRECTION_SUPPORT:%s",info.m_errorCorrectionSupport== CL_TRUE ? L"yes" : L"no");
					node = deviceNode->AddNode( txt );


					txt = Gwen::Utility::Format( L"CL_DEVICE_LOCAL_MEM_TYPE:%s",info.m_localMemType == 1 ? L"local" : L"global");
					node = deviceNode->AddNode( txt );

					txt = Gwen::Utility::Format( L"CL_DEVICE_LOCAL_MEM_SIZE:%u KByte",(unsigned int)(info.m_localMemSize / 1024));
					node = deviceNode->AddNode( txt );

					txt = Gwen::Utility::Format( L"CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE:%u KByte",(unsigned int)(info.m_constantBufferSize / 1024));
					node = deviceNode->AddNode( txt );

					if( info.m_queueProperties  & CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE )
					{
						txt = Gwen::Utility::Format( L"CL_DEVICE_QUEUE_PROPERTIES:\tCL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE");
						node = deviceNode->AddNode( txt );
					}
					if( info.m_queueProperties & CL_QUEUE_PROFILING_ENABLE )
					{
						txt = Gwen::Utility::Format( L"CL_DEVICE_QUEUE_PROPERTIES:\tCL_QUEUE_PROFILING_ENABLE");
						node = deviceNode->AddNode( txt );
					}

					txt = Gwen::Utility::Format( L"CL_DEVICE_IMAGE_SUPPORT:%u",info.m_imageSupport);
					node = deviceNode->AddNode( txt );

					txt = Gwen::Utility::Format( L"CL_DEVICE_MAX_READ_IMAGE_ARGS:%u",info.m_maxReadImageArgs);
					node = deviceNode->AddNode( txt );


					txt = Gwen::Utility::Format( L"CL_DEVICE_MAX_WRITE_IMAGE_ARGS:%u",info.m_maxWriteImageArgs);
					node = deviceNode->AddNode( txt );

					txt = Gwen::Utility::Format( L"CL_DEVICE_EXTENSIONS");
					Gwen::Controls::TreeNode* extensionNode = deviceNode->AddNode( txt );

					if (info.m_deviceExtensions)
					{
						Gwen::Utility::Strings::List outbits;

						Gwen::String str(info.m_deviceExtensions);

						Gwen::String sep(" ");
						Gwen::Utility::Strings::Split(str,sep, outbits);
						
						

						for (int k=0;k<outbits.size();k++)
						{
							if (outbits.at(k).size())
							{
								txt = Gwen::Utility::StringToUnicode(outbits.at(k));
								node = extensionNode->AddNode( txt );
							}
						}
					}
				}
/*	
	
	
	
	printf("\n  CL_DEVICE_IMAGE <dim>"); 
	printf("\t2D_MAX_WIDTH %u\n", info.m_image2dMaxWidth);
	printf("\t2D_MAX_HEIGHT %u\n", info.m_image2dMaxHeight);
	printf("\t3D_MAX_WIDTH %u\n", info.m_image3dMaxWidth);
	printf("\t3D_MAX_HEIGHT %u\n", info.m_image3dMaxHeight);
	printf("\t3D_MAX_DEPTH %u\n", info.m_image3dMaxDepth);
	
	printf("  CL_DEVICE_PREFERRED_VECTOR_WIDTH_<t>"); 
	printf("CHAR %u, SHORT %u, INT %u,LONG %u, FLOAT %u, DOUBLE %u\n\n\n", 
		info.m_vecWidthChar, info.m_vecWidthShort, info.m_vecWidthInt, info.m_vecWidthLong,info.m_vecWidthFloat, info.m_vecWidthDouble); 
*/

				}
			}

			m_ctrl->ExpandAll();
			m_ctrl->SetBounds( this->GetInnerBounds().x,this->GetInnerBounds().y,this->GetInnerBounds().w,this->GetInnerBounds().h);
		}

	}


	
	void	UpdateText()
	{
		static bool update=true;
		m_ctrl->SetBounds(0,0,this->GetInnerBounds().w,this->GetInnerBounds().h);
	}
	
};


struct MyTestMenuBar : public Gwen::Controls::MenuStrip
{
	MyProfileWindow* m_profileWindow;

	void MenuItemSelect(Gwen::Controls::Base* pControl)
	{
	}

	MyTestMenuBar(Gwen::Controls::Base* pParent, MyProfileWindow* prof)
		:Gwen::Controls::MenuStrip(pParent),
		m_profileWindow(prof)
	{
		{
			Gwen::Controls::MenuItem* pRoot = AddItem( L"File" );
		
			pRoot = AddItem( L"View" );
			pRoot->GetMenu()->AddItem( L"Platforms",prof,(Gwen::Event::Handler::Function)&MyProfileWindow::MenuItemSelect);

		}
	}

};


float retinaScale = 1.f;

void	setupGUI(int width, int height)
{

	primRenderer = new GLPrimitiveRenderer(width,height);
	stash = initFont(primRenderer);
	pRenderer = new GwenOpenGL3CoreRenderer(primRenderer,stash,width,height,retinaScale);

	

	skin.SetRender( pRenderer );

	pCanvas = new Gwen::Controls::Canvas( &skin );
	pCanvas->SetSize( width,height);
	pCanvas->SetDrawBackground( false);
	pCanvas->SetBackgroundColor( Gwen::Color( 150, 170, 170, 255 ) );

	//MyWindow* window = new MyWindow(pCanvas);
	prof = new MyProfileWindow(pCanvas);
	prof->UpdateText();

	MyTestMenuBar* menubar = new MyTestMenuBar(pCanvas, prof);


	
}



static void MyResizeCallback(float w, float h)
{
	sGlutScreenWidth = w;
	sGlutScreenHeight = h;
	if (pRenderer)
		pRenderer->resize(w,h);
	if (primRenderer)
		primRenderer->setScreenSize(w,h);
	if (pCanvas)
	{
		pCanvas->SetSize(w,h);
	}
}



static void MouseButtonCallback(int button, int state, float x, float y)
{
	if (pCanvas)
	{
		bool handled = false;
		if (pCanvas)
		{
			handled = pCanvas->InputMouseMoved(x,y,x, y);

			if (button>=0)
			{
				handled = pCanvas->InputMouseButton(button,state);
				if (handled)
				{
					//if (!state)
					//	return false;
				}
			}
		}

		if (handled)
		{
			sLastmousepos[0]	=	x;
			sLastmousepos[1]	=	y;
		} else
		{
			b3DefaultMouseButtonCallback(button,state,x,y);
		}
	}

}


static void	MouseMoveCallback(float x,float y)
{

	bool handled  = false;

	static int m_lastmousepos[2] = {0,0};
	static bool isInitialized = false;
	if (pCanvas)
	{
		if (!isInitialized)
		{
			isInitialized = true;
			m_lastmousepos[0] = x+1;
			m_lastmousepos[1] = y+1;
		}
		handled = pCanvas->InputMouseMoved(x,y,m_lastmousepos[0],m_lastmousepos[1]);
	}
}




int main(int argc, char* argv[])
{
	
	window = new b3gDefaultOpenGLWindow();

	b3gWindowConstructionInfo wci(sGlutScreenWidth,sGlutScreenHeight);

	window->createWindow(wci);
	window->setResizeCallback(MyResizeCallback);
	window->setMouseMoveCallback(MouseMoveCallback);
	window->setMouseButtonCallback(MouseButtonCallback);
	//window->setKeyboardCallback(MyKeyboardCallback);

	window->setWindowTitle("Bullet 3.x GPU Rigid Body http://bulletphysics.org");
	printf("-----------------------------------------------------\n");


#ifndef __APPLE__
	glewInit();
#endif

	
	setupGUI(sGlutScreenWidth,sGlutScreenHeight);

	do
		{
			b3ProfileManager::Reset();
			b3ProfileManager::Increment_Frame_Counter();


			window->startRendering();

			glClearColor(0.6,0.6,0.6,1);
			glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);
			glEnable(GL_DEPTH_TEST);


			
			//	demo->clientMoveAndDisplay();
			
			//saveOpenGLState(sGlutScreenWidth,sGlutScreenHeight);

			{
				B3_PROFILE("gui->draw");
				//gui->draw(g_OpenGLWidth,g_OpenGLHeight);
				
				{
					B3_PROFILE("UpdateText");
					if (prof)
						prof->UpdateText();
				}
				{
					B3_PROFILE("RenderCanvas");
					pCanvas->RenderCanvas();
				}
	
			//restoreOpenGLState();
			}
			{
				B3_PROFILE("window->endRendering");
				window->endRendering();
			}
			


		
		b3ProfileManager::dumpAll();



		} while (!window->requestedExit());



	
	return 0;
}
