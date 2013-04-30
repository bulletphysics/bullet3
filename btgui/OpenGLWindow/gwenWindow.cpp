
#include "gwenWindow.h"


#include "Gwen/Platform.h"
#include "Gwen/Controls/TreeControl.h"
#include "Gwen/Controls/RadioButtonController.h"
#include "Gwen/Controls/VerticalSlider.h"
#include "Gwen/Controls/HorizontalSlider.h"
#include "Gwen/Controls/GroupBox.h"
#include "Gwen/Controls/CheckBox.h"
#include "Gwen/Controls/MenuStrip.h"


#include "Gwen/Gwen.h"
#include "Gwen/Align.h"
#include "Gwen/Utility.h"
#include "Gwen/Controls/WindowControl.h"
#include "Gwen/Controls/TabControl.h"
#include "Gwen/Controls/ListBox.h"

#include "Bullet3Common/b3Quickprof.h"
#include "GwenOpenGL3CoreRenderer.h"
#include "GLPrimitiveRenderer.h"

GLPrimitiveRenderer* primRenderer=0;
GwenOpenGL3CoreRenderer* pRenderer = 0;
//Gwen::Renderer::OpenGL_DebugFont * pRenderer =0;
Gwen::Skin::Simple skin;
Gwen::Controls::Canvas* pCanvas =0;
class MyProfileWindow* profWindow = 0;




/*struct MyHander   :public Gwen::Event::Handler
{

	MyHander  (Application* app)
		:m_app(app)
	{
	}

	void onButtonA( Gwen::Controls::Base* pControl )
	{
			OpenTissue::glut::toggleIdle();
	}

	void SliderMoved(Gwen::Controls::Base* pControl )
	{
		Gwen::Controls::Slider* pSlider = (Gwen::Controls::Slider*)pControl;
		this->m_app->scaleYoungModulus(pSlider->GetValue());
		//	printf("Slider Value: %.2f", pSlider->GetValue() );
	}


	void	OnCheckChangedStiffnessWarping (Gwen::Controls::Base* pControl)
	{
		Gwen::Controls::CheckBox* labeled = (Gwen::Controls::CheckBox* )pControl;
		bool checked = labeled->IsChecked();
		m_app->m_stiffness_warp_on  = checked;
	}


};

*/



class MyProfileWindow : public Gwen::Controls::WindowControl
{
	
	//		Gwen::Controls::TabControl*	m_TabControl;
	//Gwen::Controls::ListBox*	m_TextOutput;
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
		SetTitle( L"FEM Settings" );

		SetSize( 450, 150 );
		this->SetPos(10,40);
		
//		this->Dock( Gwen::Pos::Bottom);

		

		{
			m_ctrl = new Gwen::Controls::TreeControl( this );
			m_node = m_ctrl->AddNode( L"Total Parent Time" );
			
		
			//Gwen::Controls::TreeNode* pNode = ctrl->AddNode( L"Node Two" );
			//pNode->AddNode( L"Node Two Inside" );
			//pNode->AddNode( L"Eyes" );
			//pNode->AddNode( L"Brown" )->AddNode( L"Node Two Inside" )->AddNode( L"Eyes" )->AddNode( L"Brown" );
			//Gwen::Controls::TreeNode* node = ctrl->AddNode( L"Node Three" );
				
			

			//m_ctrl->Dock(Gwen::Pos::Bottom);
			
			m_ctrl->ExpandAll();
			m_ctrl->SetBounds( this->GetInnerBounds().x,this->GetInnerBounds().y,this->GetInnerBounds().w,this->GetInnerBounds().h);
			
		}



	}


	float	dumpRecursive(b3ProfileIterator* profileIterator, Gwen::Controls::TreeNode* parentNode)
	{
		profileIterator->First();
		if (profileIterator->Is_Done())
			return 0.f;

		float accumulated_time=0,parent_time = profileIterator->Is_Root() ? b3ProfileManager::Get_Time_Since_Reset() : profileIterator->Get_Current_Parent_Total_Time();
		int i;
		int frames_since_reset = b3ProfileManager::Get_Frame_Count_Since_Reset();
		
		//printf("Profiling: %s (total running time: %.3f ms) ---\n",	profileIterator->Get_Current_Parent_Name(), parent_time );
		float totalTime = 0.f;

	
		int numChildren = 0;
		Gwen::UnicodeString txt;
		std::vector<Gwen::Controls::TreeNode*> nodes;

		for (i = 0; !profileIterator->Is_Done(); i++,profileIterator->Next())
		{
			numChildren++;
			float current_total_time = profileIterator->Get_Current_Total_Time();
			accumulated_time += current_total_time;
			double fraction = parent_time > B3_EPSILON ? (current_total_time / parent_time) * 100 : 0.f;
			
			Gwen::String name(profileIterator->Get_Current_Name());
#ifdef _WIN32
			Gwen::UnicodeString uname = Gwen::Utility::StringToUnicode(name);

			txt = Gwen::Utility::Format(L"%s (%.2f %%) :: %.3f ms / frame (%d calls)",uname.c_str(), fraction,(current_total_time / (double)frames_since_reset),profileIterator->Get_Current_Total_Calls());
			
#else
			txt = Gwen::Utility::Format(L"%s (%.2f %%) :: %.3f ms / frame (%d calls)",name.c_str(), fraction,(current_total_time / (double)frames_since_reset),profileIterator->Get_Current_Total_Calls());
			
#endif

			Gwen::Controls::TreeNode* childNode = (Gwen::Controls::TreeNode*)profileIterator->Get_Current_UserPointer();
			if (!childNode)
			{
					childNode = parentNode->AddNode(L"");
					profileIterator->Set_Current_UserPointer(childNode);
			}
			childNode->SetText(txt);
			nodes.push_back(childNode);
		
			totalTime += current_total_time;
			//recurse into children
		}
	
		for (i=0;i<numChildren;i++)
		{
			profileIterator->Enter_Child(i);
			Gwen::Controls::TreeNode* curNode = nodes[i];

			dumpRecursive(profileIterator, curNode);
			
			profileIterator->Enter_Parent();
		}
		return accumulated_time;

	}

	void	UpdateText(b3ProfileIterator*  profileIterator, bool idle)
	{
	
		static bool update=true;

			m_ctrl->SetBounds(0,0,this->GetInnerBounds().w,this->GetInnerBounds().h);

//		if (!update)
//			return;
		update=false;

	
		static int test = 1;
		test++;

		static double time_since_reset = 0.f;
		if (!idle)
		{
			time_since_reset = b3ProfileManager::Get_Time_Since_Reset();
		}

		//Gwen::UnicodeString txt = Gwen::Utility::Format( L"FEM Settings  %i fps", test );
		{
		//recompute profiling data, and store profile strings

		char blockTime[128];

		double totalTime = 0;

		int frames_since_reset = b3ProfileManager::Get_Frame_Count_Since_Reset();

		profileIterator->First();

		double parent_time = profileIterator->Is_Root() ? time_since_reset : profileIterator->Get_Current_Parent_Total_Time();

	
		Gwen::Controls::TreeNode* curParent = m_node;

		double accumulated_time = dumpRecursive(profileIterator,m_node);

		const char* name = profileIterator->Get_Current_Parent_Name();
#ifdef _WIN32
		Gwen::UnicodeString uname = Gwen::Utility::StringToUnicode(name);
		Gwen::UnicodeString txt = Gwen::Utility::Format( L"Profiling: %s total time: %.3f ms, unaccounted %.3f %% :: %.3f ms", uname.c_str(), parent_time ,
			parent_time > B3_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f, parent_time - accumulated_time);
#else
		Gwen::UnicodeString txt = Gwen::Utility::Format( L"Profiling: %s total time: %.3f ms, unaccounted %.3f %% :: %.3f ms", name, parent_time ,
			parent_time > B3_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f, parent_time - accumulated_time);
#endif
		//sprintf(blockTime,"--- Profiling: %s (total running time: %.3f ms) ---",	profileIterator->Get_Current_Parent_Name(), parent_time );
		//displayProfileString(xOffset,yStart,blockTime);
		m_node->SetText(txt);


			//printf("%s (%.3f %%) :: %.3f ms\n", "Unaccounted:",);
	

		}
		
		static bool once1 = true;
		if (once1)
		{
			once1 = false;
			m_ctrl->ExpandAll();
		}

	}
	void PrintText( const Gwen::UnicodeString& str )
	{

	}

	void Render( Gwen::Skin::Base* skin )
	{
		m_iFrames++;

		if ( m_fLastSecond < Gwen::Platform::GetTimeInSeconds() )
		{
			SetTitle( Gwen::Utility::Format( L"Profiler  %i fps", m_iFrames ) );

			m_fLastSecond = Gwen::Platform::GetTimeInSeconds() + 1.0f;
			m_iFrames = 0;
		}

		Gwen::Controls::WindowControl::Render( skin );

	}


};

struct MyTestMenuBar : public Gwen::Controls::MenuStrip
{
	MyProfileWindow* m_profileWindow;


	MyTestMenuBar(Gwen::Controls::Base* pParent, MyProfileWindow* profileWindow)
		:Gwen::Controls::MenuStrip(pParent),
		m_profileWindow(profileWindow)
	{
//		Gwen::Controls::MenuStrip* menu = new Gwen::Controls::MenuStrip( pParent );
		{
			Gwen::Controls::MenuItem* pRoot = AddItem( L"File" );
		
			pRoot = AddItem( L"View" );
//			Gwen::Event::Handler* handler =	GWEN_MCALL(&MyTestMenuBar::MenuItemSelect );
			pRoot->GetMenu()->AddItem( L"Profiler",m_profileWindow,(Gwen::Event::Handler::Function)&MyProfileWindow::MenuItemSelect);

/*			pRoot->GetMenu()->AddItem( L"New", L"test16.png", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			pRoot->GetMenu()->AddItem( L"Load", L"test16.png", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			pRoot->GetMenu()->AddItem( L"Save", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			pRoot->GetMenu()->AddItem( L"Save As..", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			pRoot->GetMenu()->AddItem( L"Quit", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			*/
		}
	}

};

void	setupGUI(int width, int height, sth_stash* font, float retinaScale,GLPrimitiveRenderer* primRender)
{
	primRenderer = primRender;
	pRenderer = new GwenOpenGL3CoreRenderer(primRenderer,font,width,height, retinaScale);
	if (primRenderer)
		primRenderer->setScreenSize(width,height);

//	pRenderer = new Gwen::Renderer::OpenGL_DebugFont();
	skin.SetRender( pRenderer );

	pCanvas = new Gwen::Controls::Canvas( &skin );
	pCanvas->SetSize( width,height);
	pCanvas->SetDrawBackground( false);
	pCanvas->SetBackgroundColor( Gwen::Color( 150, 170, 170, 255 ) );
//	pCanvas->SetScale(.5);
	//MyWindow* window = new MyWindow(pCanvas);
	profWindow = new MyProfileWindow(pCanvas);
	
	MyTestMenuBar* menubar = new MyTestMenuBar(pCanvas, profWindow);


	
}

void resizeGUI(int width, int height)
{
	pCanvas->SetSize(width,height);
	pRenderer->resize(width,height);
	primRenderer->setScreenSize(width,height);
}

void	processProfileData(b3ProfileIterator*  iterator, bool idle)
{
	if (profWindow)
	{
		
		profWindow->UpdateText(iterator, idle);
	}

}
