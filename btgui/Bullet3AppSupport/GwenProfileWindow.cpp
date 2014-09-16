#include "GwenProfileWindow.h"
#include "gwenUserInterface.h"
#include "gwenInternalData.h"
#include "LinearMath/btQuickprof.h"





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
	//	Gwen::Controls::Slider* pSlider = (Gwen::Controls::Slider*)pControl;
		//this->m_app->scaleYoungModulus(pSlider->GetValue());
		//	printf("Slider Value: %.2f", pSlider->GetValue() );
	}
	
	
	void	OnCheckChangedStiffnessWarping (Gwen::Controls::Base* pControl)
	{
	//	Gwen::Controls::CheckBox* labeled = (Gwen::Controls::CheckBox* )pControl;
//		bool checked = labeled->IsChecked();
		//m_app->m_stiffness_warp_on  = checked;
	}
public:
	
  
	CProfileIterator* profIter;
	
	MyProfileWindow (	Gwen::Controls::Base* pParent)
    : Gwen::Controls::WindowControl( pParent ),
	profIter(0)
	{
		SetTitle( L"Time Profiler" );
		
		SetSize( 450, 450 );
		this->SetPos(10,400);
		
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
            m_ctrl->SetKeyboardInputEnabled(true);
			m_ctrl->SetBounds( this->GetInnerBounds().x,this->GetInnerBounds().y,this->GetInnerBounds().w,this->GetInnerBounds().h);
			
		}
		
		
		
	}
	
	
	float	dumpRecursive(CProfileIterator* profileIterator, Gwen::Controls::TreeNode* parentNode)
	{
		profileIterator->First();
		if (profileIterator->Is_Done())
			return 0.f;
		
		float accumulated_time=0,parent_time = profileIterator->Is_Root() ? CProfileManager::Get_Time_Since_Reset() : profileIterator->Get_Current_Parent_Total_Time();
		int i;
		int frames_since_reset = CProfileManager::Get_Frame_Count_Since_Reset();
		
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
			double fraction = parent_time > SIMD_EPSILON ? (current_total_time / parent_time) * 100 : 0.f;
			
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
	
	void	UpdateText(CProfileIterator*  profileIterator, bool idle)
	{
		
	//	static bool update=true;
		
        m_ctrl->SetBounds(0,0,this->GetInnerBounds().w,this->GetInnerBounds().h);
		
        //		if (!update)
        //			return;
	//	update=false;
		
		
		static int test = 1;
		test++;
		
		static double time_since_reset = 0.f;
		if (!idle)
		{
			time_since_reset = CProfileManager::Get_Time_Since_Reset();
		}
		
		//Gwen::UnicodeString txt = Gwen::Utility::Format( L"FEM Settings  %i fps", test );
		{
            //recompute profiling data, and store profile strings
			
         //   char blockTime[128];
			
           // double totalTime = 0;
			
          //  int frames_since_reset = CProfileManager::Get_Frame_Count_Since_Reset();
			
            profileIterator->First();
			
            double parent_time = profileIterator->Is_Root() ? time_since_reset : profileIterator->Get_Current_Parent_Total_Time();
			
			
         //   Gwen::Controls::TreeNode* curParent = m_node;
			
            double accumulated_time = dumpRecursive(profileIterator,m_node);
			
            const char* name = profileIterator->Get_Current_Parent_Name();
#ifdef _WIN32
            Gwen::UnicodeString uname = Gwen::Utility::StringToUnicode(name);
            Gwen::UnicodeString txt = Gwen::Utility::Format( L"Profiling: %s total time: %.3f ms, unaccounted %.3f %% :: %.3f ms", uname.c_str(), parent_time ,
                                                            parent_time > SIMD_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f, parent_time - accumulated_time);
#else
            Gwen::UnicodeString txt = Gwen::Utility::Format( L"Profiling: %s total time: %.3f ms, unaccounted %.3f %% :: %.3f ms", name, parent_time ,
                                                            parent_time > SIMD_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f, parent_time - accumulated_time);
#endif
            //sprintf(blockTime,"--- Profiling: %s (total running time: %.3f ms) ---",	profileIterator->Get_Current_Parent_Name(), parent_time );
            //displayProfileString(xOffset,yStart,blockTime);
            m_node->SetText(txt);
			
			
			//printf("%s (%.3f %%) :: %.3f ms\n", "Unaccounted:",);
			
			
		}
		
		static int counter=10;
		if (counter)
		{
            counter--;
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

class MyMenuItems :  public Gwen::Controls::Base
{
	
public:
	
	class MyProfileWindow* m_profWindow;
    MyMenuItems() :Gwen::Controls::Base(0)
    {
    }
   
    void MenuItemSelect(Gwen::Controls::Base* pControl)
    {
		if (m_profWindow->Hidden())
		{
			m_profWindow->SetHidden(false);
		} else
		{
			m_profWindow->SetHidden(true);
		}
		
    }
};


MyProfileWindow* setupProfileWindow(GwenInternalData* data)
{
	MyMenuItems* menuItems = new MyMenuItems;
	MyProfileWindow* profWindow = new MyProfileWindow(data->pCanvas);
	//profWindow->SetHidden(true);	
	profWindow->profIter = CProfileManager::Get_Iterator();
	data->m_viewMenu->GetMenu()->AddItem( L"Profiler", menuItems,(Gwen::Event::Handler::Function)&MyMenuItems::MenuItemSelect);
	menuItems->m_profWindow = profWindow;
	return profWindow;
}


void	processProfileData( MyProfileWindow* profWindow, bool idle)
{
	if (profWindow)
	{
		
		profWindow->UpdateText(profWindow->profIter, idle);
	}	
}

void profileWindowSetVisible(MyProfileWindow* window, bool visible)
{
	window->SetHidden(!visible);
}
void destroyProfileWindow(MyProfileWindow* window)
{
	delete window;
}
