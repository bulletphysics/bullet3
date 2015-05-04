/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Controls/TreeNode.h"
#include "Gwen/Controls/TreeControl.h"
#include "Gwen/Utility.h"

using namespace Gwen;
using namespace Gwen::Controls;

class OpenToggleButton : public Button 
{
	GWEN_CONTROL_INLINE ( OpenToggleButton, Button )
	{
		SetIsToggle( true );
		SetTabable( false );

	}

	virtual void RenderFocus( Skin::Base* /*skin*/ ) {}

	virtual void Render( Skin::Base* skin )
	{
		skin->DrawTreeButton( this, GetToggleState() );
	}
};

const int TreeIndentation = 14;
const int BranchLength = 16;

GWEN_CONTROL_CONSTRUCTOR( TreeNode )
{
	m_TreeControl = NULL;

	m_ToggleButton = new OpenToggleButton( this );
	m_ToggleButton->SetBounds( 2, 2, 13, 13 );
	m_ToggleButton->onToggle.Add( this, &TreeNode::OnToggleButtonPress );

	m_Title = new Button( this );
	m_Title->Dock( Pos::Top );
	m_Title->SetMargin( Margin( BranchLength, 0, 0, 0 ) );
	m_Title->SetAlignment( Pos::Left | Pos::CenterV );
	m_Title->SetShouldDrawBackground( false );
	m_Title->onDoubleClick.Add( this, &TreeNode::OnDoubleClickName );
	m_Title->onDown.Add( this, &TreeNode::OnClickName );
	m_Title->SetHeight( 16 );

	m_InnerPanel = new Base( this );
	m_InnerPanel->Dock( Pos::Top );
	m_InnerPanel->SetHeight( 100 );
	m_InnerPanel->SetMargin( Margin( TreeIndentation, 1, 0, 0 ) );
	m_InnerPanel->Hide();

	m_bRoot = false;
	m_bSelected = false;
	m_bSelectable = true;
}

void TreeNode::Render( Skin::Base* skin )
{
	int iBottom = 0;
	if ( m_InnerPanel->Children.size() > 0 )
	{
		iBottom = m_InnerPanel->Children.back()->Y() + m_InnerPanel->Y();
	}
	
	skin->DrawTreeNode( this, m_InnerPanel->Visible(), IsSelected(), m_Title->Height(), m_Title->TextRight(), m_ToggleButton->Y() + m_ToggleButton->Height() * 0.5, iBottom, GetParent() == m_TreeControl );
}

TreeNode* TreeNode::AddNode( const UnicodeString& strLabel )
{
//	int sz = sizeof(TreeNode);
	TreeNode* node = new TreeNode( this );
	node->SetText( strLabel );
	node->Dock( Pos::Top );
	node->SetRoot( this->DynamicCastTreeControl() != NULL );
	node->SetTreeControl( m_TreeControl );

	if ( m_TreeControl )
	{
		m_TreeControl->OnNodeAdded( node );
	}

	return node;
}

TreeNode* TreeNode::AddNode( const String& strLabel )
{
	return AddNode( Utility::StringToUnicode( strLabel ) );
}


void TreeNode::Layout( Skin::Base* skin )
{
	if ( m_ToggleButton )
	{
		if ( m_InnerPanel->NumChildren() == 0 )
		{
			m_ToggleButton->Hide();
			m_ToggleButton->SetToggleState( false );
			m_InnerPanel->Hide();
		}
		else
		{
			m_ToggleButton->Show();
			m_InnerPanel->SizeToChildren( false, true );
		}
	}

	BaseClass::Layout( skin );
}
//too many calls to PostLayout...
//int numCalls = 0xfd;
void TreeNode::PostLayout( Skin::Base* /*skin*/ )
{
	
	//int bla = numCalls&0xffff;
	//if (bla==0)
	//	printf("TreeNode::PostLayout numCalls = %d\n", numCalls);

	//numCalls++;
	if ( SizeToChildren( false, true ) )
	{
		InvalidateParent();
	}
}

void TreeNode::SetText( const UnicodeString& text ){ m_Title->SetText( text ); };
void TreeNode::SetText( const String& text ){ m_Title->SetText( text ); };

UnicodeString TreeNode::GetText() const
{
	UnicodeString bla = m_Title->GetText();
	return bla;
}


void TreeNode::Open()
{
	m_InnerPanel->Show();
	if ( m_ToggleButton ) m_ToggleButton->SetToggleState( true );
	Invalidate();
	if (m_TreeControl)
		m_TreeControl->ForceUpdateScrollBars();
}

void TreeNode::Close()
{
	m_InnerPanel->Hide();
	if ( m_ToggleButton ) m_ToggleButton->SetToggleState( false );
	
	Invalidate();
	if (m_TreeControl)
		m_TreeControl->ForceUpdateScrollBars();
}

void TreeNode::ExpandAll()
{
	Open();

	Base::List& children = m_InnerPanel->GetChildren();
	for ( Base::List::iterator iter = children.begin(); iter != children.end(); ++iter )
	{
		TreeNode* pChild = (*iter)->DynamicCastTreeNode();
		if ( !pChild ) continue;

		pChild->ExpandAll();
	}
}

Button* TreeNode::GetButton(){ return m_Title; }


void TreeNode::OnToggleButtonPress( Base* /*control*/ )
{
	if ( m_ToggleButton->GetToggleState() )
	{
		Open();
	}
	else
	{
		Close();
	}
}

void TreeNode::OnDoubleClickName( Base* /*control*/ )
{
	if ( !m_ToggleButton->Visible() ) return;

	m_ToggleButton->Toggle();
}

void TreeNode::OnClickName( Base* /*control*/ )
{
	onNamePress.Call( this );

	SetSelected( !IsSelected() );
}

void TreeNode::SetSelected( bool b )
{ 
	if ( !m_bSelectable ) return;
	if ( m_bSelected == b ) return;

	m_bSelected = b; 

	onSelectChange.Call( this );

	if ( m_bSelected )
		onSelect.Call( this );
	else
		onUnselect.Call( this );
}

void TreeNode::DeselectAll()
{
	m_bSelected = false;

	Base::List& children = m_InnerPanel->GetChildren();
	for ( Base::List::iterator iter = children.begin(); iter != children.end(); ++iter )
	{
		TreeNode* pChild = (*iter)->DynamicCastTreeNode();
		if ( !pChild ) continue;

		pChild->DeselectAll( );
	}
}


void TreeNode::iterate(int action, int* curIndex, int* targetIndex)
{

	Gwen::String name = Gwen::Utility::UnicodeToString(m_Title->GetText());
	
//	int actualIndex = curIndex? *curIndex : -1;
	//printf("iterated over item %d with name = %s\n", actualIndex, name.c_str());

	if (action==ITERATE_ACTION_SELECT)
	{
		if (curIndex && targetIndex)
		{
			if ((*curIndex)==(*targetIndex))
			{
				SetSelected(true);
				
				*targetIndex=-1;
			}
		}
	}

	if (IsSelected())
	{
		//printf("current selected: name = %s\n", name.c_str());
		switch (action)
		{
		case ITERATE_ACTION_DESELECT_INDEX:
			{
				if (targetIndex && curIndex)
				{
					if (*targetIndex == *curIndex)
						SetSelected(false);
				}
				break;
			}
		case ITERATE_ACTION_FIND_SELECTED_INDEX:
			{
				if (targetIndex && curIndex)
				{
					*targetIndex = *curIndex;
				}
				break;
			}
		case ITERATE_ACTION_OPEN:
			{
				Open();
				
				
				break;
			}
		case ITERATE_ACTION_CLOSE:
		{
			//either close or select parent
			if (this->GetChildren().size())
			{
				if (m_ToggleButton && m_ToggleButton->GetToggleState())
				{
					Close();
				} else
				{
					
					TreeNode* pChild = (GetParent())->DynamicCastTreeNode();
					TreeControl* pChild2 = (GetParent())->DynamicCastTreeControl();
					if (pChild && !pChild2)
					{
						SetSelected(false);
						pChild->SetSelected(true);
					}
				}
			}
			else
			{
				
				TreeNode* pChild = (GetParent())->DynamicCastTreeNode();
				TreeControl* pChild2 = (GetParent())->DynamicCastTreeControl();
				if (pChild && !pChild2)
				{
					SetSelected(false);
					pChild->SetSelected(true);
				}
			}
			
			break;
		}
		default:
			{
			}
		};
	}

	if (curIndex)
		(*curIndex)++;

	bool needsRecursion = true;

	if (action == ITERATE_ACTION_FIND_SELECTED_INDEX || action==ITERATE_ACTION_SELECT || action==ITERATE_ACTION_DESELECT_INDEX)
	{
		if (m_ToggleButton && !m_ToggleButton->GetToggleState())
		{
			needsRecursion=false;
		}
	}

	if (needsRecursion)
	{
		Base::List& children = m_InnerPanel->GetChildren();
		for ( Base::List::iterator iter = children.begin(); iter != children.end(); ++iter )
		{
			TreeNode* pChild = (*iter)->DynamicCastTreeNode();
			if ( !pChild ) 
				continue;

			pChild->iterate(action , curIndex, targetIndex);
		}
	}

	
	
}