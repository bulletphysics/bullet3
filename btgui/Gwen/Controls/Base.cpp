/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Label.h"
#include "Gwen/Gwen.h"
#include "Gwen/BaseRender.h"
#include "Gwen/Skin.h"
#include "Gwen/Platform.h"
#include "Gwen/DragAndDrop.h"
#include "Gwen/ToolTip.h"
#include "Gwen/Utility.h"
#include <list>

#ifndef GWEN_NO_ANIMATION
#include "Gwen/Anim.h"
#endif

using namespace Gwen;
using namespace Controls;

Base::Base( Base* pParent )
{
	m_Parent = NULL;
	m_ActualParent = NULL;
	m_InnerPanel = NULL;
	m_Skin = NULL;

	SetParent( pParent );

	m_bHidden = false;
	m_Bounds = Gwen::Rect(0,0,10,10);
	m_Padding = Padding( 0, 0, 0, 0 );
	m_Margin = Margin( 0, 0, 0, 0 );

	m_iDock = 0;
	m_DragAndDrop_Package = NULL;
	m_pUserData = NULL;

	RestrictToParent( false );

	SetMouseInputEnabled( true );
	SetKeyboardInputEnabled( false );

	Invalidate();
	SetCursor( Gwen::CursorType::Normal );
	SetToolTip( NULL );
	SetTabable( false );
	SetShouldDrawBackground( true );
	m_bDisabled = false;
	m_bCacheTextureDirty = true;
	m_bCacheToTexture = false;

}

Base::~Base()
{
	{
		Canvas* canvas = GetCanvas();
		if ( canvas )
			canvas->PreDelete( this );
	}

	Base::List::iterator iter = Children.begin();
	while ( iter != Children.end() )
	{
		Base* pChild = *iter;
		iter = Children.erase( iter );
		delete pChild;
	}

	for ( AccelMap::iterator accelIt = m_Accelerators.begin(); accelIt != m_Accelerators.end(); ++accelIt )
	{
		delete accelIt->second;
	}
	m_Accelerators.clear();

	SetParent( NULL );

	if ( Gwen::HoveredControl == this ) 
		Gwen::HoveredControl = NULL;
	if ( Gwen::KeyboardFocus == this ) 
		Gwen::KeyboardFocus = NULL;
	if ( Gwen::MouseFocus == this ) Gwen::MouseFocus = NULL;

	DragAndDrop::ControlDeleted( this );
	ToolTip::ControlDeleted( this );

	#ifndef GWEN_NO_ANIMATION
	Anim::Cancel( this );
	#endif

	if ( m_DragAndDrop_Package )
	{
		delete m_DragAndDrop_Package;
		m_DragAndDrop_Package = NULL;
	}
}

 extern int avoidUpdate;


void Base::Invalidate()
{

   

	m_bNeedsLayout = true;
	m_bCacheTextureDirty = true;
	
	avoidUpdate = -3;
}

void Base::DelayedDelete()
{
	Canvas* canvas = GetCanvas();
	canvas->AddDelayedDelete( this );
}

Canvas* Base::GetCanvas()
{
	Base* pCanvas = m_Parent;
	if ( !pCanvas ) return NULL;

	return pCanvas->GetCanvas();
}

void Base::SetParent(Base* pParent)
{
	if ( m_Parent == pParent ) return;

	if ( m_Parent )
	{
		m_Parent->RemoveChild( this );
	}

	m_Parent = pParent;
	m_ActualParent = NULL;

	if ( m_Parent )
	{
		m_Parent->AddChild( this );
	}
}

void Base::Dock( int iDock )
{
	if ( m_iDock == iDock ) return;

	m_iDock = iDock;

	Invalidate();
	InvalidateParent();
}

int Base::GetDock()
{
	return m_iDock;
}

bool Base::Hidden() const
{
	return m_bHidden;
}

bool Base::Visible() const
{
	if ( Hidden() ) return false;
	if ( GetParent() )
	{
		return GetParent()->Visible();
	}

	return true;
}

void Base::InvalidateChildren( bool bRecursive )
{
	for ( Base::List::iterator it = Children.begin(); it != Children.end(); ++it )
	{
		(*it)->Invalidate();

		if ( bRecursive )
			(*it)->InvalidateChildren( bRecursive );
	}

	if ( m_InnerPanel )
	{
		for ( Base::List::iterator it = m_InnerPanel->Children.begin(); it != m_InnerPanel->Children.end(); ++it )
		{
			(*it)->Invalidate();

			if ( bRecursive )
				(*it)->InvalidateChildren( bRecursive );
		}
	}
}

void Base::Position( int pos, int xpadding, int ypadding )
{
	int w = GetParent()->Width();
	int h = GetParent()->Height();
	const Padding& padding = GetParent()->GetPadding();

	int x = X();
	int y = Y();

	if ( pos & Pos::Left ) x = padding.left + xpadding;
	if ( pos & Pos::Right ) x = w - Width() - padding.right - xpadding;
	if ( pos & Pos::CenterH ) x = padding.left + xpadding + (w - Width() - padding.left - padding.right) * 0.5;

	if ( pos & Pos::Top ) y = padding.top + ypadding;
	if ( pos & Pos::Bottom ) y = h - Height() - padding.bottom - ypadding;
	if ( pos & Pos::CenterV ) y = padding.top + ypadding + (h - Height() - padding.bottom - padding.top) * 0.5;

	SetPos( x, y );
}


void Base::SendToBack()
{
	if ( !m_Parent ) return;
	if ( m_Parent->Children.front() == this ) return;

	m_Parent->Children.remove( this );
	m_Parent->Children.push_front( this );

	InvalidateParent();
}

void Base::BringToFront()
{
	if ( !m_Parent ) return;
	if ( m_Parent->Children.back() == this ) return;

	m_Parent->Children.remove( this );
	m_Parent->Children.push_back( this );
	InvalidateParent();
}

Controls::Base* Base::FindChildByName( const Gwen::String& name, bool bRecursive )
{
	Base::List::iterator iter;
	for (iter = Children.begin(); iter != Children.end(); ++iter)
	{
		Base* pChild = *iter;
		if ( pChild->GetName() == name )
			return pChild;

		if ( bRecursive )
		{
			Controls::Base* pSubChild = pChild->FindChildByName( name, true );

			if ( pSubChild )
				return pSubChild;
		}
	}

	return NULL;
}

void Base::BringNextToControl( Controls::Base* pChild, bool bBehind )
{
	if ( !m_Parent ) return;

	m_Parent->Children.remove( this );

	Base::List::iterator it = std::find( m_Parent->Children.begin(), m_Parent->Children.end(), pChild );
	if ( it == m_Parent->Children.end() ) 
		return BringToFront();

	if ( bBehind )
	{
		++it;

		if ( it == m_Parent->Children.end() ) 
			return BringToFront();
	}

	m_Parent->Children.insert( it, this );
	InvalidateParent();
}

void Base::AddChild(Base* pChild)
{
	if ( m_InnerPanel )
	{
		m_InnerPanel->AddChild( pChild );
		return;
	}

	Children.push_back( pChild );
	OnChildAdded(pChild);

	pChild->m_ActualParent = this;

}
void Base::RemoveChild(Base* pChild)
{
	// If we removed our innerpanel
	// remove our pointer to it
	if ( m_InnerPanel == pChild )
	{
		m_InnerPanel = NULL;
	}

	if ( m_InnerPanel )
	{
		m_InnerPanel->RemoveChild( pChild );
	}

	Children.remove( pChild );
	OnChildRemoved(pChild);
}

void Base::RemoveAllChildren()
{
	while ( Children.size() > 0 )
	{
		RemoveChild( *Children.begin() );
	}
}

int Base::NumChildren()
{
	// Include m_InnerPanel's children here?

	return Children.size();
}

void Base::OnChildAdded(Base* /*pChild*/)
{
	Invalidate();
}

void Base::OnChildRemoved(Base* /*pChild*/)
{
	Invalidate();
}

Skin::Base* Base::GetSkin( void )
{
	if ( m_Skin ) return m_Skin;
	if ( m_Parent ) return m_Parent->GetSkin();

	Debug::AssertCheck( 0, "Base::GetSkin Returning NULL!\n" );
	return NULL;
}

void Base::MoveBy( int x, int y )
{
	SetBounds( X() + x, Y() + y, Width(), Height() );
}

void Base::MoveTo( int x, int y )
{
	if ( m_bRestrictToParent && GetParent() )
	{
		Base* pParent = GetParent();
		if ( x - GetPadding().left < pParent->GetMargin().left )	x = pParent->GetMargin().left + GetPadding().left;
		if ( y - GetPadding().top < pParent->GetMargin().top ) y = pParent->GetMargin().top + GetPadding().top;
		if ( x + Width() + GetPadding().right > pParent->Width() - pParent->GetMargin().right ) x = pParent->Width() - pParent->GetMargin().right - Width() - GetPadding().right;
		if ( y + Height() + GetPadding().bottom > pParent->Height() - pParent->GetMargin().bottom ) y = pParent->Height() - pParent->GetMargin().bottom - Height() - GetPadding().bottom;
	}

	SetBounds(x, y, Width(), Height());
}

void Base::SetPos( int x, int y )
{
	SetBounds( x, y, Width(), Height() );
}

bool Base::SetSize( int w, int h )
{
	return SetBounds( X(), Y(), w, h );
}

bool Base::SetBounds( const Gwen::Rect& bounds )
{
	return SetBounds( bounds.x, bounds.y, bounds.w, bounds.h );
}

bool Base::SetBounds( int x, int y, int w, int h )
{
	if ( m_Bounds.x == x && 
		 m_Bounds.y == y && 
		 m_Bounds.w == w && 
		 m_Bounds.h == h )
		return false;

	Gwen::Rect oldBounds = GetBounds();

	m_Bounds.x = x;
	m_Bounds.y = y;

	m_Bounds.w = w;
	m_Bounds.h = h;

	OnBoundsChanged( oldBounds );

	return true;
}

void Base::OnBoundsChanged(Gwen::Rect oldBounds)
{
	//Anything that needs to update on size changes
	//Iterate my children and tell them I've changed
	//
	


	if ( m_Bounds.w != oldBounds.w || m_Bounds.h != oldBounds.h )
	{
		if ( GetParent() )
			GetParent()->OnChildBoundsChanged( oldBounds, this );
		Invalidate();
	}

	Redraw();
	UpdateRenderBounds();
}

void Base::OnScaleChanged()
{
	for ( Base::List::iterator iter = Children.begin(); iter != Children.end(); ++iter )
	{
		(*iter)->OnScaleChanged();
	}
}

void Base::OnChildBoundsChanged( Gwen::Rect /*oldChildBounds*/, Base* /*pChild*/ )
{

}

void Base::Render( Gwen::Skin::Base* /*skin*/ )
{
}

void Base::DoCacheRender( Gwen::Skin::Base* skin, Gwen::Controls::Base* pMaster )
{
	Gwen::Renderer::Base* render = skin->GetRender();
	Gwen::Renderer::ICacheToTexture* cache = render->GetCTT();

	if ( !cache ) return;

	Gwen::Point pOldRenderOffset = render->GetRenderOffset();

	Gwen::Rect rOldRegion = render->ClipRegion();
	
	if ( this != pMaster )
	{
		render->AddRenderOffset( GetBounds() );
		render->AddClipRegion( GetBounds() );
	}
	else
	{
		render->SetRenderOffset( Gwen::Point( 0, 0 ) );
		render->SetClipRegion( GetBounds() );
	}

	if ( m_bCacheTextureDirty && render->ClipRegionVisible() )
	{
		render->StartClip();

		if ( ShouldCacheToTexture() )
			cache->SetupCacheTexture( this );

		//Render myself first
		Render( skin );

		if ( !Children.empty() )
		{
			//Now render my kids
			for (Base::List::iterator iter = Children.begin(); iter != Children.end(); ++iter)
			{
				Base* pChild = *iter;
				if ( pChild->Hidden() ) continue;

				pChild->DoCacheRender( skin, pMaster );
			}		
		}

		if ( ShouldCacheToTexture() )
		{
			cache->FinishCacheTexture( this );
			m_bCacheTextureDirty = false;
		}
	}

	render->SetClipRegion( rOldRegion );
	render->StartClip();
	render->SetRenderOffset( pOldRenderOffset );
	cache->DrawCachedControlTexture( this );
}

void Base::DoRender( Gwen::Skin::Base* skin )
{
	// If this control has a different skin, 
	// then so does its children.
	if ( m_Skin ) 
		skin = m_Skin;

	// Do think
	Think();

	Gwen::Renderer::Base* render = skin->GetRender();

	if ( render->GetCTT() && ShouldCacheToTexture() )
	{
		DoCacheRender( skin, this );
		return;
	}

	Gwen::Point pOldRenderOffset = render->GetRenderOffset();

	render->AddRenderOffset( GetBounds() );

	RenderUnder( skin );
	
	Gwen::Rect rOldRegion = render->ClipRegion();
	render->AddClipRegion( GetBounds() );

	if ( render->ClipRegionVisible() )
	{
		render->StartClip();
		
		//Render myself first
 		Render( skin );

		if ( !Children.empty() )
		{
			//Now render my kids
			for (Base::List::iterator iter = Children.begin(); iter != Children.end(); ++iter)
			{
				Base* pChild = *iter;
				if ( pChild->Hidden() ) continue;

				pChild->DoRender( skin );
			}		
		}

		render->SetClipRegion( rOldRegion );
		render->StartClip();

		RenderOver( skin );
	}
	else
	{
		render->SetClipRegion( rOldRegion );
	}

	RenderFocus( skin );

	render->SetRenderOffset( pOldRenderOffset );
}

void Base::SetSkin( Skin::Base* skin, bool doChildren )
{
	if ( m_Skin == skin ) return;
	m_Skin = skin;
	Invalidate();
	Redraw();
	OnSkinChanged( skin );

	if ( doChildren )
	{
		for ( Base::List::iterator it = Children.begin(); it != Children.end(); ++it )
		{
			(*it)->SetSkin( skin, true);
		}
	}
}

void Base::OnSkinChanged( Skin::Base* /*skin*/ )
{
	//Do something
}

bool Base::OnMouseWheeled( int iDelta )
{
	if ( m_ActualParent )
		return m_ActualParent->OnMouseWheeled( iDelta );

	return false;
}

void Base::OnMouseMoved( int /*x*/, int /*y*/, int /*deltaX*/, int /*deltaY*/ )
{
}

void Base::OnMouseEnter()
{
	onHoverEnter.Call( this );

	if ( GetToolTip() )
		ToolTip::Enable( this );
	else if ( GetParent() && GetParent()->GetToolTip() )
		ToolTip::Enable( GetParent() );
}

void Base::OnMouseLeave()
{
	onHoverLeave.Call( this );
	if ( GetToolTip() )
		ToolTip::Disable( this );
}


bool Base::IsHovered()
{
	return Gwen::HoveredControl == this;
}

bool Base::ShouldDrawHover()
{
	return Gwen::MouseFocus == this || Gwen::MouseFocus == NULL;
}

bool Base::HasFocus()
{
	return Gwen::KeyboardFocus == this;
}

void Base::Focus()
{
	if ( Gwen::KeyboardFocus == this ) return;

	if ( Gwen::KeyboardFocus ) 
		Gwen::KeyboardFocus->OnLostKeyboardFocus();

	Gwen::KeyboardFocus = this;

	OnKeyboardFocus();
	Redraw();
}

void Base::Blur()
{
	if ( Gwen::KeyboardFocus != this ) return;

	Gwen::KeyboardFocus = NULL;
	OnLostKeyboardFocus();

	Redraw();
}

bool Base::IsOnTop()
{
	Base::List::iterator iter = GetParent()->Children.begin();
	Base* pChild = *iter;

	if ( pChild == this )
		return true;

	return false;
}


void Base::Touch()
{
	if ( GetParent() )
		GetParent()->OnChildTouched( this );
}

void Base::OnChildTouched( Controls::Base* /*pChild*/ )
{
	Touch();
}

Base* Base::GetControlAt( int x, int y )
{
	if ( Hidden() ) 
		return NULL;

	if ( x < 0 || y < 0 || x >= Width() || y >= Height() )
		return NULL;

	Base::List::reverse_iterator iter;
	for (iter = Children.rbegin(); iter != Children.rend(); ++iter)
	{
		Base* pChild = *iter;
		Base* pFound = NULL;
		pFound = pChild->GetControlAt( x - pChild->X(), y - pChild->Y() );
		if ( pFound ) return pFound;
	}

	if ( !GetMouseInputEnabled() )
		return NULL;

	return this;
}


void Base::Layout( Skin::Base* skin )
{
	if ( skin->GetRender()->GetCTT() && ShouldCacheToTexture() )
		skin->GetRender()->GetCTT()->CreateControlCacheTexture( this );
}

int avoidUpdate = -15;

void Base::RecurseLayout( Skin::Base* skin )
{
	if ( m_Skin ) skin = m_Skin;
	if ( Hidden() ) return;

	if ( NeedsLayout() )
	{
		m_bNeedsLayout = false;
		Layout( skin );
	}
	

	if (avoidUpdate>0)
		return;

	Gwen::Rect rBounds = GetRenderBounds();

	// Adjust bounds for padding
	rBounds.x += m_Padding.left;
	rBounds.w -= m_Padding.left + m_Padding.right;
	rBounds.y += m_Padding.top;
	rBounds.h -= m_Padding.top + m_Padding.bottom;

	int sz = Children.size();
	if (sz>100)
	{
//		printf("!\n");
		
	}
	int curChild = 0;
	for (Base::List::iterator iter = Children.begin(); iter != Children.end(); ++iter)
	{
		Base* pChild = *iter;
		curChild++;
		if ( pChild->Hidden() )
			continue;

		
		int iDock = pChild->GetDock();

		if ( iDock & Pos::Fill )
			continue;

		if ( iDock & Pos::Top )
		{
			const Margin& margin = pChild->GetMargin();

			pChild->SetBounds( rBounds.x + margin.left, rBounds.y + margin.top, rBounds.w - margin.left - margin.right, pChild->Height() );

			int iHeight = margin.top + margin.bottom + pChild->Height();
			rBounds.y += iHeight;
			rBounds.h -= iHeight;
		}

		if ( iDock & Pos::Left )
		{
			const Margin& margin = pChild->GetMargin();

			pChild->SetBounds( rBounds.x + margin.left, rBounds.y + margin.top, pChild->Width(), rBounds.h - margin.top - margin.bottom );

			int iWidth = margin.left + margin.right + pChild->Width();
			rBounds.x += iWidth;
			rBounds.w -= iWidth;
		}

		if ( iDock & Pos::Right )
		{ 
			// TODO: THIS MARGIN CODE MIGHT NOT BE FULLY FUNCTIONAL
			const Margin& margin = pChild->GetMargin();

			pChild->SetBounds( (rBounds.x+rBounds.w)-pChild->Width()-margin.right, rBounds.y + margin.top, pChild->Width(), rBounds.h - margin.top - margin.bottom );

			int iWidth = margin.left + margin.right + pChild->Width();
			rBounds.w -= iWidth;
		}

		if ( iDock & Pos::Bottom )
		{
			// TODO: THIS MARGIN CODE MIGHT NOT BE FULLY FUNCTIONAL
			const Margin& margin = pChild->GetMargin();

			pChild->SetBounds( rBounds.x + margin.left, (rBounds.y+rBounds.h)-pChild->Height()-margin.bottom, rBounds.w - margin.left - margin.right, pChild->Height() );
			rBounds.h -= pChild->Height() + margin.bottom + margin.top;
		}

		pChild->RecurseLayout( skin );

	}

	m_InnerBounds = rBounds;

	curChild = 0;
	//
	// Fill uses the left over space, so do that now.
	//
	for (Base::List::iterator iter = Children.begin(); iter != Children.end(); ++iter)
	{
		Base* pChild = *iter;
		int iDock = pChild->GetDock();
		curChild++;
		if ( !(iDock & Pos::Fill) )
			continue;

		

		const Margin& margin = pChild->GetMargin();

		pChild->SetBounds( rBounds.x + margin.left, rBounds.y + margin.top, rBounds.w - margin.left - margin.right, rBounds.h - margin.top - margin.bottom );
		pChild->RecurseLayout( skin );
	}

	PostLayout( skin );

	if ( IsTabable() )
	{
		if ( !GetCanvas()->FirstTab ) GetCanvas()->FirstTab = this;
		if ( !GetCanvas()->NextTab ) GetCanvas()->NextTab = this;
	}

	if ( Gwen::KeyboardFocus == this )
	{
		GetCanvas()->NextTab = NULL;	
	}
}

bool Base::IsChild( Controls::Base* pChild )
{
	for (Base::List::iterator iter = Children.begin(); iter != Children.end(); ++iter)
	{
		if ( pChild == (*iter) ) return true;
	}

	return false;
}

Gwen::Point Base::LocalPosToCanvas( const Gwen::Point& pnt )
{
	if ( m_Parent )
	{
		int x = pnt.x + X();
		int y = pnt.y + Y();

		// If our parent has an innerpanel and we're a child of it
		// add its offset onto us.
		//
		if ( m_Parent->m_InnerPanel && m_Parent->m_InnerPanel->IsChild( this ) )
		{
			x += m_Parent->m_InnerPanel->X();
			y += m_Parent->m_InnerPanel->Y();
		}

		return m_Parent->LocalPosToCanvas( Gwen::Point( x, y ) );
	}

	return pnt;
}

Gwen::Point Base::CanvasPosToLocal( const Gwen::Point& pnt )
{
	if ( m_Parent )
	{
		int x = pnt.x - X();
		int y = pnt.y - Y();

		// If our parent has an innerpanel and we're a child of it
		// add its offset onto us.
		//
		if ( m_Parent->m_InnerPanel && m_Parent->m_InnerPanel->IsChild( this ) )
		{
			x -= m_Parent->m_InnerPanel->X();
			y -= m_Parent->m_InnerPanel->Y();
		}


		return m_Parent->CanvasPosToLocal( Gwen::Point( x, y ) );
	}

	return pnt;
}

bool Base::IsMenuComponent()
{
	if ( !m_Parent ) return false;
	return m_Parent->IsMenuComponent();
}

void Base::CloseMenus()
{
	for ( Base::List::iterator it = Children.begin(); it != Children.end(); ++it )
	{
		(*it)->CloseMenus();
	}
}

void Base::UpdateRenderBounds()
{
	m_RenderBounds.x = 0;
	m_RenderBounds.x = 0;

	m_RenderBounds.w = m_Bounds.w;
	m_RenderBounds.h = m_Bounds.h;
}

void Base::UpdateCursor()
{
	Platform::SetCursor( m_Cursor );
}

DragAndDrop::Package* Base::DragAndDrop_GetPackage( int /*x*/, int /*y*/ )
{
	return m_DragAndDrop_Package;
}

bool Base::DragAndDrop_HandleDrop( Gwen::DragAndDrop::Package* /*pPackage*/, int /*x*/, int /*y*/ )
{
	DragAndDrop::SourceControl->SetParent( this );
	return true;
}

bool Base::DragAndDrop_Draggable()
{ 
	if ( !m_DragAndDrop_Package ) return false;

	return m_DragAndDrop_Package->draggable; 
}

void Base::DragAndDrop_SetPackage( bool bDraggable, const String& strName, void* pUserData )
{
	if ( !m_DragAndDrop_Package )
	{
		m_DragAndDrop_Package = new Gwen::DragAndDrop::Package();
	}

	m_DragAndDrop_Package->draggable = bDraggable;
	m_DragAndDrop_Package->name = strName;
	m_DragAndDrop_Package->userdata = pUserData;
}

void Base::DragAndDrop_StartDragging( Gwen::DragAndDrop::Package* pPackage, int x, int y )
{
	pPackage->holdoffset = CanvasPosToLocal( Gwen::Point( x, y ) );
	pPackage->drawcontrol = this;
}

bool Base::SizeToChildren( bool w, bool h )
{
	Gwen::Point size = ChildrenSize();
	return SetSize( w ? size.x : Width(), h ? size.y : Height() );
}

Gwen::Point Base::ChildrenSize()
{
	Gwen::Point size;

	for (Base::List::iterator iter = Children.begin(); iter != Children.end(); ++iter)
	{
		Base* pChild = *iter;
		if ( pChild->Hidden() ) continue;

		size.x = GwenUtil_Max( size.x, pChild->Right() );
		size.y = GwenUtil_Max( size.y, pChild->Bottom() );
	}

	return size;
}

void Base::SetPadding( const Padding& padding )
{ 
	if ( m_Padding.left == padding.left &&
		 m_Padding.top == padding.top &&
		 m_Padding.right == padding.right &&
		 m_Padding.bottom == padding.bottom )
		 return;

	m_Padding = padding; 
	Invalidate();
	InvalidateParent();
}

void Base::SetMargin( const Margin& margin )
{ 
	if ( m_Margin.top == margin.top &&
		m_Margin.left == margin.left &&
		m_Margin.bottom == margin.bottom &&
		m_Margin.right == margin.right )
		return;

	m_Margin = margin;
	Invalidate();
	InvalidateParent(); 
}

bool Base::HandleAccelerator( Gwen::UnicodeString& accelerator )
{
	if ( Gwen::KeyboardFocus == this || !AccelOnlyFocus() )
	{
		AccelMap::iterator iter = m_Accelerators.find( accelerator );
		if ( iter != m_Accelerators.end() )
		{
			iter->second->Call( this );
			return true;
		}
	}
	
	for ( Base::List::iterator it = Children.begin(); it != Children.end(); ++it )
	{
		if ( (*it)->HandleAccelerator( accelerator ) )
			return true;
	}
	return false;
}

bool Base::OnKeyPress( int iKey, bool bPress )
{
	bool bHandled = false;
	switch ( iKey )
	{
		case Key::Tab:			bHandled = OnKeyTab( bPress ); break;
		case Key::Space:		bHandled = OnKeySpace( bPress ); break;
		case Key::Home:			bHandled = OnKeyHome( bPress ); break;
		case Key::End:			bHandled = OnKeyEnd( bPress ); break;
		case Key::Return:		bHandled = OnKeyReturn( bPress ); break;
		case Key::Backspace:	bHandled = OnKeyBackspace( bPress ); break;
		case Key::Delete:		bHandled = OnKeyDelete( bPress ); break;
		case Key::Right:		bHandled = OnKeyRight( bPress ); break;
		case Key::Left:			bHandled = OnKeyLeft( bPress ); break;
		case Key::Up:			bHandled = OnKeyUp( bPress ); break;
		case Key::Down:			bHandled = OnKeyDown( bPress ); break;
		case Key::Escape:		bHandled = OnKeyEscape( bPress ); break;

		default: break;
	}

	if ( !bHandled && GetParent() )
		GetParent()->OnKeyPress( iKey, bPress );

	return bHandled;
}

bool Base::OnKeyRelease( int iKey )
{
	return OnKeyPress( iKey, false );
}

bool Base::OnKeyTab( bool bDown )
{
	if ( !bDown ) return true;

	if ( GetCanvas()->NextTab )
	{
		GetCanvas()->NextTab->Focus();
		Redraw();
	}
	
	return true;
}

void Base::RenderFocus( Gwen::Skin::Base* skin )
{
	if ( Gwen::KeyboardFocus != this ) return;
	if ( !IsTabable() ) return;

	skin->DrawKeyboardHighlight( this, GetRenderBounds(), 3 );
}

void Base::SetToolTip( const String& strText )
{ 
	SetToolTip( Gwen::Utility::StringToUnicode( strText ) );
}

void Base::SetToolTip( const UnicodeString& strText )
{
	Label* tooltip = new Label( this );
	tooltip->SetText( strText );
	tooltip->SizeToContents();

	SetToolTip( tooltip );
}

#ifndef GWEN_NO_ANIMATION

void Base::Anim_WidthIn( float fLength, float fDelay, float fEase )
{
	Gwen::Anim::Add( this, new Gwen::Anim::Size::Width( 0, Width(), fLength, false, fDelay, fEase ) );
	SetWidth( 0 );
}

void Base::Anim_HeightIn( float fLength, float fDelay, float fEase )
{
	Gwen::Anim::Add( this, new Gwen::Anim::Size::Height( 0, Height(), fLength, false, fDelay, fEase ) );
	SetHeight( 0 );
}

void Base::Anim_WidthOut( float fLength, bool bHide, float fDelay, float fEase )
{
	Gwen::Anim::Add( this, new Gwen::Anim::Size::Width( Width(), 0, fLength, bHide, fDelay, fEase ) );
}

void Base::Anim_HeightOut( float fLength, bool bHide, float fDelay, float fEase )
{
	Gwen::Anim::Add( this, new Gwen::Anim::Size::Height( Height(), 0, fLength, bHide, fDelay, fEase ) );
}

#endif