/*
	GWEN
	Copyright (c) 2011 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_TREENODE_H
#define GWEN_CONTROLS_TREENODE_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Button.h"
#include "Gwen/Controls/ScrollControl.h"

enum
{
	ITERATE_ACTION_OPEN=1,
	ITERATE_ACTION_CLOSE,
	ITERATE_ACTION_FIND_SELECTED_INDEX,
	ITERATE_ACTION_DESELECT_INDEX,
	ITERATE_ACTION_SELECT,	
};

namespace Gwen 
{
	namespace Controls
	{
		class TreeControl;

		class GWEN_EXPORT TreeNode : public Base
		{
			public:

				GWEN_CONTROL( TreeNode, Base );

				virtual TreeNode* AddNode( const UnicodeString& strLabel );
				virtual TreeNode* AddNode( const String& strLabel );

				virtual void SetText( const UnicodeString& text );
				virtual void SetText( const String& text );
				UnicodeString GetText() const;

				virtual void Open();
				virtual void Close();

				virtual void ExpandAll();

				virtual Button* GetButton();

				virtual void Render( Skin::Base* skin );
				virtual void Layout( Skin::Base* skin );
				virtual void PostLayout( Skin::Base* skin );

				virtual void SetRoot( bool b ){ m_bRoot = b; }
				virtual void SetTreeControl( TreeControl* pCtrl ){ m_TreeControl = pCtrl; }

				virtual void SetSelectable( bool b ){ m_bSelectable = b; }
				virtual bool IsSelected(){ return m_bSelected; }
				virtual void SetSelected( bool b );

				virtual void DeselectAll();

				virtual void iterate(int action, int* curIndex, int* resultIndex);
				virtual bool OnKeyReturn(bool bDown)
				{
					static bool prevDown = false;
					if (!prevDown && bDown)
					{
						onReturnKeyDown.Call(this);
					}
					prevDown = bDown;
					return Base::OnKeyReturn(bDown);
				}

				Event::Caller	onReturnKeyDown;

				Event::Caller	onNamePress;
				Event::Caller	onSelectChange;
				Event::Caller	onSelect;
				Event::Caller	onUnselect;

			protected:

				void OnToggleButtonPress( Base* control );
				void OnDoubleClickName( Base* control );
				void OnClickName( Base* control );

				

				TreeControl*	m_TreeControl;
				Button*			m_ToggleButton;
				Button*			m_Title;

				bool			m_bRoot;
				bool			m_bSelected;
				bool			m_bSelectable;
				int			m_bUpdateScrollBar;

		};

	}
}
#endif
