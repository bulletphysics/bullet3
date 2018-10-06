/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_BASE_H
#define GWEN_CONTROLS_BASE_H

#include "Gwen/Exports.h"
#include "Gwen/Structures.h"
#include "Gwen/BaseRender.h"
#include "Gwen/Events.h"
#include <list>
#include <map>

#define GWEN_DECLARE_CAST(T)                        \
	virtual class T* DynamicCast##T() { return 0; } \
	virtual const class T* DynamicCast##T() const { return 0; }

#define GWEN_IMPLEMENT_CAST(T)                             \
	virtual class T* DynamicCast##T() { return (T*)this; } \
	virtual const class T* DynamicCast##T() const { return (T*)this; }

namespace Gwen
{
namespace ControlsInternal
{
class ColorDisplay;
class Resizer;

};  // namespace ControlsInternal

namespace Pos
{
enum
{
	None = 0,
	Left = (1 << 1),
	Right = (1 << 2),
	Top = (1 << 3),
	Bottom = (1 << 4),
	CenterV = (1 << 5),
	CenterH = (1 << 6),
	Fill = (1 << 7),
	Center = CenterV | CenterH,
};
}

namespace Skin
{
class Base;
}

namespace Controls
{
class Canvas;

namespace Layout
{
class TableRow;
};

class GWEN_EXPORT Base : public Event::Handler
{
public:
	GWEN_DECLARE_CAST(TabButton)
	GWEN_DECLARE_CAST(DockedTabControl)
	virtual class Layout::TableRow* DynamicCastLayoutTableRow() { return 0; }
	virtual const class Layout::TableRow* DynamicCastLayoutTableRow() const { return 0; }

	GWEN_DECLARE_CAST(TextBoxNumeric)
	GWEN_DECLARE_CAST(HorizontalSlider)
	GWEN_DECLARE_CAST(DockBase)
	GWEN_DECLARE_CAST(MenuItem)
	GWEN_DECLARE_CAST(PropertyRow)
	GWEN_DECLARE_CAST(WindowControl)

	GWEN_DECLARE_CAST(TreeControl)
	GWEN_DECLARE_CAST(TreeNode)
	GWEN_DECLARE_CAST(HSVColorPicker)
	GWEN_DECLARE_CAST(TabControl)

	GWEN_DECLARE_CAST(TabControlInner)
	GWEN_DECLARE_CAST(GroupBox)
	GWEN_DECLARE_CAST(Properties)
	GWEN_DECLARE_CAST(RadioButton)
	GWEN_DECLARE_CAST(LabeledRadioButton)

	virtual class ::Gwen::ControlsInternal::Resizer* DynamicCastResizer() { return 0; }
	virtual const class ::Gwen::ControlsInternal::Resizer* DynamicCastResizer() const { return 0; }

	virtual class ::Gwen::ControlsInternal::ColorDisplay* DynamicCastColorDisplay() { return 0; }
	virtual const class ::Gwen::ControlsInternal::ColorDisplay* DynamicCastColorDisplay() const { return 0; }

	typedef std::list<Base*> List;

	typedef std::map<Gwen::UnicodeString, Gwen::Event::Caller*> AccelMap;

	Base(Base* pParent);
	virtual ~Base();

	virtual void DelayedDelete();

	virtual void SetParent(Controls::Base* pParent);
	virtual Controls::Base* GetParent() const { return m_Parent; }
	virtual Controls::Canvas* GetCanvas();

	virtual Base::List& GetChildren()
	{
		if (m_InnerPanel) return m_InnerPanel->GetChildren();
		return Children;
	}
	virtual bool IsChild(Controls::Base* pChild);
	virtual int NumChildren();
	virtual bool SizeToChildren(bool w = true, bool h = true);
	virtual Gwen::Point ChildrenSize();
	virtual Controls::Base* FindChildByName(const Gwen::String& name, bool bRecursive = false);

	virtual void SetName(Gwen::String name) { m_Name = name; }
	virtual const Gwen::String& GetName() { return m_Name; }

	virtual void Think() {}
	virtual void ExpandAll() {}
	virtual void SizeToContents() {}
	virtual bool IsActive() { return false; }

	virtual void AddChild(Controls::Base* pChild);

	virtual void RemoveChild(Controls::Base* pParent);

protected:
	virtual void OnChildAdded(Controls::Base* pChild);
	virtual void OnChildRemoved(Controls::Base* pChild);

public:
	virtual void RemoveAllChildren();

	virtual void SendToBack(void);
	virtual void BringToFront(void);
	virtual void BringNextToControl(Controls::Base* pChild, bool bBehind);

	virtual Gwen::Point LocalPosToCanvas(const Gwen::Point& in);
	virtual Gwen::Point CanvasPosToLocal(const Gwen::Point& in);

	virtual void Dock(int iDock);
	virtual int GetDock();

	virtual void RestrictToParent(bool restrict) { m_bRestrictToParent = restrict; }
	virtual bool ShouldRestrictToParent() { return m_bRestrictToParent; }

	virtual int X() const
	{
		return m_Bounds.x;
	}
	virtual int Y() const
	{
		return m_Bounds.y;
	}
	virtual int Width() const
	{
		return m_Bounds.w;
	}
	virtual int Height() const
	{
		return m_Bounds.h;
	}
	virtual int Bottom() const
	{
		return m_Bounds.y + m_Bounds.h + m_Margin.bottom;
	}
	virtual int Right() const
	{
		return m_Bounds.x + m_Bounds.w + m_Margin.right;
	}

	virtual const Margin& GetMargin() const { return m_Margin; }
	virtual const Padding& GetPadding() const { return m_Padding; }

	virtual void SetPos(int x, int y);
	virtual void SetWidth(int w)
	{
		SetSize(w, Height());
	}
	virtual void SetHeight(int h)
	{
		SetSize(Width(), h);
	}
	virtual bool SetSize(int w, int h);
	virtual bool SetBounds(int x, int y, int w, int h);
	virtual bool SetBounds(const Gwen::Rect& bounds);

	virtual void SetPadding(const Padding& padding);
	virtual void SetMargin(const Margin& margin);

	// MoveTo is identical to SetPos except it uses ShouldRestrictToParent()
	virtual void MoveTo(int x, int y);
	virtual void MoveBy(int x, int y);

	virtual const Gwen::Rect& GetBounds() const
	{
		return m_Bounds;
	}

	virtual Controls::Base* GetControlAt(int x, int y);

protected:
	virtual void OnBoundsChanged(Gwen::Rect oldBounds);
	virtual void OnChildBoundsChanged(Gwen::Rect oldChildBounds, Base* pChild);

	virtual void OnScaleChanged();

public:
	// Innerbounds is the area inside the control that
	// doesn't have child controls docked to it.
	virtual const Gwen::Rect& GetInnerBounds() const { return m_InnerBounds; }

protected:
	Gwen::Rect m_InnerBounds;

public:
	virtual const Gwen::Rect& GetRenderBounds() const { return m_RenderBounds; }

protected:
	virtual void UpdateRenderBounds();

public:
	virtual void DoRender(Gwen::Skin::Base* skin);
	virtual void DoCacheRender(Gwen::Skin::Base* skin, Gwen::Controls::Base* pMaster);

protected:
	virtual void Render(Gwen::Skin::Base* skin);
	virtual void RenderUnder(Gwen::Skin::Base* /*skin*/){};
	virtual void RenderOver(Gwen::Skin::Base* /*skin*/){};
	virtual void RenderFocus(Gwen::Skin::Base* /*skin*/);

public:
	virtual void SetHidden(bool hidden)
	{
		if (m_bHidden == hidden)
			return;
		m_bHidden = hidden;
		Invalidate();
	}
	virtual bool Hidden() const;   // Returns true only if this control is hidden
	virtual bool Visible() const;  // Returns false if this control or its parents are hidden
	virtual void Hide() { SetHidden(true); }
	virtual void Show() { SetHidden(false); }

	//Skin
	virtual void SetSkin(Skin::Base* skin, bool doChildren = false);
	virtual Gwen::Skin::Base* GetSkin(void);

	// Background drawing
	virtual bool ShouldDrawBackground() { return m_bDrawBackground; }
	virtual void SetShouldDrawBackground(bool b) { m_bDrawBackground = b; }

protected:
	virtual void OnSkinChanged(Gwen::Skin::Base* newSkin);

public:
	virtual void OnMouseMoved(int x, int y, int deltaX, int deltaY);
	virtual bool OnMouseWheeled(int iDelta);
	virtual void OnMouseClickLeft(int /*x*/, int /*y*/, bool /*bDown*/){};
	virtual void OnMouseClickRight(int /*x*/, int /*y*/, bool /*bDown*/) {}
	virtual void OnMouseDoubleClickLeft(int x, int y) { OnMouseClickLeft(x, y, true); };
	virtual void OnMouseDoubleClickRight(int x, int y) { OnMouseClickRight(x, y, true); };
	virtual void OnLostKeyboardFocus() {}
	virtual void OnKeyboardFocus() {}

	virtual void SetMouseInputEnabled(bool b) { m_bMouseInputEnabled = b; }
	virtual bool GetMouseInputEnabled() { return m_bMouseInputEnabled; }

	virtual void SetKeyboardInputEnabled(bool b) { m_bKeyboardInputEnabled = b; }
	virtual bool GetKeyboardInputEnabled() const { return m_bKeyboardInputEnabled; }
	virtual bool NeedsInputChars() { return false; }

	virtual bool OnChar(Gwen::UnicodeChar /*c*/) { return false; }

	virtual bool OnKeyPress(int iKey, bool bPress = true);
	virtual bool OnKeyRelease(int iKey);

	virtual void OnPaste(Controls::Base* /*pFrom*/) {}
	virtual void OnCopy(Controls::Base* /*pFrom*/) {}
	virtual void OnCut(Controls::Base* /*pFrom*/) {}
	virtual void OnSelectAll(Controls::Base* /*pFrom*/) {}

	virtual bool OnKeyTab(bool bDown);
	virtual bool OnKeySpace(bool /*bDown*/) { return false; }
	virtual bool OnKeyReturn(bool /*bDown*/) { return false; }
	virtual bool OnKeyBackspace(bool /*bDown*/) { return false; }
	virtual bool OnKeyDelete(bool /*bDown*/) { return false; }
	virtual bool OnKeyRight(bool /*bDown*/) { return false; }
	virtual bool OnKeyLeft(bool /*bDown*/) { return false; }
	virtual bool OnKeyHome(bool /*bDown*/) { return false; }
	virtual bool OnKeyEnd(bool /*bDown*/) { return false; }
	virtual bool OnKeyUp(bool /*bDown*/) { return false; }
	virtual bool OnKeyDown(bool /*bDown*/) { return false; }
	virtual bool OnKeyEscape(bool /*bDown*/) { return false; }

	virtual void OnMouseEnter();
	virtual void OnMouseLeave();
	virtual bool IsHovered();
	virtual bool ShouldDrawHover();

	virtual void Touch();
	virtual void OnChildTouched(Controls::Base* pChild);

	virtual bool IsOnTop();

	virtual bool HasFocus();
	virtual void Focus();
	virtual void Blur();

	//Other
	virtual void SetDisabled(bool active) { m_bDisabled = active; }
	virtual bool IsDisabled() { return m_bDisabled; }

	virtual void Redraw()
	{
		m_bCacheTextureDirty = true;
		if (m_Parent) m_Parent->Redraw();
	}
	virtual void SetCacheToTexture() { m_bCacheToTexture = true; }
	virtual bool ShouldCacheToTexture() { return m_bCacheToTexture; }

	virtual void SetCursor(unsigned char c) { m_Cursor = c; }
	virtual void UpdateCursor();

	virtual Gwen::Point GetMinimumSize() { return Gwen::Point(1, 1); }
	virtual Gwen::Point GetMaximumSize() { return Gwen::Point(4096, 4096); }

	virtual void SetToolTip(const String& strText);
	virtual void SetToolTip(const UnicodeString& strText);
	virtual void SetToolTip(Base* tooltip)
	{
		m_ToolTip = tooltip;
		if (m_ToolTip)
		{
			m_ToolTip->SetParent(this);
			m_ToolTip->SetHidden(true);
		}
	}
	virtual Base* GetToolTip() { return m_ToolTip; }

	virtual bool IsMenuComponent();
	virtual void CloseMenus();

	virtual bool IsTabable() { return m_Tabable; }
	virtual void SetTabable(bool isTabable) { m_Tabable = isTabable; }

	//Accelerator functionality
	void DefaultAccel(Gwen::Controls::Base* /*pCtrl*/) { AcceleratePressed(); }
	virtual void AcceleratePressed(){};
	virtual bool AccelOnlyFocus() { return false; }
	virtual bool HandleAccelerator(Gwen::UnicodeString& accelerator);

	template <typename T>
	void AddAccelerator(Gwen::UnicodeString accelerator, T func, Gwen::Event::Handler* handler = NULL)
	{
		if (handler == NULL)
			handler = this;
		Gwen::Event::Caller* caller = new Gwen::Event::Caller();
		caller->Add(handler, func);
		m_Accelerators[accelerator] = caller;
	}

	void AddAccelerator(Gwen::UnicodeString accelerator)
	{
		AddAccelerator(accelerator, &Base::DefaultAccel, this);
	}

	AccelMap m_Accelerators;

	// Default Events

	Gwen::Event::Caller onHoverEnter;
	Gwen::Event::Caller onHoverLeave;

	// Childrens List

	Base::List Children;

protected:
	// The logical parent
	// It's usually what you expect, the control you've parented it to.
	Base* m_Parent;

	// If the innerpanel exists our children will automatically
	//  become children of that instead of us - allowing us to move
	//  them all around by moving that panel (useful for scrolling etc)
	Base* m_InnerPanel;

	// This is the panel's actual parent - most likely the logical
	//  parent's InnerPanel (if it has one). You should rarely need this.
	Base* m_ActualParent;

	Base* m_ToolTip;

	Skin::Base* m_Skin;

	Gwen::Rect m_Bounds;
	Gwen::Rect m_RenderBounds;
	Padding m_Padding;
	Margin m_Margin;

	Gwen::String m_Name;

	bool m_bRestrictToParent;
	bool m_bDisabled;
	bool m_bHidden;
	bool m_bMouseInputEnabled;
	bool m_bKeyboardInputEnabled;
	bool m_bDrawBackground;

	int m_iDock;

	unsigned char m_Cursor;

	bool m_Tabable;

public:
	bool NeedsLayout() { return m_bNeedsLayout; }
	void Invalidate();
	void InvalidateParent()
	{
		if (m_Parent)
		{
			m_Parent->Invalidate();
		}
	}
	void InvalidateChildren(bool bRecursive = false);
	void Position(int pos, int xpadding = 0, int ypadding = 0);

protected:
	virtual void RecurseLayout(Skin::Base* skin);
	virtual void Layout(Skin::Base* skin);
	virtual void PostLayout(Skin::Base* /*skin*/){};

	bool m_bNeedsLayout;
	bool m_bCacheTextureDirty;
	bool m_bCacheToTexture;

	//
	// Drag + Drop
public:
	// Giver

	virtual void DragAndDrop_SetPackage(bool bDraggable, const String& strName = "", void* pUserData = NULL);
	virtual bool DragAndDrop_Draggable();
	virtual bool DragAndDrop_ShouldStartDrag() { return true; }
	virtual void DragAndDrop_StartDragging(Gwen::DragAndDrop::Package* pPackage, int x, int y);
	virtual Gwen::DragAndDrop::Package* DragAndDrop_GetPackage(int x, int y);
	virtual void DragAndDrop_EndDragging(bool /*bSuccess*/, int /*x*/, int /*y*/){};

protected:
	DragAndDrop::Package* m_DragAndDrop_Package;

public:
	// Receiver
	virtual void DragAndDrop_HoverEnter(Gwen::DragAndDrop::Package* /*pPackage*/, int /*x*/, int /*y*/) {}
	virtual void DragAndDrop_HoverLeave(Gwen::DragAndDrop::Package* /*pPackage*/) {}
	virtual void DragAndDrop_Hover(Gwen::DragAndDrop::Package* /*pPackage*/, int /*x*/, int /*y*/){};
	virtual bool DragAndDrop_HandleDrop(Gwen::DragAndDrop::Package* pPackage, int x, int y);
	virtual bool DragAndDrop_CanAcceptPackage(Gwen::DragAndDrop::Package* /*pPackage*/) { return false; }

	//
	// This is to be used by the client implementation
	// NOT HOOKS ETC.
	//
public:
	void* GetUserData() { return m_pUserData; }
	void SetUserData(void* pData) { m_pUserData = pData; }

private:
	void* m_pUserData;

	//
	// Useful anim shortcuts
	//
public:
#ifndef GWEN_NO_ANIMATION

	virtual void Anim_WidthIn(float fLength, float fDelay = 0.0f, float fEase = 1.0f);
	virtual void Anim_HeightIn(float fLength, float fDelay = 0.0f, float fEase = 1.0f);
	virtual void Anim_WidthOut(float fLength, bool bHide = true, float fDelay = 0.0f, float fEase = 1.0f);
	virtual void Anim_HeightOut(float fLength, bool bHide = true, float fDelay = 0.0f, float fEase = 1.0f);

#endif
};

}  // namespace Controls
}  // namespace Gwen

// To be placed in the controls .h definition.
#define GWEN_CONTROL(ThisName, BaseName) \
public:                                  \
	typedef BaseName BaseClass;          \
	typedef ThisName ThisClass;          \
	GWEN_IMPLEMENT_CAST(ThisName);       \
	ThisName(Gwen::Controls::Base* pParent)

#define GWEN_CONTROL_INLINE(ThisName, BaseName) \
	GWEN_CONTROL(ThisName, BaseName) : BaseClass(pParent)

#define GWEN_CONTROL_CONSTRUCTOR(ThisName) \
	ThisName::ThisName(Gwen::Controls::Base* pParent) : BaseClass(pParent)

#endif
