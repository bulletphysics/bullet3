//	---------------------------------------------------------------------------
//
//	@file		TwBar.h
//	@brief		Tweak bar manager.
//	@author		Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//				Copyright © 2005, 2006 Philippe Decaudin.
//              For conditions of distribution and use, see License.txt
//
//	notes:		Private header
//				TAB=4
//
//	---------------------------------------------------------------------------


#if !defined ANT_TW_BAR_INCLUDED
#define ANT_TW_BAR_INCLUDED

#include <AntTweakBar.h>
#include "TwColors.h"
  
#define		ANT_TWEAK_BAR_DLL	"AntTweakBar"


//	---------------------------------------------------------------------------

struct CTwVar
{
	std::string				m_Name;
	std::string				m_Label;
	std::string				m_Help;
	bool					m_IsRoot;
	bool					m_DontClip;
	bool					m_Visible;
	signed short			m_LeftMargin;
	signed short			m_TopMargin;
	color32					m_Color;

	virtual bool			IsGroup() const = 0;
	virtual const CTwVar *	Find(const char *_Name, struct CTwVarGroup **_Parent, int *_Index) const = 0;
	virtual int				HasAttrib(const char *_Attrib, bool *_HasValue) const;
	virtual int				SetAttrib(int _AttribID, const char *_Value, TwBar *_Bar, struct CTwVarGroup *_VarParent, int _VarIndex);
	virtual	void			SetReadOnly(bool _ReadOnly) = 0;
							CTwVar()		{ m_IsRoot = false; m_DontClip = false; m_Visible = true; m_LeftMargin = 0; m_TopMargin = 0; m_Color = COLOR32_BLACK; }
	virtual					~CTwVar() 		{}

	static size_t			GetDataSize(TwType _Type);
};


struct CTwVarAtom : CTwVar
{
	ETwType					m_Type;
	void *					m_Ptr;
	TwSetVarCallback		m_SetCallback;
	TwGetVarCallback		m_GetCallback;
	TwButtonCallback		m_ButtonCallback;
	void *					m_ClientData;
	bool					m_ReadOnly;
	bool					m_NoSlider;
	int						m_KeyIncr[2];	// [0]=key_code [1]=modifiers
	int						m_KeyDecr[2];	// [0]=key_code [1]=modifiers

	template <typename _T>	struct TVal
	{
		_T					m_Min;
		_T					m_Max;
		_T					m_Step;
		signed char			m_Precision;
		bool				m_Hexa;
	};
	union UVal
	{
		TVal<unsigned char>	m_Char;
		TVal<signed char>	m_Int8;
		TVal<unsigned char>	m_UInt8;
		TVal<signed short>	m_Int16;
		TVal<unsigned short>m_UInt16;
		TVal<signed int>	m_Int32;
		TVal<unsigned int>	m_UInt32;
		TVal<float>			m_Float32;
		TVal<double>		m_Float64;
		struct CBoolVal
		{
			char *			m_TrueString;
			char *			m_FalseString;
			bool			m_FreeTrueString;
			bool			m_FreeFalseString;
		}					m_Bool;
		struct CEnumVal		// empty -> enum entries are deduced from m_Type
		{
			//typedef std::map<unsigned int, std::string> CEntries;
			//CEntries *	m_Entries;
		}					m_Enum;
		struct CShortcutVal
		{
			int				m_Incr[2];
			int				m_Decr[2];
		}					m_Shortcut;
		struct CHelpStruct
		{
			int				m_StructType;
		}					m_HelpStruct;
	};
	UVal					m_Val;

	virtual bool			IsGroup() const	{ return false; }
	virtual void			ValueToString(std::string *_Str) const;
	virtual double			ValueToDouble() const;
	virtual void			ValueFromDouble(double _Val);
	virtual void			MinMaxStepToDouble(double *_Min, double *_Max, double *_Step) const;
	virtual const CTwVar *	Find(const char *_Name, struct CTwVarGroup **_Parent, int *_Index) const;
	virtual int				HasAttrib(const char *_Attrib, bool *_HasValue) const;
	virtual int				SetAttrib(int _AttribID, const char *_Value, TwBar *_Bar, struct CTwVarGroup *_VarParent, int _VarIndex);
	virtual void			Increment(int _Step);
	virtual void			SetDefaults();
	virtual	void			SetReadOnly(bool _ReadOnly) { if(_ReadOnly || m_SetCallback || m_Ptr) m_ReadOnly=_ReadOnly; }
	//virtual int			DefineEnum(const TwEnumVal *_EnumValues, unsigned int _NbValues);
							CTwVarAtom();
	virtual					~CTwVarAtom();
};


struct CTwVarGroup : CTwVar
{
	std::vector<CTwVar *>	m_Vars;
	bool					m_Open;
	TwSummaryCallback		m_SummaryCallback;
	void *					m_SummaryClientData;
	void *					m_StructValuePtr;
	TwType					m_StructType;

	virtual bool			IsGroup() const	{ return true; }
	virtual const CTwVar *	Find(const char *_Name, CTwVarGroup **_Parent, int *_Index) const;
	virtual int				HasAttrib(const char *_Attrib, bool *_HasValue) const;
	virtual int				SetAttrib(int _AttribID, const char *_Value, TwBar *_Bar, struct CTwVarGroup *_VarParent, int _VarIndex);
	virtual CTwVarAtom *	FindShortcut(int _Key, int _Modifiers, bool *_DoIncr);
	virtual	void			SetReadOnly(bool _ReadOnly) { for(size_t i=0; i<m_Vars.size(); ++i) if(m_Vars[i]) m_Vars[i]->SetReadOnly(_ReadOnly); }
							CTwVarGroup()	{ m_Open=false; m_StructType=TW_TYPE_UNDEF; m_SummaryCallback=NULL; m_SummaryClientData=NULL; m_StructValuePtr=NULL; }
	virtual					~CTwVarGroup();
};

//	---------------------------------------------------------------------------

struct CTwBar
{
	std::string				m_Name;
	std::string				m_Label;
	std::string				m_Help;
	bool					m_Visible;
	int						m_PosX;
	int						m_PosY;
	int						m_Width;
	int						m_Height;
	color32					m_Color;
	const CTexFont *		m_Font;
	int						m_ValuesWidth;
	int						m_Sep;
	int						m_FirstLine;
	float					m_UpdatePeriod;
	bool					m_IsHelpBar;
	int						m_MinNumber;	// accessed by TwDeleteBar
	bool					m_IsPopupList;
	CTwVarAtom *			m_VarEnumLinkedToPopupList;
	CTwBar *				m_BarLinkedToPopupList;

	CTwVarGroup				m_VarRoot;

	void					NotUpToDate();
	void					Draw();
	const CTwVar *			Find(const char *_Name, CTwVarGroup **_Parent=NULL, int *_Index=NULL) const;
	CTwVar *				Find(const char *_Name, CTwVarGroup **_Parent=NULL, int *_Index=NULL);
	int						HasAttrib(const char *_Attrib, bool *_HasValue) const;
	int						SetAttrib(int _AttribID, const char *_Value);
	bool					MouseMotion(int _X, int _Y);
	bool					MouseButton(ETwMouseButtonID _Button, bool _Pressed, int _X, int _Y);
	bool					MouseWheel(int _Pos, int _PrevPos, int _MouseX, int _MouseY);
	bool					KeyPressed(int _Key, int _Modifiers);
	bool					IsMinimized() const { return m_IsMinimized; }
	bool					Show(CTwVar *_Var); // display the line associated to _Var
	bool					OpenHier(CTwVarGroup *_Root, CTwVar *_Var); // open a hierarchy if it contains _Var
	int						LineInHier(CTwVarGroup *_Root, CTwVar *_Var); // returns the number of the line associated to _Var
	void					UnHighlightLine() { m_HighlightedLine = -1; NotUpToDate(); }	// used by PopupCallback
							CTwBar(const char *_Name);
							~CTwBar();

	color32					m_ColBg, m_ColBg1, m_ColBg2;
	color32					m_ColHighBg;
	color32					m_ColLabelText;
	color32					m_ColStructText;
	color32					m_ColValBg;
	color32					m_ColValText;
	color32					m_ColValTextRO;
	color32					m_ColValMin;
	color32					m_ColValMax;
	color32					m_ColStructBg;
	color32					m_ColTitleBg;
	color32					m_ColTitleHighBg;
	color32					m_ColTitleText;
	color32					m_ColTitleShadow;
	color32					m_ColLine;
	color32					m_ColLineShadow;
	color32					m_ColUnderline;
	color32					m_ColBtn;
	color32					m_ColHighBtn;
	color32					m_ColGrpBg;
	color32					m_ColGrpText;
	color32					m_ColHierBg;
	color32					m_ColShortcutText;
	color32					m_ColShortcutBg;
	color32					m_ColInfoText;
	color32					m_ColHelpBg;
	color32					m_ColHelpText;
	color32					m_ColRoto;
	color32					m_ColRotoVal;
	color32					m_ColRotoBound;
	void					UpdateColors();

protected:
	int						m_TitleWidth;
	int						m_VarX0;
	int						m_VarX1;
	int						m_VarX2;
	int						m_VarY0;
	int						m_VarY1;
	int						m_VarY2;
	int						m_ScrollYW;
	int						m_ScrollYH;
	int						m_ScrollY0;
	int						m_ScrollY1;
	int						m_NbHierLines;
	int						m_NbDisplayedLines;
	bool					m_UpToDate;
	float					m_LastUpdateTime;
	void					Update();

	bool					m_MouseDrag;
	bool					m_MouseDragVar;
	bool					m_MouseDragTitle;
	bool					m_MouseDragScroll;
	bool					m_MouseDragResizeUR;
	bool					m_MouseDragResizeUL;
	bool					m_MouseDragResizeLR;
	bool					m_MouseDragResizeLL;
	bool					m_MouseDragValWidth;
	int						m_MouseOriginX;
	int						m_MouseOriginY;
	bool					m_VarHasBeenIncr;
	int						m_FirstLine0;
	int						m_HighlightedLine;
	int						m_HighlightedLinePrev;
	bool					m_HighlightIncrBtn;
	bool					m_HighlightDecrBtn;
	bool					m_HighlightClickBtn;
	bool					m_HighlightTitle;
	bool					m_HighlightScroll;
	bool					m_HighlightUpScroll;
	bool					m_HighlightDnScroll;
	bool					m_HighlightMinimize;
	bool					m_HighlightFont;
	bool					m_HighlightValWidth;
	bool					m_DrawHandles;

	bool					m_IsMinimized;
	int						m_MinPosX;
	int						m_MinPosY;
	bool					m_HighlightMaximize;
	bool					m_DrawIncrDecrBtn;
	bool					m_DrawClickBtn;

	struct CHierTag
	{
		CTwVar *			m_Var;
		int					m_Level;
		bool				m_Closing;
	};
	std::vector<CHierTag>	m_HierTags;
	void					BrowseHierarchy(int *_LineNum, int _CurrLevel, const CTwVar *_Var, int _First, int _Last);
	void *					m_TitleTextObj;
	void *					m_LabelsTextObj;
	void *					m_ValuesTextObj;
	void *					m_ShortcutTextObj;
	int						m_ShortcutLine;
	void					ListLabels(std::vector<std::string>& _Labels, std::vector<color32>& _Colors, const CTexFont *_Font, int _AtomWidthMax, int _GroupWidthMax);
	void					ListValues(std::vector<std::string>& _Values, std::vector<color32>& _Colors, std::vector<color32>& _BgColors, const CTexFont *_Font, int _WidthMax);
	void					DrawHierHandle();
  
	// RotoSlider
	struct	CPoint 
	{
		int					x, y;
							CPoint() {}
							CPoint(int _X, int _Y):x(_X), y(_Y) {}
		const CPoint		operator+ (const CPoint& p) const { return CPoint(x+p.x, y+p.y); }
		const CPoint		operator- (const CPoint& p) const { return CPoint(x-p.x, y-p.y); }
	};
	struct CRotoSlider
	{
							CRotoSlider();
		CTwVarAtom *		m_Var;
		double				m_PreciseValue;
		double				m_CurrentValue;
		double				m_Value0;
		double				m_ValueAngle0;
		bool				m_Active;
		bool				m_ActiveMiddle;
		CPoint				m_Origin;
		CPoint				m_Current;
		bool				m_HasPrevious;
		CPoint				m_Previous;
		double				m_Angle0;
		double				m_AngleDT;
		int					m_Subdiv;
	};
	CRotoSlider				m_Roto;
	int						m_RotoMinRadius;
	int						m_RotoNbSubdiv;	// number of steps for one turn
	void					RotoDraw();
	void					RotoOnMouseMove(int _X, int _Y);
	void					RotoOnLButtonDown(int _X, int _Y);
	void					RotoOnLButtonUp(int _X, int _Y);
	void					RotoOnMButtonDown(int _X, int _Y);
	void					RotoOnMButtonUp(int _X, int _Y);
	double					RotoGetValue() const;
	void					RotoSetValue(double _Val);
	double					RotoGetMin() const;
	double					RotoGetMax() const;
	double					RotoGetStep() const;
	double					RotoGetSteppedValue() const;

	friend struct CTwMgr;
};

//	---------------------------------------------------------------------------


#endif // !defined ANT_TW_BAR_INCLUDED
