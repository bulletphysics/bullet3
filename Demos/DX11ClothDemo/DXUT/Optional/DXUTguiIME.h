//--------------------------------------------------------------------------------------
// File: DXUTguiIME.h
//
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------
#pragma once
#ifndef DXUT_IME_H
#define DXUT_IME_H

#include <usp10.h>
#include <dimm.h>
#include <ImeUi.h>


//--------------------------------------------------------------------------------------
// Forward declarations
//--------------------------------------------------------------------------------------
class CDXUTIMEEditBox;


//-----------------------------------------------------------------------------
// IME-enabled EditBox control
//-----------------------------------------------------------------------------
#define MAX_COMPSTRING_SIZE 256


class CDXUTIMEEditBox : public CDXUTEditBox
{
public:

    static HRESULT          CreateIMEEditBox( CDXUTDialog* pDialog, int ID, LPCWSTR strText, int x, int y, int width,
                                              int height, bool bIsDefault=false, CDXUTIMEEditBox** ppCreated=NULL );

                            CDXUTIMEEditBox( CDXUTDialog* pDialog = NULL );
    virtual                 ~CDXUTIMEEditBox();

    static void             InitDefaultElements( CDXUTDialog* pDialog );

    static void WINAPI      Initialize( HWND hWnd );
    static void WINAPI      Uninitialize();

    static  HRESULT WINAPI  StaticOnCreateDevice();
    static  bool WINAPI     StaticMsgProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );

    static  void WINAPI     SetImeEnableFlag( bool bFlag );

    virtual void            Render( float fElapsedTime );
    virtual bool            MsgProc( UINT uMsg, WPARAM wParam, LPARAM lParam );
    virtual bool            HandleMouse( UINT uMsg, POINT pt, WPARAM wParam, LPARAM lParam );
    virtual void            UpdateRects();
    virtual void            OnFocusIn();
    virtual void            OnFocusOut();

    void                    PumpMessage();

    virtual void            RenderCandidateReadingWindow( float fElapsedTime, bool bReading );
    virtual void            RenderComposition( float fElapsedTime );
    virtual void            RenderIndicator( float fElapsedTime );

protected:
    static void WINAPI      EnableImeSystem( bool bEnable );

    static WORD WINAPI      GetLanguage()
    {
        return ImeUi_GetLanguage();
    }
    static WORD WINAPI      GetPrimaryLanguage()
    {
        return ImeUi_GetPrimaryLanguage();
    }
    static void WINAPI      SendKey( BYTE nVirtKey );
    static DWORD WINAPI     GetImeId( UINT uIndex = 0 )
    {
        return ImeUi_GetImeId( uIndex );
    };
    static void WINAPI      CheckInputLocale();
    static void WINAPI      CheckToggleState();
    static void WINAPI      SetupImeApi();
    static void WINAPI      ResetCompositionString();


    static void             SetupImeUiCallback();

protected:
    enum
    {
        INDICATOR_NON_IME,
        INDICATOR_CHS,
        INDICATOR_CHT,
        INDICATOR_KOREAN,
        INDICATOR_JAPANESE
    };

    struct CCandList
    {
        CUniBuffer HoriCand; // Candidate list string (for horizontal candidate window)
        int nFirstSelected; // First character position of the selected string in HoriCand
        int nHoriSelectedLen; // Length of the selected string in HoriCand
        RECT rcCandidate;   // Candidate rectangle computed and filled each time before rendered
    };

    static POINT s_ptCompString;        // Composition string position. Updated every frame.
    static int s_nFirstTargetConv;    // Index of the first target converted char in comp string.  If none, -1.
    static CUniBuffer s_CompString;       // Buffer to hold the composition string (we fix its length)
    static DWORD            s_adwCompStringClause[MAX_COMPSTRING_SIZE];
    static CCandList s_CandList;          // Data relevant to the candidate list
    static WCHAR            s_wszReadingString[32];// Used only with horizontal reading window (why?)
    static bool s_bImeFlag;			  // Is ime enabled 
	
    // Color of various IME elements
    D3DCOLOR m_ReadingColor;        // Reading string color
    D3DCOLOR m_ReadingWinColor;     // Reading window color
    D3DCOLOR m_ReadingSelColor;     // Selected character in reading string
    D3DCOLOR m_ReadingSelBkColor;   // Background color for selected char in reading str
    D3DCOLOR m_CandidateColor;      // Candidate string color
    D3DCOLOR m_CandidateWinColor;   // Candidate window color
    D3DCOLOR m_CandidateSelColor;   // Selected candidate string color
    D3DCOLOR m_CandidateSelBkColor; // Selected candidate background color
    D3DCOLOR m_CompColor;           // Composition string color
    D3DCOLOR m_CompWinColor;        // Composition string window color
    D3DCOLOR m_CompCaretColor;      // Composition string caret color
    D3DCOLOR m_CompTargetColor;     // Composition string target converted color
    D3DCOLOR m_CompTargetBkColor;   // Composition string target converted background
    D3DCOLOR m_CompTargetNonColor;  // Composition string target non-converted color
    D3DCOLOR m_CompTargetNonBkColor;// Composition string target non-converted background
    D3DCOLOR m_IndicatorImeColor;   // Indicator text color for IME
    D3DCOLOR m_IndicatorEngColor;   // Indicator text color for English
    D3DCOLOR m_IndicatorBkColor;    // Indicator text background color

    // Edit-control-specific data
    int m_nIndicatorWidth;     // Width of the indicator symbol
    RECT m_rcIndicator;         // Rectangle for drawing the indicator button

#if defined(DEBUG) || defined(_DEBUG)
    static bool    m_bIMEStaticMsgProcCalled;
#endif
};



#endif // DXUT_IME_H
