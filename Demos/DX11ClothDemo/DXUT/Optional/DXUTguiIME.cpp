//--------------------------------------------------------------------------------------
// File: DXUTguiIME.cpp
//
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------
#include "DXUT.h"
#include "DXUTgui.h"
#include "DXUTsettingsDlg.h"
#include "DXUTres.h"
#include "DXUTgui.h"
#include "DXUTguiIME.h"

#undef min // use __min instead
#undef max // use __max instead
#define DXUT_NEAR_BUTTON_DEPTH 0.6f


//--------------------------------------------------------------------------------------
// CDXUTIMEEditBox class
//--------------------------------------------------------------------------------------
// IME constants

POINT                       CDXUTIMEEditBox::s_ptCompString;      // Composition string position. Updated every frame.
int                         CDXUTIMEEditBox::s_nFirstTargetConv;  // Index of the first target converted char in comp string.  If none, -1.
CUniBuffer                  CDXUTIMEEditBox::s_CompString = CUniBuffer( 0 );
DWORD CDXUTIMEEditBox::s_adwCompStringClause[MAX_COMPSTRING_SIZE];
WCHAR CDXUTIMEEditBox::s_wszReadingString[32];
CDXUTIMEEditBox::CCandList  CDXUTIMEEditBox::s_CandList;       // Data relevant to the candidate list
bool                        CDXUTIMEEditBox::s_bImeFlag = true;


#if defined(DEBUG) || defined(_DEBUG)
bool      CDXUTIMEEditBox::m_bIMEStaticMsgProcCalled = false;
#endif


//--------------------------------------------------------------------------------------
HRESULT CDXUTIMEEditBox::CreateIMEEditBox( CDXUTDialog* pDialog, int ID, LPCWSTR strText, int x, int y, int width,
                                           int height, bool bIsDefault, CDXUTIMEEditBox** ppCreated )
{
    CDXUTIMEEditBox* pEditBox = new CDXUTIMEEditBox( pDialog );

    if( ppCreated != NULL )
        *ppCreated = pEditBox;

    if( pEditBox == NULL )
        return E_OUTOFMEMORY;

    // Set the ID and position
    pEditBox->SetID( ID );
    pEditBox->SetLocation( x, y );
    pEditBox->SetSize( width, height );
    pEditBox->m_bIsDefault = bIsDefault;

    if( strText )
        pEditBox->SetText( strText );

    return S_OK;
}


//--------------------------------------------------------------------------------------
void CDXUTIMEEditBox::InitDefaultElements( CDXUTDialog* pDialog )
{
    //-------------------------------------
    // CDXUTIMEEditBox
    //-------------------------------------

    CDXUTElement Element;
    RECT rcTexture;

    Element.SetFont( 0, D3DCOLOR_ARGB( 255, 0, 0, 0 ), DT_LEFT | DT_TOP );

    // Assign the style
    SetRect( &rcTexture, 14, 90, 241, 113 );
    Element.SetTexture( 0, &rcTexture );
    pDialog->SetDefaultElement( DXUT_CONTROL_IMEEDITBOX, 0, &Element );
    SetRect( &rcTexture, 8, 82, 14, 90 );
    Element.SetTexture( 0, &rcTexture );
    pDialog->SetDefaultElement( DXUT_CONTROL_IMEEDITBOX, 1, &Element );
    SetRect( &rcTexture, 14, 82, 241, 90 );
    Element.SetTexture( 0, &rcTexture );
    pDialog->SetDefaultElement( DXUT_CONTROL_IMEEDITBOX, 2, &Element );
    SetRect( &rcTexture, 241, 82, 246, 90 );
    Element.SetTexture( 0, &rcTexture );
    pDialog->SetDefaultElement( DXUT_CONTROL_IMEEDITBOX, 3, &Element );
    SetRect( &rcTexture, 8, 90, 14, 113 );
    Element.SetTexture( 0, &rcTexture );
    pDialog->SetDefaultElement( DXUT_CONTROL_IMEEDITBOX, 4, &Element );
    SetRect( &rcTexture, 241, 90, 246, 113 );
    Element.SetTexture( 0, &rcTexture );
    pDialog->SetDefaultElement( DXUT_CONTROL_IMEEDITBOX, 5, &Element );
    SetRect( &rcTexture, 8, 113, 14, 121 );
    Element.SetTexture( 0, &rcTexture );
    pDialog->SetDefaultElement( DXUT_CONTROL_IMEEDITBOX, 6, &Element );
    SetRect( &rcTexture, 14, 113, 241, 121 );
    Element.SetTexture( 0, &rcTexture );
    pDialog->SetDefaultElement( DXUT_CONTROL_IMEEDITBOX, 7, &Element );
    SetRect( &rcTexture, 241, 113, 246, 121 );
    Element.SetTexture( 0, &rcTexture );
    pDialog->SetDefaultElement( DXUT_CONTROL_IMEEDITBOX, 8, &Element );
    // Element 9 for IME text, and indicator button
    SetRect( &rcTexture, 0, 0, 136, 54 );
    Element.SetTexture( 0, &rcTexture );
    Element.SetFont( 0, D3DCOLOR_ARGB( 255, 0, 0, 0 ), DT_CENTER | DT_VCENTER );
    pDialog->SetDefaultElement( DXUT_CONTROL_IMEEDITBOX, 9, &Element );
}


//--------------------------------------------------------------------------------------
CDXUTIMEEditBox::CDXUTIMEEditBox( CDXUTDialog* pDialog )
{
    m_Type = DXUT_CONTROL_IMEEDITBOX;
    m_pDialog = pDialog;

    m_nIndicatorWidth = 0;
    m_ReadingColor = D3DCOLOR_ARGB( 188, 255, 255, 255 );
    m_ReadingWinColor = D3DCOLOR_ARGB( 128, 0, 0, 0 );
    m_ReadingSelColor = D3DCOLOR_ARGB( 255, 255, 0, 0 );
    m_ReadingSelBkColor = D3DCOLOR_ARGB( 128, 80, 80, 80 );
    m_CandidateColor = D3DCOLOR_ARGB( 255, 200, 200, 200 );
    m_CandidateWinColor = D3DCOLOR_ARGB( 128, 0, 0, 0 );
    m_CandidateSelColor = D3DCOLOR_ARGB( 255, 255, 255, 255 );
    m_CandidateSelBkColor = D3DCOLOR_ARGB( 128, 158, 158, 158 );
    m_CompColor = D3DCOLOR_ARGB( 255, 200, 200, 255 );
    m_CompWinColor = D3DCOLOR_ARGB( 198, 0, 0, 0 );
    m_CompCaretColor = D3DCOLOR_ARGB( 255, 255, 255, 255 );
    m_CompTargetColor = D3DCOLOR_ARGB( 255, 255, 255, 255 );
    m_CompTargetBkColor = D3DCOLOR_ARGB( 255, 150, 150, 150 );
    m_CompTargetNonColor = D3DCOLOR_ARGB( 255, 255, 255, 0 );
    m_CompTargetNonBkColor = D3DCOLOR_ARGB( 255, 150, 150, 150 );
    m_IndicatorImeColor = D3DCOLOR_ARGB( 255, 255, 255, 255 );
    m_IndicatorEngColor = D3DCOLOR_ARGB( 255, 0, 0, 0 );
    m_IndicatorBkColor = D3DCOLOR_ARGB( 255, 128, 128, 128 );
}


//--------------------------------------------------------------------------------------
CDXUTIMEEditBox::~CDXUTIMEEditBox()
{
}


//--------------------------------------------------------------------------------------
void CDXUTIMEEditBox::SendKey( BYTE nVirtKey )
{
    keybd_event( nVirtKey, 0, 0, 0 );
    keybd_event( nVirtKey, 0, KEYEVENTF_KEYUP, 0 );
}


//--------------------------------------------------------------------------------------
void CDXUTIMEEditBox::UpdateRects()
{
    // Temporary adjust m_width so that CDXUTEditBox can compute
    // the correct rects for its rendering since we need to make space
    // for the indicator button
    int nWidth = m_width;
    m_width -= m_nIndicatorWidth + m_nBorder * 2; // Make room for the indicator button
    CDXUTEditBox::UpdateRects();
    m_width = nWidth;  // Restore

    // Compute the indicator button rectangle
    SetRect( &m_rcIndicator, m_rcBoundingBox.right, m_rcBoundingBox.top, m_x + m_width, m_rcBoundingBox.bottom );
    //    InflateRect( &m_rcIndicator, -m_nBorder, -m_nBorder );
    m_rcBoundingBox.right = m_rcBoundingBox.left + m_width;
}


//--------------------------------------------------------------------------------------
//  GetImeId( UINT uIndex )
//      returns 
//  returned value:
//  0: In the following cases
//      - Non Chinese IME input locale
//      - Older Chinese IME
//      - Other error cases
//
//  Othewise:
//      When uIndex is 0 (default)
//          bit 31-24:  Major version
//          bit 23-16:  Minor version
//          bit 15-0:   Language ID
//      When uIndex is 1
//          pVerFixedInfo->dwFileVersionLS
//
//  Use IMEID_VER and IMEID_LANG macro to extract version and language information.
//  

// We define the locale-invariant ID ourselves since it doesn't exist prior to WinXP
// For more information, see the CompareString() reference.
#define LCID_INVARIANT MAKELCID(MAKELANGID(LANG_ENGLISH, SUBLANG_ENGLISH_US), SORT_DEFAULT)
//--------------------------------------------------------------------------------------
// Enable/disable the entire IME system.  When disabled, the default IME handling
// kicks in.
void CDXUTIMEEditBox::EnableImeSystem( bool bEnable )
{
    ImeUi_EnableIme( bEnable );
}


//--------------------------------------------------------------------------------------
// Resets the composition string.
void CDXUTIMEEditBox::ResetCompositionString()
{
    s_CompString.SetText( L"" );
}


//--------------------------------------------------------------------------------------
// This function is used only briefly in CHT IME handling,
// so accelerator isn't processed.
void CDXUTIMEEditBox::PumpMessage()
{
    MSG msg;

    while( PeekMessageW( &msg, NULL, 0, 0, PM_NOREMOVE ) )
    {
        if( !GetMessageW( &msg, NULL, 0, 0 ) )
        {
            PostQuitMessage( ( int )msg.wParam );
            return;
        }
        TranslateMessage( &msg );
        DispatchMessageA( &msg );
    }
}


//--------------------------------------------------------------------------------------
void CDXUTIMEEditBox::OnFocusIn()
{
    ImeUi_EnableIme( s_bImeFlag );
    CDXUTEditBox::OnFocusIn();
}


//--------------------------------------------------------------------------------------
void CDXUTIMEEditBox::OnFocusOut()
{
    ImeUi_FinalizeString();
    ImeUi_EnableIme( false );
    CDXUTEditBox::OnFocusOut();
}


//--------------------------------------------------------------------------------------
bool CDXUTIMEEditBox::StaticMsgProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{

    if( !ImeUi_IsEnabled() )
        return false;

#if defined(DEBUG) || defined(_DEBUG)
    m_bIMEStaticMsgProcCalled = true;
#endif

    switch( uMsg )
    {
        case WM_INPUTLANGCHANGE:
            DXUTTRACE( L"WM_INPUTLANGCHANGE\n" );
            {
            }
            return true;

        case WM_IME_SETCONTEXT:
            DXUTTRACE( L"WM_IME_SETCONTEXT\n" );
            //
            // We don't want anything to display, so we have to clear this
            //
            lParam = 0;
            return false;

            // Handle WM_IME_STARTCOMPOSITION here since
            // we do not want the default IME handler to see
            // this when our fullscreen app is running.
        case WM_IME_STARTCOMPOSITION:
            DXUTTRACE( L"WM_IME_STARTCOMPOSITION\n" );
            ResetCompositionString();
            // Since the composition string has its own caret, we don't render
            // the edit control's own caret to avoid double carets on screen.
            s_bHideCaret = true;
            return true;
        case WM_IME_ENDCOMPOSITION:
            DXUTTRACE( L"WM_IME_ENDCOMPOSITION\n" );
            s_bHideCaret = false;
            return false;
        case WM_IME_COMPOSITION:
            DXUTTRACE( L"WM_IME_COMPOSITION\n" );
            return false;
    }

    return false;
}


//--------------------------------------------------------------------------------------
bool CDXUTIMEEditBox::HandleMouse( UINT uMsg, POINT pt, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;
	
    switch( uMsg )
    {
        case WM_LBUTTONDOWN:
        case WM_LBUTTONDBLCLK:
            {
                DXUTFontNode* pFont = m_pDialog->GetFont( m_Elements.GetAt( 9 )->iFont );

                // Check if this click is on top of the composition string
                int nCompStrWidth;
                s_CompString.CPtoX( s_CompString.GetTextSize(), FALSE, &nCompStrWidth );

                if( s_ptCompString.x <= pt.x &&
                    s_ptCompString.y <= pt.y &&
                    s_ptCompString.x + nCompStrWidth > pt.x &&
                    s_ptCompString.y + pFont->nHeight > pt.y )
                {
                    int nCharBodyHit, nCharHit;
                    int nTrail;

                    // Determine the character clicked on.
                    s_CompString.XtoCP( pt.x - s_ptCompString.x, &nCharBodyHit, &nTrail );
                    if( nTrail && nCharBodyHit < s_CompString.GetTextSize() )
                        nCharHit = nCharBodyHit + 1;
                    else
                        nCharHit = nCharBodyHit;


                    switch( GetPrimaryLanguage() )
                    {
                        case LANG_JAPANESE:
                            // For Japanese, there are two cases.  If s_nFirstTargetConv is
                            // -1, the comp string hasn't been converted yet, and we use
                            // s_nCompCaret.  For any other value of s_nFirstTargetConv,
                            // the string has been converted, so we use clause information.

                            if( s_nFirstTargetConv != -1 )
                            {
                                int nClauseClicked = 0;
                                while( ( int )s_adwCompStringClause[nClauseClicked + 1] <= nCharBodyHit )
                                    ++nClauseClicked;

                                int nClauseSelected = 0;
                                while( ( int )s_adwCompStringClause[nClauseSelected + 1] <= s_nFirstTargetConv )
                                    ++nClauseSelected;

                                BYTE nVirtKey = nClauseClicked > nClauseSelected ? VK_RIGHT : VK_LEFT;
                                int nSendCount = abs( nClauseClicked - nClauseSelected );
                                while( nSendCount-- > 0 )
                                    SendKey( nVirtKey );

                                return true;
                            }

                            // Not converted case. Fall thru to Chinese case.

                        case LANG_CHINESE:
                        {
                            // For Chinese, use s_nCompCaret.
                            BYTE nVirtKey = nCharHit > ( int )ImeUi_GetImeCursorChars() ? VK_RIGHT : VK_LEFT;
                            int nSendCount = abs( nCharHit - ( int )ImeUi_GetImeCursorChars() );
                            while( nSendCount-- > 0 )
                                SendKey( nVirtKey );
                            break;
                        }
                    }

                    return true;
                }

                // Check if the click is on top of the candidate window
                if( ImeUi_IsShowCandListWindow() && PtInRect( &s_CandList.rcCandidate, pt ) )
                {
                    if( ImeUi_IsVerticalCand() )
                    {
                        // Vertical candidate window

                        // Compute the row the click is on
                        int nRow = ( pt.y - s_CandList.rcCandidate.top ) / pFont->nHeight;

                        if( nRow < ( int )ImeUi_GetCandidateCount() )
                        {
                            // nRow is a valid entry.
                            // Now emulate keystrokes to select the candidate at this row.
                            switch( GetPrimaryLanguage() )
                            {
                                case LANG_CHINESE:
                                case LANG_KOREAN:
                                    // For Chinese and Korean, simply send the number keystroke.
                                    SendKey( ( BYTE )( '0' + nRow + 1 ) );
                                    break;

                                case LANG_JAPANESE:
                                    // For Japanese, move the selection to the target row,
                                    // then send Right, then send Left.

                                    BYTE nVirtKey;
                                    if( nRow > ( int )ImeUi_GetCandidateSelection() )
                                        nVirtKey = VK_DOWN;
                                    else
                                        nVirtKey = VK_UP;
                                    int nNumToHit = abs( int( nRow - ImeUi_GetCandidateSelection() ) );
                                    for( int nStrike = 0; nStrike < nNumToHit; ++nStrike )
                                        SendKey( nVirtKey );

                                    // Do this to close the candidate window without ending composition.
                                    SendKey( VK_RIGHT );
                                    SendKey( VK_LEFT );

                                    break;
                            }
                        }
                    }
                    else
                    {
                        // Horizontal candidate window

                        // Determine which the character the click has hit.
                        int nCharHit;
                        int nTrail;
                        s_CandList.HoriCand.XtoCP( pt.x - s_CandList.rcCandidate.left, &nCharHit, &nTrail );

                        // Determine which candidate string the character belongs to.
                        int nCandidate = ImeUi_GetCandidateCount() - 1;

                        int nEntryStart = 0;
                        for( UINT i = 0; i < ImeUi_GetCandidateCount(); ++i )
                        {
                            if( nCharHit >= nEntryStart )
                            {
                                // Haven't found it.
                                nEntryStart += lstrlenW( ImeUi_GetCandidate( i ) ) + 1;  // plus space separator
                            }
                            else
                            {
                                // Found it.  This entry starts at the right side of the click point,
                                // so the char belongs to the previous entry.
                                nCandidate = i - 1;
                                break;
                            }
                        }

                        // Now emulate keystrokes to select the candidate entry.
                        switch( GetPrimaryLanguage() )
                        {
                            case LANG_CHINESE:
                            case LANG_KOREAN:
                                // For Chinese and Korean, simply send the number keystroke.
                                SendKey( ( BYTE )( '0' + nCandidate + 1 ) );
                                break;
                        }
                    }

                    return true;
                }
            }
    }

    // If we didn't care for the msg, let the parent process it.
    return CDXUTEditBox::HandleMouse( uMsg, pt, wParam, lParam );
}


//--------------------------------------------------------------------------------------
bool CDXUTIMEEditBox::MsgProc( UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

#if defined(DEBUG) || defined(_DEBUG)
    // DXUT.cpp used to call CDXUTIMEEditBox::StaticMsgProc() so that, but now
    // this is the application's responsiblity.  To do this, call 
    // CDXUTDialogResourceManager::MsgProc() before calling this function.
    assert( m_bIMEStaticMsgProcCalled && L"To fix, call CDXUTDialogResourceManager::MsgProc() first" );
#endif
    switch( uMsg )
    {
        case WM_DESTROY:
            ImeUi_Uninitialize();
            break;
    }	

    bool trappedData;
    bool* trapped = &trappedData;

    *trapped = false;
    if( !ImeUi_IsEnabled() )
        return CDXUTEditBox::MsgProc( uMsg, wParam, lParam );

    ImeUi_ProcessMessage( DXUTGetHWND(), uMsg, wParam, lParam, trapped );
    if( *trapped == false )
        CDXUTEditBox::MsgProc( uMsg, wParam, lParam );

    return *trapped;
}


//--------------------------------------------------------------------------------------
void CDXUTIMEEditBox::RenderCandidateReadingWindow( float fElapsedTime, bool bReading )
{
    RECT rc;
    UINT nNumEntries = bReading ? 4 : MAX_CANDLIST;
    D3DCOLOR TextColor, TextBkColor, SelTextColor, SelBkColor;
    int nX, nXFirst, nXComp;
    m_Buffer.CPtoX( m_nCaret, FALSE, &nX );
    m_Buffer.CPtoX( m_nFirstVisible, FALSE, &nXFirst );

    if( bReading )
    {
        TextColor = m_ReadingColor;
        TextBkColor = m_ReadingWinColor;
        SelTextColor = m_ReadingSelColor;
        SelBkColor = m_ReadingSelBkColor;
    }
    else
    {
        TextColor = m_CandidateColor;
        TextBkColor = m_CandidateWinColor;
        SelTextColor = m_CandidateSelColor;
        SelBkColor = m_CandidateSelBkColor;
    }

    // For Japanese IME, align the window with the first target converted character.
    // For all other IMEs, align with the caret.  This is because the caret
    // does not move for Japanese IME.
    if( GetLanguage() == MAKELANGID( LANG_CHINESE, SUBLANG_CHINESE_TRADITIONAL ) && !GetImeId() )
        nXComp = 0;
    else if( GetPrimaryLanguage() == LANG_JAPANESE )
        s_CompString.CPtoX( s_nFirstTargetConv, FALSE, &nXComp );
    else
        s_CompString.CPtoX( ImeUi_GetImeCursorChars(), FALSE, &nXComp );

    // Compute the size of the candidate window
    int nWidthRequired = 0;
    int nHeightRequired = 0;
    int nSingleLineHeight = 0;

    if( ( ImeUi_IsVerticalCand() && !bReading ) ||
        ( !ImeUi_IsHorizontalReading() && bReading ) )
    {
        // Vertical window
        for( UINT i = 0; i < nNumEntries; ++i )
        {
            if( *( ImeUi_GetCandidate( i ) ) == L'\0' )
                break;
            SetRect( &rc, 0, 0, 0, 0 );
            m_pDialog->CalcTextRect( ImeUi_GetCandidate( i ), m_Elements.GetAt( 1 ), &rc );
            nWidthRequired = __max( nWidthRequired, rc.right - rc.left );
            nSingleLineHeight = __max( nSingleLineHeight, rc.bottom - rc.top );
        }
        nHeightRequired = nSingleLineHeight * nNumEntries;
    }
    else
    {
        // Horizontal window
        SetRect( &rc, 0, 0, 0, 0 );
        if( bReading )
            m_pDialog->CalcTextRect( s_wszReadingString, m_Elements.GetAt( 1 ), &rc );
        else
        {

            WCHAR wszCand[256] = L"";

            s_CandList.nFirstSelected = 0;
            s_CandList.nHoriSelectedLen = 0;
            for( UINT i = 0; i < MAX_CANDLIST; ++i )
            {
                if( *ImeUi_GetCandidate( i ) == L'\0' )
                    break;

                WCHAR wszEntry[32];
                swprintf_s( wszEntry, 32, L"%s ", ImeUi_GetCandidate( i ) );
                // If this is the selected entry, mark its char position.
                if( ImeUi_GetCandidateSelection() == i )
                {
                    s_CandList.nFirstSelected = lstrlen( wszCand );
                    s_CandList.nHoriSelectedLen = lstrlen( wszEntry ) - 1;  // Minus space
                }
                wcscat_s( wszCand, 256, wszEntry );
            }
            wszCand[lstrlen( wszCand ) - 1] = L'\0';  // Remove the last space
            s_CandList.HoriCand.SetText( wszCand );
			
            m_pDialog->CalcTextRect( s_CandList.HoriCand.GetBuffer(), m_Elements.GetAt( 1 ), &rc );
        }
        nWidthRequired = rc.right - rc.left;
        nSingleLineHeight = nHeightRequired = rc.bottom - rc.top;
    }

    // Now that we have the dimension, calculate the location for the candidate window.
    // We attempt to fit the window in this order:
    // bottom, top, right, left.

    bool bHasPosition = false;

    // Bottom
    SetRect( &rc, s_ptCompString.x + nXComp, s_ptCompString.y + m_rcText.bottom - m_rcText.top,
             s_ptCompString.x + nXComp + nWidthRequired, s_ptCompString.y + m_rcText.bottom - m_rcText.top +
             nHeightRequired );
    // if the right edge is cut off, move it left.
    if( rc.right > m_pDialog->GetWidth() )
    {
        rc.left -= rc.right - m_pDialog->GetWidth();
        rc.right = m_pDialog->GetWidth();
    }
    if( rc.bottom <= m_pDialog->GetHeight() )
        bHasPosition = true;

    // Top
    if( !bHasPosition )
    {
        SetRect( &rc, s_ptCompString.x + nXComp, s_ptCompString.y - nHeightRequired,
                 s_ptCompString.x + nXComp + nWidthRequired, s_ptCompString.y );
        // if the right edge is cut off, move it left.
        if( rc.right > m_pDialog->GetWidth() )
        {
            rc.left -= rc.right - m_pDialog->GetWidth();
            rc.right = m_pDialog->GetWidth();
        }
        if( rc.top >= 0 )
            bHasPosition = true;
    }

    // Right
    if( !bHasPosition )
    {
        int nXCompTrail;
        s_CompString.CPtoX( ImeUi_GetImeCursorChars(), TRUE, &nXCompTrail );
        SetRect( &rc, s_ptCompString.x + nXCompTrail, 0,
                 s_ptCompString.x + nXCompTrail + nWidthRequired, nHeightRequired );
        if( rc.right <= m_pDialog->GetWidth() )
            bHasPosition = true;
    }

    // Left
    if( !bHasPosition )
    {
        SetRect( &rc, s_ptCompString.x + nXComp - nWidthRequired, 0,
                 s_ptCompString.x + nXComp, nHeightRequired );
        if( rc.right >= 0 )
            bHasPosition = true;
    }

    if( !bHasPosition )
    {
        // The dialog is too small for the candidate window.
        // Fall back to render at 0, 0.  Some part of the window
        // will be cut off.
        rc.left = 0;
        rc.right = nWidthRequired;
    }

    // If we are rendering the candidate window, save the position
    // so that mouse clicks are checked properly.
    if( !bReading )
        s_CandList.rcCandidate = rc;

    // Render the elements
    m_pDialog->DrawRect( &rc, TextBkColor );
    if( ( ImeUi_IsVerticalCand() && !bReading ) ||
        ( !ImeUi_IsHorizontalReading() && bReading ) )
    {
        // Vertical candidate window
        for( UINT i = 0; i < nNumEntries; ++i )
        {
            // Here we are rendering one line at a time
            rc.bottom = rc.top + nSingleLineHeight;
            // Use a different color for the selected string
            if( ImeUi_GetCandidateSelection() == i )
            {
                m_pDialog->DrawRect( &rc, SelBkColor );
                m_Elements.GetAt( 1 )->FontColor.Current = SelTextColor;
            }
            else
                m_Elements.GetAt( 1 )->FontColor.Current = TextColor;

            m_pDialog->DrawText( ImeUi_GetCandidate( i ), m_Elements.GetAt( 1 ), &rc );

            rc.top += nSingleLineHeight;
        }
    }
    else
    {
        // Horizontal candidate window
        m_Elements.GetAt( 1 )->FontColor.Current = TextColor;
        if( bReading )
            m_pDialog->DrawText( s_wszReadingString, m_Elements.GetAt( 1 ), &rc );
        else
            m_pDialog->DrawText( s_CandList.HoriCand.GetBuffer(), m_Elements.GetAt( 1 ), &rc );

        // Render the selected entry differently
        if( !bReading )
        {
            int nXLeft, nXRight;
            s_CandList.HoriCand.CPtoX( s_CandList.nFirstSelected, FALSE, &nXLeft );
            s_CandList.HoriCand.CPtoX( s_CandList.nFirstSelected + s_CandList.nHoriSelectedLen, FALSE, &nXRight );

            rc.right = rc.left + nXRight;
            rc.left += nXLeft;
            m_pDialog->DrawRect( &rc, SelBkColor );
            m_Elements.GetAt( 1 )->FontColor.Current = SelTextColor;
            m_pDialog->DrawText( s_CandList.HoriCand.GetBuffer() + s_CandList.nFirstSelected,
                                 m_Elements.GetAt( 1 ), &rc, false, s_CandList.nHoriSelectedLen );
        }
    }
}


//--------------------------------------------------------------------------------------
void CDXUTIMEEditBox::RenderComposition( float fElapsedTime )
{
	
    s_CompString.SetText( ImeUi_GetCompositionString() );

    RECT rcCaret =
    {
        0, 0, 0, 0
    };
    int nX, nXFirst;
    m_Buffer.CPtoX( m_nCaret, FALSE, &nX );
    m_Buffer.CPtoX( m_nFirstVisible, FALSE, &nXFirst );
    CDXUTElement* pElement = m_Elements.GetAt( 1 );

    // Get the required width
    RECT rc =
    {
        m_rcText.left + nX - nXFirst, m_rcText.top,
        m_rcText.left + nX - nXFirst, m_rcText.bottom
    };
    m_pDialog->CalcTextRect( s_CompString.GetBuffer(), pElement, &rc );

    // If the composition string is too long to fit within
    // the text area, move it to below the current line.
    // This matches the behavior of the default IME.
    if( rc.right > m_rcText.right )
        OffsetRect( &rc, m_rcText.left - rc.left, rc.bottom - rc.top );

    // Save the rectangle position for processing highlighted text.
    RECT rcFirst = rc;

    // Update s_ptCompString for RenderCandidateReadingWindow().
    s_ptCompString.x = rc.left; s_ptCompString.y = rc.top;


    D3DCOLOR TextColor = m_CompColor;
    // Render the window and string.
    // If the string is too long, we must wrap the line.
    pElement->FontColor.Current = TextColor;
    const WCHAR* pwszComp = s_CompString.GetBuffer();
    int nCharLeft = s_CompString.GetTextSize();
    for(; ; )
    {
        // Find the last character that can be drawn on the same line.
        int nLastInLine;
        int bTrail;
        s_CompString.XtoCP( m_rcText.right - rc.left, &nLastInLine, &bTrail );
        int nNumCharToDraw = __min( nCharLeft, nLastInLine );
        m_pDialog->CalcTextRect( pwszComp, pElement, &rc, nNumCharToDraw );

        // Draw the background
        // For Korean IME, blink the composition window background as if it
        // is a cursor.
        if( GetPrimaryLanguage() == LANG_KOREAN )
        {
            if( m_bCaretOn )
            {
                m_pDialog->DrawRect( &rc, m_CompWinColor );
            }
            else
            {
                // Not drawing composition string background. We
                // use the editbox's text color for composition
                // string text.
                TextColor = m_Elements.GetAt( 0 )->FontColor.States[DXUT_STATE_NORMAL];
            }
        }
        else
        {
            // Non-Korean IME. Always draw composition background.
            m_pDialog->DrawRect( &rc, m_CompWinColor );
        }

        // Draw the text
        pElement->FontColor.Current = TextColor;
        m_pDialog->DrawText( pwszComp, pElement, &rc, false, nNumCharToDraw );

        // Advance pointer and counter
        nCharLeft -= nNumCharToDraw;
        pwszComp += nNumCharToDraw;
        if( nCharLeft <= 0 )
            break;

        // Advance rectangle coordinates to beginning of next line
        OffsetRect( &rc, m_rcText.left - rc.left, rc.bottom - rc.top );
    }

    // Load the rect for the first line again.
    rc = rcFirst;

    // Inspect each character in the comp string.
    // For target-converted and target-non-converted characters,
    // we display a different background color so they appear highlighted.
    int nCharFirst = 0;
    nXFirst = 0;
    s_nFirstTargetConv = -1;
    BYTE* pAttr;
    const WCHAR* pcComp;
    for( pcComp = s_CompString.GetBuffer(), pAttr = ImeUi_GetCompStringAttr();
         *pcComp != L'\0'; ++pcComp, ++pAttr )
    {
        D3DCOLOR bkColor;

        // Render a different background for this character
        int nXLeft, nXRight;
        s_CompString.CPtoX( int( pcComp - s_CompString.GetBuffer() ), FALSE, &nXLeft );
        s_CompString.CPtoX( int( pcComp - s_CompString.GetBuffer() ), TRUE, &nXRight );

        // Check if this character is off the right edge and should
        // be wrapped to the next line.
        if( nXRight - nXFirst > m_rcText.right - rc.left )
        {
            // Advance rectangle coordinates to beginning of next line
            OffsetRect( &rc, m_rcText.left - rc.left, rc.bottom - rc.top );

            // Update the line's first character information
            nCharFirst = int( pcComp - s_CompString.GetBuffer() );
            s_CompString.CPtoX( nCharFirst, FALSE, &nXFirst );
        }

        // If the caret is on this character, save the coordinates
        // for drawing the caret later.
        if( ImeUi_GetImeCursorChars() == ( DWORD )( pcComp - s_CompString.GetBuffer() ) )
        {
            rcCaret = rc;
            rcCaret.left += nXLeft - nXFirst - 1;
            rcCaret.right = rcCaret.left + 2;
        }

        // Set up color based on the character attribute
        if( *pAttr == ATTR_TARGET_CONVERTED )
        {
            pElement->FontColor.Current = m_CompTargetColor;
            bkColor = m_CompTargetBkColor;
        }
        else if( *pAttr == ATTR_TARGET_NOTCONVERTED )
        {
            pElement->FontColor.Current = m_CompTargetNonColor;
            bkColor = m_CompTargetNonBkColor;
        }
        else
        {
            continue;
        }

        RECT rcTarget =
        {
            rc.left + nXLeft - nXFirst, rc.top, rc.left + nXRight - nXFirst, rc.bottom
        };
        m_pDialog->DrawRect( &rcTarget, bkColor );
        m_pDialog->DrawText( pcComp, pElement, &rcTarget, false, 1 );

        // Record the first target converted character's index
        if( -1 == s_nFirstTargetConv )
            s_nFirstTargetConv = int( pAttr - ImeUi_GetCompStringAttr() );
    }

    // Render the composition caret
    if( m_bCaretOn )
    {
        // If the caret is at the very end, its position would not have
        // been computed in the above loop.  We compute it here.
        if( ImeUi_GetImeCursorChars() == ( DWORD )s_CompString.GetTextSize() )
        {
            s_CompString.CPtoX( ImeUi_GetImeCursorChars(), FALSE, &nX );
            rcCaret = rc;
            rcCaret.left += nX - nXFirst - 1;
            rcCaret.right = rcCaret.left + 2;
        }

        m_pDialog->DrawRect( &rcCaret, m_CompCaretColor );
    }
}


//--------------------------------------------------------------------------------------
void CDXUTIMEEditBox::RenderIndicator( float fElapsedTime )
{
    CDXUTElement* pElement = m_Elements.GetAt( 9 );
    pElement->TextureColor.Blend( DXUT_STATE_NORMAL, fElapsedTime );

    m_pDialog->DrawSprite( pElement, &m_rcIndicator, DXUT_NEAR_BUTTON_DEPTH );
    RECT rc = m_rcIndicator;
    InflateRect( &rc, -m_nSpacing, -m_nSpacing );

    pElement->FontColor.Current = m_IndicatorImeColor;
    RECT rcCalc =
    {
        0, 0, 0, 0
    };
    // If IME system is off, draw English indicator.
    WCHAR* pwszIndicator = ImeUi_IsEnabled() ? ImeUi_GetIndicatior() : L"En";

    m_pDialog->CalcTextRect( pwszIndicator, pElement, &rcCalc );
    m_pDialog->DrawText( pwszIndicator, pElement, &rc );
}


//--------------------------------------------------------------------------------------
void CDXUTIMEEditBox::Render( float fElapsedTime )
{
    if( m_bVisible == false )
        return;

    // If we have not computed the indicator symbol width,
    // do it.
    if( !m_nIndicatorWidth )
    {
        RECT rc =
        {
            0, 0, 0, 0
        };
        m_pDialog->CalcTextRect( L"En", m_Elements.GetAt( 9 ), &rc );
        m_nIndicatorWidth = rc.right - rc.left;

        // Update the rectangles now that we have the indicator's width
        UpdateRects();
    }

    // Let the parent render first (edit control)
    CDXUTEditBox::Render( fElapsedTime );

    CDXUTElement* pElement = GetElement( 1 );
    if( pElement )
    {
        s_CompString.SetFontNode( m_pDialog->GetFont( pElement->iFont ) );
        s_CandList.HoriCand.SetFontNode( m_pDialog->GetFont( pElement->iFont ) );
    }

    //
    // Now render the IME elements
    //

    ImeUi_RenderUI();

    if( m_bHasFocus )	
    {
        // Render the input locale indicator
        RenderIndicator( fElapsedTime );

        // Display the composition string.
        // This method should also update s_ptCompString
        // for RenderCandidateReadingWindow.
        RenderComposition( fElapsedTime );

        // Display the reading/candidate window. RenderCandidateReadingWindow()
        // uses s_ptCompString to position itself.  s_ptCompString must have
        // been filled in by RenderComposition().
        if( ImeUi_IsShowReadingWindow() )
            // Reading window
            RenderCandidateReadingWindow( fElapsedTime, true );
        else if( ImeUi_IsShowCandListWindow() )
            // Candidate list window
            RenderCandidateReadingWindow( fElapsedTime, false );
    }
}


//--------------------------------------------------------------------------------------
void CDXUTIMEEditBox::SetImeEnableFlag( bool bFlag )
{
    s_bImeFlag = bFlag;
}


//--------------------------------------------------------------------------------------
void CDXUTIMEEditBox::Initialize( HWND hWnd )
{
    ImeUiCallback_DrawRect = NULL;
    ImeUiCallback_Malloc = malloc;
    ImeUiCallback_Free = free;
    ImeUiCallback_DrawFans = NULL;

    ImeUi_Initialize( hWnd );
	
    s_CompString.SetBufferSize( MAX_COMPSTRING_SIZE );
    ImeUi_EnableIme( true );
}


