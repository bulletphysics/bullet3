//--------------------------------------------------------------------------------------
// File: DXUTcamera.cpp
//
// Copyright (c) Microsoft Corporation. All rights reserved
//--------------------------------------------------------------------------------------
#include "DXUT.h"
#include "DXUTcamera.h"
#include "DXUTres.h"
#undef min // use __min instead
#undef max // use __max instead

//--------------------------------------------------------------------------------------
CD3DArcBall::CD3DArcBall()
{
    Reset();
    m_vDownPt = D3DXVECTOR3( 0, 0, 0 );
    m_vCurrentPt = D3DXVECTOR3( 0, 0, 0 );
    m_Offset.x = m_Offset.y = 0;

    RECT rc;
    GetClientRect( GetForegroundWindow(), &rc );
    SetWindow( rc.right, rc.bottom );
}





//--------------------------------------------------------------------------------------
void CD3DArcBall::Reset()
{
    D3DXQuaternionIdentity( &m_qDown );
    D3DXQuaternionIdentity( &m_qNow );
    D3DXMatrixIdentity( &m_mRotation );
    D3DXMatrixIdentity( &m_mTranslation );
    D3DXMatrixIdentity( &m_mTranslationDelta );
    m_bDrag = FALSE;
    m_fRadiusTranslation = 1.0f;
    m_fRadius = 1.0f;
}




//--------------------------------------------------------------------------------------
D3DXVECTOR3 CD3DArcBall::ScreenToVector( float fScreenPtX, float fScreenPtY )
{
    // Scale to screen
    FLOAT x = -( fScreenPtX - m_Offset.x - m_nWidth / 2 ) / ( m_fRadius * m_nWidth / 2 );
    FLOAT y = ( fScreenPtY - m_Offset.y - m_nHeight / 2 ) / ( m_fRadius * m_nHeight / 2 );

    FLOAT z = 0.0f;
    FLOAT mag = x * x + y * y;

    if( mag > 1.0f )
    {
        FLOAT scale = 1.0f / sqrtf( mag );
        x *= scale;
        y *= scale;
    }
    else
        z = sqrtf( 1.0f - mag );

    // Return vector
    return D3DXVECTOR3( x, y, z );
}




//--------------------------------------------------------------------------------------
D3DXQUATERNION CD3DArcBall::QuatFromBallPoints( const D3DXVECTOR3& vFrom, const D3DXVECTOR3& vTo )
{
    D3DXVECTOR3 vPart;
    float fDot = D3DXVec3Dot( &vFrom, &vTo );
    D3DXVec3Cross( &vPart, &vFrom, &vTo );

    return D3DXQUATERNION( vPart.x, vPart.y, vPart.z, fDot );
}




//--------------------------------------------------------------------------------------
void CD3DArcBall::OnBegin( int nX, int nY )
{
    // Only enter the drag state if the click falls
    // inside the click rectangle.
    if( nX >= m_Offset.x &&
        nX < m_Offset.x + m_nWidth &&
        nY >= m_Offset.y &&
        nY < m_Offset.y + m_nHeight )
    {
        m_bDrag = true;
        m_qDown = m_qNow;
        m_vDownPt = ScreenToVector( ( float )nX, ( float )nY );
    }
}




//--------------------------------------------------------------------------------------
void CD3DArcBall::OnMove( int nX, int nY )
{
    if( m_bDrag )
    {
        m_vCurrentPt = ScreenToVector( ( float )nX, ( float )nY );
        m_qNow = m_qDown * QuatFromBallPoints( m_vDownPt, m_vCurrentPt );
    }
}




//--------------------------------------------------------------------------------------
void CD3DArcBall::OnEnd()
{
    m_bDrag = false;
}




//--------------------------------------------------------------------------------------
// Desc:
//--------------------------------------------------------------------------------------
LRESULT CD3DArcBall::HandleMessages( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    // Current mouse position
    int iMouseX = ( short )LOWORD( lParam );
    int iMouseY = ( short )HIWORD( lParam );

    switch( uMsg )
    {
        case WM_LBUTTONDOWN:
        case WM_LBUTTONDBLCLK:
            SetCapture( hWnd );
            OnBegin( iMouseX, iMouseY );
            return TRUE;

        case WM_LBUTTONUP:
            ReleaseCapture();
            OnEnd();
            return TRUE;
        case WM_CAPTURECHANGED:
            if( ( HWND )lParam != hWnd )
            {
                ReleaseCapture();
                OnEnd();
            }
            return TRUE;

        case WM_RBUTTONDOWN:
        case WM_RBUTTONDBLCLK:
        case WM_MBUTTONDOWN:
        case WM_MBUTTONDBLCLK:
            SetCapture( hWnd );
            // Store off the position of the cursor when the button is pressed
            m_ptLastMouse.x = iMouseX;
            m_ptLastMouse.y = iMouseY;
            return TRUE;

        case WM_RBUTTONUP:
        case WM_MBUTTONUP:
            ReleaseCapture();
            return TRUE;

        case WM_MOUSEMOVE:
            if( MK_LBUTTON & wParam )
            {
                OnMove( iMouseX, iMouseY );
            }
            else if( ( MK_RBUTTON & wParam ) || ( MK_MBUTTON & wParam ) )
            {
                // Normalize based on size of window and bounding sphere radius
                FLOAT fDeltaX = ( m_ptLastMouse.x - iMouseX ) * m_fRadiusTranslation / m_nWidth;
                FLOAT fDeltaY = ( m_ptLastMouse.y - iMouseY ) * m_fRadiusTranslation / m_nHeight;

                if( wParam & MK_RBUTTON )
                {
                    D3DXMatrixTranslation( &m_mTranslationDelta, -2 * fDeltaX, 2 * fDeltaY, 0.0f );
                    D3DXMatrixMultiply( &m_mTranslation, &m_mTranslation, &m_mTranslationDelta );
                }
                else  // wParam & MK_MBUTTON
                {
                    D3DXMatrixTranslation( &m_mTranslationDelta, 0.0f, 0.0f, 5 * fDeltaY );
                    D3DXMatrixMultiply( &m_mTranslation, &m_mTranslation, &m_mTranslationDelta );
                }

                // Store mouse coordinate
                m_ptLastMouse.x = iMouseX;
                m_ptLastMouse.y = iMouseY;
            }
            return TRUE;
    }

    return FALSE;
}




//--------------------------------------------------------------------------------------
// Constructor
//--------------------------------------------------------------------------------------
CBaseCamera::CBaseCamera()
{
    m_cKeysDown = 0;
    ZeroMemory( m_aKeys, sizeof( BYTE ) * CAM_MAX_KEYS );
    ZeroMemory( m_GamePad, sizeof( DXUT_GAMEPAD ) * DXUT_MAX_CONTROLLERS );

    // Set attributes for the view matrix
    D3DXVECTOR3 vEyePt = D3DXVECTOR3( 0.0f, 0.0f, 0.0f );
    D3DXVECTOR3 vLookatPt = D3DXVECTOR3( 0.0f, 0.0f, 1.0f );

    // Setup the view matrix
    SetViewParams( &vEyePt, &vLookatPt );

    // Setup the projection matrix
    SetProjParams( D3DX_PI / 4, 1.0f, 1.0f, 1000.0f );

    GetCursorPos( &m_ptLastMousePosition );
    m_bMouseLButtonDown = false;
    m_bMouseMButtonDown = false;
    m_bMouseRButtonDown = false;
    m_nCurrentButtonMask = 0;
    m_nMouseWheelDelta = 0;

    m_fCameraYawAngle = 0.0f;
    m_fCameraPitchAngle = 0.0f;

    SetRect( &m_rcDrag, LONG_MIN, LONG_MIN, LONG_MAX, LONG_MAX );
    m_vVelocity = D3DXVECTOR3( 0, 0, 0 );
    m_bMovementDrag = false;
    m_vVelocityDrag = D3DXVECTOR3( 0, 0, 0 );
    m_fDragTimer = 0.0f;
    m_fTotalDragTimeToZero = 0.25;
    m_vRotVelocity = D3DXVECTOR2( 0, 0 );

    m_fRotationScaler = 0.01f;
    m_fMoveScaler = 5.0f;

    m_bInvertPitch = false;
    m_bEnableYAxisMovement = true;
    m_bEnablePositionMovement = true;

    m_vMouseDelta = D3DXVECTOR2( 0, 0 );
    m_fFramesToSmoothMouseData = 2.0f;

    m_bClipToBoundary = false;
    m_vMinBoundary = D3DXVECTOR3( -1, -1, -1 );
    m_vMaxBoundary = D3DXVECTOR3( 1, 1, 1 );

    m_bResetCursorAfterMove = false;
}


//--------------------------------------------------------------------------------------
// Client can call this to change the position and direction of camera
//--------------------------------------------------------------------------------------
VOID CBaseCamera::SetViewParams( D3DXVECTOR3* pvEyePt, D3DXVECTOR3* pvLookatPt )
{
    if( NULL == pvEyePt || NULL == pvLookatPt )
        return;

    m_vDefaultEye = m_vEye = *pvEyePt;
    m_vDefaultLookAt = m_vLookAt = *pvLookatPt;

    // Calc the view matrix
    D3DXVECTOR3 vUp( 0,1,0 );
    D3DXMatrixLookAtLH( &m_mView, pvEyePt, pvLookatPt, &vUp );

    D3DXMATRIX mInvView;
    D3DXMatrixInverse( &mInvView, NULL, &m_mView );

    // The axis basis vectors and camera position are stored inside the 
    // position matrix in the 4 rows of the camera's world matrix.
    // To figure out the yaw/pitch of the camera, we just need the Z basis vector
    D3DXVECTOR3* pZBasis = ( D3DXVECTOR3* )&mInvView._31;

    m_fCameraYawAngle = atan2f( pZBasis->x, pZBasis->z );
    float fLen = sqrtf( pZBasis->z * pZBasis->z + pZBasis->x * pZBasis->x );
    m_fCameraPitchAngle = -atan2f( pZBasis->y, fLen );
}




//--------------------------------------------------------------------------------------
// Calculates the projection matrix based on input params
//--------------------------------------------------------------------------------------
VOID CBaseCamera::SetProjParams( FLOAT fFOV, FLOAT fAspect, FLOAT fNearPlane,
                                 FLOAT fFarPlane )
{
    // Set attributes for the projection matrix
    m_fFOV = fFOV;
    m_fAspect = fAspect;
    m_fNearPlane = fNearPlane;
    m_fFarPlane = fFarPlane;

    D3DXMatrixPerspectiveFovLH( &m_mProj, fFOV, fAspect, fNearPlane, fFarPlane );
}




//--------------------------------------------------------------------------------------
// Call this from your message proc so this class can handle window messages
//--------------------------------------------------------------------------------------
LRESULT CBaseCamera::HandleMessages( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    UNREFERENCED_PARAMETER( hWnd );
    UNREFERENCED_PARAMETER( lParam );

    switch( uMsg )
    {
        case WM_KEYDOWN:
        {
            // Map this key to a D3DUtil_CameraKeys enum and update the
            // state of m_aKeys[] by adding the KEY_WAS_DOWN_MASK|KEY_IS_DOWN_MASK mask
            // only if the key is not down
            D3DUtil_CameraKeys mappedKey = MapKey( ( UINT )wParam );
            if( mappedKey != CAM_UNKNOWN )
            {
                if( FALSE == IsKeyDown( m_aKeys[mappedKey] ) )
                {
                    m_aKeys[ mappedKey ] = KEY_WAS_DOWN_MASK | KEY_IS_DOWN_MASK;
                    ++m_cKeysDown;
                }
            }
            break;
        }

        case WM_KEYUP:
        {
            // Map this key to a D3DUtil_CameraKeys enum and update the
            // state of m_aKeys[] by removing the KEY_IS_DOWN_MASK mask.
            D3DUtil_CameraKeys mappedKey = MapKey( ( UINT )wParam );
            if( mappedKey != CAM_UNKNOWN && ( DWORD )mappedKey < 8 )
            {
                m_aKeys[ mappedKey ] &= ~KEY_IS_DOWN_MASK;
                --m_cKeysDown;
            }
            break;
        }

        case WM_RBUTTONDOWN:
        case WM_MBUTTONDOWN:
        case WM_LBUTTONDOWN:
        case WM_RBUTTONDBLCLK:
        case WM_MBUTTONDBLCLK:
        case WM_LBUTTONDBLCLK:
            {
                // Compute the drag rectangle in screen coord.
                POINT ptCursor =
                {
                    ( short )LOWORD( lParam ), ( short )HIWORD( lParam )
                };

                // Update member var state
                if( ( uMsg == WM_LBUTTONDOWN || uMsg == WM_LBUTTONDBLCLK ) && PtInRect( &m_rcDrag, ptCursor ) )
                {
                    m_bMouseLButtonDown = true; m_nCurrentButtonMask |= MOUSE_LEFT_BUTTON;
                }
                if( ( uMsg == WM_MBUTTONDOWN || uMsg == WM_MBUTTONDBLCLK ) && PtInRect( &m_rcDrag, ptCursor ) )
                {
                    m_bMouseMButtonDown = true; m_nCurrentButtonMask |= MOUSE_MIDDLE_BUTTON;
                }
                if( ( uMsg == WM_RBUTTONDOWN || uMsg == WM_RBUTTONDBLCLK ) && PtInRect( &m_rcDrag, ptCursor ) )
                {
                    m_bMouseRButtonDown = true; m_nCurrentButtonMask |= MOUSE_RIGHT_BUTTON;
                }

                // Capture the mouse, so if the mouse button is 
                // released outside the window, we'll get the WM_LBUTTONUP message
                SetCapture( hWnd );
                GetCursorPos( &m_ptLastMousePosition );
                return TRUE;
            }

        case WM_RBUTTONUP:
        case WM_MBUTTONUP:
        case WM_LBUTTONUP:
            {
                // Update member var state
                if( uMsg == WM_LBUTTONUP )
                {
                    m_bMouseLButtonDown = false; m_nCurrentButtonMask &= ~MOUSE_LEFT_BUTTON;
                }
                if( uMsg == WM_MBUTTONUP )
                {
                    m_bMouseMButtonDown = false; m_nCurrentButtonMask &= ~MOUSE_MIDDLE_BUTTON;
                }
                if( uMsg == WM_RBUTTONUP )
                {
                    m_bMouseRButtonDown = false; m_nCurrentButtonMask &= ~MOUSE_RIGHT_BUTTON;
                }

                // Release the capture if no mouse buttons down
                if( !m_bMouseLButtonDown &&
                    !m_bMouseRButtonDown &&
                    !m_bMouseMButtonDown )
                {
                    ReleaseCapture();
                }
                break;
            }

        case WM_CAPTURECHANGED:
        {
            if( ( HWND )lParam != hWnd )
            {
                if( ( m_nCurrentButtonMask & MOUSE_LEFT_BUTTON ) ||
                    ( m_nCurrentButtonMask & MOUSE_MIDDLE_BUTTON ) ||
                    ( m_nCurrentButtonMask & MOUSE_RIGHT_BUTTON ) )
                {
                    m_bMouseLButtonDown = false;
                    m_bMouseMButtonDown = false;
                    m_bMouseRButtonDown = false;
                    m_nCurrentButtonMask &= ~MOUSE_LEFT_BUTTON;
                    m_nCurrentButtonMask &= ~MOUSE_MIDDLE_BUTTON;
                    m_nCurrentButtonMask &= ~MOUSE_RIGHT_BUTTON;
                    ReleaseCapture();
                }
            }
            break;
        }

        case WM_MOUSEWHEEL:
            // Update member var state
            m_nMouseWheelDelta += ( short )HIWORD( wParam );
            break;
    }

    return FALSE;
}

//--------------------------------------------------------------------------------------
// Figure out the velocity based on keyboard input & drag if any
//--------------------------------------------------------------------------------------
void CBaseCamera::GetInput( bool bGetKeyboardInput, bool bGetMouseInput, bool bGetGamepadInput,
                            bool bResetCursorAfterMove )
{
    m_vKeyboardDirection = D3DXVECTOR3( 0, 0, 0 );
    if( bGetKeyboardInput )
    {
        // Update acceleration vector based on keyboard state
        if( IsKeyDown( m_aKeys[CAM_MOVE_FORWARD] ) )
            m_vKeyboardDirection.z += 1.0f;
        if( IsKeyDown( m_aKeys[CAM_MOVE_BACKWARD] ) )
            m_vKeyboardDirection.z -= 1.0f;
        if( m_bEnableYAxisMovement )
        {
            if( IsKeyDown( m_aKeys[CAM_MOVE_UP] ) )
                m_vKeyboardDirection.y += 1.0f;
            if( IsKeyDown( m_aKeys[CAM_MOVE_DOWN] ) )
                m_vKeyboardDirection.y -= 1.0f;
        }
        if( IsKeyDown( m_aKeys[CAM_STRAFE_RIGHT] ) )
            m_vKeyboardDirection.x += 1.0f;
        if( IsKeyDown( m_aKeys[CAM_STRAFE_LEFT] ) )
            m_vKeyboardDirection.x -= 1.0f;
    }

    if( bGetMouseInput )
    {
        UpdateMouseDelta();
    }

    if( bGetGamepadInput )
    {
        m_vGamePadLeftThumb = D3DXVECTOR3( 0, 0, 0 );
        m_vGamePadRightThumb = D3DXVECTOR3( 0, 0, 0 );

        // Get controller state
        for( DWORD iUserIndex = 0; iUserIndex < DXUT_MAX_CONTROLLERS; iUserIndex++ )
        {
            DXUTGetGamepadState( iUserIndex, &m_GamePad[iUserIndex], true, true );

            // Mark time if the controller is in a non-zero state
            if( m_GamePad[iUserIndex].wButtons ||
                m_GamePad[iUserIndex].sThumbLX || m_GamePad[iUserIndex].sThumbLX ||
                m_GamePad[iUserIndex].sThumbRX || m_GamePad[iUserIndex].sThumbRY ||
                m_GamePad[iUserIndex].bLeftTrigger || m_GamePad[iUserIndex].bRightTrigger )
            {
                m_GamePadLastActive[iUserIndex] = DXUTGetTime();
            }
        }

        // Find out which controller was non-zero last
        int iMostRecentlyActive = -1;
        double fMostRecentlyActiveTime = 0.0f;
        for( DWORD iUserIndex = 0; iUserIndex < DXUT_MAX_CONTROLLERS; iUserIndex++ )
        {
            if( m_GamePadLastActive[iUserIndex] > fMostRecentlyActiveTime )
            {
                fMostRecentlyActiveTime = m_GamePadLastActive[iUserIndex];
                iMostRecentlyActive = iUserIndex;
            }
        }

        // Use the most recent non-zero controller if its connected
        if( iMostRecentlyActive >= 0 && m_GamePad[iMostRecentlyActive].bConnected )
        {
            m_vGamePadLeftThumb.x = m_GamePad[iMostRecentlyActive].fThumbLX;
            m_vGamePadLeftThumb.y = 0.0f;
            m_vGamePadLeftThumb.z = m_GamePad[iMostRecentlyActive].fThumbLY;

            m_vGamePadRightThumb.x = m_GamePad[iMostRecentlyActive].fThumbRX;
            m_vGamePadRightThumb.y = 0.0f;
            m_vGamePadRightThumb.z = m_GamePad[iMostRecentlyActive].fThumbRY;
        }
    }
}


//--------------------------------------------------------------------------------------
// Figure out the mouse delta based on mouse movement
//--------------------------------------------------------------------------------------
void CBaseCamera::UpdateMouseDelta()
{
    POINT ptCurMouseDelta;
    POINT ptCurMousePos;

    // Get current position of mouse
    GetCursorPos( &ptCurMousePos );

    // Calc how far it's moved since last frame
    ptCurMouseDelta.x = ptCurMousePos.x - m_ptLastMousePosition.x;
    ptCurMouseDelta.y = ptCurMousePos.y - m_ptLastMousePosition.y;

    // Record current position for next time
    m_ptLastMousePosition = ptCurMousePos;

    if( m_bResetCursorAfterMove && DXUTIsActive() )
    {
        // Set position of camera to center of desktop, 
        // so it always has room to move.  This is very useful
        // if the cursor is hidden.  If this isn't done and cursor is hidden, 
        // then invisible cursor will hit the edge of the screen 
        // and the user can't tell what happened
        POINT ptCenter;

        // Get the center of the current monitor
        MONITORINFO mi;
        mi.cbSize = sizeof( MONITORINFO );
        DXUTGetMonitorInfo( DXUTMonitorFromWindow( DXUTGetHWND(), MONITOR_DEFAULTTONEAREST ), &mi );
        ptCenter.x = ( mi.rcMonitor.left + mi.rcMonitor.right ) / 2;
        ptCenter.y = ( mi.rcMonitor.top + mi.rcMonitor.bottom ) / 2;
        SetCursorPos( ptCenter.x, ptCenter.y );
        m_ptLastMousePosition = ptCenter;
    }

    // Smooth the relative mouse data over a few frames so it isn't 
    // jerky when moving slowly at low frame rates.
    float fPercentOfNew = 1.0f / m_fFramesToSmoothMouseData;
    float fPercentOfOld = 1.0f - fPercentOfNew;
    m_vMouseDelta.x = m_vMouseDelta.x * fPercentOfOld + ptCurMouseDelta.x * fPercentOfNew;
    m_vMouseDelta.y = m_vMouseDelta.y * fPercentOfOld + ptCurMouseDelta.y * fPercentOfNew;

    m_vRotVelocity = m_vMouseDelta * m_fRotationScaler;
}




//--------------------------------------------------------------------------------------
// Figure out the velocity based on keyboard input & drag if any
//--------------------------------------------------------------------------------------
void CBaseCamera::UpdateVelocity( float fElapsedTime )
{
    D3DXMATRIX mRotDelta;
    D3DXVECTOR2 vGamePadRightThumb = D3DXVECTOR2( m_vGamePadRightThumb.x, -m_vGamePadRightThumb.z );
    m_vRotVelocity = m_vMouseDelta * m_fRotationScaler + vGamePadRightThumb * 0.02f;

    D3DXVECTOR3 vAccel = m_vKeyboardDirection + m_vGamePadLeftThumb;

    // Normalize vector so if moving 2 dirs (left & forward), 
    // the camera doesn't move faster than if moving in 1 dir
    D3DXVec3Normalize( &vAccel, &vAccel );

    // Scale the acceleration vector
    vAccel *= m_fMoveScaler;

    if( m_bMovementDrag )
    {
        // Is there any acceleration this frame?
        if( D3DXVec3LengthSq( &vAccel ) > 0 )
        {
            // If so, then this means the user has pressed a movement key\
            // so change the velocity immediately to acceleration 
            // upon keyboard input.  This isn't normal physics
            // but it will give a quick response to keyboard input
            m_vVelocity = vAccel;
            m_fDragTimer = m_fTotalDragTimeToZero;
            m_vVelocityDrag = vAccel / m_fDragTimer;
        }
        else
        {
            // If no key being pressed, then slowly decrease velocity to 0
            if( m_fDragTimer > 0 )
            {
                // Drag until timer is <= 0
                m_vVelocity -= m_vVelocityDrag * fElapsedTime;
                m_fDragTimer -= fElapsedTime;
            }
            else
            {
                // Zero velocity
                m_vVelocity = D3DXVECTOR3( 0, 0, 0 );
            }
        }
    }
    else
    {
        // No drag, so immediately change the velocity
        m_vVelocity = vAccel;
    }
}




//--------------------------------------------------------------------------------------
// Clamps pV to lie inside m_vMinBoundary & m_vMaxBoundary
//--------------------------------------------------------------------------------------
void CBaseCamera::ConstrainToBoundary( D3DXVECTOR3* pV )
{
    // Constrain vector to a bounding box 
    pV->x = __max( pV->x, m_vMinBoundary.x );
    pV->y = __max( pV->y, m_vMinBoundary.y );
    pV->z = __max( pV->z, m_vMinBoundary.z );

    pV->x = __min( pV->x, m_vMaxBoundary.x );
    pV->y = __min( pV->y, m_vMaxBoundary.y );
    pV->z = __min( pV->z, m_vMaxBoundary.z );
}




//--------------------------------------------------------------------------------------
// Maps a windows virtual key to an enum
//--------------------------------------------------------------------------------------
D3DUtil_CameraKeys CBaseCamera::MapKey( UINT nKey )
{
    // This could be upgraded to a method that's user-definable but for 
    // simplicity, we'll use a hardcoded mapping.
    switch( nKey )
    {
        case VK_CONTROL:
            return CAM_CONTROLDOWN;
        case VK_LEFT:
            return CAM_STRAFE_LEFT;
        case VK_RIGHT:
            return CAM_STRAFE_RIGHT;
        case VK_UP:
            return CAM_MOVE_FORWARD;
        case VK_DOWN:
            return CAM_MOVE_BACKWARD;
        case VK_PRIOR:
            return CAM_MOVE_UP;        // pgup
        case VK_NEXT:
            return CAM_MOVE_DOWN;      // pgdn

        case 'A':
            return CAM_STRAFE_LEFT;
        case 'D':
            return CAM_STRAFE_RIGHT;
        case 'W':
            return CAM_MOVE_FORWARD;
        case 'S':
            return CAM_MOVE_BACKWARD;
        case 'Q':
            return CAM_MOVE_DOWN;
        case 'E':
            return CAM_MOVE_UP;

        case VK_NUMPAD4:
            return CAM_STRAFE_LEFT;
        case VK_NUMPAD6:
            return CAM_STRAFE_RIGHT;
        case VK_NUMPAD8:
            return CAM_MOVE_FORWARD;
        case VK_NUMPAD2:
            return CAM_MOVE_BACKWARD;
        case VK_NUMPAD9:
            return CAM_MOVE_UP;
        case VK_NUMPAD3:
            return CAM_MOVE_DOWN;

        case VK_HOME:
            return CAM_RESET;
    }

    return CAM_UNKNOWN;
}




//--------------------------------------------------------------------------------------
// Reset the camera's position back to the default
//--------------------------------------------------------------------------------------
VOID CBaseCamera::Reset()
{
    SetViewParams( &m_vDefaultEye, &m_vDefaultLookAt );
}




//--------------------------------------------------------------------------------------
// Constructor
//--------------------------------------------------------------------------------------
CFirstPersonCamera::CFirstPersonCamera() : m_nActiveButtonMask( 0x07 )
{
    m_bRotateWithoutButtonDown = false;
}




//--------------------------------------------------------------------------------------
// Update the view matrix based on user input & elapsed time
//--------------------------------------------------------------------------------------
VOID CFirstPersonCamera::FrameMove( FLOAT fElapsedTime )
{
    if( DXUTGetGlobalTimer()->IsStopped() ) {
        if (DXUTGetFPS() == 0.0f) fElapsedTime = 0;
        else fElapsedTime = 1.0f / DXUTGetFPS();
    }

    if( IsKeyDown( m_aKeys[CAM_RESET] ) )
        Reset();

    // Get keyboard/mouse/gamepad input
    GetInput( m_bEnablePositionMovement, ( m_nActiveButtonMask & m_nCurrentButtonMask ) || m_bRotateWithoutButtonDown,
              true, m_bResetCursorAfterMove );

    //// Get the mouse movement (if any) if the mouse button are down
    //if( (m_nActiveButtonMask & m_nCurrentButtonMask) || m_bRotateWithoutButtonDown )
    //    UpdateMouseDelta( fElapsedTime );

    // Get amount of velocity based on the keyboard input and drag (if any)
    UpdateVelocity( fElapsedTime );

    // Simple euler method to calculate position delta
    D3DXVECTOR3 vPosDelta = m_vVelocity * fElapsedTime;

    // If rotating the camera 
    if( ( m_nActiveButtonMask & m_nCurrentButtonMask ) ||
        m_bRotateWithoutButtonDown ||
        m_vGamePadRightThumb.x != 0 ||
        m_vGamePadRightThumb.z != 0 )
    {
        // Update the pitch & yaw angle based on mouse movement
        float fYawDelta = m_vRotVelocity.x;
        float fPitchDelta = m_vRotVelocity.y;

        // Invert pitch if requested
        if( m_bInvertPitch )
            fPitchDelta = -fPitchDelta;

        m_fCameraPitchAngle += fPitchDelta;
        m_fCameraYawAngle += fYawDelta;

        // Limit pitch to straight up or straight down
        m_fCameraPitchAngle = __max( -D3DX_PI / 2.0f, m_fCameraPitchAngle );
        m_fCameraPitchAngle = __min( +D3DX_PI / 2.0f, m_fCameraPitchAngle );
    }

    // Make a rotation matrix based on the camera's yaw & pitch
    D3DXMATRIX mCameraRot;
    D3DXMatrixRotationYawPitchRoll( &mCameraRot, m_fCameraYawAngle, m_fCameraPitchAngle, 0 );

    // Transform vectors based on camera's rotation matrix
    D3DXVECTOR3 vWorldUp, vWorldAhead;
    D3DXVECTOR3 vLocalUp = D3DXVECTOR3( 0, 1, 0 );
    D3DXVECTOR3 vLocalAhead = D3DXVECTOR3( 0, 0, 1 );
    D3DXVec3TransformCoord( &vWorldUp, &vLocalUp, &mCameraRot );
    D3DXVec3TransformCoord( &vWorldAhead, &vLocalAhead, &mCameraRot );

    // Transform the position delta by the camera's rotation 
    D3DXVECTOR3 vPosDeltaWorld;
    if( !m_bEnableYAxisMovement )
    {
        // If restricting Y movement, do not include pitch
        // when transforming position delta vector.
        D3DXMatrixRotationYawPitchRoll( &mCameraRot, m_fCameraYawAngle, 0.0f, 0.0f );
    }
    D3DXVec3TransformCoord( &vPosDeltaWorld, &vPosDelta, &mCameraRot );

    // Move the eye position 
    m_vEye += vPosDeltaWorld;
    if( m_bClipToBoundary )
        ConstrainToBoundary( &m_vEye );

    // Update the lookAt position based on the eye position 
    m_vLookAt = m_vEye + vWorldAhead;

    // Update the view matrix
    D3DXMatrixLookAtLH( &m_mView, &m_vEye, &m_vLookAt, &vWorldUp );

    D3DXMatrixInverse( &m_mCameraWorld, NULL, &m_mView );
}


//--------------------------------------------------------------------------------------
// Enable or disable each of the mouse buttons for rotation drag.
//--------------------------------------------------------------------------------------
void CFirstPersonCamera::SetRotateButtons( bool bLeft, bool bMiddle, bool bRight, bool bRotateWithoutButtonDown )
{
    m_nActiveButtonMask = ( bLeft ? MOUSE_LEFT_BUTTON : 0 ) |
        ( bMiddle ? MOUSE_MIDDLE_BUTTON : 0 ) |
        ( bRight ? MOUSE_RIGHT_BUTTON : 0 );
    m_bRotateWithoutButtonDown = bRotateWithoutButtonDown;
}


//--------------------------------------------------------------------------------------
// Constructor 
//--------------------------------------------------------------------------------------
CModelViewerCamera::CModelViewerCamera()
{
    D3DXMatrixIdentity( &m_mWorld );
    D3DXMatrixIdentity( &m_mModelRot );
    D3DXMatrixIdentity( &m_mModelLastRot );
    D3DXMatrixIdentity( &m_mCameraRotLast );
    m_vModelCenter = D3DXVECTOR3( 0, 0, 0 );
    m_fRadius = 5.0f;
    m_fDefaultRadius = 5.0f;
    m_fMinRadius = 1.0f;
    m_fMaxRadius = FLT_MAX;
    m_bLimitPitch = false;
    m_bEnablePositionMovement = false;
    m_bAttachCameraToModel = false;

    m_nRotateModelButtonMask = MOUSE_LEFT_BUTTON;
    m_nZoomButtonMask = MOUSE_WHEEL;
    m_nRotateCameraButtonMask = MOUSE_RIGHT_BUTTON;
    m_bDragSinceLastUpdate = true;
}




//--------------------------------------------------------------------------------------
// Update the view matrix & the model's world matrix based 
//       on user input & elapsed time
//--------------------------------------------------------------------------------------
VOID CModelViewerCamera::FrameMove( FLOAT fElapsedTime )
{
    if( IsKeyDown( m_aKeys[CAM_RESET] ) )
        Reset();

    // If no dragged has happend since last time FrameMove is called,
    // and no camera key is held down, then no need to handle again.
    if( !m_bDragSinceLastUpdate && 0 == m_cKeysDown )
        return;
    m_bDragSinceLastUpdate = false;

    //// If no mouse button is held down, 
    //// Get the mouse movement (if any) if the mouse button are down
    //if( m_nCurrentButtonMask != 0 ) 
    //    UpdateMouseDelta( fElapsedTime );

    GetInput( m_bEnablePositionMovement, m_nCurrentButtonMask != 0, true, false );

    // Get amount of velocity based on the keyboard input and drag (if any)
    UpdateVelocity( fElapsedTime );

    // Simple euler method to calculate position delta
    D3DXVECTOR3 vPosDelta = m_vVelocity * fElapsedTime;

    // Change the radius from the camera to the model based on wheel scrolling
    if( m_nMouseWheelDelta && m_nZoomButtonMask == MOUSE_WHEEL )
        m_fRadius -= m_nMouseWheelDelta * m_fRadius * 0.1f / 120.0f;
    m_fRadius = __min( m_fMaxRadius, m_fRadius );
    m_fRadius = __max( m_fMinRadius, m_fRadius );
    m_nMouseWheelDelta = 0;

    // Get the inverse of the arcball's rotation matrix
    D3DXMATRIX mCameraRot;
    D3DXMatrixInverse( &mCameraRot, NULL, m_ViewArcBall.GetRotationMatrix() );

    // Transform vectors based on camera's rotation matrix
    D3DXVECTOR3 vWorldUp, vWorldAhead;
    D3DXVECTOR3 vLocalUp = D3DXVECTOR3( 0, 1, 0 );
    D3DXVECTOR3 vLocalAhead = D3DXVECTOR3( 0, 0, 1 );
    D3DXVec3TransformCoord( &vWorldUp, &vLocalUp, &mCameraRot );
    D3DXVec3TransformCoord( &vWorldAhead, &vLocalAhead, &mCameraRot );

    // Transform the position delta by the camera's rotation 
    D3DXVECTOR3 vPosDeltaWorld;
    D3DXVec3TransformCoord( &vPosDeltaWorld, &vPosDelta, &mCameraRot );

    // Move the lookAt position 
    m_vLookAt += vPosDeltaWorld;
    if( m_bClipToBoundary )
        ConstrainToBoundary( &m_vLookAt );

    // Update the eye point based on a radius away from the lookAt position
    m_vEye = m_vLookAt - vWorldAhead * m_fRadius;

    // Update the view matrix
    D3DXMatrixLookAtLH( &m_mView, &m_vEye, &m_vLookAt, &vWorldUp );

    D3DXMATRIX mInvView;
    D3DXMatrixInverse( &mInvView, NULL, &m_mView );
    mInvView._41 = mInvView._42 = mInvView._43 = 0;

    D3DXMATRIX mModelLastRotInv;
    D3DXMatrixInverse( &mModelLastRotInv, NULL, &m_mModelLastRot );

    // Accumulate the delta of the arcball's rotation in view space.
    // Note that per-frame delta rotations could be problematic over long periods of time.
    D3DXMATRIX mModelRot;
    mModelRot = *m_WorldArcBall.GetRotationMatrix();
    m_mModelRot *= m_mView * mModelLastRotInv * mModelRot * mInvView;

    if( m_ViewArcBall.IsBeingDragged() && m_bAttachCameraToModel && !IsKeyDown( m_aKeys[CAM_CONTROLDOWN] ) )
    {
        // Attach camera to model by inverse of the model rotation
        D3DXMATRIX mCameraLastRotInv;
        D3DXMatrixInverse( &mCameraLastRotInv, NULL, &m_mCameraRotLast );
        D3DXMATRIX mCameraRotDelta = mCameraLastRotInv * mCameraRot; // local to world matrix
        m_mModelRot *= mCameraRotDelta;
    }
    m_mCameraRotLast = mCameraRot;

    m_mModelLastRot = mModelRot;

    // Since we're accumulating delta rotations, we need to orthonormalize 
    // the matrix to prevent eventual matrix skew
    D3DXVECTOR3* pXBasis = ( D3DXVECTOR3* )&m_mModelRot._11;
    D3DXVECTOR3* pYBasis = ( D3DXVECTOR3* )&m_mModelRot._21;
    D3DXVECTOR3* pZBasis = ( D3DXVECTOR3* )&m_mModelRot._31;
    D3DXVec3Normalize( pXBasis, pXBasis );
    D3DXVec3Cross( pYBasis, pZBasis, pXBasis );
    D3DXVec3Normalize( pYBasis, pYBasis );
    D3DXVec3Cross( pZBasis, pXBasis, pYBasis );

    // Translate the rotation matrix to the same position as the lookAt position
    m_mModelRot._41 = m_vLookAt.x;
    m_mModelRot._42 = m_vLookAt.y;
    m_mModelRot._43 = m_vLookAt.z;

    // Translate world matrix so its at the center of the model
    D3DXMATRIX mTrans;
    D3DXMatrixTranslation( &mTrans, -m_vModelCenter.x, -m_vModelCenter.y, -m_vModelCenter.z );
    m_mWorld = mTrans * m_mModelRot;
}


void CModelViewerCamera::SetDragRect( RECT& rc )
{
    CBaseCamera::SetDragRect( rc );

    m_WorldArcBall.SetOffset( rc.left, rc.top );
    m_ViewArcBall.SetOffset( rc.left, rc.top );
    SetWindow( rc.right - rc.left, rc.bottom - rc.top );
}


//--------------------------------------------------------------------------------------
// Reset the camera's position back to the default
//--------------------------------------------------------------------------------------
VOID CModelViewerCamera::Reset()
{
    CBaseCamera::Reset();

    D3DXMatrixIdentity( &m_mWorld );
    D3DXMatrixIdentity( &m_mModelRot );
    D3DXMatrixIdentity( &m_mModelLastRot );
    D3DXMatrixIdentity( &m_mCameraRotLast );

    m_fRadius = m_fDefaultRadius;
    m_WorldArcBall.Reset();
    m_ViewArcBall.Reset();
}


//--------------------------------------------------------------------------------------
// Override for setting the view parameters
//--------------------------------------------------------------------------------------
void CModelViewerCamera::SetViewParams( D3DXVECTOR3* pvEyePt, D3DXVECTOR3* pvLookatPt )
{
    CBaseCamera::SetViewParams( pvEyePt, pvLookatPt );

    // Propogate changes to the member arcball
    D3DXQUATERNION quat;
    D3DXMATRIXA16 mRotation;
    D3DXVECTOR3 vUp( 0,1,0 );
    D3DXMatrixLookAtLH( &mRotation, pvEyePt, pvLookatPt, &vUp );
    D3DXQuaternionRotationMatrix( &quat, &mRotation );
    m_ViewArcBall.SetQuatNow( quat );

    // Set the radius according to the distance
    D3DXVECTOR3 vEyeToPoint;
    D3DXVec3Subtract( &vEyeToPoint, pvLookatPt, pvEyePt );
    SetRadius( D3DXVec3Length( &vEyeToPoint ) );

    // View information changed. FrameMove should be called.
    m_bDragSinceLastUpdate = true;
}



//--------------------------------------------------------------------------------------
// Call this from your message proc so this class can handle window messages
//--------------------------------------------------------------------------------------
LRESULT CModelViewerCamera::HandleMessages( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    CBaseCamera::HandleMessages( hWnd, uMsg, wParam, lParam );

    if( ( ( uMsg == WM_LBUTTONDOWN || uMsg == WM_LBUTTONDBLCLK ) && m_nRotateModelButtonMask & MOUSE_LEFT_BUTTON ) ||
        ( ( uMsg == WM_MBUTTONDOWN || uMsg == WM_MBUTTONDBLCLK ) && m_nRotateModelButtonMask & MOUSE_MIDDLE_BUTTON ) ||
        ( ( uMsg == WM_RBUTTONDOWN || uMsg == WM_RBUTTONDBLCLK ) && m_nRotateModelButtonMask & MOUSE_RIGHT_BUTTON ) )
    {
        int iMouseX = ( short )LOWORD( lParam );
        int iMouseY = ( short )HIWORD( lParam );
        m_WorldArcBall.OnBegin( iMouseX, iMouseY );
    }

    if( ( ( uMsg == WM_LBUTTONDOWN || uMsg == WM_LBUTTONDBLCLK ) && m_nRotateCameraButtonMask & MOUSE_LEFT_BUTTON ) ||
        ( ( uMsg == WM_MBUTTONDOWN || uMsg == WM_MBUTTONDBLCLK ) &&
          m_nRotateCameraButtonMask & MOUSE_MIDDLE_BUTTON ) ||
        ( ( uMsg == WM_RBUTTONDOWN || uMsg == WM_RBUTTONDBLCLK ) && m_nRotateCameraButtonMask & MOUSE_RIGHT_BUTTON ) )
    {
        int iMouseX = ( short )LOWORD( lParam );
        int iMouseY = ( short )HIWORD( lParam );
        m_ViewArcBall.OnBegin( iMouseX, iMouseY );
    }

    if( uMsg == WM_MOUSEMOVE )
    {
        int iMouseX = ( short )LOWORD( lParam );
        int iMouseY = ( short )HIWORD( lParam );
        m_WorldArcBall.OnMove( iMouseX, iMouseY );
        m_ViewArcBall.OnMove( iMouseX, iMouseY );
    }

    if( ( uMsg == WM_LBUTTONUP && m_nRotateModelButtonMask & MOUSE_LEFT_BUTTON ) ||
        ( uMsg == WM_MBUTTONUP && m_nRotateModelButtonMask & MOUSE_MIDDLE_BUTTON ) ||
        ( uMsg == WM_RBUTTONUP && m_nRotateModelButtonMask & MOUSE_RIGHT_BUTTON ) )
    {
        m_WorldArcBall.OnEnd();
    }

    if( ( uMsg == WM_LBUTTONUP && m_nRotateCameraButtonMask & MOUSE_LEFT_BUTTON ) ||
        ( uMsg == WM_MBUTTONUP && m_nRotateCameraButtonMask & MOUSE_MIDDLE_BUTTON ) ||
        ( uMsg == WM_RBUTTONUP && m_nRotateCameraButtonMask & MOUSE_RIGHT_BUTTON ) )
    {
        m_ViewArcBall.OnEnd();
    }

    if( uMsg == WM_CAPTURECHANGED )
    {
        if( ( HWND )lParam != hWnd )
        {
            if( ( m_nRotateModelButtonMask & MOUSE_LEFT_BUTTON ) ||
                ( m_nRotateModelButtonMask & MOUSE_MIDDLE_BUTTON ) ||
                ( m_nRotateModelButtonMask & MOUSE_RIGHT_BUTTON ) )
            {
                m_WorldArcBall.OnEnd();
            }

            if( ( m_nRotateCameraButtonMask & MOUSE_LEFT_BUTTON ) ||
                ( m_nRotateCameraButtonMask & MOUSE_MIDDLE_BUTTON ) ||
                ( m_nRotateCameraButtonMask & MOUSE_RIGHT_BUTTON ) )
            {
                m_ViewArcBall.OnEnd();
            }
        }
    }

    if( uMsg == WM_LBUTTONDOWN ||
        uMsg == WM_LBUTTONDBLCLK ||
        uMsg == WM_MBUTTONDOWN ||
        uMsg == WM_MBUTTONDBLCLK ||
        uMsg == WM_RBUTTONDOWN ||
        uMsg == WM_RBUTTONDBLCLK ||
        uMsg == WM_LBUTTONUP ||
        uMsg == WM_MBUTTONUP ||
        uMsg == WM_RBUTTONUP ||
        uMsg == WM_MOUSEWHEEL ||
        uMsg == WM_MOUSEMOVE )
    {
        m_bDragSinceLastUpdate = true;
    }

    return FALSE;
}



//--------------------------------------------------------------------------------------
// D3D9
IDirect3DDevice9*           CDXUTDirectionWidget::s_pd3d9Device = NULL;
ID3DXEffect*                CDXUTDirectionWidget::s_pD3D9Effect = NULL;
ID3DXMesh*                  CDXUTDirectionWidget::s_pD3D9Mesh = NULL;
D3DXHANDLE                  CDXUTDirectionWidget::s_hRenderWith1LightNoTexture = NULL;
D3DXHANDLE                  CDXUTDirectionWidget::s_hMaterialDiffuseColor = NULL;
D3DXHANDLE                  CDXUTDirectionWidget::s_hLightDir = NULL;
D3DXHANDLE                  CDXUTDirectionWidget::s_hWorldViewProjection = NULL;
D3DXHANDLE                  CDXUTDirectionWidget::s_hWorld = NULL;


//--------------------------------------------------------------------------------------
CDXUTDirectionWidget::CDXUTDirectionWidget()
{
    m_fRadius = 1.0f;
    m_vDefaultDir = D3DXVECTOR3( 0, 1, 0 );
    m_vCurrentDir = m_vDefaultDir;
    m_nRotateMask = MOUSE_RIGHT_BUTTON;

    D3DXMatrixIdentity( &m_mView );
    D3DXMatrixIdentity( &m_mRot );
    D3DXMatrixIdentity( &m_mRotSnapshot );
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDirectionWidget::StaticOnD3D9CreateDevice( IDirect3DDevice9* pd3dDevice )
{
    HRESULT hr;

    s_pd3d9Device = pd3dDevice;

    const char* g_strBuffer =
        "float4 g_MaterialDiffuseColor;      // Material's diffuse color\r\n"
        "float3 g_LightDir;                  // Light's direction in world space\r\n"
        "float4x4 g_mWorld;                  // World matrix for object\r\n"
        "float4x4 g_mWorldViewProjection;    // World * View * Projection matrix\r\n"
        "\r\n"
        "struct VS_OUTPUT\r\n"
        "{\r\n"
        "    float4 Position   : POSITION;   // vertex position\r\n"
        "    float4 Diffuse    : COLOR0;     // vertex diffuse color\r\n"
        "};\r\n"
        "\r\n"
        "VS_OUTPUT RenderWith1LightNoTextureVS( float4 vPos : POSITION,\r\n"
        "                                       float3 vNormal : NORMAL )\r\n"
        "{\r\n"
        "    VS_OUTPUT Output;\r\n"
        "\r\n"
        "    // Transform the position from object space to homogeneous projection space\r\n"
        "    Output.Position = mul(vPos, g_mWorldViewProjection);\r\n"
        "\r\n"
        "    // Transform the normal from object space to world space\r\n"
        "    float3 vNormalWorldSpace;\r\n"
        "    vNormalWorldSpace = normalize(mul(vNormal, (float3x3)g_mWorld)); // normal (world space)\r\n"
        "\r\n"
        "    // Compute simple directional lighting equation\r\n"
        "    Output.Diffuse.rgb = g_MaterialDiffuseColor * max(0,dot(vNormalWorldSpace, g_LightDir));\r\n"
        "    Output.Diffuse.a = 1.0f;\r\n"
        "\r\n"
        "    return Output;\r\n"
        "}\r\n"
        "\r\n"
        "float4 RenderWith1LightNoTexturePS( float4 Diffuse : COLOR0 ) : COLOR0\r\n"
        "{\r\n"
        "    return Diffuse;\r\n"
        "}\r\n"
        "\r\n"
        "technique RenderWith1LightNoTexture\r\n"
        "{\r\n"
        "    pass P0\r\n"
        "    {\r\n"
        "        VertexShader = compile vs_2_0 RenderWith1LightNoTextureVS();\r\n"
        "        PixelShader  = compile ps_2_0 RenderWith1LightNoTexturePS();\r\n"
        "    }\r\n"
        "}\r\n"
        "";

    UINT dwBufferSize = ( UINT )strlen( g_strBuffer ) + 1;

    V_RETURN( D3DXCreateEffect( s_pd3d9Device, g_strBuffer, dwBufferSize, NULL, NULL, D3DXFX_NOT_CLONEABLE,
                                NULL, &s_pD3D9Effect, NULL ) );

    // Save technique handles for use when rendering
    s_hRenderWith1LightNoTexture = s_pD3D9Effect->GetTechniqueByName( "RenderWith1LightNoTexture" );
    s_hMaterialDiffuseColor = s_pD3D9Effect->GetParameterByName( NULL, "g_MaterialDiffuseColor" );
    s_hLightDir = s_pD3D9Effect->GetParameterByName( NULL, "g_LightDir" );
    s_hWorld = s_pD3D9Effect->GetParameterByName( NULL, "g_mWorld" );
    s_hWorldViewProjection = s_pD3D9Effect->GetParameterByName( NULL, "g_mWorldViewProjection" );

    // Load the mesh with D3DX and get back a ID3DXMesh*.  For this
    // sample we'll ignore the X file's embedded materials since we know 
    // exactly the model we're loading.  See the mesh samples such as
    // "OptimizedMesh" for a more generic mesh loading example.
    V_RETURN( DXUTCreateArrowMeshFromInternalArray( s_pd3d9Device, &s_pD3D9Mesh ) );

    // Optimize the mesh for this graphics card's vertex cache 
    // so when rendering the mesh's triangle list the vertices will 
    // cache hit more often so it won't have to re-execute the vertex shader 
    // on those vertices so it will improve perf.
    DWORD* rgdwAdjacency = new DWORD[s_pD3D9Mesh->GetNumFaces() * 3];
    if( rgdwAdjacency == NULL )
        return E_OUTOFMEMORY;
    V( s_pD3D9Mesh->GenerateAdjacency( 1e-6f, rgdwAdjacency ) );
    V( s_pD3D9Mesh->OptimizeInplace( D3DXMESHOPT_VERTEXCACHE, rgdwAdjacency, NULL, NULL, NULL ) );
    delete []rgdwAdjacency;

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDirectionWidget::OnD3D9ResetDevice( const D3DSURFACE_DESC* pBackBufferSurfaceDesc )
{
    m_ArcBall.SetWindow( pBackBufferSurfaceDesc->Width, pBackBufferSurfaceDesc->Height );
    return S_OK;
}


//--------------------------------------------------------------------------------------
void CDXUTDirectionWidget::StaticOnD3D9LostDevice()
{
    if( s_pD3D9Effect )
        s_pD3D9Effect->OnLostDevice();
}


//--------------------------------------------------------------------------------------
void CDXUTDirectionWidget::StaticOnD3D9DestroyDevice()
{
    SAFE_RELEASE( s_pD3D9Effect );
    SAFE_RELEASE( s_pD3D9Mesh );
}


//--------------------------------------------------------------------------------------
LRESULT CDXUTDirectionWidget::HandleMessages( HWND hWnd, UINT uMsg,
                                              WPARAM wParam, LPARAM lParam )
{
    switch( uMsg )
    {
        case WM_LBUTTONDOWN:
        case WM_MBUTTONDOWN:
        case WM_RBUTTONDOWN:
            {
                if( ( ( m_nRotateMask & MOUSE_LEFT_BUTTON ) != 0 && uMsg == WM_LBUTTONDOWN ) ||
                    ( ( m_nRotateMask & MOUSE_MIDDLE_BUTTON ) != 0 && uMsg == WM_MBUTTONDOWN ) ||
                    ( ( m_nRotateMask & MOUSE_RIGHT_BUTTON ) != 0 && uMsg == WM_RBUTTONDOWN ) )
                {
                    int iMouseX = ( int )( short )LOWORD( lParam );
                    int iMouseY = ( int )( short )HIWORD( lParam );
                    m_ArcBall.OnBegin( iMouseX, iMouseY );
                    SetCapture( hWnd );
                }
                return TRUE;
            }

        case WM_MOUSEMOVE:
        {
            if( m_ArcBall.IsBeingDragged() )
            {
                int iMouseX = ( int )( short )LOWORD( lParam );
                int iMouseY = ( int )( short )HIWORD( lParam );
                m_ArcBall.OnMove( iMouseX, iMouseY );
                UpdateLightDir();
            }
            return TRUE;
        }

        case WM_LBUTTONUP:
        case WM_MBUTTONUP:
        case WM_RBUTTONUP:
            {
                if( ( ( m_nRotateMask & MOUSE_LEFT_BUTTON ) != 0 && uMsg == WM_LBUTTONUP ) ||
                    ( ( m_nRotateMask & MOUSE_MIDDLE_BUTTON ) != 0 && uMsg == WM_MBUTTONUP ) ||
                    ( ( m_nRotateMask & MOUSE_RIGHT_BUTTON ) != 0 && uMsg == WM_RBUTTONUP ) )
                {
                    m_ArcBall.OnEnd();
                    ReleaseCapture();
                }

                UpdateLightDir();
                return TRUE;
            }

        case WM_CAPTURECHANGED:
        {
            if( ( HWND )lParam != hWnd )
            {
                if( ( m_nRotateMask & MOUSE_LEFT_BUTTON ) ||
                    ( m_nRotateMask & MOUSE_MIDDLE_BUTTON ) ||
                    ( m_nRotateMask & MOUSE_RIGHT_BUTTON ) )
                {
                    m_ArcBall.OnEnd();
                    ReleaseCapture();
                }
            }
            return TRUE;
        }
    }

    return 0;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDirectionWidget::OnRender9( D3DXCOLOR color, const D3DXMATRIX* pmView,
                                         const D3DXMATRIX* pmProj, const D3DXVECTOR3* pEyePt )
{
    m_mView = *pmView;

    // Render the light spheres so the user can visually see the light dir
    UINT iPass, cPasses;
    D3DXMATRIX mRotate;
    D3DXMATRIX mScale;
    D3DXMATRIX mTrans;
    D3DXMATRIXA16 mWorldViewProj;
    HRESULT hr;

    V( s_pD3D9Effect->SetTechnique( s_hRenderWith1LightNoTexture ) );
    V( s_pD3D9Effect->SetVector( s_hMaterialDiffuseColor, ( D3DXVECTOR4* )&color ) );

    D3DXVECTOR3 vEyePt;
    D3DXVec3Normalize( &vEyePt, pEyePt );
    V( s_pD3D9Effect->SetValue( s_hLightDir, &vEyePt, sizeof( D3DXVECTOR3 ) ) );

    // Rotate arrow model to point towards origin
    D3DXMATRIX mRotateA, mRotateB;
    D3DXVECTOR3 vAt = D3DXVECTOR3( 0, 0, 0 );
    D3DXVECTOR3 vUp = D3DXVECTOR3( 0, 1, 0 );
    D3DXMatrixRotationX( &mRotateB, D3DX_PI );
    D3DXMatrixLookAtLH( &mRotateA, &m_vCurrentDir, &vAt, &vUp );
    D3DXMatrixInverse( &mRotateA, NULL, &mRotateA );
    mRotate = mRotateB * mRotateA;

    D3DXVECTOR3 vL = m_vCurrentDir * m_fRadius * 1.0f;
    D3DXMatrixTranslation( &mTrans, vL.x, vL.y, vL.z );
    D3DXMatrixScaling( &mScale, m_fRadius * 0.2f, m_fRadius * 0.2f, m_fRadius * 0.2f );

    D3DXMATRIX mWorld = mRotate * mScale * mTrans;
    mWorldViewProj = mWorld * ( m_mView )*( *pmProj );

    V( s_pD3D9Effect->SetMatrix( s_hWorldViewProjection, &mWorldViewProj ) );
    V( s_pD3D9Effect->SetMatrix( s_hWorld, &mWorld ) );

    for( int iSubset = 0; iSubset < 2; iSubset++ )
    {
        V( s_pD3D9Effect->Begin( &cPasses, 0 ) );
        for( iPass = 0; iPass < cPasses; iPass++ )
        {
            V( s_pD3D9Effect->BeginPass( iPass ) );
            V( s_pD3D9Mesh->DrawSubset( iSubset ) );
            V( s_pD3D9Effect->EndPass() );
        }
        V( s_pD3D9Effect->End() );
    }

    return S_OK;
}

//--------------------------------------------------------------------------------------
HRESULT CDXUTDirectionWidget::UpdateLightDir()
{
    D3DXMATRIX mInvView;
    D3DXMatrixInverse( &mInvView, NULL, &m_mView );
    mInvView._41 = mInvView._42 = mInvView._43 = 0;

    D3DXMATRIX mLastRotInv;
    D3DXMatrixInverse( &mLastRotInv, NULL, &m_mRotSnapshot );

    D3DXMATRIX mRot = *m_ArcBall.GetRotationMatrix();
    m_mRotSnapshot = mRot;

    // Accumulate the delta of the arcball's rotation in view space.
    // Note that per-frame delta rotations could be problematic over long periods of time.
    m_mRot *= m_mView * mLastRotInv * mRot * mInvView;

    // Since we're accumulating delta rotations, we need to orthonormalize 
    // the matrix to prevent eventual matrix skew
    D3DXVECTOR3* pXBasis = ( D3DXVECTOR3* )&m_mRot._11;
    D3DXVECTOR3* pYBasis = ( D3DXVECTOR3* )&m_mRot._21;
    D3DXVECTOR3* pZBasis = ( D3DXVECTOR3* )&m_mRot._31;
    D3DXVec3Normalize( pXBasis, pXBasis );
    D3DXVec3Cross( pYBasis, pZBasis, pXBasis );
    D3DXVec3Normalize( pYBasis, pYBasis );
    D3DXVec3Cross( pZBasis, pXBasis, pYBasis );

    // Transform the default direction vector by the light's rotation matrix
    D3DXVec3TransformNormal( &m_vCurrentDir, &m_vDefaultDir, &m_mRot );

    return S_OK;
}

//--------------------------------------------------------------------------------------
HRESULT CDXUTDirectionWidget::StaticOnD3D11CreateDevice( ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dImmediateContext )
{
    

    //s_pd3d10Device = pd3dDevice;

    //const char* g_strBuffer =
    //    "float4 g_MaterialDiffuseColor;      // Material's diffuse color\r\n"
    //    "float4 g_LightDir;                  // Light's direction in world space\r\n"
    //    "float4x4 g_mWorld;                  // World matrix for object\r\n"
    //    "float4x4 g_mWorldViewProjection;    // World * View * Projection matrix\r\n"
    //    "\r\n"
    //    "struct VS_OUTPUT\r\n"
    //    "{\r\n"
    //    "    float4 Position   : SV_POSITION;   // vertex position\r\n"
    //    "    float4 Diffuse    : COLOR0;     // vertex diffuse color\r\n"
    //    "};\r\n"
    //    "\r\n"
    //    "VS_OUTPUT RenderWith1LightNoTextureVS( float3 vPos : POSITION,\r\n"
    //    "                                       float3 vNormal : NORMAL )\r\n"
    //    "{\r\n"
    //    "    VS_OUTPUT Output;\r\n"
    //    "\r\n"
    //    "    // Transform the position from object space to homogeneous projection space\r\n"
    //    "    Output.Position = mul( float4(vPos,1), g_mWorldViewProjection);\r\n"
    //    "\r\n"
    //    "    // Transform the normal from object space to world space\r\n"
    //    "    float3 vNormalWorldSpace;\r\n"
    //    "    vNormalWorldSpace = normalize(mul(vNormal, (float3x3)g_mWorld)); // normal (world space)\r\n"
    //    "\r\n"
    //    "    // Compute simple directional lighting equation\r\n"
    //    "    Output.Diffuse.rgb = g_MaterialDiffuseColor * max(0,dot(vNormalWorldSpace, g_LightDir));\r\n"
    //    "    Output.Diffuse.a = 1.0f;\r\n"
    //    "\r\n"
    //    "    return Output;\r\n"
    //    "}\r\n"
    //    "\r\n"
    //    "float4 RenderWith1LightNoTexturePS( VS_OUTPUT Input ) : SV_TARGET\r\n"
    //    "{\r\n"
    //    "    return Input.Diffuse;\r\n"
    //    "}\r\n"
    //    "\r\n"
    //    "technique10 RenderWith1LightNoTexture\r\n"
    //    "{\r\n"
    //    "    pass p0\r\n"
    //    "    {\r\n"
    //    "       SetVertexShader( CompileShader( vs_4_0, RenderWith1LightNoTextureVS() ) );\r\n"
    //    "       SetGeometryShader( NULL );\r\n"
    //    "       SetPixelShader( CompileShader( ps_4_0, RenderWith1LightNoTexturePS() ) );\r\n"
    //    "    }\r\n"
    //    "}\r\n"
    //    "";

    //UINT dwBufferSize = ( UINT )strlen( g_strBuffer ) + 1;

    //HRESULT hr = D3DX10CreateEffectFromMemory( g_strBuffer, dwBufferSize, "None", NULL, NULL, "fx_4_0",
    //                                           D3D10_SHADER_ENABLE_STRICTNESS, 0, pd3dDevice, NULL,
    //                                           NULL, &s_pD3D10Effect, NULL, NULL );
    //if( FAILED( hr ) )
    //    return hr;

    //s_pRenderTech = s_pD3D10Effect->GetTechniqueByName( "RenderWith1LightNoTexture" );
    //g_pMaterialDiffuseColor = s_pD3D10Effect->GetVariableByName( "g_MaterialDiffuseColor" )->AsVector();
    //g_pLightDir = s_pD3D10Effect->GetVariableByName( "g_LightDir" )->AsVector();
    //g_pmWorld = s_pD3D10Effect->GetVariableByName( "g_mWorld" )->AsMatrix();
    //g_pmWorldViewProjection = s_pD3D10Effect->GetVariableByName( "g_mWorldViewProjection" )->AsMatrix();

    //const D3D10_INPUT_ELEMENT_DESC layout[] =
    //{
    //    { "POSITION",  0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 0,  D3D10_INPUT_PER_VERTEX_DATA, 0 },
    //    { "NORMAL",    0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D10_INPUT_PER_VERTEX_DATA, 0 },
    //};
    //D3D10_PASS_DESC PassDesc;
    //V_RETURN( s_pRenderTech->GetPassByIndex( 0 )->GetDesc( &PassDesc ) );
    //V_RETURN( pd3dDevice->CreateInputLayout( layout, 2, PassDesc.pIAInputSignature,
    //                                         PassDesc.IAInputSignatureSize, &s_pVertexLayout ) );
    
    //TODO:  Add loading code here

    return S_OK;
}

//--------------------------------------------------------------------------------------
HRESULT CDXUTDirectionWidget::OnRender11( D3DXCOLOR color, const D3DXMATRIX* pmView, const D3DXMATRIX* pmProj,
                                          const D3DXVECTOR3* pEyePt )
{
   // NO D3DX11 YET
   // m_mView = *pmView;

   // // Render the light spheres so the user can visually see the light dir
   // D3DXMATRIX mRotate;
   // D3DXMATRIX mScale;
   // D3DXMATRIX mTrans;
   // D3DXMATRIXA16 mWorldViewProj;

   // g_pMaterialDiffuseColor->SetFloatVector( ( float* )&color );
   // D3DXVECTOR3 vEyePt;
   // D3DXVec3Normalize( &vEyePt, pEyePt );
   // g_pLightDir->SetFloatVector( ( float* )&vEyePt );

   // // Rotate arrow model to point towards origin
   // D3DXMATRIX mRotateA, mRotateB;
   // D3DXVECTOR3 vAt = D3DXVECTOR3( 0, 0, 0 );
   // D3DXVECTOR3 vUp = D3DXVECTOR3( 0, 1, 0 );
   // D3DXMatrixRotationX( &mRotateB, D3DX_PI );
   // D3DXMatrixLookAtLH( &mRotateA, &m_vCurrentDir, &vAt, &vUp );
   // D3DXMatrixInverse( &mRotateA, NULL, &mRotateA );
   // mRotate = mRotateB * mRotateA;

   // D3DXVECTOR3 vL = m_vCurrentDir * m_fRadius * 1.0f;
   // D3DXMatrixTranslation( &mTrans, vL.x, vL.y, vL.z );
   // D3DXMatrixScaling( &mScale, m_fRadius * 0.2f, m_fRadius * 0.2f, m_fRadius * 0.2f );

   // D3DXMATRIX mWorld = mRotate * mScale * mTrans;
   // mWorldViewProj = mWorld * ( m_mView )*( *pmProj );

   // g_pmWorldViewProjection->SetMatrix( ( float* )&mWorldViewProj );
   // g_pmWorld->SetMatrix( ( float* )&mWorld );

   // s_pd3d10Device->IASetInputLayout( s_pVertexLayout );
    
    //TODO:  Add rendering code here

    return S_OK;
}

//--------------------------------------------------------------------------------------
void CDXUTDirectionWidget::StaticOnD3D11DestroyDevice()
{
//    SAFE_RELEASE( s_pVertexLayout );
//    SAFE_RELEASE( s_pD3D11Effect );
}
