//-----------------------------------------------------------------------------
// File: SDKWaveFile.cpp
//
// Desc: Classes for reading and writing wav files. Feel free to use this class
//       as a starting point for adding extra functionality.
//
// XNA Developer Connection
//
// Copyright (c) Microsoft Corp. All rights reserved.
//-----------------------------------------------------------------------------
#define STRICT
#include "DXUT.h"
#include "SDKwavefile.h"
#undef min // use __min instead
#undef max // use __max instead

//-----------------------------------------------------------------------------
// Name: CWaveFile::CWaveFile()
// Desc: Constructs the class.  Call Open() to open a wave file for reading.
//       Then call Read() as needed.  Calling the destructor or Close()
//       will close the file.
//-----------------------------------------------------------------------------
CWaveFile::CWaveFile()
{
    m_pwfx = NULL;
    m_hmmio = NULL;
    m_pResourceBuffer = NULL;
    m_dwSize = 0;
    m_bIsReadingFromMemory = FALSE;
}


//-----------------------------------------------------------------------------
// Name: CWaveFile::~CWaveFile()
// Desc: Destructs the class
//-----------------------------------------------------------------------------
CWaveFile::~CWaveFile()
{
    Close();

    if( !m_bIsReadingFromMemory )
        SAFE_DELETE_ARRAY( m_pwfx );
}


//-----------------------------------------------------------------------------
// Name: CWaveFile::Open()
// Desc: Opens a wave file for reading
//-----------------------------------------------------------------------------
HRESULT CWaveFile::Open( LPWSTR strFileName, WAVEFORMATEX* pwfx, DWORD dwFlags )
{
    HRESULT hr;

    m_dwFlags = dwFlags;
    m_bIsReadingFromMemory = FALSE;

    if( m_dwFlags == WAVEFILE_READ )
    {
        if( strFileName == NULL )
            return E_INVALIDARG;
        SAFE_DELETE_ARRAY( m_pwfx );

        m_hmmio = mmioOpen( strFileName, NULL, MMIO_ALLOCBUF | MMIO_READ );

        if( NULL == m_hmmio )
        {
            HRSRC hResInfo;
            HGLOBAL hResData;
            DWORD dwSize;
            VOID* pvRes;

            // Loading it as a file failed, so try it as a resource
            if( NULL == ( hResInfo = FindResource( NULL, strFileName, L"WAVE" ) ) )
            {
                if( NULL == ( hResInfo = FindResource( NULL, strFileName, L"WAV" ) ) )
                    return DXTRACE_ERR( L"FindResource", E_FAIL );
            }

            if( NULL == ( hResData = LoadResource( GetModuleHandle( NULL ), hResInfo ) ) )
                return DXTRACE_ERR( L"LoadResource", E_FAIL );

            if( 0 == ( dwSize = SizeofResource( GetModuleHandle( NULL ), hResInfo ) ) )
                return DXTRACE_ERR( L"SizeofResource", E_FAIL );

            if( NULL == ( pvRes = LockResource( hResData ) ) )
                return DXTRACE_ERR( L"LockResource", E_FAIL );

            m_pResourceBuffer = new CHAR[ dwSize ];
            if( m_pResourceBuffer == NULL )
                return DXTRACE_ERR( L"new", E_OUTOFMEMORY );
            memcpy( m_pResourceBuffer, pvRes, dwSize );

            MMIOINFO mmioInfo;
            ZeroMemory( &mmioInfo, sizeof( mmioInfo ) );
            mmioInfo.fccIOProc = FOURCC_MEM;
            mmioInfo.cchBuffer = dwSize;
            mmioInfo.pchBuffer = ( CHAR* )m_pResourceBuffer;

            m_hmmio = mmioOpen( NULL, &mmioInfo, MMIO_ALLOCBUF | MMIO_READ );
        }

        if( FAILED( hr = ReadMMIO() ) )
        {
            // ReadMMIO will fail if its an not a wave file
            mmioClose( m_hmmio, 0 );
            return DXTRACE_ERR( L"ReadMMIO", hr );
        }

        if( FAILED( hr = ResetFile() ) )
            return DXTRACE_ERR( L"ResetFile", hr );

        // After the reset, the size of the wav file is m_ck.cksize so store it now
        m_dwSize = m_ck.cksize;
    }
    else
    {
        m_hmmio = mmioOpen( strFileName, NULL, MMIO_ALLOCBUF |
                            MMIO_READWRITE |
                            MMIO_CREATE );
        if( NULL == m_hmmio )
            return DXTRACE_ERR( L"mmioOpen", E_FAIL );

        if( FAILED( hr = WriteMMIO( pwfx ) ) )
        {
            mmioClose( m_hmmio, 0 );
            return DXTRACE_ERR( L"WriteMMIO", hr );
        }

        if( FAILED( hr = ResetFile() ) )
            return DXTRACE_ERR( L"ResetFile", hr );
    }

    return hr;
}


//-----------------------------------------------------------------------------
// Name: CWaveFile::OpenFromMemory()
// Desc: copy data to CWaveFile member variable from memory
//-----------------------------------------------------------------------------
HRESULT CWaveFile::OpenFromMemory( BYTE* pbData, ULONG ulDataSize,
                                   WAVEFORMATEX* pwfx, DWORD dwFlags )
{
    m_pwfx = pwfx;
    m_ulDataSize = ulDataSize;
    m_pbData = pbData;
    m_pbDataCur = m_pbData;
    m_bIsReadingFromMemory = TRUE;

    if( dwFlags != WAVEFILE_READ )
        return E_NOTIMPL;

    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: CWaveFile::ReadMMIO()
// Desc: Support function for reading from a multimedia I/O stream.
//       m_hmmio must be valid before calling.  This function uses it to
//       update m_ckRiff, and m_pwfx.
//-----------------------------------------------------------------------------
HRESULT CWaveFile::ReadMMIO()
{
    MMCKINFO ckIn;           // chunk info. for general use.
    PCMWAVEFORMAT pcmWaveFormat;  // Temp PCM structure to load in.

    m_pwfx = NULL;

    if( ( 0 != mmioDescend( m_hmmio, &m_ckRiff, NULL, 0 ) ) )
        return DXTRACE_ERR( L"mmioDescend", E_FAIL );

    // Check to make sure this is a valid wave file
    if( ( m_ckRiff.ckid != FOURCC_RIFF ) ||
        ( m_ckRiff.fccType != mmioFOURCC( 'W', 'A', 'V', 'E' ) ) )
        return DXTRACE_ERR( L"mmioFOURCC", E_FAIL );

    // Search the input file for for the 'fmt ' chunk.
    ckIn.ckid = mmioFOURCC( 'f', 'm', 't', ' ' );
    if( 0 != mmioDescend( m_hmmio, &ckIn, &m_ckRiff, MMIO_FINDCHUNK ) )
        return DXTRACE_ERR( L"mmioDescend", E_FAIL );

    // Expect the 'fmt' chunk to be at least as large as <PCMWAVEFORMAT>;
    // if there are extra parameters at the end, we'll ignore them
    if( ckIn.cksize < ( LONG )sizeof( PCMWAVEFORMAT ) )
        return DXTRACE_ERR( L"sizeof(PCMWAVEFORMAT)", E_FAIL );

    // Read the 'fmt ' chunk into <pcmWaveFormat>.
    if( mmioRead( m_hmmio, ( HPSTR )&pcmWaveFormat,
                  sizeof( pcmWaveFormat ) ) != sizeof( pcmWaveFormat ) )
        return DXTRACE_ERR( L"mmioRead", E_FAIL );

    // Allocate the waveformatex, but if its not pcm format, read the next
    // word, and thats how many extra bytes to allocate.
    if( pcmWaveFormat.wf.wFormatTag == WAVE_FORMAT_PCM )
    {
        m_pwfx = ( WAVEFORMATEX* )new CHAR[ sizeof( WAVEFORMATEX ) ];
        if( NULL == m_pwfx )
            return DXTRACE_ERR( L"m_pwfx", E_FAIL );

        // Copy the bytes from the pcm structure to the waveformatex structure
        memcpy( m_pwfx, &pcmWaveFormat, sizeof( pcmWaveFormat ) );
        m_pwfx->cbSize = 0;
    }
    else
    {
        // Read in length of extra bytes.
        WORD cbExtraBytes = 0L;
        if( mmioRead( m_hmmio, ( CHAR* )&cbExtraBytes, sizeof( WORD ) ) != sizeof( WORD ) )
            return DXTRACE_ERR( L"mmioRead", E_FAIL );

        m_pwfx = ( WAVEFORMATEX* )new CHAR[ sizeof( WAVEFORMATEX ) + cbExtraBytes ];
        if( NULL == m_pwfx )
            return DXTRACE_ERR( L"new", E_FAIL );

        // Copy the bytes from the pcm structure to the waveformatex structure
        memcpy( m_pwfx, &pcmWaveFormat, sizeof( pcmWaveFormat ) );
        m_pwfx->cbSize = cbExtraBytes;

        // Now, read those extra bytes into the structure, if cbExtraAlloc != 0.
        if( mmioRead( m_hmmio, ( CHAR* )( ( ( BYTE* )&( m_pwfx->cbSize ) ) + sizeof( WORD ) ),
                      cbExtraBytes ) != cbExtraBytes )
        {
            SAFE_DELETE( m_pwfx );
            return DXTRACE_ERR( L"mmioRead", E_FAIL );
        }
    }

    // Ascend the input file out of the 'fmt ' chunk.
    if( 0 != mmioAscend( m_hmmio, &ckIn, 0 ) )
    {
        SAFE_DELETE( m_pwfx );
        return DXTRACE_ERR( L"mmioAscend", E_FAIL );
    }

    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: CWaveFile::GetSize()
// Desc: Retuns the size of the read access wave file
//-----------------------------------------------------------------------------
DWORD CWaveFile::GetSize()
{
    return m_dwSize;
}


//-----------------------------------------------------------------------------
// Name: CWaveFile::ResetFile()
// Desc: Resets the internal m_ck pointer so reading starts from the
//       beginning of the file again
//-----------------------------------------------------------------------------
HRESULT CWaveFile::ResetFile()
{
    if( m_bIsReadingFromMemory )
    {
        m_pbDataCur = m_pbData;
    }
    else
    {
        if( m_hmmio == NULL )
            return CO_E_NOTINITIALIZED;

        if( m_dwFlags == WAVEFILE_READ )
        {
            // Seek to the data
            if( -1 == mmioSeek( m_hmmio, m_ckRiff.dwDataOffset + sizeof( FOURCC ),
                                SEEK_SET ) )
                return DXTRACE_ERR( L"mmioSeek", E_FAIL );

            // Search the input file for the 'data' chunk.
            m_ck.ckid = mmioFOURCC( 'd', 'a', 't', 'a' );
            if( 0 != mmioDescend( m_hmmio, &m_ck, &m_ckRiff, MMIO_FINDCHUNK ) )
                return DXTRACE_ERR( L"mmioDescend", E_FAIL );
        }
        else
        {
            // Create the 'data' chunk that holds the waveform samples.
            m_ck.ckid = mmioFOURCC( 'd', 'a', 't', 'a' );
            m_ck.cksize = 0;

            if( 0 != mmioCreateChunk( m_hmmio, &m_ck, 0 ) )
                return DXTRACE_ERR( L"mmioCreateChunk", E_FAIL );

            if( 0 != mmioGetInfo( m_hmmio, &m_mmioinfoOut, 0 ) )
                return DXTRACE_ERR( L"mmioGetInfo", E_FAIL );
        }
    }

    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: CWaveFile::Read()
// Desc: Reads section of data from a wave file into pBuffer and returns
//       how much read in pdwSizeRead, reading not more than dwSizeToRead.
//       This uses m_ck to determine where to start reading from.  So
//       subsequent calls will be continue where the last left off unless
//       Reset() is called.
//-----------------------------------------------------------------------------
HRESULT CWaveFile::Read( BYTE* pBuffer, DWORD dwSizeToRead, DWORD* pdwSizeRead )
{
    if( m_bIsReadingFromMemory )
    {
        if( m_pbDataCur == NULL )
            return CO_E_NOTINITIALIZED;
        if( pdwSizeRead != NULL )
            *pdwSizeRead = 0;

        if( ( BYTE* )( m_pbDataCur + dwSizeToRead ) >
            ( BYTE* )( m_pbData + m_ulDataSize ) )
        {
            dwSizeToRead = m_ulDataSize - ( DWORD )( m_pbDataCur - m_pbData );
        }

#pragma warning( disable: 4616 )    // disable warning about warning number '22104' being out of range
#pragma warning( disable: 22104 )   // disable PREfast warning during static code analysis
        CopyMemory( pBuffer, m_pbDataCur, dwSizeToRead );
#pragma warning( default: 22104 )
#pragma warning( default: 4616 )

        if( pdwSizeRead != NULL )
            *pdwSizeRead = dwSizeToRead;

        return S_OK;
    }
    else
    {
        MMIOINFO mmioinfoIn; // current status of m_hmmio

        if( m_hmmio == NULL )
            return CO_E_NOTINITIALIZED;
        if( pBuffer == NULL || pdwSizeRead == NULL )
            return E_INVALIDARG;

        *pdwSizeRead = 0;

        if( 0 != mmioGetInfo( m_hmmio, &mmioinfoIn, 0 ) )
            return DXTRACE_ERR( L"mmioGetInfo", E_FAIL );

        UINT cbDataIn = dwSizeToRead;
        if( cbDataIn > m_ck.cksize )
            cbDataIn = m_ck.cksize;

        m_ck.cksize -= cbDataIn;

        for( DWORD cT = 0; cT < cbDataIn; cT++ )
        {
            // Copy the bytes from the io to the buffer.
            if( mmioinfoIn.pchNext == mmioinfoIn.pchEndRead )
            {
                if( 0 != mmioAdvance( m_hmmio, &mmioinfoIn, MMIO_READ ) )
                    return DXTRACE_ERR( L"mmioAdvance", E_FAIL );

                if( mmioinfoIn.pchNext == mmioinfoIn.pchEndRead )
                    return DXTRACE_ERR( L"mmioinfoIn.pchNext", E_FAIL );
            }

            // Actual copy.
            *( ( BYTE* )pBuffer + cT ) = *( ( BYTE* )mmioinfoIn.pchNext );
            mmioinfoIn.pchNext++;
        }

        if( 0 != mmioSetInfo( m_hmmio, &mmioinfoIn, 0 ) )
            return DXTRACE_ERR( L"mmioSetInfo", E_FAIL );

        *pdwSizeRead = cbDataIn;

        return S_OK;
    }
}


//-----------------------------------------------------------------------------
// Name: CWaveFile::Close()
// Desc: Closes the wave file
//-----------------------------------------------------------------------------
HRESULT CWaveFile::Close()
{
    if( m_dwFlags == WAVEFILE_READ )
    {
        mmioClose( m_hmmio, 0 );
        m_hmmio = NULL;
        SAFE_DELETE_ARRAY( m_pResourceBuffer );
    }
    else
    {
        m_mmioinfoOut.dwFlags |= MMIO_DIRTY;

        if( m_hmmio == NULL )
            return CO_E_NOTINITIALIZED;

        if( 0 != mmioSetInfo( m_hmmio, &m_mmioinfoOut, 0 ) )
            return DXTRACE_ERR( L"mmioSetInfo", E_FAIL );

        // Ascend the output file out of the 'data' chunk -- this will cause
        // the chunk size of the 'data' chunk to be written.
        if( 0 != mmioAscend( m_hmmio, &m_ck, 0 ) )
            return DXTRACE_ERR( L"mmioAscend", E_FAIL );

        // Do this here instead...
        if( 0 != mmioAscend( m_hmmio, &m_ckRiff, 0 ) )
            return DXTRACE_ERR( L"mmioAscend", E_FAIL );

        mmioSeek( m_hmmio, 0, SEEK_SET );

        if( 0 != ( INT )mmioDescend( m_hmmio, &m_ckRiff, NULL, 0 ) )
            return DXTRACE_ERR( L"mmioDescend", E_FAIL );

        m_ck.ckid = mmioFOURCC( 'f', 'a', 'c', 't' );

        if( 0 == mmioDescend( m_hmmio, &m_ck, &m_ckRiff, MMIO_FINDCHUNK ) )
        {
            DWORD dwSamples = 0;
            mmioWrite( m_hmmio, ( HPSTR )&dwSamples, sizeof( DWORD ) );
            mmioAscend( m_hmmio, &m_ck, 0 );
        }

        // Ascend the output file out of the 'RIFF' chunk -- this will cause
        // the chunk size of the 'RIFF' chunk to be written.
        if( 0 != mmioAscend( m_hmmio, &m_ckRiff, 0 ) )
            return DXTRACE_ERR( L"mmioAscend", E_FAIL );

        mmioClose( m_hmmio, 0 );
        m_hmmio = NULL;
    }

    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: CWaveFile::WriteMMIO()
// Desc: Support function for reading from a multimedia I/O stream
//       pwfxDest is the WAVEFORMATEX for this new wave file.
//       m_hmmio must be valid before calling.  This function uses it to
//       update m_ckRiff, and m_ck.
//-----------------------------------------------------------------------------
HRESULT CWaveFile::WriteMMIO( WAVEFORMATEX* pwfxDest )
{
    DWORD dwFactChunk; // Contains the actual fact chunk. Garbage until WaveCloseWriteFile.
    MMCKINFO ckOut1;

    dwFactChunk = ( DWORD )-1;

    // Create the output file RIFF chunk of form type 'WAVE'.
    m_ckRiff.fccType = mmioFOURCC( 'W', 'A', 'V', 'E' );
    m_ckRiff.cksize = 0;

    if( 0 != mmioCreateChunk( m_hmmio, &m_ckRiff, MMIO_CREATERIFF ) )
        return DXTRACE_ERR( L"mmioCreateChunk", E_FAIL );

    // We are now descended into the 'RIFF' chunk we just created.
    // Now create the 'fmt ' chunk. Since we know the size of this chunk,
    // specify it in the MMCKINFO structure so MMIO doesn't have to seek
    // back and set the chunk size after ascending from the chunk.
    m_ck.ckid = mmioFOURCC( 'f', 'm', 't', ' ' );
    m_ck.cksize = sizeof( PCMWAVEFORMAT );

    if( 0 != mmioCreateChunk( m_hmmio, &m_ck, 0 ) )
        return DXTRACE_ERR( L"mmioCreateChunk", E_FAIL );

    // Write the PCMWAVEFORMAT structure to the 'fmt ' chunk if its that type.
    if( pwfxDest->wFormatTag == WAVE_FORMAT_PCM )
    {
        if( mmioWrite( m_hmmio, ( HPSTR )pwfxDest,
                       sizeof( PCMWAVEFORMAT ) ) != sizeof( PCMWAVEFORMAT ) )
            return DXTRACE_ERR( L"mmioWrite", E_FAIL );
    }
    else
    {
        // Write the variable length size.
        if( ( UINT )mmioWrite( m_hmmio, ( HPSTR )pwfxDest,
                               sizeof( *pwfxDest ) + pwfxDest->cbSize ) !=
            ( sizeof( *pwfxDest ) + pwfxDest->cbSize ) )
            return DXTRACE_ERR( L"mmioWrite", E_FAIL );
    }

    // Ascend out of the 'fmt ' chunk, back into the 'RIFF' chunk.
    if( 0 != mmioAscend( m_hmmio, &m_ck, 0 ) )
        return DXTRACE_ERR( L"mmioAscend", E_FAIL );

    // Now create the fact chunk, not required for PCM but nice to have.  This is filled
    // in when the close routine is called.
    ckOut1.ckid = mmioFOURCC( 'f', 'a', 'c', 't' );
    ckOut1.cksize = 0;

    if( 0 != mmioCreateChunk( m_hmmio, &ckOut1, 0 ) )
        return DXTRACE_ERR( L"mmioCreateChunk", E_FAIL );

    if( mmioWrite( m_hmmio, ( HPSTR )&dwFactChunk, sizeof( dwFactChunk ) ) !=
        sizeof( dwFactChunk ) )
        return DXTRACE_ERR( L"mmioWrite", E_FAIL );

    // Now ascend out of the fact chunk...
    if( 0 != mmioAscend( m_hmmio, &ckOut1, 0 ) )
        return DXTRACE_ERR( L"mmioAscend", E_FAIL );

    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: CWaveFile::Write()
// Desc: Writes data to the open wave file
//-----------------------------------------------------------------------------
HRESULT CWaveFile::Write( UINT nSizeToWrite, BYTE* pbSrcData, UINT* pnSizeWrote )
{
    UINT cT;

    if( m_bIsReadingFromMemory )
        return E_NOTIMPL;
    if( m_hmmio == NULL )
        return CO_E_NOTINITIALIZED;
    if( pnSizeWrote == NULL || pbSrcData == NULL )
        return E_INVALIDARG;

    *pnSizeWrote = 0;

    for( cT = 0; cT < nSizeToWrite; cT++ )
    {
        if( m_mmioinfoOut.pchNext == m_mmioinfoOut.pchEndWrite )
        {
            m_mmioinfoOut.dwFlags |= MMIO_DIRTY;
            if( 0 != mmioAdvance( m_hmmio, &m_mmioinfoOut, MMIO_WRITE ) )
                return DXTRACE_ERR( L"mmioAdvance", E_FAIL );
        }

        *( ( BYTE* )m_mmioinfoOut.pchNext ) = *( ( BYTE* )pbSrcData + cT );
        ( BYTE* )m_mmioinfoOut.pchNext++;

        ( *pnSizeWrote )++;
    }

    return S_OK;
}
