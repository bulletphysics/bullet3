//  ---------------------------------------------------------------------------
//
//  @file       TwOpenGLCore.h
//  @brief      OpenGL Core graph functions
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  note:       Private header
//
//  ---------------------------------------------------------------------------


#if !defined ANT_TW_OPENGL_CORE_INCLUDED
#define ANT_TW_OPENGL_CORE_INCLUDED

#include "TwGraph.h"

//  ---------------------------------------------------------------------------

class CTwGraphOpenGLCore : public ITwGraph
{
public:
    virtual int         Init();
    virtual int         Shut();
    virtual void        BeginDraw(int _WndWidth, int _WndHeight);
    virtual void        EndDraw();
    virtual bool        IsDrawing();
    virtual void        Restore();
    virtual void        DrawLine(int _X0, int _Y0, int _X1, int _Y1, color32 _Color0, color32 _Color1, bool _AntiAliased=false);
    virtual void        DrawLine(int _X0, int _Y0, int _X1, int _Y1, color32 _Color, bool _AntiAliased=false) { DrawLine(_X0, _Y0, _X1, _Y1, _Color, _Color, _AntiAliased); }
    virtual void        DrawRect(int _X0, int _Y0, int _X1, int _Y1, color32 _Color00, color32 _Color10, color32 _Color01, color32 _Color11);
    virtual void        DrawRect(int _X0, int _Y0, int _X1, int _Y1, color32 _Color) { DrawRect(_X0, _Y0, _X1, _Y1, _Color, _Color, _Color, _Color); }
    virtual void        DrawTriangles(int _NumTriangles, int *_Vertices, color32 *_Colors, Cull _CullMode);

    virtual void *      NewTextObj();
    virtual void        DeleteTextObj(void *_TextObj);
    virtual void        BuildText(void *_TextObj, const std::string *_TextLines, color32 *_LineColors, color32 *_LineBgColors, int _NbLines, const CTexFont *_Font, int _Sep, int _BgWidth);
    virtual void        DrawText(void *_TextObj, int _X, int _Y, color32 _Color, color32 _BgColor);

    virtual void        ChangeViewport(int _X0, int _Y0, int _Width, int _Height, int _OffsetX, int _OffsetY);
    virtual void        RestoreViewport();
    virtual void        SetScissor(int _X0, int _Y0, int _Width, int _Height);

protected:
    bool                m_Drawing;
    GLuint              m_FontTexID;
    const CTexFont *    m_FontTex;
    
    GLfloat             m_PrevLineWidth;
    GLint               m_PrevActiveTexture;
    GLint               m_PrevTexture;
    GLint               m_PrevVArray;
    GLboolean           m_PrevLineSmooth;
    GLboolean           m_PrevCullFace;
    GLboolean           m_PrevDepthTest;
    GLboolean           m_PrevBlend;
    GLint               m_PrevSrcBlend;
    GLint               m_PrevDstBlend;
    GLboolean           m_PrevScissorTest;
    GLint               m_PrevScissorBox[4];
    GLint               m_PrevViewport[4];
    GLuint              m_PrevProgramObject;

    GLuint              m_LineRectVS;
    GLuint              m_LineRectFS;
    GLuint              m_LineRectProgram;
    GLuint              m_LineRectVArray;
    GLuint              m_LineRectVertices;
    GLuint              m_LineRectColors;
    GLuint              m_TriVS;
    GLuint              m_TriFS;
    GLuint              m_TriProgram;
    GLuint              m_TriUniVS;
    GLuint              m_TriUniFS;
    GLuint              m_TriUniProgram;
    GLuint              m_TriTexVS;
    GLuint              m_TriTexFS;
    GLuint              m_TriTexProgram;
    GLuint              m_TriTexUniVS;
    GLuint              m_TriTexUniFS;
    GLuint              m_TriTexUniProgram;
    GLuint              m_TriVArray;
    GLuint              m_TriVertices;
    GLuint              m_TriUVs;
    GLuint              m_TriColors;
    GLint               m_TriLocationOffset;
    GLint               m_TriLocationWndSize;
    GLint               m_TriUniLocationOffset;
    GLint               m_TriUniLocationWndSize;
    GLint               m_TriUniLocationColor;
    GLint               m_TriTexLocationOffset;
    GLint               m_TriTexLocationWndSize;
    GLint               m_TriTexLocationTexture;
    GLint               m_TriTexUniLocationOffset;
    GLint               m_TriTexUniLocationWndSize;
    GLint               m_TriTexUniLocationColor;
    GLint               m_TriTexUniLocationTexture;
    size_t              m_TriBufferSize;

    int                 m_WndWidth;
    int                 m_WndHeight;
    int                 m_OffsetX;
    int                 m_OffsetY;

    struct Vec2         { GLfloat x, y; Vec2(){} Vec2(GLfloat _X, GLfloat _Y):x(_X),y(_Y){} Vec2(int _X, int _Y):x(GLfloat(_X)),y(GLfloat(_Y)){} };
    struct CTextObj
    {
        std::vector<Vec2>   m_TextVerts;
        std::vector<Vec2>   m_TextUVs;
        std::vector<Vec2>   m_BgVerts;
        std::vector<color32>m_Colors;
        std::vector<color32>m_BgColors;
    };
    void                ResizeTriBuffers(size_t _NewSize);
};

//  ---------------------------------------------------------------------------


#endif // !defined ANT_TW_OPENGL_CORE_INCLUDED
