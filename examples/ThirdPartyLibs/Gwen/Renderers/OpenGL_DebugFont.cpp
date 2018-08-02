
#include "OpenGL_DebugFont.h"
#include "Gwen/Utility.h"
#include "Gwen/Font.h"
#include "Gwen/Texture.h"

#include <math.h>
#ifdef B3_USE_GLFW
#include "glad/glad.h"
#include <GLFW/glfw3.h>
#else

#if defined(__APPLE__) && !defined (VMDMESA)
	#include <OpenGL/OpenGL.h>
	#include <OpenGL/gl.h>
#else
#ifdef GLEW_STATIC
#include "glad/glad.h"
#else
#ifdef NO_GLEW
#define GL_GLEXT_LEGACY
#include "third_party/GL/gl/include/GL/gl.h"
#include "third_party/GL/gl/include/GL/glext.h"
#else

#ifdef BT_NO_GLAD
#include <GL/glew.h>
#else
#include "glad/glad.h"
#endif

#endif //NO_GLEW
#endif //GLEW_STATIC
#endif//(__APPLE__)
#endif

#include "FontData.h"

	//saved OpenGL settings
					GLfloat             m_PrevLineWidth;
					GLint               m_PrevTexEnv;
					GLint               m_PrevPolygonMode[2];
					GLint               m_MaxClipPlanes;
					GLint               m_PrevTexture;
					GLint               m_PrevArrayBufferARB;
					GLint               m_PrevElementArrayBufferARB;
					GLboolean           m_PrevVertexProgramARB;
					GLboolean           m_PrevFragmentProgramARB;
					GLuint              m_PrevProgramObjectARB;
					GLboolean           m_PrevTexture3D;
					GLboolean           m_PrevActiveTexture1D[32];
					GLboolean           m_PrevActiveTexture2D[32];
					GLboolean           m_PrevActiveTexture3D[32];
					GLint               m_PrevActiveTextureARB;
					bool                m_SupportTexRect;
					GLboolean           m_PrevTexRectARB;
					GLint               m_PrevBlendEquation;
					GLint               m_PrevBlendEquationRGB;
					GLint               m_PrevBlendEquationAlpha;
					GLint               m_PrevBlendSrcRGB;
					GLint               m_PrevBlendDstRGB;
					GLint               m_PrevBlendSrcAlpha;
					GLint               m_PrevBlendDstAlpha;
					GLint               m_ViewportInit[4];
					GLfloat             m_ProjMatrixInit[16];
					GLboolean			m_texGenS;
					GLboolean			m_texGenT;
					GLboolean			m_texGenR;



				void	restoreOpenGLState()
			{
				glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, m_PrevTexEnv);
				glLineWidth(m_PrevLineWidth);
				glMatrixMode(GL_PROJECTION);
				glPopMatrix();
				glMatrixMode(GL_MODELVIEW);
				glPopMatrix();
				glMatrixMode(GL_TEXTURE);
				glPopMatrix();
				glPopClientAttrib();
				glPopAttrib();
				if (m_texGenS)
					glEnable(GL_TEXTURE_GEN_S);
				else
					glDisable(GL_TEXTURE_GEN_S);

				if (m_texGenT)
					glEnable(GL_TEXTURE_GEN_T);
				else
					glDisable(GL_TEXTURE_GEN_T);

				if (m_texGenR)
					glEnable(GL_TEXTURE_GEN_R);
				else
					glDisable(GL_TEXTURE_GEN_R);



			}

			void	saveOpenGLState(int screenWidth, int screenHeight)
			{
				glPushAttrib(GL_ALL_ATTRIB_BITS);
				glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);

				glMatrixMode(GL_TEXTURE);
				glPushMatrix();
				glLoadIdentity();
				glMatrixMode(GL_MODELVIEW);
				glPushMatrix();
				glLoadIdentity();
				glMatrixMode(GL_PROJECTION);
				glPushMatrix();
				GLint Vp[4];
				glGetIntegerv(GL_VIEWPORT, Vp);
				if (screenWidth>0 && screenHeight>0)
				{
					Vp[0] = 0;
					Vp[1] = 0;
					Vp[2] = screenWidth-1;
					Vp[3] = screenHeight-1;
					glViewport(Vp[0], Vp[1], Vp[2], Vp[3]);
				}
				glLoadIdentity();
				glOrtho(Vp[0], Vp[0]+Vp[2], Vp[1]+Vp[3], Vp[1], -1, 1);
				glGetIntegerv(GL_VIEWPORT, m_ViewportInit);
				glGetFloatv(GL_PROJECTION_MATRIX, m_ProjMatrixInit);

				glGetFloatv(GL_LINE_WIDTH, &m_PrevLineWidth);
			 //   glDisable(GL_POLYGON_STIPPLE);
				glLineWidth(1);

				glGetBooleanv(GL_TEXTURE_GEN_S,&m_texGenS);
				glGetBooleanv(GL_TEXTURE_GEN_T,&m_texGenT);
				glGetBooleanv(GL_TEXTURE_GEN_R,&m_texGenR);

				glDisable(GL_TEXTURE_GEN_S);
				glDisable(GL_TEXTURE_GEN_T);
				glDisable(GL_TEXTURE_GEN_R);

				glDisable(GL_LINE_SMOOTH);
			//    glDisable(GL_LINE_STIPPLE);
				glDisable(GL_CULL_FACE);
				glDisable(GL_DEPTH_TEST);
				glDisable(GL_LIGHTING);
				glEnable(GL_BLEND);

				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glGetTexEnviv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, &m_PrevTexEnv);
				glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
				glDisable(GL_TEXTURE_2D);

			}


namespace Gwen
{
	namespace Renderer
	{
        
		OpenGL_DebugFont::OpenGL_DebugFont(float retinaScale)
        :m_retinaScale(retinaScale)
		{
			m_iVertNum = 0;

			for ( int i=0; i<MaxVerts; i++ )
			{
				m_Vertices[ i ].z = 0.5f;
			}

			m_fLetterSpacing = 1.0f/16.0f;
			m_fFontScale[0] = 1.5f;
			m_fFontScale[1] = 1.5f;

			m_pFontTexture = new Gwen::Texture();

			// Create a little texture pointer..
			GLuint* pglTexture = new GLuint;

			// Sort out our GWEN texture
			m_pFontTexture->data = pglTexture;
			m_pFontTexture->width = 256;
			m_pFontTexture->height = 256;


			// Create the opengl texture
			glGenTextures( 1, pglTexture );
			glBindTexture( GL_TEXTURE_2D, *pglTexture );
			//glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
			//glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
			glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

			//GLenum format = GL_RGB;
			unsigned char* texdata = new unsigned char[256*256*4];
			for (int i=0;i<256*256;i++)
			{
				texdata[i*4] = sGwenFontData[i];
				texdata[i*4+1] = sGwenFontData[i];
				texdata[i*4+2] = sGwenFontData[i];
				texdata[i*4+3] = sGwenFontData[i];
			}
			glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, m_pFontTexture->width, m_pFontTexture->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const GLvoid*)texdata );
			delete[]texdata;
		}

		OpenGL_DebugFont::~OpenGL_DebugFont()
		{
			FreeTexture( m_pFontTexture );
			delete m_pFontTexture;
		}

		



		void OpenGL_DebugFont::Begin()
		{
			glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
			glAlphaFunc( GL_GREATER, 1.0f );	
			glEnable ( GL_BLEND );
		}

		void OpenGL_DebugFont::End()
		{
			if ( m_iVertNum == 0 ) return;

			glVertexPointer( 3, GL_FLOAT,  sizeof(Vertex), (void*) &m_Vertices[0].x );
			glEnableClientState( GL_VERTEX_ARRAY );

			glColorPointer( 4, GL_UNSIGNED_BYTE, sizeof(Vertex), (void*)&m_Vertices[0].r );
			glEnableClientState( GL_COLOR_ARRAY );

			glTexCoordPointer( 2, GL_FLOAT, sizeof(Vertex), (void*) &m_Vertices[0].u );
			glEnableClientState( GL_TEXTURE_COORD_ARRAY );

			glDrawArrays( GL_TRIANGLES, 0, (GLsizei) m_iVertNum );

			m_iVertNum = 0;
			glFlush();
		}

		void OpenGL_DebugFont::Flush()
		{
			if ( m_iVertNum == 0 ) return;

			glVertexPointer( 3, GL_FLOAT,  sizeof(Vertex), (void*) &m_Vertices[0].x );
			glEnableClientState( GL_VERTEX_ARRAY );

			glColorPointer( 4, GL_UNSIGNED_BYTE, sizeof(Vertex), (void*)&m_Vertices[0].r );
			glEnableClientState( GL_COLOR_ARRAY );

			glTexCoordPointer( 2, GL_FLOAT, sizeof(Vertex), (void*) &m_Vertices[0].u );
			glEnableClientState( GL_TEXTURE_COORD_ARRAY );

			glDrawArrays( GL_TRIANGLES, 0, (GLsizei) m_iVertNum );

			m_iVertNum = 0;
			glFlush();
		}

		void OpenGL_DebugFont::AddVert( int x, int y, float u, float v )
		{
			if ( m_iVertNum >= MaxVerts-1 )
			{
				Flush();
			}

			m_Vertices[ m_iVertNum ].x = (float)x*m_retinaScale;
			m_Vertices[ m_iVertNum ].y = (float)y*m_retinaScale;
			m_Vertices[ m_iVertNum ].u = u;
			m_Vertices[ m_iVertNum ].v = v;

			m_Vertices[ m_iVertNum ].r = m_Color.r;
			m_Vertices[ m_iVertNum ].g = m_Color.g;
			m_Vertices[ m_iVertNum ].b = m_Color.b;
			m_Vertices[ m_iVertNum ].a = m_Color.a;

			m_iVertNum++;
		}

		void OpenGL_DebugFont::DrawFilledRect( Gwen::Rect rect )
		{
			GLboolean texturesOn;

			glGetBooleanv(GL_TEXTURE_2D, &texturesOn);
			if ( texturesOn )
			{
				Flush();
				glDisable(GL_TEXTURE_2D);
			}	

			Translate( rect );

			AddVert( rect.x, rect.y );
			AddVert( rect.x+rect.w, rect.y );
			AddVert( rect.x, rect.y + rect.h );

			AddVert( rect.x+rect.w, rect.y );
			AddVert( rect.x+rect.w, rect.y+rect.h );
			AddVert( rect.x, rect.y + rect.h );
		}

		void OpenGL_DebugFont::SetDrawColor(Gwen::Color color)
		{
			glColor4ubv( (GLubyte*)&color );
			m_Color = color;
		}

		void OpenGL_DebugFont::StartClip()
		{
			Flush();
			Gwen::Rect rect = ClipRegion();

            float retinaScale = m_retinaScale;
            // OpenGL's coords are from the bottom left
            // so we need to translate them here.
            {
                GLint view[4];
                glGetIntegerv( GL_VIEWPORT, &view[0] );
                rect.y = view[3]/retinaScale - (rect.y + rect.h);
            }
            
            glScissor( retinaScale * rect.x * Scale(), retinaScale * rect.y * Scale(), retinaScale * rect.w * Scale(), retinaScale * rect.h * Scale() );
            glEnable( GL_SCISSOR_TEST );
            //glDisable( GL_SCISSOR_TEST );
            
		};

		void OpenGL_DebugFont::EndClip()
		{
				
			Flush();
			glDisable( GL_SCISSOR_TEST );
			
		};

		void OpenGL_DebugFont::RenderText( Gwen::Font* pFont, Gwen::Point pos, const Gwen::UnicodeString& text )
		{
			
			float fSize = pFont->size * Scale();

			if ( !text.length() )
				return;

			Gwen::String converted_string = Gwen::Utility::UnicodeToString( text );

			float yOffset=0.0f;
			for ( int i=0; i<(int)text.length(); i++ )
			{
			//	wchar_t chr = text[i];
				char ch = converted_string[i];
				float curSpacing = sGwenDebugFontSpacing[(int)ch] * m_fLetterSpacing * fSize * m_fFontScale[0];
				Gwen::Rect r( pos.x + yOffset, pos.y-fSize*0.2f, (fSize * m_fFontScale[0]), fSize * m_fFontScale[1] );

				if ( m_pFontTexture )
				{
					float uv_texcoords[8]={0.,0.,1.,1.};

					if ( ch >= 0 )
					{
						float cx= (ch%16)/16.0;
						float cy= (ch/16)/16.0;
						uv_texcoords[0] = cx;			
						uv_texcoords[1] = cy;
						uv_texcoords[4] = float(cx+1.0f/16.0f);	
						uv_texcoords[5] = float(cy+1.0f/16.0f);
					}

					DrawTexturedRect( m_pFontTexture, r, uv_texcoords[0], uv_texcoords[5], uv_texcoords[4], uv_texcoords[1] );
					yOffset+=curSpacing;
				} 
				else
				{
					DrawFilledRect( r );
					yOffset+=curSpacing;

				}
			}

		}

		void OpenGL_DebugFont::DrawTexturedRect( Gwen::Texture* pTexture, Gwen::Rect rect, float u1, float v1, float u2, float v2 )
		{
			GLuint* tex = (GLuint*)pTexture->data;

			// Missing image, not loaded properly?
			if ( !tex )
			{
				return DrawMissingImage( rect );
			}

			Translate( rect );
			GLuint boundtex;

			GLboolean texturesOn;
			glGetBooleanv(GL_TEXTURE_2D, &texturesOn);
			glGetIntegerv(GL_TEXTURE_BINDING_2D, (GLint *)&boundtex);
			if ( !texturesOn || *tex != boundtex )
			{
				Flush();
				glBindTexture( GL_TEXTURE_2D, *tex );
				glEnable(GL_TEXTURE_2D);
			}		

			AddVert( rect.x, rect.y,			u1, v1 );
			AddVert( rect.x+rect.w, rect.y,		u2, v1 );
			AddVert( rect.x, rect.y + rect.h,	u1, v2 );

			AddVert( rect.x+rect.w, rect.y,		u2, v1 );
			AddVert( rect.x+rect.w, rect.y+rect.h, u2, v2 );
			AddVert( rect.x, rect.y + rect.h, u1, v2 );			
		}

		Gwen::Point OpenGL_DebugFont::MeasureText( Gwen::Font* pFont, const Gwen::UnicodeString& text )
		{
			Gwen::Point p;
			float fSize = pFont->size * Scale();

			Gwen::String converted_string = Gwen::Utility::UnicodeToString( text );
			float spacing = 0.0f;

			for ( int i=0; i<(int)text.length(); i++ )
			{
				char ch = converted_string[i];
				spacing += sGwenDebugFontSpacing[(int)ch];
			}

			p.x = spacing*m_fLetterSpacing*fSize * m_fFontScale[0];
			p.y = pFont->size * Scale() * m_fFontScale[1];
			return p;
		}

	}
}
