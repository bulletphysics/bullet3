

#include "Gwen/Gwen.h"
#include "Gwen/Skins/Simple.h"
#include "../OpenGLWindow/OpenGLInclude.h"

#include "UnitTest.h"

extern unsigned char OpenSansData[];

#include "Gwen/Renderers/OpenGL_DebugFont.h"

#ifdef B3_USE_GLFW
#include "OpenGLWindow/GLFWOpenGLWindow.h"
#else
#ifdef __APPLE__
#include "OpenGLWindow/MacOpenGLWindow.h"
#else

//#include "CustomGL/glew.h"
#ifdef _WIN32
#include "OpenGLWindow/Win32OpenGLWindow.h"
#else
//let's cross the fingers it is Linux/X11
#include "OpenGLWindow/X11OpenGLWindow.h"
#endif //_WIN32
#endif//__APPLE__
#endif //B3_USE_GLFW
#include "OpenGLWindow/opengl_fontstashcallbacks.h"
#ifndef NO_OPENGL3
#include "OpenGLWindow/GwenOpenGL3CoreRenderer.h"
#include "OpenGLWindow/GLPrimitiveRenderer.h"
#endif
#include <assert.h>

Gwen::Controls::Canvas*		pCanvas  = NULL;
Gwen::Skin::Simple skin;

void MyMouseMoveCallback( float x, float y)
{
	//b3DefaultMouseCallback(button,state,x,y);

	static int m_lastmousepos[2] = {0,0};
	static bool isInitialized = false;
	if (pCanvas)
	{
		if (!isInitialized)
		{
			isInitialized = true;
			m_lastmousepos[0] = x+1;
			m_lastmousepos[1] = y+1;
		}
		bool handled = pCanvas->InputMouseMoved(x,y,m_lastmousepos[0],m_lastmousepos[1]);
	}
}

void MyMouseButtonCallback(int button, int state, float x, float y)
{
	//b3DefaultMouseCallback(button,state,x,y);

	if (pCanvas)
	{
		bool handled = pCanvas->InputMouseMoved(x,y,x, y);

		if (button>=0)
		{
			handled = pCanvas->InputMouseButton(button,state);
			if (handled)
			{
				if (!state)
					return;
			}
		}
	}
}

int sWidth = 800;//1050;
int sHeight = 600;//768;
GLPrimitiveRenderer* primRenderer=0;
//GwenOpenGL3CoreRenderer* gwenRenderer=0;
Gwen::Renderer::Base* gwenRenderer =0;

static void MyResizeCallback( float width, float height)
{
	sWidth = width;
	sHeight = height;
//	printf("resize(%d,%d)\n",sWidth,sHeight);
#ifndef NO_OPENGL3
	if (primRenderer)
	{
		primRenderer->setScreenSize(width,height);
	}
#endif
	if (gwenRenderer)
	{
		gwenRenderer->Resize(width,height);
	}
	if (pCanvas)
	{
		pCanvas->SetSize( sWidth, sHeight);
	}

}


	int droidRegular;//, droidItalic, droidBold, droidJapanese, dejavu;
#ifndef NO_OPENGL3
sth_stash* initFont(GLPrimitiveRenderer* primRenderer)
{
	GLint err;

		struct sth_stash* stash = 0;
			OpenGL2RenderCallbacks* renderCallbacks = new OpenGL2RenderCallbacks(primRenderer);

	stash = sth_create(512,512,renderCallbacks);//256,256);//,1024);//512,512);
    err = glGetError();
    assert(err==GL_NO_ERROR);

	if (!stash)
	{
		fprintf(stderr, "Could not create stash.\n");
		return 0;
	}


#ifdef LOAD_FONTS_FROM_FILE
		int datasize;
	unsigned char* data;
	float sx,sy,dx,dy,lh;
	GLuint texture;



	const char* fontPaths[]={
	"./",
	"../../bin/",
	"../bin/",
	"bin/"
	};

	int numPaths=sizeof(fontPaths)/sizeof(char*);

	// Load the first truetype font from memory (just because we can).

	FILE* fp = 0;
	const char* fontPath ="./";
	char fullFontFileName[1024];

	for (int i=0;i<numPaths;i++)
	{

		fontPath = fontPaths[i];
		//sprintf(fullFontFileName,"%s%s",fontPath,"OpenSans.ttf");//"DroidSerif-Regular.ttf");
		sprintf(fullFontFileName,"%s%s",fontPath,"DroidSerif-Regular.ttf");//OpenSans.ttf");//"DroidSerif-Regular.ttf");
		fp = fopen(fullFontFileName, "rb");
		if (fp)
			break;
	}

    err = glGetError();
    assert(err==GL_NO_ERROR);

    assert(fp);
    if (fp)
    {
        fseek(fp, 0, SEEK_END);
        datasize = (int)ftell(fp);
        fseek(fp, 0, SEEK_SET);
        data = (unsigned char*)malloc(datasize);
        if (data == NULL)
        {
            assert(0);
            return 0;
        }
        else
            fread(data, 1, datasize, fp);
        fclose(fp);
        fp = 0;
    }

	if (!(droidRegular = sth_add_font_from_memory(stash, data)))
    {
        assert(0);
        return 0;
    }
    err = glGetError();
    assert(err==GL_NO_ERROR);

	// Load the remaining truetype fonts directly.
    sprintf(fullFontFileName,"%s%s",fontPath,"DroidSerif-Italic.ttf");

	if (!(droidItalic = sth_add_font(stash,fullFontFileName)))
	{
        assert(0);
        return 0;
    }
     sprintf(fullFontFileName,"%s%s",fontPath,"DroidSerif-Bold.ttf");

	if (!(droidBold = sth_add_font(stash,fullFontFileName)))
	{
        assert(0);
        return 0;
    }
    err = glGetError();
    assert(err==GL_NO_ERROR);

     sprintf(fullFontFileName,"%s%s",fontPath,"DroidSansJapanese.ttf");
    if (!(droidJapanese = sth_add_font(stash,fullFontFileName)))
	{
        assert(0);
        return 0;
    }
#else
	unsigned char* data = OpenSansData;
	
	if (!(droidRegular = sth_add_font_from_memory(stash, data)))
	{
		printf("error!\n");
	}

#endif

	err = glGetError();
    assert(err==GL_NO_ERROR);

	return stash;
}
#endif
void keyCallback(int key, int value)
{
	printf("key = %d, value = %d\n", key,value);
	//pCanvas->InputKey(key,value==1);


	int gwenKey = -1;

	switch (key)
	{
	case B3G_LEFT_ARROW:
		{
			gwenKey = Gwen::Key::Left;
			break;
		}
	case B3G_RIGHT_ARROW:
		{
			gwenKey = Gwen::Key::Right;
			break;
		}
	case B3G_UP_ARROW:
		{
			gwenKey = Gwen::Key::Up;
			break;
		}
	case B3G_DOWN_ARROW:
		{
			gwenKey = Gwen::Key::Down;
			break;
		}
	case B3G_BACKSPACE:
		{
			gwenKey = Gwen::Key::Backspace;
			break;
		}
	case B3G_DELETE:
		{
			gwenKey = Gwen::Key::Delete;
			break;
		}
	case B3G_HOME:
		{
			gwenKey = Gwen::Key::Home;
			break;
		}
	case B3G_END:
		{
			gwenKey = Gwen::Key::End;
			break;
		}
	case B3G_SHIFT:
		{
			gwenKey = Gwen::Key::Shift;
			break;
		}
	case B3G_CONTROL:
		{
			gwenKey = Gwen::Key::Control;
			break;
		}



	default:
		{

		}
	};

	if (gwenKey>=0)
	{
			pCanvas->InputKey(gwenKey,value==1);
	} else
	{
		if (key<256 && value)
		{
			Gwen::UnicodeChar c = ( Gwen::UnicodeChar ) key;
			pCanvas->InputCharacter(c);
		}
	}
}

extern int avoidUpdate;

int main()
{

	b3gDefaultOpenGLWindow* window = new b3gDefaultOpenGLWindow();
	window->setKeyboardCallback(keyCallback);
	b3gWindowConstructionInfo wci;
#ifndef NO_OPENGL3
	wci.m_openglVersion = 3;
#else
    	wci.m_openglVersion = 2;
#endif
	wci.m_width = sWidth;
	wci.m_height = sHeight;
	//	wci.m_resizeCallback = MyResizeCallback;

	window->createWindow(wci);
	window->setResizeCallback(MyResizeCallback);
	

    int majorGlVersion, minorGlVersion;

    if (!sscanf((const char*)glGetString(GL_VERSION), "%d.%d", &majorGlVersion, &minorGlVersion)==2)
    {
        printf("Exit: Error cannot extract OpenGL version from GL_VERSION string\n");
        exit(0);
    }
	char title[1024];
	if (wci.m_openglVersion>2)
	{
		sprintf(title,"Gwen with OpenGL %d.%d\n",majorGlVersion,minorGlVersion);
	} else
	{
		sprintf(title,"Gwen with OpenGL %d\n",wci.m_openglVersion);
	}
	window->setWindowTitle(title);

	float retinaScale = window->getRetinaScale();

#ifndef NO_OPENGL3
    if (majorGlVersion>=3 && wci.m_openglVersion>=3)
    {
#ifndef B3_USE_GLFW		
#ifndef __APPLE__
#ifndef _WIN32
    //we need glewExperimental on Linux
    glewExperimental = GL_TRUE;
#endif // _WIN32
        glewInit();
#endif
#endif //B3_USE_GLFW
    //we ned to call glGetError twice, because of some Ubuntu/Intel/OpenGL issue

    GLuint err = glGetError();
    err = glGetError();
    assert(err==GL_NO_ERROR);


        primRenderer = new GLPrimitiveRenderer(sWidth,sHeight);

        sth_stash* font = initFont(primRenderer );


        gwenRenderer = new GwenOpenGL3CoreRenderer(primRenderer,font,sWidth,sHeight,retinaScale);

    } else
#endif
    {
        //OpenGL 2.x
        gwenRenderer = new Gwen::Renderer::OpenGL_DebugFont(retinaScale);


        skin.SetRender( gwenRenderer );

       

        glClearColor(1,0,0,1);

    }



	//
	// Create a GWEN OpenGL Renderer
	//
//		Gwen::Renderer::OpenGL_DebugFont * pRenderer = new Gwen::Renderer::OpenGL_DebugFont();

	//
	// Create a GWEN skin
	//


#ifdef USE_TEXTURED_SKIN
	Gwen::Skin::TexturedBase skin;
	skin.SetRender( pRenderer );
	skin.Init("DefaultSkin.png");
#else
	skin.SetRender( gwenRenderer );
#endif


	//
	// Create a Canvas (it's root, on which all other GWEN panels are created)
	//
	pCanvas = new Gwen::Controls::Canvas( &skin );
	pCanvas->SetSize( sWidth, sHeight);
	pCanvas->SetDrawBackground( true );
	pCanvas->SetBackgroundColor( Gwen::Color( 150, 170, 170, 255 ) );

	window->setMouseButtonCallback(MyMouseButtonCallback);
	window->setMouseMoveCallback(MyMouseMoveCallback);


	//
	// Create our unittest control (which is a Window with controls in it)
	//
	UnitTest* pUnit = new UnitTest( pCanvas );
	pUnit->SetPos( 10, 10 );

	//
	// Create a Windows Control helper
	// (Processes Windows MSG's and fires input at GWEN)
	//
	//Gwen::Input::Windows GwenInput;
	//GwenInput.Initialize( pCanvas );

	//
	// Begin the main game loop
	//
//	MSG msg;
	while( !window->requestedExit() )
	{
        if (majorGlVersion<3 || wci.m_openglVersion<3)
        {
            saveOpenGLState(sWidth,sHeight);
        }

		// Skip out if the window is closed
		//if ( !IsWindowVisible( g_pHWND ) )
			//break;

		// If we have a message from windows..
	//	if ( PeekMessage( &msg, NULL, 0, 0, PM_REMOVE ) )
		{

			// .. give it to the input handler to process
		//	GwenInput.ProcessMessage( msg );

			// if it's QUIT then quit..
		//	if ( msg.message == WM_QUIT )
			//	break;

			// Handle the regular window stuff..
		//	TranslateMessage(&msg);
		//	DispatchMessage(&msg);

		}

		window->startRendering();

		// Main OpenGL Render Loop
		{
			glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

				glEnable(GL_BLEND);
				GLint err = glGetError();
				assert(err==GL_NO_ERROR);

				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

				err = glGetError();
				assert(err==GL_NO_ERROR);

				err = glGetError();
				assert(err==GL_NO_ERROR);

				glDisable(GL_DEPTH_TEST);
				err = glGetError();
				assert(err==GL_NO_ERROR);

				//glColor4ub(255,0,0,255);

				err = glGetError();
				assert(err==GL_NO_ERROR);


				err = glGetError();
				assert(err==GL_NO_ERROR);
				glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

			//	saveOpenGLState(width,height);//m_glutScreenWidth,m_glutScreenHeight);

				err = glGetError();
				assert(err==GL_NO_ERROR);


				err = glGetError();
				assert(err==GL_NO_ERROR);

				glDisable(GL_CULL_FACE);

				glDisable(GL_DEPTH_TEST);
				err = glGetError();
				assert(err==GL_NO_ERROR);

				err = glGetError();
				assert(err==GL_NO_ERROR);

				glEnable(GL_BLEND);


				err = glGetError();
				assert(err==GL_NO_ERROR);



			pCanvas->RenderCanvas();

			if (avoidUpdate<=0)
				avoidUpdate++;

	//		SwapBuffers( GetDC( g_pHWND ) );
		}
		window->endRendering();

        if (majorGlVersion<3 || wci.m_openglVersion<3)
        {
            restoreOpenGLState();
        }
	}

	window->closeWindow();
	delete window;


}
