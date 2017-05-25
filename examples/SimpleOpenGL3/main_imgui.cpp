

//#define USE_OPENGL2
#ifdef USE_OPENGL2
#include "OpenGLWindow/SimpleOpenGL2App.h"
typedef SimpleOpenGL2App SimpleOpenGLApp ;

#else
#include "OpenGLWindow/SimpleOpenGL3App.h"
typedef SimpleOpenGL3App SimpleOpenGLApp ;

#endif //USE_OPENGL2

#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "assert.h"
#include <stdio.h>



char* gVideoFileName = 0;
char* gPngFileName = 0;

static b3WheelCallback sOldWheelCB = 0;
static b3ResizeCallback sOldResizeCB = 0;
static b3MouseMoveCallback sOldMouseMoveCB = 0;
static b3MouseButtonCallback sOldMouseButtonCB = 0;
static b3KeyboardCallback sOldKeyboardCB = 0;
//static b3RenderCallback sOldRenderCB = 0;

float gWidth = 1024;
float gHeight = 768;
SimpleOpenGLApp* app  = 0;
float gMouseX = 0;
float gMouseY = 0;
float g_MouseWheel = 0.0f;
int g_MousePressed[3] = {0};
int g_MousePressed2[3] = {0};

//#define B3_USE_IMGUI
#ifdef B3_USE_IMGUI
#include "OpenGLWindow/OpenGLInclude.h"
#include "ThirdPartyLibs/imgui/imgui.h"
static GLuint       g_FontTexture = 0;



void ImGui_ImplBullet_CreateDeviceObjects()
{
	ImGuiIO& io = ImGui::GetIO();

	unsigned char* pixels;
	int width, height;
	io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);   // Load as RGBA 32-bits (75% of the memory is wasted, but default font is so small) because it is more likely to be compatible with user's existing shaders. If your ImTextureId represent a higher-level concept than just a GL texture id, consider calling GetTexDataAsAlpha8() instead to save on GPU memory.

	// Upload texture to graphics system
	GLint last_texture;
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
	glGenTextures(1, &g_FontTexture);
	glBindTexture(GL_TEXTURE_2D, g_FontTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

	// Store our identifier
	io.Fonts->TexID = (void *)(intptr_t)g_FontTexture;

	// Restore state
	glBindTexture(GL_TEXTURE_2D, last_texture);

}


void ImGui_ImplBullet_RenderDrawLists(ImDrawData* draw_data)
{
   // Avoid rendering when minimized, scale coordinates for retina displays (screen coordinates != framebuffer coordinates)
    ImGuiIO& io = ImGui::GetIO();
    int fb_width = (int)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
    int fb_height = (int)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
    if (fb_width == 0 || fb_height == 0)
        return;
    draw_data->ScaleClipRects(io.DisplayFramebufferScale);

    // We are using the OpenGL fixed pipeline to make the example code simpler to read!
    // Setup render state: alpha-blending enabled, no face culling, no depth testing, scissor enabled, vertex/texcoord/color pointers.
    GLint last_texture; glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
    GLint last_viewport[4]; glGetIntegerv(GL_VIEWPORT, last_viewport);
    GLint last_scissor_box[4]; glGetIntegerv(GL_SCISSOR_BOX, last_scissor_box); 
    glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_SCISSOR_TEST);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnable(GL_TEXTURE_2D);
    //glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context

    // Setup viewport, orthographic projection matrix
    glViewport(0, 0, (GLsizei)fb_width, (GLsizei)fb_height);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0f, io.DisplaySize.x, io.DisplaySize.y, 0.0f, -1.0f, +1.0f);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Render command lists
    #define OFFSETOF(TYPE, ELEMENT) ((size_t)&(((TYPE *)0)->ELEMENT))
    for (int n = 0; n < draw_data->CmdListsCount; n++)
    {
        const ImDrawList* cmd_list = draw_data->CmdLists[n];
        const ImDrawVert* vtx_buffer = cmd_list->VtxBuffer.Data;
        const ImDrawIdx* idx_buffer = cmd_list->IdxBuffer.Data;
        glVertexPointer(2, GL_FLOAT, sizeof(ImDrawVert), (const GLvoid*)((const char*)vtx_buffer + OFFSETOF(ImDrawVert, pos)));
        glTexCoordPointer(2, GL_FLOAT, sizeof(ImDrawVert), (const GLvoid*)((const char*)vtx_buffer + OFFSETOF(ImDrawVert, uv)));
        glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(ImDrawVert), (const GLvoid*)((const char*)vtx_buffer + OFFSETOF(ImDrawVert, col)));

        for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++)
        {
            const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
            if (pcmd->UserCallback)
            {
                pcmd->UserCallback(cmd_list, pcmd);
            }
            else
            {
                glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)pcmd->TextureId);
                glScissor((int)pcmd->ClipRect.x, (int)(fb_height - pcmd->ClipRect.w), (int)(pcmd->ClipRect.z - pcmd->ClipRect.x), (int)(pcmd->ClipRect.w - pcmd->ClipRect.y));
                glDrawElements(GL_TRIANGLES, (GLsizei)pcmd->ElemCount, sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer);
            }
            idx_buffer += pcmd->ElemCount;
        }
    }
    #undef OFFSETOF

    // Restore modified state
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    glBindTexture(GL_TEXTURE_2D, (GLuint)last_texture);
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glViewport(last_viewport[0], last_viewport[1], (GLsizei)last_viewport[2], (GLsizei)last_viewport[3]);
    glScissor(last_scissor_box[0], last_scissor_box[1], (GLsizei)last_scissor_box[2], (GLsizei)last_scissor_box[3]);
}
#endif //B3_USE_IMGUI

void MyWheelCallback(float deltax, float deltay)
{
	g_MouseWheel += deltax+deltay;
	if (sOldWheelCB)
		sOldWheelCB(deltax,deltay);
}
void MyResizeCallback( float width, float height)
{
    gWidth = width;
    gHeight = height;
    
	if (sOldResizeCB)
		sOldResizeCB(width,height);
}
void MyMouseMoveCallback( float x, float y)
{
	printf("Mouse Move: %f, %f\n", x,y);
	gMouseX = x;
	gMouseY = y;

	if (sOldMouseMoveCB)
		sOldMouseMoveCB(x,y);
}
void MyMouseButtonCallback(int button, int state, float x, float y)
{
	gMouseX = x;
	gMouseY = y;
	{
		if (button>=0 && button<3)
		{
			if (state)
			{
				g_MousePressed[button] = state;
			}
			g_MousePressed2[button] = state;

		}
		
	}

	if (sOldMouseButtonCB)
		sOldMouseButtonCB(button,state,x,y);
}


void MyKeyboardCallback(int keycode, int state)
{
	//keycodes are in examples/CommonInterfaces/CommonWindowInterface.h
	//for example B3G_ESCAPE for escape key
	//state == 1 for pressed, state == 0 for released.
	// use app->m_window->isModifiedPressed(...) to check for shift, escape and alt keys
	printf("MyKeyboardCallback received key:%c in state %d\n",keycode,state);
	if (sOldKeyboardCB)
		sOldKeyboardCB(keycode,state);
}


bool    ImGui_ImplGlfw_Init()
{
 
	#if 0
    ImGuiIO& io = ImGui::GetIO();
    io.KeyMap[ImGuiKey_Tab] = GLFW_KEY_TAB;                     // Keyboard mapping. ImGui will use those indices to peek into the io.KeyDown[] array.
    io.KeyMap[ImGuiKey_LeftArrow] = GLFW_KEY_LEFT;
    io.KeyMap[ImGuiKey_RightArrow] = GLFW_KEY_RIGHT;
    io.KeyMap[ImGuiKey_UpArrow] = GLFW_KEY_UP;
    io.KeyMap[ImGuiKey_DownArrow] = GLFW_KEY_DOWN;
    io.KeyMap[ImGuiKey_PageUp] = GLFW_KEY_PAGE_UP;
    io.KeyMap[ImGuiKey_PageDown] = GLFW_KEY_PAGE_DOWN;
    io.KeyMap[ImGuiKey_Home] = GLFW_KEY_HOME;
    io.KeyMap[ImGuiKey_End] = GLFW_KEY_END;
    io.KeyMap[ImGuiKey_Delete] = GLFW_KEY_DELETE;
    io.KeyMap[ImGuiKey_Backspace] = GLFW_KEY_BACKSPACE;
    io.KeyMap[ImGuiKey_Enter] = GLFW_KEY_ENTER;
    io.KeyMap[ImGuiKey_Escape] = GLFW_KEY_ESCAPE;
    io.KeyMap[ImGuiKey_A] = GLFW_KEY_A;
    io.KeyMap[ImGuiKey_C] = GLFW_KEY_C;
    io.KeyMap[ImGuiKey_V] = GLFW_KEY_V;
    io.KeyMap[ImGuiKey_X] = GLFW_KEY_X;
    io.KeyMap[ImGuiKey_Y] = GLFW_KEY_Y;
    io.KeyMap[ImGuiKey_Z] = GLFW_KEY_Z;

    io.RenderDrawListsFn = ImGui_ImplGlfw_RenderDrawLists;      // Alternatively you can set this to NULL and call ImGui::GetDrawData() after ImGui::Render() to get the same ImDrawData pointer.
    io.SetClipboardTextFn = ImGui_ImplGlfw_SetClipboardText;
    io.GetClipboardTextFn = ImGui_ImplGlfw_GetClipboardText;
    io.ClipboardUserData = g_Window;
#ifdef _WIN32
    io.ImeWindowHandle = glfwGetWin32Window(g_Window);
#endif

#endif

    return true;
}

int main(int argc, char* argv[])
{
	{
		b3CommandLineArgs myArgs(argc, argv);


		app = new SimpleOpenGLApp("SimpleOpenGLApp", gWidth, gHeight);

		app->m_renderer->getActiveCamera()->setCameraDistance(13);
		app->m_renderer->getActiveCamera()->setCameraPitch(0);
		app->m_renderer->getActiveCamera()->setCameraTargetPosition(0, 0, 0);

		sOldKeyboardCB = app->m_window->getKeyboardCallback();
		app->m_window->setKeyboardCallback(MyKeyboardCallback);
		sOldMouseMoveCB = app->m_window->getMouseMoveCallback();
		app->m_window->setMouseMoveCallback(MyMouseMoveCallback);
		sOldMouseButtonCB = app->m_window->getMouseButtonCallback();
		app->m_window->setMouseButtonCallback(MyMouseButtonCallback);
		sOldWheelCB = app->m_window->getWheelCallback();
		app->m_window->setWheelCallback(MyWheelCallback);
		sOldResizeCB = app->m_window->getResizeCallback();
		app->m_window->setResizeCallback(MyResizeCallback);


		myArgs.GetCmdLineArgument("mp4_file", gVideoFileName);
		if (gVideoFileName)
			app->dumpFramesToVideo(gVideoFileName);

		myArgs.GetCmdLineArgument("png_file", gPngFileName);
		char fileName[1024];

		int textureWidth = 128;
		int textureHeight = 128;

		unsigned char*	image = new unsigned char[textureWidth*textureHeight * 4];


		int textureHandle = app->m_renderer->registerTexture(image, textureWidth, textureHeight);

		int cubeIndex = app->registerCubeShape(1, 1, 1);

		b3Vector3 pos = b3MakeVector3(0, 0, 0);
		b3Quaternion orn(0, 0, 0, 1);
		b3Vector3 color = b3MakeVector3(1, 0, 0);
		b3Vector3 scaling = b3MakeVector3 (1, 1, 1);
		app->m_renderer->registerGraphicsInstance(cubeIndex, pos, orn, color, scaling);
		app->m_renderer->writeTransforms();

		do
		{
			static int frameCount = 0;
			frameCount++;
			if (gPngFileName)
			{
				printf("gPngFileName=%s\n", gPngFileName);

				sprintf(fileName, "%s%d.png", gPngFileName, frameCount++);
				app->dumpNextFrameToPng(fileName);
			}

			
			


			//update the texels of the texture using a simple pattern, animated using frame index
			for (int y = 0; y < textureHeight; ++y)
			{
				const int	t = (y + frameCount) >> 4;
				unsigned char*	pi = image + y*textureWidth * 3;
				for (int x = 0; x < textureWidth; ++x)
				{
					const int		s = x >> 4;
					const unsigned char	b = 180;
					unsigned char			c = b + ((s + (t & 1)) & 1)*(255 - b);
					pi[0] = pi[1] = pi[2] = pi[3] = c; pi += 3;
				}
			}

			app->m_renderer->activateTexture(textureHandle);
			app->m_renderer->updateTexture(textureHandle, image);

			//float color[4] = { 255, 1, 1, 1 };
			//app->m_primRenderer->drawTexturedRect(100, 200, gWidth / 2 - 50, gHeight / 2 - 50, color, 0, 0, 1, 1, true);


			app->m_renderer->init();
			app->m_renderer->updateCamera(1);

			app->m_renderer->renderScene();
			app->drawGrid();
			char bla[1024];
			sprintf(bla, "Simple test frame %d", frameCount);

			//app->drawText(bla, 10, 10);

#ifdef B3_USE_IMGUI
			{
				bool show_test_window = true;
				bool show_another_window = false;
				ImVec4 clear_color = ImColor(114, 144, 154);

				 // Start the frame
				ImGuiIO& io = ImGui::GetIO();
				if (!g_FontTexture)
			        ImGui_ImplBullet_CreateDeviceObjects();

				io.DisplaySize = ImVec2((float)gWidth, (float)gHeight);
				io.DisplayFramebufferScale = ImVec2(gWidth > 0 ? ((float)1.) : 0, gHeight > 0 ? ((float)1.) : 0);
				io.DeltaTime = (float)(1.0f/60.0f);
				io.MousePos = ImVec2((float)gMouseX, (float)gMouseY);   
				io.RenderDrawListsFn = ImGui_ImplBullet_RenderDrawLists;


				for (int i=0;i<3;i++)
				{
					io.MouseDown[i] = g_MousePressed[i]|g_MousePressed2[i];
					g_MousePressed[i] = false;
				}

				io.MouseWheel = g_MouseWheel;

				ImGui::NewFrame();

				ImGui::ShowTestWindow();
				ImGui::ShowMetricsWindow();
				#if 0
				static float f = 0.0f;
				ImGui::Text("Hello, world!");
				ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
				ImGui::ColorEdit3("clear color", (float*)&clear_color);
				if (ImGui::Button("Test Window")) show_test_window ^= 1;
				if (ImGui::Button("Another Window")) show_another_window ^= 1;
				ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
				#endif
				ImGui::Render();
			}
#endif //B3_USE_IMGUI
			app->swapBuffer();
		} while (!app->m_window->requestedExit());



		delete app;

		delete[] image;
	}
	return 0;
}
