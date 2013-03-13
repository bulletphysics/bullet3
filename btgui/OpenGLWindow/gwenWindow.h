
#ifndef MY_GWEN_WINDOW_H
#define MY_GWEN_WINDOW_H


#include "Gwen/Gwen.h"
#include "Gwen/Controls/Button.h"
#include "Gwen/Skins/Simple.h"
#include "Gwen/Renderers/OpenGL_DebugFont.h"

struct sth_stash;

extern class GwenOpenGL3CoreRenderer* pRenderer;
//extern Gwen::Renderer::OpenGL_DebugFont * pRenderer;
extern Gwen::Skin::Simple skin;
extern Gwen::Controls::Canvas* pCanvas;

class GLPrimitiveRenderer;

void	setupGUI(int width, int height, sth_stash* font, float retinaScale,GLPrimitiveRenderer* primRender);
void	processProfileData(class CProfileIterator*  iterator, bool idle);
void	resizeGUI(int width, int height);

#endif //MY_GWEN_WINDOW_H
