/*
    ParamListGL
    - class derived from ParamList to do simple OpenGL rendering of a parameter list
    sgg 8/2001
*/

#ifndef PARAMGL_H
#define PARAMGL_H

#if defined(__APPLE__) || defined(MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <param.h>

void beginWinCoords();
void endWinCoords();
void glPrint(int x, int y, const char *s, void *font);
void glPrintShadowed(int x, int y, const char *s, void *font, float *color);

class ParamListGL : public ParamList {
public:
  ParamListGL(char *name = "");

  void Render(int x, int y, bool shadow = false);
  bool Mouse(int x, int y, int button=GLUT_LEFT_BUTTON, int state=GLUT_DOWN);
  bool Motion(int x, int y);
  void Special(int key, int x, int y);

  void SetSelectedColor(float r, float g, float b) { text_col_selected[0] = r; text_col_selected[1] = g; text_col_selected[2] = b; }
  void SetUnSelectedColor(float r, float g, float b) { text_col_unselected[0] = r; text_col_unselected[1] = g; text_col_unselected[2] = b; }

  int bar_x;
  int bar_w;
  int bar_h;
  int text_x;
  int separation;
  int value_x;
  int font_h;
  int start_x, start_y;
  int bar_offset;

  float text_col_selected[3];
  float text_col_unselected[3];
  float text_col_shadow[3];
  float bar_col_outer[3];
  float bar_col_inner[3];

  void *font;
};

#endif
