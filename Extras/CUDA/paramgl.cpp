/*
    ParamListGL
    - class derived from ParamList to do simple OpenGL rendering of a parameter list
    sgg 8/2001
*/

#include <param.h>
#include <paramgl.h>

void beginWinCoords(void)
{
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glTranslatef(0.0, glutGet(GLUT_WINDOW_HEIGHT) - 1, 0.0);
  glScalef(1.0, -1.0, 1.0);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, glutGet(GLUT_WINDOW_WIDTH), 0, glutGet(GLUT_WINDOW_HEIGHT), -1, 1);

  glMatrixMode(GL_MODELVIEW);
}

void endWinCoords(void)
{
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void glPrint(int x, int y, const char *s, void *font)
{
  int i, len;

  glRasterPos2f(x, y);
  len = (int) strlen(s);
  for (i = 0; i < len; i++) {
    glutBitmapCharacter(font, s[i]);
  }
}

void glPrintShadowed(int x, int y, const char *s, void *font, float *color)
{
  glColor3f(0.0, 0.0, 0.0);
  glPrint(x-1, y-1, s, font);

  glColor3fv((GLfloat *) color);
  glPrint(x, y, s, font);
}


ParamListGL::ParamListGL(char *name) : ParamList(name)
{
  font = (void *) GLUT_BITMAP_9_BY_15;
//    font = (void *) GLUT_BITMAP_8_BY_13;

  bar_x = 250;
  bar_w = 250;
  bar_h = 10;
  bar_offset = 5;
  text_x = 5;
  separation = 15;
  value_x = 200;
  font_h = 15;
  start_x = 0;
  start_y = 0;
  
  text_col_selected[0] = 1.0;
  text_col_selected[1] = 1.0;
  text_col_selected[2] = 1.0;

  text_col_unselected[0] = 0.75;
  text_col_unselected[1] = 0.75;
  text_col_unselected[2] = 0.75;

  bar_col_outer[0] = 0.0;
  bar_col_outer[1] = 0.0;
  bar_col_outer[2] = 0.0;

  bar_col_inner[0] = 0.0;
  bar_col_inner[1] = 0.0;
  bar_col_inner[2] = 0.0;

  text_col_shadow[0] = 0.0;
  text_col_shadow[1] = 0.0;
  text_col_shadow[2] = 0.0;
}

void
ParamListGL::Render(int x, int y, bool shadow)
{
  beginWinCoords();

  start_x = x; start_y = y;

  for(std::vector<ParamBase *>::const_iterator p = m_params.begin(); p != m_params.end(); ++p) {
    if ((*p)->IsList()) {
      ParamListGL *list = (ParamListGL *) (*p);
      list->Render(x+10, y);
      y += separation*list->GetSize();

    } else {
      if (p == m_current)
        glColor3fv(text_col_selected);
      else
        glColor3fv(text_col_unselected);
      
      if (shadow) {
        glPrintShadowed(x + text_x, y + font_h, (*p)->GetName()->c_str(), font, (p == m_current) ? text_col_selected : text_col_unselected);
        glPrintShadowed(x + value_x, y + font_h, (*p)->GetValueString().c_str(), font, (p == m_current) ? text_col_selected : text_col_unselected);
      }
      else {
        glPrint(x + text_x, y + font_h, (*p)->GetName()->c_str(), font);
        glPrint(x + value_x, y + font_h, (*p)->GetValueString().c_str(), font);
      } 

//      glColor3fv((GLfloat *) &bar_col_outer);
      glBegin(GL_LINE_LOOP);
      glVertex2f(x + bar_x, y + bar_offset);
      glVertex2f(x + bar_x + bar_w, y + bar_offset);
      glVertex2f(x + bar_x + bar_w, y + bar_offset + bar_h);
      glVertex2f(x + bar_x, y + bar_offset + bar_h);
      glEnd();

//      glColor3fv((GLfloat *) &bar_col_inner);
      glRectf(x + bar_x, y + bar_offset + bar_h, x + bar_x + (bar_w*(*p)->GetPercentage()), y + bar_offset);

      y += separation;
    }

  }

  endWinCoords();
}


bool
ParamListGL::Mouse(int x, int y, int button, int state)
{
  if ((y < start_y) || (y > (int)(start_y + (separation * m_params.size()) - 1)))
    return false;

  int i = (y - start_y) / separation;

  if ((button==GLUT_LEFT_BUTTON) && (state==GLUT_DOWN)) {
#if defined(__GNUC__) && (__GNUC__ < 3)
    m_current = &m_params[i];
#else
    // MJH: workaround since the version of vector::at used here is non-standard
    for (m_current = m_params.begin(); m_current != m_params.end() && i > 0; m_current++, i--);
    //m_current = (std::vector<ParamBase *>::const_iterator)&m_params.at(i);
#endif

    if ((x > bar_x) && (x < bar_x + bar_w)) {
      Motion(x, y);
    }
  }
  return true;
}

bool
ParamListGL::Motion(int x, int y)
{
  if ((y < start_y) || (y > start_y + (separation * (int)m_params.size()) - 1) )
    return false;

  if (x < bar_x) {
    (*m_current)->SetPercentage(0.0);
    return true;
  }
    
  if (x > bar_x + bar_w) {
    (*m_current)->SetPercentage(1.0);
    return true;
  }

  (*m_current)->SetPercentage((x-bar_x) / (float) bar_w);
  return true;
}

void
ParamListGL::Special(int key, int /*x*/, int /*y*/)
{
  switch(key) {
  case GLUT_KEY_DOWN:
    Increment();
    break;
  case GLUT_KEY_UP:
    Decrement();
    break;
  case GLUT_KEY_RIGHT:
    GetCurrent()->Increment();
    break;
  case GLUT_KEY_LEFT:
    GetCurrent()->Decrement();
    break;
  case GLUT_KEY_HOME:
    GetCurrent()->Reset();
    break;
  case GLUT_KEY_END:
    GetCurrent()->SetPercentage(1.0);
    break;
  }
  glutPostRedisplay();
}
