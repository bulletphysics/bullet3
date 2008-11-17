
/*
  Use this for rendering the little cubes when
  there is no vertex shader texture support.
*/

void main()
{
  gl_FrontColor = gl_Color ;
  gl_Position   = gl_ModelViewProjectionMatrix * gl_Vertex ;
}

