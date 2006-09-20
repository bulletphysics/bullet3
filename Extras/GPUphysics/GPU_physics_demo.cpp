#include "GPU_physics.h"
#include "fboSupport.h"
#include "shaderSupport.h"

enum DebugOptions
{
  DRAW_WITHOUT_SHADERS,
  DRAW_WITHOUT_PHYSICS,
  DRAW_WITHOUT_COLLISIONS,
  DRAW_WITHOUT_FORCES,
  DRAW_ALL
} ;


static float *positionData = NULL ;
static float *rotationData = NULL ;
static bool         noVertexTextureSupport = false ;
static DebugOptions debugOpt = DRAW_ALL ;

void checkVertexTextureSupport ( bool disableVertexTextureSupport )
{
  GLint nVertTextures ;
  GLint nFragTextures ;
  GLint nCombTextures ;

  glGetIntegerv ( GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS, & nVertTextures ) ;
  glGetIntegerv ( GL_MAX_TEXTURE_IMAGE_UNITS, & nFragTextures ) ;
  glGetIntegerv ( GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, & nCombTextures ) ;

  fprintf(stderr, "INFO: This hardware supports at most:\n"
                  "  %2d vert  texture samplers\n"
                  "  %2d frag  texture samplers\n"
                  "  %2d total texture samplers\n",
                  nVertTextures, nFragTextures, nCombTextures ) ;

  noVertexTextureSupport = ( nVertTextures < 2 ) ;

  if ( noVertexTextureSupport && debugOpt != DRAW_WITHOUT_SHADERS )
  {
    fprintf ( stderr, "\n"
                      "********************************************\n"
                      "*                                          *\n"
                      "* WARNING: This graphics card doesn't have *\n"
                      "* vertex shader texture support - a work-  *\n"
                      "* around will be used - but this demo will *\n"
                      "* be much less impressive as a result!     *\n"
                      "*                                          *\n"
                      "********************************************\n\n" ) ;
  }
 
  if ( ! noVertexTextureSupport && disableVertexTextureSupport )
  {
    fprintf ( stderr, "WARNING: Vertex Texture Support has"
                      "been disabled from the command line.\n" ) ;
    noVertexTextureSupport = true ;
  }
}


float frand ( float max )
{
  return (float)(rand() % 32767) * max / 32767.0f ;
}


static GLSL_ShaderPair *velocityGenerator  ;
static GLSL_ShaderPair *positionGenerator  ;
static GLSL_ShaderPair *collisionGenerator ;
static GLSL_ShaderPair *cubeShader         ;

static FrameBufferObject *position    ;
static FrameBufferObject *rotation    ;
static FrameBufferObject *velocity    ;
static FrameBufferObject *rotvelocity ;
static FrameBufferObject *force       ;
static FrameBufferObject *massXX      ;
static FrameBufferObject *old         ;

#define TEX_SIZE         128
#define NUM_CUBES        ( TEX_SIZE * TEX_SIZE )
#define STRIPS_PER_CUBE  2
#define VERTS_PER_STRIP  8
#define NUM_VERTS        ( NUM_CUBES * STRIPS_PER_CUBE * VERTS_PER_STRIP )

static GLuint vbo_vx = 0 ;
static GLuint vbo_tx = 0 ;
static GLuint vbo_co = 0 ;
static float  vertices  [ NUM_VERTS * 3 ] ;
static float  texcoords [ NUM_VERTS * 2 ] ;
static float  colours   [ NUM_VERTS * 4 ] ;
static int    starts    [ NUM_CUBES * STRIPS_PER_CUBE ] ;
static int    lengths   [ NUM_CUBES * STRIPS_PER_CUBE ] ;

static int win_width  = 640 ;
static int win_height = 480 ;


void keybd ( unsigned char, int, int )
{
  exit ( 0 ) ;
}


void reshape ( int wid, int ht )
{
  win_width  = wid ;
  win_height = ht  ;
}


void initGLcontext ( int argc, char **argv,
                     void (*display)(void),
                     bool disableVertexTextureSupport )
{
  glutInit            ( &argc, argv ) ;
  glutInitDisplayMode ( GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE ) ;
  glutInitWindowSize  ( win_width, win_height ) ;
  glutCreateWindow    ( "Shader Math Demo" ) ;
  glutDisplayFunc     ( display  ) ;
  glutKeyboardFunc    ( keybd    ) ;
  glutReshapeFunc     ( reshape  ) ;

  glewInit () ;

  checkVertexTextureSupport ( disableVertexTextureSupport ) ;
}


void initMotionTextures ()
{
  if ( debugOpt == DRAW_WITHOUT_SHADERS ) return ;

  position    = new FrameBufferObject ( TEX_SIZE, TEX_SIZE, 3, FBO_FLOAT ) ;
  rotation    = new FrameBufferObject ( TEX_SIZE, TEX_SIZE, 3, FBO_FLOAT ) ;
  old         = new FrameBufferObject ( TEX_SIZE, TEX_SIZE, 3, FBO_FLOAT ) ;

  if ( debugOpt == DRAW_WITHOUT_PHYSICS )
  {
    velocity    = NULL ;
    rotvelocity = NULL ;
    force       = NULL ;
    massXX      = NULL ;
  }
  else
  {
    velocity    = new FrameBufferObject ( TEX_SIZE, TEX_SIZE, 3, FBO_FLOAT ) ;
    rotvelocity = new FrameBufferObject ( TEX_SIZE, TEX_SIZE, 3, FBO_FLOAT ) ;

    if ( debugOpt == DRAW_WITHOUT_FORCES )
    {
      force  = NULL ;
      massXX = NULL ;
    }
    else
    {
      force  = new FrameBufferObject ( TEX_SIZE, TEX_SIZE, 3, FBO_FLOAT ) ;
      massXX = new FrameBufferObject ( TEX_SIZE, TEX_SIZE, 3, FBO_FLOAT ) ;
    }
  }

  positionData    = new float [ TEX_SIZE * TEX_SIZE * 3 ] ;
  rotationData    = new float [ TEX_SIZE * TEX_SIZE * 3 ] ;

  float *velocityData    ;
  float *rotvelocityData ;
  float *forceData       ;
  float *massXXData      ;

  if ( debugOpt == DRAW_WITHOUT_PHYSICS )
  {
    velocityData    = NULL ;
    rotvelocityData = NULL ;
    forceData       = NULL ;
    massXXData      = NULL ;
  }
  else
  {
    velocityData    = new float [ TEX_SIZE * TEX_SIZE * 3 ] ;
    rotvelocityData = new float [ TEX_SIZE * TEX_SIZE * 3 ] ;

    if ( debugOpt == DRAW_WITHOUT_FORCES )
    {
      forceData = NULL ;
      massXXData= NULL ;
    }
    else
    {
      forceData  = new float [ TEX_SIZE * TEX_SIZE * 3 ] ;
      massXXData = new float [ TEX_SIZE * TEX_SIZE * 3 ] ;
    }
  }

  /* Give the objects some initial position, rotation, mass, force, etc */

  for ( int i = 0 ; i < TEX_SIZE ; i++ )
    for ( int j = 0 ; j < TEX_SIZE ; j++ )
    {
      /*
        Start the cubes on a nice, regular 5m grid, 10m above the ground
        centered around the origin
      */

      positionData    [ (i*TEX_SIZE + j) * 3 + 0 ] = 5.0f * (float) (TEX_SIZE/2 - i) ;
      positionData    [ (i*TEX_SIZE + j) * 3 + 1 ] = 10.0f ;
      positionData    [ (i*TEX_SIZE + j) * 3 + 2 ] = 5.0f * (float) (TEX_SIZE/2 - j) ;

      /* Zero their rotations */
      rotationData    [ (i*TEX_SIZE + j) * 3 + 0 ] = 0.0f ;
      rotationData    [ (i*TEX_SIZE + j) * 3 + 1 ] = 0.0f ;
      rotationData    [ (i*TEX_SIZE + j) * 3 + 2 ] = 0.0f ;

      if ( debugOpt != DRAW_WITHOUT_PHYSICS )
      {
        /* Random (but predominantly upwards) velocities. */
        velocityData    [ (i*TEX_SIZE + j) * 3 + 0 ] = frand (  10.0f ) - 5.0f;
        velocityData    [ (i*TEX_SIZE + j) * 3 + 1 ] = frand ( 100.0f ) ;
        velocityData    [ (i*TEX_SIZE + j) * 3 + 2 ] = frand (  10.0f ) - 5.0f;

        /* Random rotational velocities */
        rotvelocityData [ (i*TEX_SIZE + j) * 3 + 0 ] = frand ( 3.0f ) ;
        rotvelocityData [ (i*TEX_SIZE + j) * 3 + 1 ] = frand ( 3.0f ) ;
        rotvelocityData [ (i*TEX_SIZE + j) * 3 + 2 ] = frand ( 3.0f ) ;

        if ( debugOpt != DRAW_WITHOUT_FORCES )
        {
          /* Zero forces (just gravity) */
          forceData       [ (i*TEX_SIZE + j) * 3 + 0 ] = 0.0f ;
          forceData       [ (i*TEX_SIZE + j) * 3 + 1 ] = 0.0f ;
          forceData       [ (i*TEX_SIZE + j) * 3 + 2 ] = 0.0f ;

          /* One kg in weight each */
          massXXData      [ (i*TEX_SIZE + j) * 3 + 0 ] = 1.0f ;
          massXXData      [ (i*TEX_SIZE + j) * 3 + 1 ] = 0.0f ;  /* Unused */
          massXXData      [ (i*TEX_SIZE + j) * 3 + 2 ] = 0.0f ;  /* Unused */
        }
      }
    }

  /* Initialise the textures */

  position    -> fillTexture ( positionData ) ;
  rotation    -> fillTexture ( rotationData ) ;
  old         -> fillTexture ( positionData ) ;  // Doesn't really need it...

  if ( debugOpt != DRAW_WITHOUT_PHYSICS )
  {
    velocity    -> fillTexture ( velocityData ) ;
    rotvelocity -> fillTexture ( rotvelocityData ) ;

    if ( debugOpt != DRAW_WITHOUT_FORCES )
    {
      force     -> fillTexture ( forceData ) ;
      massXX    -> fillTexture ( massXXData ) ;
    }
  }
}


void initPhysicsShaders ()
{
  if ( debugOpt == DRAW_WITHOUT_SHADERS ||
       debugOpt == DRAW_WITHOUT_PHYSICS )
    return ;

  if ( debugOpt == DRAW_WITHOUT_FORCES )
  {
    velocityGenerator = NULL ;
  }
  else
  {
    /*
      The velocity generator shader calculates:

      velocity = old_velocity + delta_T * ( F / m ) ;
    */

    velocityGenerator = new GLSL_ShaderPair (
      "VelocityGenerator",
      NULL, NULL,
      "uniform vec4      g_dt ;"
      "uniform sampler2D old_velocity ;"
      "uniform sampler2D force ;"
      "uniform sampler2D massXX ;"
      "void main() {"
      "   gl_FragColor = vec4 ("
      "                  texture2D ( old_velocity, gl_TexCoord[0].st ).xyz +"
      "                  g_dt.w * ( g_dt.xyz +"
      "                  texture2D ( force       , gl_TexCoord[0].st ).xyz /"
      "                  texture2D ( massXX      , gl_TexCoord[0].st ).x),"
      "                  1.0 ) ; }",
      "VelocityGenerator Frag Shader" ) ;
    assert ( velocityGenerator  -> compiledOK () ) ;
  }

  /*
    The position generater shader calculates:

    position = old_position + delta_T * velocity ;

    It's also used to update the rotational velocity.
  */

  positionGenerator = new GLSL_ShaderPair (
    "PositionGenerator",
    NULL, NULL,
    "uniform float delta_T ;"
    "uniform sampler2D old_position ;"
    "uniform sampler2D velocity ;"
    "void main() {"
    "   gl_FragColor = vec4 ("
    "                  texture2D ( old_position, gl_TexCoord[0].st ).xyz +"
    "                  texture2D ( velocity    , gl_TexCoord[0].st ).xyz *"
    "                  delta_T,"
    "                  1.0 ) ; }",
    "PositionGenerator Frag Shader" ) ;
  assert ( positionGenerator  -> compiledOK () ) ;

  if ( debugOpt == DRAW_WITHOUT_COLLISIONS )
  {
    collisionGenerator = NULL ;
  }
  else
  {
    collisionGenerator = new GLSL_ShaderPair (
      "CollisionGenerator",
      NULL, NULL,
      "uniform sampler2D position ;"
      "uniform sampler2D old_velocity ;"
      "void main() {"
      "   vec3 pos = texture2D ( position    , gl_TexCoord[0].st ).xyz ;"
      "   vec3 vel = texture2D ( old_velocity, gl_TexCoord[0].st ).xyz ;"
      "   if ( pos [ 1 ] < 0.0 ) vel *= vec3(0.90,-0.90,0.90) ;"
      "   gl_FragColor = vec4 ( vel, 1.0 ) ; }",
      "CollisionGenerator Frag Shader" ) ;
    assert ( collisionGenerator -> compiledOK () ) ;
   }
}


void initCubeVBO ()
{
  float *p = vertices  ;
  float *t = texcoords ;
  float *c = colours   ;
  int    nverts = 0 ;

  for ( int k = 0 ;
            k < (noVertexTextureSupport ? 1 : NUM_CUBES) * STRIPS_PER_CUBE ; k++ )
  {
    starts  [ k ] = k * VERTS_PER_STRIP ;
    lengths [ k ] =     VERTS_PER_STRIP ;
  }

  for ( int i = 0 ; i < (noVertexTextureSupport ? 1 : TEX_SIZE) ; i++ )
    for ( int j = 0 ; j < (noVertexTextureSupport ? 1 : TEX_SIZE) ; j++ )
    {
      int n = i * TEX_SIZE + j ;

      /*
        I use the colour data to set which cube is which in
        the physics textures.
      */

      for ( int k = 0 ; k < STRIPS_PER_CUBE * VERTS_PER_STRIP ; k++ )
      {
        *t++ = ((float)i+0.5f)/(float)TEX_SIZE ;
        *t++ = ((float)j+0.5f)/(float)TEX_SIZE ;

        *c++ = frand ( 1.0f ) ;
        *c++ = frand ( 1.0f ) ;
        *c++ = frand ( 1.0f ) ;
        *c++ = 1.0f ;
      }

      float dx, dy, dz ;

      if ( debugOpt == DRAW_WITHOUT_SHADERS )
      {
        dx = 5.0f * (float) (TEX_SIZE/2 - i) ;
        dy = 10.0f ;
        dz = 5.0f * (float) (TEX_SIZE/2 - j) ;
      }
      else
      {
        dx = 0.0f ;
        dy = 0.0f ;
        dz = 0.0f ;
      }

      *p++ = -1 + dx  ; *p++ = -1 + dy ; *p++ = -1 + dz ;
      *p++ = +1 + dx  ; *p++ = -1 + dy ; *p++ = -1 + dz ;
      *p++ = -1 + dx  ; *p++ = +1 + dy ; *p++ = -1 + dz ;
      *p++ = +1 + dx  ; *p++ = +1 + dy ; *p++ = -1 + dz ;
      *p++ = -1 + dx  ; *p++ = +1 + dy ; *p++ = +1 + dz ;
      *p++ = +1 + dx  ; *p++ = +1 + dy ; *p++ = +1 + dz ;
      *p++ = -1 + dx  ; *p++ = -1 + dy ; *p++ = +1 + dz ;
      *p++ = +1 + dx  ; *p++ = -1 + dy ; *p++ = +1 + dz ;

      *p++ = -1 + dx  ; *p++ = +1 + dy ; *p++ = -1 + dz ;
      *p++ = -1 + dx  ; *p++ = +1 + dy ; *p++ = +1 + dz ;
      *p++ = -1 + dx  ; *p++ = -1 + dy ; *p++ = -1 + dz ;
      *p++ = -1 + dx  ; *p++ = -1 + dy ; *p++ = +1 + dz ;
      *p++ = +1 + dx  ; *p++ = -1 + dy ; *p++ = -1 + dz ;
      *p++ = +1 + dx  ; *p++ = -1 + dy ; *p++ = +1 + dz ;
      *p++ = +1 + dx  ; *p++ = +1 + dy ; *p++ = -1 + dz ;
      *p++ = +1 + dx  ; *p++ = +1 + dy ; *p++ = +1 + dz ;

      nverts += STRIPS_PER_CUBE * VERTS_PER_STRIP ;
    }

  glGenBuffersARB ( 1, & vbo_vx ) ;
  glBindBufferARB ( GL_ARRAY_BUFFER_ARB, vbo_vx ) ;
  glBufferDataARB ( GL_ARRAY_BUFFER_ARB, nverts * 3 * sizeof(float),
                    vertices, GL_STATIC_DRAW_ARB ) ;

  glGenBuffersARB ( 1, & vbo_tx ) ;
  glBindBufferARB ( GL_ARRAY_BUFFER_ARB, vbo_tx ) ;
  glBufferDataARB ( GL_ARRAY_BUFFER_ARB, nverts * 2 * sizeof(float),
                    texcoords, GL_STATIC_DRAW_ARB ) ;

  glGenBuffersARB ( 1, & vbo_co ) ;
  glBindBufferARB ( GL_ARRAY_BUFFER_ARB, vbo_co ) ;
  glBufferDataARB ( GL_ARRAY_BUFFER_ARB, nverts * 4 * sizeof(float),
                    colours, GL_STATIC_DRAW_ARB ) ;

  glBindBufferARB ( GL_ARRAY_BUFFER_ARB, 0 ) ;

  if ( debugOpt == DRAW_WITHOUT_SHADERS )
    cubeShader = NULL ;
  else
  {
	//the filenames have additional path to make it easier for some platforms
    if ( noVertexTextureSupport )
      cubeShader = new GLSL_ShaderPair ( "CubeShader", "cubeShaderNoTexture.vert",
                                                       "cubeShader.frag" ) ;
    else
      cubeShader = new GLSL_ShaderPair ( "CubeShader", "cubeShader.vert",
                                                       "cubeShader.frag" ) ;
    assert ( cubeShader -> compiledOK () ) ;
  }
}


void drawCubesTheHardWay ()
{
  /*
    Without vertex texture support, we have to read the position/rotation
    data back from the hardware every frame and render each cube individually.
  */

  float p0 = positionData [ 0 ] ;
  float p1 = positionData [ 1 ] ;
  float p2 = positionData [ 2 ] ;

  position -> fetchTexture ( positionData ) ;
  rotation -> fetchTexture ( rotationData ) ;

  if ( positionData [ 0 ] == p0 &&
       positionData [ 1 ] == p1 &&
       positionData [ 2 ] == p2 )
  {
    fprintf ( stderr, "WARNING: If nothing seems to be working, you may\n"
                      "have an old version of the nVidia driver.\n"
                      "Version 76.76 is known to be bad.\n" ) ;
  }

  cubeShader -> use () ;  /* Math = Cube shader */

  glPushClientAttrib   ( GL_CLIENT_VERTEX_ARRAY_BIT ) ;

  glDisableClientState  ( GL_TEXTURE_COORD_ARRAY ) ;

  glEnableClientState  ( GL_COLOR_ARRAY ) ;
  glBindBufferARB      ( GL_ARRAY_BUFFER_ARB, vbo_co ) ;
  glColorPointer       ( 4, GL_FLOAT, 0, vbo_co ? NULL : colours ) ;

  glEnableClientState  ( GL_VERTEX_ARRAY ) ;
  glBindBufferARB      ( GL_ARRAY_BUFFER_ARB, vbo_vx ) ;
  glVertexPointer      ( 3, GL_FLOAT, 0, vbo_vx ? NULL : vertices ) ;

  for ( int i = 0 ; i < TEX_SIZE ; i++ )
    for ( int j = 0 ; j < TEX_SIZE ; j++ )
    {
      float *pos = & positionData [ (i*TEX_SIZE + j) * 3 ] ;
      float *rot = & rotationData [ (i*TEX_SIZE + j) * 3 ] ;

      glPushMatrix () ;
      glTranslatef ( pos [ 0 ], pos [ 1 ], pos [ 2 ] ) ;
      glRotatef ( rot [ 0 ] * 180.0f / 3.14159f, 0, 1, 0 ) ;
      glRotatef ( rot [ 1 ] * 180.0f / 3.14159f, 1, 0, 0 ) ;
      glRotatef ( rot [ 2 ] * 180.0f / 3.14159f, 0, 0, 1 ) ;
      glMultiDrawArraysEXT ( GL_TRIANGLE_STRIP, (GLint*)starts, (GLint*)lengths,
                             STRIPS_PER_CUBE ) ;
      glPopMatrix () ;
    }

  glBindBufferARB      ( GL_ARRAY_BUFFER_ARB, 0 ) ;
  glPopClientAttrib () ;
}


void drawCubeVBO ()
{
  /*
    With vertex texture support, we can leave the position/rotation
    data on the hardware and render all of the cubes in one big VBO!
  */

  if ( debugOpt != DRAW_WITHOUT_SHADERS )
  {
    cubeShader -> use () ;  /* Math = Cube shader */
    cubeShader -> applyTexture ( "position", position, 0 ) ;
    cubeShader -> applyTexture ( "rotation", rotation, 1 ) ;
  }

  glPushClientAttrib   ( GL_CLIENT_VERTEX_ARRAY_BIT ) ;

  glEnableClientState  ( GL_TEXTURE_COORD_ARRAY ) ;
  glBindBufferARB      ( GL_ARRAY_BUFFER_ARB, vbo_tx ) ;
  glTexCoordPointer    ( 2, GL_FLOAT, 0, vbo_tx ? NULL : texcoords ) ;

  glEnableClientState  ( GL_COLOR_ARRAY ) ;
  glBindBufferARB      ( GL_ARRAY_BUFFER_ARB, vbo_co ) ;
  glColorPointer       ( 4, GL_FLOAT, 0, vbo_co ? NULL : colours ) ;

  glEnableClientState  ( GL_VERTEX_ARRAY ) ;
  glBindBufferARB      ( GL_ARRAY_BUFFER_ARB, vbo_vx ) ;
  glVertexPointer      ( 3, GL_FLOAT, 0, vbo_vx ? NULL : vertices ) ;

  glMultiDrawArraysEXT ( GL_TRIANGLE_STRIP, (GLint*)starts, (GLint*)lengths,
                         NUM_CUBES * STRIPS_PER_CUBE ) ;
  glBindBufferARB      ( GL_ARRAY_BUFFER_ARB, 0 ) ;
  glPopClientAttrib () ;
}


void drawCubes ()
{
  glMatrixMode      ( GL_PROJECTION ) ;
  glLoadIdentity    () ;
  glFrustum         ( -1.0f, 1.0f,
                      -1.0f / ((float)win_width/(float)win_height),
                       1.0f / ((float)win_width/(float)win_height),
                       1.0f, 1000000.0f) ;

  glMatrixMode      ( GL_MODELVIEW ) ;
  glLoadIdentity    () ;
  glTranslatef      ( 10.0, -100.0, -500.0 ) ;
  glRotatef         ( 20.0, 1.0, 0.0, 0.0 ) ;

  glEnable          ( GL_DEPTH_TEST ) ;
  glEnable          ( GL_CULL_FACE  ) ;
  glCullFace        ( GL_FRONT ) ;

  glClearColor      ( 0.7f, 0.7f, 0.7f, 1.0f ) ;
  glClear           ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ) ;

  if ( noVertexTextureSupport )
    drawCubesTheHardWay () ;
  else
    drawCubeVBO () ;
}


void display ( void )
{
  FrameBufferObject *tmp ;

  if ( debugOpt != DRAW_WITHOUT_SHADERS &&
       debugOpt != DRAW_WITHOUT_PHYSICS )
  {
    /* Do some simple physics calculations in four stages */

    if ( debugOpt != DRAW_WITHOUT_FORCES )
    {
      /* Copy old velocity into old. */
      tmp = old ;
      old = velocity ;
      velocity = tmp ;

      velocityGenerator -> use () ;
      velocityGenerator -> applyTexture ( "old_velocity", old     , 0 ) ;
      velocityGenerator -> applyTexture ( "force"       , force   , 1 ) ;
      velocityGenerator -> applyTexture ( "massXX"      , massXX  , 2 ) ;
      velocityGenerator -> setUniform4f ( "g_dt", 0.0f, -9.8f, 0.0f, 0.016f ) ;
      velocity -> paint ()  ;
    }

    /* Copy old position into old. */
    tmp = old ;
    old = position ;
    position = tmp ;

    positionGenerator -> use () ;
    positionGenerator -> applyTexture ( "old_position", old     , 0 ) ;
    positionGenerator -> applyTexture ( "velocity"    , velocity, 1 ) ;
    positionGenerator -> setUniform1f ( "delta_T", 0.016f ) ;
    position -> paint ()  ;

    if ( debugOpt != DRAW_WITHOUT_COLLISIONS )
    {
      /* Copy old velocity into old. */
      tmp = old ;
      old = velocity ;
      velocity = tmp ;

      collisionGenerator -> use () ;
      collisionGenerator -> applyTexture ( "position"    , position, 0 ) ;
      collisionGenerator -> applyTexture ( "old_velocity", old     , 1 ) ;
      velocity -> paint ()  ;
    }

    /* Copy old rotation into old. */
    tmp = old ;
    old = rotation ;
    rotation = tmp ;

    positionGenerator -> use () ;
    positionGenerator -> applyTexture ( "old_position", old        , 0 ) ;
    positionGenerator -> applyTexture ( "velocity"    , rotvelocity, 1 ) ;
    positionGenerator -> setUniform1f ( "delta_T", 0.016f  ) ;
    rotation -> paint ()  ;

    /* Now render the scene using the results */

    restoreFrameBuffer () ;
  }

  glViewport ( 0, 0, win_width, win_height ) ;

  drawCubes () ;

  /* All done! */

  glutSwapBuffers   () ;
  glutPostRedisplay () ;
}


void help ()
{
  fprintf ( stderr, "GPUphysics: Usage -\n\n" ) ;
  fprintf ( stderr, "   GPUphysics_demo  [-c][-p][-v][-a][-v]\n\n" ) ;
  fprintf ( stderr, "Where:\n" ) ;
  fprintf ( stderr, "  -s  -- Draw with shaders at all\n" ) ;
  fprintf ( stderr, "  -p  -- Draw with shaders but no physics\n" ) ;
  fprintf ( stderr, "  -c  -- Draw with physics but no ground collisions\n" ) ;
  fprintf ( stderr, "  -f  -- Draw with physics but no forces\n" ) ;
  fprintf ( stderr, "  -a  -- Draw with all features enabled [default]\n" ) ;
  fprintf ( stderr, "  -v  -- Disable vertex textures even if "
                                 "they are supported in hardware\n" ) ;
  fprintf ( stderr, "\n" ) ;
}


int main ( int argc, char **argv )
{
  bool disableVertexTextureSupport = false ;
  debugOpt = DRAW_ALL ;

  for ( int i = 1 ; i < argc ; i++ )
  {
    if ( argv [ i ][ 0 ] == '-' || argv [ i ][ 0 ] == '+' )
      for ( int j = 1 ; argv[i][j] != '\0' ; j++ )
        switch ( argv [ i ][ j ] )
        {
          case 's' : debugOpt = DRAW_WITHOUT_SHADERS    ; break ; 
          case 'p' : debugOpt = DRAW_WITHOUT_PHYSICS    ; break ; 
          case 'c' : debugOpt = DRAW_WITHOUT_COLLISIONS ; break ; 
          case 'f' : debugOpt = DRAW_WITHOUT_FORCES     ; break ; 
          case 'a' : debugOpt = DRAW_ALL                ; break ;

          case 'v' : disableVertexTextureSupport = true ; break ;
          default  : help () ; exit ( 0 ) ;
        }
    else
    {
      help () ;
      exit ( 0 ) ;
    }
  }

  initGLcontext ( argc, argv, display, disableVertexTextureSupport ) ;
  initMotionTextures () ;
  initPhysicsShaders () ;
  initCubeVBO        () ;

  glutMainLoop () ;
  return 0 ;
}

