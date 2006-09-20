#include "GPU_physics.h"
#include "shaderSupport.h"
#include "fboSupport.h"

#define DEFAULT_VERT_SHADER \
         "void main()" \
         "{" \
         "   gl_TexCoord[0] = gl_MultiTexCoord0 ;" \
         "   gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex ;" \
         "}"

class FrameBufferObject ;


class GLSL_Shader
{
  GLenum      type   ;
  GLhandleARB handle ;
  char       *name   ;
  char       *source ;

public:

  GLSL_Shader ()
  {
    type   = (GLenum) 0 ;
    handle = 0 ;
    name   = NULL ;
    source = NULL ;
  }

  ~GLSL_Shader ()
  {
    delete [] name ;
    delete [] source ;
  }

  bool compileFile   ( const char *fname , GLenum type ) ;
  bool compileString ( const char *source, GLenum type,
                       const char *name  ) ;

  GLenum      getType   () { return type   ; }
  GLhandleARB getHandle () { return handle ; }
} ;



static char *readShaderText ( const char *fname )
{
  
  

	FILE *fd = fopen ( fname, "r" ) ;
	if (!fd)
	{
		//some platforms might require different path, try two additional locations
		char newname[256];
		sprintf(newname,"../../Extras/GPUphysics/%s",fname);
		fd = fopen( newname ,"r");
		if (!fd)
		{
			sprintf("Extras/GPUphysics/%s",fname);
			fd = fopen( newname,"r");
		}
	}

	if (!fd)
	{
		printf("cannot open file %s\n",fname);
		exit(1);
	}

	int size = 0;
	/* File operations denied? ok, just close and return failure */
	if (fseek(fd, 0, SEEK_END) || (size = ftell(fd)) == EOF || fseek(fd, 0, SEEK_SET)) 
	{
		printf("Error: cannot get filesize from %s\n", fname);
		exit (1);
	}

	char *shader = new char [ size + 1 ] ;

  if ( fd == NULL )
  {
    fprintf ( stderr, "Cannot read shader '%s'\n", fname ) ;
    exit ( 1 ) ;
  }

  int count = fread ( shader, 1, size, fd ) ;

  shader [ count ] = '\0' ;

  fclose ( fd ) ;

  return shader ;
}


/*****************************\
*                             *
*   Error handling stuff      *
*                             *
\*****************************/


static void showShaderInfo ( const char *which,
                             GLhandleARB handle, const char *name )
{
  int len = 0 ;

  showGLerror ( "showShaderInfo_0" ) ;
  glGetObjectParameterivARB ( handle, GL_OBJECT_INFO_LOG_LENGTH_ARB, (GLint*) &len ) ;
  showGLerror ( "showShaderInfo_1" ) ;

  if ( len > 0 )
  {
    int trueLen ;
    char *s = new char [ len ] ;

    glGetInfoLogARB ( handle, len, (GLint*)&trueLen, s ) ;

    if ( trueLen > 0 && s [ 0 ] != '\0' )
      fprintf ( stderr, "%s:%s - \n%s\n", which, name, s ) ;

    delete [] s ;
  }
}


/************************************\
*                                    *
*       Single Shader Stuff          *
*                                    *
\************************************/



bool GLSL_Shader::compileFile ( const char *fname, GLenum _type )
{
  return compileString ( readShaderText ( fname ), _type, fname ) ;
}


bool GLSL_Shader::compileString ( const char  *_source,
                             GLenum       _type,
                             const char  *_name )
{
  delete [] name   ;
  delete [] source ;

  type   = _type ;
  name   = strdup ( _name   ) ;
  source = strdup ( _source ) ;

  GLint stat ;

  handle = glCreateShaderObjectARB ( type ) ;

  glShaderSourceARB         ( handle, 1, (const GLcharARB **) & source, NULL);
  glCompileShaderARB        ( handle ) ;
  glGetObjectParameterivARB ( handle, GL_OBJECT_COMPILE_STATUS_ARB, & stat ) ;
  showShaderInfo            ( "Compiling", handle, name ) ;

  if ( ! stat )
  {
    fprintf ( stderr, "Failed to compile shader '%s'.\n", name ) ;
    return false ;
  }

  return true ;
}



/************************************\
*                                    *
*       Shader Pair Stuff            *
*                                    *
\************************************/



GLint GLSL_ShaderPair::getUniformLocation ( const char *uni_name )
{
  assert ( success ) ;

  GLint loc = glGetUniformLocationARB ( handle, uni_name ) ;

  if ( loc == -1 )
    fprintf ( stderr, "No such uniform as '%s' or"
                      " '%s' is unused in shader pair '%s'.\n", uni_name,
                                                                uni_name,
                                                                name ) ;
  else
    showGLerror ( "GLSL_ShaderPair::getUniformLocation" ) ;

  return loc ;
}



void GLSL_ShaderPair::showActiveUniforms ( FILE *fd )
{
  GLint maxlen   = 0 ;
  GLint maxattrs = 0 ;

  if ( fd == NULL ) fd = stderr ;

  glGetObjectParameterivARB ( handle,
                              GL_OBJECT_ACTIVE_UNIFORMS_ARB,
                              &maxattrs ) ;

  if ( maxattrs == 0 )
  {
    fprintf ( fd, "No Active Uniforms.\n" ) ;
    return ;
  }

  glGetObjectParameterivARB ( handle,
                              GL_OBJECT_ACTIVE_UNIFORM_MAX_LENGTH_ARB,
                              &maxlen ) ;

  char *name = new char [ maxlen+1 ] ;

  fprintf ( fd, "Active Uniforms:\n" ) ;

  for ( int i = 0 ; i < maxattrs ; i++ )
  {
    GLsizei len  ;
    GLint   size ;
    GLenum  vartype ;
    char *vartypename ;
    GLint   location ;

    glGetActiveUniformARB ( handle, i,
                            maxlen+1, &len, &size, &vartype, name ) ;

    location = glGetUniformLocationARB ( handle, name ) ;

    switch ( vartype )
    {
      case GL_FLOAT          : vartypename = "float    " ; break ;
      case GL_FLOAT_VEC2_ARB : vartypename = "vec2     " ; break ;
      case GL_FLOAT_VEC3_ARB : vartypename = "vec3     " ; break ;
      case GL_FLOAT_VEC4_ARB : vartypename = "vec4     " ; break ;
      case GL_INT            : vartypename = "int      " ; break ;
      case GL_INT_VEC2_ARB   : vartypename = "intvec2  " ; break ;
      case GL_INT_VEC3_ARB   : vartypename = "intvec3  " ; break ;
      case GL_INT_VEC4_ARB   : vartypename = "intvec4  " ; break ;
      case GL_BOOL           : vartypename = "bool     " ; break ;
      case GL_BOOL_VEC2_ARB  : vartypename = "boolvec2 " ; break ;
      case GL_BOOL_VEC3_ARB  : vartypename = "boolvec3 " ; break ;
      case GL_BOOL_VEC4_ARB  : vartypename = "boolvec4 " ; break ;
      case GL_FLOAT_MAT2_ARB : vartypename = "mat2     " ; break ;
      case GL_FLOAT_MAT3_ARB : vartypename = "mat3     " ; break ;
      case GL_FLOAT_MAT4_ARB : vartypename = "mat4     " ; break ;
      case GL_SAMPLER_1D_ARB : vartypename = "sampler1D" ; break ;
      case GL_SAMPLER_2D_ARB : vartypename = "sampler2D" ; break ;
      case GL_SAMPLER_3D_ARB : vartypename = "sampler3D" ; break ;
      default                : vartypename = "?????????" ; break ;
    }

    if ( size == 1 )
      fprintf ( fd, "%2d) %s %s ; // @%d\n", i,
                              vartypename, name, location ) ;
    else
      fprintf ( fd, "%2d) %s %s [ %d ] ; // @%d\n", i,
                              vartypename, name, size, location ) ;
  }

  fprintf ( fd, "\n" ) ;
  delete name ;
}


void GLSL_ShaderPair::showActiveAttribs ( FILE *fd )
{
  if ( fd == NULL ) fd = stderr ;

  GLint maxlen   = 0 ;
  GLint maxattrs = 0 ;

  glGetObjectParameterivARB ( handle,
                              GL_OBJECT_ACTIVE_ATTRIBUTES_ARB,
                              &maxattrs ) ;

  if ( maxattrs == 0 )
  {
    fprintf ( fd, "No Active Attributes.\n" ) ;
    return ;
  }

  glGetObjectParameterivARB ( handle,
                              GL_OBJECT_ACTIVE_ATTRIBUTE_MAX_LENGTH_ARB,
                              &maxlen ) ;

  char *name = new char [ maxlen+1 ] ;

  fprintf ( fd, "Active Attributes:\n" ) ;

  for ( int i = 0 ; i < maxattrs ; i++ )
  {
    GLsizei len  ;
    GLint   size ;
    GLenum  vartype ;
    char *vartypename ;
    GLint   location ;

    glGetActiveAttribARB ( handle, i, maxlen+1, &len, &size, &vartype, name ) ;

    location = glGetAttribLocationARB ( handle, name ) ;

    switch ( vartype )
    {
      case GL_FLOAT          : vartypename = "float" ; break ;
      case GL_FLOAT_VEC2_ARB : vartypename = "vec2 " ; break ;
      case GL_FLOAT_VEC3_ARB : vartypename = "vec3 " ; break ;
      case GL_FLOAT_VEC4_ARB : vartypename = "vec4 " ; break ;
      case GL_FLOAT_MAT2_ARB : vartypename = "mat2 " ; break ;
      case GL_FLOAT_MAT3_ARB : vartypename = "mat3 " ; break ;
      case GL_FLOAT_MAT4_ARB : vartypename = "mat4 " ; break ;
      default                : vartypename = "???? " ; break ;
    }

    if ( size == 1 )
      fprintf ( fd, "%2d) %s %s ; // @%d\n", i,
                              vartypename, name, location ) ;
    else
      fprintf ( fd, "%2d) %s %s [ %d ] ; // @%d\n", i,
                              vartypename, name, size, location ) ;
  }

  fprintf ( fd, "\n" ) ;
  delete name ;
}



bool GLSL_ShaderPair::link ()
{
  GLint stat ;

  handle = glCreateProgramObjectARB () ;

  glAttachObjectARB         ( handle, vertShader -> getHandle () ) ;
  glAttachObjectARB         ( handle, fragShader -> getHandle () ) ;
  glLinkProgramARB          ( handle ) ;
  glGetObjectParameterivARB ( handle, GL_OBJECT_LINK_STATUS_ARB, &stat ) ;
  showShaderInfo ( "Linking", handle, name ) ;
  glValidateProgramARB      ( handle ) ;
  showShaderInfo ( "Validate", handle, name ) ;

  if ( ! stat )
  {
    fprintf ( stderr, "Failed to link shader.\n" ) ;
    return false ;
  }

  return true ;
}


GLSL_ShaderPair::GLSL_ShaderPair ( const char *_name,
                                   const char *vertFname,
                                   const char *fragFname )
{
  name = strdup ( _name ) ;
  handle = 0 ;

  vertShader = new GLSL_Shader () ;
  fragShader = new GLSL_Shader () ;

  bool res1 = ( vertFname == NULL ) ?
               vertShader -> compileString ( DEFAULT_VERT_SHADER,
                                         GL_VERTEX_SHADER_ARB,
					 "Default Vertex Shader" ) :
               vertShader -> compileFile ( vertFname, GL_VERTEX_SHADER_ARB  ) ;

  bool res2 = fragShader -> compileFile ( fragFname, GL_FRAGMENT_SHADER_ARB ) ;

  success = ( res1 && res2 && link () ) ;
}


GLSL_ShaderPair::GLSL_ShaderPair ( const char *_name,
                                   const char *vertSource,
                                   const char *vertName,
                                   const char *fragSource,
                                   const char *fragName )
{
  name = strdup ( _name ) ;
  handle = 0 ;

  vertShader = new GLSL_Shader () ;
  fragShader = new GLSL_Shader () ;

  bool res1 = ( vertSource == NULL ) ?
               vertShader -> compileString ( DEFAULT_VERT_SHADER,
                                         GL_VERTEX_SHADER_ARB,
					 "Default Vertex Shader" ) :
               vertShader -> compileString ( vertSource,
                                            GL_VERTEX_SHADER_ARB,
                                            vertName );

  bool res2 = fragShader -> compileString ( fragSource,
                                            GL_FRAGMENT_SHADER_ARB,
                                            fragName );

  success = ( res1 && res2 && link () ) ;
}


GLSL_ShaderPair::~GLSL_ShaderPair ()
{
  delete [] name ;
  delete vertShader ;
  delete fragShader ;
}


void GLSL_ShaderPair::applyTexture ( const char *uniformName,
                                     FrameBufferObject *fbo,
                                     int slot )
{
  fbo -> use ( slot ) ;
  glUniform1iARB ( getUniformLocation ( uniformName ), slot ) ;
}


