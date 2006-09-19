
class GLSL_Shader ;
class FrameBufferObject ;

class GLSL_ShaderPair
{
  GLSL_Shader  *vertShader ;
  GLSL_Shader  *fragShader ;
  GLhandleARB   handle     ;
  char         *name       ;
  bool          success    ;

  bool link () ;

public:

  GLSL_ShaderPair ( const char *_name,
                    const char *vertFname, const char *fragFname ) ;
  GLSL_ShaderPair ( const char *_name,
                    const char *vertSource, const char *vertName,
                    const char *fragSource, const char *fragName ) ;

  ~GLSL_ShaderPair () ;

  bool compiledOK () { return success ; }

  /* Debug functions */

  void showActiveUniforms ( FILE *fd = NULL ) ;
  void showActiveAttribs  ( FILE *fd = NULL ) ;

  GLint getUniformLocation ( const char *varname ) ;

  void applyTexture ( const char *uniformName,
                      FrameBufferObject *fbo,
                      int slot ) ;
  void setUniform1f ( const char *uniformName, float valueX )
  {
    glUniform1fARB ( getUniformLocation ( uniformName ), valueX ) ;
  }

  void setUniform2f ( const char *uniformName, float valueX,
                                               float valueY )
  {
    glUniform2fARB ( getUniformLocation ( uniformName ), valueX, valueY ) ;
  }

  void setUniform3f ( const char *uniformName, float valueX,
                                               float valueY,
                                               float valueZ )
  {
    glUniform3fARB ( getUniformLocation ( uniformName ), valueX, valueY,
                                                         valueZ ) ;
  }

  void setUniform4f ( const char *uniformName, float valueX,
                                               float valueY,
                                               float valueZ,
                                               float valueW )
  {
    glUniform4fARB ( getUniformLocation ( uniformName ), valueX, valueY,
                                                         valueZ, valueW ) ;
  }

                     
  /* To apply the shaders for rendering */

  void use ()
  {
    assert ( success ) ;
    glUseProgramObjectARB ( handle ) ;
  }

} ;


