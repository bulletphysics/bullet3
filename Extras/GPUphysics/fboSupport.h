
GLuint makeTextureTarget ( GLuint textureHandle ) ;
void renderTo2DTexture ( GLuint fboHandle ) ;
void renderToFrameBuffer () ;

// #define NEED_STENCIL_BUFFER 1
// #define NEED_DEPTH_BUFFER   1


enum fboDataType
{
#ifndef GL_BYTE
 FBO_BYTE           = 0x1400,
 FBO_UNSIGNED_BYTE  = 0x1401,
 FBO_SHORT          = 0x1402,
 FBO_UNSIGNED_SHORT = 0x1403,
 FBO_INT            = 0x1404,
 FBO_UNSIGNED_INT   = 0x1405,
 FBO_FLOAT          = 0x1406,
 FBO_DOUBLE         = 0x140A,
#else
 FBO_BYTE           = GL_BYTE,
 FBO_UNSIGNED_BYTE  = GL_UNSIGNED_BYTE,
 FBO_SHORT          = GL_SHORT,
 FBO_UNSIGNED_SHORT = GL_UNSIGNED_SHORT,
 FBO_INT            = GL_INT,
 FBO_UNSIGNED_INT   = GL_UNSIGNED_INT,
 FBO_FLOAT          = GL_FLOAT,
 FBO_DOUBLE         = GL_DOUBLE,
#endif
#ifndef GL_HALF_FLOAT_NV
 FBO_HALF            = 0x140B
#else
 FBO_HALF            = GL_HALF_FLOAT_NV
#endif
} ;

class FrameBufferObject
{
  int width  ;
  int height ;
  int ncomps ;
  fboDataType type ;
  GLenum  format ;
  GLenum iformat ;

  GLuint textureHandle ;
  GLuint fboHandle ;

  void fillTexture  ( void *data ) ;
  void fetchTexture ( void *data ) ;

public:
  FrameBufferObject ( int _width,          /* Must be a power of two! */
                      int _height,         /* Must be a power of two! */
                      int _numComponents,  /* 1, 2, 3 or 4 only! */
                      fboDataType _type ) ;

  void makeDestination ()
  {
#ifndef NEED_DEPTH_BUFFER
    glDepthMask   ( 0 ) ;
#endif
#ifndef NEED_STENCIL_BUFFER
    glStencilMask ( 0 ) ;
#endif
    glBindFramebufferEXT ( GL_FRAMEBUFFER_EXT, fboHandle ) ;
  }

  void use ( int texture_unit )
  {
    glActiveTexture ( GL_TEXTURE0 + texture_unit ) ;
    glBindTexture ( GL_TEXTURE_2D, textureHandle ) ;
  }

  void prepare ( bool clear = false ) ;
  void fill    () ;

  void paint   ( bool clear = false ) { prepare ( clear ) ; fill () ; }

  void fillTexture ( float          *data ) ;
  void fillTexture ( unsigned short *data ) ;
  void fillTexture ( unsigned char  *data ) ;

  void fetchTexture ( float          *data ) ;
  void fetchTexture ( unsigned short *data ) ;
  void fetchTexture ( unsigned char  *data ) ;
} ;


inline void restoreFrameBuffer ()
{
  glBindFramebufferEXT ( GL_FRAMEBUFFER_EXT, 0 ) ;
#ifndef NEED_DEPTH_BUFFER
  glDepthMask ( 1 ) ;
#endif
#ifndef NEED_STENCIL_BUFFER
  glStencilMask ( 1 ) ;
#endif
}

