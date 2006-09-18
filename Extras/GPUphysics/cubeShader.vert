
/* Use this for rendering the little cubes. */

/* Translation & rotation passed in texture maps */

uniform sampler2D position ;
uniform sampler2D rotation ;

/* gl_Color is just the vertex colour */
/* gl_MultiTexCoord0 selects where within the texture we
   find this cubes location */

void main()
{
  vec4 pos =  vec4 ( texture2D ( position, gl_MultiTexCoord0.st ).xyz, 1.0 ) ;
  vec3 rot =  texture2D ( rotation, gl_MultiTexCoord0.st ).xyz ;

  /* Build a rotation matrix */

  float sh = sin ( rot.x ) ;  float ch = cos ( rot.x ) ;
  float sp = sin ( rot.y ) ;  float cp = cos ( rot.y ) ;
  float sr = sin ( rot.z ) ;  float cr = cos ( rot.z ) ;

  mat4 mat = mat4 ( ch * cr - sh * sr * sp,
                   -sh * cp,               
                    sr * ch + sh * cr * sp,
                    0,                     
                    cr * sh + sr * sp * ch,
                    ch * cp,               
                    sr * sh - cr * sp * ch,
                    0,                     
                   -sr * cp,               
                    sp,                    
                    cr * cp,               
                    0,                     
                    0.0, 0.0, 0.0, 1.0 ) ;

  gl_FrontColor = gl_Color ;
  pos.xyz += (mat * gl_Vertex).xyz ;
  gl_Position = gl_ModelViewProjectionMatrix * pos ;
}

