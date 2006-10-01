
uniform sampler2D position       ;
uniform sampler2D old_collisions ;

void main()
{
  vec2   my_id = gl_TexCoord[0].st ;
  vec2   pr_id = gl_TexCoord[1].st ;

  /* Object colliding with itself */

  if ( length ( my_id - pr_id ) < 0.05) discard ;

  vec2 last_id = texture2D ( old_collisions, my_id ).xy ;

  /* Object collision that will already have been dealt with */

  if ( pr_id.y >= last_id.y ) discard ;
  if ( pr_id.y == last_id.y && pr_id.x >= last_id.y) discard ;

  vec3  my_pos = texture2D ( position, my_id ).xyz ;
  vec3  pr_pos = texture2D ( position, pr_id ).xyz ;

  /* Objects that don't actually collide */

  if ( length ( my_pos - pr_pos ) >= 2.0 ) discard ;

  gl_FragColor = vec4 ( pr_id, 0, 1 ) ;
}

