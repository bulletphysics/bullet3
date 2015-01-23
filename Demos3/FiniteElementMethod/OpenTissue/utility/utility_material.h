#ifndef OPENTISSUE_UTILITY_UTILITY_MATERIAL_H
#define OPENTISSUE_UTILITY_UTILITY_MATERIAL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace utility
  {

    /**
    * A Material.
    * This class encapsulates openGL material parameters.
    */
    class Material
    {
    protected:

      float m_ambient[ 4 ];   ///< The ambient color rgba.
      float m_diffuse[ 4 ];   ///< The diffuse color rgba.
      float m_specular[ 4 ];  ///< The specular color rgba.
      float m_shininess;      ///< The shininess of this material 0..180.

    public:

      Material()
        : m_shininess(128)
      {
        set_default();
      }

      void ambient( float const & red, float const & green, float const & blue, float const & alpha = 1.0f )
      {
        m_ambient[ 0 ] = red;
        m_ambient[ 1 ] = green;
        m_ambient[ 2 ] = blue;
        m_ambient[ 3 ] = alpha;
      }

      void diffuse( float const & red, float const & green, float const & blue, float const & alpha = 1.0f )
      {
        m_diffuse[ 0 ] = red;
        m_diffuse[ 1 ] = green;
        m_diffuse[ 2 ] = blue;
        m_diffuse[ 3 ] = alpha;
      }

      void specular( float const & red, float const & green, float const & blue, float const & alpha = 1.0f )
      {
        m_specular[ 0 ] = red;
        m_specular[ 1 ] = green;
        m_specular[ 2 ] = blue;
        m_specular[ 3 ] = alpha;
      }

      void shininess( float const & value )
      {
        m_shininess = value;
      }

      /*
      void use()
      {
      if ( glIsEnabled( GL_COLOR_MATERIAL ) || !glIsEnabled( GL_LIGHTING ) )
      {
      glColor4f( m_diffuse[ 0 ], m_diffuse[ 1 ], m_diffuse[ 2 ], m_diffuse[ 3 ] );
      }
      else
      {
      glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT,   m_ambient    );
      glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE,   m_diffuse    );
      glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR,  m_specular   );
      glMaterialfv( GL_FRONT_AND_BACK, GL_SHININESS, &m_shininess );
      }
      }
      */

      void set_default()
      {
        m_ambient[ 0 ]  = 0.1f;
        m_ambient[ 1 ]  = 0.1f;
        m_ambient[ 2 ]  = 0.0f;
        m_ambient[ 3 ]  = 1.0f;
        m_diffuse[ 0 ]  = 0.6f;
        m_diffuse[ 1 ]  = 0.6f;
        m_diffuse[ 2 ]  = 0.1f;
        m_diffuse[ 3 ]  = 1.0f;
        m_specular[ 0 ] = 1.0f;
        m_specular[ 1 ] = 1.0f;
        m_specular[ 2 ] = 0.75f;
        m_specular[ 3 ] = 1.0f;
        m_shininess     = 128;
      }

    };

  } // namespace utility

} // namespace OpenTissue

//OPENTISSUE_UTILITY_UTILITY_MATERIAL_H
#endif
