#ifndef OPENTISSUE_UTILITY_UTILITY_IDENTIFIER_H
#define OPENTISSUE_UTILITY_UTILITY_IDENTIFIER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <string>

namespace OpenTissue
{
  namespace utility
  {

    /**
    * Identifier Class.
    * This is an utility class that is supposed to make it easier to create
    * new objects with unique identifiers. The idea is to simply inherit the
    * new object class from the identifier class.
    *
    * At construction-time the class tries to generate a new unique
    * identifier. Hereafter it is the users responsibility not to mess
    * up the id-name string. However, end-users can always be sure that
    * the index value provided by the identifier class is unique.
    */
    class Identifier
    {
    protected:

      std::string m_ID;      ///<  An Identifier string.
      size_t      m_index;   ///< Unique object index.

    public:

      Identifier()
      {
        generate_new_index();
        m_ID = "ID" + m_index;
      }

      virtual  ~Identifier(){}

    public:

      Identifier(Identifier const & id)
      {
        m_ID    = id.m_ID;
        m_index = id.m_index;
      }

      Identifier & operator=(Identifier const & id) 
      {
        m_ID = id.m_ID;
        m_index = id.m_index;
        return *this;
      }

      bool operator==(Identifier const & id) const {  return (m_index == id.m_index);  }

      bool operator!=(Identifier const & id) const {  return !(*this == id);  }

    public:

      void set_id(std::string const  & id) { m_ID = id; } 

      std::string const & get_id() const { return m_ID; }

      size_t get_index() const {return m_index; }

    protected:

      /**
      * Static Member Not Initialized in Header Trick (smnih)
      *
      * This way we avoid having a 
      *
      *   static size_t m_next_idx
      *
      * member variable which must be initialized in a
      * source file.
      */
      void generate_new_index()
      {
        static size_t next_index = 0;
        m_index = next_index;
        ++next_index;
      }

    };

  } // namespace utility

} // namespace OpenTissue

// OPENTISSUE_UTILITY_UTILITY_IDENTIFIER_H
#endif
