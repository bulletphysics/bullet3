#ifndef OPENTISSUE_UTILITY_UTILITY_CLASS_ID_H
#define OPENTISSUE_UTILITY_UTILITY_CLASS_ID_H
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

    /** @internal ID Generator class.
    * This class encapsulates the generation of unique IDs as used by ClassID.
    * 
    * @warning
    *   This class should never be used directly.
    */
    class BaseIDGenerator
    {

      template < class T >
      friend class ClassID;

    private:

      BaseIDGenerator(); // Disable default constructor

      /** ID generation method
      * 
      * @return
      *   An increasing unique ID.
      */
      static size_t const generate_id()
      {
        static size_t id = 0;
        return id++;
      }

    };


    /** The Class ID interface.
    */
    class ClassIDInterface
    {
    public:

      /** Destructor
      */
      virtual ~ClassIDInterface(){}

      /** Query the class ID.
      *
      * @return
      *   The object's class ID
      */
      virtual size_t const class_id() const = 0;
    };



    /** 
    * CRTP Implementation of the ClassIDInterface.
    *
    * Usage:
    *   To enable class IDs for a custom class, simply inherite this class giving
    *   itself as the template argument.
    *
    * Example:
    *   \code
    *   class SomeClass:
    *     public ClassID<SomeClass>
    *   { ... };
    *   \endcode
    *
    * @tparam T
    *   The deriving class
    */
    template < class T >
    class ClassID
      : virtual public ClassIDInterface
    {
    public:

      /**
      * Query the object ID.
      *      
      * @return
      *   The object ID
      */
      static size_t const id()
      {
        static size_t my_id = BaseIDGenerator::generate_id();
        return my_id;
      }

      /** 
      */
      virtual size_t const class_id() const
      {
        return id();
      }
    };

    /** Helper class for solving ambiguousity between class IDs.
    * This class enables inheritance from another class, which already have a class ID.
    * 
    * Problem:
    * \code
    * class SomeBase:
    *     public ClassID<SomeBase>
    * { ... };
    * class SomeClass:
    *     public SomeBase
    *   , public ClassID<SomeClass>
    * { ... }; // Error: id() and class_id() will be ambiguous
    * \endcode
    *
    * Solution:
    * \code
    * class SomeClass:
    *     public ClassIDCompositor<SomeBase, SomeClass>
    * { ... };
    * \endcode
    *
    * @tparam Base
    *   Some base class, which will also be inherited
    * @tparam Self
    *   The deriving class
    */
    template <  class Base, class Self >
    class ClassIDCompositor
      : public Base
      , public ClassID<Self>
    {
    public:

      /**
      */
      static size_t const id()
      {
        return ClassID<Self>::id();
      }

      /** 
      */
      virtual size_t const class_id() const
      {
        return ClassID<Self>::id();
      }
    };

  } // namespace utility
} // namespace OpenTissue
// OPENTISSUE_UTILITY_UTILITY_CLASS_ID_H
#endif 

