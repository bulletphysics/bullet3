#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_ITERATORS_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_ITERATORS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <iterator>

namespace OpenTissue 
{ 
  namespace grid
  {
    namespace detail
    {

      //! basic iterator for walking through the WHOLE container
      template <class grid_type_, class reference_type, class pointer_type>
      class Iterator
        : public std::iterator< std::random_access_iterator_tag, typename grid_type_::value_type >
      {
      public:

        typedef grid_type_                                            grid_type;
        typedef typename grid_type::math_types                        math_types;
      
      private:

        typedef OpenTissue::grid::detail::Iterator<grid_type, reference_type, pointer_type>     self_type;

      protected:

        typedef typename grid_type::index_vector index_vector;
        grid_type *   m_grid;
        pointer_type m_pos;

      public:


        grid_type       & get_grid()       { return *m_grid; }
        grid_type const & get_grid() const { return *m_grid; }

        pointer_type const & get_pointer() const { return m_pos; }
        pointer_type       & get_pointer()       { return m_pos; }

      public:

        Iterator() 
          : m_grid( 0 )
          , m_pos( 0 )
        {}

        Iterator( grid_type * grid, pointer_type pos ) 
          : m_grid( grid )
          , m_pos( pos )
        {}

        reference_type operator*()
        {
          return * m_pos;
        }

        self_type operator++( int )
        {
          self_type tmp = *this;
          m_pos++;
          return tmp;
        }

        self_type &operator++()
        {
          m_pos++;
          return *this;
        }

        void operator+=( size_t m )
        {
          m_pos += m;
        }

        self_type operator+ ( size_t m )
        {
          self_type tmp = *this;
          tmp.m_pos += m;
          return tmp;
        }

        self_type operator- ( size_t m )
        {
          self_type tmp = *this;
          tmp.m_pos -= m;
          return tmp;
        }

        self_type operator--( int )
        {
          self_type tmp = *this;
          m_pos--;
          return tmp;
        }

        self_type &operator--()
        {
          m_pos--;
          return *this;
        }

        void operator-=( size_t m )                       {   m_pos -= m;                 }
        int operator-  ( self_type const & other ) const  {  return m_pos -other.m_pos;   }
        bool operator< ( self_type const & other ) const  {  return m_pos < other.m_pos;  }
        bool operator<=( self_type const & other ) const  {  return m_pos <= other.m_pos; }
        bool operator> ( self_type const & other ) const  {  return m_pos > other.m_pos;  }
        bool operator>=( self_type const & other ) const  {  return m_pos >= other.m_pos; }

        // TODO: henrikd 2005-06-27 - confirm these!
        index_vector compute_index() const
        {
          int offset = m_pos - m_grid->data();
          index_vector res;
          res(2) = offset / ( m_grid->J() * m_grid->I() );
          offset = offset % ( m_grid->J() * m_grid->I() );
          res(1) = offset / m_grid->I();
          res(0) = offset % m_grid->I();
          return res;
        }

        void operator+=( index_vector const & idx )
        {
          m_pos += idx(0) + m_grid->I() * idx(1) + m_grid->I() * m_grid->J() * idx(2);
        }

        self_type const & operator=( index_vector const & idx )
        {
          m_pos = m_grid->data() + idx(0) + m_grid->I() * idx(1) + m_grid->I() * m_grid->J() * idx(2);
          return *this;
        }

        bool operator!=( self_type const & other )
        {
          return m_pos != other.m_pos;
        }

        bool operator==( self_type const & other )
        {
          return m_pos == other.m_pos;
        }
      };


      //! iterator that keeps track of i,j,k position
      /*! for walking through the WHOLE container */
      template <class grid_type, class reference_type, class pointer_type>
      class IndexIterator
        : public OpenTissue::grid::detail::Iterator<grid_type, reference_type, pointer_type>
      {
      public:

        typedef typename grid_type::math_types  math_types;

      protected:

        typedef typename math_types::vector3_type   vector3_type;
        typedef typename math_types::index_vector3_type   index_vector;

      private:

        typedef OpenTissue::grid::detail::Iterator<grid_type, reference_type, pointer_type>      base_type;
        typedef OpenTissue::grid::detail::IndexIterator<grid_type, reference_type, pointer_type> self_type;

        // TODO: Use index_vector directly when vector is implemented as it should be!
        size_t m_i;
        size_t m_j;
        size_t m_k;

        static size_t const m_out_of_bounds = 0u - 1u;

        void update_indices()
        {
          index_vector v = this->compute_index();
          m_i = v(0);
          m_j = v(1);
          m_k = v(2);
        }

      public:

        IndexIterator()
          : base_type()
          , m_i(0)
          , m_j(0)
          , m_k(0)
        {}

        IndexIterator( base_type const& other ) 
          : base_type( other )
        {
          operator=( other );
        }

        IndexIterator( grid_type * grid, pointer_type pos )
          : base_type( grid, pos )
        {
          // optionally move ijk if m_pos is not at begining of m_grid
          if ( this->m_pos != this->m_grid->data() )
          {
            base_type::operator=( base_type( this->m_grid, this->m_pos ).compute_index() );
          }
        }

      public:

        size_t const& i() const { return m_i; }
        size_t const& j() const { return m_j; }
        size_t const& k() const { return m_k; }

        index_vector get_index() const
        {
          return index_vector( m_i, m_j, m_k );
        }

        vector3_type get_coord() const
        {
          return vector3_type(
            m_i * this->m_grid->dx() + this->m_grid->min_coord( 0 )
            , m_j * this->m_grid->dy() + this->m_grid->min_coord( 1 )
            , m_k * this->m_grid->dz() + this->m_grid->min_coord( 2 )
            );
        }

      public:

        self_type operator++( int )
        {
          self_type tmp = *this;
          ++( *this );
          return tmp;
        }

        self_type &operator++()
        {
          ++this->m_pos;
          ++m_i;
          if ( m_i >= this->m_grid->I() )
          {
            m_i = 0u;
            ++m_j;
            if ( m_j >= this->m_grid->J() )
            {
              m_j = 0u;
              ++m_k;
            }
          }
          return *this;
        }

        self_type operator--( int )
        {
          self_type tmp = *this;
          --( *this );
          return tmp;
        }

        self_type &operator--()
        {
          --this->m_pos;
          --m_i;
          if ( m_i == m_out_of_bounds )
          {
            m_i = this->m_grid->I() - 1;
            --m_j;
            if ( m_j == m_out_of_bounds )
            {
              m_j = this->m_grid->J() - 1;
              --m_k;
            }
          }
          return *this;
        }

        // jump to new location
        self_type const & operator=( index_vector const& idx )
        {
          m_i = idx(0);
          m_j = idx(1);
          m_k = idx(2);
          this->m_pos = &( *this->m_grid ) ( idx );
          return *this;
        }

        void operator+=( size_t m )
        {
          this->m_pos += m;
          update_indices();
        }

        void operator-=( size_t m )
        {
          this->m_pos -= m;
          update_indices();
        }

        self_type operator+ ( size_t m )
        {
          self_type tmp = *this;
          tmp.m_pos += m;
          tmp.update_indices();
          return tmp;
        }

        self_type operator- ( size_t m )
        {
          self_type tmp = *this;
          tmp.m_pos -= m;
          tmp.update_indices();
          return tmp;
        }

        self_type const & operator= ( base_type const & other )
        {
          this->m_grid = const_cast<grid_type*>( &( other.get_grid() ) );
          this->m_pos  = other.get_pointer();
          if ( this->m_pos == this->m_grid->data() )
          {
            m_i = 0u;
            m_j = 0u;
            m_k = 0u;
          }
          else
          {
            update_indices();
          }
          return *this;
        }

      };

    } // namespace detail
  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_ITERATORS_H
#endif
