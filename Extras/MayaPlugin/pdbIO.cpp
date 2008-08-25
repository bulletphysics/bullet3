/*
Bullet Continuous Collision Detection and Physics Library Maya Plugin
Copyright (c) 2008 Walt Disney Studios
 
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising
from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:
 
1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this
software in a product, an acknowledgment in the product documentation
would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must
not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 
Written by: Nicola Candussi <nicola@fluidinteractive.com>
*/

#include "pdbIO.h"
#include <string.h>
#include <map>
#include <stdexcept>

#define	PDB_MAGIC 670

//headers used for reading and writing
   
struct pdb_channel_data_t
{
    int                 type;
    unsigned int        datasize;
    unsigned int        blocksize;
    int                 num_blocks;
    //void                **block;
    int                 block;
};

struct pdb_channel_t
{
    //char                    *name;
    int                     name;
    int                     type;
    unsigned int            size;
    unsigned int            active_start;
    unsigned int            active_end;

    char                    hide;
    char                    disconnect;
    //pdb_channel_data_t     *data;
    int                     data;

    //pdb_channel_t          *link;
    int                     link;        
    //pdb_channel_t          *next;
    int                     next;
};

struct pdb_channel_header_t
{
    char                magic;
    unsigned short      swap;
    char                encoding;
    char                type;
};

/*
struct pdb_data_t
{
    int         numAttributes;
    int         numParticles;
    float       time;
    // short      *types;
    int          types;
    // char      **names;
    int          names;
    // void      **data;
    int          data;
};
             */


struct pdb_header_t
{
    int			    magic;
    unsigned short	swap;
    float		    version;
    float		    time;
    unsigned int	data_size;
    unsigned int	num_data;
    char		    padding[32];
    //pdb_channel_t   **data;
    int             data;
};

/*
struct pdb_channel_data_t
{
    int                 type;
    unsigned            datasize;
    unsigned long       blocksize;
    int                 num_blocks;
    void                **block;
};

struct pdb_channel_t
{
    char                    *name;
    int                     type;
    unsigned long           size;
    unsigned long           active_start;
    unsigned long           active_end;

    char                    hide;
    char                    disconnect;
    pdb_channel_data_t     *data;

    pdb_channel_t          *link;
    pdb_channel_t          *next;
};

struct pdb_channel_header_t
{
    char                magic;
    unsigned short      swap;
    char                encoding;
    char                type;
};

struct pdb_data_t
{
    int         numAttributes;
    int         numParticles;
    float       time;
    short      *types;
    char      **names;
    void      **data;
};



struct pdb_header_t
{
    int			    magic;
    unsigned short	swap;
    float		    version;
    float		    time;
    unsigned		data_size;
    unsigned		num_data;
    char		    padding[32];
    pdb_channel_t   **data;
};

*/

void read_string( std::istream &in, std::string &str )
{
    int c;
    while( (c = in.get()) ) {
        if ( in.eof()) {
            str.clear();
            return;
        }
        str.push_back(c);
    }
}

void write_string( std::ostream &out, std::string &str )
{
    for(size_t i = 0; i < str.size(); ++i) {
        out.put(str[i]);
    }
    out.put(0);
}

bool pdb_io_t::read(std::istream &in)
{
    pdb_header_t            header;
    pdb_channel_header_t    channel_header;
    pdb_channel_t           channel;
    pdb_channel_data_t      channel_data;


    // read the header 
    in.read( (char*)&header, sizeof(pdb_header_t) );
    m_time = header.time;
    m_num_particles = header.data_size;

    m_attributes.clear();
    m_attributes.resize( header.num_data );

    for (unsigned int i=0; i<header.num_data; i++) { 
        //
    	in.read( (char*)&channel_header, sizeof(pdb_channel_header_t));

        in.read( (char*)&channel, sizeof(pdb_channel_t));

        read_string(in, m_attributes[i].m_name);
        m_attributes[i].m_type = channel.type;

       // std::cout << m_attributes[i].m_name << std::endl;

        in.read( (char*)&channel_data, sizeof(pdb_channel_data_t));

    	switch( channel.type ) {
        case kVector: 
            throw std::runtime_error("pdb_io_t::read: channel_data.datasize == sizeof(vec3_t)");
            m_attributes[i].m_num_elements = 1;
            m_attributes[i].m_vector_data.resize(m_attributes[i].m_num_elements * header.data_size);
            in.read( (char*)&(m_attributes[i].m_vector_data[0]), channel_data.datasize * header.data_size);  
            break;
        case kReal:
           // throw std::runtime_error("pdb_io_t::read: channel_data.datasize == sizeof(vec3_t)");
            m_attributes[i].m_num_elements = channel_data.datasize / sizeof(float);
            m_attributes[i].m_real_data.resize(m_attributes[i].m_num_elements * header.data_size);
            in.read( (char*)&(m_attributes[i].m_real_data[0]), channel_data.datasize * header.data_size);  
            break;
        case kLong:
           // assert(channel_data.datasize == sizeof(int));    
          //  throw std::runtime_error("pdb_io_t::read: channel_data.datasize == sizeof(vec3_t)");
            m_attributes[i].m_num_elements = 1;
            m_attributes[i].m_long_data.resize(m_attributes[i].m_num_elements * header.data_size);
            in.read( (char*)&(m_attributes[i].m_long_data[0]), channel_data.datasize * header.data_size);  
            break;
    	}
    }
    return true;
}


bool pdb_io_t::write(std::ostream &out)
{
    pdb_header_t            header;
    pdb_channel_header_t    channel_header;
    pdb_channel_t           channel;
    pdb_channel_data_t      channel_data;


    // read the header 
    header.magic = PDB_MAGIC;
    header.swap = 0;
    header.version = 1.0;
    header.time = m_time;
    header.data_size = m_num_particles;
    header.num_data = m_attributes.size();
    memset(header.padding, 0, 32 * sizeof(char) + sizeof(int));

    out.write( (char*)&header, sizeof(pdb_header_t) );

    for (size_t i = 0; i < m_attributes.size(); ++i) { 
        //
        channel_header.magic = 0;
        channel_header.swap = 0;
        channel_header.encoding = 0;
        channel_header.type = m_attributes[i].m_type;
    	out.write( (char*)&channel_header, sizeof(pdb_channel_header_t));

        channel.name = 0;
        channel.type = m_attributes[i].m_type;
        channel.size = 0;
        channel.active_start = 0;
        channel.active_end = 0;
        channel.hide = 0;
        channel.disconnect = 0;
        channel.data = 0;
        channel.link = 0;
        channel.next = 0;
        out.write( (char*)&channel, sizeof(pdb_channel_t));

        write_string(out, m_attributes[i].m_name);

        channel_data.type = m_attributes[i].m_type;
        //size of a single element
        switch(m_attributes[i].m_type) {
        case kVector:
            channel_data.datasize = sizeof(vec3_t);
            break;
        case kReal:
            channel_data.datasize = sizeof(float) * m_attributes[i].m_real_data.size() / m_num_particles;
            break;
        case kLong:
            channel_data.datasize = sizeof(int) * m_attributes[i].m_long_data.size() / m_num_particles;
            break;
        }
        channel_data.blocksize = 0;
        channel_data.num_blocks = 0;
        channel_data.block = 0;
        out.write( (char*)&channel_data, sizeof(pdb_channel_data_t));

    	switch(m_attributes[i].m_type ) {
        case kVector: 
            out.write( (char*)&(m_attributes[i].m_vector_data[0]), channel_data.datasize * header.data_size);  
            break;
        case kReal:
            out.write( (char*)&(m_attributes[i].m_real_data[0]), channel_data.datasize * header.data_size);  
            break;
        case kLong:
            out.write( (char*)&(m_attributes[i].m_long_data[0]), channel_data.datasize * header.data_size);  
            break;
    	}
    }
    return true;
}


