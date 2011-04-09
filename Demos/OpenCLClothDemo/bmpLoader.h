/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2010 Advanced Micro Devices

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BMPLOADER_H_
#define BMPLOADER_H_

#include <stdlib.h>

namespace amd
{

//! @fixme this needs to be moved to common types header?
#pragma pack(1)
typedef struct
{
    unsigned char x;
    unsigned char y;
    unsigned char z;
    unsigned char w;
} uchar4;

typedef uchar4 ColorPalette;

//! \struct Bitmap header info
typedef struct {
    short id;
    int size;
    short reserved1;
    short reserved2;
    int offset;
} BitMapHeader;

//! \struct Bitmap info header
typedef struct {
    int sizeInfo;
    int width;
    int height;
    short planes;
    short bitsPerPixel;
    unsigned compression;
    unsigned imageSize;
    int xPelsPerMeter;
    int yPelsPerMeter;
    int clrUsed;
    int clrImportant;
} BitMapInfoHeader;

//! \class Bitmap used to load a bitmap image from a file.
class BitMap : public BitMapHeader, public BitMapInfoHeader
{
private:
    uchar4 * pixels_;

    int numColors_;

    ColorPalette * colors_;

    bool isLoaded_;

    void releaseResources(void);

    int colorIndex(uchar4 color);
public:

    //! \brief Default constructor
    BitMap()
        : pixels_(NULL),
          numColors_(0),
          colors_(NULL),
          isLoaded_(false)
    {}

    /*!\brief Constructor
     *
     * Tries to load bitmap image from filename provided.
     *
     * \param filename pointer to null terminated string that is the path and
     * filename to the bitmap image to be loaded.
     *
     * In the base of an error, e.g. the bitmap file could not be loaded for
     * some reason, then a following call to isLoaded will return false.
     */
    BitMap(const char * filename)
        : pixels_(NULL),
          numColors_(0),
          colors_(NULL),
          isLoaded_(false)
    {
        load(filename);
    }

    /*! \brief Copy constructor
     *
     * \param rhs is the bitmap to be copied (cloned).
     */
    BitMap(const BitMap& rhs)
    {
        *this = rhs;
    }

    //! \brief Destructor
    ~BitMap()
    {
        releaseResources();
    }

    /*! \brief Assignment
     * \param rhs is the bitmap to be assigned (cloned).
     */
    BitMap& operator=(const BitMap& rhs);

    /*! \brief Load Bitmap image
     *
     * \param filename is a pointer to a null terminated string that is the
     * path and filename name to the the bitmap file to be loaded.
     *
     * In the base of an error, e.g. the bitmap file could not be loaded for
     * some reason, then a following call to isLoaded will return false.
     */
    void
    load(const char * filename);

    /*! \brief Write Bitmap image
     *
     * \param filename is a pointer to a null terminated string that is the
     * path and filename name to the the bitmap file to be written.
     *
     * \return In the case that the bitmap is written true is returned. In
     * the case that a bitmap image is not already loaded or the write fails
     * for some reason false is returned.
     */
    bool
    write(const char * filename);

    /*! \brief Get image width
     *
     * \return If a bitmap image has been successfully loaded, then the width
     * image is returned, otherwise -1;
     */
    int
    getWidth(void) const
    {
        if (isLoaded_) {
            return width;
        }
        else {
            return -1;
        }
    }

    /*! \brief Get image height
     *
     * \return If a bitmap image has been successfully loaded, then the height
     * image is returned, otherwise -1.
     */
    int
    getHeight(void) const
    {
        if (isLoaded_) {
            return height;
        }
        else {
            return -1;
        }
    }

    /*! \brief Get image width
     *
     * \return If a bitmap image has been successfully loaded, then returns
     * a pointer to image's pixels, otherwise NULL.
     */
    const uchar4 *
    getPixels(void) const { return pixels_; }

    /*! \brief Is an image currently loaded
     *
     * \return If a bitmap image has been successfully loaded, then returns
     * true, otherwise if an image could not be loaded or an image has yet
     * to be loaded false is returned.
     */
    bool
    isLoaded(void) const { return isLoaded_; }
};
#pragma pack()
}

#endif // BMPLOADER_H_
