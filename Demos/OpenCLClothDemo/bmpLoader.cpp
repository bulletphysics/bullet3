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


#include "bmpLoader.h"

#include <new>
#include <cstring>
#include <cstdio>

namespace amd
{

static const short bitMapID = 19778;

void
BitMap::releaseResources(void)
{
    if (pixels_ != NULL) {
        delete[] pixels_;
    }

    if (colors_ != NULL) {
        delete[] colors_;
    }

    pixels_    = NULL;
    colors_    = NULL;
    isLoaded_  = false;
}

BitMap& BitMap::operator=(const BitMap& rhs)
{
    if (this == &rhs) {
        return *this;
    }

    // Copy header
    id         = rhs.id;
    size       = rhs.size;
    reserved1  = rhs.reserved1;
    reserved2  = rhs.reserved2;
    offset     = rhs.offset;

    // Copy header info
    sizeInfo       = rhs.sizeInfo;
    width          = rhs.width;
    height         = rhs.height;
    planes         = rhs.planes;
    bitsPerPixel   = rhs.bitsPerPixel;
    compression    = rhs.compression;
    imageSize      = rhs.imageSize;
    xPelsPerMeter  = rhs.xPelsPerMeter;
    yPelsPerMeter  = rhs.yPelsPerMeter;
    clrUsed        = rhs.clrUsed;
    clrImportant   = rhs.clrImportant;

    numColors_ = rhs.numColors_;
    isLoaded_  = rhs.isLoaded_;

    pixels_    = NULL;
    colors_    = NULL;
    if (isLoaded_) {
        if (rhs.colors_ != NULL) {
            colors_ = new ColorPalette[numColors_];
            if (colors_ == NULL) {
                isLoaded_ = false;
                return *this;
            }
            memcpy(colors_, rhs.colors_, numColors_ * sizeof(ColorPalette));
         }

        pixels_ = new uchar4[width * height];
        if (pixels_ == NULL) {
            delete[] colors_;
            colors_   = NULL;
            isLoaded_ = false;
            return *this;
        }
        memcpy(pixels_, rhs.pixels_, width * height * sizeof(uchar4));
    }

    return *this;
}

void
BitMap::load(const char * filename)
{
    // Release any existing resources
    releaseResources();

    // Open BMP file
    FILE * fd = fopen(filename, "rb");

    // Opened OK
    if (fd != NULL) {
        // Read header
        fread((BitMapHeader *)this, sizeof(BitMapHeader), 1, fd);

        // Failed to read header
        if (ferror(fd)) {
            fclose(fd);
            return;
        }

        // Confirm that we have a bitmap file
        if (id != bitMapID) {
            fclose(fd);
            return;
        }

        // Read map info header
        fread((BitMapInfoHeader *)this, sizeof(BitMapInfoHeader), 1, fd);

        // Failed to read map info header
        if (ferror(fd)) {
            fclose(fd);
            return;
        }

        // No support for compressed images
        if (compression) {
            fclose(fd);
            return;
        }

        // Support only 8 or 24 bits images
        if (bitsPerPixel < 8) {
            fclose(fd);
            return;
        }

        // Store number of colors
        numColors_ = 1 << bitsPerPixel;

        //load the palate for 8 bits per pixel
        if(bitsPerPixel == 8) {
            colors_ = new ColorPalette[numColors_];
            if (colors_ == NULL) {
                fclose(fd);
                return;
            }
            fread(
                (char *)colors_,
                numColors_ * sizeof(ColorPalette),
                1,
                fd);

            // Failed to read colors
            if (ferror(fd)) {
                fclose(fd);
                return;
            }
        }

        // Allocate buffer to hold all pixels
        unsigned int sizeBuffer = size - offset;
        unsigned char * tmpPixels = new unsigned char[sizeBuffer];

        if (tmpPixels == NULL) {
            delete colors_;
            colors_ = NULL;
            fclose(fd);
            return;
        }

        // Read pixels from file, including any padding
        fread(tmpPixels, sizeBuffer * sizeof(unsigned char), 1, fd);

        // Failed to read pixel data
        if (ferror(fd)) {
            delete colors_;
            colors_ = NULL;
            delete tmpPixels;
            fclose(fd);
            return;
        }

        // Allocate image
        pixels_ = new uchar4[width * height];
        if (pixels_ == NULL) {
            delete colors_;
            colors_ = NULL;
            delete tmpPixels;
            fclose(fd);
            return;
        }
        // Set image, including w component (white)
        memset(pixels_, 0xff, width * height * sizeof(uchar4));

        unsigned int index = 0;
        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
                // Read RGB values
                if (bitsPerPixel == 8) {
                    pixels_[(y * width + x)] = colors_[tmpPixels[index++]];
                }
                else { // 24 bit
                    pixels_[(y * width + x)].z = tmpPixels[index++];
                    pixels_[(y * width + x)].y = tmpPixels[index++];
                    pixels_[(y * width + x)].x = tmpPixels[index++];
                }
            }

            // Handle padding
            for(int x = 0; x < (4 - (3 * width) % 4) % 4; x++) {
                index++;
            }
        }

        // Loaded file so we can close the file.
        fclose(fd);
        delete[] tmpPixels;

        // Loaded file so record this fact
        isLoaded_  = true;
    }
}

int
BitMap::colorIndex(uchar4 color)
{
    for (int i = 0; i < numColors_; i++) {
        if (colors_[i].x == color.x &&
            colors_[i].y == color.y &&
            colors_[i].z == color.z &&
            colors_[i].w == color.w) {
            return i;
        }
    }

    return 0;
}

bool
BitMap::write(const char * filename)
{
    if (!isLoaded_) {
        return false;
    }

    // Open BMP file
    FILE * fd = fopen(filename, "wb");

    // Opened OK
    if (fd != NULL) {
        // Write header
        fwrite((BitMapHeader *)this, sizeof(BitMapHeader), 1, fd);

        // Failed to write header
        if (ferror(fd)) {
            fclose(fd);
            return false;
        }

        // Write map info header
        fwrite((BitMapInfoHeader *)this, sizeof(BitMapInfoHeader), 1, fd);

        // Failed to write map info header
        if (ferror(fd)) {
            fclose(fd);
            return false;
        }

        // Write palate for 8 bits per pixel
        if(bitsPerPixel == 8) {
            fwrite(
                (char *)colors_,
                numColors_ * sizeof(ColorPalette),
                1,
                fd);

            // Failed to write colors
            if (ferror(fd)) {
                fclose(fd);
                return false;
            }
        }

        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
                // Read RGB values
                if (bitsPerPixel == 8) {
                    fputc(
                        colorIndex(
                            pixels_[(y * width + x)]),
                            fd);
                }
                else { // 24 bit
                    fputc(pixels_[(y * width + x)].z, fd);
                    fputc(pixels_[(y * width + x)].y, fd);
                    fputc(pixels_[(y * width + x)].x, fd);

                    if (ferror(fd)) {
                        fclose(fd);
                        return false;
                    }
                }
            }

            // Add padding
            for(int x = 0; x < (4 - (3 * width) % 4) % 4; x++) {
                fputc(0, fd);
            }
        }

        return true;
    }

    return false;
}

} // amd
