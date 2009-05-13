/*
  FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
  Copyright (C) 2009. Rama Hoetzlein, http://www.rchoetzlein.com

  ZLib license
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

class Image;

#ifndef IMAGE_H
	#define IMAGE_H
	
	#include "common_defs.h"
	#include <stdio.h>

	#ifdef _MSC_VER
		#include <windows.h>
	#endif
	
	// Image class code below...	
	#define IRED	0
	#define IGREEN	1
	#define IBLUE	2
	#define IALPHA	3

	class Pixel {
	public:
		Pixel ()	{ r=0;g=0;b=0;a=1;}
		Pixel ( double r_, double g_, double b_ )	{r = r_; g = g_; b = b_; a = 1.0; }
		Pixel ( double r_, double g_, double b_, double a_ )	{r = r_; g = g_; b = b_; a = a_; }
		double		r;
		double		g;
		double		b;
		double		a;
	};

	#if defined(_MSC_VER) && defined(USE_JPEG)
		#pragma comment(lib, "jpegd.lib" )	
	#endif

	/*
	 * generic multi channel 8-bit max image class.  can read and write
	 * BMP, ascii PNM, and JPG file formats, and supports some useful OpenGL 
	 * calls.
	 *
	 * get and set pixel methods use doubles from 0.0 to 1.0.  these 
	 * values are mapped to integer values from 0 to the maximum value
	 * allowed by the number of bits per channel in the image.
	 */
	class Image {
	public:
				Image ();
				~Image ();

				// create empty image with specified characteristics
				Image (int width_, int height_);
				Image (int width_, int height_, int channels_);
				Image (int width_, int height_, int channels_, 
					int bits_);

				// create image and read data from filename
				// use good() or bad() to check success
				Image (const char* filename);

				// copy constructor and assignment operator
				// _deep_ copy!
				Image (const Image& image);
		Image&		operator= (const Image& image);

				// accessors
		int		getWidth ()    { return width;    }
		int		getHeight ()   { return height;   }
		int		getChannels () { return channels; }
		int		getBits ()     { return bits;     }
		unsigned char*	getPixels ()   { return pixels;   }

		void	create ( int width_, int height_, int channels_ );

		void	refresh ();

		void	draw ();
		void	draw ( float x, float y );

		
				// unsafe!  use at your own risk!
		void		setPixels ( unsigned char *newPixels );

				// check if the image is valid
		bool		good ();
		bool		bad ();

				// set all the pixel data
		void		clear ();
		void		clear ( Pixel pixel );

				// retrieve pixel data.  methods with _ at the
				// end of their name return 0.0 if the x and y
				// are out of range.  otherwise, an assertion
				// failure occurs
		double		getPixel  (int x, int y, int channel);
		double		getPixel_ (int x, int y, int channel);
		Pixel		getPixel  (int x, int y);
		Pixel		getPixel_ (int x, int y);
		Pixel&		getPixel  (int x, int y, Pixel& pixel);
		Pixel&		getPixel_ (int x, int y, Pixel& pixel);

				// set pixel data.  if x and y are out of range,
				// an assertion failure occurs
		void		setPixel  (int x, int y, int channel, double value);
		void		setPixel_ (int x, int y, int channel, double value);
		void		setPixel  (int x, int y, Pixel pixel);
		void		setPixel_ (int x, int y, Pixel pixel);
		void		setPixel4 ( int x, int y, Pixel pixel );

		void		setAlpha (int x, int y, double value);

	#ifndef DISABLE_OPENGL
				// OpenGL call wrappers
		void		glReadPixelsWrapper ();
		void		glDrawPixelsWrapper ();
		void		glTexImage2DWrapper ();
		void		glTexImageCubeWrapper ( int i );
		void		glTexSubImage2DWrapper ( int x, int y);
	#endif

				// top-level file read and write calls,
				// determines file type
		int		read (const char* filename);
		int		read (const char* filename, const char* alphaname );
		int		write (const char* filename);

		int		readPaletteBMP ( FILE* fp, RGBQUAD*& palette, int bit_count );

				// BMP specific read and write calls
		int		readBMP (const char* filename);
		int		readBMP (FILE* file, FILE* file_a, bool bBaseImg );
		int		writeBMP (const char* filename);
		int		writeBMP (FILE* file);

				// PNM specific read and write calls
		int		readPNM (const char* filename);
		int		readPNM (FILE* file);
		int		writePNM (const char* filename);
		int		writePNM (FILE* file);

	#ifdef USE_JPEG
				// JPG specific read and write calls
		int		readJPG (const char* filename);
		int		readJPG (FILE* file);
		int		writeJPG (const char* filename);
		int		writeJPG (FILE* file);
	#endif

	  private:

		int		index(int x, int y, int c);

		int		width;
		int		height;
		int		channels;	// number of channels per pixel
		int		bits;		// number of bits per channel
		int		maxValue;	// max that can be stored in bits

		unsigned char*	pixels;		// image data

		bool		owns;		// if image owns pixels

		unsigned int		imgID;

	};


#endif // IMAGE_H
