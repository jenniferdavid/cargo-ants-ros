/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */


#ifndef NPM_GFX_PNG_IMAGE_HPP
#define NPM_GFX_PNG_IMAGE_HPP


#ifdef NPM_HAVE_PNG
#  include <png.h>
#else
#  include <stdint.h>
typedef uint32_t png_uint_32;
typedef int8_t png_byte;
typedef int8_t * png_bytep;
#endif

#include <string>


namespace npm {
  
  using std::string;
  
  /**
   *  \brief PNG-only image.
   * 
   *  This class is a quick hack to enable screenshots of the Simulator.
   * 
   *  \todo Adjust naming conventions to the other classes.
   */

  class PNGImage
  {
  public:
    PNGImage(png_uint_32 width, png_uint_32 height);
    ~PNGImage();

    bool write_png(const string &filename);
    void set_pixel(png_uint_32 x, png_uint_32 y,
		   png_byte r, png_byte g, png_byte b);
    void read_framebuf(unsigned int xoff, unsigned int yoff);

  private:
    static const int BPP = 3;

    png_uint_32 width, height;
    png_bytep *row_pointers;
    png_byte *pixel;

    ////unused?//// png_uint_32 npixels;
  };

}

#endif // NPM_GFX_PNG_IMAGE_HPP
