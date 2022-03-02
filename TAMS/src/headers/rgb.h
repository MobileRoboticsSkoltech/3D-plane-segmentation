/*
 * Software License Agreement (BSD License)
 *
 *  Technical Aspects of Multimodal Systems (TAMS) - http://tams-www.informatik.uni-hamburg.de/
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TAMS, nor the names of its contributors may
 *     be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author : Junhao Xiao
 * Email  : junhao.xiao@ieee.org, xiao@informatik.uni-hamburg.de
 */

#ifndef RGB_COLOR_H_
#define RGB_COLOR_H_
#include <stdint.h>
#include <math.h>
#include <vector>

namespace tams
{
  struct RGB
  {
    public:
      RGB () {}
      RGB (const uint8_t &_r, const uint8_t &_g, const uint8_t &_b)
      {
        r = _r;
        g = _g;
        b = _b;
      }
      ~RGB () { }

    public:
      uint8_t r;
      uint8_t g;
      uint8_t b;
  };

}

#endif
//  inline RGB
//  getHeatColour (doublegray);
//  {
//      uint8_t r, g, b;
//    if (gray >= 0.0 && gray <= 0.125)
//    {
//      r = 0;
//      g = 0;
//      b = 127 + floor (gray * 128 / 0.125);
//    }
//    else if (gray > 0.125 && gray <= 0.375)
//    {
//      r = 0;
//      g = floor ((gray - 0.125) * 255 / 0.25);
//      b = 255;
//    }
//    else if (gray > 0.375 && gray <= 0.625)
//    {
//      r = floor ((gray - 0.375) * 255 / 0.25);
//      g = 255;
//      b = 255 - floor ((gray - 0.375) * 255 / 0.25);
//    }
//    else if (gray > 0.625 && gray <= 0.875)
//    {
//      r = 255;
//      g = 255 - floor ((gray - 0.625) * 255 / 0.25);
//      b = 0;
//    }
//    else if (gray > 0.875 && gray <= 1.0)
//    {
//      r = 255 - floor ((gray - 0.875) * 128 / 0.125);
//      g = 0;
//      b = 0;
//    }
//    RGB colour;
//    colour.r = r; colour.g = g; colour.b = b;
//    return colour;
//  }


//  inline
//  RGB getHeatColour(doublevalue)
//  {
//    doubler,g,b;
//    if (0 <= value && value <= 0.125)
//    {
//      r = 0;
//      g = 0;
//      b = 4*value + .5; // .5 - 1 // b = 1/2
//    }
//    else if (0.125 < value && value <= 0.375)
//    {
//      r = 0;
//      g = 4*value - .5; // 0 - 1 // b = - 1/2
//      b = 0;
//    }
//    else if (0.375 < value && value <= 0.625)
//    {
//      r = 4*value - 1.5; // 0 - 1 // b = - 3/2
//      g = 1;
//      b = -4*value + 2.5; // 1 - 0 // b = 5/2
//    }
//    else if (0.625 < value && value <= 0.875)
//    {
//      r = 1;
//      g = -4*value + 3.5; // 1 - 0 // b = 7/2
//      b = 0;
//    }
//    else if (0.875 < value && value <= 1)
//    {
//      r = -4*value + 4.5; // 1 - .5 // b = 9/2
//      g = 0;
//      b = 0;
//    }
//    else
//    {    // should never happen - value > 1
//      r = .5;
//      g = 0;
//      b = 0;
//    }

//    std::cout << "key: " << value << "; colour: " << r << "," << g << "," << b << "; ";

//    RGB colour;
//    // scale for hex conversion
//    colour.r = static_cast<uint8_t>(r * 256);
//    colour.g = static_cast<uint8_t>(g * 256);
//    colour.b = static_cast<uint8_t>(b * 256);

//    printf("colour: %d,%d,%d\n", static_cast<int>(colour.r), static_cast<int>(colour.g), static_cast<int>(colour.b));


//    return colour;
//  }


//inline
//int getRandomColors(std::vector<RGB> &colors)
//{
//  colors.clear();
//  colors.push_back(RGB(0x99, 0xCC, 0x32));
//  colors.push_back(RGB(0xFF, 0xFF, 0x00));
//  colors.push_back(RGB(0xFF, 0xFF, 0xFF));
//  colors.push_back(RGB(0xD8, 0xD8, 0xBF));
//  colors.push_back(RGB(0xCC, 0x32, 0x99));
//  colors.push_back(RGB(0x4F, 0x2F, 0x4F));
//  colors.push_back(RGB(0xCD, 0xCD, 0xCD));
//  colors.push_back(RGB(0x5C, 0x40, 0x33));
//  colors.push_back(RGB(0xAD, 0xEA, 0xEA));
//  colors.push_back(RGB(0xD8, 0xBF, 0xD8));
//  colors.push_back(RGB(0xDB, 0x93, 0x70));
//  colors.push_back(RGB(0x38, 0xB0, 0xDE));
//  colors.push_back(RGB(0x23, 0x6B, 0x8E));
//  colors.push_back(RGB(0x00, 0xFF, 0x7F));
//  colors.push_back(RGB(0xFF, 0x1C, 0xAE));
//  colors.push_back(RGB(0x00, 0x7F, 0xFF));
//  colors.push_back(RGB(0x32, 0x99, 0xCC));
//  colors.push_back(RGB(0xE6, 0xE8, 0xFA));
//  colors.push_back(RGB(0x8E, 0x6B, 0x23));
//  colors.push_back(RGB(0x6B, 0x42, 0x26));
//  colors.push_back(RGB(0x23, 0x8E, 0x68));
//  colors.push_back(RGB(0x8C, 0x17, 0x17));
//  colors.push_back(RGB(0x6F, 0x42, 0x42));
//  colors.push_back(RGB(0x59, 0x59, 0xAB));
//  colors.push_back(RGB(0xFF, 0x00, 0x00));
//  colors.push_back(RGB(0xD9, 0xD9, 0xF3));
//  colors.push_back(RGB(0xEA, 0xAD, 0xEA));
//  colors.push_back(RGB(0xBC, 0x8F, 0x8F));
//  colors.push_back(RGB(0x8F, 0xBC, 0x8F));
//  colors.push_back(RGB(0xDB, 0x70, 0xDB));
//  colors.push_back(RGB(0xFF, 0x24, 0x00));
//  colors.push_back(RGB(0xFF, 0x7F, 0x00));
//  colors.push_back(RGB(0xCF, 0xB5, 0x3B));
//  colors.push_back(RGB(0xEB, 0xC7, 0x9E));
//  colors.push_back(RGB(0x00, 0x00, 0x9C));
//  colors.push_back(RGB(0xFF, 0x6E, 0xC7));
//  colors.push_back(RGB(0x4D, 0x4D, 0xFF));
//  colors.push_back(RGB(0x23, 0x23, 0x8E));
//  colors.push_back(RGB(0x2F, 0x2F, 0x4F));
//  colors.push_back(RGB(0xA6, 0x80, 0x64));
//  colors.push_back(RGB(0xDB, 0x70, 0x93));
//  colors.push_back(RGB(0x70, 0xDB, 0xDB));
//  colors.push_back(RGB(0x7F, 0xFF, 0x00));
//  colors.push_back(RGB(0x7F, 0x00, 0xFF));
//  colors.push_back(RGB(0x42, 0x6F, 0x42));
//  colors.push_back(RGB(0x93, 0x70, 0xDB));
//  colors.push_back(RGB(0xEA, 0xEA, 0xAE));
//  colors.push_back(RGB(0x6B, 0x8E, 0x23));
//  colors.push_back(RGB(0x32, 0x32, 0xCD));
//  colors.push_back(RGB(0x32, 0xCD, 0x99));
//  colors.push_back(RGB(0x8E, 0x23, 0x6B));
//  colors.push_back(RGB(0xE4, 0x78, 0x33));
//  colors.push_back(RGB(0xFF, 0x00, 0xFF));
//  colors.push_back(RGB(0x32, 0xCD, 0x32));
//  colors.push_back(RGB(0xE9, 0xC2, 0xA6));
//  colors.push_back(RGB(0x8F, 0x8F, 0xBD));
//  colors.push_back(RGB(0xA8, 0xA8, 0xA8));
//  colors.push_back(RGB(0xC0, 0xD9, 0xD9));
//  colors.push_back(RGB(0x9F, 0x9F, 0x5F));
//  colors.push_back(RGB(0x4E, 0x2F, 0x2F));
//  colors.push_back(RGB(0x21, 0x5E, 0x21));
//  colors.push_back(RGB(0x93, 0xDB, 0x70));
//  colors.push_back(RGB(0x52, 0x7F, 0x76));
//  colors.push_back(RGB(0x00, 0xFF, 0x00));
//  colors.push_back(RGB(0xC0, 0xC0, 0xC0));
//  colors.push_back(RGB(0xDB, 0xDB, 0x70));
//  colors.push_back(RGB(0xCD, 0x7F, 0x32));
//  colors.push_back(RGB(0x23, 0x8E, 0x23));
//  colors.push_back(RGB(0x8E, 0x23, 0x23));
//  colors.push_back(RGB(0xD1, 0x92, 0x75));
//  colors.push_back(RGB(0x85, 0x63, 0x63));
//  colors.push_back(RGB(0x54, 0x54, 0x54));
//  colors.push_back(RGB(0x85, 0x5E, 0x42));
//  colors.push_back(RGB(0x70, 0x93, 0xDB));
//  colors.push_back(RGB(0x97, 0x69, 0x4F));
//  colors.push_back(RGB(0x2F, 0x4F, 0x4F));
//  colors.push_back(RGB(0x6B, 0x23, 0x8E));
//  colors.push_back(RGB(0x87, 0x1F, 0x78));
//  colors.push_back(RGB(0x99, 0x32, 0xCD));
//  colors.push_back(RGB(0x4F, 0x4F, 0x2F));
//  colors.push_back(RGB(0x4A, 0x76, 0x6E));
//  colors.push_back(RGB(0x2F, 0x4F, 0x2F));
//  colors.push_back(RGB(0x5C, 0x40, 0x33));
//  colors.push_back(RGB(0x00, 0xFF, 0xFF));
//  colors.push_back(RGB(0x42, 0x42, 0x6F));
//  colors.push_back(RGB(0xFF, 0x7F, 0x00));
//  colors.push_back(RGB(0xB8, 0x73, 0x33));
//  colors.push_back(RGB(0xD9, 0x87, 0x19));
//  colors.push_back(RGB(0x5F, 0x9F, 0x9F));
//  colors.push_back(RGB(0xA6, 0x7D, 0x3D));
//  colors.push_back(RGB(0x8C, 0x78, 0x53));
//  colors.push_back(RGB(0xA6, 0x2A, 0x2A));
//  colors.push_back(RGB(0xD9, 0xD9, 0x19));
//  colors.push_back(RGB(0xB5, 0xA6, 0x42));
//  colors.push_back(RGB(0x9F, 0x5F, 0x9F));
//  colors.push_back(RGB(0x00, 0x00, 0xFF));
//  colors.push_back(RGB(0x5C, 0x33, 0x17));
//  colors.push_back(RGB(0x70, 0xDB, 0x93));
//  return 0;
//}
//}

