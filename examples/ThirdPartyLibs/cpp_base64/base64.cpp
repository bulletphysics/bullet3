/*
   base64.cpp and base64.h

   base64 encoding and decoding with C++.
   More information at
     https://renenyffenegger.ch/notes/development/Base64/Encoding-and-decoding-base-64-with-cpp

   Version: 2.rc.00 (release candidate)

   Copyright (C) 2004-2017, 2020 René Nyffenegger

   This source code is provided 'as-is', without any express or implied
   warranty. In no event will the author be held liable for any damages
   arising from the use of this software.

   Permission is granted to anyone to use this software for any purpose,
   including commercial applications, and to alter it and redistribute it
   freely, subject to the following restrictions:

   1. The origin of this source code must not be misrepresented; you must not
      claim that you wrote the original source code. If you use this source code
      in a product, an acknowledgment in the product documentation would be
      appreciated but is not required.

   2. Altered source versions must be plainly marked as such, and must not be
      misrepresented as being the original source code.

   3. This notice may not be removed or altered from any source distribution.

   René Nyffenegger rene.nyffenegger@adp-gmbh.ch

*/

#include "base64.h"

static std::string base64_chars =
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789"
             "??"; // These two question marks will be replaced based on the value of url in base64_encode


static std::size_t pos_of_char(const unsigned char chr) {
 //
 // Return the position of chr within base64_chars.
 //

    if      (chr >= 'A' && chr <= 'Z') return chr - 'A';
    else if (chr >= 'a' && chr <= 'z') return chr - 'a' + ('Z' - 'A')               + 1;
    else if (chr >= '0' && chr <= '9') return chr - '0' + ('Z' - 'A') + ('z' - 'a') + 2;
    else if (chr == '+' || chr == '-') return 62; // Be liberal with input and accept both url ('-') and non-url ('+') base 64 characters (
    else if (chr == '/' || chr == '_') return 63; // Ditto for '/' and '_'

    //throw "If input is correct, this line should never be reached.";
    return 0;
}

static std::string insert_linebreaks(std::string str, size_t distance) {
 //
 // Provided by https://github.com/JomaCorpFX, adapted by me.
 //
    if (!str.length()) {
        return "";
    }

    size_t pos = distance;

    while (pos < str.size()) {
        str.insert(pos, "\n");
        pos += distance + 1;
    }

    return str;
}

std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len, bool url) {
 //
 // Replace question marks in base64_chars:
 //
    if (url) {
        base64_chars[62] = '-';
        base64_chars[63] = '_';
    }
    else {
        base64_chars[62] = '+';
        base64_chars[63] = '/';
    }

    unsigned int len_encoded = (in_len +2) / 3 * 4;

    unsigned char trailing_char = url ? '.' : '=';

    std::string ret;
    ret.reserve(len_encoded);

    unsigned int pos = 0;

    while (pos < in_len) {
        ret.push_back(base64_chars[(bytes_to_encode[pos + 0] & 0xfc) >> 2]);

        if (pos+1 < in_len) {
           ret.push_back(base64_chars[((bytes_to_encode[pos + 0] & 0x03) << 4) + ((bytes_to_encode[pos + 1] & 0xf0) >> 4)]);

           if (pos+2 < in_len) {
              ret.push_back(base64_chars[((bytes_to_encode[pos + 1] & 0x0f) << 2) + ((bytes_to_encode[pos + 2] & 0xc0) >> 6)]);
              ret.push_back(base64_chars[  bytes_to_encode[pos + 2] & 0x3f]);
           }
           else {
              ret.push_back(base64_chars[(bytes_to_encode[pos + 1] & 0x0f) << 2]);
              ret.push_back(trailing_char);
           }
        }
        else {

            ret.push_back(base64_chars[(bytes_to_encode[pos + 0] & 0x03) << 4]);
            ret.push_back(trailing_char);
            ret.push_back(trailing_char);
        }

        pos += 3;
    }


    return ret;
}


std::string base64_decode(std::string const& encoded_string, bool remove_linebreaks) {

    if (remove_linebreaks) {

       if (! encoded_string.length() ) {
           return "";
       }

       std::string copy(encoded_string);

       size_t pos=0;
       while ((pos = copy.find('\n', pos)) != std::string::npos) {
           copy.erase(pos, 1);
       }

       return base64_decode(copy, false);

    }

    int length_of_string = encoded_string.length();
    if (!length_of_string) return std::string("");

    size_t in_len = length_of_string;
    size_t pos = 0;

 //
 // The approximate length (bytes) of the decoded string might be one ore
 // two bytes smaller, depending on the amount of trailing equal signs
 // in the encoded string. This approximation is needed to reserve
 // enough space in the string to be returned.
 //
    size_t approx_length_of_decoded_string = length_of_string / 4 * 3;
    std::string ret;
    ret.reserve(approx_length_of_decoded_string);

    while (pos < in_len) {

       unsigned int pos_of_char_1 = pos_of_char(encoded_string[pos+1] );

       ret.push_back( ( (pos_of_char(encoded_string[pos+0]) ) << 2 ) + ( (pos_of_char_1 & 0x30 ) >> 4));

       if (encoded_string[pos+2] != '=' && encoded_string[pos+2] != '.') { // accept URL-safe base 64 strings, too, so check for '.' also.

          unsigned int pos_of_char_2 = pos_of_char(encoded_string[pos+2] );
          ret.push_back( (( pos_of_char_1 & 0x0f) << 4) + (( pos_of_char_2 & 0x3c) >> 2));

          if (encoded_string[pos+3] != '=') {
             ret.push_back( ( (pos_of_char_2 & 0x03 ) << 6 ) + pos_of_char(encoded_string[pos+3])   );
          }
       }

       pos += 4;
    }

    return ret;
}

std::string base64_encode(std::string const& s, bool url) {
   return base64_encode(reinterpret_cast<const unsigned char*>(s.c_str()), s.length(), url);
}

std::string base64_encode_pem (std::string const& s) {
   return insert_linebreaks(base64_encode(s, false), 64);
}

std::string base64_encode_mime(std::string const& s) {
   return insert_linebreaks(base64_encode(s, false), 76);
}
