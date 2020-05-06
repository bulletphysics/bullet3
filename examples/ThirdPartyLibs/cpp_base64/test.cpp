#include "base64.h"
#include <iostream>

int main() {

  bool all_tests_passed = true;

  const std::string orig = 
    "René Nyffenegger\n"
    "http://www.renenyffenegger.ch\n"
    "passion for data\n";

    std::string encoded = base64_encode(reinterpret_cast<const unsigned char*>(orig.c_str()), orig.length());
    std::string decoded = base64_decode(encoded);

    if (encoded != "UmVuw6kgTnlmZmVuZWdnZXIKaHR0cDovL3d3dy5yZW5lbnlmZmVuZWdnZXIuY2gKcGFzc2lvbiBmb3IgZGF0YQo=") {
      std::cout << "Encoding is wrong" << std::endl;
      all_tests_passed = false;
    }

    if (decoded != orig) {
      std::cout << "decoded != orig" << std::endl;
      all_tests_passed = false;
    }

 // Test all possibilites of fill bytes (none, one =, two ==)
 // References calculated with: https://www.base64encode.org/

    std::string rest0_original = "abc";
    std::string rest0_reference = "YWJj";

    std::string rest0_encoded = base64_encode(reinterpret_cast<const unsigned char*>(rest0_original.c_str()),
      rest0_original.length());
    std::string rest0_decoded = base64_decode(rest0_encoded);

    if (rest0_decoded != rest0_original) {
      std::cout << "rest0_decoded != rest0_original" << std::endl;
      all_tests_passed = false;
    }
    if (rest0_reference != rest0_encoded) {
      std::cout << "rest0_reference != rest0_encoded" << std::endl;
      all_tests_passed = false;
    }

 // std::cout << "encoded:   " << rest0_encoded << std::endl;
 // std::cout << "reference: " << rest0_reference << std::endl;
 // std::cout << "decoded:   " << rest0_decoded << std::endl << std::endl;

    std::string rest1_original = "abcd";
    std::string rest1_reference = "YWJjZA==";

    std::string rest1_encoded = base64_encode(reinterpret_cast<const unsigned char*>(rest1_original.c_str()),
      rest1_original.length());
    std::string rest1_decoded = base64_decode(rest1_encoded);

    if (rest1_decoded != rest1_original) {
      std::cout << "rest1_decoded != rest1_original" << std::endl;
      all_tests_passed = false;
    }
    if (rest1_reference != rest1_encoded) {
      std::cout << "rest1_reference != rest1_encoded" << std::endl;
      all_tests_passed = false;
    }

 // std::cout << "encoded:   " << rest1_encoded << std::endl;
 // std::cout << "reference: " << rest1_reference << std::endl;
 // std::cout << "decoded:   " << rest1_decoded << std::endl << std::endl;

    std::string rest2_original = "abcde";
    std::string rest2_reference = "YWJjZGU=";

    std::string rest2_encoded = base64_encode(reinterpret_cast<const unsigned char*>(rest2_original.c_str()),
      rest2_original.length());
    std::string rest2_decoded = base64_decode(rest2_encoded);

    if (rest2_decoded != rest2_original) {
      std::cout << "rest2_decoded != rest2_original" << std::endl;
      all_tests_passed = false;
    }
    if (rest2_reference != rest2_encoded) {
      std::cout << "rest2_reference != rest2_encoded" << std::endl;
      all_tests_passed = false;
    }

 // std::cout << "encoded:   " << rest2_encoded << std::endl;
 // std::cout << "reference: " << rest2_reference << std::endl;
 // std::cout << "decoded:   " << rest2_decoded << std::endl << std::endl;

    if (all_tests_passed) return 0;
    return 1;
}
