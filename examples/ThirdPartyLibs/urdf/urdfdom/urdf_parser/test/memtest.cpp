#include "urdf_parser/urdf_parser.h"
#include <fstream>
#include <iostream>

int main(int argc, char** argv){
  while (true){
    std::string xml_string;
    std::fstream xml_file(argv[1], std::fstream::in);
    while ( xml_file.good() )
    {
      std::string line;
      std::getline( xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();


    urdf::parseURDF(xml_string);
  }
}
