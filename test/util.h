#ifndef UTIL_H
#define UTIL_H

#include <cinttypes>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string.h>

struct Image{
    uint16_t width, height;
    std::vector<uint8_t> data;

    const uint8_t& at(int x, int y) const
    {
        return data[y*width + x];
    }

    uint8_t& at(int x, int y)
    {
        return data[y*width + x];
    }
};

inline bool ReadImageFromPGM(const char* filename, Image* image)
{
    std::ifstream infile(filename, std::ios_base::in | std::ios_base::binary);
    std::stringstream ss;
    std::string inputLine = "";

    // First line : version
    std::getline(infile,inputLine);
    if(inputLine.compare("P5") != 0)
    {
        std::cerr << "PGM Version error "<< inputLine << std::endl;
        exit(1);
    }

    // Second line : comment
    std::getline(infile,inputLine);
    //std::cout << "Comment : " << inputLine << std::endl;

    std::getline(infile,inputLine);
    ss << inputLine;
    ss >> image->width >> image->height;

    uint8_t max_value = 0;
    std::getline(infile,inputLine);
    max_value = stoi(inputLine);

   // std::cout << image->width << " width and " << image->height << " height "
   //           << (int)max_value << " max value " << std::endl;

    const int img_size  = image->width * image->height;
    image->data.resize( img_size );

    infile.read( reinterpret_cast<char*>(image->data.data()), img_size);
    infile.close();

    return true;
}

inline void WriteImageToPGM(const char* filename, const Image& image)
{
    std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);

    char header[100];
    sprintf(header, "P5\n# Done by Davide\n%d %d\n255\n",image.width, image.height );

    outfile.write(header, strlen(header));
    outfile.write( reinterpret_cast<const char*>(image.data.data()), image.data.size() );
    outfile.close();
}



#endif // UTIL_H
