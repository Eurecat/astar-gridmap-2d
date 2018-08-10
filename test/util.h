#ifndef UTIL_H
#define UTIL_H

#include <cinttypes>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string.h>

class Image{
    uint16_t _width, _height;
    std::vector<uint8_t> _data;
public:

    uint16_t width() const { return _width; }

    uint16_t height() const { return _height; }

    const uint8_t& at(int x, int y) const
    {
        return _data[y*_width + x];
    }

    uint8_t& at(int x, int y)
    {
        return _data[y*_width + x];
    }

    const uint8_t* data() const
    {
        return _data.data();
    }

    bool readFromPGM(const char* filename)
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
        ss >> _width >> _height;

        uint8_t max_value = 0;
        std::getline(infile,inputLine);
        max_value = stoi(inputLine);

        // std::cout << image->width << " width and " << image->height << " height "
        //           << (int)max_value << " max value " << std::endl;

        const int img_size  = _width * _height;
        _data.resize( img_size );

        infile.read( reinterpret_cast<char*>(_data.data()), img_size);
        infile.close();

        return true;
    }

    void writeToPGM(const char* filename)
    {
        std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);

        char header[100];
        sprintf(header, "P5\n# Done by Davide\n%d %d\n255\n", _width, _height );

        outfile.write(header, strlen(header));
        outfile.write( reinterpret_cast<const char*>( _data.data()), _data.size() );
        outfile.close();
    }

};




#endif // UTIL_H
