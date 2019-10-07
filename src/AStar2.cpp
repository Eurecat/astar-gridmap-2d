/*
Copyright 2018 Eurecat

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the
License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
either express or implied. See the License for the specific language governing permissions
 and limitations under the License.
*/

#include "AStar2.h"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <cinttypes>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string.h>

using namespace std::placeholders;

namespace AStar{

bool Coord2D::operator == (const Coord2D& other) const
{
    return (x == other.x && y == other.y);
}

Coord2D operator + (const Coord2D& left_, const Coord2D& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}


PathFinder::PathFinder():
    _open_set( CompareScore() )
{

    _obstacle_threshold = 0;
    setHeuristic(&Heuristic::manhattan);
    _directions = {{
        { -1, -1 },  { 0, -1 }, { 1, -1 }, //0 - 2
        { -1,  0 },             { 1,  0 }, //3 - 4
        { -1,  1 },  { 0, 1 },  { 1,  1 } //5 - 7
    }};

    _direction_cost = {{
        14, 10, 14,
        10,     10,
        14, 10, 14
    }};
}

PathFinder::~PathFinder()
{
}


void PathFinder::setWorldData(unsigned width, unsigned height, const uint8_t *data, size_t bytes_per_line, uint8_t color_threshold)
{
    if( width >= std::numeric_limits<int16_t>::max() ||
        height >= std::numeric_limits<int16_t>::max() )
    {
        throw std::invalid_argument("Either width or height exceed the maximum size allowed (32768) ");
    }
    
    _world_width  = width;
    _world_height = height;
    _world_data = data;
    _bytes_per_line = bytes_per_line;
    if (_bytes_per_line==0) _bytes_per_line=width;
    _obstacle_threshold = color_threshold;
}

void PathFinder::setHeuristic(HeuristicFunction heuristic_)
{
    _heuristic = std::bind(heuristic_, _1, _2);
}


void PathFinder::clear()
{
    _open_set = decltype(_open_set)(); // priority_queue does not have "clear"
    _gridmap.resize(_world_width*_world_height);
    std::fill(_gridmap.begin(), _gridmap.end(), Cell());
}


CoordinateList PathFinder::findPath(Coord2D startPos, Coord2D goalPos)
{
    clear();

    auto toIndex = [this](Coord2D pos) -> int
    { return static_cast<int>(_world_width*pos.y + pos.x); };

    const int startIndex = toIndex(startPos);

    _open_set.push( {0, startPos } );
    _gridmap[startIndex].cost_G = 0.0;
 
    if( detectCollision( startPos ) )
    {
       return {};
    }

    bool solution_found = false;

    while (! _open_set.empty() )
    {
        Coord2D currentCoord = _open_set.top().second;
        _open_set.pop();

        if (currentCoord == goalPos) {
            solution_found = true;
            break;
        }
        int currentIndex = toIndex(currentCoord);
        Cell& currentCell = _gridmap[ currentIndex ];
        currentCell.already_visited = true;

        for (int i = 0; i < 8; ++i)
        {
            const Coord2D newCoordinates = currentCoord + _directions[i];

            if (detectCollision( newCoordinates )) {
                continue;
            }

            const size_t newIndex = toIndex(newCoordinates);
            Cell& newCell = _gridmap[newIndex];

            if ( newCell.already_visited ) {
                continue;
            }

            // Code temporary removed.
            //float factor = 1.0f + static_cast<float>(EMPTY - newCell.world) / 50.0f;
            //float new_cost = currentCell.cost_G + (_direction_cost[i] * factor);

            const float new_cost = currentCell.cost_G + _direction_cost[i];

            if( new_cost < newCell.cost_G)
            {
                auto H = _heuristic( newCoordinates, goalPos );
                _open_set.push( { new_cost + H, newCoordinates } );
                newCell.cost_G = new_cost;
                newCell.path_parent = currentCoord;
            }
        }
    }

    CoordinateList path;
    if( solution_found )
    {
        Coord2D coord = goalPos;
        while (coord != startPos)
        {
            path.push_back( coord );
            coord = cell(coord).path_parent;
        }
        path.push_back(startPos);
    }
    else
    {
        std::cout << "Solution not found\n" <<
                     " open set size= " << _open_set.size() << std::endl;
    }

    return path;
}

void PathFinder::exportPPM(const char *filename, CoordinateList* path)
{
    if (_world_data==nullptr)
        return;
    
    std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);

    char header[100];
    sprintf(header, "P6\n# Done by Davide\n%d %d\n255\n", _world_width, _world_height );
    outfile.write(header, strlen(header));

    std::vector<uint8_t> image( _world_width * _world_height * 3);

    int line_size = _world_width * 3;

    auto toIndex = [line_size](int x, int y) { return y*line_size + (x*3); };

    for (uint32_t y=0; y<_world_height; y++)
    {
        for (uint32_t x=0; x<_world_width; x++)
        {
            uint8_t world_value = _world_data[y*_bytes_per_line+x];
            
            if( world_value <= _obstacle_threshold )
            {
                uint8_t color[] = {0,0,0};
                std::memcpy( &image[ toIndex(x,y) ], color, 3 );
            }
            else if( _gridmap[ y*_world_width + x ].already_visited )
            {
                uint8_t color[] = {255,222,222};
                std::memcpy( &image[ toIndex(x,y) ], color, 3 );
            }
            else{
                uint8_t color[] = {255,255,255};
                std::memcpy( &image[ toIndex(x,y) ], color, 3 );
            }
        }
    }

    if( path )
    {
        for (const auto& point: *path)
        {
            uint8_t color[] = {50,50,250};
            std::memcpy( &image[ toIndex(point.x, point.y) ], color, 3 );
        }
    }

    outfile.write( reinterpret_cast<char*>(image.data()), image.size() );
    outfile.close();
}

}
