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

#ifndef _ASTAR2_Taffete_eurecat_
#define _ASTAR2_Taffete_eurecat_

#include <array>
#include <functional>
#include <set>
#include <map>
#include <unordered_map>
#include <queue>
#include <algorithm>

namespace AStar
{

struct Coord2D
{
    int16_t x, y;
    Coord2D(): x(-1), y(-1) {}
    Coord2D(int x_, int y_): x(x_), y(y_) {}
    bool operator == (const Coord2D& other) const;
    bool operator != (const Coord2D& other) const { return !(*this == other); }
};


using HeuristicFunction = std::function<uint32_t(Coord2D, Coord2D)>;
using CoordinateList = std::vector<Coord2D>;


typedef std::pair<uint32_t,Coord2D> ScoreCoordPair;

struct CompareScore
{
    //Note: we want the priority_queue to be ordered from smaller to larger
    bool operator() (const ScoreCoordPair& a,
                     const ScoreCoordPair& b)
    {
        return a.first > b.first;
    }
};

class PathFinder
{

public:
    PathFinder();
    ~PathFinder();

    ///
    /**
     * @brief setWorldData to be called at least once before findPath
     *
     * @param width            width of the image
     * @param height           height of the image
     * @param data             Row-major ordered map, where an obstacle is represented as a pixel with value 0 (black)
     * @param bytes_per_line   Number of bytes per line (for padded data, eg. images). Default means image is not padded
     * @param color_threshold  threshold used to detect if a grey pixel is an obstacle
     */
    void setWorldData(unsigned width, unsigned height, const uint8_t *data, size_t bytes_per_line=0, uint8_t color_threshold = 20);

    /// Default value is Heuristic::manhattan
    void setHeuristic(HeuristicFunction heuristic_);

    /// Function that performs the actual A* computation.
    CoordinateList findPath(Coord2D source_, Coord2D target_);


    /// Export the resulting solution in a visual way. Useful for debugging.
    void exportPPM(const char* filename, CoordinateList* path);

    enum{
        OBSTACLE = 0,
        EMPTY    = 255
    };

    uint8_t _obstacle_threshold;

    struct Cell{
        uint8_t  world;
        bool     already_visited;
        Coord2D  path_parent;
        float    cost_G;
    };

    const Cell& cell(Coord2D coordinates_) const
    {
        return _gridmap[coordinates_.y*_world_width + coordinates_.x];
    }

    Cell& cell(Coord2D coordinates_)
    {
        return _gridmap[coordinates_.y*_world_width + coordinates_.x];
    }

private:

    HeuristicFunction _heuristic;
    uint32_t _world_width;
    uint32_t _world_height;

    std::array<Coord2D,8>  _directions;
    std::array<uint32_t,8> _direction_cost;

    std::priority_queue<ScoreCoordPair, std::vector<ScoreCoordPair>, CompareScore> _open_set;

    bool detectCollision(Coord2D coordinates);

    std::vector<Cell> _gridmap;

    void  clean();
};

class Heuristic
{
public:
    static uint32_t manhattan(Coord2D source_, Coord2D target_);
    static uint32_t euclidean(Coord2D source_, Coord2D target_);
    static uint32_t octagonal(Coord2D source_, Coord2D target_);
};



inline bool PathFinder::detectCollision(Coord2D coordinates)
{
    return (coordinates.x < 0 || coordinates.x >= _world_width ||
            coordinates.y < 0 || coordinates.y >= _world_height ||
            cell(coordinates).world <= _obstacle_threshold );
}


inline uint32_t Heuristic::manhattan(Coord2D source, Coord2D target)
{
    auto delta = Coord2D( (source.x - target.x), (source.y - target.y) );
    return static_cast<uint32_t>(10 * ( abs(delta.x) + abs(delta.y)));
}

inline uint32_t Heuristic::euclidean(Coord2D source, Coord2D target)
{
    auto delta = Coord2D( (source.x - target.x), (source.y - target.y) );
    return static_cast<uint32_t>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

inline uint32_t Heuristic::octagonal(Coord2D source, Coord2D target)
{
    auto delta = Coord2D( abs(source.x - target.x), abs(source.y - target.y) );
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}


}

#endif
