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

#include <vector>
#include <functional>
#include <set>
#include <map>
#include <unordered_map>
#include <queue>

namespace AStar
{

struct Coord2D
{
    int x, y;
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

    /// Row-major ordered map, where an obstacle is represented as a pixel with value 0 (black)
    void setWorldData(unsigned width, unsigned height, const uint8_t *data);

    /// Default value is Heuristic::manhattan
    void setHeuristic(HeuristicFunction heuristic_);

    /// Function that performs the actual A* computation.
    CoordinateList findPath(Coord2D source_, Coord2D target_);


    /// If false, it looks at the neighbours ina 3x3 area arounf the current position.
    /// If false, a 5x5 search area is used instead.
    void allow5by5(bool allow)
    {
        _allow_5x5_search = allow;
    }

    /// Export the resulting solution in a visual way. Useful for debugging.
    void exportPPM(const char* filename, CoordinateList* path);

    enum{
        OBSTACLE = 0,
        EMPTY    = 255
    };

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
    bool _allow_5x5_search;

    std::array<Coord2D,24> _directions;
    std::array<float,24>   _direction_cost;

    std::priority_queue<ScoreCoordPair, std::vector<ScoreCoordPair>, CompareScore> _open_set;

    bool detectCollision(Coord2D coordinates);

    std::vector<Cell> _gridmap;

    void  clean();
};

class Heuristic
{
    static Coord2D getDelta(Coord2D source_, Coord2D target_);

public:
    static float manhattan(Coord2D source_, Coord2D target_);
    static float euclidean(Coord2D source_, Coord2D target_);
    static float octagonal(Coord2D source_, Coord2D target_);
};

}

#endif
