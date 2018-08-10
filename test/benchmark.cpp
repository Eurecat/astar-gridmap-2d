#include <iostream>
#include <benchmark/benchmark.h>

#include "util.h"
#include "AStar2.h"


static void BM_AStar_Smooth_1000(benchmark::State& state)
{
    AStar::PathFinder generator;
    Image image;
    ReadImageFromPGM("../data/maze_1000_smooth.pgm", &image);
    generator.setWorldData( image.width, image.height, image.data.data() );
    generator.allow5by5(false);

    AStar::CoordinateList result;
    for (auto _ : state)
    {
        result = generator.findPath(
        { image.width/2, 0 },
        { image.width/2, image.height -1 } );
    }
    generator.exportPPM("map_out_smooth.ppm", &result );
}


static void BM_AStar_Big(benchmark::State& state)
{
    AStar::PathFinder generator;
    Image image;
    ReadImageFromPGM("../data/maze_large.pgm", &image);
    generator.setWorldData( image.width, image.height, image.data.data() );
    generator.allow5by5(false);

    AStar::CoordinateList result;
    for (auto _ : state)
    {
        result = generator.findPath(
        { image.width/2, 0 },
        { image.width/2, image.height -1 } );
    }
    generator.exportPPM("map_out_large.ppm", &result );
}

static void BM_AStar_Small(benchmark::State& state)
{
    AStar::PathFinder generator;
    Image image;
    ReadImageFromPGM("../data/maze_250.pgm", &image);
    generator.setWorldData( image.width, image.height, image.data.data() );
    generator.allow5by5(false);

    AStar::CoordinateList result;
    for (auto _ : state)
    {
        result = generator.findPath(
        { image.width/2, 0 },
        { image.width/2, image.height/2 } );
    }
    generator.exportPPM("map_out_small.ppm", &result );
}


BENCHMARK(BM_AStar_Big);
BENCHMARK(BM_AStar_Smooth_1000);
BENCHMARK(BM_AStar_Small);

BENCHMARK_MAIN();

//int main()
//{
//    AStar::PathFinder generator;
//    Image image;
//    ReadImageFromPGM("../data/maze_large.pgm", &image);
//    generator.setWorldData( image.width, image.height, image.data.data() );
//    generator.allow5by5(false);

//    AStar::CoordinateList result;

//    result = generator.findPath(
//        { image.width/2, 0 },
//        { image.width/2, image.height -1 } );

//    generator.exportPPM("map_out_large.ppm", &result );

//    return 0;
//}

