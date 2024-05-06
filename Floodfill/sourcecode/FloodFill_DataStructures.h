// 16 x 16 2D array for maze
//starting point coord x=0, y=0

#ifndef FLOODFILL_DATASTRUCTURES_H
#define FLOODFILL_DATASTRUCTURES_H

#include "API.h"
#include <iostream>

void log(const std::string& text) {
    std::cerr << text << std::endl;
};
    
enum Direction {
    NORTH = 8,
    EAST = 4,
    SOUTH = 2,
    WEST = 1,
};

enum DirectionBitmask {
    NORTH_MASK = 0b1000,
    EAST_MASK  = 0b0100,
    SOUTH_MASK = 0b0010,
    WEST_MASK  = 0b0001
};

struct Coord {
    int x;
    int y;
};

struct Cell {
    Coord pos;
    Direction dir;
    bool blocked;
};

struct CellList {
    int size;
    Cell* cells;
};

struct Maze {
    Coord mouse_pos;
    Direction mouse_dir;

    int distances[16][16];
    int cellWalls[16][16];

    Coord* goalPos;
};

char dir_chars[4] = {'n', 'e', 's', 'w'};
int dir_mask[4] = {0b1000, 0b0100, 0b0010, 0b0001};


#endif //FLOODFILL_H