#include <iostream>
#include <string>

#include "API.h"

void log(const std::string& text) 
{
    std::cerr << text << std::endl;
}

void pointer_demo(int* var) // big difference between int and int*: int* is a pointer value
{
    *var = 42;
}

enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
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

struct Cell {           // each cell has a position, direction, and blocked T/F
    Coord pos;
    Direction dir;
    bool blocked;
};

struct CellList {       // a list of cells has a size and pointer to a particular cell
    int size;
    Cell* cells;        // pointer to a Cell
};

struct Maze {               // the maze has coordinates of mouse, direction of mouse, and 
    Coord mouse_pos;
    Direction mouse_dir;

    int distances[16][16];      // this 2D array represents the distance value held by each position
    int cellWalls[16][16];      // this 2D array represents the bitwise wall indicator

    Coord* goalPos;
};

char dir_chars[4] = {'n', 'e', 's', 'w'};
int dir_mask[4] = {0b1000, 0b0100, 0b0010, 0b0001};

// 2. FILL THIS IN
void updateSimulator(Maze maze) // redraws the maze in simulator after each loop in main
{
    for (int y = 0; y < 16; y++)
    {
        for (int x = 0; x < 16; x++)
        {
            // built-in functions??? ASK ABT IT
            API::setText(x, y, std::to_string(maze.distances[y][x]));
            if (maze.cellWalls[y][x] & NORTH_MASK)
                API::setWall(x, y, 'n'); // set a north wall if a mouse encounters it
            if (maze.cellWalls[y][x] & EAST_MASK)
                API::setWall(x, y, 'e'); // set an east wall if a mouse encounters it
            if (maze.cellWalls[y][x] & SOUTH_MASK)
                API::setWall(x, y, 's'); // set a south wall if a mouse encounters it
            if (maze.cellWalls[y][x] & WEST_MASK)
                API::setWall(x, y, 'w'); // set a west wall if a mouse encounters it
        }
    }
}

// 5. FILL THIS IN
void scanWalls(Maze* maze)
{
    if (API::wallFront())
        maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x] |= dir_mask[maze->mouse_dir];                 // bitwise OR chagnes specific bit to specify wall detected
    if (API::wallRight())
        maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x] |= dir_mask[(maze->mouse_dir + 1) % 4];
    if (API::wallLeft())
        maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x] |= dir_mask[(maze->mouse_dir + 3) % 4];
}

// 3. FILL THIS IN
void updateMousePos(Coord* pos, Direction dir) // Changed to pointer of coordinate so coordinates update in function
{
    if (dir == NORTH)
        pos->y++;
    if (dir == SOUTH)
        pos->y--;
    if (dir == WEST)
        pos->x--;
    if (dir == EAST)
        pos->x++;
    std::cerr << "inside function: (" << pos->x << ", " << pos->y << ")" << std::endl;
}

// 6. FILL THIS IN
CellList* getNeighborCells(Maze* maze)
{
    CellList* cellList = (CellList*)malloc(sizeof(CellList));
    int i = 0;

    // some logic to set the cell list size
    cellList->size = 4;
    cellList->cells = (Cell*)malloc(cellList->size * sizeof(Cell)); // allocates number of bits the struct requires

    for (int j = 0; j < 4; j++)
    {
        cellList->cells[j].pos.x = -1;
        cellList->cells[j].pos.y = -1;
    }

    // check if north cell
    if (maze->mouse_pos.y != 15)
    {
        int x = maze->mouse_pos.x;
        int y = maze->mouse_pos.y;

        bool is_blocked = (bool)(maze->cellWalls[y + 1][x] & NORTH_MASK);

        cellList->cells[i] = (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y+1}, NORTH, is_blocked};
        i++;
    }
    // check if east cell
    if (maze->mouse_pos.x != 15)
    {
        cellList->cells[i] = (Cell){(Coord){maze->mouse_pos.x+1, maze->mouse_pos.y}, EAST, true};
        i++;
    }
    // check if south cell
    if (maze->mouse_pos.y != 0)
    {
        cellList->cells[i] = (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y-1}, SOUTH, true};
        i++;
    }
    // check if west cell
    if (maze->mouse_pos.x != 0)
    {
        cellList->cells[i] = (Cell){(Coord){maze->mouse_pos.x-1, maze->mouse_pos.y}, WEST, true};
        i++;
    }
    
    return cellList;
};

Maze maze;

int temp_value = 20;

int main(int argc, char* argv[]) 
{
    maze.mouse_pos = (Coord){0, 0};
    maze.mouse_dir = NORTH;

    // 4. POINTER DEMO
    pointer_demo(&temp_value);
    std::cerr << temp_value << std::endl;

    // 1. FILL THIS IN
    for(int x = 0; x < 16; x++) {
        for(int y = 0; y < 16; y++) {
            maze.distances[y][x] = x + y;
        }
    }

    while (true) {
        scanWalls(&maze);
        CellList* adjacentCells = getNeighborCells(&maze);

        std::cerr << "NEIGHBOR CELLS" << std::endl;

        for (int i = 0; i < adjacentCells->size; i++)
        {
            std::cerr << adjacentCells->cells[i].pos.x << ", " << adjacentCells->cells[i].pos.y << std::endl;
        }

        free(adjacentCells->cells);
        free(adjacentCells);

        updateSimulator(maze);

        std::cerr << "(" << maze.mouse_pos.x << ", " << maze.mouse_pos.y << ")" << std::endl;

        // Left Wall Follow Code
        if (!API::wallLeft()) 
        {
            API::turnLeft();
            maze.mouse_dir = (Direction)((maze.mouse_dir + 3) % 4);
        }
        while (API::wallFront()) 
        {
            API::turnRight();
            maze.mouse_dir = (Direction)((maze.mouse_dir + 1) % 4);
        }

        API::moveForward();
    
        // 3. UPDATE THIS WITH POINTERS updateMousePos
        updateMousePos(&maze.mouse_pos, maze.mouse_dir);

        // 5. MOVE TO updateSimulator() + scanWalls() FUNCTION
        if (API::wallFront())
            API::setWall(maze.mouse_pos.x, maze.mouse_pos.y, dir_chars[maze.mouse_dir]);
        if (API::wallRight())
            API::setWall(maze.mouse_pos.x, maze.mouse_pos.y, dir_chars[(maze.mouse_dir + 1) % 4]);
        if (API::wallLeft())
            API::setWall(maze.mouse_pos.x, maze.mouse_pos.y, dir_chars[(maze.mouse_dir + 3) % 4]);
    }
}