#include <iostream>
#include <string>

#include <queue>

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

    int x = maze->mouse_pos.x;              // these are just to make it easier to type the mouse's coordinates below
    int y = maze->mouse_pos.y;

    // north cell
    cellList->cells[0].pos.x = x;
    cellList->cells[0].pos.y = y+1;
    cellList->cells[0].dir = NORTH;
    cellList->cells[0].blocked = maze->cellWalls[y][x] & NORTH_MASK;
    // ^^use MASK &'ed with cellWalls's bit value at the mouse's current coordinates to determine if a cell is blocked

    // east cell
    cellList->cells[1].pos.x = x+1;
    cellList->cells[1].pos.y = y;
    cellList->cells[1].dir = EAST;
    cellList->cells[1].blocked = maze->cellWalls[y][x] & EAST_MASK;
    
    // south cell
    cellList->cells[2].pos.x = x;
    cellList->cells[2].pos.y = y-1;
    cellList->cells[2].dir = SOUTH;
    cellList->cells[2].blocked = maze->cellWalls[y][x] & SOUTH_MASK;
    
    // west cell
    cellList->cells[3].pos.x = x-1;
    cellList->cells[3].pos.y = y;
    cellList->cells[3].dir = WEST;
    cellList->cells[3].blocked = maze->cellWalls[y][x] & WEST_MASK;
    
    return cellList;
};

Maze maze;

int temp_value = 20;

int main(int argc, char* argv[]) 
{
    maze.mouse_pos = (Coord){0, 0};
    maze.mouse_dir = NORTH;

    // 4. POINTER DEMO
    //pointer_demo(&temp_value);
    //std::cerr << temp_value << std::endl;

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
        
        std::cerr << "At (" << maze.mouse_pos.x << ", " << maze.mouse_pos.y << ")" << std::endl;

        for (int i = 0; i < adjacentCells->size; i++)
        {
            std::cerr << "(" << adjacentCells->cells[i].pos.x << ", " << adjacentCells->cells[i].pos.y << ")\t blocked: " << adjacentCells->cells[i].blocked << std::endl;
        }

        free(adjacentCells->cells);
        free(adjacentCells);

        updateSimulator(maze);


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