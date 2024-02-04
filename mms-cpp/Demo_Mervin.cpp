#include <iostream>
#include <string>

#include "API.h"
#include "FloodFill.h"

void updateSimulator(Maze maze) // redraws the maze in simulator after each loop in main
{
    for(int x = 0; x < 16; x++) 
    {
        for(int y = 0; y < 16; y++) 
        {
            API::setText(x, y, std::to_string(maze.distances[y][x]));

            if (maze.cellWalls[y][x] & NORTH_MASK)
                API::setWall(x, y, 'n');
            if (maze.cellWalls[y][x] & EAST_MASK)
                API::setWall(x, y, 'e');
            if (maze.cellWalls[y][x] & SOUTH_MASK)
                API::setWall(x, y, 's');
            if (maze.cellWalls[y][x] & WEST_MASK)
                API::setWall(x, y, 'w');
        }
    }
}

void scanWalls(Maze* maze)
{
    if (API::wallFront())
        maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x] |= dir_mask[maze->mouse_dir];
    if (API::wallRight())
        maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x] |= dir_mask[(maze->mouse_dir + 1) % 4];  
    if (API::wallLeft())
        maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x] |= dir_mask[(maze->mouse_dir + 3) % 4];   
}


void updateMousePos(Coord* pos, Direction dir)
{
    if (dir == NORTH)
        pos->y++;
    if (dir == SOUTH)
        pos->y--;
    if (dir == WEST)
        pos->x--;
    if (dir == EAST)
        pos->x++;
    
    // std::cerr << "inside function: (" << pos.x << ", " << pos.y << ")" << std::endl;
}

// 6. FILL THIS IN
CellList* getNeighborCells(Maze* maze)
{
    CellList* cellList = (CellList*)malloc(sizeof(CellList));

    // cellList->size = ???
    cellList->cells = (Cell*)malloc(cellList->size * sizeof(Cell));

    cellList->size = 2;
    cellList->cells = (Cell*)malloc(cellList->size * sizeof(Cell));

    cellList->cells[0] = (Cell){(Coord){24, 25}, NORTH, false};
    cellList->cells[1] = (Cell){(Coord){4, 5}, WEST, true};

    return cellList;
};

Maze maze;

//int temp_value = 20;

int main(int argc, char* argv[]) 
{
    maze.mouse_pos = (Coord){0, 0};
    maze.mouse_dir = NORTH;

    // 4. POINTER DEMO
    // pointer_demo(temp_value);
    // std::cerr << temp_value << std::endl;

    // 1. FILL THIS IN
    for(int x = 0; x < 16; x++)
    {
        for(int y = 0; y < 16; y++)
        {
            maze.distances[y][x] = x + y;
        }
    }

    while (true) {
        scanWalls(&maze);

        CellList* adjacentCells = getNeighborCells(&maze);
        for (int i = 0; i < adjacentCells->size; i++)
        {
            std::cerr << adjacentCells->cells[i].pos.x << ", " << adjacentCells->cells[i].pos.y << std::endl;
        }

        free(adjacentCells->cells);
        free(adjacentCells);    //free pointer address linked to *adjacentCells

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
    
        // 3. UPDATE THIS WITH POINTERS
        updateMousePos(&maze.mouse_pos, maze.mouse_dir);

        // 5. MOVE TO updateSimulator() + scanWalls() FUNCTION
    }
}
