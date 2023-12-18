/**
 * Project: UCLA IEEE Micromouse (STM32)
 * Names: Mervin, Peter, Colton
 * Description: 
*/

#include <iostream>
#include <string>

#include "API.h"
#include "FloodFill_DataStructures.h"

Direction dir = NORTH;  //from floodfill header function call
Maze maze;

//y = row, x = column
// NORTH_MASK = 0b1000,
    // EAST_MASK  = 0b0100,
    // SOUTH_MASK = 0b0010,
    // WEST_MASK  = 0b0001

//hard code walls for the outside perimeter of the the 16x16 maze
void setPerimeter(struct Maze *maze){
    for(int y = 0; y < 16; y++) //iterate through rows from bottom left to to right
        {
            for(int x = 0; x < 16; x++)
            {

                if (y == 0) 
                {
                    maze->cellWalls[x][y] |= SOUTH_MASK;
                }

                if(y == 15)
                {
                    maze->cellWalls[x][y] |= NORTH_MASK;
                }

                if(x == 0)
                {
                    maze->cellWalls[y][x] |= WEST_MASK;
                }

                if(x == 15)
                {
                    maze->cellWalls[y][x] |= EAST_MASK;
                }
            }
        }
}
int center = 1;

//Pass through pointer maze storing data in memory located at struct Maze
void checkWalls(struct Maze *maze){
    if (API::wallFront())
        maze ->cellWalls[maze->mouse_pos.y][maze ->mouse_pos.x];
}

int main(int argc, char* argv[]) {
    setPerimeter(&maze);
    API::setColor(0, 0, 'g');   //(0,0), green

    maze.mouse_pos = (Coord){0, 0};     //setting the coordinate to 0,0 facing north
    maze.mouse_dir = NORTH;
    
    std::cerr << "Begin Maze Algorithm!\n" << std::endl;
}

//initialize the distance with the center 4 cells as zero 
void mazeCenter()
{
    if (center == 1)
    {
          //This should assign a number describing how far away the specific cell is from the center
            for(int x = 0; x < 16; x++) 
        {
                for(int y = 0; y < 16; y++)
                {
                    maze.distances[y][x] = x + y;
                }
        }

    }
}

// 4 center cells in the middle get set to zero

//
// The goal cell is centered at (7,7)
// At any assesible and free cell, its distance away from the center would be
// |7-x|+|7-y|= remaining distance (essentially manhattan distance)
//
//testing
void updateMousePos(Coord* pos, Direction dir){
    
}

