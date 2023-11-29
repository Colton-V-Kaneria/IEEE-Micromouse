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

int center = 1;

//Pass through pointer maze storing data in memory located at struct Maze
void checkWalls(struct Maze *maze){
    if (API::wallFront())
        maze ->cellWalls[maze->mouse_pos.y][maze ->mouse_pos.x];
}

int main(int argc, char* argv[]) {
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
void updateMousePos(Coord* pos, Direction dir)

//testing number 2 


//Test comment 10:04 Pm 11/28/23