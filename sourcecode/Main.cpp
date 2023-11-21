/**
 * Project: UCLA IEEE Micromouse (STM32)
 * Names: Mervin, Peter, Colton
 * Description: 
*/

#include <iostream>
#include <string>

#include "API.h"
#include "Floodfill.h"

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
    
    
    //This should assign a number describing how far away the specific cell is from the center
    for(int x = 0; x < 16; x++) 
    {
        for(int y = 0; y < 16; y++)
        {
            maze.distances[y][x] = x + y;
        }
    }

}

//testing 
void updateMousePos(Coord* pos, Direction dir)