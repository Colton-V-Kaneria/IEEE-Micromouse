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
                    maze->cellWalls[y][x] |= SOUTH_MASK;
                }

                if(y == 15)
                {
                    maze->cellWalls[y][x] |= NORTH_MASK;
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


void updateMouseSimulator(struct Maze *maze){
    //update the simulator with the current mouse position and direction
    std::string direction(1, dir_chars[maze->mouse_dir]); // Convert char to string
    API::setText(maze->mouse_pos.x, maze->mouse_pos.y, direction); // Pass the string to setText function
    std::cerr << "Mouse Position: (" << maze->mouse_pos.x << ", " << maze->mouse_pos.y << ")" << std::endl;
}

CellList* getNeighborCells(Maze* maze){
     CellList* cellList = (CellList*)malloc(sizeof(CellList));
    if(cellList == NULL){   //handle memory allocation failure
        return NULL;
    }
     //Checks for existing cellls should be done before setting list size

     cellList->size = 4;
     cellList->cells = (Cell*)malloc(cellList->size * sizeof(Cell));

     if(cellList->cells == NULL){
        //handle memory allocation failure
         free(cellList);
         return NULL;
     }

    //Always add the four neighborhood cells
    cellList->cells[0]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y + 1}, NORTH, false};
    cellList->cells[1]= (Cell){(Coord){maze->mouse_pos.x + 1, maze->mouse_pos.y},  EAST, false};
    cellList->cells[2]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y - 1}, SOUTH, false};
    cellList->cells[3]= (Cell){(Coord){maze->mouse_pos.x - 1, maze->mouse_pos.y},  WEST, false};

    return cellList;
}

//     //Checks if north exists
//     if(maze->mouse_pos.y > 0 && maze->mouse_pos.y < 16 && maze->mouse_pos.x > 0 && maze->mouse_pos.x < 16 )
//     {

//         cellList->cells[i]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y + 1}, NORTH, false};
//         cellList->cells[i+1]= (Cell){(Coord){maze->mouse_pos.x + 1, maze->mouse_pos.y},  EAST, false};
//         cellList->cells[i+2]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y - 1}, SOUTH, false};
//         cellList->cells[i+3]= (Cell){(Coord){maze->mouse_pos.x - 1, maze->mouse_pos.y},  WEST, false};
//         i++;

//     }  

//     //FOR NOW IGNORE THE FOLLOWING BIT AND ONLY FOCUS ON TOP^^^

//     //WEST
//     if(maze->mouse_pos.x == 0 && maze->mouse_pos.y < 15)
//     {
//         cellList->cells[i]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y + 1}, NORTH, false};
//         cellList->cells[i+1]= (Cell){(Coord){maze->mouse_pos.x + 1, maze->mouse_pos.y},  EAST, false};
//         cellList->cells[i+2]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y - 1}, SOUTH, false};
//         cellList->cells[i+3]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y},  WEST, false};
//         i++;
//     }

//     //EAST
//     if(maze->mouse_pos.x == 15)
//     {
//         cellList->cells[i]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y + 1}, NORTH, false};
//         cellList->cells[i+1]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y},  EAST, false};
//         cellList->cells[i+2]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y - 1}, SOUTH, false};
//         cellList->cells[i+3]= (Cell){(Coord){maze->mouse_pos.x - 1, maze->mouse_pos.y},  WEST, false};
//         i++;
//     }

//     //SOUTH
//     if(maze->mouse_pos.y == 0)
//     {
//         cellList->cells[i]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y + 1}, NORTH, false};
//         cellList->cells[i+1]= (Cell){(Coord){maze->mouse_pos.x + 1, maze->mouse_pos.y},  EAST, false};
//         cellList->cells[i+2]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y}, SOUTH, false};
//         cellList->cells[i+3]= (Cell){(Coord){maze->mouse_pos.x - 1, maze->mouse_pos.y},  WEST, false};
//         i++;

//     }

//     //NORTH
//     if(maze->mouse_pos.y == 15 && maze->mouse_pos.x > 0)
//     {
//         cellList->cells[i]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y}, NORTH, false};
//         cellList->cells[i+1]= (Cell){(Coord){maze->mouse_pos.x + 1, maze->mouse_pos.y},  EAST, false};
//         cellList->cells[i+2]= (Cell){(Coord){maze->mouse_pos.x, maze->mouse_pos.y - 1}, SOUTH, false};
//         cellList->cells[i+3]= (Cell){(Coord){maze->mouse_pos.x - 1, maze->mouse_pos.y},  WEST, false};
//         i++;

//     }

// //You can uncomment starting below

//     return cellList;
// }

//Pass through pointer maze storing data in memory located at struct Maze
void checkWalls(struct Maze *maze){
    if (API::wallFront())
        maze ->cellWalls[maze->mouse_pos.x][maze ->mouse_pos.y];
}

int main(int argc, char* argv[]) {
    Maze maze;
    setPerimeter(&maze);
    API::setColor(0, 0, 'r');   //(0,0), green
    API::setColor(8, 8, 'g');   //(0,0), green        
    API::setColor(8, 7, 'g');
    API::setColor(7, 8, 'g');
    API::setColor(7, 7, 'g');
    maze.mouse_pos = (Coord){0, 0};     //setting the coordinate to 0,0 facing north
    maze.mouse_dir = NORTH;
    std::cerr << "Begin Maze Algorithm!\n" << std::endl;

    while(true){
        // Function declaration for getNeighborCells
        CellList* getNeighborCells(struct Maze* maze);

        checkWalls(&maze);
        mazeCenter(&maze);
        updateMouseSimulator(&maze);

        CellList* adjacentCells = getNeighborCells(&maze);
        std::cerr << adjacentCells->cells[0].pos.x << "," << adjacentCells->cells[0].pos.y << std::endl; //Checks north
        std::cerr << adjacentCells->cells[1].pos.x << "," << adjacentCells->cells[1].pos.y << std::endl; //Checks East
        std::cerr << adjacentCells->cells[2].pos.x << "," << adjacentCells->cells[2].pos.y << std::endl; //Checks South
        std::cerr << adjacentCells->cells[3].pos.x << "," << adjacentCells->cells[3].pos.y << std::endl; //Checks West

        free(adjacentCells->cells);
        free(adjacentCells);

        updateMousePos(&maze.mouse_pos, maze.mouse_dir);
        updateMouseSimulator(&maze);

    // Check if the mouse can move forward
        if (!API::wallFront()) {
            API::moveForward();
            updateMousePos(&maze.mouse_pos, maze.mouse_dir);
            updateMouseSimulator(&maze);
        } else {
            // If the mouse can't move forward, turn right
            API::turnRight();
            maze.mouse_dir = (Direction)((maze.mouse_dir + 1) % 4);

        }

        // Add a break condition to exit the loop
    } 

    return 0;
} 





// int main(int argc, char* argv[]) 
// {
//     setPerimeter(&maze);
//     mazeManhanDist(&maze);

//     while (true) {

//         scanWalls(&maze);

//         CellList* adjacentCells = getNeighborCells(&maze);
//         std::cerr << adjacentCells->cells[0].pos.x << "," << adjacentCells->cells[0].pos.y << std::endl; //Checks north
//         std::cerr << adjacentCells->cells[1].pos.x << "," << adjacentCells->cells[1].pos.y << std::endl; //Checks East
//         std::cerr << adjacentCells->cells[2].pos.x << "," << adjacentCells->cells[2].pos.y << std::endl; //Checks South
//         std::cerr << adjacentCells->cells[3].pos.x << "," << adjacentCells->cells[3].pos.y << std::endl; //Checks West

//         free(adjacentCells->cells);
//         free(adjacentCells);


//         updateSimulator(maze);

//         std::cerr << "(" << maze.mouse_pos.x << ", " << maze.mouse_pos.y << ")" << std::endl;

//         // Left Wall Follow Code
//         if (!API::wallLeft()) 
//         {
//             API::turnLeft();
//             maze.mouse_dir = (Direction)((maze.mouse_dir + 3) % 4);
//         }
//         while (API::wallFront()) 
//         {
//             API::turnRight();
//             maze.mouse_dir = (Direction)((maze.mouse_dir + 1) % 4);
//         }

//         API::moveForward();
//         updateMousePos(&maze.mouse_pos, maze.mouse_dir);
//     }
// }


void setPerimeter(struct Maze *maze){
    for(int y = 0; y < 16; y++) //iterate through rows from bottom left to to right
        {
            for(int x = 0; x < 16; x++)
            {
                if (y == 0) 
                {
                    maze->cellWalls[y][x] |= SOUTH_MASK;
                }

                if(y == 15)
                {
                    maze->cellWalls[y][x] |= NORTH_MASK;
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


//Pass through pointer maze storing data in memory located at struct Maze
void checkWalls(struct Maze *maze){
    if (API::wallFront())
        maze ->cellWalls[maze->mouse_pos.x][maze ->mouse_pos.y] |= NORTH_MASK;
    if (API::wallRight())
        maze ->cellWalls[maze->mouse_pos.x][maze ->mouse_pos.y] |= EAST_MASK;
    if (API::wallLeft())
        maze ->cellWalls[maze->mouse_pos.x][maze ->mouse_pos.y] |= WEST_MASK;
    
}
//Make center 4 cells zero
void mazeCenter(struct Maze *distanceMAZE)
{
    if (center == 1)
    {
          //This should assign a number describing how far away the specific cell is from the center
            for(int x = 0; x < 16; x++) 
        {
                for(int y = 0; y < 16; y++)
                {
                    if (x <= 7 && y <= 7){           //mouse is in third quadrant
                        distanceMAZE->distances[x][y] = ((7-x) + (7-y));
                    }
                    else if (x <= 7 && y>7){        //mouse is in 2nd quadrant 
                        distanceMAZE->distances[x][y] = ((7-x) + (8-y));
                    }
                    else if (x > 7 && y<=7){        //fourth quadrant
                        distanceMAZE->distances[x][y] = ((8-x) + (7-y));
                    }
                    else if (x > 7 && y>7){         //first quadrant
                        distanceMAZE->distances[x][y] = ((8-x) + (8-y));
                    }
                    else{
                        distanceMAZE->distances[x][y] = 0;
                    }
                }
        }

    }
}

void updateMousePos(Coord* pos, Direction dir){
    switch(dir){
        case NORTH:
            if (pos->y < 15) {
                pos->y++;
            } else {
                // Handle out-of-bounds error
            }
            break;
        case EAST:
            if (pos->x < 15) {
                pos->x++;
            } else {
                // Handle out-of-bounds error
            }
            break;
        case SOUTH:
            if (pos->y > 0) {
                pos->y--;
            } else {
                // Handle out-of-bounds error
            }
            break;
        case WEST:
            if (pos->x > 0) {
                pos->x--;
            } else {
                // Handle out-of-bounds error
            }
            break;
    }
}
