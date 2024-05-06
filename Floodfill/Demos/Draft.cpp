#include <iostream> //input and output stream "cin" "cout"
#include <string>   //provide class and functions for working string "std::string"

#include "API.h" //preprocesor that includes content to classes, constants, and functions
#include "FloodFill.h"

Direction dir = NORTH; //declare variable under Direction type under ununeration

//pass argument argc and argv as a pointer variable(array of strings)
int main(int argc, char* argv[]) {
    API::setColor(0, 0, 'g');   //(0,0), green

    API::setText(0, 0, "start");    //(0,0), start

    API::setWall(0, 0, 'w');    //set a visible wall at (0,0) west
    API::setWall(0, 0, 'e');    
    API::setWall(0, 0, 's');

    while (true) {
        // "std::cerr" for printing error messages, "<<" for chain output, "std::end1" inserts newline character
        std::cerr << dir << "(" << x << ", " << y << ")" << std::endl;

        //Check if there is no wall on the left.
        if (!API::wallLeft()) 
        {
            API::turnLeft();
            //update current direction after turning left from NORTH to WEST
            dir = (Direction)((dir + 3) % 4);
        }
        //turn right while a wall is in front
        while (API::wallFront()) 
        {
            API::turnRight();
            //update current direction after turning right from NORTH to EAST
            dir = (Direction)((dir + 1) % 4);
        }
        API::moveForward();
    
        //update coordinates depending on direction +1 unit
        if (dir == NORTH)
            y++;
        if (dir == SOUTH)
            y--;
        if (dir == WEST)
            x--;
        if (dir == EAST)
            x++;

        //Set the walls in the direction the mouse just moved  
        if (API::wallFront())
            API::setWall(x, y, dir_chars[dir]);
        if (API::wallRight())
            API::setWall(x, y, dir_chars[(dir + 1) % 4]);
        if (API::wallLeft())
            API::setWall(x, y, dir_chars[(dir + 3) % 4]);
    }
}

//MAIN TEST COMMENT HELLO PEOPLE
//TEST NUMBER 2 HELLO AGAIN

// OK
// Is this changed?
