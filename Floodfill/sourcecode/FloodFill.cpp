/**
 * Project: UCLA IEEE Micromouse (STM32)
 * Names: Mervin, Peter, Colton
 * Description: floodfill.cpp
*/

//PsuedoCode for FloodFill algor
//Priority list!!!
//Creating the queue properly
//Updating the correct cell 
//Creating GetNeighbor cells
//figure out how to check if cell is blocked


#include "FloodFill_DataStructures.h"
#include "API.h"

Coord queue[255];
// Declaring a coord struct that        
// will act as our queue
//FIFO-first coordinate enqueued is the first to be dequeued in the queue
int head = 0; //Start of the queue
int tail = 0; //End of the queue


int MAX_COST= 255; 

for (all distance cells) //Setting every sell to arbitrary large number
{
    distances[y][x] = MAX_COST;
}

for (all goal cells)
{
    distances[y][x] = 0; //We set the goal cells to zero since this is how far 
                         //away the cell is from the goal aka 0

}


for (all goal cells) //We add the goal cells to the queue given that this is veyr floodfill begins
{

    queue.add(goal cell position)
    tail++
}

while (tail - head > 0)
{
    cur_pos = queue[head]; //we set the current possition to the head of the queue aka a goal cell
    head++;
    newcost = distances[pos.y][pos.x] + 1; 
    //increments the distance of the current position cell by one

    neighborCells = getNeighborCells(cur_pos);

    for(cell in neighborCells)
    {
        if(cell is not blocked)
        {
            if(distances[cell.y][cell.x] > newcost)
            {
                distances[cell.y][cell.x] = newcost;
                tail++
            }
        }
    }

}