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

Coord queue[256];
// Declaring a coord struct that        
// will act as our queue
//FIFO-first coordinate enqueued is the first to be dequeued in the queue
int head = 0; //Start of the queue
int tail = 0; //End of the queue
int MAX_COST= 255; 

void floodfill(Maze* maze)
{
    

for (int i = 0; i < 16; i++) //Setting every cell to arbitrary large number
{   
    for(int j = 0; j < 16; j++)
    {
        maze->distances[j][i] = MAX_COST;
    }

}

for (int u = 7; u < 9; u++)
{
    for(int v = 7; v < 9; v++)
    {
        maze->distances[v][u] = 0; //We set the goal cells to zero since this is how far 
    }                     //away the cell is from the goal aka 0

}


for () //We add the goal cells to the queue given that this is where floodfill begins
{

    Coord goalCell1;
    goalCell1.x = 7;
    goalCell1.y =7;

    queue.push(goalCell1);
    tail++;

}



while (tail - head > 0)
{
    Coord cur_pos = queue[head]; //we set the current possition to the head of the queue aka a goal cell
    head++;
    int newcost = distances[pos.y][pos.x] + 1; 
    //increments the distance of the current position cell by one

    CellList* neighborCells = getNeighborCells(cur_pos);

    for(cell in neighborCells)
    {
        if(cell is not blocked)
        {
            if(distances[cell.y][cell.x] > newcost) //checks distance value of neighbor adjacent cells
                                                    // if its greater than current newcost. 
            {
                distances[cell.y][cell.x] = newcost; //sets the new distance to newcost distance
                queue[tail] = cell 
                tail++
            }
        }
    }

}


}