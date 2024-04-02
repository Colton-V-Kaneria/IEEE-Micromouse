#include <iostream>
#include <string>

#include <queue>

#include "API.h"

#define MAX_COST 255

using namespace std;

//These are test goal cell coordinates
int Xg = 8;
int Yg = 8;

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

    Coord goalPos;
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
        maze->cellWalls[maze->mouse_pos.y][maze->mouse_pos.x] |= dir_mask[maze->mouse_dir];                 // bitwise OR changes specific bit to specify wall detected
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
CellList* getNeighborCells(Maze* maze, Coord position)
{
    CellList* cellList = (CellList*)malloc(sizeof(CellList));
    int i = 0;

    // some logic to set the cell list size
    cellList->size = 4;
    cellList->cells = (Cell*)malloc(cellList->size * sizeof(Cell)); // allocates number of bits the struct requires

    int x = position.x;              // these are just to make it easier to type the mouse's coordinates below
    int y = position.y;

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

Cell getBestCell(CellList* cellList, Maze* maze)
{
    Cell best_cell;
    Cell prospect;
    int min_dist = 255;
    int x_new, y_new;

    for (int i = 0; i < cellList->size; i++)
    {
        prospect = cellList->cells[i];

        x_new = prospect.pos.x;
        y_new = prospect.pos.y;

        if (maze->distances[y_new][x_new] < min_dist && !prospect.blocked)
        {
            best_cell = prospect;
        }
    }
    return best_cell;
}

Direction clockwiseStep(Maze* maze)
{
    API::turnRight();
    return Direction((maze->mouse_dir + 1) % 4);
}

Direction counterClockwiseStep(Maze* maze)
{
    API::turnLeft();
    return Direction((maze->mouse_dir + 3) % 4);
}

void setGoalCell(Maze* maze, int Xg, int Yg)
{
    Coord coord;
    coord.x = Xg;
    coord.y = Yg;

    maze->goalPos = coord;
}

// void showq(queue<Coord> gq)
// {
//     queue<Coord> g = gq;
//     while (!g.empty()) {
//         cerr << "\t(" << g.front().x << ", " << g.front().y << ")";
//         g.pop();
//     }
//     cerr << '\n';
// }

Coord queue[255];
int queue_count, head, tail = 0;

bool Queue_Full()
{
    return queue_count == 255;
}

bool Queue_Empty()
{
    return queue_count == 0;
}

void Queue_Push(Coord data)
{
    if(!Queue_Full())
    {
        Coord queue[head];
        queue[head].x = data.x;
        queue[head].y = data.y;
        queue_count++; 

    }
}



void Floodfill(Maze* maze)
{
    //queue initialization
    //queue<Coord> coord_queue;
    Coord queue[255];
    for (int x = 0; x < 16; x++)
    {
        for (int y = 0; y < 16; y++)
        {
            maze->distances[y][x] = MAX_COST;
        }
    }

    for (int x = 7; x < 8; x++)
    {
        for (int y = 7; y < 8; y++)
        {
            maze->distances[y][x] = 0;
            Queue_Push(maze->goalPos);
            tail++; 

        }
    }

    while(tail - head > 0)
    {
        Coord cur_pos;
        cur_pos.x = queue[head].x;


        head++;
        int newcost = maze->distances[cur_pos.y][cur_pos.x] + 1;

        CellList *neighborCells = getNeighborCells(maze, cur_pos);

        for (int i = 0; i < 4; i++)
        {
            Cell cell = neighborCells->cells[i];
            int x = cell.pos.x;
            int y = cell.pos.y;

            if ((cell.blocked == false) && (0<=x && x<16) && (0<=y && y<16))
            {
                if (maze->distances[y][x] > newcost)
                {
                    maze->distances[y][x] = newcost;
                    
                    Coord queue[tail];
                    queue[tail].x = x;
                    queue[tail].y = y;

                    // Coord new_coord;
                    // new_coord.x = x;
                    // new_coord.y = y;
                    // coord_queue.push(new_coord);
                    //queue.push(queue[tail]);

                    tail++; 

                }
            }
        }



    }

    cerr << "Goal Cell: (" << maze->goalPos.x << ", " << maze->goalPos.y << ")\n";
    //queue.push(maze->goalPos);

    //maze->distances[maze->goalPos.y][maze->goalPos.x] = 0;

    while(!coord_queue.empty())
    {
        Coord cur_pos = coord_queue.front();
        coord_queue.pop();
        
        int newcost = maze->distances[cur_pos.y][cur_pos.x] + 1;

        CellList *neighborCells = getNeighborCells(maze, cur_pos);

        for (int i = 0; i < 4; i++)
        {
            Cell cell = neighborCells->cells[i];
            int x = cell.pos.x;
            int y = cell.pos.y;

            if ((cell.blocked == false) && (0<=x && x<16) && (0<=y && y<16))
            {
                if (maze->distances[y][x] > newcost)
                {
                    maze->distances[y][x] = newcost;
                    
                    Coord new_coord;
                    new_coord.x = x;
                    new_coord.y = y;

                    coord_queue.push(new_coord);
                }
            }
        }
    }
}

Maze maze;

int temp_value = 20;

int main(int argc, char* argv[]) 
{
    maze.mouse_pos = (Coord){0, 0};
    maze.mouse_dir = NORTH;

    setGoalCell(&maze, Xg, Yg);


    while (true) {
        Floodfill(&maze);
        
        scanWalls(&maze);
        CellList* adjacentCells = getNeighborCells(&maze, maze.mouse_pos);

        std::cerr << "NEIGHBOR CELLS" << std::endl;
        
        std::cerr << "At (" << maze.mouse_pos.x << ", " << maze.mouse_pos.y << ")" << std::endl;

        for (int i = 0; i < adjacentCells->size; i++)
        {
            std::cerr << "(" << adjacentCells->cells[i].pos.x << ", " << adjacentCells->cells[i].pos.y << ")\t blocked: " << adjacentCells->cells[i].blocked << std::endl;
        }

        Cell best = getBestCell(adjacentCells, &maze);

        std::cerr << "Best Cell: (" << best.pos.x << ", " << best.pos.y << ")" << std::endl;

        free(adjacentCells->cells);
        free(adjacentCells);

        updateSimulator(maze);
        

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
    
        updateMousePos(&maze.mouse_pos, maze.mouse_dir);

        if (API::wallFront())
            API::setWall(maze.mouse_pos.x, maze.mouse_pos.y, dir_chars[maze.mouse_dir]);
        if (API::wallRight())
            API::setWall(maze.mouse_pos.x, maze.mouse_pos.y, dir_chars[(maze.mouse_dir + 1) % 4]);
        if (API::wallLeft())
            API::setWall(maze.mouse_pos.x, maze.mouse_pos.y, dir_chars[(maze.mouse_dir + 3) % 4]);
    }
}