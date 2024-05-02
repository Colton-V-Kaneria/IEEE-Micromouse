#include <iostream>
#include <string>

//#include <queue>

#include "API.h"

#define MAX_COST 255

using namespace std;

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

    Coord goalPos[4];
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

/* helper function to determine if a cell is OOB due to coordinates */
bool OOBCheck(int x, int y) // returns 1 if OOB, 0 if not
{
    return !(0<=x && x<16 && 0<=y && y<16);
}

// 5. FILL THIS IN
void scanWalls(Maze* maze)
{
    int x = maze->mouse_pos.x;
    int y = maze->mouse_pos.y;

    int xA, yA;

    if (API::wallFront())
    {
        maze->cellWalls[y][x] |= dir_mask[maze->mouse_dir];                 // bitwise OR changes specific bit to specify wall detected
        switch ((int)(maze->mouse_dir))
        {
            case 0:
                xA = x;
                yA = y+1;
                break;
            case 1:
                xA = x+1;
                yA = y;
                break;
            case 2:
                xA = x;
                yA = y-1;
                break;
            case 3:
                xA = x-1;
                yA = y;
                break;
        }
        if (!OOBCheck(xA, yA))  // if the adjacent cell is not OOB, update its walls
        {
            cerr << "Blocked cell: (" << xA << ", " << yA << ")\n";
            maze->cellWalls[yA][xA] |= dir_mask[(maze->mouse_dir + 2) % 4];
        }
    }
    if (API::wallRight())
    {
        maze->cellWalls[y][x] |= dir_mask[(maze->mouse_dir + 1) % 4];
        switch ((int)(maze->mouse_dir))
        {
            case 3:
                xA = x;
                yA = y+1;
                break;
            case 0:
                xA = x+1;
                yA = y;
                break;
            case 1:
                xA = x;
                yA = y-1;
                break;
            case 2:
                xA = x-1;
                yA = y;
                break;
        }
        if (!OOBCheck(xA, yA))  // if the adjacent cell is not OOB, update its walls
        {
            cerr << "Blocked cell: (" << xA << ", " << yA << ")\n";
            maze->cellWalls[yA][xA] |= dir_mask[(maze->mouse_dir + 3) % 4];
        }
    }
    if (API::wallLeft())
    {
        maze->cellWalls[y][x] |= dir_mask[(maze->mouse_dir + 3) % 4];
        switch ((int)(maze->mouse_dir))
        {
            case 1:
                xA = x;
                yA = y+1;
                break;
            case 2:
                xA = x+1;
                yA = y;
                break;
            case 3:
                xA = x;
                yA = y-1;
                break;
            case 0:
                xA = x-1;
                yA = y;
                break;
        }
        if (!OOBCheck(xA, yA))  // if the adjacent cell is not OOB, update its walls
        {
            cerr << "Blocked cell: (" << xA << ", " << yA << ")\n";
            maze->cellWalls[yA][xA] |= dir_mask[(maze->mouse_dir + 1) % 4];
        }
    }
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
    std::cerr << "Inside updateMousePos: (" << pos->x << ", " << pos->y << ")" << std::endl;
}

// 6. FILL THIS IN
CellList* getNeighborCells(Maze* maze, Coord position)
{
    CellList* cellList = (CellList*)malloc(sizeof(CellList));

    // some logic to set the cell list size
    cellList->size = 4;
    cellList->cells = (Cell*)malloc(cellList->size * sizeof(Cell)); // allocates number of bits the struct requires

    int x = position.x;              // these are just to make it easier to type the mouse's coordinates below
    int y = position.y;

    // north cell
    cellList->cells[0].pos.x = x;
    cellList->cells[0].pos.y = y+1;
    cellList->cells[0].dir = NORTH;
    cellList->cells[0].blocked = (maze->cellWalls[y][x] & NORTH_MASK) || OOBCheck(x, y+1);
    // ^^cells are blocked if & with mask yields 0 OR the cell's coordinates are OOB

    // east cell
    cellList->cells[1].pos.x = x+1;
    cellList->cells[1].pos.y = y;
    cellList->cells[1].dir = EAST;
    cellList->cells[1].blocked = (maze->cellWalls[y][x] & EAST_MASK) || OOBCheck(x+1, y);
    
    // south cell
    cellList->cells[2].pos.x = x;
    cellList->cells[2].pos.y = y-1;
    cellList->cells[2].dir = SOUTH;
    cellList->cells[2].blocked = (maze->cellWalls[y][x] & SOUTH_MASK) || OOBCheck(x, y-1);
    
    // west cell
    cellList->cells[3].pos.x = x-1;
    cellList->cells[3].pos.y = y;
    cellList->cells[3].dir = WEST;
    cellList->cells[3].blocked = (maze->cellWalls[y][x] & WEST_MASK) || OOBCheck(x-1, y);
    
    return cellList;
}

Cell getBestCell(CellList* cellList, Maze* maze)
{
    Cell best_cell;
    Cell prospect;
    int min_dist = MAX_COST;
    int x_new, y_new;

    // Below is the only way we can check front, right, left, then behind in that order
    // We will increase this increment and increase the index i by this to get the directions we want
    int increment = 0;

    for (int i = (int)maze->mouse_dir; increment < 4; i+=increment) // once the increment hits 3, we are looking at the behind cell and should stop afterwards
    {
        prospect = cellList->cells[i%(cellList->size)];

        x_new = prospect.pos.x;
        y_new = prospect.pos.y;

        if (maze->distances[y_new][x_new] < min_dist && !prospect.blocked)
        {
            best_cell = prospect;
            min_dist = maze->distances[y_new][x_new]; // we need to change the minimum distance if we find one smaller
        }

        increment++;
    }
    return best_cell;
}

Direction clockwiseStep(Maze* maze)
{
    return Direction((maze->mouse_dir + 1) % 4);
}

Direction counterClockwiseStep(Maze* maze)
{
    return Direction((maze->mouse_dir + 3) % 4);
}

void rotate(Maze* maze, int cw)     // cw represents whether or not to turn clockwise
{
    if (cw)     // turn clockwise
    {
        API::turnRight();
        maze->mouse_dir = clockwiseStep(maze);      // update mouse direction in maze struct
    }
    else        // don't turn clockwise (turn counterclockwise)
    {
        API::turnLeft();
        maze->mouse_dir = counterClockwiseStep(maze);       // update mouse direction in maze struct
    }
}

void move(Maze* maze, Direction dest_dir)
{
    int rot_dir = !(counterClockwiseStep(maze) == dest_dir); // 0 if turning ccw once is easiest, 1 otherwise

    // rotate the mouse until its direction is correct
    while (maze->mouse_dir != dest_dir)
    {
       rotate(maze, rot_dir);
    }

    API::moveForward(); // move the mouse one space in the direction it is facing
}

void setGoalCells(Maze* maze, Coord goal_cells[4])
{
    //maze->goalPos = goal_cells;
    memcpy(maze->goalPos, goal_cells, sizeof(maze->goalPos));
}

// void showq(queue<Coord> gq)
// {
//     queue<Coord> g = gq;
//     cerr << "Current queue:\n";
//     while (!g.empty()) {
//         cerr << "\t(" << g.front().x << ", " << g.front().y << ")\n";
//         g.pop();
//     }
//     cerr << '\n';
// }

void Floodfill(Maze* maze)
{
    //queue initialization
    //queue<Coord> coord_queue;
    
    Coord queue[256];
    unsigned char head, tail = 0;
    
    for (int x = 0; x < 16; x++)
    {
        for (int y = 0; y < 16; y++)
        {
            maze->distances[y][x] = MAX_COST;
        }
    }

    // set the distance of the goal cell to 0
    for (int i = 0; i < sizeof(maze->goalPos) / sizeof(Coord); i++)
    {
        maze->distances[maze->goalPos[i].y][maze->goalPos[i].x] = 0;
        // push the goal cell into the queue
        //coord_queue.push(maze->goalPos[i]);
        queue[tail] = maze->goalPos[i];
        tail++;
    }

    while(head != tail)
    {
        //Coord cur_pos = coord_queue.front();
        Coord cur_pos = queue[head];
        //coord_queue.pop();
        head++;
        
        int newcost = maze->distances[cur_pos.y][cur_pos.x] + 1;
        
        CellList *neighborCells = getNeighborCells(maze, cur_pos);

        for (int i = 0; i < neighborCells->size; i++) // error is somewhere in here
        {
            Cell cell = neighborCells->cells[i];
            int x = cell.pos.x;
            int y = cell.pos.y;

            if (!cell.blocked)// && (0<=x && x<16) && (0<=y && y<16))  // removed because cells out of range will already be blocked
            {
                if (maze->distances[y][x] > newcost) // if distance > newcost
                {
                    maze->distances[y][x] = newcost;    // set newcost equal to this distance
                    
                    Coord new_coord;                // create a new coord to push into queue
                    new_coord.x = x;
                    new_coord.y = y;
                    
                    //coord_queue.push(new_coord);
                    queue[tail] = new_coord;
                    tail++;
                }
            }
        }
    }
}

int finished(Maze* maze)
{
    for (int i = 0; i < sizeof(maze->goalPos) / sizeof(Coord); i++)
    {
        if (maze->goalPos[i].x == maze->mouse_pos.x && maze->goalPos[i].y == maze->mouse_pos.y)
        {
            return 1; // if mouse pos matches a goal cell's coordinates, finished
        }
    }

    return 0; // if mouse pos did not match any goal cells, unfinished
}

void traverseMaze(Maze* maze)
{
    while (!finished(maze)) // run while the mouse position != goal position
    {
        cerr << "--------------------------------------------------------------------\n";
        cerr << "LOCATION: (" << maze->mouse_pos.x << ", " << maze->mouse_pos.y << ")\n";
        
        Floodfill(maze);
        
        scanWalls(maze);

        CellList* adjacentCells = getNeighborCells(maze, maze->mouse_pos);

        Cell best = getBestCell(adjacentCells, maze);      // initialize a cell to hold the result of getBestCell

        std::cerr << "Best Cell: (" << best.pos.x << ", " << best.pos.y << ")" << std::endl; // print out best cell

        free(adjacentCells->cells);
        free(adjacentCells);

        updateSimulator(*maze);

        move(maze, best.dir); // move the mouse in the direction of the best cell
    
        // 3. UPDATE THIS WITH POINTERS updateMousePos
        updateMousePos(&maze->mouse_pos, maze->mouse_dir);
        cerr << "\n";
    }
}

// Global variable declarations for main
Maze maze;

Coord goal_cells[4] = {(Coord){7, 7}, (Coord){7, 8}, (Coord){8, 8}, (Coord){8, 7}}; // goal cell coordinates in an array
Coord start_array[4] = {(Coord){0, 0}};

int main(int argc, char* argv[]) 
{
    maze.mouse_pos = (Coord){0, 0};
    maze.mouse_dir = NORTH;

    // hard code the first wall behind the mouse
    maze.distances[0][0] |= SOUTH_MASK;
    API::setWall(0, 0, 's');

    // Exploration phase
    setGoalCells(&maze, goal_cells);
    traverseMaze(&maze);

    // Return to start
    setGoalCells(&maze, start_array);
    traverseMaze(&maze);

    // Use fastest path to reach center
    setGoalCells(&maze, goal_cells);
    traverseMaze(&maze);
}


// Left Wall Follow Code from Kyle
// if (!API::wallLeft()) 
// {
//     API::turnLeft();
//     maze.mouse_dir = (Direction)((maze.mouse_dir + 3) % 4);
// }
// while (API::wallFront()) 
// {
//     API::turnRight();
//     maze.mouse_dir = (Direction)((maze.mouse_dir + 1) % 4);
// }
// API::moveForward();

// 5. MOVE TO updateSimulator() + scanWalls() FUNCTION                                      (????????????????? I don't know about this)
// if (API::wallFront())
//     API::setWall(maze.mouse_pos.x, maze.mouse_pos.y, dir_chars[maze.mouse_dir]);
// if (API::wallRight())
//     API::setWall(maze.mouse_pos.x, maze.mouse_pos.y, dir_chars[(maze.mouse_dir + 1) % 4]);
// if (API::wallLeft())
//     API::setWall(maze.mouse_pos.x, maze.mouse_pos.y, dir_chars[(maze.mouse_dir + 3) % 4]);