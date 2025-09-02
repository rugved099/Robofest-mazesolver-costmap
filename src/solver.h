#ifndef SOLVER_H
#define SOLVER_H

typedef enum Heading { NORTH, EAST, SOUTH, WEST } Heading;
typedef enum Action  { LEFT, FORWARD, RIGHT, IDLE } Action;
typedef enum Mode    { EXPLORATION, RETURN_HOME, OPTIMIZED_OUT } Mode;

#define MAZE_SIZE 16

/* Cell encoding (Top Right Bottom Left) */
#define _0000 0
#define _0001 1
#define _0010 2
#define _0011 3
#define _0100 4
#define _0101 5
#define _0110 6
#define _0111 7
#define _1000 8
#define _1001 9
#define _1010 10
#define _1011 11
#define _1100 12
#define _1101 13
#define _1110 14
#define _1111 15

extern unsigned int maze[MAZE_SIZE][MAZE_SIZE];
extern int costs[MAZE_SIZE][MAZE_SIZE];

struct Coordinate {
    int x;
    int y;
};

void initialize();

/* sensing + map */
void updateMaze();

/* costmap */
void resetCostsToCenter();
void resetCostsToStart();
void updateCostsToCenter();
void updateCostsToStart();

/* helpers */
int xyToSquare(int x, int y);
struct Coordinate squareToCoord(int square);
int isWallInDirection(int x, int y, Heading direction);
void updateHeading(Action nextAction);
void updatePosition(Action nextAction);

/* navigation */
Action costmapNavigate();             // greedy next-step using current costs
Action solver();                      // orchestrates phases

/* path building for optimized run */
void buildShortestPathToStart(int fromX, int fromY);  // builds path buffer to START
void buildShortestPathToCenter(int fromX, int fromY); // builds path buffer to CENTER
Action nextActionFromPath();                           // pops next precomputed action
int atCenter(int x, int y);
int atStart(int x, int y);

#endif
