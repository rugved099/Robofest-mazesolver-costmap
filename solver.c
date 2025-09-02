#include <stdio.h>
#include <stdlib.h>
#include "solver.h"
#include "API.h"
#include "queue.h"

unsigned int maze[MAZE_SIZE][MAZE_SIZE] = {0};
int costs[MAZE_SIZE][MAZE_SIZE];

struct Coordinate position;
Heading heading;
int reached_exit = 0;

// path buffer for optimized run
#define PATH_MAX 1024
Action pathBuf[PATH_MAX];
int pathLen = 0;
int pathIdx = 0;

// use Mode from solver.h
Mode currentMode = EXPLORATION;

/* ---------- Utility ---------- */
int inBounds(int x, int y) {
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

struct Coordinate squareToCoord(int square) {
    struct Coordinate c;
    c.x = square % MAZE_SIZE;
    c.y = square / MAZE_SIZE;
    return c;
}

int xyToSquare(int x, int y) {
    return y * MAZE_SIZE + x;
}

/* ---------- Reset Costs ---------- */
void resetCosts() {
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            costs[x][y] = -1;
        }
    }

    // ---- Exit-based goal (border cells) ----
    for (int i = 0; i < MAZE_SIZE; i++) {
        costs[i][0] = 0;                   // bottom row
        costs[i][MAZE_SIZE-1] = 0;         // top row
        costs[0][i] = 0;                   // left column
        costs[MAZE_SIZE-1][i] = 0;         // right column
    }
}

/* ---------- Initialize ---------- */
void initialize() {
    for (int i = 1; i < MAZE_SIZE - 1; i++) {
        maze[0][i] = _0001;
        maze[i][0] = _0010;
        maze[i][MAZE_SIZE - 1] = _1000;
        maze[MAZE_SIZE - 1][i] = _0100;
    }
    maze[0][0] = _0011;
    maze[0][MAZE_SIZE - 1] = _1001;
    maze[MAZE_SIZE - 1][0] = _0110;
    maze[MAZE_SIZE - 1][MAZE_SIZE - 1] = _1100;

    resetCosts();
    position.x = 0;
    position.y = 0;
    heading = NORTH;
}

/* ---------- Maze Update ---------- */
void updateMaze() {
    if (API_wallFront()) {
        if (heading == NORTH) { maze[position.x][position.y] |= _1000; API_setWall(position.x, position.y, 'n'); }
        if (heading == EAST)  { maze[position.x][position.y] |= _0100; API_setWall(position.x, position.y, 'e'); }
        if (heading == SOUTH) { maze[position.x][position.y] |= _0010; API_setWall(position.x, position.y, 's'); }
        if (heading == WEST)  { maze[position.x][position.y] |= _0001; API_setWall(position.x, position.y, 'w'); }
    }
    if (API_wallLeft()) {
        if (heading == NORTH) { maze[position.x][position.y] |= _0001; API_setWall(position.x, position.y, 'w'); }
        if (heading == EAST)  { maze[position.x][position.y] |= _1000; API_setWall(position.x, position.y, 'n'); }
        if (heading == SOUTH) { maze[position.x][position.y] |= _0100; API_setWall(position.x, position.y, 'e'); }
        if (heading == WEST)  { maze[position.x][position.y] |= _0010; API_setWall(position.x, position.y, 's'); }
    }
    if (API_wallRight()) {
        if (heading == NORTH) { maze[position.x][position.y] |= _0100; API_setWall(position.x, position.y, 'e'); }
        if (heading == EAST)  { maze[position.x][position.y] |= _0010; API_setWall(position.x, position.y, 's'); }
        if (heading == SOUTH) { maze[position.x][position.y] |= _0001; API_setWall(position.x, position.y, 'w'); }
        if (heading == WEST)  { maze[position.x][position.y] |= _1000; API_setWall(position.x, position.y, 'n'); }
    }
}

/* ---------- Wall Checker ---------- */
int isWallInDirection(int x, int y, Heading direction) {
    switch (direction) {
        case NORTH: return (maze[x][y] & _1000) ? 1 : 0;
        case EAST:  return (maze[x][y] & _0100) ? 1 : 0;
        case SOUTH: return (maze[x][y] & _0010) ? 1 : 0;
        case WEST:  return (maze[x][y] & _0001) ? 1 : 0;
    }
    return 0;
}

/* ---------- BFS Costmap ---------- */
void updateCosts() {
    resetCosts();
    queue q = queue_create();

    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            if (costs[x][y] == 0) queue_push(q, xyToSquare(x, y));
        }
    }

    while (!queue_is_empty(q)) {
        struct Coordinate c = squareToCoord(queue_pop(q));
        int x = c.x, y = c.y;

        if (!isWallInDirection(x, y, NORTH) && inBounds(x, y+1) && costs[x][y+1] == -1) {
            costs[x][y+1] = costs[x][y] + 1;
            queue_push(q, xyToSquare(x, y+1));
        }
        if (!isWallInDirection(x, y, EAST) && inBounds(x+1, y) && costs[x+1][y] == -1) {
            costs[x+1][y] = costs[x][y] + 1;
            queue_push(q, xyToSquare(x+1, y));
        }
        if (!isWallInDirection(x, y, SOUTH) && inBounds(x, y-1) && costs[x][y-1] == -1) {
            costs[x][y-1] = costs[x][y] + 1;
            queue_push(q, xyToSquare(x, y-1));
        }
        if (!isWallInDirection(x, y, WEST) && inBounds(x-1, y) && costs[x-1][y] == -1) {
            costs[x-1][y] = costs[x][y] + 1;
            queue_push(q, xyToSquare(x-1, y));
        }
    }

    queue_destroy(q);
}

/* ---------- Path Backtracking ---------- */
static void buildPathByBacktracking(int fromX, int fromY) {
    pathLen = 0; pathIdx = 0;
    int cx = fromX, cy = fromY;
    if (!inBounds(cx, cy) || costs[cx][cy] < 0) return;

    Heading h = heading;
    while (costs[cx][cy] != 0 && pathLen < PATH_MAX) {
        int cur = costs[cx][cy];
        int nx = cx, ny = cy;
        Heading nh = h;
        Action act = IDLE;
        int picked = 0;

        // same neighbor-selection logic as before
        if (h == NORTH) {
            if (!isWallInDirection(cx, cy, NORTH) && inBounds(cx, cy+1) && costs[cx][cy+1] == cur-1) {
                nx=cx; ny=cy+1; act=FORWARD; nh=NORTH; picked=1;
            } else if (!isWallInDirection(cx, cy, EAST) && inBounds(cx+1, cy) && costs[cx+1][cy]==cur-1) {
                nx=cx+1; ny=cy; act=RIGHT; nh=EAST; picked=1;
            } else if (!isWallInDirection(cx, cy, WEST) && inBounds(cx-1, cy) && costs[cx-1][cy]==cur-1) {
                nx=cx-1; ny=cy; act=LEFT; nh=WEST; picked=1;
            }
        }
        else if (h == EAST) {
            if (!isWallInDirection(cx, cy, EAST) && inBounds(cx+1, cy) && costs[cx+1][cy]==cur-1) {
                nx=cx+1; ny=cy; act=FORWARD; nh=EAST; picked=1;
            } else if (!isWallInDirection(cx, cy, SOUTH) && inBounds(cx, cy-1) && costs[cx][cy-1]==cur-1) {
                nx=cx; ny=cy-1; act=RIGHT; nh=SOUTH; picked=1;
            } else if (!isWallInDirection(cx, cy, NORTH) && inBounds(cx, cy+1) && costs[cx][cy+1]==cur-1) {
                nx=cx; ny=cy+1; act=LEFT; nh=NORTH; picked=1;
            }
        }
        else if (h == SOUTH) {
            if (!isWallInDirection(cx, cy, SOUTH) && inBounds(cx, cy-1) && costs[cx][cy-1]==cur-1) {
                nx=cx; ny=cy-1; act=FORWARD; nh=SOUTH; picked=1;
            } else if (!isWallInDirection(cx, cy, WEST) && inBounds(cx-1, cy) && costs[cx-1][cy]==cur-1) {
                nx=cx-1; ny=cy; act=RIGHT; nh=WEST; picked=1;
            } else if (!isWallInDirection(cx, cy, EAST) && inBounds(cx+1, cy) && costs[cx+1][cy]==cur-1) {
                nx=cx+1; ny=cy; act=LEFT; nh=EAST; picked=1;
            }
        }
        else if (h == WEST) {
            if (!isWallInDirection(cx, cy, WEST) && inBounds(cx-1, cy) && costs[cx-1][cy]==cur-1) {
                nx=cx-1; ny=cy; act=FORWARD; nh=WEST; picked=1;
            } else if (!isWallInDirection(cx, cy, NORTH) && inBounds(cx, cy+1) && costs[cx][cy+1]==cur-1) {
                nx=cx; ny=cy+1; act=RIGHT; nh=NORTH; picked=1;
            } else if (!isWallInDirection(cx, cy, SOUTH) && inBounds(cx, cy-1) && costs[cx][cy-1]==cur-1) {
                nx=cx; ny=cy-1; act=LEFT; nh=SOUTH; picked=1;
            }
        }

        if (!picked) break;

        if (act == LEFT)  pathBuf[pathLen++] = LEFT;
        if (act == RIGHT) pathBuf[pathLen++] = RIGHT;
        pathBuf[pathLen++] = FORWARD;

        h = nh; cx = nx; cy = ny;
    }
}

/* ---------- Navigation ---------- */
Action costmapNavigate() {
    int best = 999;
    Action move = IDLE;
    if (!isWallInDirection(position.x, position.y, NORTH) && costs[position.x][position.y+1] < best) {
        best = costs[position.x][position.y+1]; move = FORWARD;
    }
    if (!isWallInDirection(position.x, position.y, EAST) && costs[position.x+1][position.y] < best) {
        best = costs[position.x+1][position.y]; move = RIGHT;
    }
    if (!isWallInDirection(position.x, position.y, WEST) && costs[position.x-1][position.y] < best) {
        best = costs[position.x-1][position.y]; move = LEFT;
    }
    if (!isWallInDirection(position.x, position.y, SOUTH) && costs[position.x][position.y-1] < best) {
        best = costs[position.x][position.y-1]; move = FORWARD;
    }
    if (best == 999) move = RIGHT;
    return move;
}

void updateHeading(Action act) {
    if (act == LEFT) heading = (heading + 3) % 4;
    else if (act == RIGHT) heading = (heading + 1) % 4;
}

void updatePosition(Action act) {
    if (act == FORWARD) {
        if (heading == NORTH) position.y++;
        else if (heading == EAST) position.x++;
        else if (heading == SOUTH) position.y--;
        else if (heading == WEST) position.x--;
    }
}

/* ---------- Solver ---------- */
Action solver() {
    if (currentMode == EXPLORATION) {
        if (!reached_exit && costs[position.x][position.y] == 0) {
            reached_exit = 1;
            currentMode = OPTIMIZED_OUT;
            buildPathByBacktracking(position.x, position.y);
            return IDLE;
        }
        updateMaze();
        updateCosts();
        Action act = costmapNavigate();
        updateHeading(act);
        updatePosition(act);
        return act;
    } else if (currentMode == OPTIMIZED_OUT) {
        if (pathIdx < pathLen) {
            Action act = pathBuf[pathIdx++];
            updateHeading(act);
            updatePosition(act);
            return act;
        } else {
            return IDLE; // done
        }
    }
    return IDLE;
}
