#include <stdio.h>
#include "solver.h"
#include "API.h"

int main(int argc, char* argv[]) {
    debug_log("Starting...");
    initialize();

    while (1) {
        Action next = solver();

        // Show costs for debugging/visualization
        for (int x = 0; x < MAZE_SIZE; ++x) {
            for (int y = 0; y < MAZE_SIZE; ++y) {
                API_setText(x, y, costs[x][y]);
            }
        }

        // Execute chosen action
        switch (next) {
            case FORWARD: API_moveForward(); break;
            case LEFT:    API_turnLeft();    break;
            case RIGHT:   API_turnRight();   break;
            case IDLE:    /* do nothing this tick */ break;
        }
    }
    return 0;
}
