
#include <stdio.h>
#include "API.h"

// Define orientations
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// Function to log debug messages
void log(int number) {
    fprintf(stderr, "%d\n", number);  // Use %d to print an integer
    fflush(stderr);
}
/*void log(const int* message) {
    fprintf(stderr, "%d\n", message);
    fflush(stderr);
}*/

// Move the robot forward based on its current orientation
void moveForward(int* x, int* y, int direction) {
    API_moveForward();
    if (direction == NORTH) (*y)--;
    else if (direction == EAST) (*x)++;
    else if (direction == SOUTH) (*y)++;
    else if (direction == WEST) (*x)--;
}

// Turn the robot to the right and update orientation
void turnRight(int* direction) {
    API_turnRight();
    *direction = (*direction + 1) % 4;
}

// Turn the robot to the left and update orientation
void turnLeft(int* direction) {
    API_turnLeft();
    *direction = (*direction + 3) % 4;
}

// Turn the robot around (180 degrees) and update orientation
void turnAround(int* direction) {
    API_turnRight();
    API_turnRight();
    *direction = (*direction + 2) % 4;
}

// Check if the cell in front is accessible and is the next in the sequence
int isForwardCellValid(int x, int y, int direction, int maze[16][16], int currentValue) {
    if (direction == NORTH && y > 0 && maze[y - 1][x] == currentValue - 1) return 1;
    if (direction == EAST && x < 15 && maze[y][x + 1] == currentValue - 1) return 1;
    if (direction == SOUTH && y < 15 && maze[y + 1][x] == currentValue - 1) return 1;
    if (direction == WEST && x > 0 && maze[y][x - 1] == currentValue - 1) return 1;
    return 0;
}

// Check if the right cell is accessible and is the next in the sequence
int isRightCellValid(int x, int y, int direction, int maze[16][16], int currentValue) {
    int newDir = (direction + 1) % 4;
    return isForwardCellValid(x, y, newDir, maze, currentValue);
}

// Check if the left cell is accessible and is the next in the sequence
int isLeftCellValid(int x, int y, int direction, int maze[16][16], int currentValue) {
    int newDir = (direction + 3) % 4;
    return isForwardCellValid(x, y, newDir, maze, currentValue);
}

int main(int argc, char* argv[]) {
    API_setColor(0, 0, 'V');
    API_setText(0, 0, "20");

    // Maze definition
    int maze[16][16] = {
        {31,30,29,28,27,26,25,24,25,24,23,22,21,20,21,22},
        {32,33,48,49,50,51,26,23,22,19,18,23,22,19,20,23},
        {33,34,47,50,51,52,25,24,21,20,17,16,17,18,19,24},
        {34,35,46,45,46,53,24,23,22,19,18,15,16,17,18,25},
        {35,36,37,44,55,54,23,22,21,20,13,14,15,16,17,26},
        {36,37,38,43,42,55, 8, 9,10,11,12,13,14,15,28,27},
        {39,38,39,40,41, 6, 7, 8, 9,10,13,12,13,30,29,28},
        {40,39,40,47,50, 5, 4, 0, 0,11,10,11,14,31,30,31},
        {41,40,41,46,49, 4, 3, 0, 0, 8, 9,12,37,36,33,32},
        {58,41,42,45,48, 3, 2, 1, 6, 7,10,39,38,35,34,33},
        {57,44,43,44,47,48, 3, 4, 5, 8,41,40,39,36,35,34},
        {56,57,58,45,46,49,52,53,46,45,42,41,38,37,36,35},
        {55,56,57,48,47,50,51,52,47,44,43,40,39,38,37,36},
        {54,55,50,49,48,51,50,51,48,47,42,41,40,41,38,37},
        {53,52,51,52,49,50,49,48,49,46,43,44,41,40,39,38},
        {54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,41}
    };

    int x = 0;      // Starting x position
    int y = 15;     // Starting y position
    int direction = NORTH;  // Start facing North

    while (maze[y][x] != 0) {
        int currentValue = maze[y][x];

        // Prioritize moving forward if the next cell is available
        if (!API_wallFront() && isForwardCellValid(x, y, direction, maze, currentValue)) {
            moveForward(&x, &y, direction);
            log(maze[y][x]);
        }
        // If not, try to turn right if the right cell is the next in the sequence
        else if (!API_wallRight() && isRightCellValid(x, y, direction, maze, currentValue)) {
            turnRight(&direction);
            moveForward(&x, &y, direction);
            log(maze[y][x]);
        }
        // If not, try to turn left if the left cell is the next in the sequence
        else if (!API_wallLeft() && isLeftCellValid(x, y, direction, maze, currentValue)) {
            turnLeft(&direction);
            moveForward(&x, &y, direction);
            log(maze[y][x]);
        }
        // If all sides are blocked, turn around
        else {
            turnAround(&direction);
            moveForward(&x, &y, direction);
            log(maze[y][x]);
        }
    }
    
    
    return 0;
}

