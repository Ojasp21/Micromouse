#pragma once

// Function to get the maze width
int API_mazeWidth();

// Function to get the maze height
int API_mazeHeight();

// Function to check if there's a wall in front
int API_wallFront();

// Function to check if there's a wall to the right
int API_wallRight();

// Function to check if there's a wall to the left
int API_wallLeft();

// Function to move the robot forward; returns 0 if crash, else returns 1
int API_moveForward();

// Function to turn the robot right
void API_turnRight();

// Function to turn the robot left
void API_turnLeft();

// Function to set a wall in the maze
void API_setWall(int x, int y, char direction);

// Function to clear a wall from the maze
void API_clearWall(int x, int y, char direction);

// Function to set color at a specific coordinate
void API_setColor(int x, int y, char color);

// Function to clear color at a specific coordinate
void API_clearColor(int x, int y);

// Function to clear all colors in the maze
void API_clearAllColor();

// Function to set text at a specific coordinate
void API_setText(int x, int y, char* str);

// Function to clear text at a specific coordinate
void API_clearText(int x, int y);

// Function to clear all text in the maze
void API_clearAllText();

// Function to check if the robot was reset
int API_wasReset();

// Function to acknowledge a reset
void API_ackReset();
