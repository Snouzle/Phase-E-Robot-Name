#ifndef PATHP_H
#define PATHP_H

#include <fstream>
#include <vector> 
#include <string> 
#include <iostream>
#include <queue>
#include <algorithm>

#define ROW 9
#define TOTAL_CELLS 45

const std::string MAP_PATH = "../../MapBuilt.txt";

int cell_conversion(int row, int col);

// All the webots classes are defined in the "webots" namespace 
// HELPER FUNCTIONS used: 
// Find the paths from BFS done 
void find_paths(std::vector<std::vector<int> >& paths, std::vector<int>& path, std::vector<int> parent[], int curr_cell);

// Find if Left or Right turn was made 
char left_or_right(std::vector<std::string> route[], int p, int next_tile, int curr_tile, char curr_direction);

// returns opposite direction 
char opp_direction (char curr);

// new_pos = 999 and obstacle = 999 for initial read 
// if obstacle - put in new position in new_pos 
std::string getPath(int new_pos=-1, int obstacle=-1, char heading=' ', const std::string &map_file=MAP_PATH, 
                    const std::vector<int> &endpoints={});

#endif