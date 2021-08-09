#include <webots/Robot.hpp>
#include <fstream>
#include <vector> 
#include <string> 
#include <iostream>
#include <queue>
#include <algorithm>
#include "PathPlanner.hpp" 

#define ROW 9
#define TOTAL_CELLS 45
// All the webots classes are defined in the "webots" namespace



// Helper Functions used: 

// Find the paths from BFS done 
void find_paths(std::vector<std::vector<int> >& paths, std::vector<int>& path, std::vector<int> parent[], int curr_cell) { 
  // Base Case 
    if (curr_cell == -1) { 
        paths.push_back(path); 
        return; 
    } 

    for (int p :parent[curr_cell]) { 
        // insert current cell in path 
        path.push_back(curr_cell); 
    
        // recursive call for its parent 
        find_paths(paths, path, parent, p); 
    
        // remove the current cell 
        path.pop_back(); 
    
    } 
} 

// Find if Left or Right turn was made 
char left_or_right(std::vector<std::string> route[], int p, int next_tile, int curr_tile, char curr_direction) { 
    std::string left = "L"; 
    std::string right = "R"; 
    std::string turn = "LL"; 
    char new_direction; 
  
  // If direction is facing south 
    if (curr_direction == 'S') { 
        if (next_tile == curr_tile + 1) { 
            route[p].insert(route[p].end(), left); 
            new_direction = 'E';  
        } else if (next_tile == curr_tile - 1) { 
            route[p].insert(route[p].end(), right); 
            new_direction = 'W';  
        } else if (next_tile < curr_tile) {              // if the next tile is directly behind the direction its facing now, edge case 
            route[p].insert(route[p].end(), turn);  
            new_direction = 'U'; 
        } else { 
            new_direction = 'S'; 
        } 
    } else if (curr_direction == 'E') { 
        if (next_tile == curr_tile - 1) { 
            route[p].insert(route[p].end(), turn);            // if the next tile is directly behind the direction its facing now, edge case 
            new_direction = 'U';  
        } else if (next_tile < curr_tile) { 
            route[p].insert(route[p].end(), left); 
            new_direction = 'N';  
        } else if (next_tile != curr_tile + 1) {               
            route[p].insert(route[p].end(), right); 
            new_direction = 'S'; 
        } else { 
            new_direction = 'E'; 
        } 
    } else if (curr_direction == 'W') { 
        if (next_tile == curr_tile + 1) { 
            route[p].insert(route[p].end(), turn);            // if the next tile is directly behind the direction its facing now, edge case 
            new_direction = 'U';  
        } else if (next_tile < curr_tile - 2) { 
            route[p].insert(route[p].end(), right); 
            new_direction = 'N';  
        } else if (next_tile > (curr_tile + 2)) {               
            route[p].insert(route[p].end(), left); 
            new_direction = 'S'; 
        } else { 
            new_direction = 'W'; 
        } 
    } else { 
        if (next_tile == curr_tile + 1) { 
            route[p].insert(route[p].end(), right); 
            new_direction = 'E';  
        } else if (next_tile == curr_tile - 1) { 
            route[p].insert(route[p].end(), left); 
            new_direction = 'W';  
        } else if (next_tile > curr_tile) {              // if the next tile is directly behind the direction its facing now, edge case 
            route[p].insert(route[p].end(), turn); 
            new_direction = 'U'; 
        } else { 
            new_direction = 'N'; 
        } 
    }
  
  // Move forward by one 
    route[p].insert(route[p].end(), "F"); 
  
    return new_direction; 
} 

// returns opposite direction 
char opp_direction (char curr) { 
    if (curr == 'N') { 
        return 'S'; 
    } else if (curr == 'E') { 
        return 'W'; 
    } else if (curr == 'W') { 
        return 'E'; 
    } else { 
        return 'N'; 
    } 
} 

// TODO:: remove this? 
PathPlanner::PathPlanner(): {}

PathPlanner::getPath() { 

    // initialise variables 
    int adj_matrix[TOTAL_CELLS][TOTAL_CELLS] = {0};                     // adjancey matrix  
    std::vector<std::string> txt_line;                // Map strings 
    int start;                                        // start position 
    int end;                                          // end position 
    char direction;                                   // start direction 
    std::string start_pos;                             // string to hold starting position e.g. '00S' 

    // Open Output.txt as well 
    std::ofstream outfile; 
    outfile.open(OUTPUT_FILE, std::ofstream::out); // | std::ofstream::app
    
    std::string motion_string;
    std::fstream txtfile;
    txtfile.open(MAP_FILE);


}




