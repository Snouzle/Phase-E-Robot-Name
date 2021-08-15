#include "PathPlanner.hpp"

// USE THIS TO CONVERT ROW & COL to the cell number 
int cell_conversion(int row, int col) { 
    return (row*ROW+col); 
}

// All the webots classes are defined in the "webots" namespace 
// HELPER FUNCTIONS used: 
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

// new_pos = 999 and obstacle = 999 for initial read 
// if obstacle - put in new position in new_pos 
std::string getPath(int new_pos, int obstacle, char heading, const std::string &map_file, 
                    const std::vector<int> &endpoints) { 

    // initialise variables 
    int adj_matrix[TOTAL_CELLS][TOTAL_CELLS]{0};                     // adjancey matrix  
    std::vector<std::string> txt_line;                // Map strings 
    int start;                                        // start position 
    int end;                                          // end position 
    char direction = heading;                                   // start direction 
    std::string start_pos;                             // string to hold starting position e.g. '00S' 
    const std::string MAP_FILE = map_file;
    
    std::string motion_string;
    std::fstream txtfile;
    txtfile.open(MAP_FILE);

    if (txtfile.is_open()){ 
            while(getline(txtfile, motion_string)){ 
                // Print the string in the txtfile into console 
                std::cout << motion_string << "\n";
                txt_line.insert(txt_line.end(), motion_string); 
            }
            txtfile.close(); 
    }

    // Reading the map from the textfile and converting it into a adjancey matrix 
    int curr_x; 
    int curr_y; 
    for (int i = 0; i < 5; i++) {
        // iterate through a column
        for (int j = 0; j < 9; j++) { 
            // "set" the correct value to search through the string 
            curr_y = j*4 + 1; 
            curr_x = i*2 + 1;
      
            // the only row that can't check one row below. 
            if (i != 4) { 
                // check if a wall exists in the row below the cell - prohibits going South 
                if (txt_line[curr_x + 1][curr_y] != '-') { 
                    adj_matrix[i*ROW+j][i*ROW+j+ROW] = 1;  
                } 
            } 
            // the only row that can't check one row above. 
            if (i != 0) { 
                // check if wall exists in the row above the cell - prohibits going North 
                if (txt_line[curr_x - 1][curr_y] != '-') { 
                    adj_matrix[i*ROW+j][i*ROW+j-ROW] = 1; 
                } 
            } 
            // the only column that can't check the left cell 
            if (j != 0) { 
                // check if wall exists in the column left of the cell - prohibits going West 
                if (txt_line[curr_x][curr_y - 1] != '|') { 
                    adj_matrix[i*ROW+j][i*ROW+j-1] = 1; 
                } 
            } 
            // the only column that can't check the right cell 
            if (j != 8) { 
                // check if wall exists in the column right of the cell - prohibits going East 
                if (txt_line[curr_x][curr_y + 3] != '|') { 
                    adj_matrix[i*ROW+j][i*ROW+j+1] = 1; 
                } 
            } 
     
    
        }   
    }


    // Find the start 'cell' - direction, and destination 'cell' 
    for (int i = 0; i < (int)txt_line.size(); i++) { 
        // Note: find() returns string::npos is letter is not found 
        std::size_t south = txt_line[i].find('v'); 
        std::size_t north = txt_line[i].find('^'); 
        std::size_t west = txt_line[i].find('<');     
        std::size_t east = txt_line[i].find('>'); 
        std::size_t dest = txt_line[i].find('x'); 
        if (direction == ' ') {
            if (south != std::string::npos) { 
                start = (i/2)*ROW + south/4; 
                direction = 'S'; 
            } else if (north != std::string::npos) { 
                start = (i/2)*ROW + north/4;
                direction = 'N';  
            } else if (west != std::string::npos) { 
                start = (i/2)*ROW + west/4;
                direction = 'W';  
            } else if (east != std::string::npos) { 
                start = (i/2)*ROW + east/4;
                direction = 'E';  
            }
        } else if (dest != std::string::npos) { 
            end = (i/2)*ROW + dest/4; 
        } 
    }


    if (new_pos != -1) { 
        start = new_pos; 
    } 
    if (obstacle != -1) {
        // All directions from the obstacle cell cant be accessed   
        adj_matrix[obstacle][obstacle+1] = 0; 
        adj_matrix[obstacle+1][obstacle] = 0; 
        adj_matrix[obstacle][obstacle-1] = 0; 
        adj_matrix[obstacle-1][obstacle] = 0; 
        adj_matrix[obstacle][obstacle+9] = 0; 
        adj_matrix[obstacle+9][obstacle] = 0; 
        adj_matrix[obstacle][obstacle-9] = 0; 
        adj_matrix[obstacle-9][obstacle] = 0; 
    }

    // Save the starting position as a string - to add at the front of the path later
    curr_x = start / ROW; 
    curr_y = start % ROW; 

    start_pos.append(std::to_string(curr_x)); 
    start_pos.append(std::to_string(curr_y)); 
    std::string s(1, direction);
    start_pos.append(s); 

    // Implementing BFS search on the adjency matrix 
    std::queue<int> q; 
    q.push(start); 

    std::vector<std::vector<int> > paths;
    std::vector<int> path;
    std::vector<int> parent[TOTAL_CELLS];
    parent[start] = {-1}; 
    std::vector<int> dist(TOTAL_CELLS, INT_MAX);
    dist[start] = 0;

    // BFS 
    while(!q.empty()) { 
        int cell = q.front(); //pop_front()?
        q.pop();
        for (int i = 0; i < TOTAL_CELLS; i++) {        // Go through all cells 
            if (adj_matrix[cell][i] == 1) {                // Check if the cells are adjacent from adjacent matrix  
                if (dist[i] > dist[cell] + 1) {              // Check if a shorter distance is possible 
                    // if shorter distance is found, new parent is created for this 
                    dist[i] = dist[cell] + 1;
                    q.push(i); 
                    parent[i].clear(); 
                    parent[i].push_back(cell);   
            
                } else if (dist[i] == dist[cell] + 1) {      // Check if a similar path of same length can be found       
                    parent[i].push_back(cell); 
                }
            }    
        
        } 
    }

    if (!endpoints.empty()) {
        int minDistance = dist[endpoints[0]];
        int minPoint = endpoints[0];
        for (const int &point : endpoints) {
            if (dist[point] < minDistance) {
                minDistance = dist[point];
                minPoint = point;
            }
        }
        end = minPoint;
    }

    // Now to find all paths 
    find_paths(paths, path, parent, end); 
    // Print all possible paths into the given MAP format 
    const int path_size = paths.size(); 
    std::vector<std::string> map_routes[path_size]; 
    std::vector<std::string> plan[path_size]; 
    std::vector<int> turns{path_size, 0};  

    char curr_direction = direction; 
    char new_direction; 

    for (int p = 0; p < (int)paths.size(); p++) { 
        // First, copy and paste the entire MAP into the Map route 
        for (int t = 0; t < (int)txt_line.size(); t++) { 
            map_routes[p].insert(map_routes[p].end(), txt_line[t]);  
        }

        // Begin the path generated for this shortest sequence 
        plan[p].insert(plan[p].end(), start_pos); 
        // Find the initial tile 
        int prev = paths[p][paths[p].size() - 1]; 
        char curr_direction = direction;

        for (int i = 0; i < (int)(paths[p].size() - 1); i++) { 
            // Calculate the positions (x,y) and edit into Map route 
            curr_x = paths[p][i] / ROW; 
            curr_y = paths[p][i] % ROW; 
            // Navigate to the right row 
            int ones = i % 10; 
            int tenths = i / 10; 
            char one = '0' + ones;      // convert into char 
            char tenth = '0' + tenths; 
            if (tenths > 0) { 
                map_routes[p][(curr_x*2)+1][(curr_y*4)+2] = tenth; 
                map_routes[p][(curr_x*2)+1][(curr_y*4)+3] = one;
            } else { 
                map_routes[p][(curr_x*2)+1][(curr_y*4)+2] = one; 
            } 

            // if curr heading direction 
            new_direction = left_or_right(plan, p, paths[p][paths[p].size() -i -2], prev, curr_direction); 
            prev = paths[p][paths[p].size() -2 -i];
            if (new_direction != curr_direction) { 
                if (new_direction == 'U') { 
                    turns[p] = turns[p] + 2;
                    curr_direction = opp_direction(curr_direction);  
                } else { 
                    turns[p] = turns[p] + 1; 
                    curr_direction = new_direction;
                }
            } 
      
        }
        
    }

    // Print out to terminal here 
    for (int p = 0; p < path_size; p++) { 
        // First, copy and paste the entire MAP into the Map route 
        std::cout << "Path - " << p+1 << ":" << "\n"; 
        for (int t = 0; t < (int)txt_line.size(); t++) { 
            std::cout << map_routes[p][t] << "\n"; 
        }
    }  
    std::cout << paths.size() << " shortest paths found!" << "\n"; 
    std::cout << "Finding shortest path with least turns..." << "\n"; 

    // Find the smallest number of turns recorded in the turns array 
    int min = 0; 
    for (int i = 1; i < path_size; i++) { 
        if (turns[i] < turns[min]) { 
            min = i; 
        } 
    } 
    
    // Print the map of the shortest path 
    for (int t = 0; t < (int)txt_line.size(); t++) { 
        std::cout << map_routes[min][t] << "\n"; 
    }

    std::string final_plan = ""; 
    // Print path into a string 
    for (int i = 0; i < (int)plan[min].size(); i++) { 
        final_plan += plan[min][i];
    } 

    // Return the shortest path as a string 
    return final_plan; 
}