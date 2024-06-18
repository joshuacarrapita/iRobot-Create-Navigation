#include "graph.hpp"
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

using namespace std;
// for debugging
/* vector<vector<vector<int>>> map = {{{1,2,0}, {1,2}, {1,0}, {1,2}, {1,3}, {1,0,3}},
                                   {{2,0}, {2,3,0}, {2,0}, {2,3}, {1}, {1,0,3}},
                                   {{2,3}, {1}, {}, {1,0}, {2}, {1,0}},
                                   {{2,1,3}, {0,3}, {2}, {}, {3}, {0,3}},
                                   {{1,2}, {0,1}, {2,0}, {2}, {1,0}, {1,2,0,3}},
                                   {{0,2,3}, {2,3}, {0,3}, {2,3}, {3}, {1,0,3}}};

Graph graph = Graph(6, 6);
graph.addEdges(map);
graph.setType(); */

Graph::Graph(int n, int m) {
    num_row = n;
    num_col = m;
    nodes = new node[num_row * num_col];
    golds = {1, 11, 19, 28, 30};
    pickups = {6, 23};
}

/* void Graph::addEdges(red_fundamentals::Grid* map) {
    for(int row=0; row<num_row; row++){
        for(int col=0; col<num_col; col++){
            int idx = postoidx(row, col);
            nodes[idx].type = REGULAR;
            
            vector<int> walls = map->rows[row].cells[col].walls;
            // if you want to save the neighbors in sorted order
            // vector<int> directions = {1, 2, 0, 3};
            // for(int wall : directions){
            //    if(!contains(walls, wall)) addEdge(idx, wall);
            // } 
            for(int wall=0; wall<4; wall++){
                if(!contains(walls, wall)) addEdge(idx, wall);
            } 
        }
    }
} */

void Graph::addEdges(vector<vector<vector<int>>> map) const{
    for(int row=0; row<num_row; row++){
        for(int col=0; col<num_col; col++){
            int idx = postoidx(row, col);
            nodes[idx].type = REGULAR;

            vector<int> walls = map[row][col];
            // if you want to save the neighbors in sorted order
//            vector<int> directions = {1, 2, 0, 3};
//            for(int wall : directions){
//                if(!contains(walls, wall)) addEdge(idx, wall);
//            }
            for(int wall=0; wall<4; wall++){
                if(!contains(walls, wall)) addEdge(idx, wall);
            }
        }
    }
}

void Graph::setType(){
    for(int n : golds) nodes[n].type = GOLD;
    for(int n : pickups) nodes[n].type = PICKUP;
}

int Graph::postoidx(int row, int col) const {
    return row * num_col + col;
}

pair<int, int> Graph::idxtopos(int idx) const {
    return {idx / num_col, idx % num_col};
}

bool Graph::contains(const vector<int> &vec, int value) {
    return find(vec.begin(), vec.end(), value) != vec.end();
}

void Graph::remove(vector<int> &vec, int value) {
    vec.erase(std::remove(vec.begin(), vec.end(), value), vec.end());
}

void Graph::addEdge(int idx, int dir) const {
    switch(dir){
        case 0:
            nodes[idx].neighbors.push_back(idx + 1);
            break;
        case 1:
            nodes[idx].neighbors.push_back(idx - num_col);
            break;
        case 2:
            nodes[idx].neighbors.push_back(idx - 1);
            break;
        case 3:
            nodes[idx].neighbors.push_back(idx + num_col);
            break;
        default:
            break;
    }
}

bool Graph::is_adjacent(int v, int u) const {
    return contains(nodes[v].neighbors, u);
}

vector<int> Graph::bfs(int start, int goal) const {
    // check if start and goal positions are valid (inside the map)
    int start_row = start / num_col;
    int start_col = start % num_col;
    int goal_row = goal / num_col;
    int goal_col = goal % num_col;

    if(start_row < 0 || start_row >= num_row || start_col < 0 || start_col >= num_col ||
       goal_row < 0 || goal_row >= num_row || goal_col < 0 || goal_col >= num_col) {
        cout << "Invalid start or end position!" << endl;
        return {};
    }

    if(nodes[start].neighbors.empty() || nodes[goal].neighbors.empty()){
        cout << "Not possible to reach goal!" << endl;
        return {};
    }

    unordered_map<int, int> parent;
    vector<int> visited;    // nodes we visited
    queue<int> q;           // Queue for breadth-first search (Warteschlange)

    visited.push_back(start);   // add the start node to the visited nodes
    q.push(start);              // add the start node to the queue
    parent[start] = -1;         //start node does not have any parents

    while(!q.empty()) {         // as long as the queue is not empty
        int current_node = q.front();   // remove first node from the queue and set it to current node
        q.pop();

        if(current_node == goal) {  // if we reached our goal we can go out of the while loop
            break;
        }

        for(int neighbor : nodes[current_node].neighbors) { // iterate through all neighbors of the current node
            if(parent.find(neighbor) == parent.end()) { // check if the neighbor has not been visited before
                parent[neighbor] = current_node;            // set the parent of the neighbor to the current node
                visited.push_back(neighbor);            // mark the neighbor as visited
                q.push(neighbor);                       // add the neighbor to the queue for further exploration
            }
        }
    }

    // reconstruct the path from start to goal
    vector<int> path;
    for(int at = goal; at != -1; at = parent[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());

    // if the goal node is not reachable from the start node return an empty path
    if(path[0] != start) {
        return {};
    }

    return path;
}


vector<int> Graph::findPathVisitingAllGold(int start) {
    vector<int> best_path;
    int best_distance = 100; 

    sort(golds.begin(), golds.end());   // In case the gold is not sorted in the constructor
    
    // going through all the different permutations of golds
    while (next_permutation(golds.begin(), golds.end())){
        
        for (int pickup : pickups) {
            vector<int> current_path;
            int current_distance = 0;
            int current_start = start;

            // Go through all gold nodes and "connect" them on a path
            for (int gold : golds) {
                // find a path from the current_start to the next gold
                vector<int> path_segment = bfs(current_start, gold);
                // if there is no path possible the high current_distance is meant to prevent this current_path to be chosen as the best one
                if (path_segment.empty()) {    
                    current_distance = 100;   
                    break;
                }
                // add length and path_segment to the current_path
                current_distance += path_segment.size() - 1; 
                current_path.insert(current_path.end(), path_segment.begin(), path_segment.end() - 1);
                current_start = gold; // current_start is now where the path_segment endet (at the next gold)
            }

            // Go to the pickup node
            vector<int> path_segment = bfs(current_start, pickup);
             // if there is no path possible the high current_distance is meant to prevent this current_path to be chosen as the best one
            if (path_segment.empty()) {
                current_distance = 100;
                break;
            }
            // add length and path_segment to the current_path
            current_distance += path_segment.size() - 1;
            current_path.insert(current_path.end(), path_segment.begin(), path_segment.end());

            // Check if we found a shorter path
            if (current_distance < best_distance) {
                best_distance = current_distance;
                best_path = current_path;
            }
        }
    };

    return best_path;
}

int main(int argc, char **argv){
    vector<vector<vector<int>>> map = {{{1,2,0}, {1,2}, {1,0}, {1,2}, {1,3}, {1,0,3}},
                                   {{2,0}, {2,3,0}, {2,0}, {2,3}, {1}, {1,0,3}},
                                   {{2,3}, {1}, {}, {1,0}, {2}, {1,0}},
                                   {{2,1,3}, {0,3}, {2}, {}, {3}, {0,3}},
                                   {{1,2}, {0,1}, {2,0}, {2}, {1,0}, {1,2,0,3}},
                                   {{0,2,3}, {2,3}, {0,3}, {2,3}, {3}, {1,0,3}}};

    Graph graph = Graph(6, 6);
    graph.addEdges(map);
    graph.setType();

    // Testing findPathVisitingAllGold()

    int start = 21;

    vector<int> path = graph.findPathVisitingAllGold(start);

    // output the path
    if(path.empty()) {
        cout << "No path found" << endl;
    } else {
        cout << "Path from " << start << ": ";
        for(int idx : path) {
            if (find(graph.golds.begin(), graph.golds.end(), idx) != graph.golds.end()) {
                cout << "(" << idx << ") ";
            } else {
                cout << idx << " ";
            }
        }
        cout << endl;
    }

    cout << "Total number of nodes visited: " << path.size() << endl;

    //Testing BFS

    /* int start = graph.postoidx(0, 2);
    int goal = graph.postoidx(5, 2); 

    vector<int> path = graph.bfs(start, goal);

    // output the path
    if(path.empty()) {
        cout << "No path found" << endl;
    } else {
        cout << "Path from " << start << " to " << goal << ": ";
        for(int idx : path) {
            pair<int, int> pos = graph.idxtopos(idx);
            cout << "(" << pos.first << ", " << pos.second << ") ";
        }
        cout << endl;
    }

    // output the path
    if(path.empty()) {
        cout << "No path found" << endl;
    } else {
        cout << "Path from " << start << " to " << goal << ": ";
        for(int idx : path) {
            cout << idx << " ";
        }
        cout << endl;
    } */



}

/*vector<int> Graph::findPathVisitingAllGold1(int start) {
    vector<int> allNodes = { start };
    allNodes.insert(allNodes.end(), golds.begin(), golds.end());
    allNodes.insert(allNodes.end(), pickups.begin(), pickups.end());

    int n = allNodes.size();
    vector<vector<int>> dist(n, vector<int>(n, INT_MAX));

    // BFS to calculate shortest paths between all pairs of nodes
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            vector<int> path = bfs(allNodes[i], allNodes[j]);
            if (!path.empty()) {
                dist[i][j] = dist[j][i] = path.size() - 1;
            }
        }
    }

    // DP array to store the shortest path to visit all gold nodes
    vector<vector<int>> dp(1 << golds.size(), vector<int>(golds.size(), INT_MAX));
    vector<vector<int>> parent(1 << golds.size(), vector<int>(golds.size(), -1));

    for (int i = 0; i < golds.size(); ++i) {
        dp[1 << i][i] = dist[0][i + 1];
    }

    for (int mask = 0; mask < (1 << golds.size()); ++mask) {
        for (int i = 0; i < golds.size(); ++i) {
            if (mask & (1 << i)) {
                for (int j = 0; j < golds.size(); ++j) {
                    if (!(mask & (1 << j))) {
                        int nextMask = mask | (1 << j);
                        if (dp[nextMask][j] > dp[mask][i] + dist[i + 1][j + 1]) {
                            dp[nextMask][j] = dp[mask][i] + dist[i + 1][j + 1];
                            parent[nextMask][j] = i;
                        }
                    }
                }
            }
        }
    }

    // Find the best end node and construct the optimal path
    int minDist = INT_MAX;
    int lastNode = -1;
    int finalMask = (1 << golds.size()) - 1;

    for (int i = 0; i < golds.size(); ++i) {
        if (dp[finalMask][i] < minDist) {
            minDist = dp[finalMask][i];
            lastNode = i;
        }
    }

    vector<int> optimalGoldPath;
    int mask = finalMask;
    while (lastNode != -1) {
        optimalGoldPath.push_back(lastNode + 1);
        int temp = parent[mask][lastNode];
        mask ^= (1 << lastNode);
        lastNode = temp;
    }
    reverse(optimalGoldPath.begin(), optimalGoldPath.end());

    // Find the nearest pickup node from the last gold node visited
    int lastGoldNode = optimalGoldPath.back();
    int nearestPickupNode = -1;
    int minPickupDist = INT_MAX;

    for (int i = 0; i < pickups.size(); ++i) {
        if (dist[lastGoldNode + 1][golds.size() + 1 + i] < minPickupDist) {
            minPickupDist = dist[lastGoldNode + 1][golds.size() + 1 + i];
            nearestPickupNode = i;
        }
    }

    // Construct the final path including the path to the nearest pickup node
    vector<int> fullPath = { start };
    int currentNode = start;

    for (int idx : optimalGoldPath) {
        vector<int> segment = bfs(currentNode, allNodes[idx]);
        if (!segment.empty()) {
            fullPath.insert(fullPath.end(), segment.begin() + 1, segment.end());
            currentNode = allNodes[idx];
        }
    }

    vector<int> pickupSegment = bfs(currentNode, allNodes[golds.size() + 1 + nearestPickupNode]);
    if (!pickupSegment.empty()) {
        fullPath.insert(fullPath.end(), pickupSegment.begin() + 1, pickupSegment.end());
    }

    return fullPath;
}
*/



