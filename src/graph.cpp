#include "graph.hpp"
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <iostream>
#include <climits>
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

    vector<int> path; 
    vector<int> visited(num_row * num_col, 0); //visited nodes
    int golds_to_visit = 5; // number of gold nodes to visit

    // BFS to find shortest path
    queue<pair<int, vector<int>>> q; // queue to store current position and path taken
    q.push({start, {start}});
    visited[start] = 1;

    while (!q.empty()) {
        int current = q.front().first;
        vector<int> current_path = q.front().second;
        q.pop();

        // check if we have visited all gold nodes
        int golds_visited = 0;
        for (int gold : golds) {
            if (visited[gold]) {
                golds_visited++;
            }
        }

        // If all gold nodes are visited, we need to end at a pickup node
        if (golds_visited == golds_to_visit) {
            // find nearest pickup node
            for (int pickup : pickups) {
                if (visited[pickup]) {
                    // found a path that visits all golds and ends at a pickup
                    path = current_path;
                    path.push_back(pickup);
                    return path;
                }
            }
        }

        // explore neighbors
        for (int neighbor : nodes[current].neighbors) {
            if (!visited[neighbor]) {
                visited[neighbor] = 1;
                vector<int> next_path = current_path;
                next_path.push_back(neighbor);
                q.push({neighbor, next_path});
            }
        }
    }

    // no valid path found: return empty path
    return {};
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

    int start = graph.postoidx(0, 2);

    vector<int> path = graph.findPathVisitingAllGold(start);

    // output the path
    if(path.empty()) {
        cout << "No path found" << endl;
    } else {
        cout << "Path from " << start << ": ";
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
        cout << "Path from " << start << ": ";
        for(int idx : path) {
            cout << idx << " ";
        }
        cout << endl;
    }


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



