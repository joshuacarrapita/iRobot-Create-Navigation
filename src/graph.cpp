#include "graph.hpp"
#include <algorithm>
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




