#ifndef GRAPH_H
#define GRAPH_H
// #include "red_fundamentals/Grid.h"
// #include "red_fundamentals/Row.h"
// #include "red_fundamentals/Cell.h"

#include <vector>
using namespace std;

enum nodeType {REGULAR = 0, GOLD = 1, PICKUP = 2};
typedef struct Node {
    nodeType type;
    vector<int> neighbors;
} node;

class Graph {
public:
    int num_row, num_col;
    node* nodes;
    vector<int> golds;
    vector<int> pickups;

    Graph(int n, int m);
    // void addEdges(red_fundamentals::Grid* map);
    void addEdges(vector<vector<vector<int>>> map) const;
    void setType();
    int postoidx(int row, int col) const;
    pair<int, int> idxtopos(int idx) const;
    static bool contains(const vector<int>& vec, int value);
    static void remove(vector<int>& vec, int value);
    void addEdge(int idx, int dir) const;
    bool is_adjacent(int v, int u) const;
    vector<int> bfs(int start, int goal) const;
};


#endif
