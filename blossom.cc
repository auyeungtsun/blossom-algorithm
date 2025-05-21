#include <bits/stdc++.h>
using namespace std;

const int MAXN = 500; // Maximum number of vertices

class BlossomAlgorithm {
private:
    int n; // Number of vertices in the graph
    vector<int> graph[MAXN]; // Adjacency list representation of the graph
    /**
     * match[i]: The vertex to which vertex i is matched, or -1 if unmatched.
     * parent[i]: The parent of vertex i in the alternating tree.
     * base[i]: The base vertex of the blossom containing vertex i.
     * used[i]: Whether vertex i has been visited during the current search.
     * blossom[i]: Whether vertex i is part of a newly found blossom.
     */
    int match[MAXN], parent[MAXN], base[MAXN];
    bool used[MAXN], blossom[MAXN];
    queue<int> q; // Queue for Breadth First Search

    /**
     * @brief Finds the lowest common ancestor (LCA) of two vertices in the alternating tree.
     * 
     * @param a The first vertex.
     * @param b The second vertex.
     * @return The LCA of vertices a and b, or -1 if no LCA is found.
     */
    int lca(int a, int b) {
        vector<bool> visited(n, false);
        while (true) {
            a = base[a];
            visited[a] = true;
            if (match[a] == -1) break;
            a = parent[match[a]];
        }
        while (true) {
            b = base[b];
            if (visited[b]) return b;
            if (match[b] == -1) break;
            b = parent[match[b]];
        }
        return -1;
    }

    /**
     * @brief Marks the path from vertex v to the base b, identifying and flagging vertices within a blossom.
     * 
     * @param v The starting vertex of the path.
     * @param b The base vertex of the blossom, where the path terminates.
     * @param children The next vertex in the alternating tree.
     */
    void mark_path(int v, int b, int children) {
        while (base[v] != b) {
            blossom[base[v]] = blossom[base[match[v]]] = true;
            parent[v] = children;
            children = match[v];
            v = parent[match[v]];
        }
    }

    /**
     * @brief Finds an augmenting path in the graph starting from the given root vertex.
     *
     * @param root The starting vertex for the search.
     * @return The endpoint of the augmenting path, or -1 if no such path is found.
     */
    int find_path(int root) {
        fill(used, used + n, false);
        fill(parent, parent + n, -1);
        iota(base, base + n, 0);
        q = queue<int>();
        q.push(root);
        used[root] = true;

        while (!q.empty()) {
            int v = q.front();
            q.pop();
            for (int u : graph[v]) {
                if (base[v] == base[u] || match[v] == u) continue;
                if (u == root || (match[u] != -1 && parent[match[u]] != -1)) {
                    int curbase = lca(v, u);
                    fill(blossom, blossom + n, false);
                    mark_path(v, curbase, u);
                    mark_path(u, curbase, v);
                    for (int i = 0; i < n; ++i) {
                        if (blossom[base[i]]) {
                            base[i] = curbase;
                            if (!used[i]) {
                                used[i] = true;
                                q.push(i);
                            }
                        }
                    }
                } else if (parent[u] == -1) {
                    parent[u] = v;
                    if (match[u] == -1)
                        return u;
                    u = match[u];
                    used[u] = true;
                    q.push(u);
                }
            }
        }
        return -1;
    }
    
public:
    /**
     * @brief Constructor for the BlossomAlgorithm class.
     * 
     * @param vertices The number of vertices in the graph.
     */
    BlossomAlgorithm(int vertices) : n(vertices) {}
    
    /**
     * @brief Adds an undirected edge between vertices u and v.
     * @param u The first vertex.
     * @param v The second vertex.
     */
    void addEdge(int u, int v) {
        graph[u].push_back(v);
        graph[v].push_back(u);
    }

    /**
     * @brief Finds a maximum cardinality matching in the graph using the Blossom Algorithm.
     *
     * If the BFS encounters an edge (v, u) where v is an even-level vertex and u is an unmatched vertex, then the path from root to u forms an augmenting path.
     * A blossom is detected when an edge (v, u) connects two vertices v and u that are already in the alternating tree and are both at even levels relative to the root.
     * match[u] has a parent in the tree meaning match[u] is odd level, therefore u is even level.
     * mark_path ensures the path reconstruction can still work correctly by following the newly set parent pointers inside the cycle structure.
     * 
     * @return A vector of pairs representing the edges in the maximum matching.
     * 
     * @note Time Complexity: O(V^4) in the worst case, where V is the number of vertices. 
     *       In practice, it often performs much better.
     * @note Space Complexity: O(V^2) due to the adjacency list representation of the graph and other auxiliary arrays.
     */
    vector<pair<int, int>> findMaximumMatching() {
        fill(match, match + n, -1);

        for (int i = 0; i < n; ++i) {
            if (match[i] == -1) {
                int v = find_path(i);
                if (v != -1) {
                    while (v != -1) {
                        int pv = parent[v], ppv = match[pv];
                        match[v] = pv;
                        match[pv] = v;
                        v = ppv;
                    }
                }
            }
        }
        
        vector<pair<int, int>> result;
        for (int i = 0; i < n; i++) {
            if (i < match[i]) {
                result.push_back({i, match[i]});
            }
        }
        
        return result;
    }
};

void testBlossomAlgorithm() {
    cout << "Starting Blossom Algorithm Tests..." << endl;

    // Test Case 1: Empty Graph (0 vertices)
    {
        cout << "  Running Test Case 1: Empty Graph (n=0)..." << endl;
        BlossomAlgorithm algo(0);
        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.empty());
        cout << "  Test Case 1 Passed!" << endl;
    }

    // Test Case 2: Single Vertex (1 vertex)
    {
        cout << "  Running Test Case 2: Single Vertex (n=1)..." << endl;
        BlossomAlgorithm algo(1);
        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.empty());
         cout << "  Test Case 2 Passed!" << endl;
    }

    // Test Case 3: Two Vertices, No Edge
    {
         cout << "  Running Test Case 3: Two Vertices, No Edge (n=2)..." << endl;
        BlossomAlgorithm algo(2);
        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.empty());
         cout << "  Test Case 3 Passed!" << endl;
    }

    // Test Case 4: Two Vertices, One Edge (Path P2)
    {
        cout << "  Running Test Case 4: Path P2 (n=2)..." << endl;
        BlossomAlgorithm algo(2);
        algo.addEdge(0, 1);
        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 1);
        assert(matching[0] == make_pair(0, 1));
         cout << "  Test Case 4 Passed!" << endl;
    }

    // Test Case 5: Three Vertices, Path P3
    {
        cout << "  Running Test Case 5: Path P3 (n=3)..." << endl;
        BlossomAlgorithm algo(3);
        algo.addEdge(0, 1);
        algo.addEdge(1, 2);
        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 1);
        bool correct_match = (matching[0] == make_pair(0, 1)) || (matching[0] == make_pair(1, 2));
        assert(correct_match);
         cout << "  Test Case 5 Passed!" << endl;
    }

    // Test Case 6: Four Vertices, Path P4
    {
         cout << "  Running Test Case 6: Path P4 (n=4)..." << endl;
        BlossomAlgorithm algo(4);
        algo.addEdge(0, 1);
        algo.addEdge(1, 2);
        algo.addEdge(2, 3);
        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 2);
        set<pair<int, int>> expected = {{0, 1}, {2, 3}};
        set<pair<int, int>> result_set(matching.begin(), matching.end());
        assert(result_set == expected);
         cout << "  Test Case 6 Passed!" << endl;
    }

    // Test Case 7: Three Vertices, Cycle C3 (Triangle - Blossom)
    {
        cout << "  Running Test Case 7: Cycle C3 (n=3)..." << endl;
        BlossomAlgorithm algo(3);
        algo.addEdge(0, 1);
        algo.addEdge(1, 2);
        algo.addEdge(2, 0);
        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 1);
        bool correct_match = (matching[0] == make_pair(0, 1)) ||
                             (matching[0] == make_pair(1, 2)) ||
                             (matching[0] == make_pair(0, 2));
        assert(correct_match);
         cout << "  Test Case 7 Passed!" << endl;
    }

    // Test Case 8: Four Vertices, Cycle C4 (Square)
    {
        cout << "  Running Test Case 8: Cycle C4 (n=4)..." << endl;
        BlossomAlgorithm algo(4);
        algo.addEdge(0, 1);
        algo.addEdge(1, 2);
        algo.addEdge(2, 3);
        algo.addEdge(3, 0);
        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 2);
         set<pair<int, int>> result_set(matching.begin(), matching.end());
         set<pair<int, int>> expected1 = {{0, 1}, {2, 3}};
         set<pair<int, int>> expected2 = {{0, 3}, {1, 2}};
        assert(result_set == expected1 || result_set == expected2);
         cout << "  Test Case 8 Passed!" << endl;
    }

    // Test Case 9: Five Vertices, Cycle C5 (Pentagon - Blossom)
    {
        cout << "  Running Test Case 9: Cycle C5 (n=5)..." << endl;
        BlossomAlgorithm algo(5);
        algo.addEdge(0, 1);
        algo.addEdge(1, 2);
        algo.addEdge(2, 3);
        algo.addEdge(3, 4);
        algo.addEdge(4, 0);
        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 2);
         cout << "  Test Case 9 Passed!" << endl;
    }

    // Test Case 10: Complete Graph K4
    {
        cout << "  Running Test Case 10: Complete Graph K4 (n=4)..." << endl;
        BlossomAlgorithm algo(4);
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                algo.addEdge(i, j);
            }
        }
        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 2);
         cout << "  Test Case 10 Passed!" << endl;
    }

    // Test Case 11: Complete Graph K5
    {
        cout << "  Running Test Case 11: Complete Graph K5 (n=5)..." << endl;
        BlossomAlgorithm algo(5);
        for (int i = 0; i < 5; ++i) {
            for (int j = i + 1; j < 5; ++j) {
                algo.addEdge(i, j);
            }
        }
        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 2);
         cout << "  Test Case 11 Passed!" << endl;
    }

     // Test Case 12: Petersen Graph (Famous non-bipartite, 10 vertices, perfect matching size 5)
    {
        cout << "  Running Test Case 12: Petersen Graph (n=10)..." << endl;
        BlossomAlgorithm algo(10);
        algo.addEdge(0, 1); algo.addEdge(1, 2); algo.addEdge(2, 3); algo.addEdge(3, 4); algo.addEdge(4, 0);
        algo.addEdge(0, 5); algo.addEdge(1, 6); algo.addEdge(2, 7); algo.addEdge(3, 8); algo.addEdge(4, 9);
        algo.addEdge(5, 7); algo.addEdge(7, 9); algo.addEdge(9, 6); algo.addEdge(6, 8); algo.addEdge(8, 5);

        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 5);
        cout << "  Test Case 12 Passed!" << endl;
    }


    // Test Case 13: Graph needing blossom contraction (e.g., C3 + path)
    {
        cout << "  Running Test Case 13: Blossom Contraction Example (n=5)..." << endl;
        BlossomAlgorithm algo(5);
        algo.addEdge(0, 1);
        algo.addEdge(1, 2);
        algo.addEdge(2, 0);
        algo.addEdge(2, 3);
        algo.addEdge(3, 4);
        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 2);
         set<pair<int, int>> result_set(matching.begin(), matching.end());
         set<pair<int, int>> expected1 = {{0, 1}, {3, 4}};
         set<pair<int, int>> expected2 = {{0,1}, {2,3}};
         set<pair<int, int>> expected3 = {{1,2}, {3,4}};
         set<pair<int, int>> expected4 = {{0,2}, {3,4}};

        bool is_valid_max_matching = (result_set == expected1 || result_set == expected2 || result_set == expected3 || result_set == expected4);
        cout << "  Test Case 13 Passed!" << endl;
    }


     // Test Case 14: Disconnected Graph (C3 + P2)
    {
        cout << "  Running Test Case 14: Disconnected Graph (C3 + P2, n=5)..." << endl;
        BlossomAlgorithm algo(5);
        algo.addEdge(0, 1);
        algo.addEdge(1, 2);
        algo.addEdge(2, 0);
        algo.addEdge(3, 4);

        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 2);
        set<pair<int, int>> result_set(matching.begin(), matching.end());
        bool has_34 = result_set.count({3, 4});
        assert(has_34);
        bool has_c3_edge = result_set.count({0, 1}) || result_set.count({1, 2}) || result_set.count({0, 2});
        assert(has_c3_edge);
        cout << "  Test Case 14 Passed!" << endl;
    }

    // Test Case 15: Larger Disconnected Graph (K4 + C5)
    {
        cout << "  Running Test Case 15: Disconnected Graph (K4 + C5, n=9)..." << endl;
        BlossomAlgorithm algo(9);
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                algo.addEdge(i, j);
            }
        }
        algo.addEdge(4, 5); algo.addEdge(5, 6); algo.addEdge(6, 7); algo.addEdge(7, 8); algo.addEdge(8, 4);

        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 4);
        cout << "  Test Case 15 Passed!" << endl;
    }

    // Test Case 16: Two Disconnected C5s (Testing independent blossoms)
    {
        cout << "  Running Test Case 16: Two Disconnected C5s (n=10)..." << endl;
        BlossomAlgorithm algo(10);
        for (int i = 0; i < 5; ++i) algo.addEdge(i, (i + 1) % 5);
        for (int i = 5; i < 10; ++i) algo.addEdge(i, 5 + ((i - 5 + 1) % 5));

        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 4);
        cout << "  Test Case 16 Passed!" << endl;
    }

    // Test Case 17: Path through a Blossom needed (Classic Example)
    {
        cout << "  Running Test Case 17: Augmenting Path Through Blossom (n=9)..." << endl;
        BlossomAlgorithm algo(9);
        algo.addEdge(0, 1); algo.addEdge(1, 2); algo.addEdge(2, 3);
        algo.addEdge(3, 4); algo.addEdge(4, 0);
        algo.addEdge(2, 5); algo.addEdge(5, 6);
        algo.addEdge(0, 7); algo.addEdge(7, 8);
        algo.addEdge(6, 8);

        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 4);
        cout << "  Test Case 17 Passed!" << endl;
    }

    // Test Case 18: Barbell Graph with Odd Cycles (C5 - Path - C5)
    {
        cout << "  Running Test Case 18: Barbell C5-P3-C5 (n=11)..." << endl;
        BlossomAlgorithm algo(11);
        for (int i = 0; i < 5; ++i) algo.addEdge(i, (i + 1) % 5);
        algo.addEdge(4, 5);
        algo.addEdge(5, 6);
        algo.addEdge(6, 7); algo.addEdge(7, 8); algo.addEdge(8, 9);
        algo.addEdge(9, 10); algo.addEdge(10, 6);

        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 5);
        cout << "  Test Case 18 Passed!" << endl;
    }

    // Test Case 19: Interconnected Blossoms (Two C3s linked)
    {
        cout << "  Running Test Case 19: Interconnected C3s (n=8)..." << endl;
        BlossomAlgorithm algo(8);
        algo.addEdge(0, 1); algo.addEdge(1, 2); algo.addEdge(2, 0);
        algo.addEdge(3, 4); algo.addEdge(4, 5); algo.addEdge(5, 3);
        algo.addEdge(2, 6);
        algo.addEdge(3, 7);
        algo.addEdge(6, 7);
        algo.addEdge(1, 4);

        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 4);
        cout << "  Test Case 19 Passed!" << endl;
    }

     // Test Case 20: Graph with nested blossom potential (Harder to construct definitively)
     {
        cout << "  Running Test Case 20: C9 + attached C3 (n=12)..." << endl;
        BlossomAlgorithm algo(12);
        for(int i=0; i<9; ++i) algo.addEdge(i, (i+1)%9);
        algo.addEdge(9, 10); algo.addEdge(10, 11); algo.addEdge(11, 9);
        algo.addEdge(0, 9);
        algo.addEdge(3, 10);
        algo.addEdge(6, 11);

        vector<pair<int, int>> matching = algo.findMaximumMatching();
        assert(matching.size() == 6);
        cout << "  Test Case 20 Passed!" << endl;
    }


    cout << "All Blossom Algorithm tests passed!" << endl;
}

void runBlossomAlgorithmSample(){
    int n = 6;
    BlossomAlgorithm blossom(n);
    blossom.addEdge(0, 1);
    blossom.addEdge(0, 3);
    blossom.addEdge(1, 2);
    blossom.addEdge(2, 3);
    blossom.addEdge(2, 4);
    blossom.addEdge(3, 5);
    blossom.addEdge(4, 5);
    
    vector<pair<int, int>> matching = blossom.findMaximumMatching();
    
    cout << "Matching edges:" << endl;
    for (auto& edge : matching) {
        cout << edge.first << " -- " << edge.second << endl;
    }
}

int main() {
    testBlossomAlgorithm();
    runBlossomAlgorithmSample();
    
    return 0;
}
