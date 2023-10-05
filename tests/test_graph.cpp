
#include "../include/graph_structures.hpp"
#include <cassert>

void testBasicOperations() {
    Graph g;
    Node n1(1), n2(2), n3(3);

    g.addNode(n1);
    g.addNode(n2);
    g.addNode(n3);

    // Assertion checks
    assert(g.findNode(1) != nullptr);
    assert(g.findNode(2) != nullptr);
    assert(g.findNode(3) != nullptr);
    assert(g.findNode(4) == nullptr);

    g.addEdge(n1, n2);
    g.addEdge(n2, n3);

    // Add more assertions if you have functions to get edges, neighbors, etc.

    g.removeNode(3);

    assert(g.findNode(3) == nullptr);

    // Similarly, if you add functions to check edge existence, you can test edge removal too

    std::cout << "Basic operations test passed!" << std::endl;
}

void testDijkstras() {
    Graph g;
    Node n1(1), n2(2), n3(3), n4(4);

    g.addNode(n1);
    g.addNode(n2);
    g.addNode(n3);
    g.addNode(n4);

    g.addEdge(n1, n2);
    g.addEdge(n2, n3);
    g.addEdge(n3, n4);
    g.addEdge(n1, n4);

    auto distances = g.dijkstra(1);

    // For the given graph, the distances should be:
    assert(distances[1] == 0);
    assert(distances[2] == 1);
    assert(distances[3] == 2);
    assert(distances[4] == 1);

    std::cout << "Dijkstra's test passed!" << std::endl;
}

void testPrims() {
    Graph g;
    Node n1(1), n2(2), n3(3), n4(4);

    g.addNode(n1);
    g.addNode(n2);
    g.addNode(n3);
    g.addNode(n4);

    g.addEdge(n1, n2);
    g.addEdge(n2, n3);
    g.addEdge(n3, n4);
    g.addEdge(n1, n4);

    auto mstResult = g.primsMST();

    // Simple check: For this graph, the MST will include 3 edges
    assert(mstResult.includedEdges.size() == 3);

    // If you add weights to edges, you can also check totalWeight
    assert(mstResult.totalWeight == 3); // As all edges are assumed to have a weight of 1

    std::cout << "Prim's test passed!" << std::endl;
}

int main() {
    testBasicOperations();
    testDijkstras();
    testPrims();

    std::cout << "All tests completed successfully!" << std::endl;
    return 0;
}
