
#include <vector>
#include <list>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <climits>

class Node {
private:
    int id; // Node identifier
    // You can add more attributes if needed, such as node weight, label, etc.

public:
    Node(int _id) : id(_id) {}

    int getId() const { return id; }

    // Additional member functions if necessary
};

class Edge {
private:
    Node source;
    Node destination;
    // Add more attributes if needed, like weight for weighted graphs

public:
    Edge(const Node& _source, const Node& _destination) : source(_source), destination(_destination) {}

    Node getSource() const { return source; }
    Node getDestination() const { return destination; }

    // Additional member functions if necessary
};

class Graph {
private:
    std::list<Node> nodes;
    std::list<Edge> edges;
    int minKey(const std::unordered_map<int, int>& key, const std::unordered_set<int>& mstSet) {
        int min = INT_MAX, min_index;

        for (const auto& node : nodes) {
            if (mstSet.find(node.getId()) == mstSet.end() && key.at(node.getId()) < min) {
                min = key.at(node.getId());
                min_index = node.getId();
            }
        }

        return min_index;
    }


public:
    void addNode(const Node& node) {
        nodes.push_back(node);
    }
    
    void addEdge(const Node& source, const Node& destination) {
        Edge edge(source, destination);
        edges.push_back(edge);
    }

    Node* findNode(int id) {
        for (auto& node : nodes) {
            if (node.getId() == id) {
                return &node;
            }
        }
        return nullptr; // Return null if the node isn't found
    }

    void removeNode(int id) {
        nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
                                   [id](const Node& node) { return node.getId() == id; }),
                    nodes.end());

        edges.remove_if([id](const Edge& edge) {
            return edge.getSource().getId() == id || edge.getDestination().getId() == id;
        });
    }

    void removeEdge(const Node& source, const Node& destination) {
        edges.remove_if([&source, &destination](const Edge& edge) {
            return edge.getSource().getId() == source.getId() && edge.getDestination().getId() == destination.getId();
        });
    }

    void DFSUtil(int nodeId, std::unordered_set<int>& visited) {
        visited.insert(nodeId);
        std::cout << nodeId << " ";  // Process the node (in this case, print it)

        Node* currentNode = findNode(nodeId);
        if (currentNode) {
            // Traverse adjacent nodes
            for (const auto& edge : edges) {
                if (edge.getSource().getId() == nodeId && visited.find(edge.getDestination().getId()) == visited.end()) {
                    DFSUtil(edge.getDestination().getId(), visited);
                }
            }
        }
    }

    void DFS(int startNodeId) {
        std::unordered_set<int> visited; // To keep track of visited nodes
        DFSUtil(startNodeId, visited);
    }

    std::unordered_map<int, int> dijkstra(int startNodeId) {
        // The result will store the shortest distances from the startNodeId to all other nodes
        std::unordered_map<int, int> distances;
        for (const auto& node : nodes) {
            distances[node.getId()] = INT_MAX;
        }
        distances[startNodeId] = 0;

        // The priority queue will store pairs of (distance, nodeId)
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;
        pq.push({0, startNodeId});

        while (!pq.empty()) {
            int current_distance = pq.top().first;
            int current_id = pq.top().second;
            pq.pop();

            // If this node's shortest distance has already been found, skip it
            if (current_distance > distances[current_id]) {
                continue;
            }

            for (const auto& edge : edges) {
                if (edge.getSource().getId() == current_id) {
                    int neighbor_id = edge.getDestination().getId();
                    // Let's assume for now all edges have a weight of 1. Modify as needed.
                    int new_distance = current_distance + 1;

                    if (new_distance < distances[neighbor_id]) {
                        distances[neighbor_id] = new_distance;
                        pq.push({new_distance, neighbor_id});
                    }
                }
            }
        }

        return distances;
    }
    struct PrimsResult {
        std::list<Edge> includedEdges;
        int totalWeight;
    };

    PrimsResult primsMST() {
        PrimsResult result;
        result.totalWeight = 0;

        // If graph is empty
        if (nodes.empty()) {
            return result;
        }

        // Initializing all distances as infinite and MST as false
        std::unordered_map<int, int> key;   // Values used to pick minimum weight edge in cut
        std::unordered_map<int, int> parent;
        std::unordered_set<int> inMST;

        for (const auto& node : nodes) {
            key[node.getId()] = INT_MAX;
        }

        // Starting with the first node
        key[nodes.front().getId()] = 0;
        parent[nodes.front().getId()] = -1;


        for (int count = 0; count < nodes.size() - 1; ++count) {
            int u = minKey(key, inMST);
            inMST.insert(u);

            // Adding the picked vertex to the MST
            if (parent[u] != -1) {
                Edge e(*findNode(parent[u]), *findNode(u));
                result.includedEdges.push_back(e);
                result.totalWeight += 1; // Assuming a weight of 1 for all edges

                std::cout << "Adding edge to MST: " << parent[u] << "-" << u << std::endl;
            }

            // Update key values and parent index of the adjacent vertices of the picked vertex
            for (const auto& edge : edges) {
                if (edge.getSource().getId() == u) {
                    int v = edge.getDestination().getId();
                    if (inMST.find(v) == inMST.end() && 1 < key[v]) { // Using 1 as the weight
                        key[v] = 1; // Modify this for weighted edges
                        parent[v] = u;
                    }
                }
            }
        }

        return result;
    }
};