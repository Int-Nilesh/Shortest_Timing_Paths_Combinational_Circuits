#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <map>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <queue>
#include <limits>
#include <stack>

using namespace std;

class DijkstraAlgorithm {
public:
    static std::vector<std::string> findShortestPath(const std::unordered_map<std::string, std::list<std::pair<std::string, int>>>& adjacencyList, const std::string& start, const std::string& destination) {
        std::unordered_map<std::string, int> distance;
        std::unordered_map<std::string, std::string> predecessors;
        std::priority_queue<std::pair<int, std::string>, std::vector<std::pair<int, std::string>>, std::greater<std::pair<int, std::string>>> pq;

        // Initialize distances to infinity and add start node to the priority queue
        for (const auto& entry : adjacencyList) {
            distance[entry.first] = std::numeric_limits<int>::max();
        }
        distance[start] = 0;
        pq.push(std::make_pair(0, start));

        // Dijkstra's algorithm
        while (!pq.empty()) {
            std::string current = pq.top().second;
            int dist = pq.top().first;
            pq.pop();

            // Update distance and predecessors if a shorter path is found
            if (dist <= distance[current]) {
                for (const auto& neighbor : adjacencyList.at(current)) {
                    int newDist = dist + neighbor.second;
                    if (newDist < distance[neighbor.first]) {
                        distance[neighbor.first] = newDist;
                        predecessors[neighbor.first] = current;
                        pq.push(std::make_pair(newDist, neighbor.first));
                    }
                }
            }
        }

        // Reconstruct the path
        std::vector<std::string> path;
        std::string current = destination;
        while (predecessors.find(current) != predecessors.end()) {
            path.insert(path.begin(), current);
            current = predecessors[current];
        }
        path.insert(path.begin(), start); // Add the starting node

        return path;
    }
};

class WeightedDirectedGraph {
private:
    std::unordered_map<std::string, std::list<std::pair<std::string, int>>> adjacencyList;

public:
    void addNode(const std::string& node) {
        if (adjacencyList.find(node) == adjacencyList.end()) {
            adjacencyList[node] = std::list<std::pair<std::string, int>>();
        }
    }

    void addEdge(const std::string& from, const std::string& to, int weight) {
        adjacencyList[from].push_back(std::make_pair(to, weight));
    }

    const std::unordered_map<std::string, std::list<std::pair<std::string, int>>>& getAdjacencyList() const {
        return adjacencyList;
    }

    void displayGraph() const {
        for (const auto& entry : adjacencyList) {
            std::cout << entry.first << " -> ";
            for (const auto& neighbor : entry.second) {
                std::cout << "(" << neighbor.first << ", " << neighbor.second << ") ";
            }
            std::cout << std::endl;
        }
    }
};

int main(int argc, char *argv[]) {
    bool flag =false;
    if (argc != 4) {
        cout << "Incorrect number of arguments" << endl;
        return 1;
    }
    // Declare a file stream object
    std::list <std::string> inputs; 
    std::list <std::string> outputs;
    std::list <std::string> nodesA;
    std::list <std::string> allNodes;
    std::list <std::string> nodes;
    std::ifstream file;
    // Extract command line arguments
    string fileName = argv[1];
    string source = argv[2];
    string destination = argv[3];

    std::string validFiles [15]= {"c17.bench", "c432.bench", "c499.bench", "c880.bench", "c1355.bench", "c1908.bench", "c2670.bench", "c3540.bench", "c5315.bench", "c6288.bench", "c7552.bench"};
    for ( int i = 0; i<11; i++ )
    {
        if (fileName == validFiles[i]){
            flag = true;
            break;
        }
    }
    if (flag == false){    
        std::cout << "Wrong file name\n";
    }
    if (flag == true){
    // Open the file
    file.open(fileName);

    // Check if the file was successfully opened
    if (!file.is_open()) {
        std::cerr << "Error opening the file." << std::endl;
        return 1;
    }
    // Read and display the contents of the file
    std::string line;
    string startSeq = "INPUT(";
    string outSeq = "OUTPUT(";
    string endSeq = ")";
    std::string result;

    while (std::getline(file, line)) {
        //skipping comments from file
        if (line.size() > 0 && line[0] == '#') {
            // Skip this line
            continue;
        }
        // looking for "input("
        size_t start_In_Pos = line.find(startSeq);
        // looking for "OUTPUT("
        size_t start_Out_Pos = line.find(outSeq);

        if (start_In_Pos != std::string::npos){
            //looking for ")"
            size_t  end_In_Pos = line.find(endSeq,start_In_Pos + startSeq.length());
            if (end_In_Pos != std::string::npos){
                result = line.substr(start_In_Pos + startSeq.length(), end_In_Pos - start_In_Pos - startSeq.length());
                inputs.push_back(result);
            }
        }
        if ( start_Out_Pos != std::string::npos){
            // looking for ")"
            size_t  end_Out_Pos = line.find(endSeq,start_Out_Pos + outSeq.length());
            if (end_Out_Pos != std::string::npos){
                result = line.substr(start_Out_Pos + outSeq.length(), end_Out_Pos - start_Out_Pos - outSeq.length());
                outputs.push_back(result);
            }
        }
        size_t node_Pos = line.find("=");
        if ( node_Pos != std::string::npos){
            // looking for ")"
            result = line.substr(0, node_Pos-1);
            nodes.push_back(result);
            nodesA.push_back(result);
            //cout << result;
            }
        }
    
    for (auto it = nodes.begin(); it != nodes.end(); ) {
        // Check if the node is present in the outputs list
        if (std::find(outputs.begin(), outputs.end(), *it) != outputs.end()) {
            it = nodes.erase(it); // erase returns the iterator to the next element
        } else {
            ++it;
        }
    }
    for (auto i : nodes){
        if (source == i || destination == i){
            cout <<"Signal " << i<<" not found in file"<< fileName;
            flag = false;
            return 1;
        }
    }
    for (auto i : outputs){
        if (source == i){
            flag = false;
            cout << "Signal "<< source << "is not an input pin";
            return 1;
        }
    }
    for ( auto i : inputs){
        if ( destination == i){
            flag = false;
            cout << "Signal "<< destination << "is not an output pin";
            return 1;
        }
    }
    for (auto i : inputs){
        if (source == i)
        {
            flag = true;
            break;
        }
        else {
            flag = false;
        }
    }
    if (flag == false){
        cout<< "Signal "<< source << " not found in file"<< fileName;
        return 1;
    }
    for (auto i : outputs){
        if (destination == i){
            flag = true;
            break;
        }
        else {
            flag = false;  
        }
    }
    if (flag == false){
        cout<< "Signal "<< destination << " not found in file"<< fileName;
        return 1;
    }
    allNodes.insert(allNodes.end(), inputs.begin(), inputs.end());
    allNodes.insert(allNodes.end(), nodesA.begin(), nodesA.end());

    // Rewind the file pointer to the beginning of the file
    file.clear(); // clear any error flags
    file.seekg(0, std::ios::beg);
    std::vector<std::string> extractedStrings;
    std::map<std::string, int> fanouts;
    while (std::getline(file, line)){
        // Check conditions to skip the line
        if (line.empty() || line[0] == '#' || line.find("INPUT") == 0 || line.find("OUTPUT") == 0) {
            // Skip the line
            continue;
        }
        for ( auto i : allNodes){
            int count =0;
            size_t pos = line.find(i);
            while (pos != std::string::npos) {
                // Key found, increment the count
                fanouts[i]++;
                // Search for the key in the rest of the line
                pos = line.find(i, pos + i.length());
            }
        }
    }
    list <int> weights;
    //cout<< "weight of Nodes:\n";
    for (auto& entry : fanouts){
        for (auto i : nodesA){
            //weights.push_back(entry.second);
            if ( entry.first == i)
                entry.second -=1;
        }
    }
    for (auto i : allNodes)
        for( auto& entry : fanouts){
            if (i == entry.first){
                weights.push_back(entry.second);
            }
        }

    std::vector<std::vector<std::string>> neighbours;

    for (const auto& node : allNodes) {
            std::vector<std::string> nodeNeighbours; // Sub-vector for each node

            file.clear(); // Clear any previous errors
            file.seekg(0); // Move the file pointer to the beginning

            while (std::getline(file, line)) {
                if (line.empty() || line[0] == '#' || line.find("INPUT") == 0 || line.find("OUTPUT") == 0) {
                // Skip the line
                continue;
                }
                std::size_t found = line.find(node);
                if (found != std::string::npos) {
                    std::size_t delimiterPos = line.find(" =");
                    std::string neighbor = line.substr(0, delimiterPos);
                    if (neighbor != node)
                        nodeNeighbours.push_back(neighbor);
                }
            }

            neighbours.push_back(nodeNeighbours);
        }

    // Create the weighted directed graph
    WeightedDirectedGraph graph;

    // Add nodes to the graph
    for (const auto& node : allNodes) {
        graph.addNode(node);
    }

    // Add edges to the graph with weights
    auto nodeIt = allNodes.begin();
    auto weightIt = weights.begin();
    for (const auto& neighborList : neighbours) {
        for (const auto& neighbor : neighborList) {
            graph.addEdge(*nodeIt, neighbor, *weightIt);
        }
        ++nodeIt;
        ++weightIt;
    }

    // Find the shortest path
    std::vector<std::string> shortestPath = DijkstraAlgorithm::findShortestPath(graph.getAdjacencyList(), source, destination);

    if (shortestPath.size()==1) {
        std::cout << "No path present." << std::endl;
    } else {
        std::cout << "Shortest path from " << source << " to " << destination << ": ";
        for (const auto& node : shortestPath) {
            std::cout << node << " ";
        }
        std::cout << std::endl;
    }
    //Close the file
    file.close();
    }

    return 0;
}