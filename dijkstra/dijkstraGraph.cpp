#include <climits>
#include <iostream>
#include <queue>
#include <vector>

#include "../graph/graph.cpp"

template <typename TId, typename TWeight>
class Dijkstra
{
public:
    Dijkstra(const Graph<TId, TWeight>& graph, const VertexId<TId> src_id, const VertexId<TId> dest_id) : graph_(graph), src_id_(src_id), dest_id_(dest_id)
    {
    }

    void run()
    {
        const auto& vertices = graph_.getVertices();

        // 1.
        // Populate shortest distance and visited containers with initial values.
        for (const auto& src_id : vertices)
        {
            distances_[src_id] = UINT_MAX;
            visited_[src_id] = false;
            // Optional: populate previous container with initial values.
            previous_[src_id] = src_id;
        }

        // 2.
        // Set shortest distance of source vertex (to source vertex) to 0.
        // Add source vertex and current distance (which is the shorest distance for the source vertex) to priority queue.
        const auto src_distance = 0;
        distances_[src_id_] = src_distance;
        priority_q_.emplace(src_distance, src_id_);

        // 3. Iterate through vertices in priority queue.
        while (!priority_q_.empty())
        {
            // 4. Get current vertex with shortest distance to source vertex.
            // Pop vertex from priority queue.
            // Track vertex's info, including current distance from source vertex.
            const auto current_vertex = priority_q_.top();
            priority_q_.pop();
            const auto current_distance = current_vertex.first;
            const auto current_id = current_vertex.second;

            // Optional: early exit if only desire shortest path to destination vertex.
            //if (current_id == dest_id_)
            //{
            //    break;
            //}

            // 5. Mark vertex as visited.
            visited_[current_id] = true;

            // 6. Find all adjacent vertices.
            const auto& edges = graph_.getEdges(current_id);

            for (const auto& [adj_id, adj_distance] : edges)
            {
                // 7. Make sure adjacent vertex is unvisited.
                if (!visited_[adj_id])
                {
                    // 8. Get adjacent vertex's current shortest distance to source vertex.
                    // Compute distance from current vertex to adjacent vertex.
                    const auto shortest_distance = distances_[adj_id];
                    const auto new_distance = current_distance + adj_distance;

                    // 9. Compare adjacent vertex's current shortest distance to new distance from source vertex, through current vertex, to adjacent vertex.
                    if (new_distance < shortest_distance)
                    {
                        // 10. If new shortest distance for adjacent vertex to source vertex:
                        // Update shortest distance for adjacent vertex.
                        // Set previous vertex for adjacent vertex as current vertex.
                        // Add adjacent vertex and shortest distance to priority queue.
                        distances_[adj_id] = new_distance;
                        previous_[adj_id] = current_id;
                        priority_q_.emplace(new_distance, adj_id);
                    }
                }
            }
        }
    }

    const std::vector<VertexId<TId>>& getShortestPath()
    {
        // Get shortest path.
        shortest_path_.emplace_back(dest_id_);
        auto current_id{dest_id_}; 

        while (current_id != src_id_)
        {
            shortest_path_.emplace_back(previous_[current_id]);
            current_id = previous_[current_id];
        }

        std::reverse(shortest_path_.begin(), shortest_path_.end());

        return shortest_path_;
    }

    void printHelpers() const
    {
        std::cout << "-------HELPERS---------\n";
        std::cout << "Visited:\n";

        for (const auto& [key, val] : visited_)
        {
            std::cout << key << ": " << val << ", ";
        }
        std::cout << "\n";
        std::cout << "Shortest Distances:\n";

        for (const auto& [key, val] : distances_)
        {
            std::cout << key << ": " << val << ", ";
        }
        std::cout << "\n";
        std::cout << "Previous:\n";

        for (const auto& [key, val] : previous_)
        {
            std::cout << key << ": " << val << ", ";
        }
        std::cout << "\n";
    }

private:
    Graph<TId, TWeight> graph_;
    VertexId<TId> src_id_;
    VertexId<TId> dest_id_;
    std::priority_queue<std::pair<EdgeWeight<TWeight>, VertexId<TId>>, std::vector<std::pair<EdgeWeight<TWeight>, VertexId<TId>>>, std::greater<std::pair<EdgeWeight<TWeight>, VertexId<TId>>>> priority_q_;
    std::unordered_map<VertexId<TId>, EdgeWeight<TWeight>> distances_;
    std::unordered_map<VertexId<TId>, bool> visited_;
    std::unordered_map<VertexId<TId>, VertexId<TId>> previous_;
    std::vector<VertexId<TId>> shortest_path_;
};

template <typename TId>
void printPath(const std::vector<VertexId<TId>>& path)
{
    for (unsigned int i = 0; i < path.size(); i++)
    {
        std::cout << path[i] << "-> ";
    }

    std::cout << "\n";
}

int main(int argc, char* argv[])
{
    {
        constexpr VertexId<const char> SRC_ID{'a'};
        constexpr VertexId<const char> DEST_ID{'c'};

        // Non-negative weights.
        Graph<char, unsigned int> graph;
        graph.addBidirectionalEdge(SRC_ID, 'd', 1);
        graph.addBidirectionalEdge(SRC_ID, 'b', 6);
        graph.addBidirectionalEdge('b', 'd', 2);
        graph.addBidirectionalEdge('d', 'e', 1);
        graph.addBidirectionalEdge('b', 'e', 2);
        graph.addBidirectionalEdge('b', DEST_ID, 5);
        graph.addBidirectionalEdge('e', DEST_ID, 5);

        std::cout << "-----------------------\n";
        printGraph(graph);
        std::cout << "-----------------------\n";

        Dijkstra<char, unsigned int> dijkstra(graph, SRC_ID, DEST_ID);
        dijkstra.run();
        dijkstra.printHelpers();

        const auto& shortest_path = dijkstra.getShortestPath();
        printPath(shortest_path);
    }

    {
        constexpr VertexId<const unsigned int> SRC_ID{0};
        constexpr VertexId<const unsigned int> DEST_ID{4};

        // Non-negative weights.
        Graph<unsigned int, unsigned int> graph;
        graph.addBidirectionalEdge(SRC_ID, 1, 4);
        graph.addBidirectionalEdge(SRC_ID, 7, 8);
        graph.addBidirectionalEdge(1, 7, 11);
        graph.addBidirectionalEdge(7, 8, 7);
        graph.addBidirectionalEdge(7, 6, 1);
        graph.addBidirectionalEdge(6, 8, 6);
        graph.addBidirectionalEdge(1, 2, 8);
        graph.addBidirectionalEdge(2, 8, 2);
        graph.addBidirectionalEdge(2, 5, 4);
        graph.addBidirectionalEdge(6, 5, 2);
        graph.addBidirectionalEdge(2, 3, 7);
        graph.addBidirectionalEdge(3, 5, 14);
        graph.addBidirectionalEdge(3, DEST_ID, 9);
        graph.addBidirectionalEdge(5, DEST_ID, 10);

        std::cout << "-----------------------\n";
        printGraph(graph);
        std::cout << "-----------------------\n";

        Dijkstra<unsigned int, unsigned int> dijkstra(graph, SRC_ID, DEST_ID);
        dijkstra.run();
        dijkstra.printHelpers();

        const auto& shortest_path = dijkstra.getShortestPath();
        printPath(shortest_path);
    }

    return 0;
}
