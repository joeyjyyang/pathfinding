#include <climits>
#include <iostream>
#include <queue>
#include <vector>

#include "graph.cpp"

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

        for (const auto& src_id : vertices)
        {
            distances_[src_id] = UINT_MAX;
            visited_[src_id] = false;
        }

        distances_[src_id_] = 0;

        priority_q_.emplace(0, src_id_);

        while (!priority_q_.empty())
        {
            const auto current_vertex = priority_q_.top();
            priority_q_.pop();

            const auto current_distance = current_vertex.first;
            const auto current_id = current_vertex.second;

            // Optional early exit; if only desire shortest path to destination vertex.
            //if (current_id == dest_id_)
            //{
            //    break;
            //}

            visited_[current_id] = true;

            const auto& edges = graph_.getEdges(current_id);

            for (const auto& [adj_id, adj_distance] : edges)
            {
                if (!visited_[adj_id])
                {
                    auto shortest_distance = distances_[adj_id];
                    auto new_distance = current_distance + adj_distance;

                    // Found new shortest path from source vertex, through current vertex, to adjacent vertex.
                    if (new_distance < shortest_distance)
                    {
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
        for (const auto& [key, val] : visited_)
        {
            std::cout << key << ": " << val << ", ";
        }
        std::cout << "\n";

        for (const auto& [key, val] : distances_)
        {
            std::cout << key << ": " << val << ", ";
        }
        std::cout << "\n";

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
