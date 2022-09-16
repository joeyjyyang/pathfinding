#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

template <typename TId>
using VertexId = TId;

template <typename TWeight>
using EdgeWeight = TWeight;

template <typename TId, typename TWeight>
using Edges = std::unordered_map<VertexId<TId>, EdgeWeight<TWeight>>;

template <typename TId, typename TWeight>
using AdjacencyList = std::unordered_map<VertexId<TId>, Edges<TId, TWeight>>;

// This is minimalistically correct.
//template <typename TId, typename TWeight>
//using Graph = AdjacencyList<TId, TWeight>;

template <typename TId, typename TWeight>
class Graph
{
public:
    Graph()
    {
    }

    void addEdge(const VertexId<TId> src_id, const VertexId<TId> dest_id, const EdgeWeight<TWeight> weight)
    {
        // Source vertex with edge(s) already exists in graph, and
        // destination vertex from source vertex already exists.
        if (graph_.contains(src_id) && graph_[src_id].contains(dest_id))
        {
            throw std::invalid_argument("Could not add edge to vertex. Edge already exists!");
        }

        graph_[src_id].emplace(dest_id, weight);
    }

    void addBidirectionalEdge(const VertexId<TId> src_id, const VertexId<TId> dest_id, const EdgeWeight<TWeight> weight)
    {
        addEdge(src_id, dest_id, weight);
        addEdge(dest_id, src_id, weight);
    }

    const Edges<TId, TWeight>& getEdges(const VertexId<TId> src_id) const
    {
        // Unnecessary since cannot add vertex without edge to graph anyways.
        //if (!graph_.contains(src_id))
        //{
        //    throw std::invalid_argument("Cannot find edges for vertex. Vertex does not exist!");  
        //}

        return graph_.at(src_id);
    }

    // Important to not return by reference here, since vertices is a local variable.
    const std::vector<VertexId<TId>> getVertices() const
    {
        std::vector<VertexId<TId>> vertices;

        for (auto const& [src_id , weight] : graph_)
        {
            vertices.emplace_back(src_id);
        }

        return vertices;
    }

    // Unnecessary to expose in API.
    //const AdjacencyList& getGraph()
    //{
    //    return graph_;
    //}

    template <typename TIdFriend, typename TWeightFriend>
    friend void printGraph(const Graph<TIdFriend, TWeightFriend>& graph);

private:
    AdjacencyList<TId, TWeight> graph_;
};

template <typename TId, typename TWeight>
void printGraph(const Graph<TId, TWeight>& graph)
{
    for (const auto& [src_id, edge_list] : graph.graph_)
    {
        std::cout << src_id << " : ";

        for (const auto& [dest_id, weight] : edge_list)
        {
            std::cout << "(" << dest_id << ", " << weight << ") ";
        }
        
        std::cout << "\n";
    }
}
