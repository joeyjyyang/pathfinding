#include "dijkstra.cpp"

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
        constexpr VertexId<char> SRC_ID{'a'};
        constexpr VertexId<char> DEST_ID{'c'};

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
        constexpr VertexId<unsigned int> SRC_ID{0};
        constexpr VertexId<unsigned int> DEST_ID{4};

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
