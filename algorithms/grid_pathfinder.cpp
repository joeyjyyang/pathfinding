#include <climits>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

#include "../data_structures/grid.cpp"
#include "../helpers/heuristics.cpp"

template <typename TId>
class GridPathfinder
{
public:
    GridPathfinder(const Grid<TId>& grid, const VertexIndices src_indices, const VertexIndices dest_indices) : grid_(grid), src_indices_(src_indices), dest_indices_(dest_indices)
    {
    }

    void run(const std::string algorithm)
    {
        std::cout << "Running pathfinding algorithm: " << algorithm << ".\n";
        
        const std::vector<std::pair<int, int>> moves = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        const auto grid_dimensions = grid_.getDimensions();
        const auto num_rows = grid_dimensions.first; 
        const auto num_cols = grid_dimensions.second;

        // 1.
        // Populate shortest distance and visited containers with initial values.
        distances_ = std::vector<std::vector<unsigned int>>(num_rows, std::vector<unsigned int>(num_cols, UINT_MAX));
        visited_ = std::vector<std::vector<bool>>(num_rows, std::vector<bool>(num_cols, false));
        // For grid, populate previous container with initial values.
        previous_ = std::vector<std::vector<VertexIndices>>(num_rows, std::vector<VertexIndices>(num_cols, {0, 0}));

        // 2.
        // Set shortest distance of source vertex (to source vertex) to 0.
        // Add source vertex and current distance (which is the shorest distance for the source vertex) to priority queue.
        const auto src_distance = 0;
        distances_[src_indices_.first][src_indices_.second] = src_distance;
        priority_q_.emplace(src_distance, src_indices_);

        // 3. Iterate through vertices in priority queue.
        while (!priority_q_.empty())
        {
            // 4. Get current vertex with shortest distance to source vertex.
            // Pop vertex from priority queue.
            // Track vertex's info, including current distance from source vertex.
            const auto current_vertex = priority_q_.top();
            priority_q_.pop();
            const auto current_distance = current_vertex.first;
            const auto current_indices = current_vertex.second;
            const auto current_row = current_indices.first;
            const auto current_col = current_indices.second;

            // Optional early exit; if only desire shortest path to destination vertex.
            // Required for A* and Greedy Best First Search.
            // Optional for Dijkstra.
            if (current_row == dest_indices_.first && current_col == dest_indices_.second)
            {
                break;
            }

            // 5. Mark vertex as visited.
            visited_[current_row][current_col] = true;

            // 6. Find all adjacent vertices.
            for (const auto& move : moves)
            {
                const auto adj_vertex_row = current_row + move.first;
                const auto adj_vertex_col = current_col + move.second;
                VertexIndices adj_indices = {adj_vertex_row, adj_vertex_col};

                // For grid, make sure adjacent vertex is within bounds.
                if (adj_vertex_row >= 0 && adj_vertex_row < num_rows && adj_vertex_col >= 0 && adj_vertex_col < num_cols)
                {
                    // For grid, make sure adjacent vertex is not blocked.
                    if (!grid_.isBlocked({adj_vertex_row, adj_vertex_col}))
                    {
                        // 7. Make sure adjacent vertex is unvisited.
                        if (!visited_[adj_vertex_row][adj_vertex_col])
                        {
                            // 8. Get adjacent vertex's current shortest distance to source vertex.
                            // Compute new distance from source vertex, through current vertex, to adjacent vertex.
                            const auto shortest_distance = distances_[adj_vertex_row][adj_vertex_col];
                            const auto new_distance = current_distance + 1; // Edge weight between adjacent vertices is always 1.

                            // 9. Compare adjacent vertex's current shortest distance to new distance from source vertex, through current vertex, to adjacent vertex.
                            if (new_distance < shortest_distance)
                            {
                                // 10. If new shortest distance for adjacent vertex to source vertex:
                                // Update shortest distance for adjacent vertex.
                                // Set previous vertex for adjacent vertex as current vertex.
                                // Add adjacent vertex and shortest distance to priority queue.
                                distances_[adj_vertex_row][adj_vertex_col] = new_distance;
                                previous_[adj_vertex_row][adj_vertex_col] = current_indices;

                                double priority = 0;

                                if (algorithm == "dijkstra")
                                {
                                    priority = new_distance;
                                }
                                else if (algorithm == "a star")
                                {
                                    priority = new_distance + getEuclideanDistance(adj_vertex_col, adj_vertex_row, dest_indices_.second, dest_indices_.first);
                                }
                                else if (algorithm == "greedy best first search")
                                {
                                    priority = getEuclideanDistance(adj_vertex_col, adj_vertex_row, dest_indices_.second, dest_indices_.first);
                                }
                                else
                                {
                                    throw std::invalid_argument("Invalid pathfinding algorithm specified!");       
                                }

                                priority_q_.emplace(priority, adj_indices);
                            }
                        }
                    }
                }
            }
        }
    }

    const std::vector<VertexIndices>& getShortestPath()
    {
        // Get shortest path.
        shortest_path_.emplace_back(dest_indices_);
        auto current_indices{dest_indices_}; 

        while (current_indices != src_indices_)
        {
            shortest_path_.emplace_back(previous_[current_indices.first][current_indices.second]);
            current_indices = previous_[current_indices.first][current_indices.second];
        }

        std::reverse(shortest_path_.begin(), shortest_path_.end());

        return shortest_path_;
    }

    void printHelpers() const
    {        
        std::cout << "------------------------------\n";
        std::cout << "--------- Visited ------------\n";
        std::cout << "------------------------------\n";

        for (unsigned int r_i = 0; r_i < visited_.size(); r_i++)
        {
            for (unsigned int c_i = 0; c_i < visited_[0].size(); c_i++)
            {
                std::cout << visited_[r_i][c_i];
            }
            
            std::cout << "\n";
        }

        /*std::cout << "------------------------------\n";
        std::cout << "----- Shortest Distances -----\n";
        std::cout << "------------------------------\n";

        for (unsigned int r_i = 0; r_i < distances_.size(); r_i++)
        {
            for (unsigned int c_i = 0; c_i < distances_[0].size(); c_i++)
            {
                std::cout << distances_[r_i][c_i];
            }
            
            std::cout << "\n";
        }

        std::cout << "------------------------------\n";
        std::cout << "--------- Previous -----------\n";
        std::cout << "------------------------------\n";

        for (unsigned int r_i = 0; r_i < previous_.size(); r_i++)
        {
            for (unsigned int c_i = 0; c_i < previous_[0].size(); c_i++)
            {
                std::cout << "(" << previous_[r_i][c_i].first << ", " << previous_[r_i][c_i].second << ") ";
            }
            
            std::cout << "\n";
        }*/
    }

private:
    Grid<TId> grid_;
    VertexIndices src_indices_;
    VertexIndices dest_indices_;
    std::priority_queue<std::pair<double, VertexIndices>, std::vector<std::pair<double, VertexIndices>>, std::greater<std::pair<double, VertexIndices>>> priority_q_;
    std::vector<std::vector<unsigned int>> distances_;
    std::vector<std::vector<bool>> visited_;
    std::vector<std::vector<VertexIndices>> previous_;
    std::vector<VertexIndices> shortest_path_;
};

void printPath(const std::vector<VertexIndices>& path)
{
    for (unsigned int i = 0; i < path.size(); i++)
    {
        std::cout << "(" << path[i].first << "," << path[i].second << ") -> ";
    }

    std::cout << "\n";
}

int main(int argc, char* argv[])
{
    std::string algorithm{""};

    if (argc == 1)
    {
        algorithm = "a star";
    }
    else
    {
        algorithm = argv[1];
    }

    constexpr VertexIndices SRC_INDICES{12, 0};
    constexpr VertexIndices DEST_INDICES{1, 11};

    // Set to 'o' for non-blocked..
    Grid<char> grid(15, 15, 'o');

    grid.setVertex(SRC_INDICES, 's');
    grid.setVertex(DEST_INDICES, 'd');
    // Set blocked vertices.
    grid.setBlocked({12, 2});
    grid.setBlocked({12, 3});
    grid.setBlocked({12, 4});
    grid.setBlocked({12, 5});
    grid.setBlocked({12, 6});
    grid.setBlocked({12, 7});
    grid.setBlocked({12, 8});
    grid.setBlocked({12, 9});
    grid.setBlocked({12, 10});
    grid.setBlocked({12, 11});
    grid.setBlocked({12, 12});
    grid.setBlocked({11, 12});
    grid.setBlocked({10, 12});
    grid.setBlocked({9, 12});
    grid.setBlocked({8, 12});
    grid.setBlocked({7, 12});
    grid.setBlocked({6, 12});
    grid.setBlocked({5, 12});
    grid.setBlocked({4, 12});
    grid.setBlocked({3, 12});
    grid.setBlocked({2, 12});
    grid.setBlocked({2, 11});
    grid.setBlocked({2, 10});
    grid.setBlocked({2, 9});
    grid.setBlocked({2, 8});
    grid.setBlocked({2, 7});
    grid.setBlocked({2, 6});
    grid.setBlocked({2, 5});

    std::cout << "-----------------------\n";
    printGrid(grid);
    std::cout << "-----------------------\n";

    GridPathfinder<char> grid_pathfinder(grid, SRC_INDICES, DEST_INDICES);
    grid_pathfinder.run(algorithm);
    grid_pathfinder.printHelpers();

    const auto& shortest_path = grid_pathfinder.getShortestPath();
    printPath(shortest_path);

    return 0;
}
