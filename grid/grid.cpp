#include <iostream>
#include <vector>

template <typename TId>
using VertexId = TId;

// Cannot be std::pair of const unsigned int, since std::priority_queue does not support value of std::pair<const unsigned int, const unsigned int> (anymore?).
using VertexIndices = std::pair<unsigned int, unsigned int>;

template <typename TId>
class Grid
{
public:
    Grid(const unsigned int num_rows, const unsigned int num_cols, const VertexId<TId> initial_value) : grid_(num_rows, std::vector<VertexId<TId>>(num_cols, initial_value))
    {
    }

    void setVertex(const VertexIndices indices, const VertexId<TId> value)
    {
        grid_[indices.first][indices.second] = value;
    }

    void setBlocked(const VertexIndices indices)
    {
        setVertex(indices, 'b');
    }

    bool isBlocked(const VertexIndices indices)
    {
        return (grid_[indices.first][indices.second] == 'b' ? true : false);
    }

    // Important to not return by reference here, since grid dimensions are temporary variables.
    const std::pair<const unsigned int, const unsigned int> getDimensions() const
    {
        return {grid_.size(), grid_[0].size()};
    }

    template <typename TIdFriend>
    friend void printGrid(const Grid<TIdFriend>& grid);

private:
    std::vector<std::vector<VertexId<TId>>> grid_;
};

template <typename TIdFriend>
void printGrid(const Grid<TIdFriend>& grid)
{
    for (unsigned int r_i = 0; r_i < grid.grid_.size(); r_i++)
    {
        for (unsigned int c_i = 0; c_i < grid.grid_[0].size(); c_i++)
        {
            std::cout << grid.grid_[r_i][c_i];
        }
        
        std::cout << "\n";
    }
}
