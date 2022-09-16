#include <vector>

template <typename TId>
class Grid
{
public:
    Grid(const unsigned int num_rows, const unsigned int num_cols, const TId initial_value) : num_rows_(num_rows), num_cols_(num_cols), grid_(num_rows, std::vector<TId>(num_cols, initial_value))
    {
    }

private:
    const unsigned int num_rows_;
    const unsigned int num_cols_;
    std::vector<std::vector<TId>> grid_;
};

int main(int argc, char const *argv[])
{
    Grid<char> grid(5, 5, 'a');

    return 0;
}
