#include <algorithm> 
#include <cmath>

#include "../grid/grid.cpp"

constexpr double D{1};
const double D2{std::sqrt(2)};

double getManhattanDistance(const int current_row, const int current_col, const int dest_row, const int dest_col)
{
    double dx = std::abs(current_col - dest_col);
    double dy = std::abs(current_row - dest_row);
    double distance = D * (dx + dy);

    return distance;
}

double getDiagonalDistance(const int current_row, const int current_col, const int dest_row, const int dest_col)
{
    double dx = std::abs(current_col - dest_col);
    double dy = std::abs(current_row - dest_row);
    double distance = D * (dx + dy) + (D2 - 2 * D) + std::min(dx, dy);
 
    return distance;
}

double getEuclideanDistance(const int current_row, const int current_col, const int dest_row, const int dest_col)
{
    double dx = std::abs(current_col - dest_col);
    double dy = std::abs(current_row - dest_row);
    double distance = D * std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    
    return distance;
}

int main(int argc, char const *argv[])
{
    auto test1 = getManhattanDistance(1, 1, 1, 1);
    auto test2 = getDiagonalDistance(1, 1, 1, 1);
    auto test3 = getEuclideanDistance(1, 1, 1, 1);

    return 0;
}
