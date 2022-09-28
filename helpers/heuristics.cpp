#include <algorithm> 
#include <cmath>

constexpr double D{1};
const double D2{std::sqrt(2)};

double getManhattanDistance(const double current_x, const double current_y, const double dest_x, const double dest_y)
{
    double dx = std::abs(current_y - dest_y);
    double dy = std::abs(current_x - dest_x);
    double distance = D * (dx + dy);

    return distance;
}

double getDiagonalDistance(const double current_x, const double current_y, const double dest_x, const double dest_y)
{
    double dx = std::abs(current_y - dest_y);
    double dy = std::abs(current_x - dest_x);
    double distance = D * (dx + dy) + (D2 - 2 * D) + std::min(dx, dy);
 
    return distance;
}

double getEuclideanDistance(const double current_x, const double current_y, const double dest_x, const double dest_y)
{
    double dx = std::abs(current_y - dest_y);
    double dy = std::abs(current_x - dest_x);
    double distance = D * std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    
    return distance;
}
