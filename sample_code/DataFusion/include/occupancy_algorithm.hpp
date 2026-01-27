#ifndef OCCUPANCY_ALGORITHM_HPP
#define OCCUPANCY_ALGORITHM_HPP

#include <vector>
#include <set>

namespace occupancy
{
struct Pt
{
    double x, y;
};

double dot(const Pt &a, const Pt &b);
Pt sub(const Pt &a, const Pt &b);
void projInterval(const std::vector<Pt> &pts, const Pt &axis,
                  double &outMin, double &outMax);
bool overlap1D(double aMin, double aMax, double bMin, double bMax);
std::vector<Pt> aabbCorners(double x0, double y0, double x1, double y1);
bool convexPolyIntersectsAabbSAT(const std::vector<Pt> &poly,
                                 double x0, double y0, double x1, double y1);
std::set<std::pair<int, int>> rasterizePolygonToCells(
    const std::vector<Pt> &poly, double cellSize);
} // namespace occupancy

#endif
