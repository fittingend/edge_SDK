#include "occupancy_algorithm.hpp"

#include <cmath>
#include <limits>

namespace occupancy
{
static constexpr double EPS = 1e-12;

double dot(const Pt &a, const Pt &b)
{
    return a.x * b.x + a.y * b.y;
}

Pt sub(const Pt &a, const Pt &b)
{
    return Pt{a.x - b.x, a.y - b.y};
}

void projInterval(const std::vector<Pt> &pts, const Pt &axis,
                  double &outMin, double &outMax)
{
    outMin = std::numeric_limits<double>::infinity();
    outMax = -std::numeric_limits<double>::infinity();
    for (const auto &p : pts)
    {
        double v = dot(p, axis);
        if (v < outMin)
            outMin = v;
        if (v > outMax)
            outMax = v;
    }
}

bool overlap1D(double aMin, double aMax, double bMin, double bMax)
{
    if (aMax <= bMin + EPS)
        return false;
    if (bMax <= aMin + EPS)
        return false;
    return true;
}

std::vector<Pt> aabbCorners(double x0, double y0, double x1, double y1)
{
    return {Pt{x0, y0}, Pt{x1, y0}, Pt{x1, y1}, Pt{x0, y1}};
}

bool convexPolyIntersectsAabbSAT(const std::vector<Pt> &poly,
                                 double x0, double y0, double x1, double y1)
{
    auto rect = aabbCorners(x0, y0, x1, y1);

    std::vector<Pt> axes;
    axes.reserve(2 + poly.size());
    axes.push_back(Pt{1.0, 0.0});
    axes.push_back(Pt{0.0, 1.0});

    const int n = static_cast<int>(poly.size());
    for (int i = 0; i < n; i++)
    {
        const Pt &p0 = poly[i];
        const Pt &p1 = poly[(i + 1) % n];
        Pt e = sub(p1, p0);
        Pt axis{-e.y, e.x};
        if (std::abs(axis.x) < EPS && std::abs(axis.y) < EPS)
            continue;
        axes.push_back(axis);
    }

    for (const auto &ax : axes)
    {
        double pMin, pMax, rMin, rMax;
        projInterval(poly, ax, pMin, pMax);
        projInterval(rect, ax, rMin, rMax);
        if (!overlap1D(pMin, pMax, rMin, rMax))
            return false;
    }
    return true;
}

std::vector<std::pair<int, int>> rasterizePolygonToCells(
    const std::vector<Pt> &poly, double cellSize)
{
    if (poly.empty())
    {
        return {};
    }

    double minx = std::numeric_limits<double>::infinity();
    double maxx = -std::numeric_limits<double>::infinity();
    double miny = std::numeric_limits<double>::infinity();
    double maxy = -std::numeric_limits<double>::infinity();

    for (const auto &p : poly)
    {
        minx = std::min(minx, p.x);
        maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y);
        maxy = std::max(maxy, p.y);
    }

    int ix0 = static_cast<int>(std::floor(minx / cellSize));
    int ix1 = static_cast<int>(std::ceil(maxx / cellSize)) - 1;
    int iy0 = static_cast<int>(std::floor(miny / cellSize));
    int iy1 = static_cast<int>(std::ceil(maxy / cellSize)) - 1;

    std::vector<std::pair<int, int>> cells;
    if (ix1 < ix0 || iy1 < iy0)
    {
        return cells;
    }

    const std::size_t estimated = static_cast<std::size_t>(ix1 - ix0 + 1) * static_cast<std::size_t>(iy1 - iy0 + 1);
    cells.reserve(estimated);

    // 다각형 방향(시계/반시계)
    double signedArea2 = 0.0;
    for (std::size_t i = 0; i < poly.size(); ++i)
    {
        const auto &a = poly[i];
        const auto &b = poly[(i + 1) % poly.size()];
        signedArea2 += (a.x * b.y - b.x * a.y);
    }
    const bool ccw = (signedArea2 >= 0.0);

    auto insideConvex = [&](double px, double py) -> bool
    {
        for (std::size_t i = 0; i < poly.size(); ++i)
        {
            const auto &a = poly[i];
            const auto &b = poly[(i + 1) % poly.size()];
            const double cross = (b.x - a.x) * (py - a.y) - (b.y - a.y) * (px - a.x);
            if (ccw)
            {
                if (cross < -EPS)
                    return false;
            }
            else
            {
                if (cross > EPS)
                    return false;
            }
        }
        return true;
    };

    for (int ix = ix0; ix <= ix1; ix++)
    {
        const double cx = (static_cast<double>(ix) + 0.5) * cellSize;
        for (int iy = iy0; iy <= iy1; iy++)
        {
            const double cy = (static_cast<double>(iy) + 0.5) * cellSize;
            if (insideConvex(cx, cy))
            {
                cells.push_back({ix, iy});
            }
        }
    }
    return cells;
}
} // namespace occupancy
