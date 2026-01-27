#include <vector>
#include <set>
#include <cmath>
#include <utility>
#include <limits>

struct Pt {
  double x, y;
};

static inline double dot(const Pt& a, const Pt& b) {
  return a.x * b.x + a.y * b.y;
}

static inline Pt sub(const Pt& a, const Pt& b) {
  return Pt{a.x - b.x, a.y - b.y};
}

static inline void projInterval(const std::vector<Pt>& pts, const Pt& axis,
                                double& outMin, double& outMax) {
  outMin = std::numeric_limits<double>::infinity();
  outMax = -std::numeric_limits<double>::infinity();
  for (const auto& p : pts) {
    double v = dot(p, axis);
    if (v < outMin) outMin = v;
    if (v > outMax) outMax = v;
  }
}

static inline bool overlap1D(double aMin, double aMax, double bMin, double bMax) {
  constexpr double EPS = 1e-12;
  // "면적 겹침" 기준: 접촉만(선/점)인 경우는 false 처리
  // 접촉도 점유로 보려면 <= 를 < 로 바꾸세요.
  if (aMax <= bMin + EPS) return false;
  if (bMax <= aMin + EPS) return false;
  return true;
}

static inline std::vector<Pt> aabbCorners(double x0, double y0, double x1, double y1) {
  return {Pt{x0,y0}, Pt{x1,y0}, Pt{x1,y1}, Pt{x0,y1}};
}

static bool convexPolyIntersectsAabbSAT(const std::vector<Pt>& poly,
                                        double x0, double y0, double x1, double y1) {
  auto rect = aabbCorners(x0, y0, x1, y1);

  std::vector<Pt> axes;
  axes.reserve(2 + poly.size());
  axes.push_back(Pt{1.0, 0.0});
  axes.push_back(Pt{0.0, 1.0});

  const int n = static_cast<int>(poly.size());
  for (int i = 0; i < n; i++) {
    const Pt& p0 = poly[i];
    const Pt& p1 = poly[(i + 1) % n];
    Pt e = sub(p1, p0);
    Pt axis{-e.y, e.x}; // edge normal
    if (std::abs(axis.x) < 1e-12 && std::abs(axis.y) < 1e-12) continue;
    axes.push_back(axis);
  }

  for (const auto& ax : axes) {
    double pMin, pMax, rMin, rMax;
    projInterval(poly, ax, pMin, pMax);
    projInterval(rect, ax, rMin, rMax);
    if (!overlap1D(pMin, pMax, rMin, rMax)) return false;
  }
  return true;
}

static std::set<std::pair<int,int>> rasterizePolygonToCells(
    const std::vector<Pt>& poly, double cellSize) {

  double minx = std::numeric_limits<double>::infinity();
  double maxx = -std::numeric_limits<double>::infinity();
  double miny = std::numeric_limits<double>::infinity();
  double maxy = -std::numeric_limits<double>::infinity();

  for (const auto& p : poly) {
    minx = std::min(minx, p.x);
    maxx = std::max(maxx, p.x);
    miny = std::min(miny, p.y);
    maxy = std::max(maxy, p.y);
  }

  int ix0 = static_cast<int>(std::floor(minx / cellSize));
  int ix1 = static_cast<int>(std::ceil (maxx / cellSize)) - 1;
  int iy0 = static_cast<int>(std::floor(miny / cellSize));
  int iy1 = static_cast<int>(std::ceil (maxy / cellSize)) - 1;

  std::set<std::pair<int,int>> cells;
  for (int ix = ix0; ix <= ix1; ix++) {
    double x0 = ix * cellSize;
    double x1 = (ix + 1) * cellSize;
    for (int iy = iy0; iy <= iy1; iy++) {
      double y0 = iy * cellSize;
      double y1 = (iy + 1) * cellSize;
      if (convexPolyIntersectsAabbSAT(poly, x0, y0, x1, y1)) {
        cells.insert({ix, iy});
      }
    }
  }
  return cells;
}
