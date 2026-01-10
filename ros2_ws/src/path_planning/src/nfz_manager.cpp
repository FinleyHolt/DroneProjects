/**
 * @file nfz_manager.cpp
 * @brief Implementation of NFZ manager
 */

#include "path_planning/nfz_manager.hpp"

#include <cmath>
#include <algorithm>

namespace path_planning
{

NFZManager::NFZManager()
{
}

void NFZManager::setNFZs(const std::vector<geometry_msgs::msg::Polygon> & nfz_polygons)
{
  std::lock_guard<std::mutex> lock(mutex_);
  nfz_polygons_ = nfz_polygons;
}

void NFZManager::setThreatZones(const std::vector<flyby_msgs::msg::ThreatZone> & threat_zones)
{
  std::lock_guard<std::mutex> lock(mutex_);
  threat_zones_ = threat_zones;
}

void NFZManager::addNFZ(const geometry_msgs::msg::Polygon & polygon)
{
  std::lock_guard<std::mutex> lock(mutex_);
  nfz_polygons_.push_back(polygon);
}

void NFZManager::clearNFZs()
{
  std::lock_guard<std::mutex> lock(mutex_);
  nfz_polygons_.clear();
}

bool NFZManager::isPointInPolygon(
  double x, double y,
  const geometry_msgs::msg::Polygon & polygon) const
{
  // Ray casting algorithm
  int n = polygon.points.size();
  if (n < 3) {
    return false;
  }

  int crossings = 0;
  for (int i = 0; i < n; ++i) {
    int j = (i + 1) % n;

    double xi = polygon.points[i].x;
    double yi = polygon.points[i].y;
    double xj = polygon.points[j].x;
    double yj = polygon.points[j].y;

    if ((yi > y) != (yj > y)) {
      double x_intersect = (xj - xi) * (y - yi) / (yj - yi) + xi;
      if (x < x_intersect) {
        crossings++;
      }
    }
  }

  return (crossings % 2) == 1;
}

bool NFZManager::segmentsIntersect(
  double ax, double ay, double bx, double by,
  double cx, double cy, double dx, double dy) const
{
  // Check if segment AB intersects segment CD
  auto ccw = [](double px, double py, double qx, double qy, double rx, double ry) {
      return (ry - py) * (qx - px) > (qy - py) * (rx - px);
    };

  return (ccw(ax, ay, cx, cy, dx, dy) != ccw(bx, by, cx, cy, dx, dy)) &&
         (ccw(ax, ay, bx, by, cx, cy) != ccw(ax, ay, bx, by, dx, dy));
}

bool NFZManager::isInNFZ(double x, double y, double z) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Check altitude bounds first (fast)
  if (z < min_altitude_ || z > max_altitude_) {
    return true;  // Outside altitude bounds is invalid
  }

  // Check each NFZ polygon
  for (const auto & nfz : nfz_polygons_) {
    if (isPointInPolygon(x, y, nfz)) {
      return true;
    }
  }

  return false;
}

bool NFZManager::segmentIntersectsNFZ(
  double x1, double y1, double z1,
  double x2, double y2, double z2) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Check altitude bounds
  if (z1 < min_altitude_ || z1 > max_altitude_ ||
    z2 < min_altitude_ || z2 > max_altitude_)
  {
    return true;
  }

  // Check if endpoints are in NFZ
  for (const auto & nfz : nfz_polygons_) {
    if (isPointInPolygon(x1, y1, nfz) || isPointInPolygon(x2, y2, nfz)) {
      return true;
    }
  }

  // Check if segment crosses any NFZ edge
  for (const auto & nfz : nfz_polygons_) {
    int n = nfz.points.size();
    for (int i = 0; i < n; ++i) {
      int j = (i + 1) % n;
      if (segmentsIntersect(
          x1, y1, x2, y2,
          nfz.points[i].x, nfz.points[i].y,
          nfz.points[j].x, nfz.points[j].y))
      {
        return true;
      }
    }
  }

  return false;
}

double NFZManager::getThreatCost(double x, double y, double z) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  double total_cost = 1.0;  // Base cost

  for (const auto & threat : threat_zones_) {
    // Check if point is within threat zone altitude range
    if (z < threat.min_altitude_m || z > threat.max_altitude_m) {
      continue;
    }

    // Check if point is within threat zone polygon
    if (isPointInPolygon(x, y, threat.boundary)) {
      // Accumulate cost multiplier
      total_cost *= threat.cost_multiplier;
    }
  }

  return total_cost;
}

size_t NFZManager::getNFZCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return nfz_polygons_.size();
}

size_t NFZManager::getThreatZoneCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return threat_zones_.size();
}

void NFZManager::setAltitudeBounds(double min_alt, double max_alt)
{
  std::lock_guard<std::mutex> lock(mutex_);
  min_altitude_ = min_alt;
  max_altitude_ = max_alt;
}

bool NFZManager::isAltitudeValid(double z) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return z >= min_altitude_ && z <= max_altitude_;
}

}  // namespace path_planning
