/**
 * @file nfz_manager.hpp
 * @brief No-Fly Zone (NFZ) manager for path planning
 *
 * Manages NFZ polygons and provides collision checking for the planner.
 * Supports dynamic NFZ updates during mission execution.
 */

#ifndef PATH_PLANNING__NFZ_MANAGER_HPP_
#define PATH_PLANNING__NFZ_MANAGER_HPP_

#include <vector>
#include <mutex>
#include <memory>

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "flyby_msgs/msg/threat_zone.hpp"

namespace path_planning
{

/**
 * @brief Manages no-fly zones and threat zones for path planning
 *
 * Provides thread-safe access to NFZ data and collision checking methods
 * used by the OMPL state validity checker.
 */
class NFZManager
{
public:
  NFZManager();

  /**
   * @brief Set the list of NFZ polygons (hard constraints)
   * @param nfz_polygons Vector of polygon boundaries
   */
  void setNFZs(const std::vector<geometry_msgs::msg::Polygon> & nfz_polygons);

  /**
   * @brief Set the list of threat zones (soft constraints)
   * @param threat_zones Vector of threat zone definitions
   */
  void setThreatZones(const std::vector<flyby_msgs::msg::ThreatZone> & threat_zones);

  /**
   * @brief Add a single NFZ polygon
   * @param polygon NFZ boundary polygon
   */
  void addNFZ(const geometry_msgs::msg::Polygon & polygon);

  /**
   * @brief Remove all NFZs
   */
  void clearNFZs();

  /**
   * @brief Check if a 3D point is inside any NFZ
   * @param x X coordinate
   * @param y Y coordinate
   * @param z Z coordinate (altitude)
   * @return True if point is in an NFZ (invalid state)
   */
  bool isInNFZ(double x, double y, double z) const;

  /**
   * @brief Check if a path segment intersects any NFZ
   * @param x1, y1, z1 Start point
   * @param x2, y2, z2 End point
   * @return True if segment intersects an NFZ
   */
  bool segmentIntersectsNFZ(
    double x1, double y1, double z1,
    double x2, double y2, double z2) const;

  /**
   * @brief Get the cost multiplier for a point based on threat zones
   * @param x X coordinate
   * @param y Y coordinate
   * @param z Z coordinate (altitude)
   * @return Cost multiplier (1.0 = no threat, >1.0 = in threat zone)
   */
  double getThreatCost(double x, double y, double z) const;

  /**
   * @brief Get number of NFZs currently defined
   */
  size_t getNFZCount() const;

  /**
   * @brief Get number of threat zones currently defined
   */
  size_t getThreatZoneCount() const;

  /**
   * @brief Set altitude bounds for planning
   * @param min_alt Minimum altitude AGL (meters)
   * @param max_alt Maximum altitude AGL (meters)
   */
  void setAltitudeBounds(double min_alt, double max_alt);

  /**
   * @brief Check if altitude is within bounds
   * @param z Altitude to check
   * @return True if altitude is valid
   */
  bool isAltitudeValid(double z) const;

  /**
   * @brief Get minimum allowed altitude
   */
  double getMinAltitude() const { return min_altitude_; }

  /**
   * @brief Get maximum allowed altitude
   */
  double getMaxAltitude() const { return max_altitude_; }

private:
  /**
   * @brief Point-in-polygon test using ray casting
   */
  bool isPointInPolygon(
    double x, double y,
    const geometry_msgs::msg::Polygon & polygon) const;

  /**
   * @brief Line segment intersection test
   */
  bool segmentsIntersect(
    double ax, double ay, double bx, double by,
    double cx, double cy, double dx, double dy) const;

  mutable std::mutex mutex_;
  std::vector<geometry_msgs::msg::Polygon> nfz_polygons_;
  std::vector<flyby_msgs::msg::ThreatZone> threat_zones_;

  double min_altitude_{20.0};   // Minimum altitude AGL (meters)
  double max_altitude_{120.0};  // Maximum altitude AGL (meters)
};

}  // namespace path_planning

#endif  // PATH_PLANNING__NFZ_MANAGER_HPP_
