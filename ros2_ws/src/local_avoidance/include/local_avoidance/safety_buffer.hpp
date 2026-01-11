/**
 * @file safety_buffer.hpp
 * @brief Velocity-scaled safety buffer calculator for obstacle avoidance
 *
 * Computes dynamic safety margins that scale with vehicle velocity,
 * providing larger buffers at higher speeds for safer obstacle avoidance.
 */

#ifndef LOCAL_AVOIDANCE__SAFETY_BUFFER_HPP_
#define LOCAL_AVOIDANCE__SAFETY_BUFFER_HPP_

#include <algorithm>
#include <cmath>

namespace local_avoidance
{

/**
 * @brief Configuration for velocity-scaled safety buffers
 */
struct SafetyBufferConfig
{
  double base_buffer{2.0};       // Minimum buffer at zero velocity (meters)
  double reaction_time{1.0};     // Time-based scaling factor (seconds)
  double max_buffer{15.0};       // Maximum buffer cap (meters)
  double critical_ratio{0.5};    // Critical buffer = safety buffer * ratio
};

/**
 * @brief Computes velocity-scaled safety buffers
 *
 * Formula: buffer = base_buffer + velocity * reaction_time
 * Clamped to [base_buffer, max_buffer]
 *
 * The reaction_time parameter represents how far ahead the vehicle
 * should maintain clearance based on stopping/maneuvering capability.
 *
 * Example at default settings:
 * - Hover (0 m/s):  2.0m buffer
 * - Slow (5 m/s):   7.0m buffer
 * - Cruise (8 m/s): 10.0m buffer
 * - Fast (15 m/s):  15.0m buffer (capped)
 */
class SafetyBuffer
{
public:
  /**
   * @brief Constructor with configuration
   * @param config Safety buffer configuration
   */
  explicit SafetyBuffer(const SafetyBufferConfig & config = SafetyBufferConfig())
  : config_(config)
  {
  }

  /**
   * @brief Compute safety buffer for given velocity
   * @param velocity_mps Current velocity in meters per second
   * @return Safety buffer distance in meters
   */
  double computeSafetyBuffer(double velocity_mps) const
  {
    double velocity = std::max(0.0, velocity_mps);
    double buffer = config_.base_buffer + velocity * config_.reaction_time;
    return std::clamp(buffer, config_.base_buffer, config_.max_buffer);
  }

  /**
   * @brief Compute critical buffer for given velocity
   *
   * Critical buffer is the emergency stop distance - if an obstacle
   * is within this range, the vehicle should halt immediately.
   *
   * @param velocity_mps Current velocity in meters per second
   * @return Critical buffer distance in meters
   */
  double computeCriticalBuffer(double velocity_mps) const
  {
    return computeSafetyBuffer(velocity_mps) * config_.critical_ratio;
  }

  /**
   * @brief Compute both buffers at once
   * @param velocity_mps Current velocity in meters per second
   * @param[out] safety_buffer Safety buffer distance
   * @param[out] critical_buffer Critical buffer distance
   */
  void computeBuffers(
    double velocity_mps,
    double & safety_buffer,
    double & critical_buffer) const
  {
    safety_buffer = computeSafetyBuffer(velocity_mps);
    critical_buffer = safety_buffer * config_.critical_ratio;
  }

  /**
   * @brief Get current configuration
   */
  const SafetyBufferConfig & getConfig() const { return config_; }

  /**
   * @brief Update configuration
   */
  void setConfig(const SafetyBufferConfig & config) { config_ = config; }

  /**
   * @brief Get base buffer (minimum at zero velocity)
   */
  double getBaseBuffer() const { return config_.base_buffer; }

  /**
   * @brief Get maximum buffer cap
   */
  double getMaxBuffer() const { return config_.max_buffer; }

private:
  SafetyBufferConfig config_;
};

}  // namespace local_avoidance

#endif  // LOCAL_AVOIDANCE__SAFETY_BUFFER_HPP_
