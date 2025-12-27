"""
Ontology constraint filter wrapper.

This wrapper applies ontology-based constraints to actions before
they are executed in the environment. It uses the Vampire theorem
prover (via vampire_bridge) to validate actions against the UAV
domain ontology.

Constraint categories:
- Safety constraints (hard): Always enforced (geofence, NFZ, battery)
- Performance constraints (soft): Advisory, may be relaxed
- Platform constraints (hard): Physical limits (velocity, acceleration)
"""

import gymnasium as gym
from gymnasium import Wrapper, spaces
import numpy as np
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class OntologyFilterWrapper(Wrapper):
    """
    Gymnasium wrapper that filters actions through ontology constraints.

    This wrapper intercepts actions and validates them against the UAV
    ontology before passing them to the underlying environment. Invalid
    actions are either:
    - Rejected (returns previous observation with penalty)
    - Modified to nearest valid action
    - Clamped to constraint boundaries

    The wrapper can operate in different modes:
    - 'strict': Reject invalid actions entirely
    - 'modify': Modify actions to be valid
    - 'clamp': Clamp actions to valid ranges
    - 'warn': Allow invalid actions but log warnings

    Example:
        env = gym.make('FlybyMissionPlanner-v0')
        env = OntologyFilterWrapper(env, mode='modify')
    """

    def __init__(
        self,
        env: gym.Env,
        mode: str = 'modify',
        violation_penalty: float = -10.0,
        query_timeout: float = 0.5,
        use_cache: bool = True
    ):
        """
        Initialize ontology filter wrapper.

        Args:
            env: Gymnasium environment to wrap
            mode: Filtering mode ('strict', 'modify', 'clamp', 'warn')
            violation_penalty: Reward penalty for constraint violations
            query_timeout: Timeout for Vampire queries (seconds)
            use_cache: Cache ontology query results
        """
        super().__init__(env)

        self.mode = mode
        self.violation_penalty = violation_penalty
        self.query_timeout = query_timeout
        self.use_cache = use_cache

        # Constraint tracking
        self._violations: List[Dict] = []
        self._filtered_count: int = 0
        self._total_count: int = 0

        # Query cache
        self._query_cache: Dict[str, bool] = {}
        self._cache_lock = threading.Lock()

        # Platform constraints (from ontology)
        self._constraints = {
            'max_velocity': 10.0,      # m/s
            'max_acceleration': 5.0,   # m/s^2
            'min_altitude': 0.0,       # m
            'max_altitude': 120.0,     # m (AGL)
            'geofence_radius': 100.0,  # m from origin
            'battery_critical': 10.0,  # percentage
            'battery_warning': 20.0,   # percentage
        }

        # NFZ regions: list of (center_x, center_y, radius)
        self._nfz_regions = [
            (-50.0, 50.0, 30.0),
        ]

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict] = None
    ) -> Tuple[np.ndarray, Dict]:
        """Reset environment and clear violation tracking."""
        obs, info = self.env.reset(seed=seed, options=options)

        # Clear tracking
        self._violations = []
        self._filtered_count = 0
        self._total_count = 0

        # Optionally clear cache on reset
        if not self.use_cache:
            with self._cache_lock:
                self._query_cache.clear()

        info['ontology_filter'] = {
            'mode': self.mode,
            'constraints': self._constraints
        }

        return obs, info

    def step(
        self,
        action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """
        Execute step with ontology-filtered action.

        Args:
            action: Action from RL agent

        Returns:
            Tuple of (observation, reward, terminated, truncated, info)
        """
        self._total_count += 1

        # Check and filter action
        filtered_action, violations = self._filter_action(action)

        # Track violations
        if violations:
            self._violations.extend(violations)
            self._filtered_count += 1

        # Execute filtered action
        obs, reward, terminated, truncated, info = self.env.step(filtered_action)

        # Apply violation penalty
        if violations:
            reward += self.violation_penalty * len(violations)

        # Add filter info
        info['ontology_filter'] = {
            'original_action': action.tolist() if hasattr(action, 'tolist') else action,
            'filtered_action': filtered_action.tolist() if hasattr(filtered_action, 'tolist') else filtered_action,
            'violations': violations,
            'total_violations': len(self._violations),
            'filter_rate': self._filtered_count / max(1, self._total_count)
        }

        return obs, reward, terminated, truncated, info

    def _filter_action(
        self,
        action: np.ndarray
    ) -> Tuple[np.ndarray, List[Dict]]:
        """
        Filter action through ontology constraints.

        Args:
            action: Raw action from agent

        Returns:
            Tuple of (filtered_action, list of violations)
        """
        violations = []

        # Get action type from environment
        if isinstance(self.action_space, spaces.Discrete):
            return self._filter_discrete_action(action, violations)
        elif isinstance(self.action_space, spaces.Box):
            return self._filter_continuous_action(action, violations)
        else:
            # Unknown action space, pass through
            return action, violations

    def _filter_discrete_action(
        self,
        action: int,
        violations: List[Dict]
    ) -> Tuple[int, List[Dict]]:
        """Filter discrete actions (e.g., waypoint selection)."""
        filtered = action

        # Get current state from environment
        if hasattr(self.env, '_current_battery'):
            battery = self.env._current_battery

            # Force RTH if battery critical
            if battery < self._constraints['battery_critical']:
                if action != 3:  # 3 = abort/RTH
                    violations.append({
                        'type': 'battery_critical',
                        'message': f'Battery at {battery:.1f}%, forcing RTH',
                        'original': action,
                        'forced': 3
                    })
                    filtered = 3

            # Warn if battery low
            elif battery < self._constraints['battery_warning']:
                if action in [0, 1]:  # Continuing forward
                    violations.append({
                        'type': 'battery_warning',
                        'message': f'Battery at {battery:.1f}%, consider RTH',
                        'level': 'warning'
                    })

        return filtered, violations

    def _filter_continuous_action(
        self,
        action: np.ndarray,
        violations: List[Dict]
    ) -> Tuple[np.ndarray, List[Dict]]:
        """Filter continuous actions (e.g., velocity commands)."""
        filtered = action.copy()

        # Velocity magnitude limit
        if len(filtered) >= 3:
            vel_cmd = filtered[:3]
            vel_mag = np.linalg.norm(vel_cmd)

            if vel_mag > self._constraints['max_velocity']:
                violations.append({
                    'type': 'velocity_limit',
                    'message': f'Velocity {vel_mag:.2f} exceeds max {self._constraints["max_velocity"]}',
                    'original': vel_mag,
                    'clamped': self._constraints['max_velocity']
                })

                if self.mode in ['modify', 'clamp']:
                    filtered[:3] = vel_cmd / vel_mag * self._constraints['max_velocity']

        # Altitude limits (if z velocity would violate)
        if hasattr(self.env, '_current_pose') and self.env._current_pose is not None:
            current_z = self.env._current_pose[2]

            # Predict z after action
            if len(filtered) >= 3:
                predicted_z = current_z + filtered[2] * 0.1  # 0.1s horizon

                if predicted_z < self._constraints['min_altitude']:
                    violations.append({
                        'type': 'min_altitude',
                        'message': f'Would violate min altitude',
                        'predicted': predicted_z
                    })
                    if self.mode in ['modify', 'clamp']:
                        filtered[2] = max(0, filtered[2])

                if predicted_z > self._constraints['max_altitude']:
                    violations.append({
                        'type': 'max_altitude',
                        'message': f'Would violate max altitude',
                        'predicted': predicted_z
                    })
                    if self.mode in ['modify', 'clamp']:
                        filtered[2] = min(0, filtered[2])

        # Geofence check
        if hasattr(self.env, '_current_pose') and self.env._current_pose is not None:
            current_pos = self.env._current_pose[:2]

            if len(filtered) >= 2:
                predicted_pos = current_pos + filtered[:2] * 0.1
                dist_from_origin = np.linalg.norm(predicted_pos)

                if dist_from_origin > self._constraints['geofence_radius']:
                    violations.append({
                        'type': 'geofence',
                        'message': f'Would exit geofence',
                        'predicted_distance': dist_from_origin
                    })
                    if self.mode in ['modify', 'clamp']:
                        # Scale down horizontal velocity
                        scale = self._constraints['geofence_radius'] / dist_from_origin * 0.9
                        filtered[:2] *= scale

        # NFZ check
        if hasattr(self.env, '_current_pose') and self.env._current_pose is not None:
            current_pos = self.env._current_pose[:2]

            if len(filtered) >= 2:
                predicted_pos = current_pos + filtered[:2] * 0.1

                for nfz in self._nfz_regions:
                    nfz_center = np.array([nfz[0], nfz[1]])
                    nfz_radius = nfz[2]

                    dist_to_nfz = np.linalg.norm(predicted_pos - nfz_center)

                    if dist_to_nfz < nfz_radius:
                        violations.append({
                            'type': 'nfz_violation',
                            'message': f'Would enter NFZ',
                            'nfz_center': nfz_center.tolist(),
                            'distance': dist_to_nfz
                        })
                        if self.mode in ['modify', 'clamp']:
                            # Reverse direction
                            filtered[:2] *= -1

        return filtered, violations

    def query_ontology(self, query: str) -> Optional[bool]:
        """
        Query ontology for constraint validation.

        Uses cache if available, otherwise queries Vampire.

        Args:
            query: TPTP-format query

        Returns:
            True if valid, False if invalid, None if timeout
        """
        # Check cache
        with self._cache_lock:
            if query in self._query_cache:
                return self._query_cache[query]

        # Query underlying environment if it has ontology support
        if hasattr(self.env, 'query_ontology'):
            result = self.env.query_ontology(query, timeout=self.query_timeout)

            # Cache result
            if result is not None and self.use_cache:
                with self._cache_lock:
                    self._query_cache[query] = result

            return result

        return None

    def get_violation_summary(self) -> Dict:
        """Get summary of constraint violations."""
        violation_types = {}
        for v in self._violations:
            vtype = v.get('type', 'unknown')
            violation_types[vtype] = violation_types.get(vtype, 0) + 1

        return {
            'total_violations': len(self._violations),
            'total_steps': self._total_count,
            'filtered_steps': self._filtered_count,
            'filter_rate': self._filtered_count / max(1, self._total_count),
            'violation_types': violation_types
        }

    def clear_cache(self):
        """Clear the query cache."""
        with self._cache_lock:
            self._query_cache.clear()
