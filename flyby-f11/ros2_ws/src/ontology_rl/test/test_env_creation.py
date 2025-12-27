"""
Test Gymnasium environment creation and basic functionality.

These tests verify that the ontology_rl environments can be:
- Created via gym.make()
- Reset properly
- Step through episodes
- Handle action/observation spaces correctly

Tests run WITHOUT ROS 2 to allow standalone testing.
"""

import pytest
import numpy as np

# Import gymnasium and register environments
import gymnasium as gym


class TestEnvironmentRegistration:
    """Test that environments are properly registered with Gymnasium."""

    def test_mission_planner_registered(self):
        """Verify FlybyMissionPlanner-v0 is registered."""
        # Import to trigger registration
        import ontology_rl.envs

        env_ids = [spec.id for spec in gym.envs.registry.values()]
        assert 'FlybyMissionPlanner-v0' in env_ids, \
            "FlybyMissionPlanner-v0 not found in registered environments"

    def test_behavior_selector_registered(self):
        """Verify FlybyBehaviorSelector-v0 is registered."""
        import ontology_rl.envs

        env_ids = [spec.id for spec in gym.envs.registry.values()]
        assert 'FlybyBehaviorSelector-v0' in env_ids, \
            "FlybyBehaviorSelector-v0 not found in registered environments"

    def test_trajectory_optimizer_registered(self):
        """Verify FlybyTrajectoryOptimizer-v0 is registered."""
        import ontology_rl.envs

        env_ids = [spec.id for spec in gym.envs.registry.values()]
        assert 'FlybyTrajectoryOptimizer-v0' in env_ids, \
            "FlybyTrajectoryOptimizer-v0 not found in registered environments"


class TestMissionPlannerEnv:
    """Test MissionPlannerEnv functionality."""

    @pytest.fixture
    def env(self):
        """Create environment fixture."""
        import ontology_rl.envs
        env = gym.make('FlybyMissionPlanner-v0', config={'use_ros': False})
        yield env
        env.close()

    def test_env_creation(self, env):
        """Test environment can be created."""
        assert env is not None
        assert env.observation_space is not None
        assert env.action_space is not None

    def test_observation_space(self, env):
        """Test observation space is correctly defined."""
        assert isinstance(env.observation_space, gym.spaces.Box)
        assert env.observation_space.shape == (8,)
        assert env.observation_space.dtype == np.float32

    def test_action_space(self, env):
        """Test action space is correctly defined."""
        assert isinstance(env.action_space, gym.spaces.Discrete)
        assert env.action_space.n == 4

    def test_reset(self, env):
        """Test environment reset."""
        obs, info = env.reset(seed=42)

        assert obs is not None
        assert obs.shape == env.observation_space.shape
        assert env.observation_space.contains(obs)
        assert isinstance(info, dict)

    def test_step(self, env):
        """Test environment step."""
        env.reset(seed=42)

        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)

        assert obs.shape == env.observation_space.shape
        assert isinstance(reward, (int, float))
        assert isinstance(terminated, bool)
        assert isinstance(truncated, bool)
        assert isinstance(info, dict)

    def test_multiple_steps(self, env):
        """Test multiple consecutive steps."""
        env.reset(seed=42)

        for _ in range(100):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)

            if terminated or truncated:
                obs, info = env.reset()

    def test_deterministic_reset(self, env):
        """Test that reset with same seed produces same state."""
        obs1, _ = env.reset(seed=123)
        obs2, _ = env.reset(seed=123)

        np.testing.assert_array_equal(obs1, obs2)


class TestBehaviorSelectorEnv:
    """Test BehaviorSelectorEnv functionality."""

    @pytest.fixture
    def env(self):
        """Create environment fixture."""
        import ontology_rl.envs
        env = gym.make('FlybyBehaviorSelector-v0', config={'use_ros': False})
        yield env
        env.close()

    def test_env_creation(self, env):
        """Test environment can be created."""
        assert env is not None

    def test_observation_space(self, env):
        """Test observation space."""
        assert isinstance(env.observation_space, gym.spaces.Box)
        assert env.observation_space.shape == (12,)

    def test_action_space(self, env):
        """Test action space (6 behaviors)."""
        assert isinstance(env.action_space, gym.spaces.Discrete)
        assert env.action_space.n == 6

    def test_reset_and_step(self, env):
        """Test reset and step."""
        obs, info = env.reset(seed=42)
        assert obs.shape == env.observation_space.shape

        obs, reward, terminated, truncated, info = env.step(0)
        assert obs.shape == env.observation_space.shape


class TestTrajectoryOptimizerEnv:
    """Test TrajectoryOptimizerEnv functionality."""

    @pytest.fixture
    def env(self):
        """Create environment fixture."""
        import ontology_rl.envs
        env = gym.make('FlybyTrajectoryOptimizer-v0', config={'use_ros': False})
        yield env
        env.close()

    def test_env_creation(self, env):
        """Test environment can be created."""
        assert env is not None

    def test_observation_space(self, env):
        """Test observation space."""
        assert isinstance(env.observation_space, gym.spaces.Box)
        assert env.observation_space.shape == (15,)

    def test_action_space(self, env):
        """Test continuous action space."""
        assert isinstance(env.action_space, gym.spaces.Box)
        assert env.action_space.shape == (4,)

    def test_reset_and_step(self, env):
        """Test reset and step with continuous action."""
        obs, info = env.reset(seed=42)
        assert obs.shape == env.observation_space.shape

        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        assert obs.shape == env.observation_space.shape


class TestEpisodeRun:
    """Test running complete episodes."""

    def test_mission_planner_episode(self):
        """Test running a complete episode of mission planner."""
        import ontology_rl.envs

        env = gym.make('FlybyMissionPlanner-v0', config={'use_ros': False})
        obs, info = env.reset(seed=42)

        total_reward = 0
        steps = 0
        done = False

        while not done and steps < 100:
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            steps += 1
            done = terminated or truncated

        assert steps > 0
        assert 'step' in info
        env.close()

    def test_behavior_selector_episode(self):
        """Test running a complete episode of behavior selector."""
        import ontology_rl.envs

        env = gym.make('FlybyBehaviorSelector-v0', config={'use_ros': False})
        obs, info = env.reset(seed=42)

        steps = 0
        done = False

        while not done and steps < 100:
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            steps += 1
            done = terminated or truncated

        assert steps > 0
        env.close()

    def test_trajectory_optimizer_episode(self):
        """Test running a complete episode of trajectory optimizer."""
        import ontology_rl.envs

        env = gym.make('FlybyTrajectoryOptimizer-v0', config={'use_ros': False})
        obs, info = env.reset(seed=42)

        steps = 0
        done = False

        while not done and steps < 100:
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            steps += 1
            done = terminated or truncated

        assert steps > 0
        env.close()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
