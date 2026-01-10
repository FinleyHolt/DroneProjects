"""
Test running multiple episodes for stability.

These tests verify that environments can run multiple episodes
without memory leaks, crashes, or state corruption.
"""

import pytest
import numpy as np
import gymnasium as gym


class TestMultipleEpisodes:
    """Test running multiple consecutive episodes."""

    def test_mission_planner_100_episodes(self):
        """Run 100 episodes of mission planner."""
        import ontology_rl.envs

        env = gym.make('FlybyMissionPlanner-v0', config={'use_ros': False})

        total_episodes = 0
        total_steps = 0

        for episode in range(100):
            obs, info = env.reset()
            episode_steps = 0
            done = False

            while not done and episode_steps < 50:  # Limit steps per episode
                action = env.action_space.sample()
                obs, reward, terminated, truncated, info = env.step(action)
                episode_steps += 1
                done = terminated or truncated

            total_episodes += 1
            total_steps += episode_steps

        env.close()

        assert total_episodes == 100
        assert total_steps >= 100  # At least 1 step per episode

    def test_behavior_selector_50_episodes(self):
        """Run 50 episodes of behavior selector."""
        import ontology_rl.envs

        env = gym.make('FlybyBehaviorSelector-v0', config={'use_ros': False})

        for episode in range(50):
            obs, info = env.reset()
            done = False
            steps = 0

            while not done and steps < 30:
                action = env.action_space.sample()
                obs, reward, terminated, truncated, info = env.step(action)
                steps += 1
                done = terminated or truncated

        env.close()

    def test_trajectory_optimizer_50_episodes(self):
        """Run 50 episodes of trajectory optimizer."""
        import ontology_rl.envs

        env = gym.make('FlybyTrajectoryOptimizer-v0', config={'use_ros': False})

        for episode in range(50):
            obs, info = env.reset()
            done = False
            steps = 0

            while not done and steps < 30:
                action = env.action_space.sample()
                obs, reward, terminated, truncated, info = env.step(action)
                steps += 1
                done = terminated or truncated

        env.close()


class TestRewardAndInfo:
    """Test reward and info dict consistency."""

    def test_reward_in_range(self):
        """Test that rewards are in reasonable range."""
        import ontology_rl.envs

        env = gym.make('FlybyMissionPlanner-v0', config={'use_ros': False})
        env.reset()

        rewards = []
        for _ in range(50):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            rewards.append(reward)
            if terminated or truncated:
                env.reset()

        env.close()

        # Rewards should be finite
        assert all(np.isfinite(r) for r in rewards)

        # Check reward range (adjust based on your reward design)
        assert min(rewards) >= -200
        assert max(rewards) <= 200

    def test_info_contains_required_keys(self):
        """Test that info dict contains expected keys."""
        import ontology_rl.envs

        env = gym.make('FlybyMissionPlanner-v0', config={'use_ros': False})
        env.reset()

        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)

        expected_keys = ['step', 'safety_violations', 'battery']
        for key in expected_keys:
            assert key in info, f"Missing key '{key}' in info dict"

        env.close()


class TestOntologyConstraints:
    """Test ontology constraint enforcement."""

    def test_battery_tracking(self):
        """Test that battery depletes over time."""
        import ontology_rl.envs

        env = gym.make('FlybyMissionPlanner-v0', config={'use_ros': False})
        obs, info = env.reset()

        initial_battery = obs[3]  # Battery is index 3

        for _ in range(20):
            action = 0  # Move forward
            obs, reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                break

        final_battery = obs[3]

        # Battery should decrease (or stay same if not moving)
        assert final_battery <= initial_battery

        env.close()

    def test_progress_tracking(self):
        """Test that mission progress is tracked."""
        import ontology_rl.envs

        env = gym.make('FlybyMissionPlanner-v0', config={'use_ros': False})
        obs, info = env.reset()

        initial_progress = obs[4]  # Progress is index 4
        assert initial_progress == 0.0

        # Take some steps
        for _ in range(10):
            action = 0  # Next waypoint
            obs, reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                break

        final_progress = obs[4]

        # Progress should increase
        assert final_progress >= initial_progress

        env.close()


class TestStability:
    """Test environment stability."""

    def test_no_nan_observations(self):
        """Test that observations never contain NaN."""
        import ontology_rl.envs

        env = gym.make('FlybyMissionPlanner-v0', config={'use_ros': False})

        for _ in range(10):
            obs, info = env.reset()
            assert not np.any(np.isnan(obs)), "NaN in reset observation"

            for _ in range(50):
                action = env.action_space.sample()
                obs, reward, terminated, truncated, info = env.step(action)
                assert not np.any(np.isnan(obs)), "NaN in step observation"
                if terminated or truncated:
                    break

        env.close()

    def test_no_inf_observations(self):
        """Test that observations never contain Inf."""
        import ontology_rl.envs

        env = gym.make('FlybyTrajectoryOptimizer-v0', config={'use_ros': False})

        for _ in range(10):
            obs, info = env.reset()
            assert not np.any(np.isinf(obs)), "Inf in reset observation"

            for _ in range(30):
                action = env.action_space.sample()
                obs, reward, terminated, truncated, info = env.step(action)
                assert not np.any(np.isinf(obs)), "Inf in step observation"
                if terminated or truncated:
                    break

        env.close()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
