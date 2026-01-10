"""
Curriculum learning callback for ISR training.

Manages progression through training stages:
1. area_coverage - Basic navigation and coverage
2. nfz_avoidance - Dynamic obstacle avoidance
3. multi_objective - Complex multi-priority ISR

Advances to next stage when success rate threshold is met.
"""

import json
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback


class CurriculumStage:
    """Configuration for a curriculum learning stage."""

    def __init__(
        self,
        scenario: str,
        success_threshold: float,
        min_episodes: int,
        description: str = "",
    ):
        """
        Initialize a curriculum stage.

        Args:
            scenario: Scenario name (area_coverage, nfz_avoidance, multi_objective)
            success_threshold: Success rate required to advance (0.0-1.0)
            min_episodes: Minimum episodes before advancement check
            description: Human-readable description
        """
        self.scenario = scenario
        self.success_threshold = success_threshold
        self.min_episodes = min_episodes
        self.description = description

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "scenario": self.scenario,
            "success_threshold": self.success_threshold,
            "min_episodes": self.min_episodes,
            "description": self.description,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CurriculumStage":
        """Create from dictionary."""
        return cls(
            scenario=data["scenario"],
            success_threshold=data["success_threshold"],
            min_episodes=data["min_episodes"],
            description=data.get("description", ""),
        )


class CurriculumCallback(BaseCallback):
    """
    Callback for managing curriculum learning across ISR scenarios.

    Tracks success rate and advances to harder scenarios when
    the current scenario is mastered.

    The callback:
    1. Monitors episode success rate (from ISRMetricsCallback or infos)
    2. Advances to next stage when threshold is met
    3. Saves curriculum progress for resuming
    4. Optionally calls a callback when stage advances

    Example:
        >>> from training.callbacks import CurriculumCallback
        >>>
        >>> stages = [
        ...     CurriculumStage("area_coverage", 0.7, 10000),
        ...     CurriculumStage("nfz_avoidance", 0.7, 10000),
        ...     CurriculumStage("multi_objective", 0.6, 20000),
        ... ]
        >>> callback = CurriculumCallback(
        ...     stages=stages,
        ...     on_stage_advance=lambda stage: print(f"Advanced to {stage.scenario}"),
        ... )
        >>> model.learn(total_timesteps=200_000_000, callback=callback)
    """

    # Default curriculum stages
    DEFAULT_STAGES = [
        CurriculumStage(
            scenario="area_coverage",
            success_threshold=0.7,
            min_episodes=10000,
            description="Basic navigation and area coverage",
        ),
        CurriculumStage(
            scenario="nfz_avoidance",
            success_threshold=0.7,
            min_episodes=10000,
            description="Transit with dynamic no-fly zones",
        ),
        CurriculumStage(
            scenario="multi_objective",
            success_threshold=0.6,
            min_episodes=20000,
            description="Multi-priority ISR with threat zones",
        ),
    ]

    def __init__(
        self,
        stages: Optional[List[CurriculumStage]] = None,
        check_freq: int = 10000,
        success_window: int = 1000,
        save_path: Optional[str] = None,
        on_stage_advance: Optional[Callable[[CurriculumStage], None]] = None,
        verbose: int = 1,
    ):
        """
        Initialize the curriculum callback.

        Args:
            stages: List of curriculum stages (uses defaults if None)
            check_freq: How often to check for advancement (in steps)
            success_window: Rolling window for success rate calculation
            save_path: Path to save curriculum progress (for resuming)
            on_stage_advance: Callback when advancing to next stage
            verbose: Verbosity level
        """
        super().__init__(verbose)
        self.stages = stages or self.DEFAULT_STAGES
        self.check_freq = check_freq
        self.success_window = success_window
        self.save_path = Path(save_path) if save_path else None
        self.on_stage_advance = on_stage_advance

        # Curriculum state
        self.current_stage_idx = 0
        self.stage_episode_count = 0
        self.stage_success_count = 0
        self.total_timesteps_at_stage_start = 0

        # Rolling success tracking
        from collections import deque
        self._recent_successes: deque = deque(maxlen=success_window)

        # History for logging
        self._stage_history: List[Dict[str, Any]] = []

    @property
    def current_stage(self) -> CurriculumStage:
        """Get the current curriculum stage."""
        return self.stages[self.current_stage_idx]

    @property
    def is_final_stage(self) -> bool:
        """Check if we're on the final stage."""
        return self.current_stage_idx >= len(self.stages) - 1

    @property
    def success_rate(self) -> float:
        """Get rolling success rate for current stage."""
        if not self._recent_successes:
            return 0.0
        return float(np.mean(self._recent_successes))

    def _on_training_start(self) -> None:
        """Called at training start."""
        # Try to load saved progress
        if self.save_path and self.save_path.exists():
            self._load_progress()

        if self.verbose >= 1:
            print(f"\n[Curriculum] Starting at stage {self.current_stage_idx + 1}/{len(self.stages)}")
            print(f"  Scenario: {self.current_stage.scenario}")
            print(f"  Success threshold: {self.current_stage.success_threshold:.0%}")
            print(f"  Min episodes: {self.current_stage.min_episodes}")

    def _on_step(self) -> bool:
        """Called after each step."""
        # Track episode completions
        dones = self.locals.get("dones", [])
        infos = self.locals.get("infos", [])

        for i, done in enumerate(dones):
            if done:
                self.stage_episode_count += 1
                info = infos[i] if i < len(infos) else {}

                # Check for success
                term_reason = info.get("termination_reason", "")
                is_success = term_reason == "mission_success"

                if is_success:
                    self.stage_success_count += 1
                self._recent_successes.append(1 if is_success else 0)

        # Check for stage advancement
        if self.n_calls % self.check_freq == 0:
            self._check_advancement()

        return True

    def _check_advancement(self) -> None:
        """Check if we should advance to next stage."""
        if self.is_final_stage:
            return

        stage = self.current_stage

        # Check minimum episodes
        if self.stage_episode_count < stage.min_episodes:
            if self.verbose >= 2:
                print(
                    f"[Curriculum] Stage {self.current_stage_idx + 1}: "
                    f"{self.stage_episode_count}/{stage.min_episodes} episodes, "
                    f"success rate: {self.success_rate:.1%}"
                )
            return

        # Check success threshold
        if self.success_rate >= stage.success_threshold:
            self._advance_stage()
        elif self.verbose >= 2:
            print(
                f"[Curriculum] Stage {self.current_stage_idx + 1}: "
                f"success rate {self.success_rate:.1%} < {stage.success_threshold:.0%}"
            )

    def _advance_stage(self) -> None:
        """Advance to the next curriculum stage."""
        # Record history
        self._stage_history.append({
            "stage_idx": self.current_stage_idx,
            "scenario": self.current_stage.scenario,
            "episodes": self.stage_episode_count,
            "successes": self.stage_success_count,
            "success_rate": self.success_rate,
            "timesteps": self.num_timesteps - self.total_timesteps_at_stage_start,
        })

        # Advance
        self.current_stage_idx += 1
        self.stage_episode_count = 0
        self.stage_success_count = 0
        self.total_timesteps_at_stage_start = self.num_timesteps
        self._recent_successes.clear()

        if self.verbose >= 1:
            print(f"\n{'='*60}")
            print(f"[Curriculum] ADVANCING to stage {self.current_stage_idx + 1}/{len(self.stages)}")
            print(f"  Scenario: {self.current_stage.scenario}")
            print(f"  Success threshold: {self.current_stage.success_threshold:.0%}")
            print(f"  Min episodes: {self.current_stage.min_episodes}")
            print(f"{'='*60}\n")

        # Log to TensorBoard
        self.logger.record("curriculum/stage", self.current_stage_idx)
        self.logger.record("curriculum/scenario", self.current_stage.scenario)

        # Save progress
        if self.save_path:
            self._save_progress()

        # Call user callback
        if self.on_stage_advance:
            self.on_stage_advance(self.current_stage)

    def _on_training_end(self) -> None:
        """Called at training end."""
        # Save final progress
        if self.save_path:
            self._save_progress()

        if self.verbose >= 1:
            print(f"\n[Curriculum] Training complete!")
            print(f"  Final stage: {self.current_stage_idx + 1}/{len(self.stages)}")
            print(f"  Scenario: {self.current_stage.scenario}")
            print(f"  Success rate: {self.success_rate:.1%}")

            if self._stage_history:
                print(f"\n  Stage history:")
                for h in self._stage_history:
                    print(
                        f"    {h['scenario']}: {h['episodes']} eps, "
                        f"{h['success_rate']:.1%} success, "
                        f"{h['timesteps']:,} steps"
                    )

    def _save_progress(self) -> None:
        """Save curriculum progress to file."""
        if not self.save_path:
            return

        self.save_path.parent.mkdir(parents=True, exist_ok=True)

        progress = {
            "current_stage_idx": self.current_stage_idx,
            "stage_episode_count": self.stage_episode_count,
            "stage_success_count": self.stage_success_count,
            "total_timesteps_at_stage_start": self.total_timesteps_at_stage_start,
            "stage_history": self._stage_history,
            "stages": [s.to_dict() for s in self.stages],
        }

        with open(self.save_path, "w") as f:
            json.dump(progress, f, indent=2)

        if self.verbose >= 2:
            print(f"[Curriculum] Progress saved to {self.save_path}")

    def _load_progress(self) -> None:
        """Load curriculum progress from file."""
        if not self.save_path or not self.save_path.exists():
            return

        try:
            with open(self.save_path) as f:
                progress = json.load(f)

            self.current_stage_idx = progress["current_stage_idx"]
            self.stage_episode_count = progress["stage_episode_count"]
            self.stage_success_count = progress["stage_success_count"]
            self.total_timesteps_at_stage_start = progress["total_timesteps_at_stage_start"]
            self._stage_history = progress.get("stage_history", [])

            if self.verbose >= 1:
                print(f"[Curriculum] Loaded progress from {self.save_path}")
                print(f"  Resuming at stage {self.current_stage_idx + 1}")

        except Exception as e:
            if self.verbose >= 1:
                print(f"[Curriculum] Warning: Could not load progress: {e}")

    def get_current_scenario(self) -> str:
        """Get the current scenario name."""
        return self.current_stage.scenario

    def get_progress(self) -> Dict[str, Any]:
        """
        Get current curriculum progress.

        Returns:
            Dict with progress information
        """
        return {
            "current_stage": self.current_stage_idx + 1,
            "total_stages": len(self.stages),
            "scenario": self.current_stage.scenario,
            "episodes": self.stage_episode_count,
            "min_episodes": self.current_stage.min_episodes,
            "success_rate": self.success_rate,
            "success_threshold": self.current_stage.success_threshold,
            "is_final_stage": self.is_final_stage,
        }
