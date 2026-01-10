"""
Parameter-based curriculum learning callback.

Advances through training stages by updating environment parameters
(e.g., coverage targets, spawn radius, domain randomization).

Unlike scenario-based curriculum, this modifies continuous parameters
rather than switching discrete scenarios.
"""

import json
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback


class ParameterCurriculumCallback(BaseCallback):
    """
    Callback for parameter-based curriculum learning.

    Advances through stages by modifying environment parameters based on
    training progress (timesteps completed).

    Example:
        >>> stages = [
        ...     {"coverage_target_pct": 70.0, "spawn_area_radius": 30.0},
        ...     {"coverage_target_pct": 80.0, "spawn_area_radius": 40.0},
        ...     {"coverage_target_pct": 85.0, "spawn_area_radius": 50.0},
        ... ]
        >>> callback = ParameterCurriculumCallback(
        ...     env=raw_env,
        ...     stages=stages,
        ...     total_timesteps=2_000_000,
        ... )
    """

    def __init__(
        self,
        env,
        stages: List[Dict[str, Any]],
        total_timesteps: int,
        save_path: Optional[str] = None,
        verbose: int = 1,
    ):
        """
        Initialize the parameter curriculum callback.

        Args:
            env: The raw (unwrapped) training environment with set_config method
            stages: List of parameter dicts for each stage
            total_timesteps: Total training timesteps (for stage timing)
            save_path: Path to save curriculum progress
            verbose: Verbosity level
        """
        super().__init__(verbose)
        self.env = env
        self.stages = stages
        self.total_timesteps = total_timesteps
        self.save_path = Path(save_path) if save_path else None

        # Calculate stage boundaries (evenly distributed)
        self.num_stages = len(stages)
        self.stage_boundaries = [
            int((i + 1) * total_timesteps / self.num_stages)
            for i in range(self.num_stages)
        ]

        # State tracking
        self.current_stage_idx = 0
        self._stage_history: List[Dict[str, Any]] = []

    @property
    def current_stage(self) -> Dict[str, Any]:
        """Get current stage parameters."""
        return self.stages[self.current_stage_idx]

    def _on_training_start(self) -> None:
        """Called at training start."""
        # Try to load saved progress
        if self.save_path and self.save_path.exists():
            self._load_progress()

        # Apply initial stage parameters
        self._apply_stage_params()

        if self.verbose >= 1:
            print(f"\n[Curriculum] Starting parameter-based curriculum")
            print(f"  Total stages: {self.num_stages}")
            print(f"  Current stage: {self.current_stage_idx + 1}")
            print(f"  Stage boundaries: {self.stage_boundaries}")
            print(f"  Initial params: {self.current_stage}")

    def _on_step(self) -> bool:
        """Called after each step."""
        # Check if we should advance to next stage
        if self.current_stage_idx < self.num_stages - 1:
            next_boundary = self.stage_boundaries[self.current_stage_idx]
            if self.num_timesteps >= next_boundary:
                self._advance_stage()

        return True

    def _advance_stage(self) -> None:
        """Advance to the next curriculum stage."""
        # Record history
        self._stage_history.append({
            "stage_idx": self.current_stage_idx,
            "params": self.current_stage.copy(),
            "timesteps": self.num_timesteps,
        })

        # Advance
        self.current_stage_idx += 1

        # Apply new parameters
        self._apply_stage_params()

        if self.verbose >= 1:
            print(f"\n{'='*60}")
            print(f"[Curriculum] ADVANCING to stage {self.current_stage_idx + 1}/{self.num_stages}")
            print(f"  Timestep: {self.num_timesteps:,}")
            print(f"  New params: {self.current_stage}")
            print(f"{'='*60}\n")

        # Log to TensorBoard
        self.logger.record("curriculum/stage", self.current_stage_idx)
        for key, value in self.current_stage.items():
            if isinstance(value, (int, float, bool)):
                self.logger.record(f"curriculum/{key}", value)

        # Save progress
        if self.save_path:
            self._save_progress()

    def _apply_stage_params(self) -> None:
        """Apply current stage parameters to the environment."""
        params = self.current_stage

        # Check if environment has update methods
        if hasattr(self.env, 'set_config'):
            self.env.set_config(params)
        elif hasattr(self.env, 'config'):
            # Direct config update
            for key, value in params.items():
                if hasattr(self.env.config, key):
                    setattr(self.env.config, key, value)

        # Handle specific parameters that may need special treatment
        if hasattr(self.env, 'coverage_target_pct') and 'coverage_target_pct' in params:
            self.env.coverage_target_pct = params['coverage_target_pct']
        if hasattr(self.env, 'spawn_area_radius') and 'spawn_area_radius' in params:
            self.env.spawn_area_radius = params['spawn_area_radius']
        if hasattr(self.env, 'mass_randomization') and 'mass_randomization' in params:
            self.env.mass_randomization = params['mass_randomization']
        if hasattr(self.env, 'wind_enabled') and 'wind_enabled' in params:
            self.env.wind_enabled = params['wind_enabled']

    def _on_training_end(self) -> None:
        """Called at training end."""
        if self.save_path:
            self._save_progress()

        if self.verbose >= 1:
            print(f"\n[Curriculum] Training complete!")
            print(f"  Final stage: {self.current_stage_idx + 1}/{self.num_stages}")
            print(f"  Final params: {self.current_stage}")

            if self._stage_history:
                print(f"\n  Stage history:")
                for h in self._stage_history:
                    print(f"    Stage {h['stage_idx'] + 1}: {h['timesteps']:,} steps")

    def _save_progress(self) -> None:
        """Save curriculum progress."""
        if not self.save_path:
            return

        self.save_path.parent.mkdir(parents=True, exist_ok=True)

        progress = {
            "current_stage_idx": self.current_stage_idx,
            "stage_history": self._stage_history,
            "stages": self.stages,
            "total_timesteps": self.total_timesteps,
        }

        with open(self.save_path, "w") as f:
            json.dump(progress, f, indent=2)

    def _load_progress(self) -> None:
        """Load curriculum progress."""
        if not self.save_path or not self.save_path.exists():
            return

        try:
            with open(self.save_path) as f:
                progress = json.load(f)

            self.current_stage_idx = progress["current_stage_idx"]
            self._stage_history = progress.get("stage_history", [])

            if self.verbose >= 1:
                print(f"[Curriculum] Loaded progress from {self.save_path}")
                print(f"  Resuming at stage {self.current_stage_idx + 1}")

        except Exception as e:
            if self.verbose >= 1:
                print(f"[Curriculum] Warning: Could not load progress: {e}")

    def get_progress(self) -> Dict[str, Any]:
        """Get current curriculum progress."""
        return {
            "current_stage": self.current_stage_idx + 1,
            "total_stages": self.num_stages,
            "params": self.current_stage,
            "progress_pct": 100 * (self.current_stage_idx + 1) / self.num_stages,
        }
