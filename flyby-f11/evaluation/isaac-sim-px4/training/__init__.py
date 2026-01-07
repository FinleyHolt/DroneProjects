"""
Fast Training Package for Flyby F-11 ISR RL

This package provides optimized environments for faster-than-realtime RL training.
Key optimizations:
- Render frame skipping (Phase 1)
- Direct dynamics without PX4 SITL (Phase 2)
- Multi-drone vectorized environments (Phase 3)

Use evaluation/ package for full-fidelity PX4 SITL validation.
"""

__version__ = "0.1.0"
