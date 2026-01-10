#!/usr/bin/env python3
"""
Minimal test to isolate Isaac Sim crash.
Run with: /isaac-sim/python.sh /workspace/training/scripts/test_minimal_isaac.py
"""

import sys
from pathlib import Path

print("Step 1: Setting up paths...")
_script_dir = Path(__file__).parent.absolute()
_training_dir = _script_dir.parent
_eval_dir = _training_dir.parent
sys.path.insert(0, str(_training_dir))
sys.path.insert(0, str(_eval_dir))

print("Step 2: Importing isaacsim...")
from isaacsim import SimulationApp

print("Step 3: Creating SimulationApp...")
simulation_app = SimulationApp({"headless": True, "width": 1280, "height": 720})

print("Step 4: SimulationApp created successfully!")

print("Step 5: Importing torch...")
import torch
print(f"  torch version: {torch.__version__}")

print("Step 6: Importing numpy...")
import numpy as np
print(f"  numpy version: {np.__version__}")

print("Step 7: Importing stable_baselines3...")
from stable_baselines3 import PPO
print("  stable_baselines3 imported successfully")

print("Step 8: Importing training.environments...")
from training.environments import SearchTrainingEnv, SearchTrainingConfig
print("  training.environments imported successfully")

print("Step 9: Creating SearchTrainingConfig...")
config = SearchTrainingConfig(num_envs=4, device="cuda:0")
print(f"  Config created: {config.num_envs} envs")

print("Step 10: Creating SearchTrainingEnv...")
env = SearchTrainingEnv(config)
env.set_simulation_app(simulation_app)
print("  Environment created")

print("Step 11: Calling env.setup()...")
env.setup()
print("  Environment setup complete!")

print("\n" + "="*60)
print("ALL STEPS COMPLETED SUCCESSFULLY!")
print("="*60)

# Cleanup
env.close() if hasattr(env, 'close') else None
simulation_app.close()
