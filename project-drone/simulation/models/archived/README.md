# Archived Custom Models

This directory contains custom Gazebo models that were previously used in test-drone simulation.

## Contents

- `test_drone_x500/` - Custom x500 quadcopter model with T265 and D455 sensors

## Why Archived?

Switched to using the standard PX4 x500 model for simplicity. The custom model included:
- Intel RealSense T265 visual odometry camera (dual fisheye + IMU)
- Intel RealSense D455 depth camera (RGB + depth + dual IR)

These sensors were removed to use the standard PX4 SITL x500 airframe without custom modifications.

## Restoration

To restore the custom model:

1. Move `test_drone_x500/` back to `../` (parent models directory)
2. Restore the custom airframe from `config/px4/archived/4007_gz_test_drone_x500`
3. Update `scripts/start_simulation.sh` to use `gz_test_drone_x500` instead of `gz_x500`
4. Ensure the model is symlinked in the PX4 Gazebo models directory

## Date Archived

December 2, 2025
