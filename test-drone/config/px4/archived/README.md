# Archived Custom Airframe

This directory contains the custom PX4 airframe configuration that was previously used for test-drone.

## Contents

- `4007_gz_test_drone_x500` - Custom airframe with T265 and D455 sensor parameters

## Why Archived?

Switched to using the standard PX4 x500 airframe (`gz_x500`) for simplicity and better compatibility with upstream PX4. The custom sensors (T265 visual odometry and D455 depth camera) were removed from the simulation model.

## Restoration

To restore the custom airframe:

1. Copy `4007_gz_test_drone_x500` back to `../` (parent directory)
2. Restore the custom model from `simulation/models/archived/test_drone_x500/`
3. Update `scripts/start_simulation.sh` to use `gz_test_drone_x500` instead of `gz_x500`
4. Rebuild PX4 to register the custom airframe target

## Date Archived

December 2, 2025
