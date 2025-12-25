# Creating Custom Environments for Stable-Baselines3

## Gymnasium Interface Requirements

Custom environments must follow the Gymnasium interface and inherit from the Gym class. The documentation states that environments need to implement specific methods, with observations and actions defined using `gym.spaces` objects.

## Image Input Specifications

For vision-based tasks, the framework has particular requirements. As noted in the documentation: *"If you are using images as input, the observation must be of type `np.uint8` and be within a space `Box` bounded by [0, 255]"*

The library supports both channel-first and channel-last formats, though channel-first is recommended. When using pre-normalized images with CNN policies, you must set `normalize_images=False` through `policy_kwargs`.

## Environment Structure

A minimal custom environment requires:
- `__init__()`: Define action and observation spaces
- `step()`: Return observation, reward, terminated, truncated, and info
- `reset()`: Initialize and return observation with info
- `render()` and `close()`: Optional rendering and cleanup methods

## Validation and Registration

The framework provides a checking utility: *"from stable_baselines3.common.env_checker import check_env"* to verify environment compatibility. This tool identifies issues before training begins.

Environments can optionally be registered with Gymnasium using the `register()` function, enabling convenient instantiation via `gym.make()`.

## Compatibility Notes

SB3 doesn't support `Discrete` and `MultiDiscrete` spaces with non-zero start values. The documentation provides a wrapper solution to address this limitation.
