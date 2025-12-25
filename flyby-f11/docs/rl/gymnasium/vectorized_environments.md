# Gymnasium Vectorized Environments Documentation

## Overview

Gymnasium provides vectorized environments to run multiple independent environment copies in parallel, enabling "linear speed-up in the steps taken per second through sampling multiple sub-environments."

## Key Components

**Base Class: VectorEnv**
The foundational class for all vectorized environments provides batched operations for observations, rewards, terminations, truncations, and info dictionaries.

**Implementation Options:**
- `AsyncVectorEnv` - Asynchronous parallel execution
- `SyncVectorEnv` - Synchronous parallel execution

## Core Methods

The vectorized environment interface includes:

1. **`step(actions)`** - Processes batched actions and returns "(observations, rewards, terminations, truncations, infos)"

2. **`reset()`** - Initializes all parallel environments and "return[s] a batch of initial observations and info"

3. **`render()`** - "Returns the rendered frames from the parallel environments"

4. **`close()`** - Releases resources across all sub-environments

## Key Attributes

- `num_envs` - Count of parallel sub-environments
- `action_space` / `observation_space` - Batched spaces
- `single_action_space` / `single_observation_space` - Individual environment spaces

## Important Feature: Autoreset

Vector environments implement autoreset functionality to "avoid having to wait for all sub-environments to terminated before resetting." This is essential for correct training algorithm implementation.

## Creation

Use `gymnasium.make_vec()` as the vector environment equivalent to `gymnasium.make()` for convenient instantiation with vectorization configuration options.
