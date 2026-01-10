# Changelog

## [1.0.0] - 2024-12-29

### Added
- Initial release of Flyby World Generator
- Terrain generation with Perlin noise
- PBR material system with forest floor textures
- HDRI skybox support
- Modular spawner architecture:
  - VegetationSpawner for trees and bushes
  - VehicleSpawner for cars and military vehicles
  - PeopleSpawner for pedestrians and groups
- WorldGenerator API for RL training integration
- Domain randomization support
- Extension UI for manual control

### Changed
- Refactored from original `company.hello.world` extension
- Renamed to `flyby.world_generator`
- Improved tree spawning with correct orientation (90° X + 180° Y rotation)
- Added proper scaling (50x base scale for tree models)

### Fixed
- Tree orientation for Y-up to Z-up conversion
- Ground material application with UV mapping
