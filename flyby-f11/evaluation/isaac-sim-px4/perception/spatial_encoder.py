"""
Spatial grid encoder for perception observations.

Encodes ALL detections into an 8x8 spatial grid, ensuring no information loss
regardless of detection count. Essential for real-world scenarios with crowds,
convoys, or complex urban environments.
"""
import numpy as np
from typing import List, Tuple
from dataclasses import dataclass

try:
    from .detector import Detection
except ImportError:
    from detector import Detection


@dataclass
class SpatialGridConfig:
    """Configuration for spatial grid encoding."""
    grid_size: Tuple[int, int] = (8, 8)
    channels: int = 6  # person_count, obstacle_count, nearest_dist, motion_x, motion_y, max_priority
    max_count_per_cell: int = 10  # For normalization
    max_distance: float = 100.0   # meters


class SpatialGridEncoder:
    """
    Encodes detections into fixed-size spatial grid.

    Grid layout (8x8 cells, 6 channels each = 384 dims):
    - Channel 0: Person count (normalized)
    - Channel 1: Obstacle count (vehicles + buildings + trees)
    - Channel 2: Nearest detection distance (inverted: 0=far, 1=close)
    - Channel 3: Average motion vector X
    - Channel 4: Average motion vector Y
    - Channel 5: Highest priority in cell (1=person, lower=safer)

    This ensures ALL detections contribute to the observation, even with
    100+ objects in frame.
    """

    def __init__(self, config: SpatialGridConfig = None):
        self.config = config or SpatialGridConfig()
        self.grid_h, self.grid_w = self.config.grid_size
        self.channels = self.config.channels

        # Output dimensions: 8 x 8 x 6 = 384
        self.output_dim = self.grid_h * self.grid_w * self.channels

    def encode(self, detections: List[Detection]) -> np.ndarray:
        """
        Encode detections into spatial grid.

        Args:
            detections: List of Detection objects with bbox (normalized 0-1)

        Returns:
            Flat numpy array of shape (384,) with values in [0, 1]
        """
        # Initialize grid: (H, W, C)
        grid = np.zeros((self.grid_h, self.grid_w, self.channels), dtype=np.float32)

        # Track per-cell statistics
        cell_distances = [[[] for _ in range(self.grid_w)] for _ in range(self.grid_h)]
        cell_motions = [[[] for _ in range(self.grid_w)] for _ in range(self.grid_h)]
        cell_priorities = [[[] for _ in range(self.grid_w)] for _ in range(self.grid_h)]

        for det in detections:
            # Map bbox center to grid cell
            cx, cy = det.bbox[0], det.bbox[1]
            cell_x = min(int(cx * self.grid_w), self.grid_w - 1)
            cell_y = min(int(cy * self.grid_h), self.grid_h - 1)

            # Channel 0: Person count
            if det.class_id == 1:  # person
                grid[cell_y, cell_x, 0] += 1

            # Channel 1: Obstacle count (vehicles, buildings, trees, power lines)
            if det.class_id in [2, 3, 6, 7]:
                grid[cell_y, cell_x, 1] += 1

            # Track distance for this cell
            cell_distances[cell_y][cell_x].append(det.distance)

            # Track motion if available
            if det.velocity is not None:
                cell_motions[cell_y][cell_x].append(det.velocity[:2])

            # Track priority
            cell_priorities[cell_y][cell_x].append(det.priority)

        # Normalize counts
        grid[:, :, 0] = np.clip(grid[:, :, 0] / self.config.max_count_per_cell, 0, 1)
        grid[:, :, 1] = np.clip(grid[:, :, 1] / self.config.max_count_per_cell, 0, 1)

        # Compute per-cell aggregates
        for y in range(self.grid_h):
            for x in range(self.grid_w):
                # Channel 2: Nearest distance (inverted: 1 = close/dangerous)
                if cell_distances[y][x]:
                    min_dist = min(cell_distances[y][x])
                    grid[y, x, 2] = 1.0 - np.clip(min_dist / self.config.max_distance, 0, 1)

                # Channels 3-4: Average motion vector
                if cell_motions[y][x]:
                    avg_motion = np.mean(cell_motions[y][x], axis=0)
                    grid[y, x, 3] = np.clip(avg_motion[0] / 10.0, -1, 1)  # Normalize to +/-10 m/s
                    grid[y, x, 4] = np.clip(avg_motion[1] / 10.0, -1, 1)

                # Channel 5: Highest priority (lowest number = most important)
                if cell_priorities[y][x]:
                    min_priority = min(cell_priorities[y][x])
                    # Invert so 1.0 = highest priority (person), 0.0 = lowest
                    grid[y, x, 5] = 1.0 - (min_priority - 1) / 10.0

        # Flatten to 1D vector
        return grid.flatten()

    def decode(self, flat_grid: np.ndarray) -> np.ndarray:
        """
        Decode flat vector back to spatial grid (for visualization).

        Args:
            flat_grid: Array of shape (384,)

        Returns:
            Grid of shape (8, 8, 6)
        """
        return flat_grid.reshape(self.grid_h, self.grid_w, self.channels)

    def get_danger_cells(self, flat_grid: np.ndarray, threshold: float = 0.5) -> List[Tuple[int, int]]:
        """
        Get grid cells with high danger (persons nearby).

        Returns list of (row, col) tuples.
        """
        grid = self.decode(flat_grid)
        danger_cells = []

        for y in range(self.grid_h):
            for x in range(self.grid_w):
                # High person count or close proximity
                if grid[y, x, 0] > 0.1 or grid[y, x, 2] > threshold:
                    danger_cells.append((y, x))

        return danger_cells
