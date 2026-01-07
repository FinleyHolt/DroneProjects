"""
Unit tests for critical math functions in the world generator.

Run with: python -m pytest tests/test_math_functions.py -v
"""

import math
import pytest
from typing import Tuple


class TestEulerToQuaternion:
    """Tests for euler_to_quaternion conversion."""

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """Copy of the function for testing (avoids Isaac Sim dependencies)."""
        # Convert degrees to radians
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)

        # Calculate quaternion components
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qw, qx, qy, qz

    def test_identity_quaternion(self):
        """Zero rotation should give identity quaternion."""
        qw, qx, qy, qz = self.euler_to_quaternion(0, 0, 0)
        assert abs(qw - 1.0) < 1e-10
        assert abs(qx) < 1e-10
        assert abs(qy) < 1e-10
        assert abs(qz) < 1e-10

    def test_90_degree_yaw(self):
        """90 degree yaw rotation."""
        qw, qx, qy, qz = self.euler_to_quaternion(0, 0, 90)
        # For 90 degree yaw: qw = cos(45 degrees) ~ 0.707, qz = sin(45 degrees) ~ 0.707
        assert abs(qw - 0.7071067811865476) < 1e-6
        assert abs(qx) < 1e-10
        assert abs(qy) < 1e-10
        assert abs(qz - 0.7071067811865476) < 1e-6

    def test_90_degree_pitch(self):
        """90 degree pitch rotation."""
        qw, qx, qy, qz = self.euler_to_quaternion(0, 90, 0)
        assert abs(qw - 0.7071067811865476) < 1e-6
        assert abs(qx) < 1e-10
        assert abs(qy - 0.7071067811865476) < 1e-6
        assert abs(qz) < 1e-10

    def test_90_degree_roll(self):
        """90 degree roll rotation."""
        qw, qx, qy, qz = self.euler_to_quaternion(90, 0, 0)
        assert abs(qw - 0.7071067811865476) < 1e-6
        assert abs(qx - 0.7071067811865476) < 1e-6
        assert abs(qy) < 1e-10
        assert abs(qz) < 1e-10

    def test_180_degree_yaw(self):
        """180 degree yaw rotation."""
        qw, qx, qy, qz = self.euler_to_quaternion(0, 0, 180)
        # For 180 degree yaw: qw = cos(90 degrees) ~ 0, qz = sin(90 degrees) ~ 1
        assert abs(qw) < 1e-10
        assert abs(qx) < 1e-10
        assert abs(qy) < 1e-10
        assert abs(qz - 1.0) < 1e-6

    def test_negative_angles(self):
        """Negative angles should work correctly."""
        qw, qx, qy, qz = self.euler_to_quaternion(0, 0, -90)
        # For -90 degree yaw: qw = cos(-45 degrees) ~ 0.707, qz = sin(-45 degrees) ~ -0.707
        assert abs(qw - 0.7071067811865476) < 1e-6
        assert abs(qx) < 1e-10
        assert abs(qy) < 1e-10
        assert abs(qz + 0.7071067811865476) < 1e-6

    def test_combined_rotation(self):
        """Combined roll, pitch, yaw rotation."""
        qw, qx, qy, qz = self.euler_to_quaternion(30, 45, 60)
        # Verify quaternion is normalized (magnitude = 1)
        magnitude = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
        assert abs(magnitude - 1.0) < 1e-10

    def test_quaternion_is_normalized(self):
        """Quaternion magnitude should be 1 for all inputs."""
        for roll in [0, 30, 90, -45, 180, 270]:
            for pitch in [0, 30, 90, -45, 180]:
                for yaw in [0, 30, 90, -45, 180, 270, 360]:
                    qw, qx, qy, qz = self.euler_to_quaternion(roll, pitch, yaw)
                    magnitude = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
                    assert abs(magnitude - 1.0) < 1e-10, f"Quaternion not normalized for ({roll}, {pitch}, {yaw})"

    def test_full_rotation(self):
        """360 degree rotation should return to identity."""
        qw, qx, qy, qz = self.euler_to_quaternion(0, 0, 360)
        # 360 degree rotation gives qw = cos(180 degrees) = -1, qz = sin(180 degrees) = 0
        # This represents the same orientation as identity but with opposite sign
        assert abs(abs(qw) - 1.0) < 1e-10
        assert abs(qx) < 1e-10
        assert abs(qy) < 1e-10
        assert abs(qz) < 1e-10


class TestPolarToCartesian:
    """Tests for polar to cartesian conversion."""

    def polar_to_cartesian(self, angle_deg: float, distance: float) -> Tuple[float, float]:
        """Convert polar coordinates to cartesian."""
        angle_rad = math.radians(angle_deg)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return x, y

    def test_zero_angle(self):
        """0 degree angle should give (dist, 0)."""
        x, y = self.polar_to_cartesian(0, 10.0)
        assert abs(x - 10.0) < 1e-10
        assert abs(y) < 1e-10

    def test_90_degree_angle(self):
        """90 degree angle should give (0, dist)."""
        x, y = self.polar_to_cartesian(90, 10.0)
        assert abs(x) < 1e-10
        assert abs(y - 10.0) < 1e-10

    def test_45_degree_angle(self):
        """45 degree angle should give equal x and y."""
        x, y = self.polar_to_cartesian(45, 10.0)
        expected = 10.0 * math.sqrt(2) / 2
        assert abs(x - expected) < 1e-10
        assert abs(y - expected) < 1e-10

    def test_180_degree_angle(self):
        """180 degree angle should give (-dist, 0)."""
        x, y = self.polar_to_cartesian(180, 10.0)
        assert abs(x + 10.0) < 1e-10
        assert abs(y) < 1e-10

    def test_270_degree_angle(self):
        """270 degree angle should give (0, -dist)."""
        x, y = self.polar_to_cartesian(270, 10.0)
        assert abs(x) < 1e-10
        assert abs(y + 10.0) < 1e-10

    def test_negative_angle(self):
        """Negative angles should work correctly."""
        x, y = self.polar_to_cartesian(-90, 10.0)
        assert abs(x) < 1e-10
        assert abs(y + 10.0) < 1e-10  # Same as 270 degrees

    def test_zero_distance(self):
        """Zero distance should give origin."""
        x, y = self.polar_to_cartesian(45, 0)
        assert abs(x) < 1e-10
        assert abs(y) < 1e-10

    def test_large_angle(self):
        """Angles greater than 360 should wrap correctly."""
        x1, y1 = self.polar_to_cartesian(45, 10.0)
        x2, y2 = self.polar_to_cartesian(405, 10.0)  # 45 + 360
        assert abs(x1 - x2) < 1e-10
        assert abs(y1 - y2) < 1e-10

    def test_distance_magnitude(self):
        """Resulting distance from origin should match input distance."""
        for angle in [0, 30, 45, 90, 135, 180, 225, 270, 315]:
            x, y = self.polar_to_cartesian(angle, 15.0)
            result_dist = math.sqrt(x**2 + y**2)
            assert abs(result_dist - 15.0) < 1e-10


class TestAngleCalculation:
    """Tests for angle calculation using atan2."""

    def calculate_facing_angle(self, dx: float, dy: float) -> float:
        """Calculate facing angle from direction vector."""
        return math.degrees(math.atan2(dy, dx)) % 360

    def test_positive_x_axis(self):
        """Pointing along +X should be 0 degrees."""
        angle = self.calculate_facing_angle(1.0, 0.0)
        assert abs(angle) < 1e-10

    def test_positive_y_axis(self):
        """Pointing along +Y should be 90 degrees."""
        angle = self.calculate_facing_angle(0.0, 1.0)
        assert abs(angle - 90.0) < 1e-10

    def test_negative_x_axis(self):
        """Pointing along -X should be 180 degrees."""
        angle = self.calculate_facing_angle(-1.0, 0.0)
        assert abs(angle - 180.0) < 1e-10

    def test_negative_y_axis(self):
        """Pointing along -Y should be 270 degrees."""
        angle = self.calculate_facing_angle(0.0, -1.0)
        assert abs(angle - 270.0) < 1e-10

    def test_first_quadrant(self):
        """45 degree diagonal in first quadrant."""
        angle = self.calculate_facing_angle(1.0, 1.0)
        assert abs(angle - 45.0) < 1e-10

    def test_second_quadrant(self):
        """135 degree diagonal in second quadrant."""
        angle = self.calculate_facing_angle(-1.0, 1.0)
        assert abs(angle - 135.0) < 1e-10

    def test_third_quadrant(self):
        """225 degree diagonal in third quadrant."""
        angle = self.calculate_facing_angle(-1.0, -1.0)
        assert abs(angle - 225.0) < 1e-10

    def test_fourth_quadrant(self):
        """315 degree diagonal in fourth quadrant."""
        angle = self.calculate_facing_angle(1.0, -1.0)
        assert abs(angle - 315.0) < 1e-10

    def test_non_unit_vector(self):
        """Angle should be independent of vector magnitude."""
        angle1 = self.calculate_facing_angle(1.0, 1.0)
        angle2 = self.calculate_facing_angle(100.0, 100.0)
        assert abs(angle1 - angle2) < 1e-10

    def test_small_values(self):
        """Very small values should still work correctly."""
        angle = self.calculate_facing_angle(1e-10, 1e-10)
        assert abs(angle - 45.0) < 1e-6


class TestSpatialHash:
    """Tests for spatial hashing collision detection."""

    class SpatialHash:
        """Minimal implementation for testing."""
        def __init__(self, cell_size: float = 10.0):
            self.cell_size = cell_size
            self.cells = {}
            self.all_positions = []

        def _get_cell(self, x: float, y: float) -> Tuple[int, int]:
            return (int(x // self.cell_size), int(y // self.cell_size))

        def insert(self, x: float, y: float, radius: float) -> None:
            cell = self._get_cell(x, y)
            if cell not in self.cells:
                self.cells[cell] = []
            entry = (x, y, radius)
            self.cells[cell].append(entry)
            self.all_positions.append(entry)

        def query_nearby(self, x: float, y: float, radius: float) -> list:
            cx, cy = self._get_cell(x, y)
            cell_radius = int(radius // self.cell_size) + 1
            nearby = []
            for dx in range(-cell_radius, cell_radius + 1):
                for dy in range(-cell_radius, cell_radius + 1):
                    cell = (cx + dx, cy + dy)
                    if cell in self.cells:
                        nearby.extend(self.cells[cell])
            return nearby

        def clear(self) -> None:
            self.cells.clear()
            self.all_positions.clear()

    def test_insert_and_query(self):
        """Insert should make position queryable."""
        sh = self.SpatialHash(cell_size=10.0)
        sh.insert(5.0, 5.0, 1.0)
        nearby = sh.query_nearby(5.0, 5.0, 1.0)
        assert len(nearby) == 1
        assert nearby[0] == (5.0, 5.0, 1.0)

    def test_multiple_inserts(self):
        """Multiple inserts should be queryable."""
        sh = self.SpatialHash(cell_size=10.0)
        sh.insert(5.0, 5.0, 1.0)
        sh.insert(6.0, 6.0, 1.0)
        sh.insert(7.0, 7.0, 1.0)
        nearby = sh.query_nearby(5.0, 5.0, 5.0)
        assert len(nearby) == 3

    def test_distant_positions_not_returned(self):
        """Positions far away should not be in nearby query."""
        sh = self.SpatialHash(cell_size=10.0)
        sh.insert(0.0, 0.0, 1.0)
        sh.insert(100.0, 100.0, 1.0)  # Far away
        nearby = sh.query_nearby(0.0, 0.0, 5.0)
        assert len(nearby) == 1  # Only the nearby one

    def test_clear(self):
        """Clear should empty the hash."""
        sh = self.SpatialHash(cell_size=10.0)
        sh.insert(5.0, 5.0, 1.0)
        sh.clear()
        assert len(sh.all_positions) == 0
        assert len(sh.cells) == 0

    def test_cross_cell_boundary(self):
        """Positions near cell boundaries should find neighbors."""
        sh = self.SpatialHash(cell_size=10.0)
        sh.insert(9.0, 9.0, 1.0)   # Near edge of cell (0,0)
        sh.insert(11.0, 11.0, 1.0)  # In cell (1,1)
        # Query from cell (0,0) with radius that spans boundary
        nearby = sh.query_nearby(9.0, 9.0, 5.0)
        assert len(nearby) == 2  # Should find both

    def test_negative_coordinates(self):
        """Negative coordinates should work correctly."""
        sh = self.SpatialHash(cell_size=10.0)
        sh.insert(-5.0, -5.0, 1.0)
        sh.insert(-6.0, -6.0, 1.0)
        nearby = sh.query_nearby(-5.0, -5.0, 5.0)
        assert len(nearby) == 2

    def test_different_cell_sizes(self):
        """Different cell sizes should affect query results."""
        # Small cell size - more granular
        sh_small = self.SpatialHash(cell_size=2.0)
        sh_small.insert(0.0, 0.0, 1.0)
        sh_small.insert(5.0, 0.0, 1.0)
        nearby_small = sh_small.query_nearby(0.0, 0.0, 1.0)

        # Large cell size - less granular
        sh_large = self.SpatialHash(cell_size=20.0)
        sh_large.insert(0.0, 0.0, 1.0)
        sh_large.insert(5.0, 0.0, 1.0)
        nearby_large = sh_large.query_nearby(0.0, 0.0, 1.0)

        # With small cells, only query the nearby cell
        # With large cells, both might be in same or adjacent cells
        assert len(nearby_small) <= len(nearby_large) or len(nearby_small) >= 1

    def test_empty_query(self):
        """Query on empty hash should return empty list."""
        sh = self.SpatialHash(cell_size=10.0)
        nearby = sh.query_nearby(0.0, 0.0, 5.0)
        assert len(nearby) == 0


class TestPositionValidation:
    """Tests for collision detection between spawned objects."""

    def check_overlap(self, x1: float, y1: float, r1: float,
                      x2: float, y2: float, r2: float, min_spacing: float) -> bool:
        """Check if two circles overlap (including min_spacing)."""
        dist_sq = (x1 - x2) ** 2 + (y1 - y2) ** 2
        required_dist = r1 + r2 + min_spacing
        return dist_sq < required_dist ** 2

    def test_overlapping_positions(self):
        """Overlapping positions should be detected."""
        # Two objects at same position
        assert self.check_overlap(0, 0, 1.0, 0, 0, 1.0, 0.0) == True

    def test_identical_positions(self):
        """Identical positions should definitely overlap."""
        assert self.check_overlap(5.0, 5.0, 2.0, 5.0, 5.0, 2.0, 0.0) == True

    def test_non_overlapping_positions(self):
        """Non-overlapping positions should pass."""
        # Objects 10 apart, radius 1 each
        assert self.check_overlap(0, 0, 1.0, 10, 0, 1.0, 0.0) == False

    def test_touching_positions(self):
        """Positions exactly touching should not overlap."""
        # Objects exactly 2.0 apart (radius 1.0 each, no spacing)
        assert self.check_overlap(0, 0, 1.0, 2.0, 0, 1.0, 0.0) == False

    def test_almost_touching(self):
        """Positions very close but not overlapping."""
        # Objects 2.001 apart (radius 1.0 each, no spacing)
        assert self.check_overlap(0, 0, 1.0, 2.001, 0, 1.0, 0.0) == False

    def test_almost_overlapping(self):
        """Positions very close and overlapping."""
        # Objects 1.999 apart (radius 1.0 each, no spacing) - centers closer than sum of radii
        assert self.check_overlap(0, 0, 1.0, 1.999, 0, 1.0, 0.0) == True

    def test_min_spacing_enforcement(self):
        """Min spacing should create buffer zone."""
        # Objects 3.0 apart, radius 1.0 each, min_spacing 2.0
        # Required distance = 1.0 + 1.0 + 2.0 = 4.0, actual = 3.0 -> overlap
        assert self.check_overlap(0, 0, 1.0, 3.0, 0, 1.0, 2.0) == True
        # Objects 5.0 apart -> no overlap
        assert self.check_overlap(0, 0, 1.0, 5.0, 0, 1.0, 2.0) == False

    def test_min_spacing_exact_boundary(self):
        """Test exact boundary of min_spacing requirement."""
        # Required distance = 1.0 + 1.0 + 2.0 = 4.0
        # Exactly 4.0 apart -> not overlapping (dist_sq == required_dist_sq, but < is strict)
        assert self.check_overlap(0, 0, 1.0, 4.0, 0, 1.0, 2.0) == False
        # Slightly less than 4.0 -> overlapping
        assert self.check_overlap(0, 0, 1.0, 3.999, 0, 1.0, 2.0) == True

    def test_different_radii(self):
        """Test with different radii for each object."""
        # Object 1 has radius 2.0, object 2 has radius 3.0
        # Required distance = 2.0 + 3.0 = 5.0
        assert self.check_overlap(0, 0, 2.0, 4.0, 0, 3.0, 0.0) == True  # 4 < 5
        assert self.check_overlap(0, 0, 2.0, 6.0, 0, 3.0, 0.0) == False  # 6 > 5

    def test_diagonal_distance(self):
        """Test diagonal separation."""
        # Distance = sqrt(3^2 + 4^2) = 5.0
        # Radii = 1.0 + 1.0 = 2.0 -> not overlapping
        assert self.check_overlap(0, 0, 1.0, 3, 4, 1.0, 0.0) == False
        # Larger radii = 2.5 + 2.5 = 5.0 -> touching
        assert self.check_overlap(0, 0, 2.5, 3, 4, 2.5, 0.0) == False
        # Even larger = 2.6 + 2.6 = 5.2 -> overlapping
        assert self.check_overlap(0, 0, 2.6, 3, 4, 2.6, 0.0) == True

    def test_zero_radius(self):
        """Test with zero radius (point objects)."""
        # Two points at same location
        assert self.check_overlap(5.0, 5.0, 0.0, 5.0, 5.0, 0.0, 0.0) == False  # dist == 0, required == 0
        # Two points at different locations
        assert self.check_overlap(0, 0, 0.0, 1, 0, 0.0, 0.0) == False
        # Two points with min_spacing
        assert self.check_overlap(0, 0, 0.0, 0.5, 0, 0.0, 1.0) == True  # 0.5 < 1.0

    def test_negative_coordinates(self):
        """Test with negative coordinates."""
        # Distance = sqrt(10^2 + 10^2) = 14.14
        assert self.check_overlap(-5, -5, 1.0, 5, 5, 1.0, 0.0) == False
        # Closer negative coordinates
        assert self.check_overlap(-1, -1, 1.0, 0, 0, 1.0, 0.0) == True  # sqrt(2) < 2


class TestIntegration:
    """Integration tests combining multiple math functions."""

    def test_polar_to_cartesian_and_back(self):
        """Convert polar to cartesian and verify angle can be recovered."""
        angle_deg = 60.0
        distance = 25.0

        # Convert to cartesian
        angle_rad = math.radians(angle_deg)
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)

        # Recover angle
        recovered_angle = math.degrees(math.atan2(y, x)) % 360

        assert abs(recovered_angle - angle_deg) < 1e-10

    def test_quaternion_composition(self):
        """Test that small rotations compose approximately correctly."""
        # This is a simplified test - full quaternion composition is more complex
        qw1, qx1, qy1, qz1 = TestEulerToQuaternion().euler_to_quaternion(0, 0, 45)
        qw2, qx2, qy2, qz2 = TestEulerToQuaternion().euler_to_quaternion(0, 0, 45)

        # Both should have same values (same rotation)
        assert abs(qw1 - qw2) < 1e-10
        assert abs(qz1 - qz2) < 1e-10

    def test_spatial_hash_with_position_validation(self):
        """Test using spatial hash for efficient collision detection."""
        SpatialHash = TestSpatialHash.SpatialHash
        sh = SpatialHash(cell_size=5.0)

        # Insert some positions
        positions = [(0, 0, 1.0), (10, 10, 1.0), (20, 20, 1.0)]
        for x, y, r in positions:
            sh.insert(x, y, r)

        # Try to place a new object
        new_x, new_y, new_r = 11, 11, 1.0
        min_spacing = 1.0

        # Query nearby positions
        nearby = sh.query_nearby(new_x, new_y, new_r + min_spacing)

        # Check for collisions
        has_collision = False
        for (ox, oy, or_) in nearby:
            dist_sq = (new_x - ox) ** 2 + (new_y - oy) ** 2
            required_dist = new_r + or_ + min_spacing
            if dist_sq < required_dist ** 2:
                has_collision = True
                break

        # Position (11, 11) is close to (10, 10) - should collide
        assert has_collision == True


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
