"""
test_cold_fact_cache.py - Unit tests for ColdFactCache

Tests the cache for static/rarely-changing facts as specified
in ADR-002: State Persistence Architecture.
"""

import os
import tempfile
import pytest
from pathlib import Path
from vampire_bridge.cold_fact_cache import ColdFactCache


class TestColdFactCache:
    """Tests for ColdFactCache class."""

    @pytest.fixture
    def temp_dir(self):
        """Create a temporary directory for test files."""
        with tempfile.TemporaryDirectory() as tmpdir:
            yield Path(tmpdir)

    @pytest.fixture
    def sample_tptp_files(self, temp_dir):
        """Create sample TPTP files for testing."""
        # Create geofence file
        geofence_content = """
% Geofence definitions
fof(geofence_main, axiom, geofence_boundary(main_fence, polygon_1)).
fof(geofence_max_alt, axiom, geofence_max_altitude(main_fence, 120.0)).
"""
        (temp_dir / 'geofence.tptp').write_text(geofence_content)

        # Create NFZ file
        nfz_content = """
% No-fly zone definitions
fof(nfz_airport, axiom, no_fly_zone(airport_nfz_1)).
fof(nfz_military, axiom, no_fly_zone(military_zone_1)).
"""
        (temp_dir / 'nfz_zones.tptp').write_text(nfz_content)

        return temp_dir

    def test_basic_load(self, sample_tptp_files):
        """Test basic file loading."""
        cache = ColdFactCache(base_path=sample_tptp_files)

        loaded = cache.load(['geofence.tptp', 'nfz_zones.tptp'])

        assert loaded == 2

        block = cache.get_cached_block()
        assert 'geofence_boundary' in block
        assert 'no_fly_zone' in block

    def test_load_with_source_comments(self, sample_tptp_files):
        """Test that loaded files have source comments."""
        cache = ColdFactCache(base_path=sample_tptp_files)
        cache.load(['geofence.tptp'])

        block = cache.get_cached_block()

        assert 'Cold facts from: geofence.tptp' in block

    def test_load_missing_file(self, temp_dir):
        """Test loading with missing files."""
        cache = ColdFactCache(base_path=temp_dir)

        loaded = cache.load(['nonexistent.tptp'])

        assert loaded == 0
        assert cache.get_cached_block() == ''

    def test_load_partial_missing(self, sample_tptp_files):
        """Test loading with some missing files."""
        cache = ColdFactCache(base_path=sample_tptp_files)

        loaded = cache.load(['geofence.tptp', 'nonexistent.tptp'])

        assert loaded == 1
        block = cache.get_cached_block()
        assert 'geofence_boundary' in block

    def test_needs_reload_unchanged(self, sample_tptp_files):
        """Test needs_reload when files unchanged."""
        cache = ColdFactCache(base_path=sample_tptp_files)
        cache.load(['geofence.tptp'])

        assert cache.needs_reload() is False

    def test_needs_reload_changed(self, sample_tptp_files):
        """Test needs_reload when file changes."""
        cache = ColdFactCache(base_path=sample_tptp_files)
        cache.load(['geofence.tptp'])

        # Modify the file
        (sample_tptp_files / 'geofence.tptp').write_text('% Modified content')

        assert cache.needs_reload() is True

    def test_needs_reload_deleted(self, sample_tptp_files):
        """Test needs_reload when file deleted."""
        cache = ColdFactCache(base_path=sample_tptp_files)
        cache.load(['geofence.tptp'])

        # Delete the file
        (sample_tptp_files / 'geofence.tptp').unlink()

        assert cache.needs_reload() is True

    def test_reload(self, sample_tptp_files):
        """Test reloading files."""
        cache = ColdFactCache(base_path=sample_tptp_files)
        cache.load(['geofence.tptp'])

        # Modify file
        (sample_tptp_files / 'geofence.tptp').write_text(
            "fof(new_fact, axiom, new_content)."
        )

        reloaded = cache.reload()

        assert reloaded == 1
        block = cache.get_cached_block()
        assert 'new_content' in block

    def test_add_dynamic_nfz(self):
        """Test adding dynamic no-fly zone."""
        cache = ColdFactCache()

        vertices = [(47.5, -122.3), (47.6, -122.3), (47.6, -122.4), (47.5, -122.4)]
        cache.add_nfz('temp_nfz_1', vertices)

        block = cache.get_cached_block()

        assert 'nfz_temp_nfz_1' in block
        assert 'no_fly_zone(temp_nfz_1' in block

    def test_remove_dynamic_nfz(self):
        """Test removing dynamic no-fly zone."""
        cache = ColdFactCache()

        vertices = [(47.5, -122.3), (47.6, -122.3)]
        cache.add_nfz('temp_nfz_1', vertices)
        cache.add_nfz('temp_nfz_2', vertices)

        result = cache.remove_nfz('temp_nfz_1')

        assert result is True
        block = cache.get_cached_block()
        assert 'temp_nfz_1' not in block
        assert 'temp_nfz_2' in block

    def test_remove_nonexistent_nfz(self):
        """Test removing non-existent NFZ."""
        cache = ColdFactCache()

        result = cache.remove_nfz('nonexistent')

        assert result is False

    def test_add_geofence(self):
        """Test adding dynamic geofence."""
        cache = ColdFactCache()

        vertices = [(47.5, -122.3), (47.6, -122.3), (47.6, -122.4)]
        cache.add_geofence('mission_fence', vertices, max_altitude=120.0)

        block = cache.get_cached_block()

        assert 'geofence_mission_fence' in block
        assert 'geofence_boundary(mission_fence' in block
        assert '120.0' in block

    def test_add_geofence_without_altitude(self):
        """Test adding geofence without altitude."""
        cache = ColdFactCache()

        vertices = [(47.5, -122.3), (47.6, -122.3)]
        cache.add_geofence('simple_fence', vertices)

        block = cache.get_cached_block()

        assert 'geofence_simple_fence' in block
        # Should not have altitude parameter
        assert 'geofence_boundary(simple_fence, [' in block

    def test_add_mission_param(self):
        """Test adding mission parameter."""
        cache = ColdFactCache()

        cache.add_mission_param('home_position', 'pos_47_122_0')
        cache.add_mission_param('max_speed', '15.0')

        block = cache.get_cached_block()

        assert 'mission_param(home_position, pos_47_122_0)' in block
        assert 'mission_param(max_speed, 15.0)' in block

    def test_combined_static_and_dynamic(self, sample_tptp_files):
        """Test combining static files with dynamic facts."""
        cache = ColdFactCache(base_path=sample_tptp_files)
        cache.load(['geofence.tptp'])

        # Add dynamic NFZ
        cache.add_nfz('popup_nfz', [(47.5, -122.3), (47.6, -122.3)])

        block = cache.get_cached_block()

        # Should contain both static and dynamic
        assert 'geofence_boundary' in block  # Static
        assert 'popup_nfz' in block  # Dynamic
        assert 'Dynamic cold facts' in block  # Dynamic section header

    def test_invalidate_all_dynamic(self):
        """Test invalidating all dynamic facts."""
        cache = ColdFactCache()

        cache.add_nfz('nfz_1', [(0, 0)])
        cache.add_nfz('nfz_2', [(0, 0)])
        cache.add_mission_param('param1', 'value1')

        count = cache.invalidate()

        assert count == 3
        assert cache.get_cached_block() == ''

    def test_invalidate_by_pattern(self):
        """Test invalidating dynamic facts by pattern."""
        cache = ColdFactCache()

        cache.add_nfz('nfz_1', [(0, 0)])
        cache.add_nfz('nfz_2', [(0, 0)])
        cache.add_mission_param('param1', 'value1')

        count = cache.invalidate(fact_pattern='nfz')

        assert count == 2
        block = cache.get_cached_block()
        assert 'nfz' not in block
        assert 'param1' in block

    def test_clear(self, sample_tptp_files):
        """Test clearing all facts."""
        cache = ColdFactCache(base_path=sample_tptp_files)
        cache.load(['geofence.tptp'])
        cache.add_nfz('temp_nfz', [(0, 0)])

        cache.clear()

        assert cache.get_cached_block() == ''
        assert len(cache) == 0

    def test_statistics(self, sample_tptp_files):
        """Test cache statistics."""
        cache = ColdFactCache(base_path=sample_tptp_files)

        stats = cache.get_stats()
        assert stats['loaded_files'] == 0
        assert stats['load_count'] == 0

        cache.load(['geofence.tptp', 'nfz_zones.tptp'])
        cache.add_nfz('temp_nfz', [(0, 0)])

        stats = cache.get_stats()
        assert stats['loaded_files'] == 2
        assert stats['dynamic_fact_count'] == 1
        assert stats['load_count'] == 1

    def test_reload_count(self, sample_tptp_files):
        """Test reload counter."""
        cache = ColdFactCache(base_path=sample_tptp_files)
        cache.load(['geofence.tptp'])

        assert cache.get_stats()['reload_count'] == 0

        cache.reload()

        assert cache.get_stats()['reload_count'] == 1

    def test_absolute_path_loading(self, sample_tptp_files):
        """Test loading with absolute paths."""
        cache = ColdFactCache()  # No base_path

        absolute_path = str(sample_tptp_files / 'geofence.tptp')
        loaded = cache.load([absolute_path])

        assert loaded == 1
        assert 'geofence_boundary' in cache.get_cached_block()

    def test_len(self, sample_tptp_files):
        """Test __len__ method."""
        cache = ColdFactCache(base_path=sample_tptp_files)

        assert len(cache) == 0

        cache.load(['geofence.tptp', 'nfz_zones.tptp'])
        assert len(cache) == 2

        cache.add_nfz('temp', [(0, 0)])
        assert len(cache) == 3

    def test_thread_safety(self, sample_tptp_files):
        """Test thread safety of cache operations."""
        import threading

        cache = ColdFactCache(base_path=sample_tptp_files)
        cache.load(['geofence.tptp'])
        errors = []

        def reader():
            try:
                for _ in range(100):
                    cache.get_cached_block()
            except Exception as e:
                errors.append(e)

        def writer():
            try:
                for i in range(50):
                    cache.add_nfz(f'nfz_{i}', [(0, 0)])
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=reader),
            threading.Thread(target=reader),
            threading.Thread(target=writer),
        ]

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0

    def test_empty_file(self, temp_dir):
        """Test loading empty file."""
        (temp_dir / 'empty.tptp').write_text('')

        cache = ColdFactCache(base_path=temp_dir)
        loaded = cache.load(['empty.tptp'])

        assert loaded == 1  # File was read, even if empty

    def test_tptp_format_validity(self, sample_tptp_files):
        """Test that dynamic facts produce valid TPTP."""
        cache = ColdFactCache()

        cache.add_nfz('test_nfz', [(47.5, -122.3), (47.6, -122.4)])
        cache.add_geofence('test_fence', [(47.5, -122.3)], max_altitude=100.0)
        cache.add_mission_param('test_param', 'value')

        block = cache.get_cached_block()

        # Each fact line should be valid TPTP
        for line in block.strip().split('\n'):
            if line.startswith('fof('):
                assert ', axiom, ' in line
                assert line.endswith(').')


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
