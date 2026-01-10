"""
test_hot_fact_buffer.py - Unit tests for HotFactBuffer

Tests the thread-safe buffer for volatile state facts as specified
in ADR-002: State Persistence Architecture.
"""

import time
import threading
import pytest
from vampire_bridge.hot_fact_buffer import HotFactBuffer


class TestHotFactBuffer:
    """Tests for HotFactBuffer class."""

    def test_basic_update_and_snapshot(self):
        """Test basic fact update and snapshot retrieval."""
        buffer = HotFactBuffer(uav_id='test_uav')

        buffer.update('battery_status', status='battery_nominal')

        snapshot = buffer.get_snapshot()
        assert 'battery_status_state' in snapshot
        assert 'has_battery_status(test_uav, battery_nominal)' in snapshot

    def test_multiple_facts(self):
        """Test updating multiple facts and getting combined snapshot."""
        buffer = HotFactBuffer(uav_id='drone_alpha')

        buffer.update('battery_status', status='battery_low')
        buffer.update('comms_status', status='comms_operational')
        buffer.update('distance_to_home', dist='5.5')

        snapshot = buffer.get_snapshot()

        assert 'battery_status' in snapshot
        assert 'comms_status' in snapshot
        assert 'distance_to_home' in snapshot
        assert 'battery_low' in snapshot
        assert 'comms_operational' in snapshot
        assert '5.5' in snapshot

    def test_fact_update_replaces_previous(self):
        """Test that updating a fact replaces the previous value."""
        buffer = HotFactBuffer()

        buffer.update('battery_status', status='battery_nominal')
        buffer.update('battery_status', status='battery_low')

        snapshot = buffer.get_snapshot()

        # Should only contain the new value
        assert 'battery_low' in snapshot
        assert snapshot.count('battery_status_state') == 1

    def test_unknown_fact_type_returns_false(self):
        """Test that unknown fact types are rejected gracefully."""
        buffer = HotFactBuffer()

        result = buffer.update('unknown_fact_type', value='test')

        assert result is False
        assert len(buffer) == 0

    def test_missing_template_param_returns_false(self):
        """Test that missing template parameters are handled."""
        buffer = HotFactBuffer()

        # battery_status requires 'status' parameter
        result = buffer.update('battery_status', wrong_param='value')

        assert result is False
        assert len(buffer) == 0

    def test_raw_fact_update(self):
        """Test updating with pre-formed TPTP axiom."""
        buffer = HotFactBuffer()

        axiom = "fof(custom_fact, axiom, custom_predicate(arg1, arg2))."
        buffer.update_raw('custom', axiom)

        snapshot = buffer.get_snapshot()
        assert axiom in snapshot

    def test_source_topic_tracking(self):
        """Test that source topics are tracked in metadata."""
        buffer = HotFactBuffer()

        buffer.update(
            'battery_status',
            source_topic='/mavros/battery',
            status='battery_nominal'
        )

        # Source topic is tracked internally for debugging
        stats = buffer.get_stats()
        assert 'battery_status' in stats['fact_types']

    def test_stale_fact_detection(self):
        """Test detection of stale facts."""
        buffer = HotFactBuffer(default_stale_threshold=0.1)
        # Override battery_status threshold (default is 5.0s)
        buffer.set_stale_threshold('battery_status', 0.1)

        buffer.update('battery_status', status='battery_nominal')

        # Initially not stale
        stale = buffer.get_stale_facts()
        assert len(stale) == 0

        # Wait for staleness
        time.sleep(0.15)

        stale = buffer.get_stale_facts()
        assert 'battery_status' in stale

    def test_stale_threshold_override(self):
        """Test overriding staleness threshold."""
        buffer = HotFactBuffer(default_stale_threshold=10.0)

        buffer.update('battery_status', status='battery_nominal')

        # Not stale with default threshold
        stale = buffer.get_stale_facts()
        assert len(stale) == 0

        # Stale with overridden threshold
        time.sleep(0.05)
        stale = buffer.get_stale_facts(max_age_sec=0.01)
        assert 'battery_status' in stale

    def test_per_fact_stale_thresholds(self):
        """Test per-fact-type staleness thresholds."""
        buffer = HotFactBuffer()

        # Position has shorter default threshold than battery
        buffer.set_stale_threshold('position', 0.05)
        buffer.set_stale_threshold('battery_status', 1.0)

        buffer.update('position', pos_id='pos_0_0_0')
        buffer.update('battery_status', status='battery_nominal')

        time.sleep(0.1)

        stale = buffer.get_stale_facts()

        # Position should be stale, battery should not
        assert 'position' in stale
        assert 'battery_status' not in stale

    def test_get_fact_age(self):
        """Test getting age of specific fact."""
        buffer = HotFactBuffer()

        buffer.update('battery_status', status='battery_nominal')
        time.sleep(0.05)

        age = buffer.get_fact_age('battery_status')
        assert age is not None
        assert age >= 0.05
        assert age < 0.1

    def test_get_fact_age_missing(self):
        """Test getting age of non-existent fact."""
        buffer = HotFactBuffer()

        age = buffer.get_fact_age('nonexistent')
        assert age is None

    def test_is_fact_stale(self):
        """Test checking if specific fact is stale."""
        buffer = HotFactBuffer()
        buffer.set_stale_threshold('battery_status', 0.05)

        buffer.update('battery_status', status='battery_nominal')

        assert not buffer.is_fact_stale('battery_status')

        time.sleep(0.1)

        assert buffer.is_fact_stale('battery_status')

    def test_is_fact_stale_missing(self):
        """Test that missing facts are considered stale."""
        buffer = HotFactBuffer()

        assert buffer.is_fact_stale('nonexistent')

    def test_remove_fact(self):
        """Test removing a fact from buffer."""
        buffer = HotFactBuffer()

        buffer.update('battery_status', status='battery_nominal')
        assert len(buffer) == 1

        result = buffer.remove('battery_status')
        assert result is True
        assert len(buffer) == 0

        # Removing again returns False
        result = buffer.remove('battery_status')
        assert result is False

    def test_clear(self):
        """Test clearing all facts."""
        buffer = HotFactBuffer()

        buffer.update('battery_status', status='battery_nominal')
        buffer.update('comms_status', status='comms_operational')

        buffer.clear()

        assert len(buffer) == 0
        assert buffer.get_snapshot() == ''

    def test_empty_snapshot(self):
        """Test snapshot of empty buffer."""
        buffer = HotFactBuffer()

        snapshot = buffer.get_snapshot()
        assert snapshot == ''

    def test_snapshot_dict(self):
        """Test getting snapshot as dictionary."""
        buffer = HotFactBuffer()

        buffer.update('battery_status', status='battery_nominal')
        buffer.update('comms_status', status='comms_operational')

        snapshot_dict = buffer.get_snapshot_dict()

        assert 'battery_status' in snapshot_dict
        assert 'comms_status' in snapshot_dict
        assert 'battery_nominal' in snapshot_dict['battery_status']

    def test_uav_id_property(self):
        """Test UAV ID property."""
        buffer = HotFactBuffer(uav_id='custom_uav')
        assert buffer.uav_id == 'custom_uav'

    def test_statistics(self):
        """Test buffer statistics."""
        buffer = HotFactBuffer()

        stats = buffer.get_stats()
        assert stats['fact_count'] == 0
        assert stats['update_count'] == 0
        assert stats['snapshot_count'] == 0

        buffer.update('battery_status', status='battery_nominal')
        buffer.update('battery_status', status='battery_low')
        buffer.get_snapshot()

        stats = buffer.get_stats()
        assert stats['fact_count'] == 1
        assert stats['update_count'] == 2
        assert stats['snapshot_count'] == 1

    def test_thread_safety_concurrent_updates(self):
        """Test thread safety with concurrent updates."""
        buffer = HotFactBuffer()
        errors = []

        def updater(thread_id):
            try:
                for i in range(100):
                    buffer.update(
                        'battery_status',
                        status=f'status_{thread_id}_{i}'
                    )
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=updater, args=(i,)) for i in range(5)]

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0

    def test_thread_safety_concurrent_snapshots(self):
        """Test thread safety with concurrent snapshots during updates."""
        buffer = HotFactBuffer()
        errors = []
        snapshots = []

        def updater():
            try:
                for i in range(100):
                    buffer.update('battery_status', status=f'status_{i}')
            except Exception as e:
                errors.append(e)

        def reader():
            try:
                for _ in range(100):
                    snapshot = buffer.get_snapshot()
                    snapshots.append(snapshot)
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=updater),
            threading.Thread(target=reader),
            threading.Thread(target=reader),
        ]

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0
        # All snapshots should be valid (not corrupted)
        for snapshot in snapshots:
            if snapshot:  # Non-empty snapshots
                assert 'fof(' in snapshot
                assert ').' in snapshot

    def test_snapshot_atomicity(self):
        """Test that snapshot is atomic (all facts from same point in time)."""
        buffer = HotFactBuffer()

        # Add multiple related facts
        buffer.update('battery_status', status='battery_nominal')
        buffer.update('current_battery_level', level='85.0')

        snapshot = buffer.get_snapshot()

        # Both facts should be present
        lines = snapshot.strip().split('\n')
        assert len(lines) == 2

    def test_tptp_format_validity(self):
        """Test that generated TPTP facts are syntactically valid."""
        buffer = HotFactBuffer(uav_id='drone_alpha')

        buffer.update('battery_status', status='battery_nominal')
        buffer.update('distance_to_home', dist='3.5')
        buffer.update('comms_status', status='comms_operational')

        snapshot = buffer.get_snapshot()

        # Each line should be a valid TPTP fact
        for line in snapshot.strip().split('\n'):
            assert line.startswith('fof(')
            assert ', axiom, ' in line
            assert line.endswith(').')

    def test_latency_requirement(self):
        """Test that snapshot retrieval meets latency requirement (<0.5ms)."""
        buffer = HotFactBuffer()

        # Add typical number of facts
        buffer.update('battery_status', status='battery_nominal')
        buffer.update('current_battery_level', level='75.0')
        buffer.update('position', pos_id='pos_10_20_30')
        buffer.update('distance_to_home', dist='5.5')
        buffer.update('comms_status', status='comms_operational')
        buffer.update('armed_status', armed='true')
        buffer.update('flight_mode', mode='guided')

        # Measure snapshot time
        start = time.perf_counter()
        for _ in range(1000):
            buffer.get_snapshot()
        elapsed = (time.perf_counter() - start) / 1000 * 1000  # ms per call

        # Should be well under 0.5ms
        assert elapsed < 0.5, f"Snapshot latency {elapsed:.3f}ms exceeds 0.5ms budget"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
