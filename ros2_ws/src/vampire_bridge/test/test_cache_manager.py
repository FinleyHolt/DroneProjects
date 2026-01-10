"""
test_cache_manager.py - Unit tests for QueryCache

Tests the LRU cache with TTL functionality for Vampire query results.
"""

import time
import pytest
from vampire_bridge.cache_manager import QueryCache


class TestQueryCache:
    """Tests for QueryCache class."""

    def test_basic_put_get(self):
        """Test basic cache put and get operations."""
        cache = QueryCache(max_size=100, default_ttl=60.0)

        # Put a result
        cache.put('safety', 'nfz_check', ['uav=drone1'], 'result1')

        # Get it back
        result = cache.get('safety', 'nfz_check', ['uav=drone1'])
        assert result == 'result1'

    def test_cache_miss(self):
        """Test cache miss returns None."""
        cache = QueryCache(max_size=100)

        result = cache.get('safety', 'nonexistent', [])
        assert result is None

    def test_different_parameters(self):
        """Test that different parameters create different cache entries."""
        cache = QueryCache(max_size=100)

        cache.put('safety', 'nfz_check', ['uav=drone1'], 'result1')
        cache.put('safety', 'nfz_check', ['uav=drone2'], 'result2')

        assert cache.get('safety', 'nfz_check', ['uav=drone1']) == 'result1'
        assert cache.get('safety', 'nfz_check', ['uav=drone2']) == 'result2'

    def test_parameter_order_independence(self):
        """Test that parameter order doesn't affect cache key."""
        cache = QueryCache(max_size=100)

        cache.put('safety', 'check', ['a=1', 'b=2'], 'result')

        # Same parameters in different order should hit cache
        result = cache.get('safety', 'check', ['b=2', 'a=1'])
        assert result == 'result'

    def test_lru_eviction(self):
        """Test LRU eviction when cache is full."""
        cache = QueryCache(max_size=3)

        cache.put('safety', 'q1', [], 'r1')
        cache.put('safety', 'q2', [], 'r2')
        cache.put('safety', 'q3', [], 'r3')

        # Access q1 to make it recently used
        cache.get('safety', 'q1', [])

        # Add q4 - should evict q2 (least recently used)
        cache.put('safety', 'q4', [], 'r4')

        assert cache.get('safety', 'q1', []) == 'r1'  # Still present
        assert cache.get('safety', 'q2', []) is None  # Evicted
        assert cache.get('safety', 'q3', []) == 'r3'  # Still present
        assert cache.get('safety', 'q4', []) == 'r4'  # New entry

    def test_ttl_expiration(self):
        """Test that entries expire after TTL."""
        # Must override ttl_by_type to empty dict, otherwise 'safety' uses DEFAULT_TTL (10s)
        cache = QueryCache(max_size=100, default_ttl=0.1, ttl_by_type={})  # 100ms TTL

        cache.put('safety', 'query', [], 'result')
        assert cache.get('safety', 'query', []) == 'result'

        # Wait for expiration
        time.sleep(0.15)

        assert cache.get('safety', 'query', []) is None

    def test_ttl_by_query_type(self):
        """Test different TTL for different query types."""
        cache = QueryCache(
            max_size=100,
            ttl_by_type={
                'safety': 0.05,  # 50ms
                'planning': 0.2,  # 200ms
            }
        )

        cache.put('safety', 'q1', [], 'r1')
        cache.put('planning', 'q2', [], 'r2')

        # Wait 100ms
        time.sleep(0.1)

        # Safety should expire, planning should still be valid
        assert cache.get('safety', 'q1', []) is None
        assert cache.get('planning', 'q2', []) == 'r2'

    def test_invalidate_all(self):
        """Test invalidating entire cache."""
        cache = QueryCache(max_size=100)

        cache.put('safety', 'q1', [], 'r1')
        cache.put('operational', 'q2', [], 'r2')

        count = cache.invalidate()

        assert count == 2
        assert cache.get('safety', 'q1', []) is None
        assert cache.get('operational', 'q2', []) is None

    def test_invalidate_by_type(self):
        """Test invalidating by query type."""
        cache = QueryCache(max_size=100)

        cache.put('safety', 'q1', [], 'r1')
        cache.put('safety', 'q2', [], 'r2')
        cache.put('operational', 'q3', [], 'r3')

        count = cache.invalidate(query_type='safety')

        assert count == 2
        assert cache.get('safety', 'q1', []) is None
        assert cache.get('safety', 'q2', []) is None
        assert cache.get('operational', 'q3', []) == 'r3'

    def test_get_stats(self):
        """Test cache statistics reporting."""
        cache = QueryCache(max_size=100)

        # Initial stats
        stats = cache.get_stats()
        assert stats['size'] == 0
        assert stats['hits'] == 0
        assert stats['misses'] == 0

        # Add entry and access
        cache.put('safety', 'q1', [], 'r1')
        cache.get('safety', 'q1', [])  # Hit
        cache.get('safety', 'q2', [])  # Miss

        stats = cache.get_stats()
        assert stats['size'] == 1
        assert stats['hits'] == 1
        assert stats['misses'] == 1
        assert stats['hit_rate_percent'] == 50.0

    def test_clear(self):
        """Test clearing cache and stats."""
        cache = QueryCache(max_size=100)

        cache.put('safety', 'q1', [], 'r1')
        cache.get('safety', 'q1', [])

        cache.clear()

        stats = cache.get_stats()
        assert stats['size'] == 0
        assert stats['hits'] == 0
        assert stats['misses'] == 0

    def test_thread_safety(self):
        """Test cache operations are thread-safe."""
        import threading

        cache = QueryCache(max_size=1000)
        errors = []

        def writer(thread_id):
            try:
                for i in range(100):
                    cache.put('safety', f'q{thread_id}_{i}', [], f'r{thread_id}_{i}')
            except Exception as e:
                errors.append(e)

        def reader(thread_id):
            try:
                for i in range(100):
                    cache.get('safety', f'q{thread_id % 5}_{i}', [])
            except Exception as e:
                errors.append(e)

        threads = []
        for i in range(10):
            threads.append(threading.Thread(target=writer, args=(i,)))
            threads.append(threading.Thread(target=reader, args=(i,)))

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0

    def test_update_existing_entry(self):
        """Test updating an existing cache entry."""
        cache = QueryCache(max_size=100)

        cache.put('safety', 'q1', [], 'result1')
        cache.put('safety', 'q1', [], 'result2')

        result = cache.get('safety', 'q1', [])
        assert result == 'result2'

        # Should still only have one entry
        stats = cache.get_stats()
        assert stats['size'] == 1


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
