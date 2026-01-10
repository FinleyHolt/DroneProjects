"""
cache_manager.py - LRU cache with TTL for Vampire query results

Provides memoization for repeated reasoning queries to reduce latency.
Cache hit should return results in <1ms vs ~50ms for Vampire execution.

Caching Strategy by Query Type:
    - safety: Short TTL (5-10s), high priority - state changes quickly
    - operational: Medium TTL (30-60s), state-dependent
    - planning: Long TTL (5-10 min), rarely changes during mission
"""

import hashlib
import time
import threading
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any, Optional


@dataclass
class CacheEntry:
    """Represents a cached query result with metadata."""
    result: Any
    timestamp: float
    query_type: str
    hit_count: int = 0


class QueryCache:
    """
    Thread-safe LRU cache with TTL for Vampire query results.

    Features:
        - LRU eviction when at capacity
        - TTL-based expiration (configurable per query type)
        - Thread-safe for concurrent ROS 2 callbacks
        - Cache statistics for monitoring
    """

    # Default TTL values by query type (seconds)
    DEFAULT_TTL = {
        'safety': 10.0,       # Short TTL - state changes quickly
        'operational': 60.0,  # Medium TTL
        'planning': 300.0,    # Long TTL - mission plans rarely change
    }

    def __init__(
        self,
        max_size: int = 1000,
        default_ttl: float = 60.0,
        ttl_by_type: Optional[dict] = None
    ):
        """
        Initialize the query cache.

        Args:
            max_size: Maximum number of entries to cache
            default_ttl: Default TTL in seconds for unknown query types
            ttl_by_type: Optional dict mapping query_type to TTL in seconds
        """
        self.max_size = max_size
        self.default_ttl = default_ttl
        self.ttl_by_type = ttl_by_type if ttl_by_type is not None else self.DEFAULT_TTL.copy()

        self._cache: OrderedDict[str, CacheEntry] = OrderedDict()
        self._lock = threading.RLock()

        # Statistics
        self._hits = 0
        self._misses = 0
        self._evictions = 0
        self._expirations = 0

    def _generate_key(
        self,
        query_type: str,
        query_name: str,
        parameters: list
    ) -> str:
        """
        Generate a unique cache key from query parameters.

        Args:
            query_type: Type of query (safety, operational, planning)
            query_name: Name of the query template
            parameters: List of query parameters

        Returns:
            MD5 hash of the query content as cache key
        """
        # Sort parameters for consistent key generation
        sorted_params = sorted(parameters) if parameters else []
        content = f"{query_type}:{query_name}:{':'.join(sorted_params)}"
        return hashlib.md5(content.encode()).hexdigest()

    def _get_ttl(self, query_type: str) -> float:
        """Get TTL for a given query type."""
        return self.ttl_by_type.get(query_type, self.default_ttl)

    def _is_expired(self, entry: CacheEntry, query_type: str) -> bool:
        """Check if a cache entry has expired."""
        ttl = self._get_ttl(query_type)
        return (time.time() - entry.timestamp) > ttl

    def get(
        self,
        query_type: str,
        query_name: str,
        parameters: list
    ) -> Optional[Any]:
        """
        Retrieve a cached result if available and not expired.

        Args:
            query_type: Type of query (safety, operational, planning)
            query_name: Name of the query template
            parameters: List of query parameters

        Returns:
            Cached result if found and valid, None otherwise
        """
        key = self._generate_key(query_type, query_name, parameters)

        with self._lock:
            if key not in self._cache:
                self._misses += 1
                return None

            entry = self._cache[key]

            # Check expiration
            if self._is_expired(entry, query_type):
                del self._cache[key]
                self._expirations += 1
                self._misses += 1
                return None

            # Move to end (LRU update)
            self._cache.move_to_end(key)
            entry.hit_count += 1
            self._hits += 1

            return entry.result

    def put(
        self,
        query_type: str,
        query_name: str,
        parameters: list,
        result: Any
    ) -> None:
        """
        Store a query result in the cache.

        Args:
            query_type: Type of query (safety, operational, planning)
            query_name: Name of the query template
            parameters: List of query parameters
            result: The result to cache
        """
        key = self._generate_key(query_type, query_name, parameters)

        with self._lock:
            # Remove existing entry if present
            if key in self._cache:
                del self._cache[key]

            # Evict oldest entries if at capacity
            while len(self._cache) >= self.max_size:
                self._cache.popitem(last=False)
                self._evictions += 1

            # Add new entry
            self._cache[key] = CacheEntry(
                result=result,
                timestamp=time.time(),
                query_type=query_type,
                hit_count=0
            )

    def invalidate(
        self,
        query_type: Optional[str] = None,
        query_name: Optional[str] = None
    ) -> int:
        """
        Invalidate cache entries matching the given criteria.

        Args:
            query_type: If provided, only invalidate entries of this type
            query_name: If provided, only invalidate entries with this name

        Returns:
            Number of entries invalidated
        """
        with self._lock:
            if query_type is None and query_name is None:
                # Clear entire cache
                count = len(self._cache)
                self._cache.clear()
                return count

            # Selective invalidation (less efficient, but flexible)
            to_remove = []
            for key, entry in self._cache.items():
                if query_type and entry.query_type != query_type:
                    continue
                # Note: We don't store query_name in entry, so can't filter by it
                # For full query_name filtering, would need to store more metadata
                to_remove.append(key)

            for key in to_remove:
                del self._cache[key]

            return len(to_remove)

    def get_stats(self) -> dict:
        """
        Get cache statistics.

        Returns:
            Dict with cache performance metrics
        """
        with self._lock:
            total_requests = self._hits + self._misses
            hit_rate = (self._hits / total_requests * 100) if total_requests > 0 else 0.0

            return {
                'size': len(self._cache),
                'max_size': self.max_size,
                'hits': self._hits,
                'misses': self._misses,
                'hit_rate_percent': round(hit_rate, 2),
                'evictions': self._evictions,
                'expirations': self._expirations,
            }

    def clear(self) -> None:
        """Clear all cache entries and reset statistics."""
        with self._lock:
            self._cache.clear()
            self._hits = 0
            self._misses = 0
            self._evictions = 0
            self._expirations = 0
