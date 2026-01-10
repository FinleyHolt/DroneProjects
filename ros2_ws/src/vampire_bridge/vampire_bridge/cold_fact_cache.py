"""
cold_fact_cache.py - Cache for static/rarely-changing facts

Implements the ColdFactCache from ADR-002: State Persistence Architecture.
Stores facts that change rarely (<0.1Hz) such as geofence boundaries,
no-fly zone definitions, and mission parameters.

Design constraints (from ADR):
    - Loaded once at mission initialization
    - Memory budget: ~100KB for cold facts
    - Reloaded only on explicit mission change events
    - No file I/O during active flight queries
"""

import hashlib
import threading
from pathlib import Path
from typing import Optional


class ColdFactCache:
    """
    Cache for static/rarely-changing facts loaded from TPTP files.

    Cold facts are state that changes infrequently and can be loaded once
    at mission initialization. They are cached in memory to avoid file I/O
    during query construction, which is critical for meeting the 50ms
    latency budget.

    Typical cold facts:
        - Geofence boundary definitions
        - No-fly zone (NFZ) definitions
        - Mission parameters (home position, waypoints)
        - Static environment facts
    """

    def __init__(self, base_path: Optional[Path] = None):
        """
        Initialize the cold fact cache.

        Args:
            base_path: Base directory for cold fact TPTP files.
                       If None, files must be specified with absolute paths.
        """
        self._base_path = Path(base_path) if base_path else None
        self._cached_block: str = ""
        self._file_hashes: dict[str, str] = {}
        self._loaded_files: list[str] = []
        self._lock = threading.RLock()

        # Statistics
        self._load_count = 0
        self._reload_count = 0

        # Dynamic NFZ facts (can be updated via API)
        self._dynamic_facts: dict[str, str] = {}

    def load(self, file_list: list[str]) -> int:
        """
        Load and cache cold fact files.

        Reads each file, stores content hash for change detection, and
        assembles the cached block. Files are loaded relative to base_path
        if set, otherwise interpreted as absolute paths.

        Args:
            file_list: List of TPTP filenames to load

        Returns:
            Number of files successfully loaded

        Raises:
            FileNotFoundError: If base_path is set but doesn't exist
        """
        blocks = []
        loaded = 0

        with self._lock:
            self._file_hashes.clear()
            self._loaded_files = file_list.copy()

            for filename in file_list:
                filepath = self._resolve_path(filename)

                if filepath is None or not filepath.exists():
                    continue

                try:
                    content = filepath.read_text()
                    file_hash = hashlib.md5(content.encode()).hexdigest()
                    self._file_hashes[filename] = file_hash

                    # Add source comment and content
                    blocks.append(f"% Cold facts from: {filename}")
                    blocks.append(content.strip())
                    blocks.append("")  # Blank line separator
                    loaded += 1

                except (IOError, PermissionError):
                    # Skip files we can't read
                    continue

            self._cached_block = '\n'.join(blocks)
            self._load_count += 1

        return loaded

    def _resolve_path(self, filename: str) -> Optional[Path]:
        """
        Resolve filename to absolute path.

        Args:
            filename: Filename (relative or absolute)

        Returns:
            Resolved Path or None if cannot resolve
        """
        path = Path(filename)

        # If already absolute, use directly
        if path.is_absolute():
            return path

        # If base_path set, resolve relative to it
        if self._base_path:
            return self._base_path / filename

        return None

    def get_cached_block(self) -> str:
        """
        Return cached cold facts as TPTP block.

        Includes both file-loaded facts and dynamically added facts.
        Thread-safe for concurrent query access.

        Returns:
            Multi-line string of TPTP axioms
        """
        with self._lock:
            if not self._dynamic_facts:
                return self._cached_block

            # Combine static and dynamic facts
            dynamic_block = '\n'.join([
                f"% Dynamic cold facts",
                *self._dynamic_facts.values()
            ])

            if self._cached_block:
                return f"{self._cached_block}\n\n{dynamic_block}"
            return dynamic_block

    def needs_reload(self) -> bool:
        """
        Check if any source file has changed since load.

        Performs file I/O to check hashes - should only be called during
        mission transitions, not during active flight.

        Returns:
            True if any file changed and cache should be reloaded
        """
        with self._lock:
            for filename, old_hash in self._file_hashes.items():
                filepath = self._resolve_path(filename)
                if filepath is None:
                    continue

                if not filepath.exists():
                    # File was removed
                    return True

                try:
                    new_hash = hashlib.md5(
                        filepath.read_text().encode()
                    ).hexdigest()
                    if new_hash != old_hash:
                        return True
                except (IOError, PermissionError):
                    # Can't read file, assume changed
                    return True

        return False

    def reload(self) -> int:
        """
        Reload all previously loaded files.

        Should only be called during mission transitions or when
        needs_reload() returns True.

        Returns:
            Number of files successfully reloaded
        """
        with self._lock:
            if not self._loaded_files:
                return 0

            self._reload_count += 1
            return self.load(self._loaded_files)

    def add_nfz(self, nfz_id: str, vertices: list[tuple[float, float]]) -> None:
        """
        Add or update a dynamic no-fly zone.

        Use this for NFZs received at runtime (e.g., temporary flight
        restrictions, pop-up NFZs from ATC).

        Args:
            nfz_id: Unique identifier for the NFZ
            vertices: List of (lat, lon) tuples defining the boundary
        """
        # Create TPTP fact for NFZ
        vertex_str = ', '.join(
            f"vertex({v[0]:.6f}, {v[1]:.6f})" for v in vertices
        )
        axiom = f"fof(nfz_{nfz_id}, axiom, no_fly_zone({nfz_id}, [{vertex_str}]))."

        with self._lock:
            self._dynamic_facts[f"nfz_{nfz_id}"] = axiom

    def remove_nfz(self, nfz_id: str) -> bool:
        """
        Remove a dynamic no-fly zone.

        Args:
            nfz_id: Identifier of NFZ to remove

        Returns:
            True if NFZ was removed, False if not found
        """
        fact_key = f"nfz_{nfz_id}"
        with self._lock:
            if fact_key in self._dynamic_facts:
                del self._dynamic_facts[fact_key]
                return True
            return False

    def add_geofence(
        self,
        geofence_id: str,
        vertices: list[tuple[float, float]],
        max_altitude: Optional[float] = None
    ) -> None:
        """
        Add or update a dynamic geofence boundary.

        Args:
            geofence_id: Unique identifier for the geofence
            vertices: List of (lat, lon) tuples defining the boundary
            max_altitude: Optional maximum altitude in meters
        """
        vertex_str = ', '.join(
            f"vertex({v[0]:.6f}, {v[1]:.6f})" for v in vertices
        )

        if max_altitude is not None:
            axiom = (
                f"fof(geofence_{geofence_id}, axiom, "
                f"geofence_boundary({geofence_id}, [{vertex_str}], {max_altitude:.1f}))."
            )
        else:
            axiom = (
                f"fof(geofence_{geofence_id}, axiom, "
                f"geofence_boundary({geofence_id}, [{vertex_str}]))."
            )

        with self._lock:
            self._dynamic_facts[f"geofence_{geofence_id}"] = axiom

    def add_mission_param(self, param_name: str, value: str) -> None:
        """
        Add or update a mission parameter fact.

        Args:
            param_name: Name of the parameter
            value: Parameter value (will be used as-is in TPTP)
        """
        axiom = f"fof(mission_{param_name}, axiom, mission_param({param_name}, {value}))."

        with self._lock:
            self._dynamic_facts[f"mission_{param_name}"] = axiom

    def invalidate(self, fact_pattern: Optional[str] = None) -> int:
        """
        Invalidate cached facts matching a pattern.

        Args:
            fact_pattern: If provided, only invalidate facts whose key
                         contains this pattern. If None, clear all dynamic facts.

        Returns:
            Number of facts invalidated
        """
        with self._lock:
            if fact_pattern is None:
                count = len(self._dynamic_facts)
                self._dynamic_facts.clear()
                return count

            to_remove = [
                key for key in self._dynamic_facts
                if fact_pattern in key
            ]
            for key in to_remove:
                del self._dynamic_facts[key]
            return len(to_remove)

    def clear(self) -> None:
        """Clear all cached facts (both static and dynamic)."""
        with self._lock:
            self._cached_block = ""
            self._file_hashes.clear()
            self._loaded_files.clear()
            self._dynamic_facts.clear()

    def get_stats(self) -> dict:
        """
        Get cache statistics.

        Returns:
            Dict with cache metrics
        """
        with self._lock:
            return {
                'loaded_files': len(self._loaded_files),
                'file_list': self._loaded_files.copy(),
                'static_size_bytes': len(self._cached_block.encode()),
                'dynamic_fact_count': len(self._dynamic_facts),
                'load_count': self._load_count,
                'reload_count': self._reload_count,
            }

    def __len__(self) -> int:
        """Return number of loaded files plus dynamic facts."""
        with self._lock:
            return len(self._loaded_files) + len(self._dynamic_facts)
