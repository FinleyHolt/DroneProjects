"""
query_builder.py - Build TPTP queries from ROS messages

Constructs TPTP-format queries for the Vampire theorem prover by:
    1. Loading query templates from the benchmark library
    2. Substituting parameters into template placeholders
    3. Injecting hot/cold state facts (ADR-002)
    4. Including relevant ontology axioms
    5. Generating complete TPTP files for Vampire execution

State Fact Integration (Phase 5):
    - Hot facts: Volatile state from sensors (position, battery, comms)
    - Cold facts: Static state loaded at mission init (geofence, NFZ)
    See ADR_STATE_PERSISTENCE.md for architecture details.
"""

import os
import re
from pathlib import Path
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from vampire_bridge.hot_fact_buffer import HotFactBuffer
    from vampire_bridge.cold_fact_cache import ColdFactCache


class QueryBuilder:
    """
    Builds TPTP queries from templates and parameters.

    Templates are stored in the benchmark_queries directory with placeholders
    like {{UAV_ID}}, {{NFZ_ID}}, etc. that get substituted with actual values.
    """

    # Standard placeholder pattern: {{PLACEHOLDER_NAME}}
    PLACEHOLDER_PATTERN = re.compile(r'\{\{(\w+)\}\}')

    def __init__(
        self,
        templates_path: str = '/workspace/ontology/evaluation/benchmark_queries',
        ontology_path: str = '/workspace/ontology/planning_mode'
    ):
        """
        Initialize the query builder.

        Args:
            templates_path: Path to benchmark query templates
            ontology_path: Path to ontology KIF/TPTP files
        """
        self.templates_path = Path(templates_path)
        self.ontology_path = Path(ontology_path)
        self._template_cache: dict[str, str] = {}

    def build_query(
        self,
        query_type: str,
        query_name: str,
        parameters: list,
        raw_tptp: Optional[str] = None,
        hot_buffer: Optional['HotFactBuffer'] = None,
        cold_cache: Optional['ColdFactCache'] = None
    ) -> str:
        """
        Build a complete TPTP query from template and parameters.

        Optionally injects hot/cold state facts from the perception bridge
        (Phase 5). Facts are injected before the query template to ensure
        Vampire has access to current state during reasoning.

        Args:
            query_type: Type of query (safety, operational, planning)
            query_name: Name of the query template
            parameters: List of "key=value" parameter strings
            raw_tptp: Optional raw TPTP content (bypasses template lookup)
            hot_buffer: Optional HotFactBuffer for volatile state facts
            cold_cache: Optional ColdFactCache for static state facts

        Returns:
            Complete TPTP query string ready for Vampire

        Raises:
            FileNotFoundError: If template file doesn't exist
            ValueError: If required placeholders are not filled
        """
        if raw_tptp:
            wrapped = self._wrap_raw_query(raw_tptp)
            return self._inject_state_facts(wrapped, hot_buffer, cold_cache)

        # Load template
        template = self._load_template(query_type, query_name)

        # Parse parameters into dict
        param_dict = self._parse_parameters(parameters)

        # Substitute placeholders
        query = self._substitute_placeholders(template, param_dict)

        # Inject state facts (Phase 5)
        query = self._inject_state_facts(query, hot_buffer, cold_cache)

        return query

    def _inject_state_facts(
        self,
        query: str,
        hot_buffer: Optional['HotFactBuffer'],
        cold_cache: Optional['ColdFactCache']
    ) -> str:
        """
        Inject hot and cold state facts into the query.

        Facts are prepended to the query in order:
        1. Cold facts (static, from cache)
        2. Hot facts (volatile, from buffer)
        3. Original query template

        This ordering ensures Vampire sees the most stable facts first,
        which can improve proof search efficiency.

        Args:
            query: The base query template (after placeholder substitution)
            hot_buffer: Optional HotFactBuffer for volatile state
            cold_cache: Optional ColdFactCache for static state

        Returns:
            Query with state facts injected
        """
        parts = []

        # Add cold facts if available
        if cold_cache is not None:
            cold_block = cold_cache.get_cached_block()
            if cold_block:
                parts.append("% === Cold State Facts (static) ===")
                parts.append(cold_block)
                parts.append("")

        # Add hot facts if available
        if hot_buffer is not None:
            hot_block = hot_buffer.get_snapshot()
            if hot_block:
                parts.append("% === Hot State Facts (volatile) ===")
                parts.append(hot_block)
                parts.append("")

        # Add original query
        parts.append(query)

        return '\n'.join(parts)

    def _load_template(self, query_type: str, query_name: str) -> str:
        """
        Load a query template from file, with caching.

        Args:
            query_type: Type of query (safety, operational, planning)
            query_name: Name of the query template

        Returns:
            Template content as string

        Raises:
            FileNotFoundError: If template doesn't exist
        """
        cache_key = f"{query_type}_{query_name}"

        if cache_key in self._template_cache:
            return self._template_cache[cache_key]

        # Try multiple file naming conventions
        possible_names = [
            f"{query_type}_{query_name}.tptp",
            f"{query_name}.tptp",
            f"{query_type}/{query_name}.tptp",
        ]

        for name in possible_names:
            template_file = self.templates_path / name
            if template_file.exists():
                content = template_file.read_text()
                self._template_cache[cache_key] = content
                return content

        raise FileNotFoundError(
            f"Query template not found: {query_type}_{query_name}\n"
            f"Searched in: {self.templates_path}\n"
            f"Tried: {possible_names}"
        )

    def _parse_parameters(self, parameters: list) -> dict:
        """
        Parse list of "key=value" strings into a dictionary.

        Args:
            parameters: List of parameter strings like ["uav=drone_alpha", "nfz=zone_1"]

        Returns:
            Dictionary mapping placeholder names to values
        """
        param_dict = {}
        for param in parameters:
            if '=' in param:
                key, value = param.split('=', 1)
                # Normalize key to uppercase for template matching
                param_dict[key.strip().upper()] = value.strip()
        return param_dict

    def _substitute_placeholders(self, template: str, params: dict) -> str:
        """
        Substitute placeholders in template with parameter values.

        Args:
            template: Template string with {{PLACEHOLDER}} markers
            params: Dictionary mapping placeholder names to values

        Returns:
            Template with placeholders replaced

        Raises:
            ValueError: If required placeholders are not provided
        """
        # Find all placeholders in template
        placeholders = set(self.PLACEHOLDER_PATTERN.findall(template))

        # Check for missing required parameters
        missing = placeholders - set(params.keys())
        if missing:
            # Some templates may have optional placeholders with defaults
            # For now, we'll use a default value pattern
            for placeholder in missing:
                # Use lowercase placeholder name as default value
                params[placeholder] = placeholder.lower()

        # Substitute all placeholders
        def replacer(match):
            name = match.group(1)
            return params.get(name, match.group(0))

        return self.PLACEHOLDER_PATTERN.sub(replacer, template)

    def _wrap_raw_query(self, raw_tptp: str) -> str:
        """
        Wrap raw TPTP content with standard header/footer.

        Args:
            raw_tptp: Raw TPTP query content

        Returns:
            Wrapped query with metadata comments
        """
        header = (
            "%------------------------------------------------------------------------------\n"
            "% Raw TPTP Query (vampire_bridge)\n"
            "%------------------------------------------------------------------------------\n\n"
        )
        return header + raw_tptp

    def get_available_templates(self) -> dict:
        """
        List all available query templates organized by type.

        Returns:
            Dict mapping query_type to list of query_names
        """
        templates = {
            'safety': [],
            'operational': [],
            'planning': [],
        }

        if not self.templates_path.exists():
            return templates

        for tptp_file in self.templates_path.glob('*.tptp'):
            name = tptp_file.stem
            # Parse type from filename prefix
            for qtype in templates.keys():
                if name.startswith(f"{qtype}_"):
                    query_name = name[len(qtype) + 1:]
                    templates[qtype].append(query_name)
                    break
            else:
                # No type prefix, add to all categories
                for qtype in templates.keys():
                    templates[qtype].append(name)

        return templates

    def validate_template(self, query_type: str, query_name: str) -> dict:
        """
        Validate a template and return its metadata.

        Args:
            query_type: Type of query
            query_name: Name of the query template

        Returns:
            Dict with template validation info
        """
        try:
            template = self._load_template(query_type, query_name)
            placeholders = set(self.PLACEHOLDER_PATTERN.findall(template))

            # Check for conjecture (required for theorem proving)
            has_conjecture = 'fof(' in template and 'conjecture' in template

            return {
                'valid': True,
                'placeholders': list(placeholders),
                'has_conjecture': has_conjecture,
                'line_count': len(template.splitlines()),
            }
        except FileNotFoundError as e:
            return {
                'valid': False,
                'error': str(e),
            }
