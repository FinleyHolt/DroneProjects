"""
test_query_builder.py - Unit tests for QueryBuilder

Tests TPTP query construction from templates and parameters.
"""

import tempfile
from pathlib import Path
import pytest
from vampire_bridge.query_builder import QueryBuilder


class TestQueryBuilder:
    """Tests for QueryBuilder class."""

    @pytest.fixture
    def temp_templates_dir(self):
        """Create a temporary directory with test templates."""
        with tempfile.TemporaryDirectory() as tmpdir:
            templates_path = Path(tmpdir)

            # Create a simple template
            template_content = """
%% Test template
fof(test_axiom, axiom, uav({{UAV_ID}})).
fof(test_conjecture, conjecture, safe({{UAV_ID}})).
"""
            (templates_path / 'safety_test_query.tptp').write_text(template_content)

            # Create a template with multiple placeholders
            multi_template = """
fof(assignment, axiom, assigned({{UAV_ID}}, {{MISSION_ID}})).
fof(location, axiom, at({{UAV_ID}}, {{LOCATION}})).
"""
            (templates_path / 'operational_multi_param.tptp').write_text(multi_template)

            yield templates_path

    def test_basic_query_building(self, temp_templates_dir):
        """Test basic query building with parameter substitution."""
        builder = QueryBuilder(templates_path=str(temp_templates_dir))

        query = builder.build_query(
            'safety',
            'test_query',
            ['uav_id=drone_alpha']
        )

        assert 'uav(drone_alpha)' in query
        assert 'safe(drone_alpha)' in query
        assert '{{UAV_ID}}' not in query

    def test_multiple_parameters(self, temp_templates_dir):
        """Test query building with multiple parameters."""
        builder = QueryBuilder(templates_path=str(temp_templates_dir))

        query = builder.build_query(
            'operational',
            'multi_param',
            ['uav_id=drone_1', 'mission_id=mission_a', 'location=base']
        )

        assert 'assigned(drone_1, mission_a)' in query
        assert 'at(drone_1, base)' in query

    def test_parameter_case_insensitivity(self, temp_templates_dir):
        """Test that parameter keys are case-insensitive."""
        builder = QueryBuilder(templates_path=str(temp_templates_dir))

        query = builder.build_query(
            'safety',
            'test_query',
            ['UAV_ID=drone_upper', 'uav_id=drone_lower']  # Last wins
        )

        # Should use the last value
        assert 'drone_lower' in query or 'drone_upper' in query

    def test_template_not_found(self, temp_templates_dir):
        """Test error handling for missing templates."""
        builder = QueryBuilder(templates_path=str(temp_templates_dir))

        with pytest.raises(FileNotFoundError):
            builder.build_query('safety', 'nonexistent_query', [])

    def test_raw_tptp_bypass(self, temp_templates_dir):
        """Test that raw_tptp bypasses template lookup."""
        builder = QueryBuilder(templates_path=str(temp_templates_dir))

        raw_content = "fof(raw_test, axiom, test(x))."
        query = builder.build_query(
            'safety',
            'nonexistent',  # Would fail if template lookup happened
            [],
            raw_tptp=raw_content
        )

        assert raw_content in query

    def test_template_caching(self, temp_templates_dir):
        """Test that templates are cached after first load."""
        builder = QueryBuilder(templates_path=str(temp_templates_dir))

        # First load
        query1 = builder.build_query('safety', 'test_query', ['uav_id=d1'])

        # Modify the file (shouldn't affect cached version)
        template_file = temp_templates_dir / 'safety_test_query.tptp'
        template_file.write_text("modified content")

        # Second load should use cache
        query2 = builder.build_query('safety', 'test_query', ['uav_id=d2'])

        # Both should have the original template structure
        assert 'fof(test_axiom' in query1
        assert 'fof(test_axiom' in query2

    def test_get_available_templates(self, temp_templates_dir):
        """Test listing available templates."""
        builder = QueryBuilder(templates_path=str(temp_templates_dir))

        templates = builder.get_available_templates()

        assert 'test_query' in templates['safety']
        assert 'multi_param' in templates['operational']

    def test_validate_template(self, temp_templates_dir):
        """Test template validation."""
        builder = QueryBuilder(templates_path=str(temp_templates_dir))

        result = builder.validate_template('safety', 'test_query')

        assert result['valid'] is True
        assert 'UAV_ID' in result['placeholders']
        assert result['has_conjecture'] is True

    def test_validate_missing_template(self, temp_templates_dir):
        """Test validation of missing template."""
        builder = QueryBuilder(templates_path=str(temp_templates_dir))

        result = builder.validate_template('safety', 'nonexistent')

        assert result['valid'] is False
        assert 'error' in result

    def test_missing_placeholder_uses_default(self, temp_templates_dir):
        """Test that missing placeholders use default values."""
        builder = QueryBuilder(templates_path=str(temp_templates_dir))

        # Don't provide UAV_ID parameter
        query = builder.build_query('safety', 'test_query', [])

        # Should use lowercase placeholder name as default
        assert 'uav_id' in query.lower()

    def test_parameter_parsing(self, temp_templates_dir):
        """Test parameter string parsing."""
        builder = QueryBuilder(templates_path=str(temp_templates_dir))

        # Test various parameter formats
        params = [
            'key1=value1',
            'key2 = value2',  # With spaces
            'key3=value=with=equals',  # Value containing equals
        ]

        parsed = builder._parse_parameters(params)

        assert parsed['KEY1'] == 'value1'
        assert parsed['KEY2'] == 'value2'
        assert parsed['KEY3'] == 'value=with=equals'


class TestQueryBuilderEdgeCases:
    """Edge case tests for QueryBuilder."""

    def test_empty_parameters(self):
        """Test with empty parameter list."""
        with tempfile.TemporaryDirectory() as tmpdir:
            templates_path = Path(tmpdir)
            (templates_path / 'safety_simple.tptp').write_text("fof(ax, axiom, test).")

            builder = QueryBuilder(templates_path=str(templates_path))
            query = builder.build_query('safety', 'simple', [])

            assert 'fof(ax, axiom, test)' in query

    def test_special_characters_in_values(self):
        """Test parameter values with special characters."""
        with tempfile.TemporaryDirectory() as tmpdir:
            templates_path = Path(tmpdir)
            (templates_path / 'safety_special.tptp').write_text("name({{ID}}).")

            builder = QueryBuilder(templates_path=str(templates_path))
            query = builder.build_query(
                'safety',
                'special',
                ['id=drone_1_alpha']  # Underscores
            )

            assert 'name(drone_1_alpha)' in query


class TestQueryBuilderStateFactInjection:
    """Tests for state fact injection (Phase 5)."""

    @pytest.fixture
    def temp_templates_dir(self):
        """Create a temporary directory with test templates."""
        with tempfile.TemporaryDirectory() as tmpdir:
            templates_path = Path(tmpdir)
            template_content = "fof(test_query, conjecture, safe(drone))."
            (templates_path / 'safety_test.tptp').write_text(template_content)
            yield templates_path

    @pytest.fixture
    def temp_cold_facts_dir(self):
        """Create a temporary directory with cold fact files."""
        with tempfile.TemporaryDirectory() as tmpdir:
            cold_path = Path(tmpdir)
            cold_content = "fof(geofence, axiom, geofence_boundary(main, poly1))."
            (cold_path / 'geofence.tptp').write_text(cold_content)
            yield cold_path

    def test_hot_fact_injection(self, temp_templates_dir):
        """Test that hot facts are injected into query."""
        from vampire_bridge.hot_fact_buffer import HotFactBuffer

        builder = QueryBuilder(templates_path=str(temp_templates_dir))
        hot_buffer = HotFactBuffer(uav_id='test_drone')
        hot_buffer.update('battery_status', status='battery_nominal')
        hot_buffer.update('comms_status', status='comms_operational')

        query = builder.build_query(
            'safety',
            'test',
            [],
            hot_buffer=hot_buffer
        )

        assert 'Hot State Facts' in query
        assert 'battery_status_state' in query
        assert 'comms_status_state' in query
        assert 'test_query' in query  # Original template still present

    def test_cold_fact_injection(self, temp_templates_dir, temp_cold_facts_dir):
        """Test that cold facts are injected into query."""
        from vampire_bridge.cold_fact_cache import ColdFactCache

        builder = QueryBuilder(templates_path=str(temp_templates_dir))
        cold_cache = ColdFactCache(base_path=temp_cold_facts_dir)
        cold_cache.load(['geofence.tptp'])

        query = builder.build_query(
            'safety',
            'test',
            [],
            cold_cache=cold_cache
        )

        assert 'Cold State Facts' in query
        assert 'geofence_boundary' in query
        assert 'test_query' in query  # Original template still present

    def test_combined_hot_cold_injection(self, temp_templates_dir, temp_cold_facts_dir):
        """Test that both hot and cold facts are injected."""
        from vampire_bridge.hot_fact_buffer import HotFactBuffer
        from vampire_bridge.cold_fact_cache import ColdFactCache

        builder = QueryBuilder(templates_path=str(temp_templates_dir))
        hot_buffer = HotFactBuffer(uav_id='test_drone')
        hot_buffer.update('battery_status', status='battery_low')

        cold_cache = ColdFactCache(base_path=temp_cold_facts_dir)
        cold_cache.load(['geofence.tptp'])

        query = builder.build_query(
            'safety',
            'test',
            [],
            hot_buffer=hot_buffer,
            cold_cache=cold_cache
        )

        # Check ordering: cold facts before hot facts before query
        cold_pos = query.find('Cold State Facts')
        hot_pos = query.find('Hot State Facts')
        query_pos = query.find('test_query')

        assert cold_pos < hot_pos < query_pos

    def test_empty_buffers_no_injection(self, temp_templates_dir):
        """Test that empty buffers don't inject unnecessary content."""
        from vampire_bridge.hot_fact_buffer import HotFactBuffer
        from vampire_bridge.cold_fact_cache import ColdFactCache

        builder = QueryBuilder(templates_path=str(temp_templates_dir))
        hot_buffer = HotFactBuffer()  # Empty
        cold_cache = ColdFactCache()  # Empty

        query = builder.build_query(
            'safety',
            'test',
            [],
            hot_buffer=hot_buffer,
            cold_cache=cold_cache
        )

        # Should not have section headers for empty buffers
        assert 'Hot State Facts' not in query
        assert 'Cold State Facts' not in query

    def test_raw_tptp_with_state_injection(self, temp_templates_dir):
        """Test that raw TPTP also gets state injection."""
        from vampire_bridge.hot_fact_buffer import HotFactBuffer

        builder = QueryBuilder(templates_path=str(temp_templates_dir))
        hot_buffer = HotFactBuffer()
        hot_buffer.update('battery_status', status='battery_nominal')

        raw_content = "fof(raw_query, conjecture, custom(x))."
        query = builder.build_query(
            'safety',
            'ignored',
            [],
            raw_tptp=raw_content,
            hot_buffer=hot_buffer
        )

        assert 'Hot State Facts' in query
        assert 'battery_status_state' in query
        assert 'raw_query' in query

    def test_none_buffers_backward_compatible(self, temp_templates_dir):
        """Test that None buffers maintain backward compatibility."""
        builder = QueryBuilder(templates_path=str(temp_templates_dir))

        # Should work without state buffers (backward compatible)
        query = builder.build_query(
            'safety',
            'test',
            [],
            hot_buffer=None,
            cold_cache=None
        )

        assert 'test_query' in query
        assert 'State Facts' not in query


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
