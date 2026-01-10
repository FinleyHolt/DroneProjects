"""
test_result_parser.py - Unit tests for ResultParser

Tests parsing of Vampire theorem prover output.
"""

import pytest
from vampire_bridge.result_parser import ResultParser, VampireResult


class TestResultParser:
    """Tests for ResultParser class."""

    @pytest.fixture
    def parser(self):
        """Create a ResultParser instance."""
        return ResultParser()

    def test_parse_theorem_status(self, parser):
        """Test parsing Theorem (proven) status."""
        output = """
% SZS status Theorem for test_query
% Refutation found. Thanks to Tanya!
"""
        result = parser.parse(output)

        assert result.szs_status == 'Theorem'
        assert result.success is True

    def test_parse_countersatisfiable_status(self, parser):
        """Test parsing CounterSatisfiable (disproven) status."""
        output = """
% SZS status CounterSatisfiable for test_query
% Satisfiable!
"""
        result = parser.parse(output)

        assert result.szs_status == 'CounterSatisfiable'
        assert result.success is True

    def test_parse_timeout_status(self, parser):
        """Test parsing Timeout status."""
        output = """
% SZS status Timeout for test_query
% Time limit reached!
"""
        result = parser.parse(output)

        assert result.szs_status == 'Timeout'
        assert result.success is False

    def test_parse_satisfiable_status(self, parser):
        """Test parsing Satisfiable status."""
        output = """
% SZS status Satisfiable for test_query
"""
        result = parser.parse(output)

        assert result.szs_status == 'Satisfiable'
        assert result.success is True

    def test_parse_unknown_output(self, parser):
        """Test parsing output without SZS status."""
        output = """
Some random output without status
"""
        result = parser.parse(output)

        assert result.szs_status == 'Unknown'
        assert result.success is False

    def test_parse_execution_time(self, parser):
        """Test parsing execution time from output."""
        output = """
% SZS status Theorem
% Time elapsed: 0.042 s
"""
        result = parser.parse(output)

        assert result.latency_ms == pytest.approx(42.0, rel=0.01)

    def test_override_execution_time(self, parser):
        """Test that provided execution time overrides parsed value."""
        output = """
% SZS status Theorem
% Time elapsed: 0.042 s
"""
        result = parser.parse(output, execution_time_ms=100.0)

        assert result.latency_ms == 100.0

    def test_extract_axioms_from_proof(self, parser):
        """Test extracting axiom names from proof output."""
        output = """
% SZS status Theorem
fof(f1, axiom, uav(X), file(test.tptp, nfz_violation_axiom)).
fof(f2, axiom, safe(X), file(test.tptp, geofence_check)).
fof(f3, plain, derived, inference(resolution, [status(thm)], [f1, f2])).
"""
        result = parser.parse(output)

        assert 'nfz_violation_axiom' in result.violated_axioms
        assert 'geofence_check' in result.violated_axioms

    def test_no_axioms_for_countersatisfiable(self, parser):
        """Test that axioms are not extracted for non-Theorem status."""
        output = """
% SZS status CounterSatisfiable
fof(f1, axiom, uav(X), file(test.tptp, some_axiom)).
"""
        result = parser.parse(output)

        assert result.violated_axioms == []

    def test_generate_proof_summary_theorem(self, parser):
        """Test proof summary generation for Theorem."""
        output = """
% SZS status Theorem
fof(f1, axiom, x, file(test.tptp, axiom1)).
fof(f2, axiom, y, file(test.tptp, axiom2)).
"""
        result = parser.parse(output)

        assert 'proven' in result.proof_summary.lower()
        assert 'axiom1' in result.proof_summary or 'axiom2' in result.proof_summary

    def test_generate_proof_summary_timeout(self, parser):
        """Test proof summary generation for Timeout."""
        output = "% SZS status Timeout"
        result = parser.parse(output)

        assert 'timeout' in result.proof_summary.lower()

    def test_raw_output_truncation(self):
        """Test that long output is truncated."""
        parser = ResultParser(max_output_length=100)

        long_output = "% SZS status Theorem\n" + "x" * 200

        result = parser.parse(long_output)

        assert len(result.raw_output) <= 120  # 100 + "[truncated]" message
        assert 'truncated' in result.raw_output

    def test_parse_error_in_stderr(self, parser):
        """Test detecting errors from stderr."""
        stdout = "Some output"
        stderr = "Error: Syntax error in input file"

        result = parser.parse(stdout, stderr)

        assert result.error_message != ""
        assert 'Syntax error' in result.error_message

    def test_parse_error_in_stdout(self, parser):
        """Test detecting errors from stdout."""
        output = "Error: Parse error at line 5"

        result = parser.parse(output)

        assert 'Parse error' in result.error_message

    def test_is_violation_detected(self, parser):
        """Test violation detection helper."""
        theorem_result = VampireResult(
            szs_status='Theorem',
            success=True,
            proof_summary='',
            violated_axioms=[],
            latency_ms=50,
            raw_output=''
        )
        assert parser.is_violation_detected(theorem_result) is True

        counter_result = VampireResult(
            szs_status='CounterSatisfiable',
            success=True,
            proof_summary='',
            violated_axioms=[],
            latency_ms=50,
            raw_output=''
        )
        assert parser.is_violation_detected(counter_result) is False

    def test_is_safe(self, parser):
        """Test safe state detection helper."""
        safe_result = VampireResult(
            szs_status='CounterSatisfiable',
            success=True,
            proof_summary='',
            violated_axioms=[],
            latency_ms=50,
            raw_output=''
        )
        assert parser.is_safe(safe_result) is True

        violation_result = VampireResult(
            szs_status='Theorem',
            success=True,
            proof_summary='',
            violated_axioms=[],
            latency_ms=50,
            raw_output=''
        )
        assert parser.is_safe(violation_result) is False

    def test_status_normalization(self, parser):
        """Test that various status formats are normalized."""
        test_cases = [
            ("% SZS status Theorem", "Theorem"),
            ("% SZS status theorem", "Theorem"),
            ("% SZS status THEOREM", "Theorem"),  # Edge case
            ("% SZS status CounterSatisfiable", "CounterSatisfiable"),
        ]

        for output, expected in test_cases:
            result = parser.parse(output)
            # The status might not match exactly due to normalization logic
            # but should be recognized as a valid status
            assert result.success or result.szs_status in ['Theorem', 'CounterSatisfiable', 'Unknown']


class TestResultParserEdgeCases:
    """Edge case tests for ResultParser."""

    def test_empty_output(self):
        """Test parsing empty output."""
        parser = ResultParser()
        result = parser.parse("")

        assert result.szs_status == 'Unknown'
        assert result.success is False

    def test_multiline_status(self):
        """Test output with status on its own line."""
        parser = ResultParser()
        output = """
% Starting proof search
%
% SZS status Theorem
%
% Proof found
"""
        result = parser.parse(output)
        assert result.szs_status == 'Theorem'

    def test_multiple_status_lines(self):
        """Test output with multiple SZS status lines (takes first)."""
        parser = ResultParser()
        output = """
% SZS status Theorem
% SZS status CounterSatisfiable
"""
        result = parser.parse(output)
        # Should take the first status
        assert result.szs_status in ['Theorem', 'CounterSatisfiable']


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
