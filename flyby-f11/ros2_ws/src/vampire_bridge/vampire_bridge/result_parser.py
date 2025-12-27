"""
result_parser.py - Parse Vampire SZS output to ROS messages

Parses the output from Vampire theorem prover and extracts:
    - SZS status (Theorem, CounterSatisfiable, Timeout, etc.)
    - Proof details (axioms used, inference steps)
    - Performance metrics (time taken)

SZS Status Codes (relevant subset):
    - Theorem (THM): Conjecture proven from axioms
    - CounterSatisfiable (CSA): Found model where axioms true but conjecture false
    - Satisfiable (SAT): Axioms are satisfiable (no conjecture)
    - Unsatisfiable (UNS): Axioms are contradictory
    - Timeout (TMO): Time limit exceeded
    - GaveUp (GUP): Prover gave up
    - Error (ERR): Parsing or execution error
"""

import re
from dataclasses import dataclass
from typing import Optional


@dataclass
class VampireResult:
    """Parsed result from Vampire theorem prover."""
    szs_status: str
    success: bool
    proof_summary: str
    violated_axioms: list
    latency_ms: float
    raw_output: str
    error_message: str = ""


class ResultParser:
    """
    Parses Vampire theorem prover output.

    Extracts SZS status, proof information, and timing from Vampire's
    standard output format.
    """

    # SZS status pattern
    SZS_PATTERN = re.compile(r'% SZS status (\w+)')

    # Time pattern (Vampire reports time in seconds)
    TIME_PATTERN = re.compile(r'% Time elapsed: ([\d.]+) s')

    # Axiom usage pattern (in refutation proofs)
    AXIOM_PATTERN = re.compile(r'file\([^,]+,\s*(\w+)\)')

    # Inference pattern for proof steps
    INFERENCE_PATTERN = re.compile(r'inference\((\w+),')

    # Human-readable status descriptions
    STATUS_DESCRIPTIONS = {
        'Theorem': 'Conjecture proven - conditions satisfied',
        'CounterSatisfiable': 'Conjecture disproven - conditions not met',
        'Satisfiable': 'Axioms consistent, conjecture undetermined',
        'Unsatisfiable': 'Axioms are contradictory',
        'Timeout': 'Query timeout - time limit exceeded',
        'GaveUp': 'Prover unable to determine result',
        'ResourceOut': 'Memory or resource limit exceeded',
        'Unknown': 'Unable to parse Vampire output',
        'Error': 'Execution or parsing error',
    }

    # Map common variations to canonical status
    STATUS_NORMALIZATION = {
        'theorem': 'Theorem',
        'countersatisfiable': 'CounterSatisfiable',
        'satisfiable': 'Satisfiable',
        'unsatisfiable': 'Unsatisfiable',
        'timeout': 'Timeout',
        'gaveup': 'GaveUp',
        'resourceout': 'ResourceOut',
        'unknown': 'Unknown',
        'error': 'Error',
        # Additional SZS variants
        'thm': 'Theorem',
        'csa': 'CounterSatisfiable',
        'sat': 'Satisfiable',
        'uns': 'Unsatisfiable',
        'tmo': 'Timeout',
    }

    def __init__(self, max_output_length: int = 10000):
        """
        Initialize the result parser.

        Args:
            max_output_length: Maximum characters to include in raw_output
        """
        self.max_output_length = max_output_length

    def parse(
        self,
        vampire_output: str,
        vampire_stderr: str = "",
        execution_time_ms: Optional[float] = None
    ) -> VampireResult:
        """
        Parse Vampire output into a structured result.

        Args:
            vampire_output: Standard output from Vampire
            vampire_stderr: Standard error from Vampire (for error detection)
            execution_time_ms: Measured execution time (overrides Vampire's report)

        Returns:
            VampireResult with parsed information
        """
        # Extract SZS status
        szs_status = self._extract_status(vampire_output, vampire_stderr)

        # Extract timing
        if execution_time_ms is not None:
            latency_ms = execution_time_ms
        else:
            latency_ms = self._extract_time(vampire_output)

        # Extract axioms used in proof
        violated_axioms = self._extract_axioms(vampire_output, szs_status)

        # Generate proof summary
        proof_summary = self._generate_summary(szs_status, violated_axioms)

        # Check for errors
        error_message = self._extract_error(vampire_output, vampire_stderr)

        # Truncate raw output if too long
        raw_output = vampire_output
        if len(raw_output) > self.max_output_length:
            raw_output = raw_output[:self.max_output_length] + "\n... [truncated]"

        return VampireResult(
            szs_status=szs_status,
            success=(szs_status not in ['Error', 'Unknown', 'Timeout', 'GaveUp', 'ResourceOut'] and not error_message),
            proof_summary=proof_summary,
            violated_axioms=violated_axioms,
            latency_ms=latency_ms,
            raw_output=raw_output,
            error_message=error_message,
        )

    def _extract_status(self, output: str, stderr: str) -> str:
        """Extract and normalize SZS status from output."""
        # Check for error conditions first
        if "error" in stderr.lower() or "Error" in output:
            error_match = re.search(r'error[:\s]+(.+)', output + stderr, re.IGNORECASE)
            if error_match and "SZS status" not in output:
                return 'Error'

        # Find SZS status line
        match = self.SZS_PATTERN.search(output)
        if match:
            raw_status = match.group(1)
            normalized = self.STATUS_NORMALIZATION.get(
                raw_status.lower(),
                raw_status  # Keep original if not in map
            )
            return normalized

        # No SZS status found
        return 'Unknown'

    def _extract_time(self, output: str) -> float:
        """Extract execution time from Vampire output."""
        match = self.TIME_PATTERN.search(output)
        if match:
            return float(match.group(1)) * 1000  # Convert to ms
        return 0.0

    def _extract_axioms(self, output: str, status: str) -> list:
        """
        Extract axioms used in the proof.

        For 'Theorem' status, these are the axioms that contributed to
        proving the conjecture (e.g., safety violations).
        """
        if status != 'Theorem':
            return []

        # Find all axiom references in the proof
        axioms = set()
        for match in self.AXIOM_PATTERN.finditer(output):
            axiom_name = match.group(1)
            # Filter out auto-generated names
            if not axiom_name.startswith('sP') and not axiom_name.isdigit():
                axioms.add(axiom_name)

        return sorted(list(axioms))

    def _generate_summary(self, status: str, axioms: list) -> str:
        """Generate human-readable proof summary."""
        base_desc = self.STATUS_DESCRIPTIONS.get(status, f"Status: {status}")

        if status == 'Theorem' and axioms:
            axiom_list = ', '.join(axioms[:5])
            if len(axioms) > 5:
                axiom_list += f", ... ({len(axioms)} total)"
            return f"{base_desc}. Key axioms: {axiom_list}"

        return base_desc

    def _extract_error(self, output: str, stderr: str) -> str:
        """Extract error message if present."""
        combined = output + stderr

        # Check for common error patterns
        error_patterns = [
            r'Error[:\s]+(.+)',
            r'Syntax error[:\s]+(.+)',
            r'Parse error[:\s]+(.+)',
            r'SIGKILL',
            r'out of memory',
        ]

        for pattern in error_patterns:
            match = re.search(pattern, combined, re.IGNORECASE)
            if match:
                if match.groups():
                    return match.group(1).strip()[:200]
                return match.group(0).strip()[:200]

        return ""

    def is_violation_detected(self, result: VampireResult) -> bool:
        """
        Check if the result indicates a safety violation was detected.

        For safety queries structured as "prove violation exists",
        a Theorem status means a violation was found.

        Args:
            result: Parsed Vampire result

        Returns:
            True if violation detected (conjecture proven)
        """
        return result.szs_status == 'Theorem'

    def is_safe(self, result: VampireResult) -> bool:
        """
        Check if the result indicates safe state.

        For safety queries structured as "prove violation exists",
        a CounterSatisfiable status means no violation was found.

        Args:
            result: Parsed Vampire result

        Returns:
            True if no violation detected
        """
        return result.szs_status == 'CounterSatisfiable'
