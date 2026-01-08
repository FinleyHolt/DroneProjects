"""
Fast Training Tests

Full-stack integration tests that produce video evidence.

Test Files:
- test_render_skipping.py: Phase 1 - Validates render frame skipping optimization
- test_direct_dynamics.py: Phase 2a - Legacy direct dynamics (deprecated)
- test_isr_training_env.py: Phase 2b - ISR training env with frustum-filtered GT

Usage:
    # Phase 1: Render skipping
    /isaac-sim/python.sh /workspace/training/tests/test_render_skipping.py --headless

    # Phase 2: ISR training environment (recommended)
    /isaac-sim/python.sh /workspace/training/tests/test_isr_training_env.py --headless --num-envs 64

    # Benchmark with more environments
    /isaac-sim/python.sh /workspace/training/tests/test_isr_training_env.py --headless --num-envs 1024 --benchmark-only
"""
