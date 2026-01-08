"""
TensorRT inference wrapper for Jetson Orin NX deployment.

Provides optimized inference using TensorRT for real-time ISR policy execution
at 50Hz+ on Jetson Orin NX.

Conversion workflow:
1. Export to ONNX: python -m training.deployment.onnx_export --checkpoint model.zip --output policy.onnx
2. Convert to TensorRT (on Jetson): trtexec --onnx=policy.onnx --saveEngine=policy.trt --fp16
3. Run inference: policy = TensorRTPolicy("policy.trt"); action = policy.predict(obs)

Author: Finley Holt
"""

import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np


class TensorRTPolicy:
    """
    TensorRT-optimized policy inference for Jetson deployment.

    Provides real-time inference at 50Hz+ on Jetson Orin NX with FP16 precision.
    Handles TensorRT engine loading, memory allocation, and CUDA stream management.

    Example:
        >>> policy = TensorRTPolicy("models/policy.trt")
        >>> observation = np.array([...], dtype=np.float32)
        >>> action = policy.predict(observation)
        >>> print(f"Action: {action}, Latency: {policy.last_inference_ms:.2f}ms")
    """

    def __init__(
        self,
        engine_path: str,
        device_id: int = 0,
        verbose: bool = False,
    ):
        """
        Initialize TensorRT policy.

        Args:
            engine_path: Path to TensorRT engine file (.trt)
            device_id: CUDA device ID
            verbose: Print debug information

        Raises:
            RuntimeError: If TensorRT or CUDA initialization fails
        """
        self.engine_path = Path(engine_path)
        self.device_id = device_id
        self.verbose = verbose
        self.last_inference_ms = 0.0

        # Lazy imports (may not be available on all systems)
        self._trt = None
        self._cuda = None
        self._engine = None
        self._context = None
        self._stream = None

        # Bindings
        self._input_binding = None
        self._output_binding = None
        self._d_input = None
        self._d_output = None
        self._h_output = None

        # Initialize
        self._initialize()

    def _initialize(self) -> None:
        """Initialize TensorRT engine and allocate buffers."""
        try:
            import tensorrt as trt
            import pycuda.driver as cuda
            import pycuda.autoinit  # noqa: F401

            self._trt = trt
            self._cuda = cuda

        except ImportError as e:
            raise RuntimeError(
                f"TensorRT or PyCUDA not available: {e}\n"
                "Install with: pip install tensorrt pycuda\n"
                "On Jetson, these should be pre-installed with JetPack."
            )

        if self.verbose:
            print(f"[TRT] Loading engine: {self.engine_path}")

        # Load engine
        logger = self._trt.Logger(self._trt.Logger.WARNING)

        with open(self.engine_path, "rb") as f:
            engine_data = f.read()

        runtime = self._trt.Runtime(logger)
        self._engine = runtime.deserialize_cuda_engine(engine_data)

        if self._engine is None:
            raise RuntimeError(f"Failed to load TensorRT engine: {self.engine_path}")

        # Create execution context
        self._context = self._engine.create_execution_context()

        # Create CUDA stream
        self._stream = self._cuda.Stream()

        # Setup bindings
        self._setup_bindings()

        if self.verbose:
            print(f"[TRT] Engine loaded successfully")
            print(f"[TRT] Input shape: {self._input_shape}")
            print(f"[TRT] Output shape: {self._output_shape}")

    def _setup_bindings(self) -> None:
        """Setup input/output bindings and allocate GPU memory."""
        # Find input and output bindings
        for i in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(i)
            shape = self._engine.get_tensor_shape(name)
            dtype = self._engine.get_tensor_dtype(name)

            if self._engine.get_tensor_mode(name) == self._trt.TensorIOMode.INPUT:
                self._input_binding = name
                self._input_shape = tuple(shape)
                self._input_dtype = self._trt_to_numpy_dtype(dtype)
            else:
                self._output_binding = name
                self._output_shape = tuple(shape)
                self._output_dtype = self._trt_to_numpy_dtype(dtype)

        # Calculate sizes
        input_size = int(np.prod(self._input_shape)) * np.dtype(self._input_dtype).itemsize
        output_size = int(np.prod(self._output_shape)) * np.dtype(self._output_dtype).itemsize

        # Allocate device memory
        self._d_input = self._cuda.mem_alloc(input_size)
        self._d_output = self._cuda.mem_alloc(output_size)

        # Allocate host output buffer
        self._h_output = np.empty(self._output_shape, dtype=self._output_dtype)

        # Set tensor addresses
        self._context.set_tensor_address(self._input_binding, int(self._d_input))
        self._context.set_tensor_address(self._output_binding, int(self._d_output))

    def _trt_to_numpy_dtype(self, trt_dtype):
        """Convert TensorRT dtype to numpy dtype."""
        dtype_map = {
            self._trt.DataType.FLOAT: np.float32,
            self._trt.DataType.HALF: np.float16,
            self._trt.DataType.INT32: np.int32,
            self._trt.DataType.INT8: np.int8,
        }
        return dtype_map.get(trt_dtype, np.float32)

    def predict(self, observation: np.ndarray) -> np.ndarray:
        """
        Run inference on observation.

        Args:
            observation: Input observation of shape (obs_dim,) or (1, obs_dim)

        Returns:
            Action array of shape (action_dim,)
        """
        start_time = time.perf_counter()

        # Ensure correct shape and dtype
        if observation.ndim == 1:
            observation = observation.reshape(1, -1)
        observation = observation.astype(self._input_dtype)

        # Copy input to device
        self._cuda.memcpy_htod_async(
            self._d_input,
            observation,
            self._stream
        )

        # Run inference
        self._context.execute_async_v3(stream_handle=self._stream.handle)

        # Copy output to host
        self._cuda.memcpy_dtoh_async(
            self._h_output,
            self._d_output,
            self._stream
        )

        # Synchronize
        self._stream.synchronize()

        # Record latency
        self.last_inference_ms = (time.perf_counter() - start_time) * 1000

        return self._h_output.flatten()

    def predict_batch(self, observations: np.ndarray) -> np.ndarray:
        """
        Run batch inference (if engine supports dynamic batch).

        Args:
            observations: Input array of shape (batch, obs_dim)

        Returns:
            Actions array of shape (batch, action_dim)
        """
        # For now, run sequential inference
        # TODO: Support dynamic batch in TensorRT engine
        actions = []
        for obs in observations:
            actions.append(self.predict(obs))
        return np.array(actions)

    @property
    def obs_dim(self) -> int:
        """Get observation dimension."""
        return self._input_shape[-1]

    @property
    def action_dim(self) -> int:
        """Get action dimension."""
        return self._output_shape[-1]

    def benchmark(self, num_iterations: int = 1000, warmup: int = 100) -> dict:
        """
        Benchmark inference latency.

        Args:
            num_iterations: Number of test iterations
            warmup: Warmup iterations

        Returns:
            Dict with latency statistics
        """
        # Warmup
        dummy = np.random.randn(self.obs_dim).astype(np.float32)
        for _ in range(warmup):
            self.predict(dummy)

        # Benchmark
        latencies = []
        for _ in range(num_iterations):
            obs = np.random.randn(self.obs_dim).astype(np.float32)
            self.predict(obs)
            latencies.append(self.last_inference_ms)

        return {
            "mean_ms": np.mean(latencies),
            "std_ms": np.std(latencies),
            "min_ms": np.min(latencies),
            "max_ms": np.max(latencies),
            "p95_ms": np.percentile(latencies, 95),
            "p99_ms": np.percentile(latencies, 99),
            "max_hz": 1000 / np.max(latencies),
        }

    def __del__(self):
        """Clean up resources."""
        # TensorRT handles cleanup automatically through RAII


class ONNXRuntimePolicy:
    """
    ONNX Runtime policy for cross-platform inference.

    Use this for testing on non-Jetson systems or when TensorRT is not available.
    Provides compatible API with TensorRTPolicy.
    """

    def __init__(
        self,
        onnx_path: str,
        use_gpu: bool = True,
        verbose: bool = False,
    ):
        """
        Initialize ONNX Runtime policy.

        Args:
            onnx_path: Path to ONNX model file
            use_gpu: Use GPU acceleration if available
            verbose: Print debug information
        """
        import onnxruntime as ort

        self.onnx_path = onnx_path
        self.verbose = verbose
        self.last_inference_ms = 0.0

        # Session options
        sess_options = ort.SessionOptions()
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        # Provider selection
        providers = ["CPUExecutionProvider"]
        if use_gpu:
            if "CUDAExecutionProvider" in ort.get_available_providers():
                providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
            elif "TensorrtExecutionProvider" in ort.get_available_providers():
                providers = ["TensorrtExecutionProvider", "CPUExecutionProvider"]

        if verbose:
            print(f"[ORT] Loading: {onnx_path}")
            print(f"[ORT] Providers: {providers}")

        self._session = ort.InferenceSession(onnx_path, sess_options, providers=providers)

        # Get input/output info
        self._input_name = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name
        self._input_shape = self._session.get_inputs()[0].shape
        self._output_shape = self._session.get_outputs()[0].shape

        if verbose:
            print(f"[ORT] Input: {self._input_name} {self._input_shape}")
            print(f"[ORT] Output: {self._output_name} {self._output_shape}")

    def predict(self, observation: np.ndarray) -> np.ndarray:
        """
        Run inference on observation.

        Args:
            observation: Input observation

        Returns:
            Action array
        """
        start_time = time.perf_counter()

        if observation.ndim == 1:
            observation = observation.reshape(1, -1)
        observation = observation.astype(np.float32)

        outputs = self._session.run(
            [self._output_name],
            {self._input_name: observation}
        )

        self.last_inference_ms = (time.perf_counter() - start_time) * 1000

        return outputs[0].flatten()

    @property
    def obs_dim(self) -> int:
        """Get observation dimension."""
        return self._input_shape[-1]

    @property
    def action_dim(self) -> int:
        """Get action dimension."""
        return self._output_shape[-1]

    def benchmark(self, num_iterations: int = 1000, warmup: int = 100) -> dict:
        """Benchmark inference latency."""
        dummy = np.random.randn(self.obs_dim).astype(np.float32)
        for _ in range(warmup):
            self.predict(dummy)

        latencies = []
        for _ in range(num_iterations):
            obs = np.random.randn(self.obs_dim).astype(np.float32)
            self.predict(obs)
            latencies.append(self.last_inference_ms)

        return {
            "mean_ms": np.mean(latencies),
            "std_ms": np.std(latencies),
            "min_ms": np.min(latencies),
            "max_ms": np.max(latencies),
            "p95_ms": np.percentile(latencies, 95),
            "p99_ms": np.percentile(latencies, 99),
            "max_hz": 1000 / np.max(latencies),
        }


def create_policy(model_path: str, verbose: bool = False):
    """
    Create appropriate policy based on file extension.

    Args:
        model_path: Path to model file (.trt or .onnx)
        verbose: Print debug information

    Returns:
        TensorRTPolicy or ONNXRuntimePolicy
    """
    path = Path(model_path)

    if path.suffix == ".trt":
        return TensorRTPolicy(model_path, verbose=verbose)
    elif path.suffix == ".onnx":
        return ONNXRuntimePolicy(model_path, verbose=verbose)
    else:
        raise ValueError(f"Unknown model format: {path.suffix}")


def main():
    """Command-line interface for policy benchmarking."""
    import argparse

    parser = argparse.ArgumentParser(description="Benchmark TensorRT/ONNX policy")
    parser.add_argument("model_path", help="Path to model file (.trt or .onnx)")
    parser.add_argument("--iterations", type=int, default=1000, help="Benchmark iterations")
    parser.add_argument("--warmup", type=int, default=100, help="Warmup iterations")
    args = parser.parse_args()

    print(f"Loading model: {args.model_path}")
    policy = create_policy(args.model_path, verbose=True)

    print(f"\nRunning benchmark ({args.iterations} iterations)...")
    results = policy.benchmark(args.iterations, args.warmup)

    print(f"\nResults:")
    print(f"  Mean latency: {results['mean_ms']:.3f} ms")
    print(f"  Std latency:  {results['std_ms']:.3f} ms")
    print(f"  P95 latency:  {results['p95_ms']:.3f} ms")
    print(f"  P99 latency:  {results['p99_ms']:.3f} ms")
    print(f"  Max Hz:       {results['max_hz']:.0f}")

    # Check if we meet 50Hz requirement
    if results['p99_ms'] < 20:
        print(f"\n[PASS] Policy meets 50Hz requirement (P99 < 20ms)")
    else:
        print(f"\n[WARN] Policy may not meet 50Hz requirement (P99 >= 20ms)")


if __name__ == "__main__":
    main()
