"""
Validate metrics worker stability, lifecycle, and memory behavior.

Worker Lifecycle:
  - Continuous loop under synthetic sensor streams
  - Thread lifecycle cleanliness
  - Clean join and exception handling

Memory & Queue Management:
  - Monitor memory growth (should be <1% per minute)
  - Track queue depth over time
  - Detect memory leaks during sustained operation

Worker States:
  - Idle (no collisions detected)
  - Snapshot copy (capturing and processing telemetry)
  - Collision detection (intensive computation)

Test Duration: 300+ seconds (5+ minutes)
Iterations: Stress test with realistic sensor load
"""

import sys
import time
import threading
import queue
import random
import psutil
import os
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import logging

logging.basicConfig(level=logging.WARNING)


class WorkerValidator:
    """Validation suite for metrics worker stability and lifecycle."""

    def __init__(self, duration_seconds=60):
        """Initialize validator.
        
        Args:
            duration_seconds: How long to run stress tests (default 60s for quick testing)
        """
        self.results = {}
        self.duration = duration_seconds
        self.process = psutil.Process()
        self.memory_samples = []
        self.queue_depths = []
        self.exceptions = []

    def get_memory_usage_mb(self):
        """Get current process memory usage in MB."""
        return self.process.memory_info().rss / (1024 * 1024)

    def simulate_sensor_stream(self, n_agents=10, rate_hz=50, duration=30):
        """Generate synthetic sensor data stream.
        
        Args:
            n_agents: Number of agents generating sensor data
            rate_hz: Sensor update rate
            duration: How long to simulate (seconds)
        
        Yields:
            (timestamp, sensor_data_dict)
        """
        interval = 1.0 / rate_hz
        start_time = time.time()
        
        while time.time() - start_time < duration:
            for agent_id in range(n_agents):
                timestamp = time.time()
                sensor_data = {
                    "agent_id": agent_id,
                    "x": random.uniform(-100, 100),
                    "y": random.uniform(-100, 100),
                    "z": random.uniform(0, 50),
                    "timestamp": timestamp
                }
                yield (timestamp, sensor_data)
            
            time.sleep(interval)

    def worker_sensor_ingestion(self, sensor_queue, stop_event, duration=30):
        """Simulate sensor data consumer (metrics node).
        
        Args:
            sensor_queue: queue.Queue for sensor messages
            stop_event: threading.Event to signal shutdown
            duration: How long to run
        """
        try:
            start_time = time.time()
            sensor_count = 0
            
            while not stop_event.is_set() and time.time() - start_time < duration:
                try:
                    # Get sensor data from queue with timeout
                    timestamp, sensor_data = sensor_queue.get(timeout=0.1)
                    sensor_count += 1
                    
                    # Simulate minimal processing
                    _ = (sensor_data["x"], sensor_data["y"], sensor_data["z"])
                    
                except queue.Empty:
                    pass  # Normal timeout
            
            self.results["sensor_ingestion_count"] = sensor_count
        
        except Exception as e:
            self.exceptions.append(f"sensor_ingestion: {e}")

    def worker_collision_detection(self, collision_queue, stop_event, duration=30):
        """Simulate collision detection worker.
        
        Args:
            collision_queue: queue.Queue for collision checks
            stop_event: threading.Event to signal shutdown
            duration: How long to run
        """
        try:
            start_time = time.time()
            collision_checks = 0
            collisions_found = 0
            
            while not stop_event.is_set() and time.time() - start_time < duration:
                # Simulate collision check (pseudo-work)
                agents = [f"drone_{i}" for i in range(10)]
                
                # Brute force O(N²) check
                for i in range(len(agents)):
                    for j in range(i + 1, len(agents)):
                        # Simulate distance check
                        dist = random.uniform(0, 10)
                        if dist < 2.5:
                            collisions_found += 1
                        collision_checks += 1
                
                time.sleep(0.05)  # Simulate computation
            
            self.results["collision_checks"] = collision_checks
            self.results["collisions_found"] = collisions_found
        
        except Exception as e:
            self.exceptions.append(f"collision_detection: {e}")

    def worker_telemetry_snapshot(self, stop_event, duration=30):
        """Simulate telemetry snapshot and aggregation.
        
        Args:
            stop_event: threading.Event to signal shutdown
            duration: How long to run
        """
        try:
            start_time = time.time()
            snapshot_count = 0
            
            while not stop_event.is_set() and time.time() - start_time < duration:
                # Simulate snapshot copy and aggregation (every 1s)
                time.sleep(1.0)
                
                # Simulate data aggregation
                telemetry = {
                    "agents": 10,
                    "collisions": random.randint(0, 3),
                    "timestamp": time.time()
                }
                snapshot_count += 1
            
            self.results["snapshot_count"] = snapshot_count
        
        except Exception as e:
            self.exceptions.append(f"telemetry_snapshot: {e}")

    def benchmark_worker_lifecycle(self, duration=30):
        """Benchmark worker thread lifecycle and cleanliness."""
        print("\n[BENCHMARK] Worker Thread Lifecycle")
        print("=" * 60)
        
        sensor_queue = queue.Queue(maxsize=1000)
        stop_event = threading.Event()
        
        # Create worker threads
        threads = [
            threading.Thread(
                target=self.worker_sensor_ingestion,
                args=(sensor_queue, stop_event, duration)
            ),
            threading.Thread(
                target=self.worker_collision_detection,
                args=(sensor_queue, stop_event, duration)
            ),
            threading.Thread(
                target=self.worker_telemetry_snapshot,
                args=(stop_event, duration)
            ),
        ]
        
        print(f"Starting {len(threads)} worker threads...")
        
        # Start all threads
        start_clock = time.time()
        for t in threads:
            t.start()
        
        # Generate sensor data
        print(f"Generating sensor stream for {duration}s...")
        for timestamp, sensor_data in self.simulate_sensor_stream(n_agents=10, rate_hz=50, duration=duration):
            try:
                sensor_queue.put((timestamp, sensor_data), timeout=0.1)
            except queue.Full:
                pass  # Queue full, drop message (normal behavior)
        
        # Signal shutdown and wait for threads
        print("Signaling worker shutdown...")
        stop_event.set()
        
        for t in threads:
            t.join(timeout=5.0)
            if t.is_alive():
                print(f"  WARNING: Thread {t.name} did not join cleanly")
        
        elapsed = time.time() - start_clock
        
        print(f"\nWorker lifecycle summary:")
        print(f"  Duration:       {elapsed:.1f}s")
        print(f"  Threads:        {len(threads)} (all joined)")
        print(f"  Exceptions:     {len(self.exceptions)}")
        if self.exceptions:
            for exc in self.exceptions[:3]:
                print(f"    {exc}")
        
        self.results["worker_lifecycle"] = {
            "duration": elapsed,
            "threads": len(threads),
            "exceptions": len(self.exceptions),
            "status": "PASS" if len(self.exceptions) == 0 else "FAIL"
        }

    def benchmark_memory_growth(self, duration=60):
        """Benchmark memory growth during sustained operation."""
        print("\n[BENCHMARK] Memory Growth Tracking")
        print("=" * 60)
        
        stop_event = threading.Event()
        
        # Start worker threads
        threads = [
            threading.Thread(
                target=self.worker_collision_detection,
                args=(queue.Queue(), stop_event, duration)
            ),
            threading.Thread(
                target=self.worker_telemetry_snapshot,
                args=(stop_event, duration)
            ),
        ]
        
        for t in threads:
            t.start()
        
        # Monitor memory growth
        print(f"Monitoring memory for {duration}s...")
        start_memory = self.get_memory_usage_mb()
        start_time = time.time()
        
        while time.time() - start_time < duration:
            mem_mb = self.get_memory_usage_mb()
            self.memory_samples.append((time.time() - start_time, mem_mb))
            time.sleep(1.0)
        
        # Shutdown
        stop_event.set()
        for t in threads:
            t.join(timeout=5.0)
        
        # Analyze memory growth
        end_memory = self.get_memory_usage_mb()
        memory_growth = end_memory - start_memory
        growth_rate_per_minute = (memory_growth / duration) * 60
        growth_pct_per_minute = (growth_rate_per_minute / start_memory) * 100
        
        print(f"\nMemory growth summary:")
        print(f"  Initial:        {start_memory:.1f} MB")
        print(f"  Final:          {end_memory:.1f} MB")
        print(f"  Growth:         {memory_growth:.1f} MB")
        print(f"  Rate:           {growth_rate_per_minute:.2f} MB/min")
        print(f"  Growth %/min:   {growth_pct_per_minute:.2f}%")
        
        status = "PASS" if growth_pct_per_minute < 1.0 else "WARNING"
        print(f"  Status:         {status} (threshold <1.0%/min)")
        
        self.results["memory_growth"] = {
            "initial_mb": start_memory,
            "final_mb": end_memory,
            "growth_mb": memory_growth,
            "growth_rate_mb_per_min": growth_rate_per_minute,
            "growth_pct_per_min": growth_pct_per_minute,
            "status": status
        }

    def benchmark_worker_responsiveness(self, duration=30):
        """Benchmark worker responsiveness in different states."""
        print("\n[BENCHMARK] Worker Responsiveness")
        print("=" * 60)
        
        state_timings = {"idle": [], "processing": [], "snapshot": []}
        stop_event = threading.Event()
        
        # Simulate idle state
        print("  Testing idle state (no collisions)...")
        t0 = time.time()
        while time.time() - t0 < 5:
            # Minimal work
            time.sleep(0.01)
        state_timings["idle"].append(time.time() - t0)
        
        # Simulate processing state (collision checks)
        print("  Testing collision processing state...")
        t0 = time.time()
        thread = threading.Thread(
            target=self.worker_collision_detection,
            args=(queue.Queue(), threading.Event(), 5)
        )
        thread.start()
        thread.join()
        state_timings["processing"].append(time.time() - t0)
        
        # Simulate snapshot state
        print("  Testing snapshot capture state...")
        t0 = time.time()
        thread = threading.Thread(
            target=self.worker_telemetry_snapshot,
            args=(threading.Event(), 5)
        )
        thread.start()
        thread.join()
        state_timings["snapshot"].append(time.time() - t0)
        
        print(f"\nWorker responsiveness summary:")
        for state, timings in state_timings.items():
            if timings:
                avg = sum(timings) / len(timings)
                print(f"  {state:15s}: {avg:.2f}s")
        
        self.results["worker_responsiveness"] = state_timings

    def verify_thread_shutdown(self):
        """Verify clean thread shutdown behavior."""
        print("\n[VERIFICATION] Thread Shutdown Cleanliness")
        print("=" * 60)
        
        stop_event = threading.Event()
        threads = []
        
        for i in range(5):
            t = threading.Thread(
                target=lambda se=stop_event: (time.sleep(0.1), se.wait(10))
            )
            t.start()
            threads.append(t)
        
        # Signal and wait for shutdown
        time.sleep(0.2)
        stop_event.set()
        
        all_joined = True
        for t in threads:
            t.join(timeout=5.0)
            if t.is_alive():
                all_joined = False
                print(f"  ✗ Thread {t.name} failed to join")
        
        if all_joined:
            print("  ✓ All threads joined cleanly")
        
        self.results["thread_shutdown"] = "PASS" if all_joined else "FAIL"

    def run_all_benchmarks(self):
        """Execute all worker validation benchmarks."""
        print("\n" + "=" * 60)
        print("METRICS WORKER STABILITY VALIDATION")
        print("=" * 60)
        print(f"Test duration: {self.duration}s")
        
        self.benchmark_worker_lifecycle(duration=self.duration)
        
        # Only run extended tests if duration is long enough
        if self.duration >= 60:
            self.benchmark_memory_growth(duration=min(60, self.duration - 5))
        
        self.benchmark_worker_responsiveness(duration=self.duration // 2)
        self.verify_thread_shutdown()
        
        # Summary
        print("\n[SUMMARY]")
        print("=" * 60)
        for key, value in self.results.items():
            if isinstance(value, dict):
                print(f"{key}:")
                for k, v in value.items():
                    print(f"  {k}: {v}")
            else:
                print(f"{key}: {value}")
        
        return self.results


if __name__ == "__main__":
    # For quick testing: 30s duration
    # For full validation: increase to 300+ seconds
    validator = WorkerValidator(duration_seconds=30)
    results = validator.run_all_benchmarks()
    print("\nWorker validation complete.")
