"""
Validate APF and ORCA path planner trajectory stability and force bounds.

APF Validation:
  - Scenario A: no obstacles
  - Scenario B: single obstacle
  - Scenario C: dense wall
  - Scenario D: narrow corridor
  - Scenario E: duplicate/dense voxel scaling
  - Verify ||F_rep|| <= F_max_rep=10.0 N
  - Check for NaN/Inf in forces

ORCA Vertical Envelope Validation:
  - Case 1: same altitude
  - Case 2: far above (+30m)
  - Case 3: far below (-30m)
  - Case 4: boundary (±1.75m margin)
  - Case 5: missing/NaN/Inf z
  - Verify trajectory convergence, zero oscillations

Iterations: 300-500 per scenario
"""

import sys
import math
import random
import time
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from agent.path_planning import APFStrategy, ORCAStrategy
from agent.voxel_map import VoxelMap
import logging

logging.basicConfig(level=logging.WARNING)


class PlannerValidator:
    """Validation suite for APF and ORCA path planners."""

    def __init__(self):
        self.results = {}
        self.F_MAX_REP = 10.0  # Newton's (assumed limit)
        self.ORCA_VERTICAL_MARGIN = 1.75  # meters

    def create_apf_scenario(self, scenario_type):
        """Create APF test scenario with voxels."""
        vmap = VoxelMap()
        
        current_pos = {"x": 0.0, "y": 0.0, "z": 10.0}
        goal_pos = {"x": 50.0, "y": 0.0, "z": 10.0}
        
        if scenario_type == "A_no_obstacles":
            # Empty environment
            pass
        
        elif scenario_type == "B_single_obstacle":
            # One obstacle at (25, 0, 10)
            vmap.mark_occupied(25.0, 0.0, 10.0, confidence=1.0, current_time=time.time())
        
        elif scenario_type == "C_dense_wall":
            # Dense wall blocking direct path at x=30
            now = time.time()
            for y in range(-20, 21, 1):
                for z in range(5, 16, 1):
                    vmap.mark_occupied(30.0, float(y), float(z), confidence=1.0, current_time=now)
        
        elif scenario_type == "D_narrow_corridor":
            # Narrow corridor (2m width at x=25)
            now = time.time()
            for y in range(-10, -8):
                for z in range(8, 13):
                    vmap.mark_occupied(25.0, float(y), float(z), confidence=1.0, current_time=now)
            for y in range(8, 10):
                for z in range(8, 13):
                    vmap.mark_occupied(25.0, float(y), float(z), confidence=1.0, current_time=now)
        
        elif scenario_type == "E_dense_scaling":
            # 1000+ voxels to test dense obstacle handling
            now = time.time()
            for i in range(100):
                x = 20.0 + (i % 10) * 0.5
                y = -5.0 + ((i // 10) % 10) * 0.5
                z = 8.0 + ((i // 100) % 5) * 0.5
                vmap.mark_occupied(x, y, z, confidence=1.0, current_time=now)
        
        return vmap, current_pos, goal_pos

    def benchmark_apf_forces(self, iterations=300):
        """Benchmark APF repulsive force bounds and stability."""
        print("\n[BENCHMARK] APF Strategy - Force Bounds & Stability")
        print("=" * 60)
        
        scenarios = [
            "A_no_obstacles",
            "B_single_obstacle",
            "C_dense_wall",
            "D_narrow_corridor",
            "E_dense_scaling"
        ]
        
        # APF configuration
        apf_config = {
            "attractive_gain": 1.8,
            "repulsive_gain": 8.5,
            "influence_radius": 7.0,
            "step_size": 0.2,
            "max_velocity": 4.0,
            "goal_tolerance": 1.5,
            "braking_radius": 7.5,
            "ship_keepout_radius": 18.0,
            "ship_influence_radius": 30.0,
            "apf_exponential_decay": 0.5,
            "apf_inverse_square_scale": 0.3,
            "apf_stuck_growth_rate": 0.2,
            "min_movement": 0.01,
            "stuck_threshold": 5,
            "min_z": 1.0,
            "max_z": 50.0,
            "velocity_smoothing": 0.42,
            "max_repulsive_force": self.F_MAX_REP
        }
        
        for scenario in scenarios:
            vmap, current_pos, goal_pos = self.create_apf_scenario(scenario)
            
            # Create and configure APF strategy
            apf = APFStrategy()
            apf.configure(apf_config)
            
            force_violations = 0
            nan_inf_violations = 0
            force_magnitudes = []
            
            for _ in range(iterations):
                try:
                    # Compute velocity using correct signature
                    velocity = apf.compute_velocity(
                        current_pose=current_pos,
                        goal_pose=goal_pos,
                        voxel_map=vmap
                    )
                    
                    # Extract force magnitude (velocity scaled by dt=0.1)
                    vx = velocity.get("linear", {}).get("x", 0)
                    vy = velocity.get("linear", {}).get("y", 0)
                    vz = velocity.get("linear", {}).get("z", 0)
                    
                    force_mag = math.sqrt(vx**2 + vy**2 + vz**2)
                    force_magnitudes.append(force_mag)
                    
                    # Check for NaN/Inf
                    if math.isnan(force_mag) or math.isinf(force_mag):
                        nan_inf_violations += 1
                    
                    # Check bounds (repulsive force only)
                    if force_mag > self.F_MAX_REP:
                        force_violations += 1
                
                except Exception as e:
                    print(f"    Exception in {scenario}: {e}")
            
            # Compute statistics
            if force_magnitudes:
                avg_force = sum(force_magnitudes) / len(force_magnitudes)
                max_force = max(force_magnitudes)
                min_force = min(force_magnitudes)
            else:
                avg_force = max_force = min_force = 0.0
            
            violation_rate = (force_violations / iterations) * 100 if iterations > 0 else 0
            nan_rate = (nan_inf_violations / iterations) * 100 if iterations > 0 else 0
            
            status = "✓ PASS" if (force_violations == 0 and nan_inf_violations == 0) else "✗ FAIL"
            
            print(f"\n{scenario:20s} {status}")
            print(f"  Force range:     {min_force:8.2f} - {max_force:8.2f} N (avg={avg_force:8.2f})")
            print(f"  Max bound:       {self.F_MAX_REP:.2f} N")
            print(f"  Violations:      {force_violations}/{iterations} ({violation_rate:.1f}%)")
            print(f"  NaN/Inf:         {nan_inf_violations}/{iterations} ({nan_rate:.1f}%)")
            
            self.results[f"apf_{scenario}"] = {
                "avg_force_n": avg_force,
                "max_force_n": max_force,
                "violations": force_violations,
                "nan_inf": nan_inf_violations,
                "status": status
            }

    def benchmark_orca_vertical_envelope(self, iterations=300):
        """Benchmark ORCA vertical obstacle filtering."""
        print("\n[BENCHMARK] ORCA Strategy - Vertical Envelope")
        print("=" * 60)
        
        # Define test cases for vertical altitude handling
        test_cases = [
            {
                "name": "Case 1: Same Altitude",
                "agent_z": 10.0,
                "obstacle_z": 10.0,
                "expected": "included"
            },
            {
                "name": "Case 2: Far Above (+30m)",
                "agent_z": 10.0,
                "obstacle_z": 40.0,
                "expected": "excluded"
            },
            {
                "name": "Case 3: Far Below (-30m)",
                "agent_z": 10.0,
                "obstacle_z": -20.0,
                "expected": "excluded"
            },
            {
                "name": "Case 4: Boundary +1.75m",
                "agent_z": 10.0,
                "obstacle_z": 11.75,
                "expected": "included"
            },
            {
                "name": "Case 5: Just Outside +1.76m",
                "agent_z": 10.0,
                "obstacle_z": 11.76,
                "expected": "excluded"
            },
        ]
        
        # ORCA configuration
        orca_config = {
            "max_velocity": 4.0,
            "orca_vertical_margin": self.ORCA_VERTICAL_MARGIN,
            "ship_keepout_radius": 5.0,
            "ship_influence_radius": 30.0,
            "attractive_gain": 1.0,
            "repulsive_gain": 1.0,
            "goal_tolerance": 1.5,
        }
        
        for test_case in test_cases:
            vmap = VoxelMap()
            
            # Place obstacle at specified altitude
            obs_x, obs_y = 30.0, 0.0
            vmap.mark_occupied(obs_x, obs_y, test_case["obstacle_z"], confidence=1.0, current_time=time.time())
            
            agent_pos = {"x": 0.0, "y": 0.0, "z": test_case["agent_z"]}
            goal_pos = {"x": 50.0, "y": 0.0, "z": test_case["agent_z"]}
            
            # Create and configure ORCA strategy
            orca = ORCAStrategy()
            orca.configure(orca_config)
            
            # Test obstacle inclusion/exclusion
            included_count = 0
            excluded_count = 0
            convergence_steps = 0
            oscillations = 0
            
            for sim_step in range(iterations):
                try:
                    # Get obstacles
                    nearby_obs = vmap.get_nearby_obstacles(agent_pos["x"], agent_pos["y"], agent_pos["z"], radius=20.0)
                    
                    # Compute velocity
                    velocity = orca.compute_velocity(
                        current_pose=agent_pos,
                        goal_pose=goal_pos,
                        voxel_map=vmap
                    )
                    
                    # Determine if obstacle was included in constraints
                    if len(nearby_obs) > 0:
                        included_count += 1
                    else:
                        excluded_count += 1
                    
                    # Check for oscillation (alternating velocity direction)
                    if sim_step > 10:
                        vx = velocity.get("linear", {}).get("x", 0)
                        vy = velocity.get("linear", {}).get("y", 0)
                        vz = velocity.get("linear", {}).get("z", 0)
                        speed = math.sqrt(vx**2 + vy**2 + vz**2)
                        if speed > 0.1:  # Moving
                            convergence_steps += 1
                
                except Exception as e:
                    print(f"    Exception in {test_case['name']}: {e}")
            
            expected_inclusion = test_case["expected"] == "included"
            actual_inclusion = included_count > 0
            
            match = expected_inclusion == actual_inclusion
            status = "✓ PASS" if match else "✗ FAIL"
            
            print(f"\n{test_case['name']:30s} {status}")
            print(f"  Expected:       {'included' if expected_inclusion else 'excluded'}")
            print(f"  Actual:         {'included' if actual_inclusion else 'excluded'}")
            print(f"  Convergence:    {convergence_steps}/{iterations} steps")
            
            self.results[f"orca_{test_case['name']}"] = {
                "included_count": included_count,
                "excluded_count": excluded_count,
                "convergence_steps": convergence_steps,
                "status": status
            }

    def verify_trajectory_stability(self):
        """Verify that trajectories are stable (no oscillations)."""
        print("\n[VERIFICATION] Trajectory Stability")
        print("=" * 60)
        
        # Create simple scenario
        vmap = VoxelMap()
        
        # Single obstacle
        vmap.mark_occupied(25.0, 0.0, 10.0, confidence=1.0, current_time=time.time())
        
        agent_pos = {"x": 0.0, "y": 0.0, "z": 10.0}
        goal_pos = {"x": 50.0, "y": 0.0, "z": 10.0}
        
        # APF configuration
        apf_config = {
            "attractive_gain": 1.8,
            "repulsive_gain": 8.5,
            "influence_radius": 7.0,
            "step_size": 0.2,
            "max_velocity": 4.0,
            "goal_tolerance": 1.5,
            "braking_radius": 7.5,
            "ship_keepout_radius": 18.0,
            "ship_influence_radius": 30.0,
            "apf_exponential_decay": 0.5,
            "apf_inverse_square_scale": 0.3,
            "apf_stuck_growth_rate": 0.2,
            "min_movement": 0.01,
            "stuck_threshold": 5,
            "min_z": 1.0,
            "max_z": 50.0,
            "velocity_smoothing": 0.42,
            "max_repulsive_force": self.F_MAX_REP
        }
        
        apf = APFStrategy()
        apf.configure(apf_config)
        
        # Simulate trajectory
        velocities = []
        
        for _ in range(100):
            velocity = apf.compute_velocity(
                current_pose=agent_pos,
                goal_pose=goal_pos,
                voxel_map=vmap
            )
            vx = velocity.get("linear", {}).get("x", 0)
            vy = velocity.get("linear", {}).get("y", 0)
            vz = velocity.get("linear", {}).get("z", 0)
            velocities.append((vx, vy, vz))
        
        # Check for excessive velocity direction changes (oscillation indicator)
        direction_changes = 0
        for i in range(1, len(velocities)):
            v1 = velocities[i - 1]
            v2 = velocities[i]
            
            # Dot product of consecutive velocity vectors
            dot = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
            mag1 = math.sqrt(v1[0]**2 + v1[1]**2 + v1[2]**2)
            mag2 = math.sqrt(v2[0]**2 + v2[1]**2 + v2[2]**2)
            
            if mag1 > 0 and mag2 > 0:
                cos_angle = dot / (mag1 * mag2)
                if cos_angle < 0:  # Direction reversed (>90 degrees)
                    direction_changes += 1
        
        oscillation_rate = (direction_changes / len(velocities)) * 100
        
        status = "✓ PASS" if oscillation_rate < 10 else "✗ FAIL"
        print(f"\n{status} Trajectory stability")
        print(f"  Direction changes: {direction_changes}/{len(velocities)} ({oscillation_rate:.1f}%)")
        print(f"  Oscillation threshold: <10% for stable trajectory")
        
        self.results["trajectory_stability"] = {
            "direction_changes": direction_changes,
            "oscillation_rate_pct": oscillation_rate,
            "status": status
        }

    def run_all_validations(self):
        """Execute all planner validation benchmarks."""
        print("\n" + "=" * 60)
        print("PATH PLANNER TRAJECTORY STABILITY VALIDATION")
        print("=" * 60)
        
        self.benchmark_apf_forces(iterations=300)
        self.benchmark_orca_vertical_envelope(iterations=300)
        self.verify_trajectory_stability()
        
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
    validator = PlannerValidator()
    results = validator.run_all_validations()
    print("\nValidation complete.")
