#!/usr/bin/env python3
"""
Random Agent Generator for DeNDDron-Swarm
This script generates random agent configurations and updates the config.yaml file.
"""

import random
import argparse
import yaml
from typing import List, Dict, Tuple

def generate_random_color() -> List[float]:
    """Generate a random RGB color with values between 0.0 and 1.0"""
    return [random.random(), random.random(), random.random()]

def generate_random_position(world_min: List[float], world_max: List[float], 
                             margin: float = 2.0) -> List[float]:
    """Generate a random position within world bounds with margin"""
    return [
        random.uniform(world_min[0] + margin, world_max[0] - margin),
        random.uniform(world_min[1] + margin, world_max[1] - margin),
        random.uniform(world_min[2] + margin, world_max[2] - margin)
    ]

def check_collision(pos: List[float], existing_positions: List[List[float]], 
                     min_distance: float = 3.0) -> bool:
    """Check if a position is too close to existing positions"""
    for existing in existing_positions:
        dist = sum((a - b) ** 2 for a, b in zip(pos, existing)) ** 0.5
        if dist < min_distance:
            return True
    return False

def generate_agents(num_agents: int, world_bounds: Dict, 
                     default_radius: float = 0.5, 
                     default_speed: float = 2.0,
                     min_separation: float = 3.0) -> List[Dict]:
    """
    Generate random agents with start and goal positions
    
    Args:
        num_agents: Number of agents to generate
        world_bounds: Dictionary with 'min' and 'max' keys for world boundaries
        default_radius: Default agent radius
        default_speed: Default agent speed
        min_separation: Minimum distance between agent start/goal positions
    
    Returns:
        List of agent configurations
    """
    agents = []
    occupied_positions = []
    
    world_min = world_bounds['min']
    world_max = world_bounds['max']
    
    print(f"Generating {num_agents} random agents...")
    print(f"World bounds: {world_min} to {world_max}")
    
    for i in range(num_agents):
        max_attempts = 100
        attempts = 0
        
        # Generate start position
        while attempts < max_attempts:
            start = generate_random_position(world_min, world_max)
            if not check_collision(start, occupied_positions, min_separation):
                break
            attempts += 1
        
        if attempts >= max_attempts:
            print(f"Warning: Could not find collision-free start for agent {i}")
        
        occupied_positions.append(start)
        
        # Generate goal position
        attempts = 0
        while attempts < max_attempts:
            goal = generate_random_position(world_min, world_max)
            if not check_collision(goal, occupied_positions, min_separation):
                break
            attempts += 1
        
        if attempts >= max_attempts:
            print(f"Warning: Could not find collision-free goal for agent {i}")
        
        occupied_positions.append(goal)
        
        # Generate agent configuration
        agent = {
            'id': i,
            'start': [round(x, 2) for x in start],
            'goal': [round(x, 2) for x in goal],
            'color': [round(c, 2) for c in generate_random_color()],
            'radius': default_radius,
            'speed': default_speed
        }
        
        agents.append(agent)
        print(f"  Agent {i}: start={agent['start']} -> goal={agent['goal']}")
    
    return agents

def generate_obstacles(num_obstacles: int, world_bounds: Dict,
                       default_size: List[float] = [2.0, 2.0, 2.0],
                       size_variation: float = 0.3,
                       min_separation: float = 3.0) -> Tuple[List[Dict], int]:
    """
    Generate random obstacle configurations
    
    Args:
        num_obstacles: Number of obstacles to generate
        world_bounds: Dictionary with 'min' and 'max' keys for world boundaries
        default_size: Default obstacle size [x, y, z]
        size_variation: Size variation factor (0.0 to 1.0)
        min_separation: Minimum distance between obstacles
    
    Returns:
        Tuple of (list of obstacle positions, total count)
    """
    obstacles = []
    obstacle_positions = []
    
    world_min = world_bounds['min']
    world_max = world_bounds['max']
    
    print(f"\nGenerating {num_obstacles} random obstacles...")
    
    for i in range(num_obstacles):
        max_attempts = 50
        attempts = 0
        
        while attempts < max_attempts:
            center = generate_random_position(world_min, world_max, margin=3.0)
            if not check_collision(center, obstacle_positions, min_separation):
                break
            attempts += 1
        
        if attempts >= max_attempts:
            print(f"Warning: Could not find collision-free position for obstacle {i}")
        
        obstacle_positions.append(center)
        
        # Vary size slightly
        variation = 1.0 + random.uniform(-size_variation, size_variation)
        size = [round(s * variation, 2) for s in default_size]
        
        obstacle = {
            'center': [round(x, 2) for x in center],
            'size': size
        }
        
        obstacles.append(obstacle)
        print(f"  Obstacle {i}: center={obstacle['center']}, size={obstacle['size']}")
    
    return obstacles, num_obstacles

def load_existing_config(config_path: str) -> Dict:
    """Load existing configuration file"""
    try:
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Config file not found: {config_path}")
        print("Creating default configuration...")
        return {
            'world': {
                'bounds': {
                    'min': [-20.0, -20.0, -20.0],
                    'max': [20.0, 20.0, 20.0]
                }
            },
            'obstacles': {
                'count': 0,
                'default_size': [2.0, 2.0, 2.0],
                'positions': []
            },
            'agents': {
                'default_radius': 0.5,
                'default_speed': 2.0,
                'agent_list': []
            }
        }

def save_config(config: Dict, config_path: str):
    """Save configuration to YAML file"""
    with open(config_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    print(f"\nConfiguration saved to: {config_path}")

def main():
    parser = argparse.ArgumentParser(
        description='Generate random agents and obstacles for DeNDDron-Swarm simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate 10 agents
  python3 generate_random_agents.py -n 10
  
  # Generate 20 agents with 15 obstacles
  python3 generate_random_agents.py -n 20 -o 15
  
  # Generate 5 agents with custom world bounds
  python3 generate_random_agents.py -n 5 --world-min -30 -30 -30 --world-max 30 30 30
  
  # Save to custom config file
  python3 generate_random_agents.py -n 8 -c my_config.yaml
        """
    )
    
    parser.add_argument('-n', '--num-agents', type=int, default=5,
                        help='Number of agents to generate (default: 5)')
    parser.add_argument('-o', '--num-obstacles', type=int, default=None,
                        help='Number of obstacles to generate (default: keep existing)')
    parser.add_argument('-c', '--config', type=str, default='config.yaml',
                        help='Path to config file (default: config.yaml)')
    parser.add_argument('--world-min', type=float, nargs=3, default=None,
                        help='World minimum bounds [x y z]')
    parser.add_argument('--world-max', type=float, nargs=3, default=None,
                        help='World maximum bounds [x y z]')
    parser.add_argument('--agent-radius', type=float, default=0.5,
                        help='Default agent radius (default: 0.5)')
    parser.add_argument('--agent-speed', type=float, default=2.0,
                        help='Default agent speed (default: 2.0)')
    parser.add_argument('--min-separation', type=float, default=3.0,
                        help='Minimum separation between agents/obstacles (default: 3.0)')
    parser.add_argument('--keep-obstacles', action='store_true',
                        help='Keep existing obstacles from config file')
    parser.add_argument('--seed', type=int, default=None,
                        help='Random seed for reproducibility')
    
    args = parser.parse_args()
    
    # Set random seed if provided
    if args.seed is not None:
        random.seed(args.seed)
        print(f"Using random seed: {args.seed}")
    
    # Load existing configuration
    config = load_existing_config(args.config)
    
    # Update world bounds if provided
    if args.world_min:
        config['world']['bounds']['min'] = args.world_min
    if args.world_max:
        config['world']['bounds']['max'] = args.world_max
    
    world_bounds = config['world']['bounds']
    
    # Generate agents
    agents = generate_agents(
        num_agents=args.num_agents,
        world_bounds=world_bounds,
        default_radius=args.agent_radius,
        default_speed=args.agent_speed,
        min_separation=args.min_separation
    )
    
    # Update agent configuration
    config['agents']['default_radius'] = args.agent_radius
    config['agents']['default_speed'] = args.agent_speed
    config['agents']['agent_list'] = agents
    
    # Handle obstacles
    if args.num_obstacles is not None and not args.keep_obstacles:
        # Generate new obstacles
        obstacles, count = generate_obstacles(
            num_obstacles=args.num_obstacles,
            world_bounds=world_bounds,
            default_size=config['obstacles'].get('default_size', [2.0, 2.0, 2.0]),
            min_separation=args.min_separation
        )
        config['obstacles']['count'] = count
        config['obstacles']['positions'] = obstacles
    elif not args.keep_obstacles:
        # Keep obstacle count but regenerate positions if specified
        if config['obstacles'].get('count', 0) > 0:
            print("\nKeeping existing obstacle count, regenerating positions...")
            obstacles, count = generate_obstacles(
                num_obstacles=config['obstacles']['count'],
                world_bounds=world_bounds,
                default_size=config['obstacles'].get('default_size', [2.0, 2.0, 2.0]),
                min_separation=args.min_separation
            )
            config['obstacles']['positions'] = obstacles
    else:
        print("\nKeeping existing obstacles from config file")
    
    # Save configuration
    save_config(config, args.config)
    
    # Print summary
    print("\n" + "="*60)
    print("CONFIGURATION SUMMARY")
    print("="*60)
    print(f"Agents generated: {len(agents)}")
    print(f"Obstacles: {config['obstacles']['count']}")
    print(f"World bounds: {world_bounds['min']} to {world_bounds['max']}")
    print(f"Config file: {args.config}")
    print("="*60)
    print("\nYou can now run the simulation:")
    print("  cd build && ./multi_agent_planning")

if __name__ == '__main__':
    main()
